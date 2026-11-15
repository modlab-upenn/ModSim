"""Array-backed MuJoCo snapshots.

A snapshot used to build one Python object per body, joint, and connector site
on every step, which dominated runtime cost for large worlds even though most
of those objects were never read. Here a snapshot is one bulk copy of the
relevant ``MjData`` rows, gathered through index arrays computed once at load.
``BodyState``, ``JointState``, and connector ``Transform`` values are built on
first lookup and cached for the lifetime of that snapshot.

Values are bit-identical to the former eager snapshot: the same MuJoCo routines
(the helpers ``mju_transformSpatial`` uses inside ``mj_objectVelocity``, and
``mju_mat2Quat``) run on exact copies of the same rows. The copies are taken
when the snapshot is created, so later physics steps or dock commits cannot
change a snapshot that has already been handed to ModSim.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from typing import Generic, TypeVar

import mujoco
import numpy as np

from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, split_connector_instance_id
from modsim.core.snapshot import BodyState, FrozenMapping, JointState
from modsim.core.transforms import Transform

_V = TypeVar("_V")


@dataclass(frozen=True, slots=True)
class SampleLayout:
    """Index arrays and per-module row tables, computed once per compiled model."""

    body_rows: dict[ModuleInstanceId, dict[str, int]]
    body_index: np.ndarray
    body_root_index: np.ndarray
    site_rows: dict[ModuleInstanceId, dict[str, int]]
    site_index: np.ndarray
    joint_rows: dict[ModuleInstanceId, dict[str, tuple[int, int, int]]]
    """(qpos address, dof address, actuator id or -1) per semantic joint."""

    @classmethod
    def build(
        cls,
        model: mujoco.MjModel,
        body_ids: Mapping[tuple[ModuleInstanceId, str], int],
        site_ids: Mapping[ConnectorInstanceId, int],
        joint_ids: Mapping[tuple[ModuleInstanceId, str], int],
        actuator_ids: Mapping[tuple[ModuleInstanceId, str], int],
    ) -> SampleLayout:
        # Insertion order follows the compiled maps, matching the eager snapshot.
        body_rows: dict[ModuleInstanceId, dict[str, int]] = {}
        bodies: list[int] = []
        for (module_id, link), body_id in body_ids.items():
            body_rows.setdefault(module_id, {})[link] = len(bodies)
            bodies.append(body_id)
        site_rows: dict[ModuleInstanceId, dict[str, int]] = {}
        sites: list[int] = []
        for instance, site_id in site_ids.items():
            module_id, connector_id = split_connector_instance_id(instance)
            site_rows.setdefault(module_id, {})[connector_id] = len(sites)
            sites.append(site_id)
        joint_rows: dict[ModuleInstanceId, dict[str, tuple[int, int, int]]] = {}
        for (module_id, joint_id), native in joint_ids.items():
            joint_rows.setdefault(module_id, {})[joint_id] = (
                int(model.jnt_qposadr[native]),
                int(model.jnt_dofadr[native]),
                actuator_ids.get((module_id, joint_id), -1),
            )
        body_index = np.asarray(bodies, dtype=np.intp)
        return cls(
            body_rows=body_rows,
            body_index=body_index,
            body_root_index=np.asarray(model.body_rootid, dtype=np.intp)[body_index],
            site_rows=site_rows,
            site_index=np.asarray(sites, dtype=np.intp),
            joint_rows=joint_rows,
        )


class MujocoSample:
    """Immutable copies of the ``MjData`` rows one snapshot needs."""

    __slots__ = (
        "_actuator_force",
        "_cvel",
        "_qpos",
        "_qvel",
        "_site_xmat",
        "_site_xpos",
        "_subtree_com",
        "_xipos",
        "_xpos",
        "_xquat",
        "layout",
    )

    def __init__(self, layout: SampleLayout, data: mujoco.MjData) -> None:
        self.layout = layout
        bodies = layout.body_index
        # Fancy indexing and .copy() both allocate fresh arrays: later steps
        # cannot alter this sample.
        self._xpos = data.xpos[bodies]
        self._xquat = data.xquat[bodies]
        self._xipos = data.xipos[bodies]
        self._cvel = data.cvel[bodies]
        self._subtree_com = data.subtree_com[layout.body_root_index]
        self._site_xpos = data.site_xpos[layout.site_index]
        self._site_xmat = data.site_xmat[layout.site_index]
        self._qpos = data.qpos.copy()
        self._qvel = data.qvel.copy()
        self._actuator_force = data.actuator_force.copy()
        for array in (
            self._xpos, self._xquat, self._xipos, self._cvel, self._subtree_com, self._site_xpos,
            self._site_xmat, self._qpos, self._qvel, self._actuator_force,
        ):  # fmt: skip
            array.flags.writeable = False

    def body_state(self, row: int) -> BodyState:
        # Identical to mj_objectVelocity(..., mjOBJ_BODY, id, res, flg_local=0).
        # mjOBJ_BODY is the body's *inertial* frame, so the com-based spatial
        # velocity is shifted to xipos (the body's centre of mass), not to the
        # body origin xpos. The pose, by contrast, is the body frame (xpos,
        # xquat). Both match what the eager snapshot reported. The shift uses
        # the same three BLAS helpers mju_transformSpatial calls with rot=NULL.
        cvel = self._cvel[row]
        offset = np.zeros(3)
        mujoco.mju_sub3(offset, self._xipos[row], self._subtree_com[row])
        lever = np.zeros(3)
        mujoco.mju_cross(lever, offset, cvel[0:3])
        linear = np.array(cvel[3:6])
        mujoco.mju_subFrom3(linear, lever)
        velocity = (float(cvel[0]), float(cvel[1]), float(cvel[2]))
        position = self._xpos[row]
        quat = self._xquat[row]
        return BodyState(
            pose=Transform(
                translation=(float(position[0]), float(position[1]), float(position[2])),
                rotation=(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            ),
            angular_velocity_rad_s=velocity,
            linear_velocity_m_s=(float(linear[0]), float(linear[1]), float(linear[2])),
        )

    def site_frame(self, row: int) -> Transform:
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.array(self._site_xmat[row], dtype=float))
        position = self._site_xpos[row]
        return Transform(
            translation=(float(position[0]), float(position[1]), float(position[2])),
            rotation=(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
        )

    def joint_state(self, row: tuple[int, int, int]) -> JointState:
        qpos_address, dof_address, actuator_id = row
        return JointState(
            position=float(self._qpos[qpos_address]),
            velocity=float(self._qvel[dof_address]),
            effort=0.0 if actuator_id < 0 else float(self._actuator_force[actuator_id]),
        )

    def link_states(self) -> FrozenMapping[ModuleInstanceId, Mapping[str, BodyState]]:
        return _PerModule(self.layout.body_rows, self.body_state)

    def connector_frames(self) -> FrozenMapping[ModuleInstanceId, Mapping[str, Transform]]:
        return _PerModule(self.layout.site_rows, self.site_frame)

    def joint_states(self) -> FrozenMapping[ModuleInstanceId, Mapping[str, JointState]]:
        return _PerModule(self.layout.joint_rows, self.joint_state)


class _Rows(FrozenMapping[str, _V], Generic[_V]):
    """One module's name -> value view, materialising each value once."""

    __slots__ = ("_cache", "_make", "_rows")

    def __init__(self, rows: Mapping[str, object], make: object) -> None:
        self._rows = rows
        self._make = make
        self._cache: dict[str, _V] = {}

    def __getitem__(self, key: str) -> _V:
        try:
            return self._cache[key]
        except KeyError:
            row = self._rows[key]  # raises KeyError for unknown names
            value = self._make(row)  # type: ignore[operator]
            self._cache[key] = value
            return value

    def __contains__(self, key: object) -> bool:
        return key in self._rows

    def __iter__(self) -> Iterator[str]:
        return iter(self._rows)

    def __len__(self) -> int:
        return len(self._rows)


class _PerModule(FrozenMapping[ModuleInstanceId, Mapping[str, _V]], Generic[_V]):
    """Module -> per-module view, created on first lookup."""

    __slots__ = ("_cache", "_make", "_table")

    def __init__(
        self, table: Mapping[ModuleInstanceId, Mapping[str, object]], make: object
    ) -> None:
        self._table = table
        self._make = make
        self._cache: dict[ModuleInstanceId, _Rows[_V]] = {}

    def __getitem__(self, module_id: ModuleInstanceId) -> Mapping[str, _V]:
        try:
            return self._cache[module_id]
        except KeyError:
            view: _Rows[_V] = _Rows(self._table[module_id], self._make)
            self._cache[module_id] = view
            return view

    def __contains__(self, key: object) -> bool:
        return key in self._table

    def __iter__(self) -> Iterator[ModuleInstanceId]:
        return iter(self._table)

    def __len__(self) -> int:
        return len(self._table)
