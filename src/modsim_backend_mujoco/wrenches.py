"""Batched conversion of MuJoCo equality solver rows into connection wrenches.

Every active weld and hinge reports a :class:`ConstraintWrench` each step. The
per-connection formulas are documented in ``docs/backends.md``; this module
evaluates them for all connections at once with array operations:

* force vectors and their magnitudes are computed when the rows are captured,
  because the core's overload check reads every magnitude every tick;
* moments need the kinematics the rows were built from, so the capture copies
  that state, and the moments of every connection are computed together the
  first time any moment is read.

Per-connection Python work is a dictionary entry, so a world with hundreds of
welds costs a handful of small array operations per step instead of hundreds
of tiny numpy calls.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence

import mujoco
import numpy as np

from modsim.core.ids import ConstraintHandle
from modsim.core.snapshot import ConstraintWrench, FrozenMapping
from modsim_backend_mujoco.hinges import CONNECT_ANCHOR_BODY_A
from modsim_backend_mujoco.welds import RELPOSE_POSITION, RELPOSE_ROTATION, TORQUE_SCALE

_EQUALITY = int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
_CONJUGATE = np.array((1.0, -1.0, -1.0, -1.0))
_PURE_AXES = np.array(((0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)))


class EqualityRows:
    """Solver rows of every equality constraint, grouped by equality id.

    ``efc_id`` identifies the equality that produced each scalar solver row;
    filtering by both constraint kind and equality id avoids mixing in contact,
    friction, or limit forces. A stable sort keeps each group's constraint
    order. Lookups are vectorized: :meth:`gather` fetches the rows of many
    equalities at once.
    """

    __slots__ = ("_counts", "_forces", "_ids", "_starts")

    def __init__(self, data: mujoco.MjData) -> None:
        count = data.nefc
        if count == 0:
            self._forces = np.empty(0)
            self._ids = self._starts = self._counts = np.empty(0, dtype=np.int64)
            return
        equality = np.asarray(data.efc_type[:count]) == _EQUALITY
        identifiers = np.asarray(data.efc_id[:count])[equality]
        forces = np.asarray(data.efc_force[:count])[equality]
        order = np.argsort(identifiers, kind="stable")
        self._forces = forces[order]
        self._ids, self._starts, self._counts = np.unique(
            identifiers[order], return_index=True, return_counts=True
        )

    def __call__(self, equality_id: int) -> np.ndarray:
        """Return solver rows belonging to one equality, in constraint order."""
        position = int(np.searchsorted(self._ids, equality_id))
        if position >= len(self._ids) or self._ids[position] != equality_id:
            return np.empty(0)
        start = int(self._starts[position])
        return self._forces[start : start + int(self._counts[position])]

    def gather(self, equality_ids: np.ndarray, width: int) -> np.ndarray:
        """Return an ``(n, width)`` array of rows; zeros where an equality has none.

        An equality switched on after the rows were built has no rows yet
        (rule 5 of the timing contract); it reports zero force, as before.
        """
        out = np.zeros((len(equality_ids), width))
        if len(equality_ids) == 0 or len(self._ids) == 0:
            return out
        position = np.searchsorted(self._ids, equality_ids)
        clipped = np.minimum(position, len(self._ids) - 1)
        valid = (position < len(self._ids)) & (self._ids[clipped] == equality_ids)
        valid &= self._counts[clipped] == width
        if valid.any():
            starts = self._starts[clipped[valid]]
            out[valid] = self._forces[starts[:, None] + np.arange(width)]
        return out


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two ``(..., 4)`` quaternion arrays (w, x, y, z)."""
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        axis=-1,
    )


class _Batch:
    """State captured for one solve, and the moments computed from it on demand."""

    __slots__ = ("_moment_source", "_torques", "forces", "magnitudes")

    def __init__(self, forces: np.ndarray, moment_source: Callable[[], np.ndarray]) -> None:
        self.forces = forces
        self.magnitudes = np.sqrt(np.einsum("ij,ij->i", forces, forces))
        self._moment_source: Callable[[], np.ndarray] | None = moment_source
        self._torques: np.ndarray | None = None

    @property
    def torques(self) -> np.ndarray:
        if self._torques is None:
            assert self._moment_source is not None
            self._torques = self._moment_source()
            self._moment_source = None
        return self._torques


class WrenchTable(FrozenMapping[ConstraintHandle, ConstraintWrench]):
    """Handle to wrench for one solve; wrench objects are built when read."""

    __slots__ = ("_batch", "_index")

    def __init__(self, index: dict[ConstraintHandle, int], batch: _Batch) -> None:
        self._index = index
        self._batch = batch

    @classmethod
    def empty(cls) -> WrenchTable:
        return cls({}, _Batch(np.zeros((0, 3)), lambda: np.zeros((0, 3))))

    def __getitem__(self, handle: ConstraintHandle) -> ConstraintWrench:
        row = self._index[handle]
        force = self._batch.forces[row]
        torque = self._batch.torques[row]
        return ConstraintWrench(
            force_n=(float(force[0]), float(force[1]), float(force[2])),
            torque_nm=(float(torque[0]), float(torque[1]), float(torque[2])),
        )

    def __iter__(self) -> Iterator[ConstraintHandle]:
        return iter(self._index)

    def __len__(self) -> int:
        return len(self._index)

    def __contains__(self, handle: object) -> bool:
        return handle in self._index

    def force_magnitude_n(self, handle: ConstraintHandle) -> float:
        """Return one connection's force magnitude without building a wrench."""
        return float(self._batch.magnitudes[self._index[handle]])

    def restricted(self, keep: Callable[[ConstraintHandle], bool]) -> WrenchTable:
        """Return a view of the handles ``keep`` accepts, sharing the arrays."""
        return WrenchTable({h: i for h, i in self._index.items() if keep(h)}, self._batch)


def capture_wrenches(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    welds: Sequence[tuple[ConstraintHandle, int, np.ndarray | None]],
    hinges: Sequence[tuple[ConstraintHandle, tuple[int, int], np.ndarray | None]],
) -> WrenchTable:
    """Capture the wrench of every active connection from the last solve.

    ``welds`` lists ``(handle, equality id, connector A origin in body A)`` and
    ``hinges`` lists ``(handle, (equality id, equality id), origin)``. Must be
    called while ``data`` still holds the kinematics the solved rows were
    built from (directly after ``mj_step2`` or ``mj_forward``).

    Conventions (see ``ConstraintWrench``): world-frame force and moment the
    connection applies to body A, the moment about connector A's origin. A
    weld's translational rows are the force at its anchor; its rotational rows
    map to a moment through ``0.5 * torquescale * vec(neg(q_b) * omega * q_a *
    relpose)`` (``mj_instantiateEquality``), so the moment is ``A.T @ rows``. A
    hinge's two point forces sum to the force, and their moments about the
    origin sum to the moment.
    """
    rows = EqualityRows(data)
    weld_count, hinge_count = len(welds), len(hinges)
    if weld_count + hinge_count == 0:
        return WrenchTable.empty()

    weld_ids = np.fromiter((w[1] for w in welds), dtype=np.int64, count=weld_count)
    weld_rows = rows.gather(weld_ids, 6)
    hinge_ids = np.fromiter(
        (equality for h in hinges for equality in h[1]), dtype=np.int64, count=2 * hinge_count
    )
    hinge_rows = rows.gather(hinge_ids, 3).reshape(hinge_count, 2, 3)
    forces = np.concatenate((weld_rows[:, :3], hinge_rows.sum(axis=1)))

    # Copy the kinematics now: the next mj_step1 overwrites them.
    weld_a = model.eq_obj1id[weld_ids]
    weld_b = model.eq_obj2id[weld_ids]
    weld_data = model.eq_data[weld_ids].copy()
    weld_pos_a = data.xpos[weld_a].copy()
    weld_mat_a = data.xmat[weld_a].reshape(-1, 3, 3).copy()
    weld_quat_a = data.xquat[weld_a].copy()
    weld_quat_b = data.xquat[weld_b].copy()
    weld_origin = _origins([w[2] for w in welds])

    hinge_first = hinge_ids[0::2]
    hinge_a = model.eq_obj1id[hinge_first]
    hinge_anchor_local = model.eq_data[hinge_ids][:, CONNECT_ANCHOR_BODY_A].reshape(
        hinge_count, 2, 3
    )
    hinge_pos_a = data.xpos[hinge_a].copy()
    hinge_mat_a = data.xmat[hinge_a].reshape(-1, 3, 3).copy()
    hinge_origin = _origins([h[2] for h in hinges])

    def moments() -> np.ndarray:
        # Welds: rotational rows to a couple, plus the anchor's lever arm.
        quat = _quat_mul(weld_quat_a, weld_data[:, RELPOSE_ROTATION])
        neg_b = weld_quat_b * _CONJUGATE
        scale = 0.5 * weld_data[:, TORQUE_SCALE]
        columns = _quat_mul(_quat_mul(neg_b[:, None, :], _PURE_AXES[None, :, :]), quat[:, None, :])
        rotation_map = columns[:, :, 1:] * scale[:, None, None]  # [w, axis k, component i]
        weld_torque = np.einsum("wki,wi->wk", rotation_map, weld_rows[:, 3:6])
        anchor = weld_pos_a + np.einsum("wij,wj->wi", weld_mat_a, weld_data[:, RELPOSE_POSITION])
        origin = _world_points(weld_pos_a, weld_mat_a, weld_origin, anchor)
        weld_torque += np.cross(anchor - origin, weld_rows[:, :3])
        # Hinges: moments of the two point forces about connector A's origin.
        anchors = hinge_pos_a[:, None, :] + np.einsum(
            "hij,hkj->hki", hinge_mat_a, hinge_anchor_local
        )
        hinge_origin_world = _world_points(hinge_pos_a, hinge_mat_a, hinge_origin, hinge_pos_a)
        hinge_torque = np.cross(anchors - hinge_origin_world[:, None, :], hinge_rows).sum(axis=1)
        return np.concatenate((weld_torque, hinge_torque))

    index = {w[0]: row for row, w in enumerate(welds)}
    index.update({h[0]: weld_count + row for row, h in enumerate(hinges)})
    return WrenchTable(index, _Batch(forces, moments))


def _origins(locals_: Sequence[np.ndarray | None]) -> tuple[np.ndarray, np.ndarray]:
    """Return connector origins in body A as an array, and which are known."""
    known = np.fromiter((value is not None for value in locals_), dtype=bool, count=len(locals_))
    points = np.zeros((len(locals_), 3))
    for row, value in enumerate(locals_):
        if value is not None:
            points[row] = value
    return points, known


def _world_points(
    position: np.ndarray,
    rotation: np.ndarray,
    origins: tuple[np.ndarray, np.ndarray],
    fallback: np.ndarray,
) -> np.ndarray:
    """Return connector origins in the world; ``fallback`` where unknown."""
    local, known = origins
    world = position + np.einsum("nij,nj->ni", rotation, local)
    return np.where(known[:, None], world, fallback)
