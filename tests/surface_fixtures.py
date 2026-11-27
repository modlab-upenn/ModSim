"""A synthetic sphere-and-strut Robot Pack for continuous-docking tests.

It deliberately does not depend on FreeSN assets. A ``node`` module carries one
spherical ``shell`` surface connector on its root link. A ``strut`` module
carries two genderless point connectors, ``upper`` at +z and ``lower`` at -z,
each with its docking axis pointing out of the strut, so a strut seated below
a node has ``upper`` pointing at the node centre. All connectors sit on root
links, so the kinematic mock places them correctly.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any

from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import (
    Transform,
    Vec3,
    minimal_rotation,
    quat_from_axis_angle,
    quat_multiply,
    vec_add,
    vec_normalize,
    vec_scale,
)
from modsim.robot_packs.schema import RobotPack

RADIUS_M = 0.06
HALF_LENGTH_M = 0.03
CAPACITY = 4
MIN_SEPARATION_RAD = math.pi / 3
SEAT_TOLERANCE_M = 0.003
TILT_TOLERANCE_RAD = 0.1
VELOCITY_TOLERANCE_M_S = 0.05


def _shell_type() -> dict[str, Any]:
    return {
        "id": "shell",
        "name": "Steel shell",
        "active": False,
        "gender": "genderless",
        "compatible_with": ["magnet"],
        "allowed_orientations": {"mode": "continuous", "values_rad": []},
        "acceptance_region": {
            "shape": "sphere",
            "position_tolerance_m": SEAT_TOLERANCE_M,
            "orientation_tolerance_rad": TILT_TOLERANCE_RAD,
            "max_relative_velocity_m_s": VELOCITY_TOLERANCE_M_S,
        },
        "physical_connection": {"constraint": "fixed"},
        "limits": {
            "max_normal_force_n": 140.0,
            "max_shear_force_n": 49.0,
            "max_bending_moment_nm": 1.6,
        },
        "supports_undocking": True,
        "interface": "surface",
        "surface": {
            "shape": "sphere",
            "center_m": [0.0, 0.0, 0.0],
            "radius_m": RADIUS_M,
            "capacity": CAPACITY,
            "min_separation_rad": MIN_SEPARATION_RAD,
        },
    }


def _magnet_type(realizations: Iterable[str]) -> dict[str, Any]:
    return {
        "id": "magnet",
        "name": "Magnet array",
        "active": True,
        "gender": "genderless",
        "compatible_with": ["shell"],
        "allowed_orientations": {"mode": "continuous", "values_rad": []},
        "acceptance_region": {
            "shape": "sphere",
            "position_tolerance_m": SEAT_TOLERANCE_M,
            "orientation_tolerance_rad": TILT_TOLERANCE_RAD,
            "max_relative_velocity_m_s": VELOCITY_TOLERANCE_M_S,
        },
        "physical_connection": {
            "constraint": "fixed",
            "surface_realizations": list(realizations),
        },
        "limits": {
            "max_normal_force_n": 79.0,
            "max_shear_force_n": 49.0,
            "max_bending_moment_nm": 1.6,
            "max_torsion_moment_nm": 1.3,
        },
        "supports_undocking": True,
        "docking_policy": {
            "auto_latch": False,
            "alignment": "measured",
            "redock_cooldown_s": 0.2,
            "break_force_n": None,
        },
    }


def _point(connector_id: str, sign: float, *, contact: bool, drive: bool) -> dict[str, Any]:
    connector: dict[str, Any] = {
        "id": connector_id,
        "connector_type": "magnet",
        "parent_link": "base_link",
        "frame": None,
        "local_pose": {"xyz_m": [0.0, 0.0, sign * HALF_LENGTH_M], "rpy_rad": [0.0, 0.0, 0.0]},
        "docking_axis": [0.0, 0.0, sign],
        "approach_axis": [0.0, 0.0, sign],
    }
    if contact:
        connector["surface_contact_links"] = ["base_link"]
    if drive:
        connector["magnet_height_joint"] = "lift_upper"
        connector["surface_drive"] = {
            "model": "differential",
            "left_joint": "wheel_left",
            "right_joint": "wheel_right",
            "wheel_radius_m": 0.01,
            "track_m": 0.056,
        }
    return connector


def _joint(joint_id: str, joint_type: str, axis: Vec3) -> dict[str, Any]:
    return {
        "id": joint_id,
        "source_joint_name": joint_id,
        "type": joint_type,
        "parent_link": "base_link",
        "child_link": f"{joint_id}_link",
        "axis": list(axis),
    }


def surface_pack(
    base: RobotPack,
    *,
    realizations: Iterable[str] = ("driven_fixed", "static_fixed"),
    shell: dict[str, Any] | None = None,
    magnet: dict[str, Any] | None = None,
    articulated: bool = True,
    radius_m: float | None = None,
) -> RobotPack:
    """Return ``base`` (the generic cube pack) extended with node and strut modules.

    With ``articulated=False`` the strut declares no wheel or lift joints, so a
    backend that compiles the cube asset (MuJoCo) can load it; the strut then
    has no magnet-height joint or drive model, only its contact links.
    ``radius_m``, when given, sets the shell radius. The cube asset is 0.1 m wide, so a
    radius of at least about 0.11 m keeps a seated strut's cube clear of the
    node's cube in every direction.
    """
    data: dict[str, Any] = base.model_dump(mode="python")
    catalog = data["hardware_catalog"]
    asset = next(iter(catalog["module_types"].values()))["asset_ref"]
    shell_type = _shell_type() | (shell or {})
    if radius_m is not None:
        shell_type["surface"] = shell_type["surface"] | {"radius_m": radius_m}
    magnet_type = _magnet_type(realizations) | (magnet or {})
    catalog["connector_types"]["shell"] = shell_type
    catalog["connector_types"]["magnet"] = magnet_type
    catalog["module_types"]["node"] = {
        "id": "node",
        "name": "Node",
        "asset_ref": asset,
        "root_link": "base_link",
        "mass_kg": 0.31,
        "joints": [],
        "connectors": [
            {
                "id": "shell",
                "connector_type": "shell",
                "parent_link": "base_link",
                "frame": None,
                "local_pose": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
                "docking_axis": [0.0, 0.0, 1.0],
                "approach_axis": [0.0, 0.0, 1.0],
            }
        ],
        "capabilities": [],
    }
    catalog["module_types"]["strut"] = {
        "id": "strut",
        "name": "Strut",
        "asset_ref": asset,
        "root_link": "base_link",
        "mass_kg": 0.43,
        "joints": [
            _joint("wheel_left", "continuous", (0.0, 1.0, 0.0)),
            _joint("wheel_right", "continuous", (0.0, 1.0, 0.0)),
            _joint("lift_upper", "prismatic", (0.0, 0.0, 1.0)),
        ]
        if articulated
        else [],
        "connectors": [
            _point("upper", 1.0, contact=True, drive=articulated),
            _point("lower", -1.0, contact=False, drive=False),
        ],
        "capabilities": [],
    }
    for mapping in data["backend_mappings"].values():
        mapping["module_types"]["node"] = {
            "asset_ref": asset,
            "link_map": {"base_link": "base_link"},
            "joint_map": {},
            "connector_frame_map": {},
            "constraint_map": {},
        }
        mapping["module_types"]["strut"] = {
            "asset_ref": asset,
            "link_map": {"base_link": "base_link"},
            "joint_map": (
                {name: name for name in ("wheel_left", "wheel_right", "lift_upper")}
                if articulated
                else {}
            ),
            "connector_frame_map": {},
            "constraint_map": {},
        }
    return RobotPack.model_validate(data)


def strut_pose(
    normal: Vec3,
    *,
    center: Vec3 = (0.0, 0.0, 0.0),
    gap_m: float = 0.0,
    spin_rad: float = 0.0,
    tilt_axis: Vec3 | None = None,
    tilt_rad: float = 0.0,
    radius_m: float = RADIUS_M,
) -> Transform:
    """Return a strut root pose that seats its ``upper`` connector on the shell.

    ``normal`` is the outward surface normal at the attachment, in the world
    frame. The strut's +z (the upper docking axis) points into the sphere,
    rotated by ``spin_rad`` about that axis and optionally tilted about
    ``tilt_axis`` by ``tilt_rad`` around the connector point.
    """
    n = vec_normalize(normal)
    inward = vec_scale(n, -1.0)
    rotation = quat_multiply(
        quat_from_axis_angle(inward, spin_rad), minimal_rotation((0.0, 0.0, 1.0), inward)
    )
    connector_point = vec_add(center, vec_scale(n, radius_m + gap_m))
    if tilt_axis is not None and tilt_rad:
        rotation = quat_multiply(quat_from_axis_angle(tilt_axis, tilt_rad), rotation)
    axis = Transform(rotation=rotation).apply_direction((0.0, 0.0, 1.0))
    origin = vec_add(connector_point, vec_scale(axis, -HALF_LENGTH_M))
    return Transform(translation=origin, rotation=rotation)


NODE = ModuleInstanceId("node_0")


def strut_id(index: int) -> ModuleInstanceId:
    return ModuleInstanceId(f"strut_{index}")


def surface_scene(*strut_poses: Transform, node_pose: Transform | None = None) -> SceneSpec:
    """Return a scene with one node at ``node_pose`` and one strut per pose."""
    placements = [ModulePlacement(NODE, "node", node_pose or Transform.identity())]
    placements.extend(
        ModulePlacement(strut_id(index), "strut", pose) for index, pose in enumerate(strut_poses)
    )
    return SceneSpec.of(placements)
