"""Unit tests for strict Robot Pack models."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from modsim.robot_packs import (
    AcceptanceRegion,
    AcceptanceShape,
    AssetManifest,
    ConnectorSpec,
    ConnectorTypeSpec,
    JointLimits,
    JointSpec,
    JointType,
    ModuleType,
    PoseSpec,
)


def test_schema_rejects_unknown_fields() -> None:
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        ModuleType.model_validate(
            {
                "id": "cube",
                "asset_ref": "cube",
                "root_link": "base_link",
                "unexpected": True,
            }
        )


@pytest.mark.parametrize(
    "path",
    [
        "/absolute/module.urdf",
        "../outside/module.urdf",
        "assets/../outside.urdf",
        r"C:\robots\module.urdf",
        "~/module.urdf",
    ],
)
def test_schema_rejects_nonportable_or_escaping_paths(path: str) -> None:
    with pytest.raises(ValidationError):
        AssetManifest(urdf={"cube": path})


def test_schema_rejects_bad_joint_limit_range() -> None:
    with pytest.raises(ValidationError, match="angular lower limit must not exceed upper"):
        JointLimits(lower_position_rad=1.0, upper_position_rad=-1.0)


def test_schema_rejects_nonpositive_physical_values() -> None:
    with pytest.raises(ValidationError):
        AcceptanceRegion(
            shape=AcceptanceShape.BOX,
            position_tolerance_m=0.0,
            orientation_tolerance_rad=math.pi / 4,
            max_relative_velocity_m_s=0.1,
        )


def test_connector_requires_a_frame_or_local_pose() -> None:
    with pytest.raises(ValidationError, match="either frame or local_pose"):
        ConnectorSpec(
            id="front",
            connector_type="fixed_face",
            parent_link="base_link",
        )


def test_connector_accepts_explicit_local_pose() -> None:
    connector = ConnectorSpec(
        id="front",
        connector_type="fixed_face",
        parent_link="base_link",
        local_pose=PoseSpec(xyz_m=(0.1, 0.0, 0.0)),
    )
    assert connector.local_pose is not None
    assert connector.local_pose.xyz_m == (0.1, 0.0, 0.0)


def test_connector_and_type_accept_json_custom_metadata() -> None:
    connector = ConnectorSpec(
        id="front",
        connector_type="fixed_face",
        parent_link="base_link",
        local_pose=PoseSpec(),
        metadata={
            "hardware.revision": "4.2",
            "channel_count": 4,
            "enabled": True,
            "calibration": [0.1, 0.2],
            "vendor": {"serial": None},
        },
    )
    connector_type = ConnectorTypeSpec(
        id="fixed_face",
        metadata={"electrical.bus": "can", "pins": 8},
    )

    assert connector.metadata["channel_count"] == 4
    assert connector.metadata["vendor"] == {"serial": None}
    assert connector_type.metadata == {"electrical.bus": "can", "pins": 8}


def test_custom_metadata_rejects_bad_keys_and_non_json_values() -> None:
    with pytest.raises(ValidationError):
        ConnectorTypeSpec(id="fixed_face", metadata={"bad field": True})

    with pytest.raises(ValidationError):
        ConnectorTypeSpec.model_validate({"id": "fixed_face", "metadata": {"path": object()}})


def test_schema_rejects_duplicate_module_child_ids() -> None:
    connector: dict[str, object] = {
        "id": "front",
        "connector_type": "fixed_face",
        "parent_link": "base_link",
        "local_pose": {},
    }
    with pytest.raises(ValidationError, match="connector IDs must be unique"):
        ModuleType.model_validate(
            {
                "id": "cube",
                "asset_ref": "cube",
                "root_link": "base_link",
                "connectors": [connector, connector],
            }
        )


def test_revolute_joint_requires_unit_axis() -> None:
    with pytest.raises(ValidationError, match="unit axis"):
        JointSpec(
            id="tilt",
            source_joint_name="tilt_joint",
            type=JointType.REVOLUTE,
            parent_link="base_link",
            child_link="tilt_link",
        )

    with pytest.raises(ValidationError, match="unit length"):
        JointSpec(
            id="tilt",
            source_joint_name="tilt_joint",
            type=JointType.REVOLUTE,
            parent_link="base_link",
            child_link="tilt_link",
            axis=(0.0, 2.0, 0.0),
        )


def test_joint_limit_units_must_match_joint_type() -> None:
    with pytest.raises(ValidationError, match="angular joints must not define linear"):
        JointSpec(
            id="tilt",
            source_joint_name="tilt_joint",
            type=JointType.REVOLUTE,
            parent_link="base_link",
            child_link="tilt_link",
            axis=(0.0, 1.0, 0.0),
            limits=JointLimits(max_velocity_m_per_s=0.2),
        )


def test_continuous_joint_rejects_position_bounds() -> None:
    with pytest.raises(ValidationError, match="must not define position bounds"):
        JointSpec(
            id="wheel",
            source_joint_name="wheel_joint",
            type=JointType.CONTINUOUS,
            parent_link="base_link",
            child_link="wheel_link",
            axis=(0.0, 1.0, 0.0),
            limits=JointLimits(lower_position_rad=-1.0),
        )


def test_asset_manifest_supports_multiple_urdfs() -> None:
    assets = AssetManifest(
        urdf={
            "cube": "assets/urdf/cube.urdf",
            "wheeled": "assets/urdf/wheeled.urdf",
        }
    )

    assert tuple(assets.urdf) == ("cube", "wheeled")


@pytest.mark.parametrize(
    ("model_type", "payload"),
    [
        (PoseSpec, {"xyz_m": (True, 0.0, 0.0)}),
        (JointLimits, {"max_effort_n": "1.0"}),
    ],
)
def test_physical_values_reject_booleans_and_numeric_strings(
    model_type: type[PoseSpec] | type[JointLimits],
    payload: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        model_type.model_validate(payload)


def test_physical_values_accept_yaml_integer_numbers() -> None:
    pose = PoseSpec(xyz_m=(1, 0, 0))

    assert pose.xyz_m == (1.0, 0.0, 0.0)
