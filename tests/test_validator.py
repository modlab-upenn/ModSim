"""Tests for cross-document and asset validation."""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim.robot_packs import (
    RobotPackLoader,
    RobotPackValidationError,
    RobotPackValidator,
    Severity,
    ValidationIssue,
    ValidationProfile,
    ValidationReport,
)


@pytest.mark.parametrize(
    "profile",
    [ValidationProfile.AUTHORING, ValidationProfile.SIMULATION],
)
def test_generic_pack_validates_without_issues(
    example_pack_dir: Path, profile: ValidationProfile
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    report = RobotPackValidator().validate(loaded, profile=profile)

    assert report.valid
    assert report.issues == ()


def test_missing_urdf_is_reported(copied_pack: Path) -> None:
    (copied_pack / "assets" / "urdf" / "generic_cube.urdf").unlink()
    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    assert not report.valid
    assert {issue.code for issue in report.errors} == {"asset.missing"}
    assert any(
        issue.location == "robot_pack.yaml:/assets/urdf/generic_cube" for issue in report.errors
    )


def test_symlink_escape_is_rejected(copied_pack: Path, tmp_path: Path) -> None:
    asset_path = copied_pack / "assets" / "urdf" / "generic_cube.urdf"
    outside_asset = tmp_path / "outside.urdf"
    outside_asset.write_text("<robot name='outside'/>", encoding="utf-8")
    asset_path.unlink()
    try:
        asset_path.symlink_to(outside_asset)
    except OSError:
        pytest.skip("symlinks are not available on this platform")

    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    assert not report.valid
    assert any(issue.code == "path.outside_pack" for issue in report.errors)


def test_unknown_asset_reference_is_reported(copied_pack: Path) -> None:
    extra_asset = copied_pack / "assets" / "urdf" / "alternate.urdf"
    extra_asset.write_text("<robot name='alternate'/>", encoding="utf-8")
    module_path = copied_pack / "specs" / "module_types.yaml"
    module_path.write_text(
        module_path.read_text(encoding="utf-8").replace(
            "asset_ref: generic_cube",
            "asset_ref: alternate",
        ),
        encoding="utf-8",
    )

    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    assert any(issue.code == "reference.asset_unknown" for issue in report.errors)
    assert not any(issue.code == "asset.missing" for issue in report.errors)


def test_unknown_connector_type_is_reported(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    module_path.write_text(
        module_path.read_text(encoding="utf-8").replace(
            "connector_type: fixed_face",
            "connector_type: missing_type",
            1,
        ),
        encoding="utf-8",
    )

    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    issue = next(
        issue for issue in report.errors if issue.code == "reference.connector_type_unknown"
    )
    assert issue.entity_ref == "module_type:generic_cube/connector:front"
    assert issue.json_pointer.endswith("/connectors/0/connector_type")


def test_unknown_compatibility_target_is_reported(copied_pack: Path) -> None:
    connector_path = copied_pack / "specs" / "connector_types.yaml"
    connector_path.write_text(
        connector_path.read_text(encoding="utf-8").replace(
            "      - fixed_face",
            "      - missing_type",
            1,
        ),
        encoding="utf-8",
    )

    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    assert any(issue.code == "reference.compatibility_target_unknown" for issue in report.errors)


def test_incomplete_connector_warns_for_authoring_and_errors_for_simulation(
    copied_pack: Path,
) -> None:
    connector_path = copied_pack / "specs" / "connector_types.yaml"
    content = connector_path.read_text(encoding="utf-8")
    start = content.index("    acceptance_region:")
    end = content.index("    physical_connection:")
    connector_path.write_text(content[:start] + content[end:], encoding="utf-8")

    loaded = RobotPackLoader().load(copied_pack)
    authoring = RobotPackValidator().validate(loaded, profile=ValidationProfile.AUTHORING)
    simulation = RobotPackValidator().validate(loaded, profile=ValidationProfile.SIMULATION)

    assert authoring.valid
    assert any(issue.code == "connector.acceptance_region_missing" for issue in authoring.warnings)
    assert not simulation.valid
    assert any(issue.code == "connector.acceptance_region_missing" for issue in simulation.errors)


def test_missing_connector_axis_warns_then_blocks_simulation(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    content = module_path.read_text(encoding="utf-8")
    module_path.write_text(
        content.replace("        docking_axis: [1.0, 0.0, 0.0]\n", "", 1),
        encoding="utf-8",
    )

    loaded = RobotPackLoader().load(copied_pack)
    authoring = RobotPackValidator().validate(loaded)
    simulation = RobotPackValidator().validate(loaded, profile=ValidationProfile.SIMULATION)

    assert authoring.valid
    assert any(issue.code == "connector.docking_axis_missing" for issue in authoring.warnings)
    assert any(issue.code == "connector.docking_axis_missing" for issue in simulation.errors)


def test_unknown_control_limits_warn_then_block_simulation(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    module_path.write_text(
        module_path.read_text(encoding="utf-8").replace(
            "    joints: []",
            """    joints:
      - id: wheel
        source_joint_name: wheel_joint
        type: continuous
        parent_link: base_link
        child_link: wheel_link
        axis: [0.0, 1.0, 0.0]
        control_modes: [velocity, effort]
        limits:
          max_velocity_rad_per_s: null
          max_effort_nm: null""",
        ),
        encoding="utf-8",
    )
    mapping_path = copied_pack / "mappings" / "urdf_mapping.yaml"
    mapping_path.write_text(
        mapping_path.read_text(encoding="utf-8").replace(
            "    joint_map: {}",
            "    joint_map:\n      wheel: wheel_joint",
        ),
        encoding="utf-8",
    )

    loaded = RobotPackLoader().load(copied_pack)
    authoring = RobotPackValidator().validate(loaded)
    simulation = RobotPackValidator().validate(
        loaded,
        profile=ValidationProfile.SIMULATION,
    )

    assert len([issue for issue in authoring.warnings if issue.code == "joint.limit_unknown"]) == 2
    assert len([issue for issue in simulation.errors if issue.code == "joint.limit_unknown"]) == 2


def test_backend_asset_must_be_declared(copied_pack: Path) -> None:
    alternate_asset = copied_pack / "assets" / "urdf" / "backend_only.urdf"
    alternate_asset.write_text("<robot name='backend_only'/>", encoding="utf-8")
    mapping_path = copied_pack / "mappings" / "urdf_mapping.yaml"
    mapping_path.write_text(
        mapping_path.read_text(encoding="utf-8").replace(
            "asset_ref: generic_cube",
            "asset_ref: backend_only",
        ),
        encoding="utf-8",
    )

    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    assert any(issue.code == "reference.asset_unknown" for issue in report.errors)


def test_capability_connector_requirements_are_checked(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    content = module_path.read_text(encoding="utf-8")
    start = content.index("    connectors:")
    end = content.index("    capabilities:")
    module_path.write_text(
        content[:start] + "    connectors: []\n" + content[end:],
        encoding="utf-8",
    )

    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    assert any(issue.code == "capability.connector_requirement_missing" for issue in report.errors)


def test_validator_revalidates_mutated_nested_mappings(example_pack_dir: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    module = loaded.pack.hardware_catalog.module_types["generic_cube"]
    loaded.pack.hardware_catalog.module_types["wrong_key"] = module

    report = RobotPackValidator().validate(loaded)

    assert not report.valid
    assert {issue.code for issue in report.errors} == {"schema.invalid_in_memory"}


def test_report_is_deterministic_and_derived_properties_agree() -> None:
    issues = [
        ValidationIssue(
            severity=Severity.WARNING,
            code="z.warning",
            message="warning",
            document="z.yaml",
        ),
        ValidationIssue(
            severity=Severity.ERROR,
            code="b.error",
            message="second",
            document="b.yaml",
        ),
        ValidationIssue(
            severity=Severity.ERROR,
            code="a.error",
            message="first",
            document="a.yaml",
        ),
    ]
    report = ValidationReport.from_issues(issues)

    assert [issue.code for issue in report.issues] == [
        "a.error",
        "b.error",
        "z.warning",
    ]
    assert not report.valid
    assert len(report.errors) == 2
    assert len(report.warnings) == 1
    with pytest.raises(RobotPackValidationError, match="2 error"):
        report.raise_for_errors()
