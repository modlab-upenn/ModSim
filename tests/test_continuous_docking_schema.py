"""Schema and validator rules for continuous (point-to-surface) docking.

See docs/continuous_docking.md sections 3.1, 3.2 and 14.3 (A7, A8).
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from modsim.robot_packs import (
    LoadedRobotPack,
    RobotPack,
    RobotPackLoader,
    RobotPackValidator,
    RobotPackWriter,
    ValidationProfile,
)
from modsim.robot_packs.issues import Severity
from modsim.robot_packs.schema import (
    ConnectorInterface,
    ConnectorTypeSpec,
    PhysicalConnectionSpec,
    SurfaceRealization,
)
from surface_fixtures import surface_pack


@pytest.fixture
def loaded(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


def _with(loaded: LoadedRobotPack, pack: RobotPack) -> LoadedRobotPack:
    return loaded.with_pack(pack)


def _codes(loaded: LoadedRobotPack, profile: ValidationProfile) -> dict[str, Severity]:
    report = RobotPackValidator().validate(loaded, profile=profile)
    return {issue.code: issue.severity for issue in report.issues}


def _edit(pack: RobotPack, edit: Callable[[dict[str, Any]], object]) -> RobotPack:
    data: dict[str, Any] = pack.model_dump(mode="python")
    edit(data["hardware_catalog"])
    return RobotPack.model_validate(data)


def _set_face_physical(catalog: dict[str, Any], physical: dict[str, Any]) -> None:
    catalog["connector_types"]["fixed_face"]["physical_connection"] = physical


# ----------------------------------------------------------------------
# schema
# ----------------------------------------------------------------------


def test_the_synthetic_pack_is_valid_in_both_profiles(loaded: LoadedRobotPack) -> None:
    candidate = _with(loaded, surface_pack(loaded.pack))
    for profile in ValidationProfile:
        report = RobotPackValidator().validate(candidate, profile=profile)
        assert report.valid, [issue.code for issue in report.errors]
    shell = candidate.pack.hardware_catalog.connector_types["shell"]
    assert shell.interface is ConnectorInterface.SURFACE
    assert shell.is_surface
    assert shell.surface is not None and shell.surface.capacity == 4


def test_point_is_the_default_interface(loaded: LoadedRobotPack) -> None:
    face = loaded.pack.hardware_catalog.connector_types["fixed_face"]
    assert face.interface is ConnectorInterface.POINT
    assert face.surface is None


def test_surface_interface_requires_geometry_and_point_forbids_it() -> None:
    base: dict[str, Any] = {"id": "shell", "interface": "surface"}
    with pytest.raises(ValidationError, match="surface connectors require surface geometry"):
        ConnectorTypeSpec.model_validate(base)
    with pytest.raises(ValidationError, match="surface geometry requires interface: surface"):
        ConnectorTypeSpec.model_validate(
            {"id": "face", "surface": {"radius_m": 0.06, "capacity": 1}}
        )


@pytest.mark.parametrize(
    ("surface", "message"),
    [
        ({"radius_m": 0.0, "capacity": 1}, "greater than 0"),
        ({"radius_m": 0.06, "capacity": 0}, "greater than or equal to 1"),
        ({"radius_m": 0.06, "capacity": 1, "min_separation_rad": 4.0}, "less than or equal"),
        ({"radius_m": 0.06, "capacity": 1, "shape": "plane"}, "sphere"),
    ],
)
def test_surface_geometry_is_strict(surface: dict[str, Any], message: str) -> None:
    with pytest.raises(ValidationError, match=message):
        ConnectorTypeSpec.model_validate(
            {"id": "shell", "interface": "surface", "surface": surface}
        )


def test_surface_realizations_need_fixed_and_no_duplicates() -> None:
    with pytest.raises(ValidationError, match="require constraint: fixed"):
        PhysicalConnectionSpec.model_validate(
            {
                "constraint": "hinge",
                "hinge": {"axis": [0, 1, 0], "anchor_separation_m": 0.04},
                "surface_realizations": ["ball"],
            }
        )
    with pytest.raises(ValidationError, match="duplicates"):
        PhysicalConnectionSpec.model_validate(
            {"constraint": "fixed", "surface_realizations": ["ball", "ball"]}
        )
    spec = PhysicalConnectionSpec.model_validate(
        {"constraint": "fixed", "surface_realizations": ["driven_fixed", "ball"]}
    )
    assert spec.surface_realizations == (SurfaceRealization.DRIVEN_FIXED, SurfaceRealization.BALL)


def test_ball_parameters_are_optional_for_ball_and_forbidden_otherwise() -> None:
    bare = PhysicalConnectionSpec.model_validate({"constraint": "ball"})
    assert bare.ball is None
    anchored = PhysicalConnectionSpec.model_validate(
        {"constraint": "ball", "ball": {"anchor_m": [0.0, 0.0, -0.06]}}
    )
    assert anchored.ball is not None and anchored.ball.anchor_m == (0.0, 0.0, -0.06)
    with pytest.raises(ValidationError, match="ball parameters require a ball constraint"):
        PhysicalConnectionSpec.model_validate({"constraint": "fixed", "ball": {}})


def test_new_limit_policy_and_connector_fields_parse(loaded: LoadedRobotPack) -> None:
    pack = surface_pack(
        loaded.pack,
        magnet={
            "docking_policy": {
                "break_limits": {"max_normal_force_n": 79.0, "max_torsion_moment_nm": 1.3}
            }
        },
    )
    magnet = pack.hardware_catalog.connector_types["magnet"]
    assert magnet.limits is not None and magnet.limits.max_torsion_moment_nm == 1.3
    limits = magnet.effective_docking_policy.break_limits
    assert limits is not None and limits.max_torsion_moment_nm == 1.3
    upper = next(
        c for c in pack.hardware_catalog.module_types["strut"].connectors if c.id == "upper"
    )
    assert upper.magnet_height_joint == "lift_upper"
    assert upper.magnet_height_offset_m == 0.0
    assert upper.surface_contact_links == ("base_link",)
    assert upper.surface_drive is not None and upper.surface_drive.track_m == 0.056


def test_ball_and_custom_unsupported_rule_honours_ball_anchor(loaded: LoadedRobotPack) -> None:
    anchored = _edit(
        loaded.pack,
        lambda catalog: _set_face_physical(
            catalog, {"constraint": "ball", "ball": {"anchor_m": [0.0, 0.0, 0.0]}}
        ),
    )
    codes = _codes(_with(loaded, anchored), ValidationProfile.SIMULATION)
    assert "connector.physical_connection_unsupported" not in codes


# ----------------------------------------------------------------------
# section 3.2 rules
# ----------------------------------------------------------------------


def test_a_surface_type_must_allow_continuous_orientations(loaded: LoadedRobotPack) -> None:
    pack = surface_pack(
        loaded.pack, shell={"allowed_orientations": {"mode": "discrete", "values_rad": [0.0]}}
    )
    for profile in ValidationProfile:
        assert _codes(_with(loaded, pack), profile)["connector.surface_orientation_discrete"] is (
            Severity.ERROR
        )
    omitted = surface_pack(loaded.pack, shell={"allowed_orientations": None})
    assert "connector.surface_orientation_discrete" not in _codes(
        _with(loaded, omitted), ValidationProfile.AUTHORING
    )


def test_two_surface_types_never_mate(loaded: LoadedRobotPack) -> None:
    pack = surface_pack(loaded.pack, shell={"compatible_with": ["magnet", "shell"]})
    for profile in ValidationProfile:
        assert _codes(_with(loaded, pack), profile)["connector.surface_pair_invalid"] is (
            Severity.ERROR
        )


def test_surface_realizations_on_a_point_only_pair_warn_then_error(
    loaded: LoadedRobotPack,
) -> None:
    pack = _edit(
        loaded.pack,
        lambda catalog: _set_face_physical(
            catalog, {"constraint": "fixed", "surface_realizations": ["ball"]}
        ),
    )
    candidate = _with(loaded, pack)
    assert _codes(candidate, ValidationProfile.AUTHORING)[
        "connector.surface_realizations_unused"
    ] is (Severity.WARNING)
    assert _codes(candidate, ValidationProfile.SIMULATION)[
        "connector.surface_realizations_unused"
    ] is (Severity.ERROR)


@pytest.mark.parametrize(
    "physical",
    [
        {"constraint": "hinge", "hinge": {"axis": [0, 1, 0], "anchor_separation_m": 0.04}},
        {"constraint": "ball", "ball": {"anchor_m": [0.0, 0.0, 0.0]}},
        {"constraint": "compliant", "compliance": {"translational_stiffness_n_per_m": 10.0}},
    ],
    ids=["hinge", "ball", "compliant"],
)
def test_a_surface_pair_must_author_constraint_fixed(
    loaded: LoadedRobotPack, physical: dict[str, Any]
) -> None:
    pack = surface_pack(loaded.pack, magnet={"physical_connection": physical})
    for profile in ValidationProfile:
        assert _codes(_with(loaded, pack), profile)["connector.surface_constraint_invalid"] is (
            Severity.ERROR
        )


def test_a_surface_connector_off_the_root_link_warns_then_errors(
    loaded: LoadedRobotPack,
) -> None:
    def move(catalog: dict[str, Any]) -> None:
        catalog["module_types"]["node"]["connectors"][0]["parent_link"] = "shell_link"

    candidate = _with(loaded, _edit(surface_pack(loaded.pack), move))
    assert _codes(candidate, ValidationProfile.AUTHORING)["connector.surface_not_on_root_link"] is (
        Severity.WARNING
    )
    assert _codes(candidate, ValidationProfile.SIMULATION)[
        "connector.surface_not_on_root_link"
    ] is (Severity.ERROR)


def test_differing_surface_realization_lists_conflict(loaded: LoadedRobotPack) -> None:
    pack = surface_pack(
        loaded.pack,
        shell={"physical_connection": {"constraint": "fixed", "surface_realizations": ["ball"]}},
    )
    assert _codes(_with(loaded, pack), ValidationProfile.AUTHORING)[
        "connector.surface_realizations_conflict"
    ] is (Severity.ERROR)

    identical = surface_pack(
        loaded.pack,
        shell={
            "physical_connection": {
                "constraint": "fixed",
                "surface_realizations": ["driven_fixed", "static_fixed"],
            }
        },
    )
    assert "connector.surface_realizations_conflict" not in _codes(
        _with(loaded, identical), ValidationProfile.SIMULATION
    )


def test_the_physical_realization_is_rejected_for_simulation(loaded: LoadedRobotPack) -> None:
    candidate = _with(loaded, surface_pack(loaded.pack, realizations=("physical", "static_fixed")))
    assert _codes(candidate, ValidationProfile.AUTHORING)[
        "connector.surface_realization_unsupported"
    ] is (Severity.WARNING)
    assert _codes(candidate, ValidationProfile.SIMULATION)[
        "connector.surface_realization_unsupported"
    ] is (Severity.ERROR)


def test_magnet_height_and_drive_joints_must_exist_with_the_right_type(
    loaded: LoadedRobotPack,
) -> None:
    def break_joints(catalog: dict[str, Any]) -> None:
        upper = catalog["module_types"]["strut"]["connectors"][0]
        upper["magnet_height_joint"] = "wheel_left"
        upper["surface_drive"]["right_joint"] = "lift_upper"

    codes = _codes(
        _with(loaded, _edit(surface_pack(loaded.pack), break_joints)), ValidationProfile.AUTHORING
    )
    assert codes["connector.magnet_height_joint_invalid"] is Severity.ERROR
    assert codes["connector.surface_drive_joint_invalid"] is Severity.ERROR


@pytest.mark.parametrize("profile", list(ValidationProfile))
def test_surface_contact_links_must_name_links_of_the_module(
    loaded: LoadedRobotPack, profile: ValidationProfile
) -> None:
    def misname(catalog: dict[str, Any]) -> None:
        upper = catalog["module_types"]["strut"]["connectors"][0]
        upper["surface_contact_links"] = ["base_link", "wheel_left_link", "no_such_link"]

    candidate = _with(loaded, _edit(surface_pack(loaded.pack), misname))
    issues = [
        issue
        for issue in RobotPackValidator().validate(candidate, profile=profile).issues
        if issue.code == "connector.surface_contact_link_unknown"
    ]

    # base_link is the root and wheel_left_link a joint's child; only the third is unknown.
    assert [issue.severity for issue in issues] == [Severity.ERROR]
    assert "no_such_link" in issues[0].message
    assert issues[0].path[-2:] == ("surface_contact_links", 2)


def test_new_torsion_limit_is_not_a_completeness_requirement(loaded: LoadedRobotPack) -> None:
    # Existing packs declare normal, shear and bending only. Adding torsion to
    # the schema must not make them incomplete for simulation.
    report = RobotPackValidator().validate(loaded, profile=ValidationProfile.SIMULATION)
    assert report.valid


# ----------------------------------------------------------------------
# persistence
# ----------------------------------------------------------------------


def test_surface_fields_round_trip_through_the_writer(
    loaded: LoadedRobotPack, tmp_path: Path
) -> None:
    pack = surface_pack(
        loaded.pack,
        realizations=("driven_fixed", "ball", "static_fixed"),
        magnet={
            "docking_policy": {
                "alignment": "nominal",
                "break_limits": {"max_shear_force_n": 49.0, "max_torsion_moment_nm": 1.3},
            }
        },
    )
    destination = tmp_path / "exported"
    RobotPackWriter().write(_with(loaded, pack), destination)

    reloaded = RobotPackLoader().load(destination).pack
    assert reloaded == pack
