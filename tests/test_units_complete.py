"""Physical dimensions, full profiles, and canonical telemetry cross boundaries safely."""

from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
from typing import cast

import pytest

from modsim_robotics.planning.mblocks.models import (
    LatticeBlock,
    LatticeGoal,
    LatticePlanningSnapshot,
)
from modsim_robotics.runtime import RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim_robotics.runtime.inspection_protocol import (
    RUNTIME_PROTOCOL_PREFIX,
    RuntimeFrame,
    RuntimeHello,
    RuntimeProtocolError,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim_robotics.unit_settings import load_unit_preferences, save_unit_preferences
from modsim_robotics.units import (
    QuantityKind,
    UnitError,
    UnitPreferences,
    from_si,
    get_unit,
    get_units,
    joint_quantity_kind,
    joint_value_to_si,
    quantity_for_field,
    to_si,
)


@pytest.mark.parametrize("kind", tuple(QuantityKind))
def test_every_quantity_has_valid_profile_choices_and_a_round_trip(kind: QuantityKind) -> None:
    selected = UnitPreferences().unit_for(kind)
    assert get_unit(selected).kind is kind
    choices = get_units(kind)
    assert choices and len({choice.symbol for choice in choices}) == len(choices)
    for choice in choices:
        assert choice.kind is kind
        assert from_si(
            to_si(12.5, choice.symbol, kind=kind), choice.symbol, kind=kind
        ) == pytest.approx(12.5)


def test_derived_speed_choices_follow_distance_and_angle_without_changing_time_basis() -> None:
    preferences = UnitPreferences("cm", "rad", time_unit="ms")
    assert preferences.unit_for(QuantityKind.LINEAR_VELOCITY) == "cm/s"
    assert preferences.unit_for(QuantityKind.ANGULAR_VELOCITY) == "rad/s"
    assert preferences.unit_for(QuantityKind.LINEAR_ACCELERATION) == "cm/s^2"
    assert preferences.unit_for(QuantityKind.TIME) == "ms"
    explicit = replace(preferences, angular_velocity_unit="rpm", linear_velocity_unit="mm/s")
    assert explicit.unit_for(QuantityKind.ANGULAR_VELOCITY) == "rpm"
    assert explicit.unit_for(QuantityKind.LINEAR_VELOCITY) == "mm/s"
    with pytest.raises(UnitError):
        preferences.unit_for(cast(QuantityKind, "unknown"))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"time_unit": "mm"},
        {"linear_stiffness_unit": "N*m/deg"},
        {"angular_velocity_unit": "Hz"},
        {"frequency_unit": "rpm"},
        {"angular_damping_unit": "N*s/m"},
        {"linear_acceleration_unit": "mm/s"},
    ],
)
def test_profiles_reject_cross_quantity_preferences(kwargs: dict[str, str]) -> None:
    with pytest.raises(UnitError):
        UnitPreferences(**kwargs)


def test_stiffness_and_damping_conversions_preserve_the_resulting_effort() -> None:
    # A 2 N/mm spring displaced 3 mm must still produce 6 N.
    assert to_si(2.0, "N/mm") * to_si(3.0, "mm") == pytest.approx(6.0)
    # Angular units occur in the denominator, so this factor is the inverse.
    assert to_si(2.0, "N*m/deg") * to_si(3.0, "deg") == pytest.approx(6.0)
    assert to_si(2.0, "N*mm/deg") * to_si(3.0, "deg") == pytest.approx(0.006)
    assert to_si(2.0, "N*s/mm") * to_si(3.0, "mm/s") == pytest.approx(6.0)
    assert to_si(2.0, "N*m*s/deg") * to_si(3.0, "deg/s") == pytest.approx(6.0)


def test_joint_dimensions_use_mechanical_type_and_do_not_wrap_turns() -> None:
    assert joint_quantity_kind("prismatic", "effort") == QuantityKind.FORCE
    assert joint_quantity_kind("revolute", "effort") == QuantityKind.TORQUE
    assert joint_value_to_si(
        450, "deg", joint_type="continuous", control_mode="position"
    ) == pytest.approx(2.5 * math.pi)
    assert joint_value_to_si(50, "mm", joint_type="prismatic", control_mode="position") == 0.05
    with pytest.raises(UnitError):
        joint_value_to_si(50, "mm", joint_type="revolute", control_mode="position")
    with pytest.raises(UnitError):
        joint_quantity_kind("floating", "position")


def test_field_metadata_is_explicit_and_leaves_nonphysical_values_unmapped() -> None:
    assert quantity_for_field("position_tolerance_m") == QuantityKind.LENGTH
    assert quantity_for_field("rotational_stiffness_nm_per_rad") == QuantityKind.ANGULAR_STIFFNESS
    assert quantity_for_field("position", joint_type="prismatic") == QuantityKind.LENGTH
    assert (
        quantity_for_field("value", joint_type="revolute", control_mode="effort")
        == QuantityKind.TORQUE
    )
    for field in ("axis", "orientation_world_wxyz", "cell", "friction", "my_metadata_mm"):
        assert quantity_for_field(field) is None
    with pytest.raises(UnitError):
        quantity_for_field("position")


def test_complete_preference_profile_round_trips_and_reads_old_files(tmp_path: Path) -> None:
    (tmp_path / "units.toml").write_text(
        "schema_version = 1\n[units]\nlength = 'cm'\nangle = 'rad'\n", encoding="utf-8"
    )
    old = load_unit_preferences(tmp_path)
    assert old == UnitPreferences("cm", "rad")
    full = replace(
        old,
        time_unit="ms",
        mass_unit="g",
        angular_velocity_unit="rpm",
        linear_velocity_unit="mm/s",
        linear_stiffness_unit="N/mm",
        angular_stiffness_unit="N*mm/deg",
        angular_damping_unit="N*m*s/deg",
        force_unit="mN",
        torque_unit="N*mm",
        inertia_unit="kg*mm^2",
        frequency_unit="kHz",
    )
    save_unit_preferences(full, tmp_path)
    assert load_unit_preferences(tmp_path) == full


def _lattice_snapshot(**changes: float) -> LatticePlanningSnapshot:
    return LatticePlanningSnapshot.model_validate(
        {
            "time_s": 0.0,
            "sample_sequence": 0,
            "topology_revision": 0,
            "plan_revision": 1,
            "goal": LatticeGoal(id="line", cells=((0, 0), (1, 0))),
            "anchor": "a",
            "origin_world_m": (0.0, 0.0, 0.0),
            "frame_yaw_rad": 0.0,
            "blocks": (LatticeBlock(id="a", cell=(0, 0)), LatticeBlock(id="b", cell=(1, 0))),
            **changes,
        }
    )


def test_planning_storage_is_canonical_with_read_only_legacy_display_properties() -> None:
    snapshot = _lattice_snapshot(
        planning_time_s=0.025, flywheel_angular_velocity_rad_s=math.tau, pivot_angle_rad=math.pi / 2
    )
    assert snapshot.planning_time_s == 0.025
    assert snapshot.flywheel_angular_velocity_rad_s == pytest.approx(math.tau)
    assert snapshot.pivot_angle_rad == pytest.approx(math.pi / 2)
    assert snapshot.flywheel_rpm == pytest.approx(60.0)
    exported = snapshot.model_dump()
    assert not {"planning_ms", "flywheel_rpm", "pivot_angle_deg"} & exported.keys()
    with pytest.raises(ValueError):
        _lattice_snapshot(pivot_angle_deg=90.0, pivot_angle_rad=1.0)


def test_v3_record_import_upgrades_to_canonical_v4_output(example_pack_dir: Path) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(pack_path=example_pack_dir, backend="mock")
    )
    try:
        frame = runner.frame()
    finally:
        runner.shutdown()
    source = frame.view.source
    snapshot = _lattice_snapshot().model_copy(
        update={
            "time_s": source.world_time_s,
            "sample_sequence": source.sample_sequence,
            "topology_revision": source.topology_revision,
        }
    )
    record = RuntimeFrame(frame=frame.model_copy(update={"lattice_planning": snapshot}))
    encoded = encode_runtime_message(record)
    document = json.loads(encoded.removeprefix(RUNTIME_PROTOCOL_PREFIX.encode()).decode())
    document["protocol_version"] = 3
    legacy = document["frame"]["lattice_planning"]
    legacy.pop("planning_time_s")
    legacy.pop("flywheel_angular_velocity_rad_s")
    legacy.pop("pivot_angle_rad")
    legacy.update(planning_ms=25.0, flywheel_rpm=60.0, pivot_angle_deg=90.0)
    migrated = decode_runtime_message(b"MODSIM_RUNTIME/3 " + json.dumps(document).encode() + b"\n")
    assert isinstance(migrated, RuntimeFrame) and migrated.frame.lattice_planning is not None
    assert migrated.frame.lattice_planning.pivot_angle_rad == pytest.approx(math.pi / 2)
    assert encode_runtime_message(migrated).startswith(b"MODSIM_RUNTIME/4 ")
    assert b'"pivot_angle_deg"' not in encode_runtime_message(migrated)
    assert (
        decode_runtime_message(b'MODSIM_RUNTIME/3 {"protocol_version":3,"kind":"hello"}\n')
        == RuntimeHello()
    )
    with pytest.raises(RuntimeProtocolError):
        decode_runtime_message(b'MODSIM_RUNTIME/3 {"protocol_version":4,"kind":"hello"}\n')


@pytest.mark.parametrize("value", [4.0, True, "4"])
def test_protocol_versions_require_actual_integer_values(value: object) -> None:
    record = {"protocol_version": value, "kind": "hello"}
    with pytest.raises(RuntimeProtocolError):
        decode_runtime_message(b"MODSIM_RUNTIME/4 " + json.dumps(record).encode() + b"\n")
