"""Backend-neutral joint command validation and state-ingestion tests."""

from __future__ import annotations

import math
from dataclasses import FrozenInstanceError
from typing import Any

import pytest

from modsim.backends.base import BackendCapabilities, BackendError
from modsim.backends.mock import MockBackendAdapter
from modsim.core.entities import JointCommand
from modsim.core.ids import (
    JointInstanceId,
    ModuleInstanceId,
    joint_instance_id,
    split_joint_instance_id,
)
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, JointState
from modsim.core.state import WorldState
from modsim.robot_packs.schema import ControlMode, RobotPack
from modsim.runtime.session import JointCommandError, RuntimeSession

MODULE = ModuleInstanceId("generic_cube_0")
TILT = JointInstanceId(f"{MODULE}/tilt")
WHEEL = JointInstanceId(f"{MODULE}/wheel")
SLIDER = JointInstanceId(f"{MODULE}/slider")
FIXED_MOUNT = JointInstanceId(f"{MODULE}/fixed_mount")


class RecordingJointBackend(MockBackendAdapter):
    """Mock physics plus an observable implementation of the command protocol."""

    def __init__(self, modes: frozenset[ControlMode] | None = None) -> None:
        super().__init__()
        self.modes = modes or frozenset(ControlMode)
        self.command_batches: list[tuple[JointCommand, ...]] = []
        self.clear_batches: list[tuple[JointInstanceId, ...] | None] = []

    def capabilities(self) -> BackendCapabilities:
        return BackendCapabilities(
            name="recording",
            supports_runtime_constraints=True,
            supports_constraint_removal=True,
            supported_joint_control_modes=self.modes,
        )

    def set_joint_commands(self, commands: tuple[JointCommand, ...]) -> None:
        self.command_batches.append(commands)

    def clear_joint_commands(
        self,
        joints: tuple[JointInstanceId, ...] | None = None,
    ) -> None:
        self.clear_batches.append(joints)


class AdvertisingOnlyBackend(MockBackendAdapter):
    """Invalid backend that advertises commands without implementing them."""

    def capabilities(self) -> BackendCapabilities:
        return BackendCapabilities(
            name="advertising_only",
            supported_joint_control_modes=frozenset({ControlMode.EFFORT}),
        )


@pytest.fixture
def controlled_pack(example_pack: RobotPack) -> RobotPack:
    """Return the generic pack with angular, continuous, and linear joints."""
    data: dict[str, Any] = example_pack.model_dump(mode="python")
    module = data["hardware_catalog"]["module_types"]["generic_cube"]
    module["joints"] = [
        {
            "id": "tilt",
            "source_joint_name": "tilt_joint",
            "type": "revolute",
            "parent_link": "base_link",
            "child_link": "tilt_link",
            "axis": (0.0, 1.0, 0.0),
            "control_modes": ("position", "velocity", "effort"),
            "limits": {
                "lower_position_rad": -1.0,
                "upper_position_rad": 1.0,
                "max_velocity_rad_per_s": 2.0,
                "max_effort_nm": 3.0,
            },
        },
        {
            "id": "wheel",
            "source_joint_name": "wheel_joint",
            "type": "continuous",
            "parent_link": "base_link",
            "child_link": "wheel_link",
            "axis": (0.0, 1.0, 0.0),
            "control_modes": ("position", "velocity", "effort"),
            "limits": {
                "max_velocity_rad_per_s": 4.0,
                "max_effort_nm": 5.0,
            },
        },
        {
            "id": "slider",
            "source_joint_name": "slider_joint",
            "type": "prismatic",
            "parent_link": "base_link",
            "child_link": "slider_link",
            "axis": (1.0, 0.0, 0.0),
            "control_modes": ("position", "velocity", "effort"),
            "limits": {
                "lower_position_m": -0.2,
                "upper_position_m": 0.3,
                "max_velocity_m_per_s": 0.4,
                "max_effort_n": 6.0,
            },
        },
        {
            "id": "fixed_mount",
            "source_joint_name": "fixed_mount_joint",
            "type": "fixed",
            "parent_link": "base_link",
            "child_link": "fixed_link",
        },
    ]
    return RobotPack.model_validate(data)


def make_session(
    pack: RobotPack,
    backend: MockBackendAdapter | None = None,
) -> tuple[RuntimeSession, MockBackendAdapter]:
    """Create a one-module runtime around a caller-selected mock derivative."""
    resolved = backend or RecordingJointBackend()
    session = RuntimeSession.create(
        pack,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        resolved,
    )
    return session, resolved


def test_joint_identifier_round_trip_is_strict() -> None:
    assert joint_instance_id(MODULE, "tilt") == TILT
    assert split_joint_instance_id(TILT) == (MODULE, "tilt")

    for malformed in ("tilt", "/tilt", "module/"):
        with pytest.raises(ValueError, match="malformed joint instance id"):
            split_joint_instance_id(JointInstanceId(malformed))


def test_joint_command_is_an_immutable_typed_value() -> None:
    command = JointCommand(TILT, ControlMode.EFFORT, 1.5)

    assert command.joint == TILT
    assert command.mode is ControlMode.EFFORT
    assert command.value == 1.5
    with pytest.raises(FrozenInstanceError):
        command.value = 0.0  # type: ignore[misc]


def test_capability_modes_enable_the_legacy_support_flag() -> None:
    capabilities = BackendCapabilities(
        name="effort_backend",
        supported_joint_control_modes=frozenset({ControlMode.EFFORT}),
    )

    assert capabilities.supports_joint_commands
    assert capabilities.supported_joint_control_modes == frozenset({ControlMode.EFFORT})


def test_valid_batch_is_delegated_once_in_input_order(controlled_pack: RobotPack) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)
    commands = (
        JointCommand(TILT, ControlMode.EFFORT, -2.5),
        JointCommand(SLIDER, ControlMode.VELOCITY, 0.4),
    )

    session.set_joint_commands(iter(commands))

    assert backend.command_batches == [commands]


def test_continuous_position_is_finite_but_unbounded(controlled_pack: RobotPack) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)
    command = JointCommand(WHEEL, ControlMode.POSITION, 1000.0)

    session.set_joint_commands((command,))

    assert backend.command_batches == [(command,)]


@pytest.mark.parametrize(
    ("command", "message"),
    [
        (JointCommand(TILT, ControlMode.POSITION, 1.01), "outside"),
        (JointCommand(TILT, ControlMode.VELOCITY, -2.01), "exceeds"),
        (JointCommand(TILT, ControlMode.EFFORT, 3.01), "exceeds"),
        (JointCommand(SLIDER, ControlMode.POSITION, 0.31), "outside"),
        (JointCommand(SLIDER, ControlMode.VELOCITY, -0.41), "exceeds"),
        (JointCommand(SLIDER, ControlMode.EFFORT, 6.01), "exceeds"),
    ],
)
def test_mode_appropriate_limits_are_enforced(
    controlled_pack: RobotPack,
    command: JointCommand,
    message: str,
) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)

    with pytest.raises(JointCommandError, match=message):
        session.set_joint_commands((command,))

    assert backend.command_batches == []


def test_required_unknown_limit_refuses_command(controlled_pack: RobotPack) -> None:
    data: dict[str, Any] = controlled_pack.model_dump(mode="python")
    wheel = data["hardware_catalog"]["module_types"]["generic_cube"]["joints"][1]
    wheel["limits"]["max_effort_nm"] = None
    pack = RobotPack.model_validate(data)
    backend = RecordingJointBackend()
    session, _ = make_session(pack, backend)

    with pytest.raises(JointCommandError, match="requires a maximum effort limit"):
        session.set_joint_commands((JointCommand(WHEEL, ControlMode.EFFORT, 0.1),))

    assert backend.command_batches == []


@pytest.mark.parametrize(
    ("joint", "message"),
    [
        (JointInstanceId("tilt"), "malformed joint instance id"),
        (JointInstanceId("unknown_0/tilt"), "unknown module 'unknown_0'"),
        (JointInstanceId(f"{MODULE}/unknown"), "has no joint 'unknown'"),
    ],
)
def test_joint_target_must_resolve(
    controlled_pack: RobotPack,
    joint: JointInstanceId,
    message: str,
) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)

    with pytest.raises(JointCommandError, match=message):
        session.set_joint_commands((JointCommand(joint, ControlMode.EFFORT, 0.1),))

    assert backend.command_batches == []


def test_joint_must_declare_the_command_mode(controlled_pack: RobotPack) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)

    with pytest.raises(JointCommandError, match="does not declare 'effort' control"):
        session.set_joint_commands((JointCommand(FIXED_MOUNT, ControlMode.EFFORT, 0.1),))

    assert backend.command_batches == []


def test_backend_must_support_the_specific_mode(controlled_pack: RobotPack) -> None:
    backend = RecordingJointBackend(frozenset({ControlMode.EFFORT}))
    session, _ = make_session(controlled_pack, backend)

    with pytest.raises(JointCommandError, match="does not support 'velocity'"):
        session.set_joint_commands((JointCommand(WHEEL, ControlMode.VELOCITY, 1.0),))

    assert backend.command_batches == []


def test_backend_must_implement_its_advertised_protocol(controlled_pack: RobotPack) -> None:
    session, _ = make_session(controlled_pack, AdvertisingOnlyBackend())

    with pytest.raises(BackendError, match="does not implement SupportsJointCommands"):
        session.set_joint_commands((JointCommand(TILT, ControlMode.EFFORT, 0.1),))


@pytest.mark.parametrize("value", [math.inf, -math.inf, math.nan, True, "0.1"])
def test_invalid_numeric_value_refuses_the_whole_batch(
    controlled_pack: RobotPack,
    value: Any,
) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)
    commands = (
        JointCommand(TILT, ControlMode.EFFORT, 0.1),
        JointCommand(SLIDER, ControlMode.EFFORT, value),
    )

    with pytest.raises(JointCommandError, match="must be a finite number"):
        session.set_joint_commands(commands)

    assert backend.command_batches == []


def test_duplicate_target_refuses_the_whole_batch(controlled_pack: RobotPack) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)

    with pytest.raises(JointCommandError, match="duplicate target"):
        session.set_joint_commands(
            (
                JointCommand(TILT, ControlMode.EFFORT, 0.1),
                JointCommand(TILT, ControlMode.EFFORT, 0.2),
            )
        )

    assert backend.command_batches == []


def test_clear_targets_are_validated_then_delegated(controlled_pack: RobotPack) -> None:
    backend = RecordingJointBackend()
    session, _ = make_session(controlled_pack, backend)

    session.clear_joint_commands((TILT, WHEEL))
    session.clear_joint_commands()

    assert backend.clear_batches == [(TILT, WHEEL), None]

    with pytest.raises(JointCommandError, match="has no joint 'unknown'"):
        session.clear_joint_commands((TILT, JointInstanceId(f"{MODULE}/unknown")))
    assert backend.clear_batches == [(TILT, WHEEL), None]


def test_world_ingests_and_looks_up_joint_state_without_link_state(
    controlled_pack: RobotPack,
) -> None:
    world = WorldState.from_scene(
        controlled_pack,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
    )
    measured = JointState(position=0.25, velocity=-1.5, effort=0.75)

    with pytest.raises(KeyError, match="has not reported state"):
        world.joint_state(TILT)

    world.ingest(
        BackendStateSnapshot(
            time_s=0.1,
            joint_states={MODULE: {"tilt": measured}},
        )
    )

    assert world.modules[MODULE].joint_states == {"tilt": measured}
    assert world.joint_spec(TILT).id == "tilt"
    assert world.joint_state(TILT) is measured

    world.ingest(BackendStateSnapshot(time_s=0.2, joint_states={MODULE: {}}))
    assert world.modules[MODULE].joint_states == {}
    with pytest.raises(KeyError, match="has not reported state"):
        world.joint_state(TILT)
