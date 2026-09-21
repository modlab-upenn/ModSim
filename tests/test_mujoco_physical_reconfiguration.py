"""End-to-end wheel-driven SMORES-EP Driver-to-Snake regression."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.core.events import (
    AssemblyMerged,
    AssemblySplit,
    DockCommitted,
    DockFailed,
    Event,
    UndockCommitted,
    UndockFailed,
)
from modsim.core.ids import ModuleInstanceId, split_connector_instance_id
from modsim.core.transforms import quat_rotate
from modsim.runtime import ReconfigurationPhase, RuntimeDemo
from modsim.runtime.inspector_runner import RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim.runtime.physical_reconfiguration import DifferentialDriveReconfigurationScenario
from modsim.runtime.session import RuntimeSession

pytestmark = pytest.mark.mujoco

_DURATION_S = 210.0
_COMPLETED_HOLD_S = 20.0
_WHEEL_JOINTS = ("joint_left_wheel", "joint_right_wheel")
_MOTION_PHASES = frozenset(
    {
        ReconfigurationPhase.HOLDING_SEPARATED,
        ReconfigurationPhase.APPROACHING,
        ReconfigurationPhase.DOCKING,
    }
)
_THREE_MODULE_COMPONENTS = {
    2: frozenset(ModuleInstanceId(f"module_{index}") for index in (1, 2, 3)),
    3: frozenset(ModuleInstanceId(f"module_{index}") for index in (5, 6, 7)),
}
_FINAL_CONNECTOR_EDGES = frozenset(
    {
        frozenset(("module_1/pan", "module_3/bottom")),
        frozenset(("module_2/bottom", "module_3/pan")),
        frozenset(("module_2/pan", "module_4/bottom")),
        frozenset(("module_4/pan", "module_5/bottom")),
        frozenset(("module_5/pan", "module_6/bottom")),
        frozenset(("module_6/pan", "module_7/bottom")),
    }
)


def _wheel_positions(session: RuntimeSession) -> dict[tuple[ModuleInstanceId, str], float]:
    return {
        (module_id, joint_id): module.joint_states[joint_id].position
        for module_id, module in session.world.modules.items()
        for joint_id in _WHEEL_JOINTS
    }


def _maximum_wheel_speed(session: RuntimeSession) -> float:
    return max(
        abs(module.joint_states[joint_id].velocity)
        for module in session.world.modules.values()
        for joint_id in _WHEEL_JOINTS
    )


def _assert_all_state_is_finite(session: RuntimeSession) -> None:
    for module in session.world.modules.values():
        values = (
            *module.pose.translation,
            *module.pose.rotation,
            *module.linear_velocity_m_s,
            *module.angular_velocity_rad_s,
            *(
                value
                for state in module.joint_states.values()
                for value in (state.position, state.velocity, state.effort)
            ),
        )
        assert all(math.isfinite(value) for value in values)


def _connector_edges(session: RuntimeSession) -> frozenset[frozenset[str]]:
    return frozenset(
        frozenset((str(connection.connector_a), str(connection.connector_b)))
        for connection in session.world.connections.values()
    )


def _assert_completed_snake_is_stable(session: RuntimeSession) -> None:
    _assert_all_state_is_finite(session)
    assert _connector_edges(session) == _FINAL_CONNECTOR_EDGES
    assert len(session.world.connections) == 6
    assert session.world.assemblies.count == 1
    for module in session.world.modules.values():
        quaternion_norm = math.sqrt(sum(value * value for value in module.pose.rotation))
        assert quaternion_norm == pytest.approx(1.0, abs=1e-6)
        assert quat_rotate(module.pose.rotation, (0.0, 0.0, 1.0))[2] > math.cos(0.15)
        assert 0.025 < module.pose.translation[2] < 0.06


def _semantic_event_types(events: tuple[Event, ...]) -> tuple[type[Event], ...]:
    semantic_types = (
        DockCommitted,
        AssemblyMerged,
        UndockCommitted,
        AssemblySplit,
        DockFailed,
        UndockFailed,
    )
    return tuple(type(event) for event in events if isinstance(event, semantic_types))


def test_smores_driver_to_snake_completes_through_wheel_and_contact_physics(
    smores_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Finish within the runtime budget, then hold the final snake for at least 20 s."""
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=smores_pack_dir,
            demo=RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
            backend="mujoco",
            duration_s=_DURATION_S,
            dt_s=0.002,
            approach_m_s=0.03,
            gravity=True,
            ground=True,
            height_m=0.05,
        )
    )
    try:
        scenario = runner.scenario
        assert isinstance(scenario, DifferentialDriveReconfigurationScenario)
        assert runner.session.world.time_s == 0.0
        assert len(runner.session.world.modules) == 7
        assert len(runner.session.world.connections) == 6
        assert runner.session.world.assemblies.count == 1

        adapter_type = type(runner.session.adapter)

        def reject_root_control(*_args: object, **_kwargs: object) -> None:
            raise AssertionError(
                "physical reconfiguration wrote root pose, twist, or wrench after creation"
            )

        monkeypatch.setattr(adapter_type, "set_module_pose", reject_root_control)
        monkeypatch.setattr(adapter_type, "set_module_twist", reject_root_control)
        monkeypatch.setattr(adapter_type, "apply_module_wrench", reject_root_control)

        connection_count_transitions = [len(runner.session.world.connections)]
        release_started_at_s: dict[int, float] = {}
        release_events: dict[int, UndockCommitted] = {}
        released_components: dict[int, frozenset[ModuleInstanceId]] = {}
        maximum_released_face_separation_m = {2: 0.0, 3: 0.0}
        wheel_travel_rad = {
            action_index: {module_id: 0.0 for module_id in modules}
            for action_index, modules in _THREE_MODULE_COMPONENTS.items()
        }
        previous_wheel_positions = _wheel_positions(runner.session)
        maximum_measured_wheel_speed_rad_s = _maximum_wheel_speed(runner.session)

        completion_step_count: int | None = None
        for step_index in range(runner.step_count):
            step_events = runner.step()
            status = scenario.status
            action_index = status.action_index

            if status.phase is ReconfigurationPhase.FAILED:
                pytest.fail(f"physical reconfiguration failed: {status.detail}")
            if status.phase is ReconfigurationPhase.UNDOCKING:
                assert action_index is not None
                release_started_at_s.setdefault(action_index, status.time_s)

            for event in step_events:
                if not isinstance(event, UndockCommitted):
                    continue
                assert action_index is not None
                release_events[action_index] = event
                action = scenario.plan.actions[action_index]
                assert action.dock is not None
                moving_module = split_connector_instance_id(action.dock.moving_connector)[0]
                moving_assembly = runner.session.world.assemblies.assembly_of(moving_module)
                released_components[action_index] = frozenset(
                    runner.session.world.assemblies.members(moving_assembly)
                )

            wheel_positions = _wheel_positions(runner.session)
            maximum_measured_wheel_speed_rad_s = max(
                maximum_measured_wheel_speed_rad_s,
                _maximum_wheel_speed(runner.session),
            )
            if action_index in _THREE_MODULE_COMPONENTS and status.phase in _MOTION_PHASES:
                for module_id in _THREE_MODULE_COMPONENTS[action_index]:
                    wheel_travel_rad[action_index][module_id] += sum(
                        abs(
                            wheel_positions[(module_id, joint_id)]
                            - previous_wheel_positions[(module_id, joint_id)]
                        )
                        for joint_id in _WHEEL_JOINTS
                    )
                action = scenario.plan.actions[action_index]
                assert action.undock is not None
                old_a = runner.session.world.connector(action.undock.fixed_connector)
                old_b = runner.session.world.connector(action.undock.moving_connector)
                maximum_released_face_separation_m[action_index] = max(
                    maximum_released_face_separation_m[action_index],
                    math.dist(
                        old_a.world_pose.translation,
                        old_b.world_pose.translation,
                    ),
                )
            previous_wheel_positions = wheel_positions

            connection_count = len(runner.session.world.connections)
            if connection_count != connection_count_transitions[-1]:
                connection_count_transitions.append(connection_count)

            if step_index % 250 == 0:
                _assert_all_state_is_finite(runner.session)
            if status.phase is ReconfigurationPhase.COMPLETE:
                completion_step_count = step_index + 1
                break
        else:
            pytest.fail(
                "physical reconfiguration did not complete within "
                f"{runner.config.duration_s:.1f} s: {scenario.status}"
            )

        assert completion_step_count is not None
        assert scenario.status.phase is ReconfigurationPhase.COMPLETE
        assert scenario.status.time_s <= runner.config.duration_s
        completion_time_s = scenario.status.time_s
        assert connection_count_transitions == [6, 5, 6, 5, 6, 5, 6, 5, 6]

        assert released_components[0] == {ModuleInstanceId("module_1")}
        assert released_components[1] == {ModuleInstanceId("module_7")}
        assert released_components[2] == _THREE_MODULE_COMPONENTS[2]
        assert released_components[3] == _THREE_MODULE_COMPONENTS[3]
        for action_index, modules in _THREE_MODULE_COMPONENTS.items():
            assert maximum_released_face_separation_m[action_index] > 0.02
            assert all(wheel_travel_rad[action_index][module_id] > 0.5 for module_id in modules)

        assert set(release_started_at_s) == {0, 1, 2, 3}
        assert set(release_events) == {0, 1, 2, 3}
        for action_index, event in release_events.items():
            assert (
                event.time_s - release_started_at_s[action_index]
                >= scenario.config.release_delay_s - 1e-9
            )

        events = tuple(runner.session.world.event_log)
        assert not [event for event in events if isinstance(event, DockFailed | UndockFailed)]
        expected_semantic_types = (DockCommitted, AssemblyMerged) * 6 + (
            UndockCommitted,
            AssemblySplit,
            DockCommitted,
            AssemblyMerged,
        ) * 4
        assert _semantic_event_types(events) == expected_semantic_types

        dock_events = tuple(event for event in events if isinstance(event, DockCommitted))
        undock_events = tuple(event for event in events if isinstance(event, UndockCommitted))
        expected_docks = (
            *(pair.connection_id for pair in scenario.plan.initial_connections),
            *(
                action.dock.connection_id
                for action in scenario.plan.actions
                if action.dock is not None
            ),
        )
        expected_undocks = tuple(
            action.undock.connection_id
            for action in scenario.plan.actions
            if action.undock is not None
        )
        assert tuple(event.connection_id for event in dock_events) == expected_docks
        assert tuple(event.connection_id for event in undock_events) == expected_undocks

        assert _connector_edges(runner.session) == _FINAL_CONNECTOR_EDGES

        metrics = runner.session.metrics()
        assert metrics.module_count_total == 7
        assert metrics.module_count_connected == 7
        assert metrics.module_count_free == 0
        assert metrics.assembly_count == 1
        assert metrics.largest_assembly_size == 7
        assert metrics.connection_count_active == 6
        assert metrics.docking_success_count == 10
        assert metrics.docking_failure_count == 0
        assert metrics.undocking_success_count == 4
        assert metrics.undocking_failure_count == 0
        assert metrics.assembly_merge_count == 10
        assert metrics.assembly_split_count == 4

        _assert_completed_snake_is_stable(runner.session)

        assert maximum_measured_wheel_speed_rad_s == pytest.approx(
            scenario.maximum_wheel_speed_rad_s_observed,
            abs=1e-9,
        )
        assert (
            scenario.maximum_wheel_speed_rad_s_observed
            <= scenario.config.maximum_wheel_speed_rad_s + 0.05
        )
        assert (
            scenario.maximum_lateral_residual_m_s
            <= scenario.config.lateral_residual_limit_m_s + 1e-12
        )

        # Reuse this expensive run to check the completed state's braked hold.
        # Allow the full hold interval even when completion leaves less than
        # 20 s in the display budget; the completion deadline above still applies.
        event_count_at_completion = len(runner.session.world.event_log)
        hold_step_count = max(
            runner.step_count - completion_step_count,
            math.ceil(_COMPLETED_HOLD_S / runner.config.dt_s),
        )
        for hold_step_index in range(hold_step_count):
            assert not runner.step()
            assert scenario.status.phase is ReconfigurationPhase.COMPLETE
            if hold_step_index % 250 == 0:
                _assert_completed_snake_is_stable(runner.session)
                assert (
                    _maximum_wheel_speed(runner.session)
                    <= scenario.config.maximum_wheel_speed_rad_s + 0.05
                )

        assert scenario.status.time_s == pytest.approx(
            max(runner.config.duration_s, completion_time_s + _COMPLETED_HOLD_S),
            abs=runner.config.dt_s,
        )
        assert scenario.status.phase is ReconfigurationPhase.COMPLETE
        assert len(runner.session.world.event_log) == event_count_at_completion
        _assert_completed_snake_is_stable(runner.session)
    finally:
        runner.shutdown()
