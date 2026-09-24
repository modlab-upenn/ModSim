"""Spatial control failures and deadlines without simulator dependencies."""

from __future__ import annotations

from dataclasses import replace

import pytest

from modsim.backends.mock import MockBackendAdapter
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, JointState
from modsim.core.transforms import Vec3
from modsim.planning.spatial import Bond, JointPoint
from modsim.robot_packs.schema import RobotPack
from modsim.runtime.session import RuntimeSession
from modsim.runtime.spatial import (
    SpatialHandoffProblem,
    SpatialObservation,
    SpatialReconfigurationScenario,
)

HELPER = ModuleInstanceId("generic_cube_0")
PAYLOAD = ModuleInstanceId("generic_cube_1")
RECEIVER = ModuleInstanceId("generic_cube_2")


class FeedbackBackend(MockBackendAdapter):
    measured = True

    def snapshot(self) -> BackendStateSnapshot:
        snapshot = super().snapshot()
        return replace(
            snapshot, joint_states={HELPER: {"motor": JointState()}} if self.measured else {}
        )


class TestMotionServices:
    __test__ = False
    motor_keys = ((HELPER, "motor"),)
    stale = False

    def __init__(self, session: RuntimeSession) -> None:
        self.session = session
        self.target_positions: dict[str, Vec3] = {
            str(m): module.pose.translation for m, module in session.world.modules.items()
        }

    def feasible(self, point: JointPoint, *, transferred: bool = False) -> bool:
        return True

    def targets(self, point: JointPoint) -> tuple[float, ...]:
        return (0.0,)

    def configuration_positions(
        self, point: JointPoint, *, transferred: bool = False
    ) -> dict[str, Vec3]:
        return self.target_positions

    def command_positions(self, positions: tuple[float, ...]) -> None:
        assert positions == (0.0,)

    def observe_safety(self) -> SpatialObservation:
        return SpatialObservation(self.session.world.time_s + (1 if self.stale else 0), 0, True)


@pytest.mark.parametrize("outcome", ["timeout", "missing_feedback", "stale", "stopped"])
def test_controller_uses_measured_state_and_explicit_terminal_outcomes(
    example_pack: RobotPack,
    outcome: str,
) -> None:
    backend = FeedbackBackend()
    session = RuntimeSession.create(
        example_pack, SceneSpec.grid("generic_cube", 3, spacing_m=0.1), backend
    )
    try:
        initial = Bond(f"{HELPER}/front", f"{PAYLOAD}/rear")
        capture = Bond(f"{RECEIVER}/front", f"{PAYLOAD}/front")
        session.request_dock(ConnectorInstanceId(initial.a), ConnectorInstanceId(initial.b))
        session.process_docking()
        motion = TestMotionServices(session)
        scenario = SpatialReconfigurationScenario(
            session,
            motion,
            SpatialHandoffProblem(
                frozenset(map(str, (HELPER, PAYLOAD, RECEIVER))),
                frozenset(map(str, (HELPER, RECEIVER))),
                frozenset((initial,)),
                frozenset((capture,)),
                str(HELPER),
            ),
            duration_s=0.003,
        )
        if outcome == "missing_feedback":
            backend.measured = False
        elif outcome == "stale":
            motion.stale = True
        elif outcome == "stopped":
            scenario.stop()
        for _ in range(5):
            scenario.step()
        assert scenario.terminal
        assert scenario.bonds == frozenset((initial,))
        if outcome == "timeout":
            assert scenario.phase == "timeout"
            assert session.world.time_s == pytest.approx(0.003)
        elif outcome == "stopped":
            assert scenario.phase == "stopped"
            assert session.world.time_s == 0
        else:
            assert scenario.phase == "failed"
            assert session.world.time_s == pytest.approx(0.001)
            assert (
                "Missing" in scenario.detail
                if outcome == "missing_feedback"
                else "time" in scenario.detail
            )
    finally:
        session.shutdown()
