"""End-to-end SMORES-EP docking through wheel effort and ground contact."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.core.events import AssemblyMerged, AssemblySplit, DockCommitted, UndockCommitted
from modsim.runtime import ReconfigurationPhase, RuntimeDemo
from modsim.runtime.inspector_runner import RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim.runtime.physics_docking import DifferentialDriveDockingScenario
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter

pytestmark = pytest.mark.mujoco


def test_smores_physical_demo_drives_docks_releases_and_reverses(
    smores_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=smores_pack_dir,
            demo=RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK,
            backend="mujoco",
            connector_gap_m=0.02,
            approach_m_s=0.03,
            retract_m_s=0.03,
            duration_s=12.0,
            dt_s=0.002,
            gravity=True,
            ground=True,
            height_m=0.05,
        )
    )
    try:
        assert isinstance(runner.scenario, DifferentialDriveDockingScenario)
        adapter_type = type(runner.session.adapter)

        def reject_runtime_root_write(*_args: object, **_kwargs: object) -> None:
            raise AssertionError("physical scenario wrote root kinematics after initial placement")

        monkeypatch.setattr(adapter_type, "set_module_pose", reject_runtime_root_write)
        monkeypatch.setattr(adapter_type, "set_module_twist", reject_runtime_root_write)
        monkeypatch.setattr(adapter_type, "apply_module_wrench", reject_runtime_root_write)

        edge_counts: set[int] = set()
        for _ in range(runner.step_count):
            runner.step()
            edge_counts.add(len(runner.session.world.connections))
            if runner.scenario.status.phase is ReconfigurationPhase.COMPLETE:
                break

        assert runner.scenario.status.phase is ReconfigurationPhase.COMPLETE
        assert edge_counts == {0, 1}
        events = tuple(runner.session.world.event_log)
        committed = tuple(
            type(event)
            for event in events
            if isinstance(event, DockCommitted | AssemblyMerged | UndockCommitted | AssemblySplit)
        )
        assert committed == (DockCommitted, AssemblyMerged, UndockCommitted, AssemblySplit)
        dock = next(event for event in events if isinstance(event, DockCommitted))
        undock = next(event for event in events if isinstance(event, UndockCommitted))
        assert {dock.connector_a, dock.connector_b} == {
            runner.scenario.config.fixed_connector,
            runner.scenario.config.moving_connector,
        }
        assert math.dist(dock.relative_transform.translation, (0.0, 0.0, 0.0)) <= (
            runner.scenario.config.latch_distance_m + 1e-6
        )
        assert undock.connection_id == dock.connection_id
        assert undock.time_s - dock.time_s >= 1.08 - 1e-9

        metrics = runner.session.metrics()
        assert metrics.docking_success_count == 1
        assert metrics.undocking_success_count == 1
        assert metrics.docking_failure_count == 0
        assert metrics.undocking_failure_count == 0
        assert metrics.connection_count_active == 0
        assert metrics.assembly_count == 2

        moving = runner.session.world.modules[runner.scenario.config.moving_module]
        assert moving.pose.translation[0] < -0.12
        assert moving.pose.translation[2] == pytest.approx(0.04, abs=0.005)
        assert all(math.isfinite(value) for value in moving.pose.translation)
        assert abs(moving.joint_states["joint_left_wheel"].position) > 0.2
        assert abs(moving.joint_states["joint_right_wheel"].position) > 0.2
        assert abs(moving.joint_states["joint_tilt"].position) < 0.03
        assert abs(moving.joint_states["joint_pan"].position) < 0.01
    finally:
        runner.shutdown()


def test_physical_demo_propagates_the_requested_solver_timestep(
    smores_pack_dir: Path,
) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=smores_pack_dir,
            demo=RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK,
            backend="mujoco",
            duration_s=0.01,
            dt_s=0.001,
            gravity=True,
            ground=True,
            height_m=0.05,
        )
    )
    try:
        assert isinstance(runner.session.adapter, MuJoCoBackendAdapter)
        timestep = float(runner.session.adapter.model.opt.timestep)
        assert timestep == pytest.approx(0.001)
    finally:
        runner.shutdown()


def test_smores_physical_demo_rejects_nonphysical_launch_options(
    smores_pack_dir: Path,
) -> None:
    with pytest.raises(ValueError, match="requires gravity and the ground plane"):
        RuntimeInspectorRunner.create(
            RuntimeInspectorConfig(
                pack_path=smores_pack_dir,
                demo=RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK,
                backend="mujoco",
                gravity=False,
                ground=False,
                height_m=0.05,
            )
        )
