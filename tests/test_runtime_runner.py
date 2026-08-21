"""Qt-free Runtime Inspector execution ownership tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim.runtime import (
    DockingPairPhase,
    DockingPairScenario,
    DockingPairScenarioStatus,
    RuntimeDemo,
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
)


def test_runner_owns_scenario_frame_cursor_and_shutdown(example_pack_dir: Path) -> None:
    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            backend="mock",
            connector_gap_m=0.005,
            approach_m_s=0.03,
            duration_s=0.05,
            dt_s=0.01,
            publish_hz=100.0,
            # Presentation topology is intentionally irrelevant to execution.
            viewer_enabled=True,
        ),
        status_callback=statuses.append,
    )
    try:
        assert runner.step_count == 5
        assert runner.event_cursor == 0
        initial = runner.frame()
        assert initial.view.edges == ()
        assert initial.events == ()
        assert runner.event_cursor == 0

        for _ in range(runner.step_count):
            runner.step()

        docked = runner.frame()
        assert len(docked.view.edges) == 1
        assert [row.kind for row in docked.events] == [
            "DockCandidateDetected",
            "DockCommitted",
            "AssemblyMerged",
        ]
        assert docked.event_start_sequence == 0
        assert docked.next_event_sequence == 3
        assert runner.event_cursor == 3

        unchanged = runner.frame()
        assert unchanged.events == ()
        assert unchanged.event_start_sequence == unchanged.next_event_sequence == 3
        assert statuses == [
            "Loading and validating Robot Pack…",
            "Starting mock backend…",
            "Running generic_cube: front ↔ front",
        ]
    finally:
        runner.shutdown()

    runner.shutdown()
    assert runner.closed
    with pytest.raises(RuntimeError, match="shut down"):
        runner.step()
    with pytest.raises(RuntimeError, match="shut down"):
        runner.frame()


def test_runner_rejects_incomplete_connector_pair_before_backend_start(
    example_pack_dir: Path,
) -> None:
    with pytest.raises(ValueError, match="must be supplied together"):
        RuntimeInspectorRunner.create(
            RuntimeInspectorConfig(
                pack_path=example_pack_dir,
                backend="mock",
                fixed_connector="front",
            )
        )


def test_runner_dock_undock_demo_supplies_a_release_schedule(
    example_pack_dir: Path,
) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            demo=RuntimeDemo.DOCK_UNDOCK,
            backend="mock",
            connector_gap_m=0.005,
            approach_m_s=0.03,
            retract_m_s=0.0,
            duration_s=0.1,
            dt_s=0.01,
            publish_hz=100.0,
        )
    )
    try:
        assert isinstance(runner.scenario, DockingPairScenario)
        assert runner.scenario.config.release_after_s == pytest.approx(0.055)
        frames = [runner.frame()]
        for _ in range(runner.step_count):
            runner.step()
            frames.append(runner.frame())
    finally:
        runner.shutdown()

    phases: list[DockingPairPhase] = []
    for frame in frames:
        assert isinstance(frame.scenario, DockingPairScenarioStatus)
        phases.append(frame.scenario.phase)
    assert phases == [
        DockingPairPhase.APPROACHING,
        *(DockingPairPhase.DOCKED for _ in range(6)),
        *(DockingPairPhase.COMPLETE for _ in range(4)),
    ]
    assert [len(frame.view.edges) for frame in frames] == [0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0]
    assert [(frame.event_start_sequence, frame.next_event_sequence) for frame in frames] == [
        (0, 0),
        (0, 3),
        (3, 3),
        (3, 3),
        (3, 3),
        (3, 3),
        (3, 3),
        (3, 6),
        (6, 6),
        (6, 6),
        (6, 6),
    ]
    assert [row.kind for frame in frames for row in frame.events] == [
        "DockCandidateDetected",
        "DockCommitted",
        "AssemblyMerged",
        "UndockCommitted",
        "AssemblySplit",
        # The zero-retract faces remain coincident, so candidacy is observed
        # again during the release step without being explicitly re-latched.
        "DockCandidateDetected",
    ]
    final = frames[-1]
    assert final.metrics.docking_success_count == 1
    assert final.metrics.undocking_success_count == 1
    assert final.metrics.connection_count_active == 0
