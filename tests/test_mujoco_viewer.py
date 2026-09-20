"""Presentation defaults for the optional MuJoCo passive viewer."""

from __future__ import annotations

import math
from contextlib import AbstractContextManager, nullcontext
from pathlib import Path
from types import TracebackType
from typing import Self

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.core.scene import SceneSpec
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.scene import (
    ENVIRONMENT_GEOM_GROUP,
    URDF_COLLISION_GEOM_GROUP,
)
from modsim_backend_mujoco.viewer import run_with_viewer

pytestmark = pytest.mark.mujoco


class _ViewerOptions:
    def __init__(self) -> None:
        self.geomgroup = [1, 1, 1, 1, 1, 1]


class _PassiveViewer:
    def __init__(self) -> None:
        self.opt = _ViewerOptions()
        self.sync_count = 0
        self.exited = False
        self.text_overlays: list[object] = []

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc_value, traceback
        self.exited = True

    def lock(self) -> AbstractContextManager[None]:
        return nullcontext()

    def is_running(self) -> bool:
        return True

    def sync(self) -> None:
        self.sync_count += 1

    def set_texts(self, texts: object) -> None:
        self.text_overlays.append(texts)


def test_terminal_scenario_freezes_physics_and_keeps_viewer_interactive(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session = RuntimeSession.create(
        RobotPackLoader().load(example_pack_dir),
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
    )
    viewer = _PassiveViewer()
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive", lambda *_args: viewer
    )
    complete: list[float] = []
    try:
        run_with_viewer(
            session,
            duration_s=10.0,
            step_once=lambda: session.step(0.01),
            execution_finished=lambda: session.world.time_s >= 0.02,
            pause_requested=lambda: False,
            on_scenario_complete=lambda: complete.append(session.world.time_s),
            stop_requested=lambda: bool(complete) and viewer.sync_count >= 5,
            hold=True,
        )
        assert session.world.time_s == pytest.approx(0.02)
        assert complete == [pytest.approx(0.02)]
        assert viewer.sync_count >= 5
        assert "FINISHED" in str(viewer.text_overlays[-1])
    finally:
        session.shutdown()


def test_viewer_starts_with_collision_proxies_hidden(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    viewer = _PassiveViewer()

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )

    try:
        run_with_viewer(
            session,
            duration_s=0.0,
            step_once=lambda: (),
            hold=False,
        )
    finally:
        session.shutdown()

    assert viewer.opt.geomgroup[URDF_COLLISION_GEOM_GROUP] == 0
    assert viewer.opt.geomgroup[1] == 1
    assert viewer.opt.geomgroup[ENVIRONMENT_GEOM_GROUP] == 1
    assert viewer.text_overlays == []


def test_viewer_scales_wall_deadlines_without_changing_simulated_steps(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    viewer = _PassiveViewer()
    deadlines: list[float] = []
    simulated_started = session.world.time_s

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    def step_once() -> tuple[()]:
        session.step(0.01)
        return ()

    def record_deadline(target_s: float, stop_requested: object) -> bool:
        del stop_requested
        deadlines.append(target_s)
        return True

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )
    monkeypatch.setattr("modsim_backend_mujoco.viewer.time.perf_counter", lambda: 100.0)
    monkeypatch.setattr("modsim_backend_mujoco.viewer._wait_until", record_deadline)

    try:
        run_with_viewer(
            session,
            duration_s=0.02,
            step_once=step_once,
            real_time_factor=4.0,
            hold=False,
        )
    finally:
        session.shutdown()

    assert session.world.time_s - simulated_started == pytest.approx(0.02)
    assert deadlines == pytest.approx([100.0025, 100.005])
    assert viewer.sync_count == 2


def test_viewer_sync_is_throttled_without_skipping_physics_steps(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    viewer = _PassiveViewer()
    wall_time = 100.0
    step_count = 0

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    def step_once() -> tuple[()]:
        nonlocal wall_time, step_count
        session.step(0.002)
        wall_time += 0.002
        step_count += 1
        return ()

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.time.perf_counter",
        lambda: wall_time,
    )
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer._wait_until",
        lambda target_s, stop_requested: True,
    )

    try:
        run_with_viewer(
            session,
            duration_s=0.02,
            step_once=step_once,
            hold=False,
        )
    finally:
        session.shutdown()

    assert step_count == 10
    assert viewer.sync_count == 2


def test_slow_viewer_sync_does_not_trigger_again_on_the_next_physics_step(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    wall_time = 100.0

    class _SlowPassiveViewer(_PassiveViewer):
        def sync(self) -> None:
            nonlocal wall_time
            super().sync()
            wall_time += 0.03

    viewer = _SlowPassiveViewer()
    step_count = 0

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    def step_once() -> tuple[()]:
        nonlocal wall_time, step_count
        session.step(0.002)
        wall_time += 0.002
        step_count += 1
        return ()

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.time.perf_counter",
        lambda: wall_time,
    )
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer._wait_until",
        lambda target_s, stop_requested: True,
    )

    try:
        run_with_viewer(
            session,
            duration_s=0.01,
            step_once=step_once,
            hold=False,
        )
    finally:
        session.shutdown()

    assert step_count == 5
    assert viewer.sync_count == 2


@pytest.mark.parametrize("real_time_factor", (0.0, -1.0, math.nan, math.inf, -math.inf))
def test_viewer_rejects_invalid_real_time_factor_before_opening(
    example_pack_dir: Path,
    real_time_factor: float,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )

    try:
        with pytest.raises(ValueError, match="real_time_factor"):
            run_with_viewer(
                session,
                duration_s=0.0,
                step_once=lambda: (),
                real_time_factor=real_time_factor,
                hold=False,
            )
    finally:
        session.shutdown()


def test_viewer_supports_cooperative_stop_and_lifecycle_hooks(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    viewer = _PassiveViewer()
    stopped = False
    lifecycle: list[str] = []

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )

    def step_once() -> tuple[()]:
        nonlocal stopped
        lifecycle.append("step")
        stopped = True
        return ()

    try:
        run_with_viewer(
            session,
            duration_s=1.0,
            step_once=step_once,
            stop_requested=lambda: stopped,
            on_started=lambda: lifecycle.append("started"),
            after_step=lambda: lifecycle.append("after_step"),
            on_stopped=lambda: lifecycle.append("stopped"),
        )
    finally:
        session.shutdown()

    assert lifecycle == ["started", "step", "after_step", "stopped"]
    assert viewer.sync_count == 1
    assert viewer.exited


def test_viewer_cooperative_stop_interrupts_final_state_hold(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    viewer = _PassiveViewer()
    lifecycle: list[str] = []

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )

    try:
        run_with_viewer(
            session,
            duration_s=0.0,
            step_once=lambda: (),
            stop_requested=lambda: viewer.sync_count >= 2,
            on_scenario_complete=lambda: lifecycle.append("complete"),
            on_stopped=lambda: lifecycle.append("stopped"),
        )
    finally:
        session.shutdown()

    assert viewer.sync_count == 2
    assert viewer.exited
    assert lifecycle == ["complete", "stopped"]


def test_viewer_passes_through_key_callback_and_advertises_space_action(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    viewer = _PassiveViewer()
    received_callback: object | None = None
    lifecycle: list[str] = []

    def key_callback(keycode: int) -> None:
        del keycode

    def launch_passive(
        model: object,
        data: object,
        *,
        key_callback: object,
    ) -> _PassiveViewer:
        nonlocal received_callback
        del model, data
        received_callback = key_callback
        return viewer

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )

    try:
        run_with_viewer(
            session,
            duration_s=0.0,
            step_once=lambda: (),
            key_callback=key_callback,
            on_started=lambda: lifecycle.append("started"),
            on_pause_changed=lambda paused: lifecycle.append(f"pause:{paused}"),
            hold=False,
        )
    finally:
        session.shutdown()

    assert received_callback is key_callback
    assert lifecycle == ["started", "pause:False"]
    assert len(viewer.text_overlays) == 1
    overlay = viewer.text_overlays[0]
    assert isinstance(overlay, tuple)
    assert overlay[2:] == ("ModSim playback", "RUNNING\nSpace: Pause")


def test_viewer_pause_keeps_rendering_and_resume_rebases_wall_pacing(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    paused = True
    wall_time = 100.0

    class _ResumingViewer(_PassiveViewer):
        def sync(self) -> None:
            nonlocal paused, wall_time
            super().sync()
            if paused:
                # Model a long wall-clock pause ending through a viewer event.
                wall_time = 130.0
                paused = False

    viewer = _ResumingViewer()
    transitions: list[bool] = []
    step_start_times: list[float] = []
    wait_deadlines: list[float] = []

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    def step_once() -> tuple[()]:
        step_start_times.append(session.world.time_s)
        session.step(0.01)
        return ()

    def record_wait(target_s: float, wake_requested: object) -> bool:
        del wake_requested
        wait_deadlines.append(target_s)
        return True

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.time.perf_counter",
        lambda: wall_time,
    )
    monkeypatch.setattr("modsim_backend_mujoco.viewer._wait_until", record_wait)

    try:
        run_with_viewer(
            session,
            duration_s=0.02,
            step_once=step_once,
            pause_requested=lambda: paused,
            on_pause_changed=transitions.append,
            hold=False,
        )
    finally:
        session.shutdown()

    assert step_start_times == pytest.approx([0.0, 0.01])
    assert transitions == [False, True, False]
    assert viewer.sync_count == 3
    assert wait_deadlines[-2:] == pytest.approx([130.01, 130.02])
    overlay_text = [overlay[3] for overlay in viewer.text_overlays]  # type: ignore[index]
    assert overlay_text == ["RUNNING", "PAUSED", "RUNNING"]


def test_pause_arriving_during_pacing_is_acknowledged_before_another_step(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    paused = False
    wait_count = 0
    timeline: list[str] = []

    class _ResumeFromViewer(_PassiveViewer):
        def sync(self) -> None:
            nonlocal paused
            super().sync()
            if paused:
                paused = False

    viewer = _ResumeFromViewer()

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    def step_once() -> tuple[()]:
        timeline.append("step")
        session.step(0.01)
        return ()

    def pause_changed(value: bool) -> None:
        timeline.append(f"pause:{value}")

    def interrupt_first_pacing_wait(target_s: float, wake_requested: object) -> bool:
        nonlocal paused, wait_count
        del target_s
        wait_count += 1
        if wait_count == 1:
            paused = True
            assert callable(wake_requested)
            assert wake_requested()
            return False
        return True

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer._wait_until",
        interrupt_first_pacing_wait,
    )

    try:
        run_with_viewer(
            session,
            duration_s=0.02,
            step_once=step_once,
            pause_requested=lambda: paused,
            on_pause_changed=pause_changed,
            hold=False,
        )
    finally:
        session.shutdown()

    assert timeline == [
        "pause:False",
        "step",
        "pause:True",
        "pause:False",
        "step",
    ]


def test_viewer_stop_while_paused_never_advances_runtime(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    viewer = _PassiveViewer()
    stopped = False
    step_count = 0

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    def sync_and_stop() -> None:
        nonlocal stopped
        _PassiveViewer.sync(viewer)
        stopped = True

    def step_once() -> tuple[()]:
        nonlocal step_count
        step_count += 1
        session.step(0.01)
        return ()

    viewer.sync = sync_and_stop  # type: ignore[method-assign]
    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )

    try:
        run_with_viewer(
            session,
            duration_s=1.0,
            step_once=step_once,
            stop_requested=lambda: stopped,
            pause_requested=lambda: True,
            hold=False,
        )
    finally:
        session.shutdown()

    assert step_count == 0
    assert session.world.time_s == pytest.approx(0.0)
    assert viewer.sync_count == 1
    assert viewer.exited


def test_viewer_close_while_paused_never_advances_runtime(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid("generic_cube", 1, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )

    class _ClosingViewer(_PassiveViewer):
        def is_running(self) -> bool:
            return self.sync_count == 0

    viewer = _ClosingViewer()
    step_count = 0

    def launch_passive(model: object, data: object) -> _PassiveViewer:
        del model, data
        return viewer

    def step_once() -> tuple[()]:
        nonlocal step_count
        step_count += 1
        session.step(0.01)
        return ()

    monkeypatch.setattr(
        "modsim_backend_mujoco.viewer.mujoco.viewer.launch_passive",
        launch_passive,
    )

    try:
        run_with_viewer(
            session,
            duration_s=1.0,
            step_once=step_once,
            pause_requested=lambda: True,
            hold=False,
        )
    finally:
        session.shutdown()

    assert step_count == 0
    assert session.world.time_s == pytest.approx(0.0)
    assert viewer.sync_count == 1
    assert viewer.exited
