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
