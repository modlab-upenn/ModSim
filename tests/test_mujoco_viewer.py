"""Presentation defaults for the optional MuJoCo passive viewer."""

from __future__ import annotations

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
