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

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc_value, traceback

    def lock(self) -> AbstractContextManager[None]:
        return nullcontext()

    def is_running(self) -> bool:
        return True


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

    run_with_viewer(
        session,
        duration_s=0.0,
        step_once=lambda: (),
        hold=False,
    )

    assert viewer.opt.geomgroup[URDF_COLLISION_GEOM_GROUP] == 0
    assert viewer.opt.geomgroup[1] == 1
    assert viewer.opt.geomgroup[ENVIRONMENT_GEOM_GROUP] == 1
