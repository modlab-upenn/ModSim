"""Built-in demo configurations remain usable without a repository checkout."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


def test_demo_builders_run_from_an_isolated_package_copy(tmp_path: Path) -> None:
    """Run without examples or legacy imports, then alongside an unrelated modsim."""
    import modsim_robotics

    package_root = Path(modsim_robotics.__file__).parent
    shutil.copytree(
        package_root,
        tmp_path / "modsim_robotics",
        ignore=shutil.ignore_patterns("__pycache__"),
    )
    (tmp_path / "modsim.py").write_text(
        'identity = "unrelated top-level module"\n', encoding="utf-8"
    )
    script = """
import importlib.abc
import sys
from pathlib import Path

isolated_root = Path(sys.argv[1])
sys.path.insert(0, str(isolated_root))
legacy_namespaces = ("modsim", "modsim_studio", "modsim_backend_mujoco")

class RejectLegacyNamespaces(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == name or fullname.startswith(name + ".") for name in legacy_namespaces):
            raise AssertionError(f"Unexpected dependency on old namespace: {fullname}")
        return None

legacy_guard = RejectLegacyNamespaces()
sys.meta_path.insert(0, legacy_guard)
import modsim_robotics
from modsim_robotics.core.ids import ModuleInstanceId
from modsim_robotics.demo_configs import (
    mblocks_five_module_pivot,
    mblocks_twelve_module_line,
    mblocks_twelve_module_physics,
    mblocks_twelve_module_staircase,
    mblocks_two_module_momentum_pivot,
    smores_driver_to_snake,
    smores_ep_diff_drive_dock_undock,
    smores_ep_physical_driver_to_snake,
)
from modsim_robotics.runtime.inspector_runner import (
    _load_smores_example_plan,
    _load_mblocks_kinematic_pivot_example,
    _load_mblocks_twelve_module_line_example,
)

assert Path(modsim_robotics.__file__).resolve() == isolated_root / "modsim_robotics/__init__.py"

def exercise_builders():
    assert len(_load_smores_example_plan().module_ids) == 7
    assert len(_load_mblocks_kinematic_pivot_example()[1]) == 3
    assert len(_load_mblocks_twelve_module_line_example()[1]) == 11
    assert len(mblocks_two_module_momentum_pivot.build_scene().placements) == 2
    assert mblocks_two_module_momentum_pivot.build_config().dt_s > 0
    assert len(mblocks_twelve_module_physics.build_scene().placements) == 12
    assert len(mblocks_twelve_module_physics.build_plan().pivots) == 11
    assert len(mblocks_twelve_module_staircase.build_scene().placements) == 12
    assert len(mblocks_twelve_module_staircase.build_plan().actions) == 11
    assert len(mblocks_twelve_module_staircase.build_plan(physical=True).actions) == 11
    config = smores_ep_diff_drive_dock_undock.build_config(
        ModuleInstanceId("fixed"), ModuleInstanceId("moving"), dt_s=0.002,
        initial_gap_m=0.02, approach_speed_m_s=0.03, retract_speed_m_s=0.03,
    )
    assert config.dt_s == 0.002
    assert len(smores_ep_physical_driver_to_snake.build_config(
        dt_s=0.002, navigation_speed_m_s=0.03, approach_speed_m_s=0.03,
    ).routes) == 4

exercise_builders()
assert not any(
    loaded == name or loaded.startswith(name + ".")
    for loaded in sys.modules for name in legacy_namespaces
)
sys.meta_path.remove(legacy_guard)
import modsim
assert modsim.identity == "unrelated top-level module"
assert Path(modsim.__file__).resolve() == isolated_root / "modsim.py"
sys.meta_path.insert(0, legacy_guard)
exercise_builders()
assert "mujoco" not in sys.modules
assert "PySide6" not in sys.modules
assert "modsim_robotics_studio" not in sys.modules
assert "modsim_robotics_backend_mujoco" not in sys.modules
"""
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [sys.executable, "-I", "-c", script, str(tmp_path)],
        cwd=tmp_path,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
