"""Inspector launch checks stay separate from pack format and simulator execution."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from types import ModuleType

import pytest
from typer.testing import CliRunner

from modsim_robotics.backends.base import BackendAdapter
from modsim_robotics.backends.mock import MockBackendAdapter
from modsim_robotics.backends.registry import BackendEntry
from modsim_robotics.cli import app
from modsim_robotics.robot_packs import (
    LoadedRobotPack,
    ModelViewMode,
    ModelViewSpec,
    RobotPack,
    RobotPackLoader,
    RobotPackValidator,
    ValidationProfile,
)
from modsim_robotics.runtime import RuntimeDemo, RuntimeInspectorConfig
from modsim_robotics.runtime.inspector_runner import validate_runtime_inspector_config
from modsim_robotics_studio import runtime_launch
from modsim_robotics_studio.project import StudioProject
from modsim_robotics_studio.runtime_launch import (
    add_default_topology_view,
    runtime_config_defaults,
    runtime_recipes,
    validate_runtime_launch,
)


def _with_recipes(loaded: LoadedRobotPack, *recipes: ModelViewSpec) -> LoadedRobotPack:
    data = loaded.pack.model_dump(mode="python")
    data["manifest"]["model_views"] = recipes
    return replace(loaded, pack=RobotPack.model_validate(data))


def test_preflight_does_not_instantiate_backend(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def forbidden_factory() -> BackendAdapter:
        pytest.fail("readiness must not create an engine")

    entry = BackendEntry(name="mock", summary="test", loader=lambda: forbidden_factory)

    def test_entry(name: str) -> BackendEntry:
        return entry

    monkeypatch.setattr(runtime_launch, "backend_entry", test_entry)
    loaded = RobotPackLoader().load(example_pack_dir)
    config = runtime_config_defaults(example_pack_dir, backend="mock")
    report = validate_runtime_launch(loaded, config)
    assert report.valid, report.errors


def test_missing_view_is_inspector_error_not_pack_format_error(example_pack_dir: Path) -> None:
    loaded = _with_recipes(RobotPackLoader().load(example_pack_dir))
    assert RobotPackValidator().validate(loaded, profile=ValidationProfile.SIMULATION).valid
    report = validate_runtime_launch(
        loaded, runtime_config_defaults(example_pack_dir, backend="mock")
    )
    assert not report.valid
    assert [issue.code for issue in report.errors] == ["runtime.model_view"]
    assert "add a topology view" in (report.errors[0].suggested_fix or "")


def test_explicit_nondefault_runtime_view_is_ready(example_pack_dir: Path) -> None:
    recipe = ModelViewSpec(id="selected_view", builder="module_topology_graph", default=False)
    loaded = _with_recipes(RobotPackLoader().load(example_pack_dir), recipe)
    config = replace(
        runtime_config_defaults(example_pack_dir, backend="mock"), view_id="selected_view"
    )
    assert validate_runtime_launch(loaded, config).valid


@pytest.mark.parametrize(
    ("builder", "configuration", "message"),
    [
        ("missing_plugin", {}, "not registered"),
        ("module_topology_graph", {"unexpected": True}, "configuration"),
        ("cubic_lattice", {}, "configuration"),
    ],
)
def test_preflight_explains_unknown_builder_and_bad_configuration(
    example_pack_dir: Path,
    builder: str,
    configuration: dict[str, object],
    message: str,
) -> None:
    recipe = ModelViewSpec.model_validate(
        {"id": "broken", "builder": builder, "default": True, "configuration": configuration}
    )
    loaded = _with_recipes(RobotPackLoader().load(example_pack_dir), recipe)
    report = validate_runtime_launch(
        loaded, runtime_config_defaults(example_pack_dir, backend="mock")
    )
    errors = [issue for issue in report.errors if issue.code == "runtime.model_view"]
    assert len(errors) == 1
    assert message in errors[0].message


def test_preflight_reports_missing_optional_backend(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def missing_dependency() -> type[BackendAdapter]:
        raise ImportError("engine is absent")

    entry = BackendEntry(
        name="mujoco", summary="test", loader=missing_dependency, install_hint="Install MuJoCo"
    )

    def test_entry(name: str) -> BackendEntry:
        return entry

    monkeypatch.setattr(runtime_launch, "backend_entry", test_entry)
    report = validate_runtime_launch(
        RobotPackLoader().load(example_pack_dir), runtime_config_defaults(example_pack_dir)
    )
    assert any(
        issue.code == "runtime.backend" and "Install MuJoCo" in issue.message
        for issue in report.errors
    )


def test_preflight_includes_simulation_metadata_and_explicit_connector_errors(
    example_pack_dir: Path,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    data = loaded.pack.model_dump(mode="python")
    data["hardware_catalog"]["module_types"]["generic_cube"]["connectors"][0]["docking_axis"] = None
    changed = replace(loaded, pack=RobotPack.model_validate(data))
    config = replace(
        runtime_config_defaults(example_pack_dir, backend="mock"),
        fixed_connector="not_present",
        moving_connector="front",
    )
    report = validate_runtime_launch(changed, config)
    assert any(issue.code == "runtime.connectors" for issue in report.errors)
    assert any(issue.code.startswith("connector.") for issue in report.errors)


def test_preflight_rejects_incompatible_explicit_pair(example_pack_dir: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    data = loaded.pack.model_dump(mode="python")
    connector_type = next(iter(data["hardware_catalog"]["connector_types"].values()))
    connector_type["gender"] = "male"
    loaded = replace(loaded, pack=RobotPack.model_validate(data))
    config = replace(
        runtime_config_defaults(example_pack_dir, backend="mock"),
        fixed_connector="front",
        moving_connector="rear",
    )
    report = validate_runtime_launch(loaded, config)
    assert any(
        issue.code == "runtime.connectors" and "incompatible" in issue.message
        for issue in report.errors
    )


@pytest.mark.parametrize("demo", list(RuntimeDemo))
def test_demo_presets_produce_valid_execution_options(demo: RuntimeDemo) -> None:
    config = runtime_config_defaults(Path("pack"), demo)
    validate_runtime_inspector_config(config)
    assert config.viewer_enabled


@pytest.mark.parametrize("demo", list(RuntimeDemo))
def test_platform_presets_accept_matching_committed_pack(
    demo: RuntimeDemo, example_pack_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def test_entry(name: str) -> BackendEntry:
        return BackendEntry(name=name, summary="test", loader=lambda: MockBackendAdapter)

    monkeypatch.setattr(runtime_launch, "backend_entry", test_entry)
    pack_id = (
        "smores_ep"
        if demo.value.startswith("smores_")
        else ("mblocks_3d" if demo.value.startswith("mblocks_") else "generic_cube")
    )
    path = example_pack_dir.parent / pack_id
    loaded = RobotPackLoader().load(path)
    config = runtime_config_defaults(path, demo)
    report = validate_runtime_launch(loaded, config)
    assert report.valid, report.errors


@pytest.mark.parametrize("demo", list(RuntimeDemo))
def test_studio_defaults_stay_equivalent_to_cli_presets(
    demo: RuntimeDemo,
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: list[RuntimeInspectorConfig] = []

    def capture(config: RuntimeInspectorConfig) -> int:
        captured.append(config)
        return 0

    fake_app = ModuleType("modsim_robotics_studio.runtime_app")
    fake_app.__dict__["main"] = capture
    monkeypatch.setitem(sys.modules, "modsim_robotics_studio.runtime_app", fake_app)
    result = CliRunner().invoke(app, ["runtime", str(example_pack_dir), "--demo", demo.value])
    assert result.exit_code == 0, result.output
    studio = runtime_config_defaults(example_pack_dir, demo)
    # The CLI leaves special M-Blocks view selection to its runner. Studio
    # makes that same selection visible in the form before the runner starts.
    assert studio == replace(captured[0], view_id=studio.view_id)


def test_wrong_platform_pack_has_actionable_error(example_pack_dir: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    config = runtime_config_defaults(example_pack_dir, RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK)
    report = validate_runtime_launch(loaded, config)
    assert any(issue.code == "runtime.demo_pack" for issue in report.errors)


def test_add_topology_view_edits_only_memory_and_preserves_existing_data(
    copied_pack: Path,
) -> None:
    original = StudioProject.open(copied_pack)
    before = (copied_pack / "robot_pack.yaml").read_bytes()
    updated = add_default_topology_view(original)
    assert updated.dirty
    assert not original.dirty
    assert updated.pack.manifest.model_views[:-1] == original.pack.manifest.model_views
    added = updated.pack.manifest.model_views[-1]
    assert added.id == "module_topology_2"
    assert not added.default
    assert added.builder == "module_topology_graph"
    assert (copied_pack / "robot_pack.yaml").read_bytes() == before


def test_added_view_becomes_default_when_only_authoring_default_exists(
    example_pack_dir: Path,
) -> None:
    original = StudioProject.open(example_pack_dir)
    authoring = ModelViewSpec(
        id="authoring", builder="external", modes=(ModelViewMode.AUTHORING,), default=True
    )
    project = replace(original, loaded=_with_recipes(original.loaded, authoring))
    updated = add_default_topology_view(project)
    assert runtime_recipes(updated.pack)[0].default
    assert runtime_recipes(updated.pack)[0].id == "module_topology"
    assert validate_runtime_launch(
        updated.loaded, runtime_config_defaults(example_pack_dir, backend="mock")
    ).valid
