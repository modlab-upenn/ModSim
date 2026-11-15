"""Commands require a named or explicit pack from the selected user library."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import sys
from pathlib import Path
from types import ModuleType

import pytest
from typer.testing import CliRunner

from modsim_robotics import cli
from modsim_robotics.runtime import RuntimeInspectorConfig
from modsim_robotics.user_data import initialize_user_home, resolve_robot_pack

runner = CliRunner()
READ_COMMANDS = (
    ("pack", "inspect"),
    ("pack", "validate"),
    ("views",),
    ("dock", "--count", "2"),
    ("run", "--duration", "0.002"),
    ("runtime", "--backend", "mock", "--no-viewer"),
    ("run", "--gui", "--backend", "mock", "--no-viewer"),
    ("studio",),
)


@pytest.fixture(autouse=True)
def application_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    home = tmp_path / "application-home"
    monkeypatch.setenv("MODSIM_PATH", str(home))
    monkeypatch.delenv("MODSIM_HOME", raising=False)
    return home


@pytest.fixture
def gui_launches(monkeypatch: pytest.MonkeyPatch) -> list[Path]:
    """Capture GUI dispatch without importing native desktop dependencies."""
    paths: list[Path] = []
    studio = ModuleType("modsim_robotics_studio.app")
    runtime = ModuleType("modsim_robotics_studio.runtime_app")

    def studio_main(pack_path: Path | None) -> int:
        assert pack_path is not None
        paths.append(pack_path)
        return 0

    def runtime_main(config: RuntimeInspectorConfig) -> int:
        paths.append(config.pack_path)
        return 0

    studio.main = studio_main  # type: ignore[attr-defined]
    runtime.main = runtime_main  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, studio.__name__, studio)
    monkeypatch.setitem(sys.modules, runtime.__name__, runtime)
    return paths


def _install_pack(source: Path, destination: Path, *, pack_id: str, version: str) -> Path:
    shutil.copytree(source, destination)
    manifest = destination / "robot_pack.yaml"
    manifest.write_text(
        manifest.read_text()
        .replace("\nid: generic_cube\n", f"\nid: {pack_id}\n")
        .replace("\nversion: 0.1.0\n", f"\nversion: {version}\n")
    )
    return destination


@pytest.mark.parametrize("command", READ_COMMANDS)
@pytest.mark.parametrize("style", ("positional", "option"))
def test_named_pack_selection_resolves_once_and_preserves_setup_json(
    application_home: Path,
    gui_launches: list[Path],
    monkeypatch: pytest.MonkeyPatch,
    command: tuple[str, ...],
    style: str,
) -> None:
    setup = runner.invoke(cli.app, ["setup", "--output", "json"])
    assert setup.exit_code == 0, setup.output
    payload = json.loads(setup.stdout)
    assert set(payload) == {"home", "starter_pack", "created"}
    assert payload["created"]
    configuration = (application_home / "config.toml").read_bytes()
    resolutions: list[str | Path | None] = []

    def capture_resolution(
        pack_path: str | Path | None = None, *, home: Path | None = None
    ) -> Path:
        resolutions.append(pack_path)
        return resolve_robot_pack(pack_path, home=home)

    monkeypatch.setattr(cli, "resolve_robot_pack", capture_resolution)
    arguments = [*command]
    arguments.extend(["generic_cube"] if style == "positional" else ["--pack", "generic_cube"])

    result = runner.invoke(cli.app, arguments)

    assert result.exit_code == 0, result.output
    assert resolutions == ["generic_cube"]
    if gui_launches:
        assert gui_launches == [Path(payload["starter_pack"])]
    else:
        assert "generic_cube@0.1.0" in result.stdout
    assert (application_home / "config.toml").read_bytes() == configuration


@pytest.mark.parametrize("command", READ_COMMANDS)
def test_pack_choice_is_required_even_when_only_one_pack_is_installed(
    application_home: Path, gui_launches: list[Path], command: tuple[str, ...]
) -> None:
    initialize_user_home(application_home)
    configuration = (application_home / "config.toml").read_bytes()

    result = runner.invoke(cli.app, list(command))

    assert result.exit_code == 1, result.output
    assert "pack" in result.stderr.lower()
    assert not gui_launches
    assert (application_home / "config.toml").read_bytes() == configuration


@pytest.mark.parametrize("command", READ_COMMANDS)
def test_named_pack_lookup_reports_missing_setup_without_creating_storage(
    application_home: Path, command: tuple[str, ...]
) -> None:
    result = runner.invoke(cli.app, [*command, "generic_cube"])

    assert result.exit_code == 1, result.output
    assert "setup" in result.stderr
    assert str(application_home) in result.stderr
    assert not application_home.exists()


@pytest.mark.parametrize("command", READ_COMMANDS)
@pytest.mark.parametrize("style", ("positional", "option"))
def test_explicit_paths_bypass_missing_config_and_resolve_once(
    application_home: Path,
    example_pack_dir: Path,
    gui_launches: list[Path],
    monkeypatch: pytest.MonkeyPatch,
    command: tuple[str, ...],
    style: str,
) -> None:
    resolutions: list[str | Path | None] = []

    def capture_resolution(
        pack_path: str | Path | None = None, *, home: Path | None = None
    ) -> Path:
        resolutions.append(pack_path)
        return resolve_robot_pack(pack_path, home=home)

    monkeypatch.setattr(cli, "resolve_robot_pack", capture_resolution)
    arguments = [*command]
    arguments.extend(
        [str(example_pack_dir)] if style == "positional" else ["--pack", str(example_pack_dir)]
    )

    result = runner.invoke(cli.app, arguments)

    assert result.exit_code == 0, result.output
    assert resolutions == [str(example_pack_dir)]
    if gui_launches:
        assert gui_launches == [example_pack_dir]
    assert not application_home.exists()


@pytest.mark.parametrize("command", READ_COMMANDS)
def test_reader_rejects_both_selector_forms(
    application_home: Path, command: tuple[str, ...]
) -> None:
    result = runner.invoke(cli.app, [*command, "generic_cube", "--pack", "other_cube"])

    assert result.exit_code == 2
    assert "not both" in result.stderr
    assert not application_home.exists()


@pytest.mark.parametrize("command", READ_COMMANDS)
def test_home_override_selects_the_library_for_a_named_pack(
    tmp_path: Path,
    application_home: Path,
    gui_launches: list[Path],
    command: tuple[str, ...],
) -> None:
    selected = initialize_user_home(tmp_path / "selected-home")

    result = runner.invoke(cli.app, [*command, "generic_cube", "--home", str(selected.home)])

    assert result.exit_code == 0, result.output
    if gui_launches:
        assert gui_launches == [selected.starter_pack]
    assert not application_home.exists()


def test_library_lists_manifest_names_and_requires_versions_when_ambiguous(
    application_home: Path, example_pack_dir: Path
) -> None:
    original = initialize_user_home(application_home)
    second = _install_pack(
        example_pack_dir,
        application_home / "robot_packs/unrelated-folder/second",
        pack_id="generic_cube",
        version="0.2.0",
    )
    another = _install_pack(
        example_pack_dir,
        application_home / "robot_packs/arbitrary-folder",
        pack_id="other_cube",
        version="1.0.0",
    )

    listed = runner.invoke(cli.app, ["pack", "list", "--output", "json"])

    assert listed.exit_code == 0, listed.output
    assert json.loads(listed.stdout) == {
        "packs": [
            {"id": "generic_cube", "version": "0.1.0", "path": str(original.starter_pack)},
            {"id": "generic_cube", "version": "0.2.0", "path": str(second)},
            {"id": "other_cube", "version": "1.0.0", "path": str(another)},
        ]
    }
    ambiguous = runner.invoke(cli.app, ["pack", "inspect", "generic_cube", "--output", "json"])
    assert ambiguous.exit_code == 1
    assert ambiguous.stdout == ""
    assert "0.1.0" in ambiguous.stderr and "0.2.0" in ambiguous.stderr
    selected = runner.invoke(
        cli.app, ["pack", "inspect", "--pack", "generic_cube@0.2.0", "--output", "json"]
    )
    assert selected.exit_code == 0, selected.output
    assert json.loads(selected.stdout)["manifest"]["version"] == "0.2.0"
    other = runner.invoke(cli.app, ["pack", "inspect", "other_cube", "--output", "json"])
    assert other.exit_code == 0, other.output
    assert json.loads(other.stdout)["manifest"]["id"] == "other_cube"
    text = runner.invoke(cli.app, ["pack", "list"])
    assert text.exit_code == 0, text.output
    assert "Installed Robot Packs" in text.stdout
    assert "generic_cube" in text.stdout and "other_cube" in text.stdout

    _install_pack(
        example_pack_dir,
        application_home / "robot_packs/duplicate-version",
        pack_id="generic_cube",
        version="0.2.0",
    )
    duplicate = runner.invoke(cli.app, ["pack", "inspect", "generic_cube@0.2.0"])
    explicit = runner.invoke(
        cli.app, ["pack", "inspect", "--pack", str(second), "--output", "json"]
    )
    assert duplicate.exit_code == 1
    assert "explicit path" in duplicate.stderr
    assert explicit.exit_code == 0, explicit.output
    assert json.loads(explicit.stdout)["manifest"]["version"] == "0.2.0"


def test_library_directory_is_relative_to_home_not_cwd(
    tmp_path: Path, application_home: Path, example_pack_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    initialize_user_home(application_home)
    installed = _install_pack(
        example_pack_dir,
        application_home / "custom-library/nested",
        pack_id="other_cube",
        version="1.0.0",
    )
    (application_home / "config.toml").write_text(
        'schema_version = 1\n[paths]\nrobot_packs = "custom-library"\n'
    )
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)

    listed = runner.invoke(cli.app, ["pack", "list", "--output", "json"])
    selected = runner.invoke(cli.app, ["pack", "inspect", "other_cube", "--output", "json"])

    assert listed.exit_code == 0, listed.output
    assert json.loads(listed.stdout) == {
        "packs": [{"id": "other_cube", "version": "1.0.0", "path": str(installed)}]
    }
    assert selected.exit_code == 0, selected.output
    assert json.loads(selected.stdout)["manifest"]["id"] == "other_cube"


def test_bare_names_are_library_ids_and_dot_paths_are_explicit(
    tmp_path: Path,
    application_home: Path,
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_pack(
        example_pack_dir, tmp_path / "generic_cube", pack_id="local_cube", version="1.0.0"
    )
    monkeypatch.chdir(tmp_path)

    named = runner.invoke(cli.app, ["pack", "inspect", "generic_cube"])
    explicit = runner.invoke(
        cli.app, ["pack", "inspect", "--pack", "./generic_cube", "--output", "json"]
    )

    assert named.exit_code == 1
    assert "setup" in named.stderr
    assert explicit.exit_code == 0, explicit.output
    assert json.loads(explicit.stdout)["manifest"]["id"] == "local_cube"
    assert not application_home.exists()


def test_legacy_default_is_never_used_to_select_a_pack(
    application_home: Path, copied_pack: Path
) -> None:
    initialize_user_home(application_home)
    manifest = copied_pack / "robot_pack.yaml"
    manifest.write_text(manifest.read_text().replace("\nid: generic_cube\n", "\nid: legacy_cube\n"))
    (application_home / "config.toml").write_text(
        "schema_version = 1\n[defaults]\nrobot_pack = " + json.dumps(str(copied_pack)) + "\n"
    )

    missing = runner.invoke(cli.app, ["pack", "inspect", "--output", "json"])
    named = runner.invoke(cli.app, ["pack", "inspect", "generic_cube", "--output", "json"])

    assert missing.exit_code == 1
    assert missing.stdout == ""
    assert named.exit_code == 0, named.output
    assert json.loads(named.stdout)["manifest"]["id"] == "generic_cube"
    unknown = runner.invoke(cli.app, ["pack", "inspect", "legacy_cube"])
    assert unknown.exit_code == 1
    assert "No installed Robot Pack" in unknown.stderr


def test_library_home_accepts_the_legacy_environment_variable(
    application_home: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    initialize_user_home(application_home)
    monkeypatch.delenv("MODSIM_PATH")
    monkeypatch.setenv("MODSIM_HOME", str(application_home))

    result = runner.invoke(cli.app, ["pack", "inspect", "generic_cube"])

    assert result.exit_code == 0, result.output
    assert "generic_cube@0.1.0" in result.stdout


@pytest.mark.parametrize("style", ("positional", "option"))
@pytest.mark.parametrize("kind", ("directory", "manifest"))
def test_explicit_paths_preserve_symlinks_for_loader_rejection(
    tmp_path: Path, application_home: Path, example_pack_dir: Path, style: str, kind: str
) -> None:
    if kind == "directory":
        selected = tmp_path / "linked-pack"
        target = example_pack_dir
        expected_issue = "pack.root_symlink_not_allowed"
    else:
        selected = tmp_path / "linked-manifest/robot_pack.yaml"
        selected.parent.mkdir()
        target = example_pack_dir / "robot_pack.yaml"
        expected_issue = "document.symlink_not_allowed"
    try:
        selected.symlink_to(target, target_is_directory=kind == "directory")
    except OSError:
        pytest.skip("Symbolic links are unavailable on this platform")
    arguments = ["pack", "inspect", "--output", "json"]
    arguments.extend([str(selected)] if style == "positional" else ["--pack", str(selected)])

    result = runner.invoke(cli.app, arguments)

    assert result.exit_code == 1, result.output
    assert json.loads(result.stdout)["issues"][0]["code"] == expected_issue
    assert not application_home.exists()


@pytest.mark.parametrize(
    "command",
    (
        ("--help",),
        ("--version",),
        ("setup", "--help"),
        ("pack", "init", "--help"),
        ("pack", "list", "--help"),
        *((*command, "--help") for command in READ_COMMANDS),
    ),
)
def test_help_and_version_do_not_read_or_create_the_library(
    application_home: Path, monkeypatch: pytest.MonkeyPatch, command: tuple[str, ...]
) -> None:
    def unexpected_resolution(
        pack_path: str | Path | None = None, *, home: Path | None = None
    ) -> Path:
        raise AssertionError("Help and version must not resolve packs")

    def unexpected_listing(*, home: Path | None = None) -> tuple[object, ...]:
        raise AssertionError("Help must not read the library")

    monkeypatch.setattr(cli, "resolve_robot_pack", unexpected_resolution)
    monkeypatch.setattr(cli, "list_robot_packs", unexpected_listing)

    result = runner.invoke(cli.app, list(command))

    assert result.exit_code == 0, result.output
    assert not application_home.exists()


def test_help_documents_names_and_home_environment_precedence() -> None:
    result = runner.invoke(cli.app, ["--help"])
    assert result.exit_code == 0
    assert "ID@VERSION" in result.stdout
    assert "MODSIM_PATH" in result.stdout
    assert "MODSIM_HOME" in result.stdout
    assert "configured default" not in result.stdout


def test_missing_selector_keeps_json_stdout_clean(application_home: Path) -> None:
    result = runner.invoke(cli.app, ["pack", "inspect", "--output", "json"])
    assert result.exit_code == 1
    assert result.stdout == ""
    assert "pack" in result.stderr.lower()
    assert not application_home.exists()


def test_listing_missing_library_reports_error_without_creating_storage(
    application_home: Path,
) -> None:
    result = runner.invoke(cli.app, ["pack", "list", "--output", "json"])
    assert result.exit_code == 1
    assert result.stdout == ""
    assert "setup" in result.stderr
    assert not application_home.exists()


def test_listing_an_empty_configured_library(application_home: Path) -> None:
    initialize_user_home(application_home)
    (application_home / "empty-library").mkdir()
    (application_home / "config.toml").write_text(
        'schema_version = 1\n[paths]\nrobot_packs = "empty-library"\n'
    )
    result = runner.invoke(cli.app, ["pack", "list", "--output", "json"])
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout) == {"packs": []}


def test_listing_an_explicit_home_ignores_the_environment_library(
    application_home: Path, tmp_path: Path
) -> None:
    selected = initialize_user_home(tmp_path / "other-home")

    result = runner.invoke(
        cli.app, ["pack", "list", "--home", str(selected.home), "--output", "json"]
    )

    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout) == {
        "packs": [{"id": "generic_cube", "version": "0.1.0", "path": str(selected.starter_pack)}]
    }
    assert not application_home.exists()


def test_setup_prints_shell_safe_persistence_commands_without_applying_them(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    home = tmp_path / "robot's $HOME; `command` folder"
    monkeypatch.setattr(cli.sys, "platform", "linux")
    monkeypatch.setenv("MODSIM_PATH", "original-value")

    result = runner.invoke(cli.app, ["setup", "--home", str(home)])

    assert result.exit_code == 0, result.output
    assert f"Configuration: {home / 'config.toml'}" in result.stdout
    lines = [line.strip() for line in result.stdout.splitlines()]
    export = next(line for line in lines if line.startswith("export MODSIM_PATH="))
    assert shlex.split(export) == ["export", f"MODSIM_PATH={home}"]
    profiles = [line for line in lines if line.startswith("printf ")]
    assert [shlex.split(line) for line in profiles] == [
        ["printf", "%s\\n", export, ">>", "~/.bashrc"],
        ["printf", "%s\\n", export, ">>", "~/.zshrc"],
    ]
    assert os.environ["MODSIM_PATH"] == "original-value"


def test_setup_prints_powershell_literals_without_interpolation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    home = tmp_path / "robot's $HOME; `command` folder"
    monkeypatch.setattr(cli.sys, "platform", "win32")

    result = runner.invoke(cli.app, ["setup", "--home", str(home)])

    assert result.exit_code == 0, result.output
    literal = "'" + str(home).replace("'", "''") + "'"
    assert f"$env:MODSIM_PATH = {literal}" in result.stdout
    assert (
        f"[Environment]::SetEnvironmentVariable('MODSIM_PATH', {literal}, 'User')" in result.stdout
    )
