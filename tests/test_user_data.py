"""User storage initialization preserves edits and provides a runnable starter."""

from __future__ import annotations

import errno
import json
import shutil
import tomllib
from pathlib import Path

import pytest
from typer.testing import CliRunner

from modsim_robotics import user_data
from modsim_robotics.cli import app
from modsim_robotics.robot_packs import RobotPackLoader, RobotPackLoadError
from modsim_robotics.user_data import (
    UserDataError,
    initialize_user_home,
    list_robot_packs,
    resolve_robot_pack,
    resolve_user_home,
)


@pytest.fixture(autouse=True)
def isolated_modsim_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    """Keep storage tests independent of the developer's configured library."""
    monkeypatch.delenv("MODSIM_PATH", raising=False)
    monkeypatch.delenv("MODSIM_HOME", raising=False)


def test_setup_installs_a_valid_starter_and_runs_mock_docking(tmp_path: Path) -> None:
    result = initialize_user_home(tmp_path / "user-data")

    assert result.created
    loaded = RobotPackLoader().load(result.starter_pack)
    assert loaded.pack.id == "generic_cube"
    assert loaded.pack.version == "0.1.0"
    assert result.starter_pack == result.home / "robot_packs/modlab/generic_cube/0.1.0"
    configuration = tomllib.loads((result.home / "config.toml").read_text(encoding="utf-8"))
    assert configuration == {
        "schema_version": 1,
        "paths": {"robot_packs": "robot_packs"},
    }
    assert resolve_robot_pack("generic_cube", home=result.home) == result.starter_pack
    assert (result.home / "workspaces").is_dir()
    run = CliRunner().invoke(
        app,
        ["dock", str(result.starter_pack), "--backend", "mock", "--count", "2", "--output", "json"],
    )
    assert run.exit_code == 0, run.output
    payload = json.loads(run.stdout)
    assert payload["metrics"]["connection_count_active"] == 1


def test_repeated_setup_preserves_configuration_and_valid_pack_edits(tmp_path: Path) -> None:
    first = initialize_user_home(tmp_path)
    config = first.home / "config.toml"
    config.write_text("# My configuration\nschema_version = 1\n", encoding="utf-8")
    manifest = first.starter_pack / "robot_pack.yaml"
    customized = manifest.read_text().replace(
        "Simulator-neutral example pack for the ModSim schema and CLI.", "My edited starter."
    )
    manifest.write_text(customized, encoding="utf-8")
    notes = first.starter_pack / "notes.txt"
    notes.write_text("Preserve my notes.", encoding="utf-8")

    repeated = initialize_user_home(tmp_path)

    assert not repeated.created
    assert repeated.starter_pack == first.starter_pack
    assert manifest.read_text() == customized
    assert notes.read_text() == "Preserve my notes."
    assert config.read_text() == "# My configuration\nschema_version = 1\n"


@pytest.mark.parametrize(
    ("contents", "message"),
    [
        ("not valid TOML", "Cannot read ModSim configuration"),
        ("schema_version = 2", "schema_version = 1"),
        ("schema_version = 1\npaths = 'cube'", "TOML table"),
        ("schema_version = 1\n[paths]\nrobot_packs = 42", "non-empty path string"),
    ],
)
def test_setup_rejects_invalid_configuration_without_replacing_it(
    tmp_path: Path, contents: str, message: str
) -> None:
    config = tmp_path / "config.toml"
    config.write_text(contents, encoding="utf-8")

    with pytest.raises(UserDataError, match=message):
        initialize_user_home(tmp_path)

    assert config.read_text(encoding="utf-8") == contents
    assert not (tmp_path / "robot_packs/modlab/generic_cube/0.1.0").exists()


def test_setup_preserves_custom_settings_and_ignores_a_legacy_default_pack(
    tmp_path: Path,
) -> None:
    config = tmp_path / "config.toml"
    contents = (
        "# My future robot library\nschema_version = 1\norganization = 'my lab'\n\n"
        "[defaults]\nrobot_pack = 'workspaces/future_robot'\n\n"
        "[appearance]\ntheme = 'custom'\n"
    )
    config.write_text(contents, encoding="utf-8")

    first = initialize_user_home(tmp_path)
    repeated = initialize_user_home(tmp_path)

    assert first.created
    assert not repeated.created
    assert first.starter_pack.is_dir()
    assert not (tmp_path / "workspaces/future_robot").exists()
    assert config.read_text(encoding="utf-8") == contents


def test_setup_does_not_replace_an_invalid_existing_pack(tmp_path: Path) -> None:
    first = initialize_user_home(tmp_path)
    manifest = first.starter_pack / "robot_pack.yaml"
    manifest.write_text("invalid: yes\n", encoding="utf-8")

    with pytest.raises(UserDataError, match="was not replaced"):
        initialize_user_home(tmp_path)

    assert manifest.read_text() == "invalid: yes\n"


def test_setup_refuses_a_starter_symlink(tmp_path: Path) -> None:
    destination = tmp_path / "home/robot_packs/modlab/generic_cube/0.1.0"
    destination.parent.mkdir(parents=True)
    outside = tmp_path / "outside"
    outside.mkdir()
    try:
        destination.symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip("Symbolic links are unavailable on this platform")

    with pytest.raises(UserDataError, match="symbolic-link"):
        initialize_user_home(tmp_path / "home")

    assert list(outside.iterdir()) == []


@pytest.mark.parametrize(
    "relative",
    [
        "robot_packs",
        "catalog",
        "cache",
        "cache/downloads",
        "workspaces",
        "robot_packs/modlab",
        "robot_packs/modlab/generic_cube",
        "config.toml",
    ],
)
def test_setup_refuses_symlinks_in_managed_storage(tmp_path: Path, relative: str) -> None:
    home = tmp_path / "home"
    linked_path = home / relative
    linked_path.parent.mkdir(parents=True)
    outside = tmp_path / "outside"
    outside.mkdir()
    notes = outside / "notes.txt"
    notes.write_text("Preserve outside files.", encoding="utf-8")
    target = notes if relative == "config.toml" else outside
    try:
        linked_path.symlink_to(target, target_is_directory=target.is_dir())
    except OSError:
        pytest.skip("Symbolic links are unavailable on this platform")

    with pytest.raises(UserDataError, match="symbolic-link"):
        initialize_user_home(home)

    assert linked_path.is_symlink()
    assert list(outside.iterdir()) == [notes]
    assert notes.read_text() == "Preserve outside files."


def test_setup_accepts_an_explicit_home_symlink(tmp_path: Path) -> None:
    home = tmp_path / "actual-home"
    home.mkdir()
    alias = tmp_path / "chosen-home"
    try:
        alias.symlink_to(home, target_is_directory=True)
    except OSError:
        pytest.skip("Symbolic links are unavailable on this platform")

    result = initialize_user_home(alias)

    assert result.created
    assert result.home == home
    assert result.starter_pack.is_dir()


def test_setup_preserves_an_empty_destination_created_during_staging(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, example_pack_dir: Path
) -> None:
    home = tmp_path / "home"
    destination = home / "robot_packs/modlab/generic_cube/0.1.0"

    def create_empty_destination() -> Path:
        destination.mkdir()
        return example_pack_dir

    monkeypatch.setattr(user_data, "_starter_source", create_empty_destination)

    with pytest.raises(UserDataError, match="was not replaced"):
        initialize_user_home(home)

    assert list(destination.iterdir()) == []
    assert list(destination.parent.iterdir()) == [destination]


@pytest.mark.parametrize("error_number", [errno.EEXIST, errno.ENOTEMPTY])
def test_setup_keeps_a_valid_concurrent_install_after_rename_conflict(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, error_number: int
) -> None:
    destination = tmp_path / "robot_packs/modlab/generic_cube/0.1.0"

    def concurrent_install(staged: Path, target: Path) -> Path:
        assert target == destination
        shutil.copytree(staged, target)
        (target / "notes.txt").write_text("Concurrent installation.", encoding="utf-8")
        raise OSError(error_number, "Destination appeared during rename", str(target))

    monkeypatch.setattr(Path, "rename", concurrent_install)

    result = initialize_user_home(tmp_path)

    assert not result.created
    assert result.starter_pack == destination
    assert (destination / "notes.txt").read_text() == "Concurrent installation."
    assert RobotPackLoader().load(destination).pack.id == "generic_cube"
    assert list(destination.parent.iterdir()) == [destination]


def test_setup_cleans_staging_when_the_bundled_pack_is_invalid(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "broken-source"
    source.mkdir()
    (source / "robot_pack.yaml").write_text("invalid: yes\n", encoding="utf-8")
    monkeypatch.setattr("modsim_robotics.user_data._starter_source", lambda: source)

    with pytest.raises(UserDataError, match="was not replaced"):
        initialize_user_home(tmp_path / "home")

    parent = tmp_path / "home/robot_packs/modlab/generic_cube"
    assert list(parent.iterdir()) == []


def test_home_resolution_obeys_explicit_environment_and_default_locations(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    monkeypatch.delenv("MODSIM_HOME", raising=False)
    assert resolve_user_home() == tmp_path / ".modsim"
    monkeypatch.setenv("MODSIM_HOME", str(tmp_path / "environment"))
    assert resolve_user_home() == tmp_path / "environment"
    monkeypatch.setenv("MODSIM_PATH", str(tmp_path / "standard"))
    assert resolve_user_home() == tmp_path / "standard"
    assert resolve_user_home(tmp_path / "explicit") == tmp_path / "explicit"
    monkeypatch.setenv("MODSIM_HOME", "")
    assert resolve_user_home() == tmp_path / "standard"
    monkeypatch.setenv("MODSIM_PATH", " ")
    with pytest.raises(UserDataError, match=r"MODSIM_PATH.*empty"):
        resolve_user_home()
    monkeypatch.delenv("MODSIM_PATH")
    with pytest.raises(UserDataError, match=r"MODSIM_HOME.*empty"):
        resolve_user_home()
    assert resolve_user_home(tmp_path / "explicit") == tmp_path / "explicit"
    assert not (tmp_path / ".modsim").exists()


def test_cli_setup_reports_json_and_honors_explicit_home(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("MODSIM_HOME", str(tmp_path / "environment"))
    result = CliRunner().invoke(
        app, ["setup", "--home", str(tmp_path / "explicit"), "--output", "json"]
    )

    assert result.exit_code == 0, result.output
    payload = json.loads(result.stdout)
    assert payload["home"] == str(tmp_path / "explicit")
    assert payload["created"] is True
    assert Path(payload["starter_pack"]).is_dir()
    assert not (tmp_path / "environment").exists()


def test_help_and_version_do_not_initialize_user_data(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    home = tmp_path / "unused"
    monkeypatch.setenv("MODSIM_PATH", str(home))
    runner = CliRunner()

    assert runner.invoke(app, ["--help"]).exit_code == 0
    assert runner.invoke(app, ["--version"]).exit_code == 0
    assert runner.invoke(app, ["setup", "--help"]).exit_code == 0
    assert not home.exists()


def test_cli_reports_unwritable_home_without_traceback(tmp_path: Path) -> None:
    blocker = tmp_path / "file"
    blocker.write_text("Keep this file.", encoding="utf-8")

    result = CliRunner().invoke(app, ["setup", "--home", str(blocker)])

    assert result.exit_code == 1
    assert "Cannot initialize ModSim user data" in result.output
    assert blocker.read_text() == "Keep this file."


def _copy_library_pack(
    source: Path, destination: Path, *, pack_id: str, version: str = "0.1.0"
) -> Path:
    shutil.copytree(source, destination)
    manifest = destination / "robot_pack.yaml"
    contents = manifest.read_text(encoding="utf-8")
    contents = contents.replace("\nid: generic_cube\n", f"\nid: {pack_id}\n")
    contents = contents.replace("\nversion: 0.1.0\n", f"\nversion: {version}\n")
    manifest.write_text(contents, encoding="utf-8")
    return destination


def test_listing_uses_the_home_selected_by_modsim_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    setup = initialize_user_home(tmp_path / "standard")
    legacy = tmp_path / "legacy"
    monkeypatch.setenv("MODSIM_PATH", str(setup.home))
    monkeypatch.setenv("MODSIM_HOME", str(legacy))

    packs = list_robot_packs()

    assert len(packs) == 1
    assert packs[0].as_dict() == {
        "id": "generic_cube",
        "version": "0.1.0",
        "path": str(setup.starter_pack),
    }
    assert resolve_robot_pack("generic_cube") == setup.starter_pack
    assert not legacy.exists()


@pytest.mark.parametrize("absolute", [False, True])
def test_setup_and_listing_use_the_configured_library_without_rewriting_preferences(
    tmp_path: Path, absolute: bool, monkeypatch: pytest.MonkeyPatch
) -> None:
    home = tmp_path / "home"
    home.mkdir()
    library = tmp_path / "external-library" if absolute else home / "my-library"
    configured = library if absolute else library.relative_to(home)
    contents = (
        "# My library preferences\nschema_version = 1\ncustom_setting = 'keep me'\n\n"
        f"[paths]\nrobot_packs = {json.dumps(str(configured))}\n\n"
        "[defaults]\nrobot_pack = 'old-selected-robot'\n"
    )
    config = home / "config.toml"
    config.write_text(contents, encoding="utf-8")
    monkeypatch.chdir(tmp_path)

    setup = initialize_user_home(home)
    repeated = initialize_user_home(home)
    packs = list_robot_packs(home=home)

    assert setup.starter_pack == library / "modlab/generic_cube/0.1.0"
    assert setup.created and not repeated.created
    assert packs[0].path == setup.starter_pack
    assert resolve_robot_pack("generic_cube", home=home) == setup.starter_pack
    assert not (home / "robot_packs").exists()
    assert config.read_text(encoding="utf-8") == contents


@pytest.mark.parametrize(
    "legacy", ["", "[defaults]\nrobot_pack = 'missing-old-default'\n", "defaults = 42\n"]
)
def test_legacy_configuration_uses_standard_library_and_selects_no_robot(
    tmp_path: Path, legacy: str
) -> None:
    config = tmp_path / "config.toml"
    contents = f"# Keep this legacy configuration.\nschema_version = 1\n{legacy}"
    config.write_text(contents, encoding="utf-8")
    setup = initialize_user_home(tmp_path)

    assert setup.starter_pack == tmp_path / "robot_packs/modlab/generic_cube/0.1.0"
    assert resolve_robot_pack("generic_cube", home=tmp_path) == setup.starter_pack
    with pytest.raises(UserDataError, match="Select a Robot Pack by name"):
        resolve_robot_pack(home=tmp_path)
    assert config.read_text(encoding="utf-8") == contents


def test_explicit_pack_bypasses_invalid_environment_and_configuration(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("MODSIM_PATH", "")
    monkeypatch.setenv("MODSIM_HOME", "")
    config = tmp_path / "config.toml"
    contents = "this is not TOML"
    config.write_text(contents, encoding="utf-8")
    missing_pack = tmp_path / "not-installed"

    assert resolve_robot_pack(missing_pack, home=tmp_path) == missing_pack
    assert not missing_pack.exists()
    assert config.read_text(encoding="utf-8") == contents


@pytest.mark.parametrize(
    "selection", ["./cube", "../cube", "~/cube", "folder/cube", r"folder\cube", ".", ".."]
)
def test_explicit_path_strings_expand_without_setup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, selection: str
) -> None:
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("MODSIM_PATH", "")

    assert resolve_robot_pack(selection) == Path(selection).expanduser().absolute()
    assert list(tmp_path.iterdir()) == []


def test_bare_pack_name_uses_manifest_identity_independently_of_cwd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, example_pack_dir: Path
) -> None:
    setup = initialize_user_home(tmp_path / "home")
    library = setup.home / "robot_packs"
    installed = _copy_library_pack(
        example_pack_dir, library / "vendor/arbitrary-folder", pack_id="my_robot"
    )
    working = tmp_path / "cwd"
    working.mkdir()
    _copy_library_pack(example_pack_dir, working / "my_robot", pack_id="different_robot")
    monkeypatch.chdir(working)

    assert resolve_robot_pack("my_robot", home=setup.home) == installed
    assert resolve_robot_pack(Path("my_robot"), home=setup.home) == working / "my_robot"


def test_listing_is_sorted_and_preserves_all_installed_names(
    tmp_path: Path, example_pack_dir: Path
) -> None:
    setup = initialize_user_home(tmp_path)
    library = setup.home / "robot_packs"
    _copy_library_pack(example_pack_dir, library / "vendor/zeta", pack_id="zeta", version="2.0.0")
    alpha = _copy_library_pack(example_pack_dir, library / "vendor/alpha", pack_id="alpha")

    packs = list_robot_packs(home=setup.home)

    assert [(pack.id, pack.version) for pack in packs] == [
        ("alpha", "0.1.0"),
        ("generic_cube", "0.1.0"),
        ("zeta", "2.0.0"),
    ]
    assert resolve_robot_pack("alpha", home=setup.home) == alpha
    assert json.loads(json.dumps([pack.as_dict() for pack in packs]))[0]["path"] == str(alpha)


def test_multiple_versions_require_a_version_or_explicit_path(
    tmp_path: Path, example_pack_dir: Path
) -> None:
    setup = initialize_user_home(tmp_path)
    second = _copy_library_pack(
        example_pack_dir,
        tmp_path / "robot_packs/vendor/new-version",
        pack_id="generic_cube",
        version="0.2.0",
    )

    with pytest.raises(UserDataError, match="ambiguous") as captured:
        resolve_robot_pack("generic_cube", home=tmp_path)
    assert "generic_cube@0.1.0" in str(captured.value)
    assert "generic_cube@0.2.0" in str(captured.value)
    assert str(setup.starter_pack) in str(captured.value)
    assert str(second) in str(captured.value)
    assert resolve_robot_pack("generic_cube@0.1.0", home=tmp_path) == setup.starter_pack
    assert resolve_robot_pack("generic_cube@0.2.0", home=tmp_path) == second


def test_duplicate_identity_requires_an_explicit_location(
    tmp_path: Path, example_pack_dir: Path
) -> None:
    setup = initialize_user_home(tmp_path)
    duplicate = _copy_library_pack(
        example_pack_dir, tmp_path / "robot_packs/another-copy", pack_id="generic_cube"
    )

    with pytest.raises(UserDataError, match=r"ambiguous.*explicit path") as captured:
        resolve_robot_pack("generic_cube@0.1.0", home=tmp_path)
    assert str(setup.starter_pack) in str(captured.value)
    assert str(duplicate) in str(captured.value)
    assert resolve_robot_pack(duplicate, home=tmp_path) == duplicate


@pytest.mark.parametrize("selection", ["missing_robot", "generic_cube@9.0.0"])
def test_missing_name_or_version_has_an_actionable_error(tmp_path: Path, selection: str) -> None:
    initialize_user_home(tmp_path)

    with pytest.raises(UserDataError, match="No installed Robot Pack matches") as captured:
        resolve_robot_pack(selection, home=tmp_path)
    assert selection in str(captured.value)
    assert "pack list" in str(captured.value)


@pytest.mark.parametrize("selection", [None, "", "cube@", "@0.1.0", "cube@0.1.0@other"])
def test_missing_or_malformed_selection_creates_no_storage(
    tmp_path: Path, selection: str | None
) -> None:
    home = tmp_path / "missing-home"

    with pytest.raises(UserDataError, match="Select a Robot Pack"):
        resolve_robot_pack(selection, home=home)
    assert not home.exists()


def test_invalid_unrelated_manifests_do_not_hide_valid_packs(
    tmp_path: Path, example_pack_dir: Path
) -> None:
    setup = initialize_user_home(tmp_path)
    library = tmp_path / "robot_packs"
    for name, contents in (
        ("broken_yaml", "[not: valid: yaml"),
        ("broken_schema", "id: invalid\nversion: unknown\n"),
        ("unsafe_tag", "!!python/object/apply:os.system ['false']"),
    ):
        directory = library / name
        directory.mkdir()
        (directory / "robot_pack.yaml").write_text(contents, encoding="utf-8")
    _copy_library_pack(
        example_pack_dir, library / "broken_schema/nested_pack", pack_id="should_be_hidden"
    )

    assert [pack.id for pack in list_robot_packs(home=tmp_path)] == ["generic_cube"]
    assert resolve_robot_pack("generic_cube", home=tmp_path) == setup.starter_pack


@pytest.mark.parametrize("pack_id", ["assets", "meshes"])
def test_legal_asset_named_library_folders_are_discoverable(
    tmp_path: Path, example_pack_dir: Path, pack_id: str
) -> None:
    setup = initialize_user_home(tmp_path)
    installed = _copy_library_pack(
        example_pack_dir, tmp_path / "robot_packs/modlab" / pack_id / "0.1.0", pack_id=pack_id
    )
    _copy_library_pack(
        example_pack_dir, setup.starter_pack / "assets/nested_pack", pack_id="not_a_library_pack"
    )
    _copy_library_pack(
        example_pack_dir, tmp_path / "robot_packs/.staging/hidden_pack", pack_id="not_published"
    )

    assert {pack.id for pack in list_robot_packs(home=tmp_path)} == {"generic_cube", pack_id}
    assert resolve_robot_pack(pack_id, home=tmp_path) == installed


@pytest.mark.parametrize("symbolic_entry", ["root", "manifest", "namespace"])
def test_library_discovery_skips_symlinked_entries(
    tmp_path: Path, example_pack_dir: Path, symbolic_entry: str
) -> None:
    setup = initialize_user_home(tmp_path / "home")
    library = setup.home / "robot_packs"
    outside = _copy_library_pack(
        example_pack_dir, tmp_path / "outside/robot", pack_id="linked_robot"
    )
    try:
        if symbolic_entry == "root":
            (library / "linked").symlink_to(outside, target_is_directory=True)
        elif symbolic_entry == "namespace":
            (library / "linked").symlink_to(outside.parent, target_is_directory=True)
        else:
            linked = library / "linked"
            linked.mkdir()
            (linked / "robot_pack.yaml").symlink_to(outside / "robot_pack.yaml")
    except OSError:
        pytest.skip("Symbolic links are unavailable on this platform")

    assert [pack.id for pack in list_robot_packs(home=setup.home)] == ["generic_cube"]
    with pytest.raises(UserDataError, match="No installed Robot Pack matches"):
        resolve_robot_pack("linked_robot", home=setup.home)


@pytest.mark.parametrize("symbolic_entry", ["root", "manifest"])
def test_explicit_pack_preserves_symlinks_for_loader_rejection(
    tmp_path: Path, example_pack_dir: Path, symbolic_entry: str
) -> None:
    pack = tmp_path / "cube"
    shutil.copytree(example_pack_dir, pack)
    try:
        if symbolic_entry == "root":
            selected = tmp_path / "linked-cube"
            selected.symlink_to(pack, target_is_directory=True)
            expected_code = "pack.root_symlink_not_allowed"
        else:
            selected = pack / "robot_pack.yaml"
            target = pack / "original-manifest.yaml"
            selected.rename(target)
            selected.symlink_to(target)
            expected_code = "document.symlink_not_allowed"
    except OSError:
        pytest.skip("Symbolic links are unavailable on this platform")

    resolved = resolve_robot_pack(selected)

    assert resolved == selected.absolute()
    assert resolved.is_symlink()
    with pytest.raises(RobotPackLoadError) as captured:
        RobotPackLoader().load(resolved)
    assert captured.value.issue.code == expected_code


@pytest.mark.parametrize("manifest", [False, True])
def test_explicit_normal_absolute_pack_path_is_unchanged(
    example_pack_dir: Path, manifest: bool
) -> None:
    selected = example_pack_dir / "robot_pack.yaml" if manifest else example_pack_dir

    assert resolve_robot_pack(selected) == selected
    assert resolve_robot_pack(str(selected)) == selected
    assert RobotPackLoader().load(resolve_robot_pack(selected)).pack.id == "generic_cube"


def test_missing_setup_is_actionable_and_does_not_create_storage(tmp_path: Path) -> None:
    home = tmp_path / "missing-home"

    with pytest.raises(UserDataError, match=r"not set up.*config.toml.*missing.*setup"):
        list_robot_packs(home=home)
    assert not home.exists()


@pytest.mark.parametrize(
    ("contents", "message"),
    [
        ("not valid TOML", "Cannot read ModSim configuration"),
        ("[paths]\nrobot_packs = 'library'", "schema_version = 1"),
        ("schema_version = 2", "schema_version = 1"),
        ("schema_version = true", "schema_version = 1"),
        ("schema_version = 1\npaths = 'library'", "TOML table"),
        ("schema_version = 1\n[paths]\nrobot_packs = 42", "non-empty path string"),
        ("schema_version = 1\n[paths]\nrobot_packs = ' '", "non-empty path string"),
    ],
)
def test_invalid_configuration_errors_preserve_all_files(
    tmp_path: Path, contents: str, message: str
) -> None:
    config = tmp_path / "config.toml"
    config.write_text(contents, encoding="utf-8")

    with pytest.raises(UserDataError, match=message):
        list_robot_packs(home=tmp_path)
    assert list(tmp_path.iterdir()) == [config]
    assert config.read_text(encoding="utf-8") == contents


def test_listing_an_empty_or_missing_library_never_creates_it(tmp_path: Path) -> None:
    (tmp_path / "config.toml").write_text("schema_version = 1\n", encoding="utf-8")
    library = tmp_path / "robot_packs"

    with pytest.raises(UserDataError, match=r"library.*does not exist.*setup"):
        list_robot_packs(home=tmp_path)
    assert not library.exists()
    library.mkdir()
    assert list_robot_packs(home=tmp_path) == ()
    assert list(library.iterdir()) == []
