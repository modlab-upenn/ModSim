"""Studio launchers resolve a selected pack before starting the optional UI."""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from types import ModuleType

import pytest

from modsim_robotics.user_data import initialize_user_home
from modsim_robotics_studio import launcher


@pytest.fixture(autouse=True)
def isolated_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MODSIM_PATH", str(tmp_path / "modsim-home"))
    monkeypatch.delenv("MODSIM_HOME", raising=False)


def _launch(
    arguments: list[str], monkeypatch: pytest.MonkeyPatch
) -> tuple[int | str | None, list[Path | None]]:
    received: list[Path | None] = []
    application = ModuleType("modsim_robotics_studio.app")

    def main(initial_path: Path | None) -> int:
        received.append(initial_path)
        return 0

    application.__dict__["main"] = main
    monkeypatch.setitem(sys.modules, "modsim_robotics_studio.app", application)
    monkeypatch.setattr(sys, "argv", ["modsim-robotics-studio", *arguments])
    with pytest.raises(SystemExit) as result:
        launcher.cli()
    return result.value.code, received


@pytest.mark.parametrize("selector", ["generic_cube", "generic_cube@0.1.0"])
def test_standalone_studio_finds_the_selected_pack(
    monkeypatch: pytest.MonkeyPatch, selector: str
) -> None:
    setup = initialize_user_home()

    code, received = _launch([selector], monkeypatch)

    assert code == 0
    assert received == [setup.starter_pack]


@pytest.mark.parametrize("named", [False, True])
def test_standalone_explicit_pack_bypasses_missing_home(
    copied_pack: Path, monkeypatch: pytest.MonkeyPatch, named: bool
) -> None:
    arguments = ["--pack", str(copied_pack)] if named else [str(copied_pack)]

    code, received = _launch(arguments, monkeypatch)

    assert code == 0
    assert received == [copied_pack.resolve()]


def test_standalone_home_override_selects_a_pack_from_that_library(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    setup = initialize_user_home(tmp_path / "another-home")

    code, received = _launch(["generic_cube", "--home", str(setup.home)], monkeypatch)

    assert code == 0
    assert received == [setup.starter_pack]


def test_standalone_conflicting_paths_are_rejected(
    copied_pack: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    code, received = _launch([str(copied_pack), "--pack", str(copied_pack)], monkeypatch)

    assert code == 2
    assert received == []
    assert "not both" in capsys.readouterr().err


def test_standalone_missing_setup_is_actionable_and_does_not_create_data(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    code, received = _launch(["generic_cube"], monkeypatch)

    assert code == 2
    assert received == []
    assert "modsim-robotics setup" in capsys.readouterr().err
    assert not (tmp_path / "modsim-home").exists()


@pytest.mark.parametrize("option", ["--help", "--version"])
def test_standalone_information_does_not_require_or_create_home(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, option: str
) -> None:
    code, received = _launch([option], monkeypatch)

    assert code == 0
    assert received == []
    assert not (tmp_path / "modsim-home").exists()


def test_studio_app_passes_the_selected_pack_to_window(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pytest.importorskip("PySide6")
    pytest.importorskip("pyvistaqt")
    from modsim_robotics_studio import app as studio_app

    setup = initialize_user_home()
    opened: list[Path] = []

    class Application:
        def __init__(self, _arguments: list[str]) -> None:
            pass

        @staticmethod
        def instance() -> None:
            return None

        def setApplicationName(self, _name: str) -> None:
            pass

        def setOrganizationName(self, _name: str) -> None:
            pass

        def exec(self) -> int:
            return 0

    class Window:
        def __init__(
            self,
            *,
            initial_path: Path,
            session_logger: logging.Logger,
            session_log_path: Path,
        ) -> None:
            del session_logger
            assert session_log_path.is_relative_to(setup.home)
            opened.append(initial_path)

        def show(self) -> None:
            pass

    monkeypatch.setattr(studio_app, "QApplication", Application)
    monkeypatch.setattr(studio_app, "MainWindow", Window)

    assert studio_app.main("generic_cube") == 0
    assert opened == [setup.starter_pack]


def test_studio_app_reports_missing_setup_before_creating_a_window(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    pytest.importorskip("PySide6")
    pytest.importorskip("pyvistaqt")
    from modsim_robotics_studio import app as studio_app

    assert studio_app.main("generic_cube") == 2
    assert "modsim-robotics setup" in capsys.readouterr().err
    assert not (tmp_path / "modsim-home").exists()


def test_standalone_requires_selection_even_with_one_installed_pack(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    initialize_user_home()

    code, received = _launch([], monkeypatch)

    assert code == 2
    assert received == []
    assert "select" in capsys.readouterr().err.lower()


def test_standalone_pack_option_accepts_an_installed_name(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    setup = initialize_user_home()

    code, received = _launch(["--pack", "generic_cube"], monkeypatch)

    assert code == 0
    assert received == [setup.starter_pack]


def test_standalone_distinguishes_a_pack_id_from_a_local_folder(
    copied_pack: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    setup = initialize_user_home()
    monkeypatch.chdir(copied_pack.parent)

    code, selected_by_name = _launch(["generic_cube"], monkeypatch)
    path_code, selected_by_path = _launch(["./generic_cube"], monkeypatch)

    assert code == path_code == 0
    assert selected_by_name == [setup.starter_pack]
    assert selected_by_path == [copied_pack.absolute()]
