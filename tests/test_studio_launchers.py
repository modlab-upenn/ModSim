"""Studio opens blank or resolves an explicit selection before starting its UI."""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from modsim_robotics.user_data import initialize_user_home
from modsim_robotics_studio import launcher


@pytest.fixture(autouse=True)
def isolated_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MODSIM_PATH", str(tmp_path / "modsim-home"))
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "desktop-data"))
    monkeypatch.delenv("MODSIM_HOME", raising=False)


def _launch(
    arguments: list[str],
    monkeypatch: pytest.MonkeyPatch,
    *,
    homes: list[Path | None] | None = None,
) -> tuple[int | str | None, list[Path | None]]:
    received: list[Path | None] = []
    application = ModuleType("modsim_robotics_studio.app")

    def main(initial_path: Path | None, *, home: Path | None = None) -> int:
        received.append(initial_path)
        if homes is not None:
            homes.append(home)
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
    homes: list[Path | None] = []

    code, received = _launch(["generic_cube", "--home", str(setup.home)], monkeypatch, homes=homes)

    assert code == 0
    assert received == [setup.starter_pack]
    assert homes == [setup.home]


@pytest.mark.parametrize("named", [False, True])
def test_standalone_explicit_pack_keeps_home_for_display_preferences(
    tmp_path: Path, copied_pack: Path, monkeypatch: pytest.MonkeyPatch, named: bool
) -> None:
    selected_home = tmp_path / "display-preferences"
    arguments = ["--pack", str(copied_pack)] if named else [str(copied_pack)]
    homes: list[Path | None] = []

    code, received = _launch([*arguments, "--home", str(selected_home)], monkeypatch, homes=homes)

    assert code == 0
    assert received == [copied_pack.resolve()]
    assert homes == [selected_home]
    assert not selected_home.exists()
    assert not (tmp_path / "modsim-home").exists()


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


@pytest.mark.parametrize("selector", [None, "generic_cube", "explicit"])
@pytest.mark.parametrize("override_home", [False, True])
def test_studio_app_passes_the_optional_pack_to_window(
    tmp_path: Path,
    copied_pack: Path,
    monkeypatch: pytest.MonkeyPatch,
    selector: str | None,
    override_home: bool,
) -> None:
    pytest.importorskip("PySide6")
    pytest.importorskip("pyvistaqt")
    from PySide6.QtGui import QIcon

    from modsim_robotics_studio import app as studio_app
    from modsim_robotics_studio._desktop import APPLICATION_ID

    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("MODSIM_STUDIO_LOG", raising=False)
    selected_home = tmp_path / "preferences-home" if override_home else None
    expected_path = (
        initialize_user_home(selected_home).starter_pack
        if selector == "generic_cube"
        else copied_pack
        if selector == "explicit"
        else None
    )
    initial_path = copied_pack if selector == "explicit" else selector
    expected_log_path = (
        expected_path.parent / ".modsim" / "logs" / expected_path.name / "studio.log"
        if expected_path is not None
        else tmp_path / ".modsim" / "logs" / "studio.log"
    )
    opened: list[Path | None] = []
    homes: list[str | Path | None] = []
    stages: list[str] = []

    if selector is None:

        def unexpected_resolution(
            pack_path: str | Path | None = None, *, home: Path | None = None
        ) -> Path:
            raise AssertionError("Blank Studio must not resolve a pack")

        monkeypatch.setattr(studio_app, "resolve_robot_pack", unexpected_resolution)

    class Application:
        def __init__(self, _arguments: list[str]) -> None:
            pass

        @staticmethod
        def instance() -> None:
            return None

        def setApplicationName(self, _name: str) -> None:
            pass

        def setApplicationDisplayName(self, name: str) -> None:
            assert name == "ModSim Studio"

        def setDesktopFileName(self, name: str) -> None:
            assert name == APPLICATION_ID
            stages.append("desktop identity set")

        def setOrganizationName(self, _name: str) -> None:
            pass

        def setWindowIcon(self, _icon: QIcon) -> None:
            pass

        def exec(self) -> int:
            return 0

    class Splash:
        def __init__(self, _workspace: str) -> None:
            pass

        def show(self) -> None:
            stages.append("splash shown")

        def set_stage(self, _message: str) -> None:
            stages.append("stage painted")

        def play_intro(self) -> bool:
            stages.append("intro played")
            return True

        def present(self, _window: object) -> bool:
            stages.append("splash finished")
            return True

        def close(self) -> None:
            stages.append("splash closed")

    class Window:
        def __init__(
            self,
            *,
            initial_path: Path | None,
            session_logger: logging.Logger,
            session_log_path: Path,
            home: str | Path | None = None,
        ) -> None:
            del session_logger
            assert session_log_path == expected_log_path
            opened.append(initial_path)
            homes.append(home)
            stages.append("window created")

        def show(self) -> None:
            pass

    monkeypatch.setattr(studio_app, "QApplication", Application)
    monkeypatch.setattr(studio_app, "_create_window", Window)
    monkeypatch.setattr(studio_app, "StartupSplash", Splash)

    def app_icon(_theme: str) -> QIcon:
        return QIcon()

    monkeypatch.setattr(studio_app, "application_icon", app_icon)
    monkeypatch.setattr(
        studio_app, "theme_manager", lambda: SimpleNamespace(theme=SimpleNamespace(id="midnight"))
    )

    assert studio_app.main(initial_path, home=selected_home) == 0
    assert opened == [expected_path]
    assert homes == [selected_home]
    assert stages == [
        "desktop identity set",
        "splash shown",
        "stage painted",
        "intro played",
        "stage painted",
        "window created",
        "splash finished",
    ]
    assert expected_log_path.is_file()
    if selector != "generic_cube":
        assert not (tmp_path / "modsim-home").exists()
        if selected_home is not None:
            assert not selected_home.exists()


def test_create_window_forwards_home_to_the_builder(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    pytest.importorskip("PySide6")
    from modsim_robotics_studio import app as studio_app

    selected_home = tmp_path / "selected-home"
    received: list[str | Path | None] = []
    module = ModuleType("modsim_robotics_studio.main_window")

    class Window:
        def __init__(
            self,
            *,
            initial_path: str | Path | None,
            session_logger: logging.Logger,
            session_log_path: Path,
            home: str | Path | None = None,
        ) -> None:
            del session_logger
            assert initial_path is None
            assert session_log_path == tmp_path / "studio.log"
            received.append(home)

    module.__dict__["MainWindow"] = Window
    monkeypatch.setitem(sys.modules, module.__name__, module)

    window = studio_app._create_window(  # pyright: ignore[reportPrivateUsage]
        initial_path=None,
        session_logger=logging.getLogger(__name__),
        session_log_path=tmp_path / "studio.log",
        home=selected_home,
    )

    assert isinstance(window, Window)
    assert received == [selected_home]
    assert not selected_home.exists()


def test_studio_app_reports_missing_setup_before_creating_a_window(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    pytest.importorskip("PySide6")
    pytest.importorskip("pyvistaqt")
    from modsim_robotics_studio import app as studio_app

    assert studio_app.main("generic_cube") == 2
    assert "modsim-robotics setup" in capsys.readouterr().err
    assert not (tmp_path / "modsim-home").exists()


@pytest.mark.parametrize("initialized", [False, True])
@pytest.mark.parametrize("override_home", [False, True])
def test_standalone_without_selection_opens_blank_without_reading_the_library(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    initialized: bool,
    override_home: bool,
) -> None:
    selected_home = tmp_path / ("other-home" if override_home else "modsim-home")
    configuration = None
    if initialized:
        initialize_user_home(selected_home)
        configuration = (selected_home / "config.toml").read_bytes()

    def unexpected_resolution(
        pack_path: str | Path | None = None, *, home: Path | None = None
    ) -> Path:
        raise AssertionError("Blank Studio must not resolve a pack or read its library")

    monkeypatch.setattr(launcher, "resolve_robot_pack", unexpected_resolution)
    arguments = ["--home", str(selected_home)] if override_home else []
    homes: list[Path | None] = []
    code, received = _launch(arguments, monkeypatch, homes=homes)

    assert code == 0
    assert received == [None]
    assert homes == [selected_home if override_home else None]
    assert selected_home.exists() == initialized
    if initialized:
        assert (selected_home / "config.toml").read_bytes() == configuration


def test_blank_standalone_explains_missing_dependencies(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setitem(sys.modules, "modsim_robotics_studio.app", None)
    monkeypatch.setattr(sys, "argv", ["modsim-robotics-studio"])

    with pytest.raises(SystemExit) as result:
        launcher.cli()

    assert result.value.code == 2
    assert "pip install modsim-robotics" in capsys.readouterr().err
    assert not (tmp_path / "modsim-home").exists()


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
