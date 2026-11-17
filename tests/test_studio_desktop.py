"""Native taskbar identity and Linux metadata are independent of the launcher."""

from __future__ import annotations

import configparser
import ctypes
import logging
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from modsim_robotics_studio import _desktop
from modsim_robotics_studio._desktop import (
    APPLICATION_ID,
    configure_native_process_identity,
    register_desktop_entry,
)


class _WindowsIdentityCall:
    def __init__(self, result: int = 0) -> None:
        self.result = result
        self.calls: list[str] = []
        self.argtypes: list[object] | None = None
        self.restype: object = None

    def __call__(self, application_id: str) -> int:
        self.calls.append(application_id)
        return self.result


@pytest.fixture
def windows_identity(monkeypatch: pytest.MonkeyPatch) -> _WindowsIdentityCall:
    function = _WindowsIdentityCall()

    def load_library(name: str, *, use_last_error: bool) -> SimpleNamespace:
        assert name == "shell32"
        assert use_last_error
        return SimpleNamespace(SetCurrentProcessExplicitAppUserModelID=function)

    monkeypatch.setattr(_desktop.sys, "platform", "win32")
    monkeypatch.setattr(ctypes, "WinDLL", load_library, raising=False)
    return function


def test_windows_identity_uses_shared_product_id_and_wide_native_signature(
    windows_identity: _WindowsIdentityCall, caplog: pytest.LogCaptureFixture
) -> None:
    configure_native_process_identity()
    assert windows_identity.calls == [APPLICATION_ID]
    assert windows_identity.argtypes == [ctypes.c_wchar_p]
    assert windows_identity.restype is ctypes.c_long
    assert not caplog.records


@pytest.mark.parametrize("result", [0x80004005, -2147467259])
def test_failing_windows_hresult_is_logged_without_aborting_startup(
    windows_identity: _WindowsIdentityCall, caplog: pytest.LogCaptureFixture, result: int
) -> None:
    windows_identity.result = result
    with caplog.at_level(logging.WARNING, logger="modsim_robotics_studio.desktop"):
        configure_native_process_identity()
    assert windows_identity.calls == [APPLICATION_ID]
    assert "HRESULT 0x80004005" in caplog.text


@pytest.mark.parametrize("error", [OSError("Shell32 unavailable"), AttributeError("Missing API")])
def test_unavailable_windows_api_is_nonfatal(
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
    error: Exception,
) -> None:
    def unavailable(*_args: object, **_kwargs: object) -> None:
        raise error

    monkeypatch.setattr(_desktop.sys, "platform", "win32")
    monkeypatch.setattr(ctypes, "WinDLL", unavailable, raising=False)
    with caplog.at_level(logging.WARNING, logger="modsim_robotics_studio.desktop"):
        configure_native_process_identity()
    assert "Could not set Windows taskbar identity" in caplog.text


@pytest.mark.parametrize("platform", ["linux", "darwin"])
def test_non_windows_startup_never_loads_windows_libraries(
    monkeypatch: pytest.MonkeyPatch, platform: str
) -> None:
    def forbidden(*_args: object, **_kwargs: object) -> None:
        pytest.fail("Non-Windows startup attempted to load a Windows DLL")

    monkeypatch.setattr(_desktop.sys, "platform", platform)
    monkeypatch.setattr(ctypes, "WinDLL", forbidden, raising=False)
    configure_native_process_identity()


@pytest.fixture
def desktop_data(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    root = tmp_path / "desktop-data"
    monkeypatch.setenv("XDG_DATA_HOME", str(root))
    monkeypatch.setattr(_desktop.sys, "platform", "linux")
    return root


@pytest.fixture
def icon_file(tmp_path: Path) -> Path:
    icon = tmp_path / "packaged-icon.png"
    icon.write_bytes(b"test icon content")
    return icon


def _entry(path: Path) -> configparser.SectionProxy:
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(path, encoding="utf-8")
    return parser["Desktop Entry"]


def test_registration_copies_icon_and_preserves_venv_executable(
    desktop_data: Path, icon_file: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    executable = tmp_path / "venv with spaces" / "bin" / "python"
    executable.parent.mkdir(parents=True)
    executable.symlink_to(sys.executable)
    monkeypatch.setattr(_desktop.sys, "executable", str(executable))

    path = register_desktop_entry(icon_file)
    assert path == desktop_data / "applications" / f"{APPLICATION_ID}.desktop"
    assert path is not None
    entry = _entry(path)
    assert entry["StartupWMClass"] == APPLICATION_ID
    assert entry["Name"] == "ModSim Studio"
    assert entry["Exec"] == f'"{executable}" -m modsim_robotics_studio'
    installed_icon = Path(entry["Icon"])
    assert installed_icon.is_relative_to(desktop_data)
    assert installed_icon.read_bytes() == icon_file.read_bytes()
    assert not entry.getboolean("Terminal")
    assert "DBusActivatable" not in entry

    # A normal launch must not invalidate shell caches by rewriting metadata.
    timestamps = (path.stat().st_mtime_ns, installed_icon.stat().st_mtime_ns)
    assert register_desktop_entry(icon_file) == path
    assert (path.stat().st_mtime_ns, installed_icon.stat().st_mtime_ns) == timestamps


def test_new_environment_updates_launcher_without_replacing_unchanged_icon(
    desktop_data: Path, icon_file: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = register_desktop_entry(icon_file)
    assert path is not None
    installed_icon = Path(_entry(path)["Icon"])
    timestamp = installed_icon.stat().st_mtime_ns
    other_python = tmp_path / "another-env" / "bin" / "python"
    monkeypatch.setattr(_desktop.sys, "executable", str(other_python))

    register_desktop_entry(icon_file)
    assert str(other_python) in _entry(path)["Exec"]
    assert installed_icon.stat().st_mtime_ns == timestamp


@pytest.mark.parametrize("configured", [None, "relative/data"])
def test_unset_or_relative_data_home_uses_user_data_directory(
    desktop_data: Path,
    icon_file: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    configured: str | None,
) -> None:
    def user_home() -> Path:
        return tmp_path / "home"

    monkeypatch.setattr(Path, "home", staticmethod(user_home))
    if configured is None:
        monkeypatch.delenv("XDG_DATA_HOME")
    else:
        monkeypatch.setenv("XDG_DATA_HOME", configured)
    path = register_desktop_entry(icon_file)
    assert path == user_home() / ".local/share/applications" / f"{APPLICATION_ID}.desktop"
    assert not desktop_data.exists()


@pytest.mark.parametrize("platform", ["darwin", "win32"])
def test_other_platforms_do_not_read_or_register_linux_assets(
    desktop_data: Path, monkeypatch: pytest.MonkeyPatch, platform: str
) -> None:
    monkeypatch.setattr(_desktop.sys, "platform", platform)
    assert register_desktop_entry(desktop_data / "missing.png") is None
    assert not desktop_data.exists()


def test_registered_desktop_file_passes_system_validation(
    desktop_data: Path, icon_file: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    validator = shutil.which("desktop-file-validate")
    if validator is None:
        pytest.skip("desktop-file-validate is not installed")
    executable = tmp_path / 'venv with spaces % $ ` " \\' / "python"
    monkeypatch.setattr(_desktop.sys, "executable", str(executable))
    path = register_desktop_entry(icon_file)
    assert path is not None
    result = subprocess.run([validator, str(path)], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr


def test_registration_surfaces_unwritable_desktop_location(
    desktop_data: Path, icon_file: Path
) -> None:
    desktop_data.mkdir()
    (desktop_data / "applications").write_text("Not a directory", encoding="utf-8")
    with pytest.raises(OSError):
        register_desktop_entry(icon_file)
