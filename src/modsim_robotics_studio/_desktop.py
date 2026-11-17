"""Native process identity and per-user Linux desktop metadata for Studio."""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from tempfile import NamedTemporaryFile

APPLICATION_ID = "io.github.modlab_upenn.ModSim"
_LOGGER = logging.getLogger("modsim_robotics_studio.desktop")


def _set_windows_process_identity(application_id: str) -> int:
    """Bind the Shell32 API lazily so other platforms never load Windows DLLs."""
    if sys.platform != "win32":
        return 0
    import ctypes

    shell = ctypes.WinDLL("shell32", use_last_error=True)
    set_identity = shell.SetCurrentProcessExplicitAppUserModelID
    set_identity.argtypes = [ctypes.c_wchar_p]
    set_identity.restype = ctypes.c_long
    return int(set_identity(application_id))


def configure_native_process_identity() -> None:
    """Associate Windows processes with ModSim before creating any native UI.

    Call before constructing QApplication or showing the first splash/window.
    Both desktop applications use the same explicit AppUserModelID for taskbar
    grouping. Native errors and failing HRESULTs are logged and do not prevent
    startup. Linux desktop registration remains a separate GUI setup step;
    this function performs no work on Linux, macOS, or other platforms.
    """
    if sys.platform != "win32":
        return
    try:
        result = _set_windows_process_identity(APPLICATION_ID)
        if result != 0:
            _LOGGER.warning(
                "Could not set Windows taskbar identity (HRESULT 0x%08X)",
                result & 0xFFFFFFFF,
            )
    except Exception:
        # Optional shell integration must not prevent the Qt application from
        # opening when Shell32 is unavailable or the native call fails.
        _LOGGER.warning("Could not set Windows taskbar identity", exc_info=True)


def _desktop_string(value: str) -> str:
    return (
        value.replace("\\", "\\\\").replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t")
    )


def _exec_executable(value: str) -> str:
    # Desktop Entry quoting is distinct from shell quoting. Field codes are
    # expanded by the launcher, including inside a quoted executable path.
    if any(character in value for character in ("\0", "\n", "\r", "=")):
        raise ValueError("The Python executable path cannot be represented in a desktop entry")
    value = value.replace("%", "%%")
    for character in ("\\", '"', "`", "$"):
        value = value.replace(character, "\\" + character)
    return _desktop_string('"' + value + '"')


def _write_if_changed(path: Path, content: bytes) -> None:
    try:
        if path.read_bytes() == content:
            return
    except FileNotFoundError:
        pass
    path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(content)
            stream.close()
            temporary.chmod(0o644)
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


def register_desktop_entry(icon: Path) -> Path | None:
    """Register the product identity and icon before a Linux GUI window appears.

    Both Studio and the Inspector use this identity. Activating the product's
    launcher opens the blank Builder in the last GUI launch's environment.
    This function performs no work on other platforms and never changes shell
    profiles or RobotPacks. Callers should log filesystem failures and continue
    opening the application.

    Args:
        icon: Packaged production PNG to copy to the user's desktop data folder.

    Returns:
        The desktop entry path on Linux, or None on another platform.

    Raises:
        OSError: An asset or per-user desktop file cannot be read or written.
        ValueError: The executable path cannot be encoded as a desktop command.
    """
    if not sys.platform.startswith("linux"):
        return None
    configured = os.environ.get("XDG_DATA_HOME", "")
    data_home = Path(configured) if configured else Path.home() / ".local" / "share"
    if not data_home.is_absolute():
        data_home = Path.home() / ".local" / "share"
    icon_path = data_home / "modsim-robotics" / "icons" / "modsim.png"
    desktop = data_home / "applications" / f"{APPLICATION_ID}.desktop"
    # Keep the virtualenv path: resolving the Python symlink loses its venv.
    executable = _exec_executable(str(Path(sys.executable).absolute()))
    text = (
        "[Desktop Entry]\n"
        "Type=Application\n"
        "Name=ModSim Studio\n"
        "Comment=Build Robot Packs and inspect modular robot simulations\n"
        f"Exec={executable} -m modsim_robotics_studio\n"
        f"Icon={_desktop_string(str(icon_path))}\n"
        f"StartupWMClass={APPLICATION_ID}\n"
        "Terminal=false\n"
        "Categories=Science;\n"
        "Keywords=ModSim;robotics;simulation;URDF;\n"
    )
    _write_if_changed(icon_path, icon.read_bytes())
    _write_if_changed(desktop, text.encode("utf-8"))
    return desktop
