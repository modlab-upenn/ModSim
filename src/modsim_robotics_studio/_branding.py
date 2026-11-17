"""Package-local production branding for the optional Studio applications."""

from __future__ import annotations

import json
import logging
import math
import sys
from functools import lru_cache
from pathlib import Path
from typing import cast

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QGuiApplication, QIcon, QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer

from modsim_robotics_studio._desktop import APPLICATION_ID, register_desktop_entry

_ASSET_ROOT = Path(__file__).resolve().parent / "assets" / "branding"
_THEME_DIRECTORIES = {
    "graphite": "dark",
    "dark": "dark",
    "light": "light",
    "midnight": "midnight",
}


def configure_application_identity(application: QGuiApplication, display_name: str) -> None:
    """Match the window's identity to its desktop entry before creating windows.

    Args:
        application: The active Qt GUI application.
        display_name: Human-readable name of the current Studio workspace.
    """
    application.setApplicationName(
        APPLICATION_ID if sys.platform.startswith("linux") else display_name
    )
    application.setApplicationDisplayName(display_name)
    application.setOrganizationName("ModSim")
    application.setDesktopFileName(APPLICATION_ID)
    try:
        register_desktop_entry(asset_path("midnight", "icons/png/app/modsim-app-1024.png"))
    except (OSError, ValueError):
        logging.getLogger(__name__).warning(
            "Could not register the ModSim desktop icon; continuing with the window icon",
            exc_info=True,
        )


def _theme_directory(theme_id: str) -> str:
    try:
        return _THEME_DIRECTORIES[theme_id]
    except KeyError as error:
        raise ValueError(f"Unknown Studio branding theme: {theme_id!r}") from error


def asset_path(theme_id: str, relative: str) -> Path:
    """Locate an existing production asset inside the selected packaged theme.

    Args:
        theme_id: Studio theme ID. The legacy ``graphite`` ID uses the ``dark`` pack.
        relative: File path relative to that pack, such as
            ``logos/svg/modsim-horizontal.svg``.

    Returns:
        The absolute path of the packaged asset.

    Raises:
        ValueError: The theme is unknown or the path leaves its pack directory.
        FileNotFoundError: The selected asset is absent or is not a regular file.
    """
    root = (_ASSET_ROOT / _theme_directory(theme_id)).resolve()
    requested = Path(relative)
    if requested.is_absolute():
        raise ValueError("Branding asset paths must be relative to their theme pack")
    result = (root / requested).resolve()
    if not result.is_relative_to(root):
        raise ValueError("Branding asset paths must remain inside their theme pack")
    if not result.is_file():
        raise FileNotFoundError(f"Packaged Studio branding asset not found: {result}")
    return result


@lru_cache(maxsize=3)
def _palette_items(theme_directory: str) -> tuple[tuple[str, str], ...]:
    data: object = json.loads(asset_path(theme_directory, "theme/colors.json").read_text())
    if not isinstance(data, dict):
        raise ValueError("A Studio branding palette must be a JSON object")
    values = cast(dict[str, object], data)
    result: list[tuple[str, str]] = []
    for name, value in values.items():
        if not isinstance(value, str):
            raise ValueError(f"Studio branding palette value {name!r} must be a string")
        result.append((name, value))
    return tuple(result)


def theme_palette(theme_id: str) -> dict[str, str]:
    """Return the selected production palette without sharing mutable cached data.

    Args:
        theme_id: Studio theme ID, with legacy ``graphite`` mapped to ``dark``.

    Returns:
        A fresh mapping of the authoritative ``theme/colors.json`` values,
        including its theme/name metadata and production color tokens.
    """
    return dict(_palette_items(_theme_directory(theme_id)))


def vector_pixmap(
    theme_id: str,
    relative: str,
    size: QSize,
    *,
    device_pixel_ratio: float | None = None,
) -> QPixmap:
    """Rasterize a packaged SVG proportionally onto a transparent high-DPI canvas.

    Call after creating a GUI application. Rasterization uses at least two
    physical pixels per logical pixel, or the supplied screen's higher ratio.
    The complete SVG is centered within the requested logical canvas.

    Args:
        theme_id: Studio theme ID, with legacy ``graphite`` mapped to ``dark``.
        relative: SVG file path relative to the selected pack.
        size: Positive logical width and height of the resulting pixmap.
        device_pixel_ratio: The destination widget's current scale factor.
            Defaults to the primary screen's ratio when omitted.

    Returns:
        A transparent pixmap with its device pixel ratio set for logical sizing.

    Raises:
        RuntimeError: A GUI application has not been created.
        ValueError: The size, theme/path, or SVG view box is invalid.
        FileNotFoundError: The selected asset is absent.
    """
    if not isinstance(QGuiApplication.instance(), QGuiApplication):
        raise RuntimeError("Create a Qt GUI application before rendering Studio branding")
    if size.width() <= 0 or size.height() <= 0:
        raise ValueError("Branding pixmap dimensions must be positive")
    renderer = QSvgRenderer(str(asset_path(theme_id, relative)))
    view = renderer.viewBoxF()
    if not renderer.isValid() or view.isEmpty():
        raise ValueError(f"Invalid Studio branding SVG: {relative}")

    if device_pixel_ratio is None:
        device_pixel_ratio = QGuiApplication.primaryScreen().devicePixelRatio()
    if not math.isfinite(device_pixel_ratio) or device_pixel_ratio <= 0:
        raise ValueError("Branding device pixel ratio must be finite and positive")
    ratio = max(2.0, device_pixel_ratio)
    width = math.ceil(size.width() * ratio)
    height = math.ceil(size.height() * ratio)
    pixmap = QPixmap(width, height)
    pixmap.fill(Qt.GlobalColor.transparent)
    scale = min(width / view.width(), height / view.height())
    target = QRectF(
        (width - view.width() * scale) / 2,
        (height - view.height() * scale) / 2,
        view.width() * scale,
        view.height() * scale,
    )
    painter = QPainter(pixmap)
    try:
        renderer.render(painter, target)
    finally:
        painter.end()
    pixmap.setDevicePixelRatio(ratio)
    return pixmap


def application_icon(theme_id: str) -> QIcon:
    """Build an application icon from its scalable mark and supplied raster sizes.

    Args:
        theme_id: Studio theme ID, with legacy ``graphite`` mapped to ``dark``.

    Returns:
        A theme-matched app icon with scalable SVG and 256/512/1024 px PNG sources.
    """
    icon = QIcon(str(asset_path(theme_id, "icons/svg/modsim-app.svg")))
    for side in (256, 512, 1024):
        icon.addFile(
            str(asset_path(theme_id, f"icons/png/app/modsim-app-{side}.png")),
            QSize(side, side),
        )
    return icon
