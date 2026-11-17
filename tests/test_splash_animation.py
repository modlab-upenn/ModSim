"""The startup logo animates real SVG layers and stops when its window hides."""

from __future__ import annotations

import os
import subprocess
import sys

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QEvent, QTimer
from PySide6.QtGui import QImage
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from modsim_robotics_studio._branding import vector_pixmap
from modsim_robotics_studio._splash_animation import AnimatedBrandLogo
from modsim_robotics_studio.appearance import theme_manager


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


def _pixels(image: QImage) -> bytes:
    return bytes(image.constBits())


def test_logo_frames_change_and_hiding_stops_periodic_work(application: QApplication) -> None:
    logo = AnimatedBrandLogo()
    logo.resize(360, 76)
    timer = logo.findChild(QTimer, "modsimLogoAnimation")
    assert timer is not None
    try:
        logo.start()
        assert not timer.isActive()
        logo.show()
        application.processEvents()
        assert timer.isActive()
        first = _pixels(logo.grab().toImage())
        QTest.qWait(250)
        second = _pixels(logo.grab().toImage())
        assert first != second
        QTest.qWait(350)
        assert second != _pixels(logo.grab().toImage())

        logo.hide()
        assert not timer.isActive()
        logo.show()
        application.processEvents()
        frozen = _pixels(logo.grab().toImage())
        QTest.qWait(100)
        assert not timer.isActive()
        assert frozen == _pixels(logo.grab().toImage())

        logo.start()
        assert timer.isActive()
        logo.stop()
        assert not timer.isActive()
    finally:
        logo.close()
        assert not timer.isActive()
        logo.deleteLater()
        application.processEvents()


@pytest.mark.parametrize("theme", ["light", "dark", "midnight"])
def test_complete_logo_preserves_approved_artwork_after_resize_and_theme_change(
    application: QApplication, theme: str
) -> None:
    manager = theme_manager()
    previous_theme = manager.theme.id
    logo = AnimatedBrandLogo()
    try:
        logo.show()
        manager.set_theme(theme, persist=False)
        for width in (360, 260):
            logo.resize(width, 76)
            QApplication.sendEvent(logo, QEvent(QEvent.Type.DevicePixelRatioChange))
            application.processEvents()
            actual = logo.grab().toImage()
            reference = (
                vector_pixmap(
                    theme,
                    "logos/svg/modsim-horizontal.svg",
                    logo.size(),
                    device_pixel_ratio=logo.devicePixelRatioF(),
                )
                .toImage()
                .scaled(actual.size())
            )
            matches = 0
            samples = 0
            # Compare fully opaque interiors, excluding antialiased boundaries
            # where the reference helper's oversampling can legitimately differ.
            for y in range(1, actual.height() - 1, 3):
                for x in range(1, actual.width() - 1, 3):
                    color = reference.pixelColor(x, y)
                    if color.alpha() != 255 or any(
                        reference.pixelColor(x + dx, y + dy) != color
                        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1))
                    ):
                        continue
                    samples += 1
                    if actual.pixelColor(x, y).rgb() == color.rgb():
                        matches += 1
            assert samples >= 50
            assert matches / samples >= 0.98
    finally:
        manager.set_theme(previous_theme, persist=False)
        logo.close()
        logo.deleteLater()
        application.processEvents()


def test_animation_renders_distinct_frames_at_two_times_display_scaling() -> None:
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["QT_SCALE_FACTOR"] = "2"
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "from PySide6.QtWidgets import QApplication; "
            "from PySide6.QtTest import QTest; "
            "from modsim_robotics_studio._splash_animation import AnimatedBrandLogo; "
            "app = QApplication([]); logo = AnimatedBrandLogo(); logo.resize(360, 76); "
            "logo.show(); logo.start(); app.processEvents(); "
            "first = logo.grab(); assert first.devicePixelRatio() == 2; "
            "assert first.width() == 720 and first.height() == 152; "
            "image = first.toImage(); pixels = bytes(image.constBits()); QTest.qWait(250); "
            "image = logo.grab().toImage(); assert pixels != bytes(image.constBits()); "
            "logo.close()",
        ],
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
