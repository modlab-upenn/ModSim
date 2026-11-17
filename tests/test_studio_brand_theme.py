"""Approved brand palettes and compatibility with saved Studio preferences."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QCoreApplication, QEvent, QSettings, QSize, Qt
from PySide6.QtGui import QColor, QFont, QFontInfo, QImage, QPalette
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QStyle,
    QStyleOptionComboBox,
    QStyleOptionFrame,
    QStyleOptionSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from modsim_robotics_studio.appearance import DEFAULT_THEME, THEMES, ThemeManager


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.mark.parametrize(
    ("theme_id", "background", "text", "accent", "focus"),
    [
        ("light", "#F8FBFE", "#10263C", "#0B7D73", "#1E4E8C"),
        ("dark", "#17191F", "#EEF2F6", "#35B3A3", "#8CBCEE"),
        ("midnight", "#0B1521", "#C5CCD3", "#35B3A3", "#8CBCEE"),
    ],
)
def test_brand_palette_reaches_native_widgets(
    application: QApplication,
    tmp_path: Path,
    theme_id: str,
    background: str,
    text: str,
    accent: str,
    focus: str,
) -> None:
    settings = QSettings(str(tmp_path / "appearance.ini"), QSettings.Format.IniFormat)
    settings.setValue("appearance/theme", theme_id)
    manager = ThemeManager(application, settings)
    button = QPushButton("Start simulation")
    button.setProperty("primary", True)
    button.ensurePolished()
    try:
        theme = manager.theme
        assert theme.shell == background.lower()
        assert theme.text == text.lower()
        assert theme.accent == accent.lower()
        assert theme.focus == focus.lower()
        assert application.palette().color(QPalette.ColorRole.Window) == QColor(background)
        assert application.palette().color(QPalette.ColorRole.WindowText) == QColor(text)
        assert button.palette().color(QPalette.ColorRole.ButtonText) == QColor(theme.on_accent)
        assert QApplication.testAttribute(Qt.ApplicationAttribute.AA_DontUseNativeDialogs)
        assert not application.windowIcon().isNull()
    finally:
        button.deleteLater()
        manager.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)


def test_graphite_preference_restores_dark_without_implicit_rewrite(
    application: QApplication, tmp_path: Path
) -> None:
    settings = QSettings(str(tmp_path / "legacy.ini"), QSettings.Format.IniFormat)
    settings.setValue("appearance/theme", "graphite")
    manager = ThemeManager(application, settings)
    try:
        assert manager.theme.id == "dark"
        assert settings.value("appearance/theme") == "graphite"
        manager.set_theme("graphite")
        assert manager.theme.id == "dark"
        assert settings.value("appearance/theme") == "dark"
    finally:
        manager.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)


def test_theme_choices_are_the_three_approved_packs() -> None:
    assert tuple(THEMES) == ("light", "dark", "midnight")
    assert DEFAULT_THEME == "midnight"


def test_appearance_preserves_accessibility_font_and_scales_semantic_labels(
    application: QApplication, tmp_path: Path
) -> None:
    original = application.font()
    enlarged = QFont(original)
    enlarged.setPointSizeF(16.0)
    application.setFont(enlarged)
    manager = ThemeManager(
        application, QSettings(str(tmp_path / "large-font.ini"), QSettings.Format.IniFormat)
    )
    labels: list[QLabel] = []
    try:
        assert application.font().pointSizeF() == pytest.approx(16.0)
        family = QFontInfo(application.font()).family()
        for name, factor in (
            ("BrandTitle", 1.7),
            ("Workspace", 1.0),
            ("LoadingStage", 1.15),
            ("LoadingDetail", 0.9),
        ):
            label = QLabel("Readable interface text")
            label.setObjectName(name)
            label.ensurePolished()
            labels.append(label)
            assert label.font().pointSizeF() == pytest.approx(16.0 * factor, abs=0.05)
            assert QFontInfo(label.font()).family() == family
    finally:
        for label in labels:
            label.deleteLater()
        manager.deleteLater()
        application.setFont(original)
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)


@pytest.mark.parametrize("theme_id", ["light", "dark", "midnight"])
@pytest.mark.parametrize("point_size", [10.0, 16.0])
def test_native_controls_and_delegate_editors_remain_readable(
    application: QApplication, tmp_path: Path, theme_id: str, point_size: float
) -> None:
    original = application.font()
    font = QFont(original)
    font.setPointSizeF(point_size)
    application.setFont(font)
    settings = QSettings(str(tmp_path / "controls.ini"), QSettings.Format.IniFormat)
    settings.setValue("appearance/theme", theme_id)
    manager = ThemeManager(application, settings)
    panel = QWidget()
    layout = QVBoxLayout(panel)
    combo = QComboBox()
    combo.addItems(["ModSim Midnight", "ModSim Light"])
    spin = QDoubleSpinBox()
    spin.setValue(5.0)
    layout.addWidget(combo)
    layout.addWidget(spin)
    views = (QListWidget(), QTreeWidget(), QTableWidget(1, 1))
    for view in views:
        view.setMaximumHeight(130)
        layout.addWidget(view)
    list_item = QListWidgetItem("Rename this RobotPack", views[0])
    list_item.setFlags(list_item.flags() | Qt.ItemFlag.ItemIsEditable)
    tree_item = QTreeWidgetItem(views[1], ["Rename this RobotPack"])
    tree_item.setFlags(tree_item.flags() | Qt.ItemFlag.ItemIsEditable)
    table_item = QTableWidgetItem("Rename this RobotPack")
    views[2].setItem(0, 0, table_item)
    views[2].verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
    panel.resize(650, 600)
    panel.show()
    application.processEvents()
    expected_dpr = os.environ.get("MODSIM_TEST_EXPECTED_DPR")
    if expected_dpr is not None:
        assert panel.devicePixelRatioF() == pytest.approx(float(expected_dpr))
    views[0].openPersistentEditor(list_item)
    views[1].openPersistentEditor(tree_item)
    views[2].openPersistentEditor(table_item)
    application.processEvents()
    try:
        for view in views:
            editor = view.findChild(QLineEdit)
            assert editor is not None
            option = QStyleOptionFrame()
            editor.initStyleOption(option)
            contents = editor.style().subElementRect(
                QStyle.SubElement.SE_LineEditContents, option, editor
            )
            assert contents.height() >= editor.fontMetrics().height(), (
                theme_id,
                point_size,
                view.metaObject().className(),
                contents,
                editor.fontMetrics().height(),
            )

        # A custom border/drop-down stylesheet previously removed these native
        # glyphs entirely. Inspect the rendered control interiors, not the frame.
        combo_option = QStyleOptionComboBox()
        combo.initStyleOption(combo_option)
        spin_option = QStyleOptionSpinBox()
        spin.initStyleOption(spin_option)
        for widget, option, control, arrow in (
            (
                combo,
                combo_option,
                QStyle.ComplexControl.CC_ComboBox,
                QStyle.SubControl.SC_ComboBoxArrow,
            ),
            (
                spin,
                spin_option,
                QStyle.ComplexControl.CC_SpinBox,
                QStyle.SubControl.SC_SpinBoxUp,
            ),
            (
                spin,
                spin_option,
                QStyle.ComplexControl.CC_SpinBox,
                QStyle.SubControl.SC_SpinBoxDown,
            ),
        ):
            area = (
                widget.style().subControlRect(control, option, arrow, widget).adjusted(3, 2, -3, -2)
            )
            image = widget.grab().toImage()
            ratio = image.devicePixelRatio()
            brightness = [
                sum(image.pixelColor(round(x * ratio), round(y * ratio)).getRgb()[:3]) / 3
                for y in range(area.top(), area.bottom() + 1)
                for x in range(area.left(), area.right() + 1)
            ]
            assert max(brightness) - min(brightness) > 45, (theme_id, point_size, arrow)
    finally:
        panel.close()
        panel.deleteLater()
        manager.deleteLater()
        application.setFont(original)
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)


@pytest.mark.parametrize("scale", [1.5, 2.0])
def test_native_controls_and_editors_at_fractional_and_high_dpi(
    tmp_path: Path, scale: float
) -> None:
    environment = dict(
        os.environ,
        QT_QPA_PLATFORM="offscreen",
        QT_SCALE_FACTOR=str(scale),
        MODSIM_TEST_EXPECTED_DPR=str(scale),
    )
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            f"{__file__}::test_native_controls_and_delegate_editors_remain_readable",
            "--no-cov",
            f"--basetemp={tmp_path / 'scaled-controls'}",
            "--tb=short",
            "-q",
        ],
        env=environment,
        text=True,
        capture_output=True,
        check=False,
        timeout=40,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_wordmark_renders_for_the_destination_monitor_scale(application: QApplication) -> None:
    from modsim_robotics_studio._branding import vector_pixmap

    pixmap = vector_pixmap(
        "midnight",
        "logos/svg/modsim-horizontal.svg",
        QSize(240, 48),
        device_pixel_ratio=3.0,
    )
    assert pixmap.size() == QSize(720, 144)
    assert pixmap.devicePixelRatio() == 3.0
    assert pixmap.deviceIndependentSize().toSize() == QSize(240, 48)
    assert not pixmap.isNull()


@pytest.mark.parametrize("theme_id", ["light", "dark", "midnight"])
def test_app_icons_include_original_1024_sources_and_vector_retina_rendering(
    application: QApplication, theme_id: str
) -> None:
    from modsim_robotics_studio._branding import application_icon, asset_path

    source = QImage(str(asset_path(theme_id, "icons/png/app/modsim-app-1024.png")))
    assert source.size() == QSize(1024, 1024)
    icon = application_icon(theme_id)
    retina = icon.pixmap(QSize(512, 512), 2.0)
    assert not retina.isNull()
    assert retina.size() == QSize(1024, 1024)
    assert retina.devicePixelRatio() == 2.0


@pytest.mark.parametrize("scale", [1.0, 2.0, 3.0, 4.0])
def test_toolbar_icons_supply_physical_pixels_at_each_display_scale(
    application: QApplication, scale: float
) -> None:
    from modsim_robotics_studio.icons import studio_icon

    pixmap = studio_icon("modules", "#35B3A3").pixmap(QSize(20, 20), scale)
    assert pixmap.size() == QSize(int(20 * scale), int(20 * scale))
    assert pixmap.devicePixelRatio() == scale


def test_linux_identity_matches_desktop_entry_before_a_window_opens(
    application: QApplication, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from modsim_robotics_studio import _branding, _desktop

    previous = (
        application.applicationName(),
        application.applicationDisplayName(),
        application.organizationName(),
        application.desktopFileName(),
    )
    monkeypatch.setattr(_desktop.sys, "platform", "linux")
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path))
    try:
        _branding.configure_application_identity(application, "ModSim Runtime Inspector")
        assert application.applicationName() == _desktop.APPLICATION_ID
        assert application.desktopFileName() == _desktop.APPLICATION_ID
        assert application.applicationDisplayName() == "ModSim Runtime Inspector"
        assert (tmp_path / "applications" / f"{_desktop.APPLICATION_ID}.desktop").is_file()
    finally:
        application.setApplicationName(previous[0])
        application.setApplicationDisplayName(previous[1])
        application.setOrganizationName(previous[2])
        application.setDesktopFileName(previous[3])


def test_failed_desktop_registration_logs_warning_and_keeps_studio_available(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    from modsim_robotics_studio import _branding

    def denied(_icon: Path) -> None:
        raise PermissionError("Desktop data is read-only")

    monkeypatch.setattr(_branding, "register_desktop_entry", denied)
    _branding.configure_application_identity(application, application.applicationDisplayName())
    assert "Could not register the ModSim desktop icon" in caplog.text
