"""Window geometry respects logical desktops, content, and local preferences."""

from __future__ import annotations

import os
from collections.abc import Callable
from pathlib import Path
from typing import cast

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import (
    QCoreApplication,
    QEvent,
    QObject,
    QPoint,
    QPointF,
    QRect,
    QSettings,
    QSize,
    Qt,
    QTimer,
    Signal,
)
from PySide6.QtGui import QCloseEvent, QFont, QKeyEvent, QMouseEvent, QScreen
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QDialogButtonBox,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from modsim_robotics_studio import _window_sizing
from modsim_robotics_studio._window_sizing import WindowSizingPolicy, fit_geometry, manage_window


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.fixture
def preferences(tmp_path: Path) -> QSettings:
    return QSettings(str(tmp_path / "window-sizing.ini"), QSettings.Format.IniFormat)


@pytest.fixture
def predicted_narrow_monitor(monkeypatch: pytest.MonkeyPatch) -> QScreen:
    class PredictedScreen(QObject):
        availableGeometryChanged = Signal(QRect)

        def name(self) -> str:
            return "predicted-portrait-monitor"

        def availableGeometry(self) -> QRect:
            return QRect(0, 0, 320, 800)

    predicted = cast(QScreen, PredictedScreen())

    def initial_guess(_window: QWidget | None = None) -> QScreen:
        return predicted

    monkeypatch.setattr(_window_sizing, "target_screen", initial_guess)
    return predicted


def _delay_screen_assignment(
    dialog: QDialog, predicted: QScreen, monkeypatch: pytest.MonkeyPatch
) -> Callable[[], None]:
    actual = dialog.screen()
    reported = [predicted]

    def reported_screen() -> QScreen:
        return reported[0]

    def assign_actual_screen() -> None:
        reported[0] = actual
        handle = dialog.windowHandle()
        assert handle is not None
        handle.screenChanged.emit(actual)

    monkeypatch.setattr(dialog, "screen", reported_screen)
    return assign_actual_screen


def _dialog(point_size: int = 10) -> tuple[QDialog, QPushButton]:
    dialog = QDialog()
    font = QFont(dialog.font())
    font.setPointSize(point_size)
    dialog.setFont(font)
    layout = QVBoxLayout(dialog)
    button = QPushButton("Open Robot Pack", dialog)
    layout.addWidget(button)
    return dialog, button


@pytest.mark.parametrize(
    "available",
    [QRect(0, 0, 1920, 1040), QRect(-1080, -240, 1080, 1920), QRect(1920, 120, 800, 600)],
)
def test_initial_geometry_leaves_desktop_space_on_landscape_and_portrait_monitors(
    available: QRect,
) -> None:
    fitted = fit_geometry(QRect(30_000, -30_000, 6000, 4000), available, initial=True)
    assert available.contains(fitted)
    assert 0 < fitted.width() <= int(available.width() * 0.88)
    assert 0 < fitted.height() <= int(available.height() * 0.86)
    assert fitted.left() > available.left()
    assert fitted.top() > available.top()
    assert fitted.right() < available.right()
    assert fitted.bottom() < available.bottom()


def test_remembered_geometry_keeps_user_size_but_returns_to_a_connected_monitor() -> None:
    available = QRect(-1920, 120, 1920, 1040)
    requested = QRect(25_000, -25_000, 1200, 760)
    fitted = fit_geometry(requested, available)
    assert fitted.size() == requested.size()
    assert available.contains(fitted)
    assert fitted.left() > available.left()
    assert fitted.top() > available.top()


def test_remembered_geometry_larger_than_a_replacement_monitor_is_reduced() -> None:
    available = QRect(0, 0, 1024, 720)
    fitted = fit_geometry(QRect(-3000, 0, 1800, 1000), available)
    assert available.contains(fitted)
    assert 0 < fitted.width() < available.width()
    assert 0 < fitted.height() < available.height()


def test_show_uses_content_and_remains_a_normal_window(
    application: QApplication, preferences: QSettings
) -> None:
    dialog, button = _dialog()
    _policy = manage_window(
        dialog, "normal-dialog", preferred=QSize(420, 300), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        available = dialog.screen().availableGeometry()
        assert not dialog.isMaximized()
        assert not dialog.isFullScreen()
        assert dialog.width() >= button.sizeHint().width()
        assert dialog.frameGeometry().width() < available.width()
        assert dialog.frameGeometry().height() < available.height()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_enlarged_font_gets_more_space_and_button_text_fits(
    application: QApplication, preferences: QSettings
) -> None:
    small, _ = _dialog(8)
    large, large_button = _dialog(20)
    _policies = [
        manage_window(small, "small-font", preferred=QSize(260, 180), settings=preferences),
        manage_window(large, "large-font", preferred=QSize(260, 180), settings=preferences),
    ]
    try:
        small.show()
        large.show()
        application.processEvents()
        assert large.width() > small.width() + 40
        assert large.height() > small.height()
        assert large_button.width() >= large_button.sizeHint().width()
    finally:
        for dialog in (small, large):
            dialog.close()
            dialog.deleteLater()
        application.processEvents()


@pytest.mark.parametrize("finish", ["close", "accept", "reject"])
def test_resizing_is_restored_after_each_dialog_finish_path(
    application: QApplication, preferences: QSettings, finish: str
) -> None:
    first, _ = _dialog()
    second, _ = _dialog()
    _first_policy = manage_window(
        first, "remember-dialog", preferred=QSize(400, 300), settings=preferences
    )
    try:
        first.show()
        application.processEvents()
        resized = QSize(580, 430)
        first.resize(resized)
        application.processEvents()
        getattr(first, finish)()
        application.processEvents()
        assert not first.isVisible()
        stored = preferences.value("windows/remember-dialog/geometry")
        assert isinstance(stored, QRect)
        assert stored.isValid()
        _second_policy = manage_window(
            second, "remember-dialog", preferred=QSize(260, 180), settings=preferences
        )
        second.show()
        application.processEvents()
        assert second.size() == resized
        assert not second.isMaximized()
    finally:
        for dialog in (first, second):
            dialog.close()
            dialog.deleteLater()
        application.processEvents()


def test_maximized_session_reopens_with_its_normal_geometry(
    application: QApplication, preferences: QSettings
) -> None:
    first, _ = _dialog()
    second, _ = _dialog()
    _first_policy = manage_window(
        first, "normal-restore", preferred=QSize(400, 300), settings=preferences
    )
    try:
        first.show()
        application.processEvents()
        first.resize(540, 410)
        application.processEvents()
        normal_size = first.size()
        first.showMaximized()
        application.processEvents()
        assert first.isMaximized()
        first.close()
        application.processEvents()
        _second_policy = manage_window(
            second, "normal-restore", preferred=QSize(400, 300), settings=preferences
        )
        second.show()
        application.processEvents()
        assert not second.isMaximized()
        assert not second.isFullScreen()
        assert second.size() == normal_size
    finally:
        for dialog in (first, second):
            dialog.close()
            dialog.deleteLater()
        application.processEvents()


def test_different_windows_do_not_share_remembered_geometry(
    application: QApplication, preferences: QSettings
) -> None:
    first, _ = _dialog()
    second, _ = _dialog()
    _policies = [
        manage_window(first, "studio", preferred=QSize(400, 300), settings=preferences),
        manage_window(second, "runtime", preferred=QSize(400, 300), settings=preferences),
    ]
    try:
        first.show()
        application.processEvents()
        first.resize(580, 430)
        first.close()
        application.processEvents()
        second.show()
        application.processEvents()
        assert second.size() != QSize(580, 430)
    finally:
        for dialog in (first, second):
            dialog.close()
            dialog.deleteLater()
        application.processEvents()


def test_saved_geometry_on_a_disconnected_monitor_is_visible_on_reopen(
    application: QApplication, preferences: QSettings
) -> None:
    dialog, _ = _dialog()
    preferences.setValue("windows/disconnected/geometry", QRect(25_000, -2000, 1800, 1100))
    preferences.setValue("windows/disconnected/screen", "disconnected-monitor")
    preferences.setValue("windows/disconnected/available", QRect(24_000, -2000, 3840, 2160))
    preferences.setValue("windows/disconnected/font_height", dialog.fontMetrics().height())
    _policy = manage_window(dialog, "disconnected", preferred=QSize(420, 300), settings=preferences)
    try:
        dialog.show()
        application.processEvents()
        assert dialog.screen().availableGeometry().contains(dialog.frameGeometry())
        assert not dialog.isMaximized()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_transient_windows_can_skip_geometry_preferences(
    application: QApplication, preferences: QSettings
) -> None:
    dialog, _ = _dialog()
    _policy = manage_window(
        dialog, "transient", preferred=QSize(400, 300), settings=preferences, persist=False
    )
    try:
        dialog.show()
        application.processEvents()
        dialog.resize(580, 430)
        dialog.close()
        application.processEvents()
        assert preferences.allKeys() == []
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_font_size_change_expands_an_open_window_when_space_is_available(
    application: QApplication, preferences: QSettings
) -> None:
    dialog, button = _dialog(8)
    _policy = manage_window(
        dialog, "changing-font", preferred=QSize(260, 180), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        dialog.move(50, 50)
        application.processEvents()
        original_width = dialog.width()
        font = QFont(dialog.font())
        font.setPointSize(20)
        dialog.setFont(font)
        application.processEvents()
        application.processEvents()
        assert dialog.width() > original_width + 40
        assert button.width() >= button.sizeHint().width()
        assert dialog.frameGeometry().width() < dialog.screen().availableGeometry().width()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_child_dialog_uses_visible_parent_screen_before_pointer_screen(
    application: QApplication, monkeypatch: pytest.MonkeyPatch
) -> None:
    parent, _ = _dialog()
    child = QDialog(parent)

    def unexpected_pointer_query(_point: object) -> None:
        raise AssertionError("A parented dialog should use its visible parent's monitor")

    try:
        parent.show()
        application.processEvents()
        monkeypatch.setattr(_window_sizing.QGuiApplication, "screenAt", unexpected_pointer_query)
        assert _window_sizing.target_screen(child) is parent.screen()
    finally:
        parent.close()
        child.deleteLater()
        parent.deleteLater()
        application.processEvents()


def test_cancelled_close_does_not_replace_geometry_preferences(
    application: QApplication, preferences: QSettings
) -> None:
    class ConfirmCloseDialog(QDialog):
        allow_close = False

        def closeEvent(self, event: QCloseEvent) -> None:
            if self.allow_close:
                super().closeEvent(event)
            else:
                event.ignore()

    dialog = ConfirmCloseDialog()
    _policy = manage_window(
        dialog, "confirm-close", preferred=QSize(400, 300), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        dialog.resize(580, 430)
        dialog.close()
        application.processEvents()
        assert dialog.isVisible()
        assert preferences.allKeys() == []
        dialog.allow_close = True
        dialog.close()
        application.processEvents()
        assert not dialog.isVisible()
        assert preferences.contains("windows/confirm-close/geometry")
    finally:
        dialog.allow_close = True
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


@pytest.mark.parametrize("stored_font", ["broken", 0, -5])
def test_invalid_font_preference_keeps_a_reasonable_remembered_size(
    application: QApplication, preferences: QSettings, stored_font: str | int
) -> None:
    dialog, _ = _dialog()
    preferences.setValue("windows/invalid-font/geometry", QRect(50, 50, 400, 300))
    preferences.setValue("windows/invalid-font/font_height", stored_font)
    _policy = manage_window(dialog, "invalid-font", preferred=QSize(400, 300), settings=preferences)
    try:
        dialog.show()
        application.processEvents()
        assert dialog.frameGeometry().size() == QSize(400, 300)
        assert not dialog.isMaximized()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


@pytest.mark.parametrize("stored_geometry", ["broken", QRect(), QRect(40, 40, -20, 300)])
def test_invalid_geometry_preference_uses_normal_initial_sizing(
    application: QApplication, preferences: QSettings, stored_geometry: str | QRect
) -> None:
    dialog, button = _dialog()
    preferences.setValue("windows/invalid-geometry/geometry", stored_geometry)
    _policy = manage_window(
        dialog, "invalid-geometry", preferred=QSize(400, 300), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        available = dialog.screen().availableGeometry()
        assert available.contains(dialog.frameGeometry())
        assert 0 < dialog.frameGeometry().width() <= int(available.width() * 0.88)
        assert 0 < dialog.frameGeometry().height() <= int(available.height() * 0.86)
        assert button.width() >= button.sizeHint().width()
        assert not dialog.isMaximized()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_tall_creation_form_scrolls_with_confirmation_controls_always_visible(
    application: QApplication, preferences: QSettings, monkeypatch: pytest.MonkeyPatch
) -> None:
    pytest.importorskip("pyvistaqt")
    from modsim_robotics_studio import main_window

    def isolated_manage(window: QWidget, key: str, *, preferred: QSize) -> WindowSizingPolicy:
        return manage_window(window, key, preferred=preferred, settings=preferences)

    monkeypatch.setattr(main_window, "manage_window", isolated_manage)
    parent = QWidget()
    dialog = main_window._FormDialog(parent, "A long creation form", "long-form", QSize(500, 360))
    font = QFont(dialog.font())
    font.setPointSize(18)
    dialog.setFont(font)
    fields = []
    for index in range(25):
        field = QLineEdit(f"Value {index}")
        dialog.form.addRow(f"Field {index}", field)
        fields.append(field)
    errors: list[BaseException] = []

    def inspect_dialog() -> None:
        try:
            scroll = dialog.findChild(QScrollArea)
            buttons = dialog.findChild(QDialogButtonBox)
            assert scroll is not None and buttons is not None
            assert scroll.verticalScrollBar().maximum() > 0
            assert dialog.rect().contains(buttons.geometry())
            assert not scroll.isAncestorOf(buttons)
            for button in buttons.buttons():
                assert button.isVisible()
                assert button.width() >= button.minimumSizeHint().width()
                assert button.height() >= button.minimumSizeHint().height()
            scroll.ensureWidgetVisible(fields[-1])
            application.processEvents()
            last_rect = QRect(fields[-1].mapTo(scroll.viewport(), QPoint()), fields[-1].size())
            assert scroll.viewport().rect().contains(last_rect)
            assert not dialog.isMaximized()
        except BaseException as error:
            errors.append(error)
        finally:
            dialog.reject()

    try:
        parent.show()
        QTimer.singleShot(30, inspect_dialog)
        assert dialog.exec() == int(QDialog.DialogCode.Rejected)
        if errors:
            raise errors[0]
    finally:
        dialog.close()
        parent.close()
        dialog.deleteLater()
        parent.deleteLater()
        application.processEvents()


def test_screen_changes_are_handled_after_native_window_recreation(
    application: QApplication, preferences: QSettings
) -> None:
    dialog, _ = _dialog()
    _policy = manage_window(
        dialog, "recreated-handle", preferred=QSize(400, 300), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        original_handle = dialog.windowHandle()
        assert original_handle is not None
        dialog.destroy()
        dialog.show()
        application.processEvents()
        replacement_handle = dialog.windowHandle()
        assert replacement_handle is not None and replacement_handle is not original_handle
        dialog.move(30_000, -30_000)
        application.processEvents()
        replacement_handle.screenChanged.emit(dialog.screen())
        application.processEvents()
        assert dialog.screen().availableGeometry().contains(dialog.frameGeometry())
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_first_native_exposure_sizes_new_defaults_for_the_actual_monitor(
    application: QApplication, preferences: QSettings, predicted_narrow_monitor: QScreen
) -> None:
    dialog, button = _dialog()
    _policy = manage_window(
        dialog, "mapped-default", preferred=QSize(500, 360), settings=preferences
    )
    predicted_width = dialog.width()
    try:
        dialog.show()
        application.processEvents()
        application.processEvents()
        assert dialog.width() > predicted_width + 150
        assert button.width() >= button.minimumSizeHint().width()
        available = dialog.screen().availableGeometry()
        assert dialog.frameGeometry().width() <= int(available.width() * 0.88)
        assert dialog.frameGeometry().height() <= int(available.height() * 0.86)
        assert not dialog.isMaximized()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_first_native_exposure_preserves_remembered_geometry(
    application: QApplication, preferences: QSettings, predicted_narrow_monitor: QScreen
) -> None:
    dialog, _ = _dialog()
    remembered = QSize(240, 260)
    preferences.setValue("windows/mapped-restore/geometry", QRect(QPoint(30, 30), remembered))
    _policy = manage_window(
        dialog, "mapped-restore", preferred=QSize(500, 360), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        application.processEvents()
        assert dialog.frameGeometry().size() == remembered
        assert not dialog.isMaximized()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_first_native_exposure_preserves_explicit_resize_before_show(
    application: QApplication, preferences: QSettings, predicted_narrow_monitor: QScreen
) -> None:
    dialog, _ = _dialog()
    _policy = manage_window(
        dialog, "mapped-explicit", preferred=QSize(500, 360), settings=preferences
    )
    explicit = QSize(420, 330)
    dialog.resize(explicit)
    try:
        dialog.show()
        application.processEvents()
        application.processEvents()
        assert dialog.size() == explicit
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_later_monitor_changes_preserve_user_resize_after_initial_mapping(
    application: QApplication, preferences: QSettings, predicted_narrow_monitor: QScreen
) -> None:
    dialog, _ = _dialog()
    _policy = manage_window(
        dialog, "mapped-user-resize", preferred=QSize(500, 360), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        application.processEvents()
        resized = QSize(420, 330)
        dialog.resize(resized)
        application.processEvents()
        handle = dialog.windowHandle()
        assert handle is not None
        handle.screenChanged.emit(dialog.screen())
        application.processEvents()
        assert dialog.size() == resized
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_monitor_assigned_after_first_exposure_gets_comfortable_default_size(
    application: QApplication,
    preferences: QSettings,
    predicted_narrow_monitor: QScreen,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    dialog, _ = _dialog()
    assign_actual_screen = _delay_screen_assignment(dialog, predicted_narrow_monitor, monkeypatch)
    _policy = manage_window(dialog, "late-monitor", preferred=QSize(500, 360), settings=preferences)
    try:
        dialog.show()
        application.processEvents()
        application.processEvents()
        handle = dialog.windowHandle()
        assert handle is not None and handle.isExposed()
        initial_width = dialog.width()
        assign_actual_screen()
        application.processEvents()
        application.processEvents()
        assert dialog.width() > initial_width + 150
        assert dialog.screen().availableGeometry().contains(dialog.frameGeometry())
        assert not dialog.isMaximized()
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


def test_user_resize_before_late_monitor_assignment_is_preserved(
    application: QApplication,
    preferences: QSettings,
    predicted_narrow_monitor: QScreen,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    dialog, _ = _dialog()
    assign_actual_screen = _delay_screen_assignment(dialog, predicted_narrow_monitor, monkeypatch)
    _policy = manage_window(
        dialog, "late-monitor-resize", preferred=QSize(500, 360), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        application.processEvents()
        resized = QSize(420, 330)
        dialog.resize(resized)
        assign_actual_screen()
        application.processEvents()
        application.processEvents()
        assert dialog.size() == resized
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()


@pytest.mark.parametrize("input_kind", ["mouse", "key"])
def test_user_input_ends_initial_monitor_correction(
    application: QApplication,
    preferences: QSettings,
    predicted_narrow_monitor: QScreen,
    monkeypatch: pytest.MonkeyPatch,
    input_kind: str,
) -> None:
    dialog, _ = _dialog()
    assign_actual_screen = _delay_screen_assignment(dialog, predicted_narrow_monitor, monkeypatch)
    _policy = manage_window(
        dialog, "late-monitor-input", preferred=QSize(500, 360), settings=preferences
    )
    try:
        dialog.show()
        application.processEvents()
        application.processEvents()
        initial_size = dialog.size()
        handle = dialog.windowHandle()
        assert handle is not None
        if input_kind == "mouse":
            event = QMouseEvent(
                QEvent.Type.MouseButtonPress,
                QPointF(10, 10),
                QPointF(10, 10),
                Qt.MouseButton.LeftButton,
                Qt.MouseButton.LeftButton,
                Qt.KeyboardModifier.NoModifier,
            )
        else:
            event = QKeyEvent(
                QEvent.Type.KeyPress, Qt.Key.Key_Shift, Qt.KeyboardModifier.NoModifier
            )
        QCoreApplication.sendEvent(handle, event)
        assign_actual_screen()
        application.processEvents()
        application.processEvents()
        assert dialog.size() == initial_size
    finally:
        dialog.close()
        dialog.deleteLater()
        application.processEvents()
