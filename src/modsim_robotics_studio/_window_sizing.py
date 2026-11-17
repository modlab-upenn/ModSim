"""Shared logical-pixel sizing and local geometry preferences for Qt windows."""

from __future__ import annotations

import weakref
from contextlib import suppress

from PySide6.QtCore import QByteArray, QEvent, QObject, QPoint, QRect, QSettings, QSize, Qt, QTimer
from PySide6.QtGui import QCursor, QGuiApplication, QScreen, QWindow
from PySide6.QtWidgets import QMainWindow, QWidget


def _native_handle(window: QWidget) -> QWindow | None:
    # PySide's annotation omits None, which Qt returns before native creation.
    return window.windowHandle()


def target_screen(window: QWidget | None = None) -> QScreen:
    """Choose the visible parent's screen, then the pointer's screen.

    Returns:
        The display whose usable logical geometry should bound a new window.
    """
    parent = window.parentWidget() if window is not None else None
    if parent is not None and parent.window().isVisible():
        return parent.window().screen()
    return QGuiApplication.screenAt(QCursor.pos()) or (
        window.screen() if window is not None else QGuiApplication.primaryScreen()
    )


def fit_geometry(requested: QRect, available: QRect, *, initial: bool = False) -> QRect:
    """Fit a normal window inside one monitor, including negative monitor origins.

    Args:
        requested: Preferred outer geometry in logical pixels.
        available: Usable monitor rectangle, excluding desktop panels where reported.
        initial: Leave generous margins for a new window. A remembered or manually
            resized window may use more of the desktop while remaining reachable.

    Returns:
        A positive rectangle wholly within the usable desktop, with margins.
    """
    margin = min(24, max(0, min(available.width(), available.height()) // 20))
    bounds = available.adjusted(margin, margin, -margin, -margin)
    width_limit = max(1, bounds.width())
    height_limit = max(1, bounds.height())
    if initial:
        width_limit = min(width_limit, max(1, int(available.width() * 0.88)))
        height_limit = min(height_limit, max(1, int(available.height() * 0.86)))
    width = max(1, min(requested.width(), width_limit))
    height = max(1, min(requested.height(), height_limit))
    x = min(max(requested.x(), bounds.left()), bounds.right() - width + 1)
    y = min(max(requested.y(), bounds.top()), bounds.bottom() - height + 1)
    return QRect(x, y, width, height)


class WindowSizingPolicy(QObject):
    """Keep a window normal-sized, readable, reachable, and locally remembered.

    Geometry is measured in Qt logical pixels. Device pixel ratios belong to
    Qt's renderer and must not be multiplied into these dimensions. Parenting
    this controller to the window keeps timers and signal connections bounded
    by the window's lifetime.
    """

    def __init__(
        self,
        window: QWidget,
        key: str,
        *,
        preferred: QSize,
        minimum: QSize | None = None,
        settings: QSettings | None = None,
        persist: bool = True,
    ) -> None:
        super().__init__(window)
        # The window already owns this QObject. A strong reverse Python
        # reference retains whole Qt/plotting trees until interpreter shutdown,
        # potentially destroying them after QApplication's screens are gone.
        self._window_ref = weakref.ref(window)
        self._prefix = f"windows/{key}"
        self._preferred = QSize(preferred)
        self._minimum = QSize(minimum) if minimum is not None else QSize(0, 0)
        self._settings = settings if settings is not None else QSettings("ModSim", "Studio")
        self._persist = persist
        self._shown = False
        self._awaiting_exposure = True
        self._unconfirmed_screen = True
        self._explicit_initial_resize = False
        self._applying = False
        self._connected_handle: QWindow | None = None
        self._screen: QScreen | None = None
        self._font_height = window.fontMetrics().height()
        self._pending = QTimer(self)
        self._pending.setSingleShot(True)
        self._pending.timeout.connect(self._refresh_bounds)
        self._prepare()
        self._prepared_size = window.size()
        window.installEventFilter(self)

    @property
    def _window(self) -> QWidget:
        window = self._window_ref()
        if window is None:
            raise RuntimeError("The managed window has been destroyed.")
        return window

    @property
    def has_saved_layout(self) -> bool:
        """Whether a persisted main-window dock arrangement is available."""
        return self._persist and isinstance(
            self._settings.value(f"{self._prefix}/dock_state"), QByteArray
        )

    def _natural_size(self) -> QSize:
        window = self._window
        window.ensurePolished()
        layout = window.layout()
        if layout is not None:
            layout.activate()
        scale = max(1.0, window.fontMetrics().height() / 17.0)
        preferred = QSize(
            round(self._preferred.width() * scale), round(self._preferred.height() * scale)
        )
        natural = window.sizeHint().expandedTo(window.minimumSizeHint())
        return preferred.expandedTo(natural).expandedTo(self._minimum)

    def _prepare(self) -> None:
        screen = target_screen(self._window)
        geometry = self._settings.value(f"{self._prefix}/geometry") if self._persist else None
        restored = isinstance(geometry, QRect) and geometry.isValid()
        self._restored_geometry = restored
        if restored:
            assert isinstance(geometry, QRect)
            # Independent application windows return to their previous monitor
            # when it still exists. Dialogs stay on their visible parent's monitor.
            parent = self._window.parentWidget()
            if parent is None or not parent.window().isVisible():
                name = self._settings.value(f"{self._prefix}/screen", "", type=str)
                screen = next(
                    (item for item in QGuiApplication.screens() if item.name() == name), screen
                )
            available = screen.availableGeometry()
            old_available = self._settings.value(f"{self._prefix}/available")
            if isinstance(old_available, QRect) and old_available.isValid():
                geometry.translate(available.topLeft() - old_available.topLeft())
            stored_font = self._settings.value(f"{self._prefix}/font_height", 0, type=int)
            old_font = stored_font if isinstance(stored_font, int) else 0
            current_font = self._window.fontMetrics().height()
            font_scale = current_font / old_font if old_font > 0 else 1.0
            geometry.setSize(
                QSize(round(geometry.width() * font_scale), round(geometry.height() * font_scale))
            )
            geometry = fit_geometry(geometry, available)
        else:
            geometry = self._default_geometry(screen)
        self._prepared_screen_name = screen.name()
        self._apply(geometry)
        if isinstance(self._window, QMainWindow) and self._persist:
            state = self._settings.value(f"{self._prefix}/dock_state")
            if isinstance(state, QByteArray):
                self._window.restoreState(state, 1)

    def _default_geometry(self, screen: QScreen) -> QRect:
        available = screen.availableGeometry()
        geometry = QRect(QPoint(), self._natural_size())
        geometry.moveCenter(available.center())
        geometry = fit_geometry(geometry, available, initial=True)
        parent = self._window.parentWidget()
        center = (
            parent.window().frameGeometry().center()
            if parent is not None and parent.window().isVisible()
            else available.center()
        )
        geometry.moveCenter(center)
        return fit_geometry(geometry, available)

    def _first_exposure(self) -> None:
        self._awaiting_exposure = False
        # Wayland may place a window on a different monitor from the pointer
        # position reported before this process owns any surfaces. Re-evaluate
        # new-window preferences once the compositor has actually mapped it.
        # Saved geometry and explicit pre-show resizing remain user choices.
        self._pending.start(0)

    def _reconcile_screen(self, screen: QScreen) -> None:
        if not self._unconfirmed_screen:
            return
        if (
            self._restored_geometry
            or self._explicit_initial_resize
            or self._window.size() != self._last_policy_size
        ):
            self._unconfirmed_screen = False
            return
        if screen.name() != self._prepared_screen_name:
            self._unconfirmed_screen = False
            self._apply(self._default_geometry(screen))

    def _apply(self, outer: QRect) -> None:
        self._applying = True
        try:
            window = self._window
            # QWidget geometry excludes decorations; the policy bounds the full
            # frame. Margins are known once the native window has been created.
            handle = _native_handle(window)
            margins = handle.frameMargins() if handle is not None else None
            horizontal = margins.left() + margins.right() if margins is not None else 0
            vertical = margins.top() + margins.bottom() if margins is not None else 0
            size = QSize(max(1, outer.width() - horizontal), max(1, outer.height() - vertical))
            # Layouts provide natural minima, but scroll areas must still be
            # allowed to fit a small monitor rather than pushing buttons off it.
            minimum = window.minimumSize().boundedTo(size)
            window.setMinimumSize(minimum)
            window.resize(size)
            window.move(outer.topLeft())
            self._last_policy_size = window.size()
        finally:
            self._applying = False

    def _watch_screen(self, screen: QScreen) -> None:
        if self._screen is screen:
            return
        if self._screen is not None:
            # A removed monitor may already have been destroyed by Qt.
            with suppress(RuntimeError, TypeError):
                self._screen.availableGeometryChanged.disconnect(self._available_changed)
        self._screen = screen
        screen.availableGeometryChanged.connect(self._available_changed)

    def _available_changed(self, _available: QRect) -> None:
        self._pending.start(0)

    def _detach_screen(self) -> None:
        self._pending.stop()
        if self._screen is not None:
            with suppress(RuntimeError, TypeError):
                self._screen.availableGeometryChanged.disconnect(self._available_changed)
            self._screen = None
        if self._connected_handle is not None:
            with suppress(RuntimeError, TypeError):
                self._connected_handle.screenChanged.disconnect(self._screen_changed)
                self._connected_handle.removeEventFilter(self)
            self._connected_handle = None

    def _screen_changed(self, screen: QScreen) -> None:
        self._watch_screen(screen)
        if not self._applying:
            # wl_surface.enter can arrive after the first Expose. Resolve that
            # first actual monitor while the size is still policy-chosen. A
            # user resize or native input ends this initial reconciliation.
            self._pending.start(0)

    def _refresh_bounds(self) -> None:
        window = self._window
        if not window.isVisible() or window.isMaximized() or window.isFullScreen():
            return
        screen = window.screen()
        self._watch_screen(screen)
        # Resize after the native screen/configure event has finished. Changing
        # size inside screenChanged can be overwritten by that same Wayland
        # configure event's old dimensions.
        self._reconcile_screen(screen)
        original = window.frameGeometry()
        geometry = QRect(original)
        font_height = window.fontMetrics().height()
        if font_height > self._font_height:
            geometry.setSize(geometry.size().expandedTo(self._natural_size()))
        self._font_height = font_height
        fitted = fit_geometry(geometry, screen.availableGeometry())
        if fitted != original:
            self._apply(fitted)

    def _remember(self) -> None:
        if not self._persist or not self._shown:
            return
        window = self._window
        # Never persist maximized/fullscreen state. On the next launch even a
        # previously maximized application opens at its last normal dimensions.
        geometry = window.frameGeometry()
        if window.isMaximized() or window.isFullScreen():
            geometry = window.normalGeometry()
            handle = _native_handle(window)
            if handle is not None:
                margins = handle.frameMargins()
                geometry.adjust(-margins.left(), -margins.top(), margins.right(), margins.bottom())
        self._settings.setValue(f"{self._prefix}/geometry", geometry)
        self._settings.setValue(f"{self._prefix}/screen", window.screen().name())
        self._settings.setValue(f"{self._prefix}/available", window.screen().availableGeometry())
        self._settings.setValue(f"{self._prefix}/font_height", window.fontMetrics().height())
        if isinstance(window, QMainWindow):
            self._settings.setValue(f"{self._prefix}/dock_state", window.saveState(1))
        self._settings.sync()

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        # During QObject teardown Qt may still deliver child events after the
        # Python wrapper's attributes have been cleared.
        reference = getattr(self, "_window_ref", None)
        window = reference() if reference is not None else None
        if window is not None and watched is window:
            if event.type() == QEvent.Type.Show:
                if not self._shown:
                    # Clear any inherited state instead of restoring maximized
                    # geometry. Explicit subsequent user maximization still works.
                    self._window.setWindowState(Qt.WindowState.WindowNoState)
                    self._explicit_initial_resize = self._window.size() != self._prepared_size
                    if not self._explicit_initial_resize:
                        self._prepare()
                self._shown = True
                handle = _native_handle(self._window)
                if handle is not None and handle is not self._connected_handle:
                    self._detach_screen()
                    handle.installEventFilter(self)
                    handle.screenChanged.connect(self._screen_changed)
                    self._connected_handle = handle
                self._watch_screen(self._window.screen())
                self._pending.start(0)
            elif event.type() == QEvent.Type.Hide:
                # QDialog.accept/reject hide without Close; ignored close events
                # never hide and therefore never overwrite saved geometry.
                self._remember()
                self._detach_screen()
            elif event.type() in (QEvent.Type.FontChange, QEvent.Type.StyleChange):
                if self._shown and not self._applying:
                    self._pending.start(0)
        elif window is not None and watched is self._connected_handle:
            if event.type() in (
                QEvent.Type.MouseButtonPress,
                QEvent.Type.NonClientAreaMouseButtonPress,
                QEvent.Type.KeyPress,
            ):
                self._unconfirmed_screen = False
            elif (
                event.type() == QEvent.Type.Expose
                and self._awaiting_exposure
                and self._connected_handle is not None
                and self._connected_handle.isExposed()
            ):
                self._first_exposure()
        return False


def manage_window(
    window: QWidget,
    key: str,
    *,
    preferred: QSize,
    minimum: QSize | None = None,
    settings: QSettings | None = None,
    persist: bool = True,
) -> WindowSizingPolicy:
    """Attach the shared sizing policy after a window's contents are assembled.

    Args:
        window: Top-level widget or dialog to manage on the GUI thread.
        key: Stable local preference identifier, independent of filenames/titles.
        preferred: Comfortable dimensions at the standard body-font size.
        minimum: Additional preferred floor, still bounded by the display.
        settings: Optional settings store, useful for isolated tests.
        persist: Remember normal geometry and main-window dock layout on hide.

    Returns:
        Controller owned by ``window``; keep a Python reference on the caller.
    """
    return WindowSizingPolicy(
        window, key, preferred=preferred, minimum=minimum, settings=settings, persist=persist
    )
