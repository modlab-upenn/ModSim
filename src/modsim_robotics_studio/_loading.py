"""Internal branded loading widgets for the optional desktop applications."""

from __future__ import annotations

import math
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from types import TracebackType
from typing import Literal, TypeVar

from PySide6.QtCore import QElapsedTimer, QEvent, QEventLoop, QPointF, QSize, Qt, QTimer
from PySide6.QtGui import (
    QCloseEvent,
    QColor,
    QMouseEvent,
    QPainter,
    QPaintEvent,
    QPixmap,
    QResizeEvent,
    QShowEvent,
)
from PySide6.QtWidgets import (
    QAbstractButton,
    QApplication,
    QDialog,
    QFrame,
    QLabel,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QSplashScreen,
    QVBoxLayout,
    QWidget,
)

from modsim_robotics import __version__
from modsim_robotics_studio._branding import asset_path, vector_pixmap
from modsim_robotics_studio._splash_animation import AnimatedBrandLogo
from modsim_robotics_studio._window_sizing import manage_window, target_screen
from modsim_robotics_studio.appearance import theme_manager

_Result = TypeVar("_Result")
_SPLASH_MINIMUM_MS = 3_000
_SPLASH_INTRO_MS = 1_100


class BrandLogo(QLabel):
    """Render the supplied outlined wordmark at the available widget size."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAccessibleName("ModSim")
        self.setMinimumWidth(0)
        self.setFixedHeight(52)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        self._appearance = theme_manager()
        self._appearance.changed.connect(self._refresh)
        self._refresh()

    def _refresh(self) -> None:
        self.setPixmap(
            vector_pixmap(
                self._appearance.theme.id,
                "logos/svg/modsim-horizontal.svg",
                QSize(max(1, self.width()), self.height()),
                device_pixel_ratio=self.devicePixelRatioF(),
            )
        )

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)
        self._refresh()

    def event(self, event: QEvent) -> bool:
        handled = super().event(event)
        if event.type() == QEvent.Type.DevicePixelRatioChange:
            self._refresh()
        return handled


class _StartupSteps(QWidget):
    """Mark actual startup milestones without a time-based progress estimate."""

    def __init__(self, titles: tuple[str, ...], parent: QWidget) -> None:
        super().__init__(parent)
        self.setAccessibleName("Runtime startup steps")
        self._titles = titles
        self._current = 0
        self._outcome: Literal["complete", "stopped", "failed"] | None = None
        self._labels: list[QLabel] = []
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(6)
        for index in range(len(titles)):
            label = QLabel()
            label.setObjectName(f"StartupStep{index}")
            label.setWordWrap(True)
            layout.addWidget(label)
            self._labels.append(label)
        self._appearance = theme_manager()
        self._appearance.changed.connect(self._refresh)
        self._refresh()

    def advance(self, index: int) -> None:
        if not 0 <= index < len(self._titles):
            raise ValueError("Startup step must identify an existing milestone")
        if self._outcome is None and index >= self._current:
            self._current = index
            self._refresh()

    def finish(self, outcome: Literal["complete", "stopped", "failed"]) -> None:
        if self._outcome == "complete":
            return
        self._outcome = outcome
        self._refresh()

    def _refresh(self) -> None:
        theme = self._appearance.theme
        for index, (label, title) in enumerate(zip(self._labels, self._titles, strict=True)):
            if self._outcome == "complete" or index < self._current:
                marker, state, color = "✓", "complete", theme.success
            elif index > self._current:
                marker, state, color = "○", "waiting", theme.muted
            elif self._outcome == "failed":
                marker, state, color = "!", "failed", theme.danger
            elif self._outcome == "stopped":
                marker, state, color = "■", "stopped", theme.warning
            else:
                marker, state, color = "●", "current", theme.accent
            label.setText(f"{marker}  {title}")
            label.setAccessibleName(f"{title}: {state}")
            weight = 600 if state == "current" else 400
            label.setStyleSheet(f"color: {color}; font-weight: {weight};")


class LoadingPanel(QWidget):
    """Show approved robot artwork with live status and an indeterminate bar."""

    def __init__(
        self, workspace: str, parent: QWidget | None = None, *, animated_logo: bool = False
    ) -> None:
        super().__init__(parent)
        self.setObjectName("LoadingPanel")
        self.setMinimumSize(460, 280)
        self._appearance = theme_manager()
        self._background = QPixmap()
        self._scaled_background: QPixmap | None = None
        self._background_scale: tuple[int, int, float] | None = None
        self._steps: _StartupSteps | None = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.addStretch(1)
        self.copy = QWidget()
        self.copy.setObjectName("LoadingCopy")
        self.copy.setMaximumWidth(340)
        content = QVBoxLayout(self.copy)
        self._content = content
        content.setContentsMargins(0, 0, 0, 0)
        content.setSpacing(12)
        self.logo = AnimatedBrandLogo() if animated_logo else BrandLogo()
        content.addWidget(self.logo)
        title = QLabel(workspace)
        title.setObjectName("BrandTitle")
        title.setWordWrap(True)
        content.addWidget(title)
        self.stage = QLabel("Preparing your workspace…")
        self.stage.setObjectName("LoadingStage")
        self.stage.setTextFormat(Qt.TextFormat.PlainText)
        self.stage.setWordWrap(True)
        self.stage.setAccessibleName("Loading status")
        content.addWidget(self.stage)
        self.detail = QLabel()
        self.detail.setObjectName("LoadingDetail")
        self.detail.setTextFormat(Qt.TextFormat.PlainText)
        self.detail.setWordWrap(True)
        self.detail.hide()
        content.addWidget(self.detail)
        self.progress = QProgressBar()
        self.progress.setObjectName("LoadingProgress")
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(6)
        self.progress.setAccessibleName("Operation in progress")
        content.addWidget(self.progress)
        version = QLabel(f"VERSION {__version__}")
        version.setObjectName("SplashVersion")
        content.addWidget(version)
        layout.addWidget(self.copy, 0, Qt.AlignmentFlag.AlignLeft)
        layout.addStretch(1)
        self._appearance.changed.connect(self._refresh)
        self._refresh()
        self.set_busy(True)

    def set_stage(self, message: str, detail: str = "") -> None:
        """Display the current operation without inventing percentage progress."""
        self.stage.setText(message)
        self.detail.setText(detail)
        self.detail.setVisible(bool(detail))
        self._layout_copy()

    def set_busy(self, busy: bool) -> None:
        """Animate only while initialization or document work is in progress."""
        self.progress.setRange(0, 0 if busy else 1)
        if not busy:
            self.progress.setValue(1)
        self.progress.setVisible(busy)
        self._layout_copy()

    def set_steps(self, titles: tuple[str, ...]) -> None:
        """Show ordered milestones for a workspace that reports startup phases."""
        if self._steps is not None:
            self._content.removeWidget(self._steps)
            self._steps.deleteLater()
        self._steps = _StartupSteps(titles, self.copy)
        self._content.insertWidget(self._content.indexOf(self.progress), self._steps)
        self._content.setSpacing(9)
        self._layout_copy()

    def set_step(self, index: int) -> None:
        """Advance only when the runtime reports entering the corresponding phase."""
        if self._steps is not None:
            self._steps.advance(index)
            self._layout_copy()

    def finish_steps(self, outcome: Literal["complete", "stopped", "failed"]) -> None:
        """Keep the last startup outcome visible if no runtime state was produced."""
        if self._steps is not None:
            self._steps.finish(outcome)
            self._layout_copy()

    def _layout_copy(self) -> None:
        width = max(190, min(340, int(self.width() * 0.35)))
        button_width = max(
            (
                button.minimumSizeHint().width()
                for button in self.copy.findChildren(QAbstractButton)
                if not button.isHidden()
            ),
            default=0,
        )
        width = min(max(width, button_width), max(190, self.width() - 48))
        self.copy.setFixedWidth(width)
        content = self.copy.layout()
        if content is not None:
            # Respect wrapped text before the outer layout divides spare space.
            self.copy.setFixedHeight(content.totalHeightForWidth(width))

    def changeEvent(self, event: QEvent) -> None:
        super().changeEvent(event)
        if hasattr(self, "copy") and event.type() in (
            QEvent.Type.FontChange,
            QEvent.Type.StyleChange,
            QEvent.Type.ApplicationFontChange,
        ):
            self._layout_copy()

    def _refresh(self) -> None:
        self._background = QPixmap(
            str(asset_path(self._appearance.theme.id, "backgrounds/robot-background-1920x1080.png"))
        )
        self._scaled_background = None
        self._background_scale = None
        self._layout_copy()
        self.update()

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)
        self._layout_copy()
        self._scaled_background = None
        self._background_scale = None

    def paintEvent(self, event: QPaintEvent) -> None:
        super().paintEvent(event)
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(self._appearance.theme.shell))
        if self._background.isNull():
            return
        ratio = self.devicePixelRatioF()
        scale = (self.width(), self.height(), ratio)
        if self._scaled_background is None or self._background_scale != scale:
            self._scaled_background = self._background.scaled(
                QSize(math.ceil(self.width() * ratio), math.ceil(self.height() * ratio)),
                Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                Qt.TransformationMode.SmoothTransformation,
            )
            self._scaled_background.setDevicePixelRatio(ratio)
            self._background_scale = scale
        image = self._scaled_background
        logical_size = image.deviceIndependentSize()
        # Keep the artwork's authored blank left column behind the loading copy.
        # Narrow splash windows crop the robot side rather than the text zone.
        painter.drawPixmap(
            QPointF(
                0,
                (self.height() - logical_size.height()) / 2,
            ),
            image,
        )


class StartupSplash(QSplashScreen):
    """Play a branded intro and keep startup visible for at least three seconds."""

    def __init__(self, workspace: str) -> None:
        screen = target_screen()
        width = min(900, int(screen.availableGeometry().width() * 0.85))
        height = min(max(320, int(width * 9 / 16)), int(screen.availableGeometry().height() * 0.85))
        ratio = screen.devicePixelRatio()
        canvas = QPixmap(QSize(math.ceil(width * ratio), math.ceil(height * ratio)))
        canvas.setDevicePixelRatio(ratio)
        canvas.fill(QColor(theme_manager().theme.shell))
        super().__init__(screen, canvas)
        self.setAttribute(Qt.WidgetAttribute.WA_QuitOnClose, False)
        self.setObjectName("ModSimStartupSplash")
        self._shown_clock = QElapsedTimer()
        self._waiting_loop: QEventLoop | None = None
        self._cancelled = False
        self._handed_off = False
        self.panel = LoadingPanel(workspace, self, animated_logo=True)
        self.panel.setMinimumSize(0, 0)
        self._scroll = QScrollArea(self)
        self._scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._scroll.setWidgetResizable(True)
        self._scroll.setGeometry(self.rect())
        self._scroll.setWidget(self.panel)
        self._window_sizing = manage_window(
            self, "startup-splash", preferred=QSize(width, height), persist=False
        )

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)
        scroll = getattr(self, "_scroll", None)
        if scroll is not None:
            scroll.setGeometry(self.rect())

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        if not self._shown_clock.isValid():
            self._shown_clock.start()
        if isinstance(self.panel.logo, AnimatedBrandLogo):
            self.panel.logo.start()

    def _wait_until(self, elapsed_ms: int) -> bool:
        if self._cancelled:
            return False
        if not self._shown_clock.isValid():
            self.show()
        if self._shown_clock.elapsed() >= elapsed_ms:
            return True
        loop = QEventLoop(self)
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setTimerType(Qt.TimerType.PreciseTimer)

        def check_deadline() -> None:
            remaining = elapsed_ms - self._shown_clock.elapsed()
            if remaining > 0:
                timer.start(remaining)
            else:
                loop.quit()

        timer.timeout.connect(check_deadline)
        self._waiting_loop = loop
        try:
            timer.start(max(1, elapsed_ms - self._shown_clock.elapsed()))
            loop.exec()
        finally:
            timer.stop()
            # Release the Python closure before deferring destruction of its
            # QObject references, including on exceptional startup paths.
            timer.timeout.disconnect(check_deadline)
            timer.deleteLater()
            self._waiting_loop = None
            loop.deleteLater()
        if self._shown_clock.elapsed() < elapsed_ms:
            self._cancelled = True
        return not self._cancelled

    def play_intro(self) -> bool:
        """Animate the logo reveal before GUI-thread imports and construction.

        The intro is part of the three-second minimum. False indicates the
        user closed the splash or the application stopped during startup.
        """
        return self._wait_until(_SPLASH_INTRO_MS)

    def present(self, window: QWidget) -> bool:
        """Reveal a prepared window after the remaining minimum display time.

        Initialization time counts toward the minimum, so slow startup has no
        additional three-second tail. False leaves the prepared window hidden
        for its launcher to close after a cancelled startup.
        """
        if not self._wait_until(_SPLASH_MINIMUM_MS):
            return False
        window.show()
        self._handed_off = True
        super().finish(window)
        return True

    def mousePressEvent(self, event: QMouseEvent) -> None:
        # QSplashScreen normally hides on a click, bypassing the requested intro.
        event.accept()

    def closeEvent(self, event: QCloseEvent) -> None:
        if not self._handed_off:
            self._cancelled = True
        if isinstance(self.panel.logo, AnimatedBrandLogo):
            self.panel.logo.stop()
        if self._waiting_loop is not None:
            self._waiting_loop.quit()
        super().closeEvent(event)

    def set_stage(self, message: str) -> None:
        """Paint the current startup stage before the next initialization step."""
        self.panel.set_stage(message)
        self.repaint()
        QApplication.processEvents(QEventLoop.ProcessEventsFlag.ExcludeUserInputEvents)


class BusyDialog(QDialog):
    """Keep file/model work off the GUI thread while guarding document edits."""

    def __init__(self, title: str, parent: QWidget, detail: str = "") -> None:
        super().__init__(parent)
        self.setWindowTitle(f"ModSim Studio — {title}")
        self.setWindowModality(Qt.WindowModality.WindowModal)
        self.setWindowFlag(Qt.WindowType.WindowCloseButtonHint, False)
        self._running = False
        self._show_feedback = parent.isVisible()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.panel = LoadingPanel("Robot Pack Builder", self)
        self.panel.set_stage(title, detail)
        scroll = QScrollArea()
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.panel)
        layout.addWidget(scroll)
        self._window_sizing = manage_window(
            self, "operation-progress", preferred=QSize(720, 405), persist=False
        )

    def __enter__(self) -> BusyDialog:
        self._running = True
        if self._show_feedback:
            self.show()
            self.repaint()
        return self

    def __exit__(
        self,
        _exception_type: type[BaseException] | None,
        _exception: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self._running = False
        self.panel.set_busy(False)
        self.done(0)
        self.deleteLater()

    def set_stage(self, message: str, detail: str = "") -> None:
        """Update feedback before performing the next GUI-owned stage."""
        self.panel.set_stage(message, detail)
        self.repaint()
        QApplication.processEvents(QEventLoop.ProcessEventsFlag.ExcludeUserInputEvents)

    def run(self, operation: Callable[[], _Result]) -> _Result:
        """Run Qt-free work in a worker and propagate its result or exception."""
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="modsim-document") as executor:
            future = executor.submit(operation)
            loop = QEventLoop(self)
            timer = QTimer(self)
            timer.setInterval(20)
            timer.timeout.connect(lambda: loop.quit() if future.done() else None)
            try:
                timer.start()
                if not future.done():
                    loop.exec()
                return future.result()
            finally:
                timer.stop()
                timer.deleteLater()
                loop.deleteLater()

    def reject(self) -> None:
        """Do not interrupt an in-flight filesystem operation with Escape."""
        if not self._running:
            super().reject()

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._running:
            event.ignore()
        else:
            super().closeEvent(event)
