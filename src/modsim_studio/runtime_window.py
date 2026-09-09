# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
"""Standalone Qt window for immutable ModSim runtime inspection."""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from modsim.runtime import RuntimeInspectorConfig, RuntimeInspectorFrame
from modsim_studio.runtime_events import RuntimeEventLogWidget
from modsim_studio.runtime_graph import TopologyGraphWidget
from modsim_studio.runtime_lattice import CubicLatticeWidget
from modsim_studio.runtime_presenter import (
    CubicLatticePresentation,
    RuntimeInspectorPresentation,
    RuntimeInspectorPresenter,
)
from modsim_studio.runtime_process import (
    RuntimeInspectorProcessController,
    RuntimeInspectorThreadController,
)

_LOGGER = logging.getLogger("modsim.runtime_inspector")
_FRAME_BATCH_DELAY_MS = 8


class RuntimeInspectorWindow(QMainWindow):
    """Display one automatically started Runtime Inspector demonstration."""

    def __init__(self, config: RuntimeInspectorConfig) -> None:
        super().__init__()
        self._config = config
        self._presenter = RuntimeInspectorPresenter()
        self._started = False
        self._close_pending = False
        self._last_error: str | None = None
        self._pending_frames: list[RuntimeInspectorFrame] = []
        self._frame_batch_timer = QTimer(self)
        self._frame_batch_timer.setSingleShot(True)
        self._frame_batch_timer.setInterval(_FRAME_BATCH_DELAY_MS)
        self._frame_batch_timer.timeout.connect(self._flush_pending_frames)

        self.setWindowTitle(f"ModSim Runtime Inspector — {config.pack_path.name}")
        self.resize(1180, 820)

        self.status_label = QLabel("Preparing runtime…")
        self.status_label.setWordWrap(True)
        self.speed_label = QLabel(f"Target speed: {config.real_time_factor:g}x")
        self.speed_label.setToolTip(
            "Wall-clock playback target; simulated motor speeds and physics are unchanged."
        )
        self.source_label = QLabel(str(config.pack_path))
        self.source_label.setWordWrap(True)
        self.source_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.request_stop)
        self.labels_button = QPushButton("Labels: On")
        self.labels_button.setCheckable(True)
        self.labels_button.setChecked(True)
        self.labels_button.setEnabled(False)
        self.labels_button.setToolTip(
            "Show or hide module names and lattice cell coordinates in the Runtime Inspector"
        )
        self.labels_button.setAccessibleName("Show runtime module labels")
        self.labels_button.toggled.connect(self._set_labels_visible)

        header = QWidget()
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(8, 8, 8, 4)
        status_row = QHBoxLayout()
        status_row.addWidget(self.status_label, 1)
        status_row.addWidget(self.labels_button)
        status_row.addWidget(self.stop_button)
        header_layout.addLayout(status_row)
        header_layout.addWidget(self.speed_label)
        header_layout.addWidget(self.source_label)

        self.graph = TopologyGraphWidget()
        self.lattice = CubicLatticeWidget()
        self.view_stack = QStackedWidget()
        self.view_stack.addWidget(self.graph)
        self.view_stack.addWidget(self.lattice)
        self.events = RuntimeEventLogWidget()
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(self.view_stack)
        splitter.addWidget(self.events)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([520, 250])

        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.addWidget(header)
        central_layout.addWidget(splitter, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Runtime has not started")

        self._controller = (
            RuntimeInspectorProcessController(config, self)
            if config.viewer_enabled
            else RuntimeInspectorThreadController(config, self)
        )
        self._controller.frame_ready.connect(self._receive_frame)
        self._controller.status_changed.connect(self._receive_status)
        self._controller.failed.connect(self._runtime_failed)
        self._controller.finished.connect(self._runtime_finished)
        self.graph.entity_selected.connect(self._select_entity)
        self.lattice.entity_selected.connect(self._select_entity)
        self.lattice.projection_changed.connect(self._set_lattice_projection)
        self.lattice.layer_changed.connect(self._set_lattice_layer)
        self.lattice.snap_cells_changed.connect(self._set_snap_cells_visible)
        self.lattice.orientation_axes_changed.connect(self._set_orientation_axes_visible)
        self.lattice.orbit_requested.connect(self._orbit_lattice)

        QTimer.singleShot(0, self.start)

    @property
    def last_error(self) -> str | None:
        """Return the worker or presentation failure shown by this window."""
        return self._last_error

    @property
    def exit_code(self) -> int:
        """Return a process-friendly result after the window closes."""
        return 1 if self._last_error is not None else 0

    @Slot()
    def start(self) -> None:
        """Start the configured scenario once."""
        if self._started:
            return
        self._started = True
        self.stop_button.setEnabled(True)
        self.statusBar().showMessage("Starting runtime…")
        self._controller.start()

    @Slot()
    def request_stop(self) -> None:
        """Request cooperative interruption without blocking the GUI thread."""
        if not self._controller.is_running():
            return
        self.stop_button.setEnabled(False)
        self.status_label.setText("Stopping runtime…")
        self.statusBar().showMessage("Stopping runtime…")
        self._controller.request_interruption()

    def stop_and_wait(self, timeout_ms: int = 10_000) -> bool:
        """Stop the worker and wait for backend shutdown during app teardown."""
        return self._controller.stop_and_wait(timeout_ms)

    @Slot(object)
    def _receive_frame(self, value: object) -> None:
        if not isinstance(value, RuntimeInspectorFrame):
            self._presentation_failed(
                TypeError(f"worker published unsupported frame type {type(value).__name__}")
            )
            return
        self._pending_frames.append(value)
        if not self._frame_batch_timer.isActive():
            self._frame_batch_timer.start()

    @Slot()
    def _flush_pending_frames(self) -> None:
        if not self._pending_frames:
            return
        frames = tuple(self._pending_frames)
        self._pending_frames.clear()
        try:
            presentation = self._presenter.apply_frames(frames)
        except Exception as error:
            self._presentation_failed(error)
            return
        self._apply_presentation(presentation)

    @Slot(str)
    def _receive_status(self, message: str) -> None:
        self.status_label.setText(message)
        self.statusBar().showMessage(message)

    @Slot(str)
    def _runtime_failed(self, message: str) -> None:
        self._last_error = message
        self.stop_button.setEnabled(False)
        self.status_label.setText(message)
        self.statusBar().showMessage("Runtime failed")
        _LOGGER.error("%s", message)
        if not self._close_pending:
            QMessageBox.critical(self, "Runtime Inspector", message)

    @Slot()
    def _runtime_finished(self) -> None:
        if self._pending_frames:
            self._frame_batch_timer.stop()
            self._flush_pending_frames()
        self.stop_button.setEnabled(False)
        if self._last_error is None:
            self.statusBar().showMessage("Runtime finished; results remain available")
        if self._close_pending:
            QTimer.singleShot(0, self.close)

    @Slot(str, str)
    def _select_entity(self, kind: str, entity_id: str) -> None:
        try:
            presentation = self._presenter.select(kind, entity_id)
        except (KeyError, ValueError) as error:
            _LOGGER.warning("Could not select runtime graph entity: %s", error)
            return
        self._apply_presentation(presentation)

    @Slot(str)
    def _set_lattice_projection(self, projection: str) -> None:
        try:
            presentation = self._presenter.set_lattice_projection(projection)
        except (KeyError, TypeError, ValueError) as error:
            _LOGGER.warning("Could not change lattice projection: %s", error)
            return
        self._apply_presentation(presentation)

    @Slot(object)
    def _set_lattice_layer(self, layer: object) -> None:
        if layer is not None and not isinstance(layer, int):
            _LOGGER.warning("Could not change lattice layer: expected an integer or All")
            return
        try:
            presentation = self._presenter.set_lattice_layer(layer)
        except (KeyError, TypeError, ValueError) as error:
            _LOGGER.warning("Could not change lattice layer: %s", error)
            return
        self._apply_presentation(presentation)

    @Slot(bool)
    def _set_snap_cells_visible(self, visible: bool) -> None:
        try:
            presentation = self._presenter.set_lattice_overlays(show_snap_cells=visible)
        except (TypeError, ValueError) as error:
            _LOGGER.warning("Could not change lattice snap-cell overlay: %s", error)
            return
        self._apply_presentation(presentation)

    @Slot(bool)
    def _set_orientation_axes_visible(self, visible: bool) -> None:
        try:
            presentation = self._presenter.set_lattice_overlays(show_orientation_axes=visible)
        except (TypeError, ValueError) as error:
            _LOGGER.warning("Could not change lattice orientation overlay: %s", error)
            return
        self._apply_presentation(presentation)

    @Slot(bool)
    def _set_labels_visible(self, visible: bool) -> None:
        try:
            presentation = self._presenter.set_labels_visible(visible)
        except (TypeError, ValueError) as error:
            _LOGGER.warning("Could not change runtime module labels: %s", error)
            return
        self._apply_presentation(presentation)

    @Slot(float, float)
    def _orbit_lattice(self, azimuth_delta_rad: float, elevation_delta_rad: float) -> None:
        try:
            presentation = self._presenter.orbit_lattice(
                azimuth_delta_rad,
                elevation_delta_rad,
            )
        except (TypeError, ValueError) as error:
            _LOGGER.warning("Could not orbit the lattice view: %s", error)
            return
        self._apply_presentation(presentation)

    def _apply_presentation(self, presentation: RuntimeInspectorPresentation) -> None:
        self.labels_button.blockSignals(True)
        self.labels_button.setChecked(presentation.show_labels)
        self.labels_button.setText("Labels: On" if presentation.show_labels else "Labels: Off")
        self.labels_button.blockSignals(False)
        self.labels_button.setEnabled(True)
        if isinstance(presentation, CubicLatticePresentation):
            self.lattice.set_presentation(presentation)
            self.view_stack.setCurrentWidget(self.lattice)
        else:
            self.graph.set_presentation(presentation)
            self.view_stack.setCurrentWidget(self.graph)
        self.events.set_events(presentation.events)
        self.status_label.setText(presentation.status_text)
        self.source_label.setText(presentation.source_text)
        selection = presentation.selection
        if selection is None:
            self.statusBar().showMessage(presentation.status_text)
        else:
            self.statusBar().showMessage(
                f"Selected {selection.kind}: {selection.entity_id} | {presentation.status_text}"
            )

    def _presentation_failed(self, error: Exception) -> None:
        if error.__traceback__ is None:
            _LOGGER.error("Runtime Inspector could not present a frame: %s", error)
        else:
            _LOGGER.error(
                "Runtime Inspector could not present a frame",
                exc_info=(type(error), error, error.__traceback__),
            )
        self.request_stop()
        self._runtime_failed(f"Runtime Inspector could not display an update: {error}")

    def closeEvent(self, event: QCloseEvent) -> None:
        """Keep Qt objects alive until the worker has shut its backend down."""
        if self._controller.is_running():
            self._close_pending = True
            self.request_stop()
            event.ignore()
            return
        event.accept()


__all__ = ["RuntimeInspectorWindow"]
