# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
"""Standalone Qt window for immutable ModSim runtime inspection."""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QThread, QTimer, Slot
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from modsim.runtime import RuntimeInspectorFrame
from modsim_studio.runtime_events import RuntimeEventLogWidget
from modsim_studio.runtime_graph import TopologyGraphWidget
from modsim_studio.runtime_presenter import RuntimeInspectorPresenter, RuntimePresentation
from modsim_studio.runtime_worker import RuntimeInspectorConfig, RuntimeInspectorWorker

_LOGGER = logging.getLogger("modsim.runtime_inspector")


class RuntimeInspectorWindow(QMainWindow):
    """Display one automatically started two-module runtime scenario."""

    def __init__(self, config: RuntimeInspectorConfig) -> None:
        super().__init__()
        self._config = config
        self._presenter = RuntimeInspectorPresenter()
        self._started = False
        self._close_pending = False
        self._last_error: str | None = None

        self.setWindowTitle(f"ModSim Runtime Inspector — {config.pack_path.name}")
        self.resize(1180, 820)

        self.status_label = QLabel("Preparing runtime…")
        self.status_label.setWordWrap(True)
        self.source_label = QLabel(str(config.pack_path))
        self.source_label.setWordWrap(True)
        self.source_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.request_stop)

        header = QWidget()
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(8, 8, 8, 4)
        status_row = QHBoxLayout()
        status_row.addWidget(self.status_label, 1)
        status_row.addWidget(self.stop_button)
        header_layout.addLayout(status_row)
        header_layout.addWidget(self.source_label)

        self.graph = TopologyGraphWidget()
        self.events = RuntimeEventLogWidget()
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(self.graph)
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

        self._thread = QThread(self)
        self._thread.setObjectName("ModSimRuntimeInspector")
        self._worker = RuntimeInspectorWorker(config)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.frame_ready.connect(self._receive_frame)
        self._worker.status_changed.connect(self._receive_status)
        self._worker.failed.connect(self._runtime_failed)
        self._worker.finished.connect(self._worker.deleteLater)
        self._worker.finished.connect(self._thread.quit)
        self._thread.finished.connect(self._runtime_finished)
        self.graph.entity_selected.connect(self._select_entity)

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
        self._thread.start()

    @Slot()
    def request_stop(self) -> None:
        """Request cooperative interruption without blocking the GUI thread."""
        if not self._thread.isRunning():
            return
        self.stop_button.setEnabled(False)
        self.status_label.setText("Stopping runtime…")
        self.statusBar().showMessage("Stopping runtime…")
        self._worker.request_interruption()
        self._thread.requestInterruption()

    def stop_and_wait(self, timeout_ms: int = 10_000) -> bool:
        """Stop the worker and wait for backend shutdown during app teardown."""
        if not self._thread.isRunning():
            return True
        self._worker.request_interruption()
        self._thread.requestInterruption()
        self._thread.quit()
        return self._thread.wait(timeout_ms)

    @Slot(object)
    def _receive_frame(self, value: object) -> None:
        if not isinstance(value, RuntimeInspectorFrame):
            self._presentation_failed(
                TypeError(f"worker published unsupported frame type {type(value).__name__}")
            )
            return
        try:
            presentation = self._presenter.apply_frame(value)
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

    def _apply_presentation(self, presentation: RuntimePresentation) -> None:
        self.graph.set_presentation(presentation)
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
        if self._thread.isRunning():
            self._close_pending = True
            self.request_stop()
            event.ignore()
            return
        event.accept()


__all__ = ["RuntimeInspectorWindow"]
