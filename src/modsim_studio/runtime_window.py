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
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from modsim.runtime import RuntimeInspectorConfig, RuntimeInspectorFrame
from modsim.runtime.reconfiguration import ReconfigurationPhase
from modsim_studio.appearance import theme_manager
from modsim_studio.chrome import StudioHeader
from modsim_studio.runtime_events import RuntimeEventLogWidget
from modsim_studio.runtime_graph import TopologyGraphWidget
from modsim_studio.runtime_planning import PlanningWorkspace
from modsim_studio.runtime_presenter import RuntimeInspectorPresenter, RuntimePresentation
from modsim_studio.runtime_process import (
    RuntimeInspectorProcessController,
    RuntimeInspectorThreadController,
)
from modsim_studio.visual_colors import CategoryColors

_LOGGER = logging.getLogger("modsim.runtime_inspector")


class RuntimeInspectorWindow(QMainWindow):
    """Display one automatically started Runtime Inspector demonstration."""

    def __init__(self, config: RuntimeInspectorConfig) -> None:
        super().__init__()
        self._config = config
        self._presenter = RuntimeInspectorPresenter()
        self._started = False
        self._close_pending = False
        self._last_error: str | None = None
        self._latest_frame: RuntimeInspectorFrame | None = None
        self._finished = False
        self._shutdown_requested = False
        self._paused = False
        self._playback_available = False
        self._pause_pending = False

        self.setWindowTitle(f"ModSim Runtime Inspector — {config.pack_path.name}")
        self.resize(1280, 860)
        self.header = StudioHeader(self, "Runtime Inspector")
        self.addToolBar(self.header)
        self.run_state = QLabel("Starting simulation…")
        self.run_state.setWordWrap(True)
        self.run_state.setAccessibleName("Simulation state")
        self.status_label = QLabel("Preparing runtime…")
        self.status_label.setObjectName("SectionTitle")
        self.status_label.setWordWrap(True)
        self.source_label = QLabel(str(config.pack_path))
        self.source_label.setObjectName("Muted")
        self.source_label.setWordWrap(True)
        self.source_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.speed_label = QLabel("Target speed: 1x")
        self.speed_label.setObjectName("Muted")
        self.pause_button = QPushButton("Pause")
        self.pause_button.setProperty("primary", True)
        self.pause_button.setEnabled(False)
        self.pause_button.setToolTip(
            "Pause/resume the authoritative simulation; Space in the 3D window does the same."
        )
        self.pause_button.clicked.connect(self._toggle_playback)
        self.labels_button = QPushButton("Labels: On")
        self.labels_button.setToolTip("Show module names in the live and target topology views")
        self.labels_button.setCheckable(True)
        self.labels_button.setChecked(True)
        self.labels_button.setEnabled(False)
        self.labels_button.toggled.connect(self._set_labels_visible)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setProperty("danger", True)
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.request_stop)

        header = QWidget()
        header.setObjectName("Panel")
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(14, 12, 14, 12)
        status_row = QHBoxLayout()
        status_row.addWidget(self.status_label, 1)
        status_row.addWidget(self.pause_button)
        status_row.addWidget(self.labels_button)
        status_row.addWidget(self.stop_button)
        header_layout.addWidget(self.run_state)
        header_layout.addLayout(status_row)
        header_layout.addWidget(self.speed_label)
        header_layout.addWidget(self.source_label)

        category_colors = CategoryColors()
        self.graph = TopologyGraphWidget(category_colors=category_colors)
        self.target_graph = TopologyGraphWidget(category_colors=category_colors)
        self.live_title = QLabel("Live topology")
        self.live_title.setObjectName("SectionTitle")
        live_panel = QWidget()
        live_layout = QVBoxLayout(live_panel)
        live_layout.addWidget(self.live_title)
        live_layout.addWidget(self.graph, 1)
        self.target_title = QLabel("Target topology")
        self.target_title.setObjectName("SectionTitle")
        self.target_panel = QWidget()
        target_layout = QVBoxLayout(self.target_panel)
        target_layout.addWidget(self.target_title)
        target_layout.addWidget(self.target_graph, 1)
        self.target_panel.hide()
        topology_split = QSplitter(Qt.Orientation.Horizontal)
        topology_split.addWidget(live_panel)
        topology_split.addWidget(self.target_panel)
        topology_split.setSizes([580, 580])
        self.planning = PlanningWorkspace()
        self.workspace_tabs = QTabWidget()
        self.workspace_tabs.addTab(topology_split, "Runtime state")
        self.workspace_tabs.addTab(self.planning, "Planning")
        self.workspace_tabs.setTabVisible(1, False)
        self.events = RuntimeEventLogWidget()
        events_panel = QWidget()
        events_layout = QVBoxLayout(events_panel)
        events_layout.addWidget(
            QLabel("Simulation events · committed changes and docking diagnostics")
        )
        events_layout.addWidget(self.events, 1)
        self.decisions_panel = QWidget()
        decisions_layout = QVBoxLayout(self.decisions_panel)
        decisions_layout.addWidget(
            QLabel("Planner decisions · searches, waypoints, support transfer, and completion")
        )
        decisions_layout.addWidget(self.planning.decisions, 1)
        self.decisions_panel.hide()
        log_split = QSplitter(Qt.Orientation.Vertical)
        log_split.addWidget(events_panel)
        log_split.addWidget(self.decisions_panel)
        log_split.setSizes([400, 250])
        self.workspace_tabs.addTab(log_split, "Event log")

        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(10, 10, 10, 10)
        central_layout.addWidget(header)
        central_layout.addWidget(self.workspace_tabs, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Runtime has not started")
        theme_manager().changed.connect(self._update_run_state)
        self._controller = (
            RuntimeInspectorProcessController(config, self)
            if config.viewer_enabled
            else RuntimeInspectorThreadController(config, self)
        )
        self._controller.playback_changed.connect(self._receive_playback_state)
        self._controller.frame_ready.connect(self._receive_frame)
        self._controller.status_changed.connect(self._receive_status)
        self._controller.failed.connect(self._runtime_failed)
        self._controller.finished.connect(self._runtime_finished)
        self.graph.entity_selected.connect(self._select_entity)
        self.target_graph.entity_selected.connect(self._select_target_entity)
        self._update_run_state()

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
        self._shutdown_requested = True
        self._update_run_state()
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
        try:
            presentation = self._presenter.apply_frame(value)
        except Exception as error:
            self._presentation_failed(error)
            return
        self._latest_frame = self._presenter.frame
        self._apply_presentation(presentation)
        if self._latest_frame is not None and self._latest_frame.planning is not None:
            self.workspace_tabs.setTabVisible(1, True)
            self.decisions_panel.show()
            self.planning.set_frame(self._latest_frame)
        self._update_run_state()

    @Slot(str)
    def _receive_status(self, message: str) -> None:
        self.status_label.setText(message)
        self.statusBar().showMessage(message)

    @Slot(str)
    def _runtime_failed(self, message: str) -> None:
        self._last_error = message
        self._update_run_state()
        self.stop_button.setEnabled(False)
        self.status_label.setText(message)
        self.statusBar().showMessage("Runtime failed")
        _LOGGER.error("%s", message)
        if not self._close_pending:
            QMessageBox.critical(self, "Runtime Inspector", message)

    @Slot()
    def _runtime_finished(self) -> None:
        self._finished = True
        self._update_run_state()
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
        self.labels_button.setEnabled(True)
        target = self._presenter.target_presentation()
        if target is not None:
            self.target_panel.show()
            self.target_graph.set_presentation(target)
            count = sum(edge.state == "matched" for edge in target.edges)
            self.target_title.setText(
                f"Target topology · {count}/{len(target.edges)} connections reached"
            )
        self.events.set_events(presentation.events)
        frame = self._latest_frame
        if frame is not None and frame.planning is not None:
            self.status_label.setText(
                f"SMORES supported 3D handoff · {frame.planning.phase.capitalize()} · "
                f"{frame.metrics.time_s:.2f} s · "
                f"{frame.metrics.connection_count_active} connections"
            )
            self.status_label.setToolTip(presentation.status_text)
        else:
            self.status_label.setText(presentation.status_text)
        self.source_label.setText(presentation.source_text)
        selection = presentation.selection
        if selection is None:
            self.statusBar().showMessage(presentation.status_text)
        else:
            self.statusBar().showMessage(
                f"Selected {selection.kind}: {selection.entity_id} | {presentation.status_text}"
            )

    @Slot(str, str)
    def _select_target_entity(self, kind: str, entity_id: str) -> None:
        if kind == "node":
            self._select_entity(kind, entity_id)

    @Slot(bool)
    def _set_labels_visible(self, visible: bool) -> None:
        self.labels_button.setText("Labels: On" if visible else "Labels: Off")
        if self._presenter.frame is not None:
            self._apply_presentation(self._presenter.set_show_labels(visible))

    @Slot()
    def _toggle_playback(self) -> None:
        if self._pause_pending or not self._playback_available:
            return
        self._pause_pending = True
        self.pause_button.setEnabled(False)
        self._controller.set_paused(not self._paused)

    @Slot(bool)
    def _receive_playback_state(self, paused: bool) -> None:
        self._paused = paused
        self._pause_pending = False
        self._playback_available = True
        self._update_run_state()

    def _update_run_state(self) -> None:
        theme = theme_manager().theme
        frame = self._latest_frame
        scenario = None if frame is None else frame.scenario
        phase = None if scenario is None else scenario.phase
        terminal = phase in {
            ReconfigurationPhase.COMPLETE,
            ReconfigurationPhase.FAILED,
            ReconfigurationPhase.STOPPED,
            ReconfigurationPhase.TIMED_OUT,
        }
        time_limit = frame is not None and frame.metrics.time_s >= self._config.duration_s - 1e-9
        ended = terminal or time_limit or self._finished or self._last_error is not None
        color = theme.accent
        if self._last_error is not None or phase is ReconfigurationPhase.FAILED:
            title, detail = (
                "Simulation failed",
                self._last_error or (scenario.detail if scenario else ""),
            )
            color = theme.danger
        elif phase is ReconfigurationPhase.COMPLETE:
            title = (
                "Target reached · Simulation complete"
                if frame and frame.planning
                else "Simulation complete"
            )
            detail = (
                f"Finished at {scenario.time_s:.3f} s. The final state is frozen for inspection."
                if scenario
                else ""
            )
            color = theme.success
        elif phase is ReconfigurationPhase.STOPPED:
            title, detail = "Simulation stopped", "Target not verified; results remain available."
            color = theme.warning
        elif phase is ReconfigurationPhase.TIMED_OUT or time_limit:
            title, detail = (
                "Time limit reached · Target not reached",
                "Results remain available for inspection.",
            )
            color = theme.warning
        elif self._shutdown_requested:
            title, detail = "Stopping simulation…", "Releasing simulation resources."
            color = theme.warning
        elif self._finished:
            title, detail = "Simulation stopped", "Results remain available for inspection."
            color = theme.warning
        elif self._paused:
            title, detail = "Simulation paused", "Resume to continue."
            color = theme.warning
        elif frame is None:
            title, detail = "Starting simulation…", ""
        else:
            title, detail = "Simulation running", ""
        self.run_state.setText(title + ("\n" + detail if detail else ""))
        self.run_state.setStyleSheet(
            f"QLabel {{ color: {color}; background: {theme.raised}; "
            f"border-left: 4px solid {color}; padding: 8px 12px; font-weight: 600; }}"
        )
        self.pause_button.setText("Finished" if ended else "Resume" if self._paused else "Pause")
        self.pause_button.setEnabled(
            self._playback_available
            and not ended
            and not self._pause_pending
            and not self._shutdown_requested
        )
        self.speed_label.setText(
            "Simulation frozen · results available for inspection"
            if ended
            else "Simulation paused"
            if self._paused
            else "Target speed: 1x"
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
