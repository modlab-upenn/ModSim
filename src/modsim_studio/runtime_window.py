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
from modsim_studio.runtime_lattice import CubicLatticeWidget
from modsim_studio.runtime_lattice_presenter import CubicLatticeProjector
from modsim_studio.runtime_mblocks_planning import LatticePlanningWorkspace, target_lattice_view
from modsim_studio.runtime_planning import PlanningWorkspace
from modsim_studio.runtime_presenter import (
    CubicLatticePresentation,
    RuntimeInspectorPresentation,
    RuntimeInspectorPresenter,
)
from modsim_studio.runtime_process import (
    RuntimeInspectorProcessController,
    RuntimeInspectorThreadController,
)
from modsim_studio.visual_colors import CategoryColors

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
        self._playback_paused = False
        self._playback_pending: bool | None = None
        self._playback_available = False
        self._shutdown_requested = False
        self._execution_ended = False
        self._finished = False
        self._latest_frame: RuntimeInspectorFrame | None = None
        self._pending_frames: list[RuntimeInspectorFrame] = []
        self._frame_batch_timer = QTimer(self)
        self._frame_batch_timer.setSingleShot(True)
        self._frame_batch_timer.setInterval(_FRAME_BATCH_DELAY_MS)
        self._frame_batch_timer.timeout.connect(self._flush_pending_frames)

        self.setWindowTitle(f"ModSim Runtime Inspector — {config.pack_path.name}")
        self.resize(1180, 820)
        self.header = StudioHeader(self, "Runtime Inspector")
        self.addToolBar(self.header)

        self.run_state = QLabel("Starting simulation…")
        self.run_state.setWordWrap(True)
        self.run_state.setAccessibleName("Simulation state")
        theme_manager().changed.connect(self._update_run_state)
        self.status_label = QLabel("Preparing runtime…")
        self.status_label.setObjectName("SectionTitle")
        self.status_label.setWordWrap(True)
        self.speed_label = QLabel(f"Target speed: {config.real_time_factor:g}x")
        self.speed_label.setObjectName("Muted")
        self.speed_label.setToolTip(
            "Wall-clock playback target; simulated motor speeds and physics are unchanged."
        )
        self.source_label = QLabel(str(config.pack_path))
        self.source_label.setObjectName("Muted")
        self.source_label.setWordWrap(True)
        self.source_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setProperty("danger", True)
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.request_stop)
        self.pause_button = QPushButton("Pause")
        self.pause_button.setProperty("primary", True)
        self.pause_button.setEnabled(False)
        self.pause_button.setToolTip(
            "Pause at the next simulation step boundary. Space in the MuJoCo window "
            "controls the same runtime."
        )
        self.pause_button.clicked.connect(self._toggle_playback)
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
        self.lattice = CubicLatticeWidget(category_colors=category_colors)
        self.view_stack = QStackedWidget()
        self.view_stack.addWidget(self.graph)
        self.view_stack.addWidget(self.lattice)
        self.planning = PlanningWorkspace()
        self.lattice_planning = LatticePlanningWorkspace()
        self.planning_stack = QStackedWidget()
        self.planning_stack.addWidget(self.planning)
        self.planning_stack.addWidget(self.lattice_planning)
        self.target_graph = TopologyGraphWidget(category_colors=category_colors)
        self.target_lattice = CubicLatticeWidget(category_colors=category_colors)
        self.target_lattice.snap_cells_checkbox.hide()
        self.target_lattice.orientation_axes_checkbox.hide()
        self._target_projector = CubicLatticeProjector()
        self.target_view_stack = QStackedWidget()
        self.target_view_stack.addWidget(self.target_graph)
        self.target_view_stack.addWidget(self.target_lattice)
        self.live_title = QLabel("Live topology")
        self.live_title.setObjectName("SectionTitle")
        live_panel = QWidget()
        live_layout = QVBoxLayout(live_panel)
        live_layout.addWidget(self.live_title)
        live_layout.addWidget(self.view_stack, 1)
        self.target_panel = QWidget()
        target_layout = QVBoxLayout(self.target_panel)
        self.target_title = QLabel("Target topology")
        self.target_title.setObjectName("SectionTitle")
        target_layout.addWidget(self.target_title)
        target_layout.addWidget(self.target_view_stack, 1)
        self.target_panel.hide()
        topology_split = QSplitter(Qt.Orientation.Horizontal)
        topology_split.addWidget(live_panel)
        topology_split.addWidget(self.target_panel)
        topology_split.setSizes([580, 580])
        self.workspace_tabs = QTabWidget()
        self.workspace_tabs.addTab(topology_split, "Runtime state")
        self.workspace_tabs.addTab(self.planning_stack, "Planning")
        self.workspace_tabs.setTabVisible(1, False)
        self._planning_visible = False
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
            QLabel("Planner decisions · assignments, routes, retries, and completion")
        )
        self.decisions_stack = QStackedWidget()
        self.decisions_stack.addWidget(self.planning.decisions)
        self.decisions_stack.addWidget(self.lattice_planning.decisions)
        decisions_layout.addWidget(self.decisions_stack, 1)
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

        self._controller = (
            RuntimeInspectorProcessController(config, self)
            if config.viewer_enabled
            else RuntimeInspectorThreadController(config, self)
        )
        self._controller.frame_ready.connect(self._receive_frame)
        self._controller.status_changed.connect(self._receive_status)
        self._controller.playback_changed.connect(self._receive_playback_state)
        self._controller.failed.connect(self._runtime_failed)
        self._controller.finished.connect(self._runtime_finished)
        self.graph.entity_selected.connect(self._select_entity)
        self.target_graph.entity_selected.connect(self._select_target_entity)
        self._update_run_state()
        self.lattice.entity_selected.connect(self._select_entity)
        self.lattice.projection_changed.connect(self._set_lattice_projection)
        self.target_lattice.projection_changed.connect(self._set_lattice_projection)
        self.target_lattice.orbit_requested.connect(self._orbit_lattice)
        self.target_lattice.layer_changed.connect(self._set_lattice_layer)
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
        self._shutdown_requested = True
        self.stop_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.status_label.setText("Stopping runtime…")
        self.statusBar().showMessage("Stopping runtime…")
        self._update_run_state()
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
        frame = self._presenter.frame
        if frame is None:
            return
        self._latest_frame = frame
        self._update_run_state()
        if frame.planning is not None:
            self.planning.set_frame(frame)
            self.planning_stack.setCurrentWidget(self.planning)
            self.decisions_stack.setCurrentWidget(self.planning.decisions)
        elif frame.lattice_planning is not None:
            self.lattice_planning.set_frame(frame)
            self.planning_stack.setCurrentWidget(self.lattice_planning)
            self.decisions_stack.setCurrentWidget(self.lattice_planning.decisions)
        if frame.planning is not None or frame.lattice_planning is not None:
            self.decisions_panel.show()
            if not self._planning_visible:
                self.workspace_tabs.setTabVisible(1, True)
                self._planning_visible = True

    @Slot(str)
    def _receive_status(self, message: str) -> None:
        self.status_label.setText(message)
        self.statusBar().showMessage(message)

    @Slot()
    def _toggle_playback(self) -> None:
        if (
            not self._playback_available
            or self._playback_pending is not None
            or not self._controller.is_running()
            or self._shutdown_requested
            or self._execution_ended
        ):
            return
        requested = not self._playback_paused
        self._playback_pending = requested
        self.pause_button.setEnabled(False)
        self.pause_button.setText("Pausing…" if requested else "Resuming…")
        self.statusBar().showMessage(self.pause_button.text())
        self._controller.set_paused(requested)

    @Slot(bool)
    def _receive_playback_state(self, paused: bool) -> None:
        self._playback_paused = paused
        self._playback_pending = None
        self._playback_available = True
        self.pause_button.setText("Resume" if paused else "Pause")
        self.pause_button.setEnabled(
            self._controller.is_running()
            and not self._close_pending
            and not self._shutdown_requested
            and not self._execution_ended
        )
        self.speed_label.setText(
            f"Target speed: {self._config.real_time_factor:g}x" + (" · Paused" if paused else "")
        )
        state = "paused" if paused else "running"
        self.statusBar().showMessage(f"Runtime {state}")
        self._update_run_state()
        _LOGGER.info("Runtime playback %s", state)

    @Slot(str)
    def _runtime_failed(self, message: str) -> None:
        self._last_error = message
        self._shutdown_requested = True
        self.stop_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.status_label.setText(message)
        self.statusBar().showMessage("Runtime failed")
        self._update_run_state()
        _LOGGER.error("%s", message)
        if not self._close_pending:
            QMessageBox.critical(self, "Runtime Inspector", message)

    @Slot()
    def _runtime_finished(self) -> None:
        self._shutdown_requested = True
        if self._pending_frames:
            self._frame_batch_timer.stop()
            self._flush_pending_frames()
        self.stop_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self._finished = True
        self._update_run_state()
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
            self.live_title.setText("Live lattice")
            self.lattice.set_presentation(presentation)
            self.view_stack.setCurrentWidget(self.lattice)
        else:
            self.live_title.setText("Live topology")
            self.graph.set_presentation(presentation)
            self.view_stack.setCurrentWidget(self.graph)
        target = self._presenter.target_presentation()
        if target is not None:
            self.target_panel.show()
            self.target_view_stack.setCurrentWidget(self.target_graph)
            self.target_graph.set_presentation(target)
            count = sum(edge.state == "matched" for edge in target.edges)
            self.target_title.setText(
                f"Target topology · {count}/{len(target.edges)} connections reached"
            )
        frame = self._presenter.frame
        if frame is not None and frame.lattice_planning is not None:
            target_view = target_lattice_view(frame)
            if target_view is not None and isinstance(presentation, CubicLatticePresentation):
                geometry = self._target_projector.project(
                    target_view,
                    projection=presentation.geometry.projection,
                    layer_z=presentation.geometry.layer_z,
                    camera=presentation.geometry.camera,
                    show_snap_cells=False,
                    show_orientation_axes=False,
                )
                self.target_lattice.set_presentation(
                    CubicLatticePresentation(
                        geometry=geometry,
                        events=(),
                        selection=None,
                        source_text=(
                            "Planner intent · interchangeable modules · assembly-relative frame"
                        ),
                        status_text="Target shape",
                        show_snap_cells=False,
                        show_orientation_axes=False,
                        show_labels=presentation.show_labels,
                    )
                )
                self.target_lattice.legend.set_entries(
                    (),
                    "These cubes are requested target cells, not live modules or snap diagnostics. "
                    "Any module may fill a cell. Projection and orbit follow the live lattice.",
                )
                self.target_view_stack.setCurrentWidget(self.target_lattice)
                self.target_panel.show()
                self.target_title.setText(f"Target lattice · {frame.lattice_planning.goal.id}")
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

    @Slot(str, str)
    def _select_target_entity(self, kind: str, entity_id: str) -> None:
        if kind == "node":
            self._select_entity(kind, entity_id)

    def _update_run_state(self) -> None:
        theme = theme_manager().theme
        frame = self._latest_frame
        scenario = None if frame is None else frame.scenario
        phase = None if scenario is None else scenario.phase
        terminal = phase in {ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED}
        time_limit = frame is not None and frame.metrics.time_s >= self._config.duration_s - 1e-9
        self._execution_ended = (
            terminal or time_limit or self._finished or self._last_error is not None
        )
        color = theme.accent
        if self._last_error is not None or phase is ReconfigurationPhase.FAILED:
            title = "Simulation failed"
            detail = self._last_error or (scenario.detail if scenario is not None else "")
            color = theme.danger
        elif phase is ReconfigurationPhase.COMPLETE:
            title = (
                "Target reached · Simulation complete"
                if frame is not None
                and (frame.planning is not None or frame.lattice_planning is not None)
                else "Simulation complete"
            )
            detail = (
                f"Finished at {scenario.time_s:.3f} s. The final state is frozen for inspection."
                if scenario is not None
                else ""
            )
            color = theme.success
        elif time_limit:
            title = (
                "Time limit reached · Target not reached"
                if frame is not None
                and (frame.planning is not None or frame.lattice_planning is not None)
                else "Time limit reached"
            )
            detail = "The simulation has stopped advancing. The scenario did not report completion."
            color = theme.warning
        elif self._finished:
            title, detail = "Simulation stopped", "Results remain available for inspection."
            color = theme.warning
        elif self._shutdown_requested:
            title = "Stopping simulation…"
            detail = "Closing the viewer and releasing simulation resources."
            color = theme.warning
        elif self._playback_paused:
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
        if self._execution_ended:
            self.pause_button.setEnabled(False)
            self.pause_button.setText("Finished")
            self.speed_label.setText("Simulation frozen · results available for inspection")
            self.statusBar().showMessage(title)

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
