"""Qt event-table widgets for immutable runtime event rows."""

from __future__ import annotations

from PySide6.QtCore import QAbstractTableModel, QModelIndex, QPersistentModelIndex, Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHeaderView,
    QTableView,
    QVBoxLayout,
    QWidget,
)

from modsim.runtime.inspection import RuntimeEventRow

_HEADERS = ("Seq", "Sim time (s)", "Event", "Detail")


class RuntimeEventTableModel(QAbstractTableModel):
    """Read-only table model that efficiently appends ordered event deltas."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._events: tuple[RuntimeEventRow, ...] = ()

    @property
    def events(self) -> tuple[RuntimeEventRow, ...]:
        """Return every event row currently shown."""
        return self._events

    def set_events(self, events: tuple[RuntimeEventRow, ...]) -> None:
        """Replace or append the complete ordered presentation event list."""
        sequences = tuple(event.sequence for event in events)
        if sequences != tuple(sorted(set(sequences))):
            raise ValueError("runtime event rows must have unique ascending sequence numbers")
        if events == self._events:
            return

        old_count = len(self._events)
        if len(events) >= old_count and events[:old_count] == self._events:
            self.beginInsertRows(QModelIndex(), old_count, len(events) - 1)
            self._events = events
            self.endInsertRows()
            return

        self.beginResetModel()
        self._events = events
        self.endResetModel()

    def rowCount(
        self,
        parent: QModelIndex | QPersistentModelIndex = QModelIndex(),  # noqa: B008
    ) -> int:
        if parent.isValid():
            return 0
        return len(self._events)

    def columnCount(
        self,
        parent: QModelIndex | QPersistentModelIndex = QModelIndex(),  # noqa: B008
    ) -> int:
        if parent.isValid():
            return 0
        return len(_HEADERS)

    def data(
        self,
        index: QModelIndex | QPersistentModelIndex,
        role: int = int(Qt.ItemDataRole.DisplayRole),
    ) -> object:
        if not index.isValid() or not 0 <= index.row() < len(self._events):
            return None
        event = self._events[index.row()]
        if role == int(Qt.ItemDataRole.DisplayRole):
            values = (
                str(event.sequence),
                f"{event.time_s:.4f}",
                event.kind,
                event.detail,
            )
            return values[index.column()] if 0 <= index.column() < len(values) else None
        if role == int(Qt.ItemDataRole.ToolTipRole):
            return event.detail or event.kind
        if role == int(Qt.ItemDataRole.TextAlignmentRole) and index.column() in (0, 1):
            return int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        return None

    def headerData(
        self,
        section: int,
        orientation: Qt.Orientation,
        role: int = int(Qt.ItemDataRole.DisplayRole),
    ) -> object:
        if role != int(Qt.ItemDataRole.DisplayRole):
            return None
        if orientation == Qt.Orientation.Horizontal and 0 <= section < len(_HEADERS):
            return _HEADERS[section]
        return str(section + 1) if orientation == Qt.Orientation.Vertical else None


class RuntimeEventLogWidget(QWidget):
    """Table view that follows new events only while already at the bottom."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.table = QTableView()
        self.model = RuntimeEventTableModel(self.table)
        self.table.setModel(self.model)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.table)

    def set_events(self, events: tuple[RuntimeEventRow, ...]) -> None:
        """Display a complete accumulated event sequence."""
        scrollbar = self.table.verticalScrollBar()
        follow_tail = scrollbar.value() >= scrollbar.maximum() - 1
        self.model.set_events(events)
        if follow_tail:
            self.table.scrollToBottom()


__all__ = ["RuntimeEventLogWidget", "RuntimeEventTableModel"]
