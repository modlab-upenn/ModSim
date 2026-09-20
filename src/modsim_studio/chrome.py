"""Shared native Studio header, theme picker, and compact property widgets."""

from __future__ import annotations

from html import escape

from PySide6.QtCore import QSize, Qt
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QSizePolicy,
    QToolBar,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from modsim_studio.appearance import THEMES, ThemeManager, theme_manager
from modsim_studio.icons import studio_icon


class ThemePicker(QComboBox):
    """A synchronized picker in each window; themes apply immediately."""

    def __init__(self, manager: ThemeManager, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._manager = manager
        self.setObjectName("ThemePicker")
        self.setAccessibleName("Color theme")
        self.setToolTip("Choose a Studio color theme. Your choice is remembered for both apps.")
        for theme in THEMES.values():
            self.addItem(studio_icon("palette", theme.accent), theme.name, theme.id)
        self._sync()
        self.currentIndexChanged.connect(self._select)
        manager.changed.connect(self._sync)

    def _select(self, index: int) -> None:
        identifier = self.itemData(index)
        if isinstance(identifier, str):
            self._manager.set_theme(identifier)

    def _sync(self) -> None:
        previous = self.blockSignals(True)
        self.setCurrentIndex(self.findData(self._manager.theme.id))
        self.blockSignals(previous)


class StudioHeader(QToolBar):
    """Brand and workspace identity above the application's working controls."""

    def __init__(self, window: QMainWindow, workspace: str) -> None:
        super().__init__("Studio header", window)
        self.setObjectName("StudioHeader")
        self.setMovable(False)
        self.setFloatable(False)
        self._manager = theme_manager()
        self._logo = QLabel()
        self._logo.setFixedSize(36, 32)
        self.addWidget(self._logo)
        brand = QLabel("ModSim Studio")
        brand.setObjectName("Brand")
        self.addWidget(brand)
        self.addSeparator()
        workspace_label = QLabel(workspace)
        workspace_label.setObjectName("Workspace")
        self.addWidget(workspace_label)
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.addWidget(spacer)
        appearance = QLabel("Theme  ")
        appearance.setObjectName("Muted")
        self.addWidget(appearance)
        self.theme_picker = ThemePicker(self._manager, self)
        self.addWidget(self.theme_picker)
        self._refresh_logo()
        self._manager.changed.connect(self._refresh_logo)

    def _refresh_logo(self) -> None:
        icon = studio_icon("cube", self._manager.theme.accent, 30)
        self._logo.setPixmap(icon.pixmap(QSize(30, 30)))
        window = self.parentWidget()
        if window is not None:
            window.setWindowIcon(icon)


def property_group(title: str, layout: QVBoxLayout) -> QFormLayout:
    """Add a visually grouped, responsive property section."""
    group = QGroupBox(title)
    form = QFormLayout(group)
    form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
    form.setVerticalSpacing(10)
    layout.addWidget(group)
    return form


class VectorEdit(QWidget):
    """Three component inputs that preserve the existing comma-vector parser."""

    def __init__(self, values: tuple[float, float, float], labels: str, name: str) -> None:
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(5)
        self.fields: list[QLineEdit] = []
        for label, value in zip(labels, values, strict=True):
            field = QLineEdit(f"{value:g}")
            field.setMinimumWidth(35)
            field.setAlignment(Qt.AlignmentFlag.AlignRight)
            field.setAccessibleName(f"{name} {label}")
            field.setToolTip(f"{name} · {label}")
            row.addWidget(field)
            self.fields.append(field)

    def text(self) -> str:
        """Return the editable triple for document validation on Apply."""
        return ", ".join(field.text() for field in self.fields)


class DetailsSection(QWidget):
    """Keep optional metadata available without crowding the main controls."""

    def __init__(self, title: str, content: QWidget) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.toggle = QToolButton()
        self.toggle.setText(title)
        self.toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.toggle.setCheckable(True)
        self.toggle.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.toggle.setAccessibleName(f"Expand {title}")
        content.hide()
        layout.addWidget(self.toggle)
        layout.addWidget(content)
        self.toggle.toggled.connect(content.setVisible)
        self.toggle.toggled.connect(self._set_expanded)

    def _set_expanded(self, expanded: bool) -> None:
        self.toggle.setArrowType(Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow)
        self.toggle.setAccessibleName(
            f"{'Collapse' if expanded else 'Expand'} {self.toggle.text()}"
        )


class LegendWidget(DetailsSection):
    """A compact legend that can be pinned open or read by hovering its toggle."""

    def __init__(self, title: str = "Legend") -> None:
        self.description = QLabel()
        self.description.setWordWrap(True)
        self.description.setTextFormat(Qt.TextFormat.RichText)
        super().__init__(title, self.description)

    def set_entries(self, entries: tuple[tuple[str, str, str], ...], hint: str = "") -> None:
        rows = " &nbsp; · &nbsp; ".join(
            f'<span style="color:{escape(color)}">{escape(symbol)}</span> {escape(label)}'
            for symbol, color, label in entries
        )
        text = rows + (f"<br>{escape(hint)}" if hint else "")
        self.description.setText(text)
        self.toggle.setToolTip(text)
