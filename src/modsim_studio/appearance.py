"""Application-wide Studio colors and local user appearance preferences.

This module belongs to the optional Qt application, never the Robot Pack or
runtime model. Both Studio entry points use the same explicit settings scope.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QObject, QSettings, Qt, Signal
from PySide6.QtGui import QColor, QFont, QFontDatabase, QPalette
from PySide6.QtWidgets import QApplication


@dataclass(frozen=True, kw_only=True)
class StudioTheme:
    """Semantic color tokens shared by Qt chrome and embedded renderers."""

    id: str
    name: str
    shell: str
    surface: str
    raised: str
    field: str
    border: str
    text: str
    muted: str
    accent: str
    on_accent: str
    selection: str
    viewport: str
    viewport_top: str
    grid: str
    grid_major: str
    success: str
    warning: str
    danger: str
    radius: int = 7


THEMES: dict[str, StudioTheme] = {
    "graphite": StudioTheme(
        id="graphite",
        name="Graphite Workbench",
        shell="#191c21",
        surface="#23272e",
        raised="#2d333d",
        field="#1b2027",
        border="#3b4350",
        text="#e7edf5",
        muted="#a6b2c2",
        accent="#78b4fa",
        on_accent="#111d2c",
        selection="#30475f",
        viewport="#13171c",
        viewport_top="#29333d",
        grid="#303943",
        grid_major="#65778a",
        success="#79d7ae",
        warning="#f2c66d",
        danger="#ff939b",
        radius=5,
    ),
    "light": StudioTheme(
        id="light",
        name="Light Studio",
        shell="#f2f4f8",
        surface="#ffffff",
        raised="#eaf0f8",
        field="#f8fafd",
        border="#d5dde8",
        text="#243044",
        muted="#586b83",
        accent="#4068db",
        on_accent="#ffffff",
        selection="#dce7ff",
        viewport="#e9edf2",
        viewport_top="#f5f7fb",
        grid="#c7d0dc",
        grid_major="#7c8ea5",
        success="#157652",
        warning="#8b5b0b",
        danger="#be3349",
    ),
    "midnight": StudioTheme(
        id="midnight",
        name="Midnight Panels",
        shell="#101923",
        surface="#1a2735",
        raised="#263747",
        field="#14212e",
        border="#344b5f",
        text="#e6f0f7",
        muted="#a0b5c8",
        accent="#4ac6b2",
        on_accent="#082922",
        selection="#234b50",
        viewport="#111c27",
        viewport_top="#2b3d4e",
        grid="#314657",
        grid_major="#6b849a",
        success="#6bddad",
        warning="#efc473",
        danger="#ff929b",
        radius=8,
    ),
}
DEFAULT_THEME = "midnight"
_SETTINGS_KEY = "appearance/theme"


class ThemeManager(QObject):
    """Apply live appearance changes without touching document/runtime state."""

    changed = Signal()

    def __init__(self, application: QApplication, settings: QSettings | None = None) -> None:
        super().__init__(application)
        self.setObjectName("studioAppearance")
        self._application = application
        self._settings = settings if settings is not None else QSettings("ModSim", "Studio")
        saved = self._settings.value(_SETTINGS_KEY, DEFAULT_THEME)
        self.theme = THEMES.get(str(saved), THEMES[DEFAULT_THEME])
        # Set this before constructing dialogs. Per-dialog options alone can
        # arrive after Qt has created a GTK helper and loaded its native icons.
        application.setAttribute(Qt.ApplicationAttribute.AA_DontUseNativeDialogs)
        application.setStyle("Fusion")
        families = QFontDatabase.families()
        font = QFontDatabase.systemFont(QFontDatabase.SystemFont.GeneralFont)
        for family in ("Inter", "Segoe UI", "Noto Sans", "DejaVu Sans"):
            if family in families:
                font = QFont(family, 10)
                break
        application.setFont(font)
        self._apply()

    def set_theme(self, theme_id: str, *, persist: bool = True) -> None:
        """Select a known theme; only explicit choices write a user preference."""
        theme = THEMES[theme_id]
        if persist:
            self._settings.setValue(_SETTINGS_KEY, theme_id)
            self._settings.sync()
        if theme == self.theme:
            return
        self.theme = theme
        self._apply()
        self.changed.emit()

    def _apply(self) -> None:
        theme = self.theme
        palette = QPalette()
        for role, color in (
            (QPalette.ColorRole.Window, theme.shell),
            (QPalette.ColorRole.WindowText, theme.text),
            (QPalette.ColorRole.Base, theme.field),
            (QPalette.ColorRole.AlternateBase, theme.raised),
            (QPalette.ColorRole.Text, theme.text),
            (QPalette.ColorRole.Button, theme.surface),
            (QPalette.ColorRole.ButtonText, theme.text),
            (QPalette.ColorRole.Highlight, theme.selection),
            (QPalette.ColorRole.HighlightedText, theme.text),
            (QPalette.ColorRole.ToolTipBase, theme.raised),
            (QPalette.ColorRole.ToolTipText, theme.text),
            (QPalette.ColorRole.Link, theme.accent),
            (QPalette.ColorRole.PlaceholderText, theme.muted),
            (QPalette.ColorRole.Light, theme.border),
            (QPalette.ColorRole.Mid, theme.border),
            (QPalette.ColorRole.Dark, theme.shell),
            (QPalette.ColorRole.Shadow, theme.shell),
        ):
            palette.setColor(role, QColor(color))
        for role in (
            QPalette.ColorRole.Text,
            QPalette.ColorRole.ButtonText,
            QPalette.ColorRole.WindowText,
        ):
            palette.setColor(QPalette.ColorGroup.Disabled, role, QColor(theme.muted))
        self._application.setPalette(palette)
        self._application.setStyleSheet(_stylesheet(theme))


def theme_manager() -> ThemeManager:
    """Return the single appearance controller owned by the QApplication."""
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        raise RuntimeError("Studio appearance requires a QApplication")
    manager = application.findChild(ThemeManager, "studioAppearance")
    return manager if manager is not None else ThemeManager(application)


def _stylesheet(t: StudioTheme) -> str:
    check = (Path(__file__).parent / "assets" / "check.svg").as_posix()
    return f"""
    QWidget {{ color: {t.text}; selection-background-color: {t.selection};
               selection-color: {t.text}; }}
    QMainWindow, QDialog {{ background: {t.shell}; }}
    QMainWindow::separator {{ background: {t.shell}; width: 9px; height: 9px; }}
    QMainWindow::separator:hover, QSplitter::handle:hover {{ background: {t.accent}; }}
    QWidget#Panel, QWidget#StudioHeader, QWidget#InspectorContent {{ background: {t.surface}; }}
    QLabel {{ background: transparent; }}
    QLabel#Brand {{ font-size: 20px; font-weight: 650; }}
    QLabel#Workspace {{ color: {t.accent}; background: {t.selection};
        border: 1px solid {t.border}; border-radius: {t.radius}px; padding: 7px 14px; }}
    QLabel#SectionTitle {{ font-size: 15px; font-weight: 600; }}
    QLabel#Muted, QLabel#Breadcrumb {{ color: {t.muted}; }}
    QLabel#Hint {{ color: {t.muted}; background: {t.surface}; padding: 6px 10px; }}
    QLabel#ValidationSummary {{ font-size: 15px; font-weight: 600; }}
    QLabel#ValidationSummary[state="success"] {{ color: {t.success}; }}
    QLabel#ValidationSummary[state="warning"] {{ color: {t.warning}; }}
    QLabel#ValidationSummary[state="error"] {{ color: {t.danger}; }}
    QLabel#SaveState {{ color: {t.accent}; padding: 0 12px; }}
    QToolBar {{ background: {t.surface}; border: 0; spacing: 6px; padding: 7px; }}
    QToolBar#StudioHeader {{ padding: 9px 12px; border-bottom: 1px solid {t.border}; }}
    QToolBar#DocumentToolbar {{ border-bottom: 1px solid {t.border}; }}
    QToolBar::separator {{ background: {t.border}; width: 1px; margin: 5px 8px; }}
    QDockWidget {{ font-weight: 600; border: 1px solid {t.border}; }}
    QDockWidget::title {{ background: {t.surface}; padding: 11px 12px;
        border-bottom: 1px solid {t.border}; }}
    QScrollArea {{ border: 0; background: {t.surface}; }}
    QGroupBox {{ background: {t.surface}; border: 1px solid {t.border};
        border-radius: {t.radius}px; margin-top: 16px; padding: 14px 10px 10px; }}
    QGroupBox::title {{ subcontrol-origin: margin; subcontrol-position: top left;
        left: 10px; padding: 0 5px; color: {t.text}; font-weight: 600; }}
    QPushButton, QToolButton {{ background: {t.raised}; border: 1px solid {t.border};
        border-radius: {t.radius}px; padding: 7px 12px; }}
    QToolButton {{ padding: 6px 9px; }}
    QPushButton:hover, QToolButton:hover {{ background: {t.selection}; border-color: {t.accent}; }}
    QPushButton:pressed, QToolButton:pressed {{ background: {t.selection}; }}
    QPushButton:checked, QToolButton:checked {{ background: {t.selection};
        border-color: {t.accent}; color: {t.text}; }}
    QPushButton[primary="true"], QToolButton[primary="true"] {{
        background: {t.accent}; color: {t.on_accent}; border-color: {t.accent}; font-weight: 600; }}
    QPushButton[primary="true"]:hover, QToolButton[primary="true"]:hover {{
        background: {t.selection}; color: {t.text}; }}
    QPushButton[danger="true"] {{ color: {t.danger}; background: {t.surface}; }}
    QPushButton:disabled, QToolButton:disabled {{ color: {t.muted}; background: {t.surface};
        border-color: {t.border}; }}
    QPushButton:focus, QToolButton:focus {{ border: 1px solid {t.accent}; }}
    QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit {{
        background: {t.field}; border: 1px solid {t.border}; border-radius: 4px;
        padding: 6px; }}
    QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus {{ border-color: {t.accent}; }}
    QLineEdit:disabled, QComboBox:disabled {{ color: {t.muted}; background: {t.surface}; }}
    QComboBox {{ padding-right: 22px; min-height: 18px; }}
    QComboBox::drop-down {{ border: 0; width: 22px; }}
    QComboBox QAbstractItemView {{ background: {t.surface}; border: 1px solid {t.border};
        padding: 4px; selection-background-color: {t.selection}; }}
    QCheckBox {{ spacing: 7px; padding: 3px 0; }}
    QCheckBox::indicator {{ width: 14px; height: 14px; background: {t.field};
        border: 1px solid {t.border}; border-radius: 3px; }}
    QCheckBox::indicator:checked {{ background: {t.accent}; border-color: {t.accent};
        image: url("{check}"); }}
    QCheckBox::indicator:hover, QCheckBox::indicator:focus {{ border-color: {t.accent}; }}
    QCheckBox::indicator:disabled {{ background: {t.raised}; border-color: {t.border}; }}
    QTreeView, QTableView {{ background: {t.surface}; alternate-background-color: {t.raised};
        border: 0; gridline-color: {t.border}; outline: 0; }}
    QTreeView {{ padding: 6px; show-decoration-selected: 1; }}
    QTreeView::item {{ padding: 5px 3px; border-radius: 4px; }}
    QTreeView::item:selected, QTableView::item:selected {{
        background: {t.selection}; color: {t.text}; }}
    QTreeView::item:hover {{ background: {t.raised}; }}
    QTableView::item {{ padding: 5px; }}
    QHeaderView::section {{ background: {t.surface}; color: {t.muted};
        border: 0; border-bottom: 1px solid {t.border}; padding: 8px; }}
    QTabWidget::pane {{ border: 1px solid {t.border}; background: {t.surface}; }}
    QTabBar::tab {{ background: {t.surface}; color: {t.muted};
        border-bottom: 3px solid transparent; padding: 10px 20px; }}
    QTabBar::tab:selected {{ color: {t.text}; background: {t.raised};
        border-bottom-color: {t.accent}; }}
    QTabBar::tab:hover {{ color: {t.text}; }}
    QMenuBar, QMenu {{ background: {t.surface}; }}
    QMenuBar::item {{ padding: 5px 10px; background: transparent; }}
    QMenuBar::item:selected, QMenu::item:selected {{ background: {t.selection}; }}
    QMenu {{ border: 1px solid {t.border}; padding: 5px; }}
    QMenu::item {{ padding: 6px 26px; }}
    QMenu::separator {{ height: 1px; background: {t.border}; margin: 5px; }}
    QStatusBar {{ background: {t.surface}; color: {t.muted}; border-top: 1px solid {t.border}; }}
    QStatusBar::item {{ border: 0; }}
    QSplitter::handle {{ background: {t.shell}; height: 8px; width: 8px; }}
    QScrollBar:vertical {{ background: {t.surface}; width: 10px; margin: 0; }}
    QScrollBar:horizontal {{ background: {t.surface}; height: 10px; margin: 0; }}
    QScrollBar::handle {{ background: {t.border}; border-radius: 4px;
        min-height: 24px; min-width: 24px; }}
    QScrollBar::handle:hover {{ background: {t.muted}; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
    QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}
    QToolTip {{ background: {t.raised}; color: {t.text};
        border: 1px solid {t.border}; padding: 6px; }}
    """
