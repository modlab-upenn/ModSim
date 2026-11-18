"""Application-wide Studio colors and local user appearance preferences.

This module belongs to the optional Qt application, never the Robot Pack or
runtime model. Both Studio entry points use the same explicit settings scope.
"""

from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import QObject, QSettings, Qt, Signal
from PySide6.QtGui import QColor, QFontDatabase, QFontInfo, QPalette
from PySide6.QtWidgets import QApplication

from ._branding import application_icon, asset_path, theme_palette
from .visual_colors import visual_colors


@dataclass(frozen=True, kw_only=True)
class StudioTheme:
    """Semantic color tokens shared by Qt chrome and embedded renderers.

    Brand surfaces and interaction colors come from the selected production
    palette. Functional controls use a stronger outline than decorative panel
    borders. Status colors retain the graph's categorical meanings.
    """

    id: str
    name: str
    shell: str
    surface: str
    raised: str
    field: str
    border: str
    control_border: str
    text: str
    muted: str
    disabled: str
    link: str
    accent: str
    focus: str
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


def _brand_theme(theme_id: str) -> StudioTheme:
    """Map the approved pack palette to the application's semantic tokens."""
    palette = {
        key: value.lower() if value.startswith("#") else value
        for key, value in theme_palette(theme_id).items()
    }
    status = visual_colors(theme_id)
    return StudioTheme(
        id=theme_id,
        name=f"ModSim {palette['name']}",
        shell=palette["background"],
        surface=palette["surface"],
        raised=palette["raised"],
        field=palette["surface"],
        border=palette["border"],
        control_border=palette["control_border"],
        text=palette["text"],
        muted=palette["muted"],
        disabled=palette["disabled"],
        link=palette.get("link", palette["blue"]),
        accent=palette["accent"],
        focus=palette["focus"],
        on_accent=palette["accent_text"],
        selection=palette["selection"],
        viewport=palette["background"],
        viewport_top=palette["surface"],
        grid=palette["raised"],
        grid_major=palette["slate"],
        # The brand packs define chrome colors, not error/diagnostic hues.
        # Keep the established categorical meanings and light-theme contrast.
        success=status["green"],
        warning=status["gold"],
        danger=status["red"],
        radius=6,
    )


THEMES: dict[str, StudioTheme] = {
    identifier: _brand_theme(identifier) for identifier in ("light", "dark", "midnight")
}
DEFAULT_THEME = "midnight"
_SETTINGS_KEY = "appearance/theme"
_THEME_ALIASES = {"graphite": "dark"}


class ThemeManager(QObject):
    """Apply live appearance changes without touching document/runtime state."""

    changed = Signal()

    def __init__(self, application: QApplication, settings: QSettings | None = None) -> None:
        super().__init__(application)
        self.setObjectName("studioAppearance")
        self._application = application
        self._settings = settings if settings is not None else QSettings("ModSim", "Studio")
        saved = self._settings.value(_SETTINGS_KEY, DEFAULT_THEME)
        identifier = _THEME_ALIASES.get(str(saved), str(saved))
        self.theme = THEMES.get(identifier, THEMES[DEFAULT_THEME])
        # Set this before constructing dialogs. Per-dialog options alone can
        # arrive after Qt has created a GTK helper and loaded its native icons.
        application.setAttribute(Qt.ApplicationAttribute.AA_DontUseNativeDialogs)
        application.setStyle("Fusion")
        families = QFontDatabase.families()
        # Retain the platform/user font's size, weight, and other accessibility
        # settings. Reconstructing a 10 pt font overrides those preferences.
        font = application.font()
        if font.family() not in families:
            for family in ("Inter", "Segoe UI", "Noto Sans", "DejaVu Sans"):
                if family in families:
                    font.setFamily(family)
                    break
        application.setFont(font)
        self._apply()

    def set_theme(self, theme_id: str, *, persist: bool = True) -> None:
        """Apply a theme and optionally save the user's explicit choice.

        Args:
            theme_id: ``light``, ``dark``, or ``midnight``. The legacy
                ``graphite`` name selects ``dark``.
            persist: Whether to save the canonical ID in local Qt settings.
                Restoring a preference never rewrites it implicitly.

        Raises:
            KeyError: The requested theme ID is unknown.
        """
        theme_id = _THEME_ALIASES.get(theme_id, theme_id)
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
            (QPalette.ColorRole.Link, theme.link),
            (QPalette.ColorRole.BrightText, theme.on_accent),
            (QPalette.ColorRole.PlaceholderText, theme.muted),
            (QPalette.ColorRole.Light, theme.control_border),
            (QPalette.ColorRole.Mid, theme.control_border),
            (QPalette.ColorRole.Dark, theme.shell),
            (QPalette.ColorRole.Shadow, theme.shell),
        ):
            palette.setColor(role, QColor(color))
        for role in (
            QPalette.ColorRole.Text,
            QPalette.ColorRole.ButtonText,
            QPalette.ColorRole.WindowText,
        ):
            palette.setColor(QPalette.ColorGroup.Disabled, role, QColor(theme.disabled))
        self._application.setPalette(palette)
        self._application.setStyleSheet(_stylesheet(theme))
        self._application.setWindowIcon(application_icon(theme.id))


def theme_manager() -> ThemeManager:
    """Return the single appearance controller owned by the QApplication."""
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        raise RuntimeError("Studio appearance requires a QApplication")
    manager = application.findChild(ThemeManager, "studioAppearance")
    return manager if manager is not None else ThemeManager(application)


def _stylesheet(t: StudioTheme) -> str:
    check = asset_path(t.id, "ui/svg/accent/check.svg").as_posix()
    body_size = QApplication.font().pointSizeF()
    if body_size <= 0:
        body_size = QFontInfo(QApplication.font()).pointSizeF()
    if body_size <= 0:
        body_size = 10.0
    return f"""
    QWidget {{ selection-background-color: {t.selection};
               selection-color: {t.text}; }}
    QMainWindow, QDialog, QWidget#BrandWelcome {{ background: {t.shell}; }}
    QMainWindow::separator {{ background: {t.shell}; width: 9px; height: 9px; }}
    QMainWindow::separator:hover, QSplitter::handle:hover {{ background: {t.accent}; }}
    QWidget#Panel, QWidget#StudioHeader, QWidget#InspectorContent {{ background: {t.surface}; }}
    QWidget#BrandCard, QWidget#LoadingPanel {{ background: {t.surface};
        border: 1px solid {t.border}; border-radius: 12px; }}
    QWidget#LoadingCopy {{ background: transparent; border: 0; }}
    QLabel {{ background: transparent; }}
    QLabel#Brand {{ font-size: {body_size * 1.45:.2f}pt; font-weight: 650; }}
    QLabel#BrandTitle {{ font-size: {body_size * 1.7:.2f}pt; font-weight: 700; }}
    QLabel#BrandSubtitle {{ color: {t.muted}; font-size: {body_size:.2f}pt; }}
    QLabel#LoadingStage {{ color: {t.text}; font-size: {body_size * 1.15:.2f}pt;
        font-weight: 600; }}
    QLabel#LoadingDetail, QLabel#SplashVersion {{ color: {t.muted};
        font-size: {body_size * 0.9:.2f}pt; }}
    QLabel#Workspace {{ color: {t.accent}; background: transparent; border: 0;
        font-size: {body_size:.2f}pt; font-weight: 700; padding: 0 6px; }}
    QLabel#SectionTitle {{ font-size: {body_size * 1.1:.2f}pt; font-weight: 600; }}
    QLabel#Muted, QLabel#Breadcrumb {{ color: {t.muted}; }}
    QLabel#Hint {{ color: {t.muted}; background: {t.surface}; padding: 6px 10px; }}
    QLabel#ValidationSummary {{ font-size: {body_size * 1.1:.2f}pt; font-weight: 600; }}
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
    QPushButton, QToolButton {{ background: {t.raised}; border: 1px solid {t.control_border};
        border-radius: {t.radius}px; padding: 7px 12px; }}
    QToolButton {{ padding: 6px 9px; }}
    QPushButton:hover, QToolButton:hover {{ background: {t.selection}; border-color: {t.focus}; }}
    QPushButton:pressed, QToolButton:pressed {{ background: {t.selection}; }}
    QPushButton:checked, QToolButton:checked {{ background: {t.selection};
        border-color: {t.accent}; color: {t.text}; }}
    QPushButton[primary="true"], QToolButton[primary="true"] {{
        background: {t.accent}; color: {t.on_accent}; border-color: {t.accent}; font-weight: 600; }}
    QPushButton[primary="true"]:hover, QToolButton[primary="true"]:hover {{
        background: {t.accent}; color: {t.on_accent}; border-color: {t.focus}; }}
    QPushButton[danger="true"] {{ color: {t.danger}; background: {t.surface}; }}
    QPushButton:disabled, QToolButton:disabled {{ color: {t.disabled}; background: {t.surface};
        border-color: {t.border}; }}
    QPushButton:focus, QToolButton:focus {{ border: 1px solid {t.focus}; }}
    QLineEdit, QPlainTextEdit, QTextEdit {{ color: {t.text}; background: {t.field};
        border: 1px solid {t.control_border}; border-radius: 4px; padding: 5px; }}
    QLineEdit:focus, QPlainTextEdit:focus, QTextEdit:focus {{ border-color: {t.focus}; }}
    QLineEdit:disabled, QPlainTextEdit:disabled, QTextEdit:disabled {{
        color: {t.disabled}; background: {t.surface}; }}
    QComboBox, QSpinBox, QDoubleSpinBox {{ min-height: 1.3em; }}
    QComboBox QAbstractItemView {{ background: {t.surface}; border: 1px solid {t.control_border};
        padding: 4px; selection-background-color: {t.selection}; }}
    QCheckBox {{ spacing: 7px; padding: 3px 0; }}
    QCheckBox::indicator {{ width: 14px; height: 14px; background: {t.field};
        border: 1px solid {t.control_border}; border-radius: 3px; }}
    QCheckBox::indicator:checked {{ background: {t.on_accent}; border-color: {t.accent};
        image: url("{check}"); }}
    QCheckBox::indicator:hover, QCheckBox::indicator:focus {{ border-color: {t.focus}; }}
    QCheckBox::indicator:disabled {{ background: {t.raised}; border-color: {t.border}; }}
    QTreeView, QTableView, QListView {{ color: {t.text}; background: {t.surface};
        alternate-background-color: {t.raised}; border: 0; gridline-color: {t.border};
        selection-background-color: {t.selection}; selection-color: {t.text}; outline: 0; }}
    QTreeView {{ padding: 6px; show-decoration-selected: 1; }}
    QTreeView::item {{ padding: 0 3px; border-radius: 4px; }}
    QTreeView::item:selected, QTableView::item:selected, QListView::item:selected {{
        background: {t.selection}; color: {t.text}; }}
    QTreeView::item:hover:!selected, QListView::item:hover:!selected {{
        background: {t.raised}; }}
    QTreeView::item:disabled, QTableView::item:disabled, QListView::item:disabled {{
        color: {t.disabled}; }}
    QTableView::item {{ padding: 0 3px; }}
    QAbstractItemView QLineEdit, QAbstractSpinBox QLineEdit {{
        color: {t.text}; background: {t.field}; border: 0; border-radius: 0;
        padding: 0 1px; margin: 0; min-height: 0; }}
    QAbstractItemView QLineEdit:disabled, QAbstractSpinBox QLineEdit:disabled {{
        color: {t.disabled}; }}
    QHeaderView::section {{ background: {t.surface}; color: {t.muted};
        border: 0; border-bottom: 1px solid {t.border}; padding: 8px; }}
    QTabWidget::pane {{ border: 1px solid {t.border}; background: {t.surface}; }}
    QTabBar::tab {{ background: {t.surface}; color: {t.muted};
        border-bottom: 3px solid transparent; padding: 10px 20px; }}
    QTabBar::tab:selected {{ color: {t.text}; background: {t.raised};
        border-bottom-color: {t.accent}; }}
    QTabBar::tab:hover {{ color: {t.text}; background: {t.raised}; }}
    QTabBar::tab:disabled {{ color: {t.disabled}; }}
    QMenuBar, QMenu {{ background: {t.surface}; }}
    QMenuBar::item {{ padding: 5px 10px; background: transparent; }}
    QMenuBar::item:selected, QMenu::item:selected {{ background: {t.selection}; }}
    QMenu {{ border: 1px solid {t.border}; padding: 5px; }}
    QMenu::item {{ padding: 6px 26px; }}
    QMenu::separator {{ height: 1px; background: {t.border}; margin: 5px; }}
    QStatusBar {{ background: {t.surface}; color: {t.muted}; border-top: 1px solid {t.border}; }}
    QStatusBar::item {{ border: 0; }}
    QProgressBar {{ background: {t.raised}; color: {t.text};
        border: 1px solid {t.border}; border-radius: 5px; text-align: center;
        min-height: 10px; }}
    QProgressBar::chunk {{ background: {t.accent}; border-radius: 4px; }}
    QProgressBar#LoadingProgress {{ min-height: 7px; max-height: 7px; border: 0; }}
    QSplitter::handle {{ background: {t.shell}; height: 8px; width: 8px; }}
    QScrollBar:vertical {{ background: {t.surface}; width: 10px; margin: 0; }}
    QScrollBar:horizontal {{ background: {t.surface}; height: 10px; margin: 0; }}
    QScrollBar::handle {{ background: {t.control_border}; border-radius: 4px;
        min-height: 24px; min-width: 24px; }}
    QScrollBar::handle:hover {{ background: {t.muted}; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
    QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}
    QToolTip {{ background: {t.raised}; color: {t.text};
        border: 1px solid {t.control_border}; padding: 6px; }}
    """
