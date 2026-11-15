"""Small original line icons; no third-party artwork or icon dependency."""

from PySide6.QtCore import QByteArray, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer

_PATHS = {
    "cube": '<path d="m12 2 9 5v10l-9 5-9-5V7Zm0 10 9-5M12 12 3 7m9 5v10M7.5 4.5l9 5"/>',
    "folder": '<path d="M3 6h7l2 2h9v12H3Zm0 6h18M3 6V4h7l2 2"/>',
    "save": '<path d="M4 3h13l3 3v15H4ZM8 3v6h8V3M8 21v-8h8v8"/>',
    "check": '<circle cx="12" cy="12" r="9"/><path d="m7 12 3 3 7-7"/>',
    "search": '<circle cx="10" cy="10" r="6"/><path d="m15 15 6 6"/>',
    "link": '<path d="m10 8 3-3a4 4 0 0 1 6 6l-3 3m-2 2-3 3a4 4 0 0 1-6-6l3-3m0 6 8-8"/>',
    "joint": '<circle cx="12" cy="12" r="4"/><path d="M12 2v6m0 8v6M2 12h6m8 0h6"/>',
    "connector": '<path d="M8 3v5m8-5v5M6 8h12v5a6 6 0 0 1-12 0Zm6 11v3"/>',
    "graph": '<circle cx="5" cy="6" r="3"/><circle cx="19" cy="6" r="3"/>'
    '<circle cx="12" cy="19" r="3"/><path d="M8 6h8M6.5 9l4 7m7-7-4 7"/>',
    "file": '<path d="M5 2h9l5 5v15H5Zm9 0v6h5M8 12h8m-8 4h8"/>',
    "fit": '<path d="M3 9V3h6m6 0h6v6M3 15v6h6m6 0h6v-6M8 12h8m-4-4v8"/>',
    "palette": '<circle cx="12" cy="12" r="9"/><path d="M12 3a9 9 0 0 0 0 18Z"/>',
}


def studio_icon(name: str, color: str, size: int = 20) -> QIcon:
    """Render a scalable, recolorable icon into a high-DPI Qt pixmap."""
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" '
        f'fill="none" stroke="{QColor(color).name()}" stroke-width="1.6" '
        f'stroke-linecap="round" stroke-linejoin="round">{_PATHS[name]}</svg>'
    )
    pixmap = QPixmap(size * 2, size * 2)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    QSvgRenderer(QByteArray(svg.encode())).render(painter)
    painter.end()
    pixmap.setDevicePixelRatio(2.0)
    return QIcon(pixmap)
