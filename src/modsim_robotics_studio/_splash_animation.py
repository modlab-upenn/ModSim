"""Animate the approved ModSim vector mark without changing its artwork."""

from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass
from xml.etree import ElementTree

from PySide6.QtCore import QElapsedTimer, QEvent, QPointF, QRectF, QSize, Qt, QTimer
from PySide6.QtGui import QHideEvent, QPainter, QPaintEvent, QShowEvent
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import QSizePolicy, QWidget

from modsim_robotics_studio._branding import asset_path
from modsim_robotics_studio.appearance import theme_manager

_SVG_NAMESPACE = "http://www.w3.org/2000/svg"
_FRAME_INTERVAL_MS = 33
_ASSEMBLY_DURATION_MS = 1_100.0


@dataclass(frozen=True, slots=True)
class _LogoLayer:
    renderer: QSvgRenderer
    offset: QPointF
    delay_ms: float
    duration_ms: float


class AnimatedBrandLogo(QWidget):
    """Assemble the supplied symbol, reveal its wordmark, then softly pulse.

    Animation belongs to the splash's Qt event loop. ``start`` may be called
    before showing the widget; hiding or closing it stops its timer. The
    parent controls how long the splash remains visible.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("AnimatedBrandLogo")
        self.setAccessibleName("ModSim")
        self.setAccessibleDescription("Animated ModSim startup logo")
        self.setFixedHeight(76)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self._appearance = theme_manager()
        self._layers: list[_LogoLayer] = []
        self._view_box = QRectF()
        self._elapsed_ms: float | None = None
        self._start_requested = False
        self._clock = QElapsedTimer()
        self._timer = QTimer(self)
        self._timer.setObjectName("modsimLogoAnimation")
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.setInterval(_FRAME_INTERVAL_MS)
        self._timer.timeout.connect(self._advance)
        self._appearance.changed.connect(self._load_artwork)
        self._load_artwork()

    def sizeHint(self) -> QSize:
        return QSize(360, 76)

    def start(self) -> None:
        """Start or restart the visual sequence without blocking initialization."""
        self._start_requested = True
        self._elapsed_ms = 0.0
        if self.isVisible():
            self._clock.start()
            self._timer.start()
        self.update()

    def stop(self) -> None:
        """Freeze the current frame and release the periodic GUI work."""
        self._start_requested = False
        self._timer.stop()

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        if self._start_requested:
            self._clock.start()
            self._timer.start()

    def hideEvent(self, event: QHideEvent) -> None:
        self.stop()
        super().hideEvent(event)

    def event(self, event: QEvent) -> bool:
        handled = super().event(event)
        if event.type() == QEvent.Type.DevicePixelRatioChange:
            # SVG painting uses the destination's current device transform.
            self.update()
        return handled

    def _advance(self) -> None:
        if not self.isVisible():
            self.stop()
            return
        self._elapsed_ms = float(self._clock.elapsed())
        self.update()

    def _load_artwork(self) -> None:
        root = ElementTree.parse(
            asset_path(self._appearance.theme.id, "logos/svg/modsim-horizontal.svg")
        ).getroot()
        groups = [child for child in root if child.tag.rsplit("}", 1)[-1] == "g"]
        if len(groups) != 2:
            raise ValueError("The approved horizontal logo must contain its symbol and wordmark")
        view_box = root.attrib["viewBox"]
        self._view_box = QRectF(*(float(value) for value in view_box.split()))
        layers: list[_LogoLayer] = []
        offsets = ((0.0, -30.0), (-30.0, 0.0), (0.0, 30.0), (30.0, 20.0))
        for index, part in enumerate(groups[0]):
            group = ElementTree.Element("g", groups[0].attrib)
            group.append(deepcopy(part))
            offset = offsets[index % len(offsets)]
            layers.append(
                _LogoLayer(
                    self._renderer(group, view_box),
                    QPointF(*offset),
                    index * 100.0,
                    550.0,
                )
            )
        layers.append(
            _LogoLayer(self._renderer(groups[1], view_box), QPointF(18.0, 0.0), 250.0, 700.0)
        )
        for layer in self._layers:
            layer.renderer.deleteLater()
        self._layers = layers
        self.update()

    def _renderer(self, source: ElementTree.Element, view_box: str) -> QSvgRenderer:
        # Keep every approved path and transform. Only separate the existing
        # groups into in-memory layers; packaged artwork is never rewritten.
        svg = ElementTree.Element("svg", {"xmlns": _SVG_NAMESPACE, "viewBox": view_box})
        group = deepcopy(source)
        for node in group.iter():
            node.tag = node.tag.rsplit("}", 1)[-1]
        svg.append(group)
        renderer = QSvgRenderer(ElementTree.tostring(svg), self)
        if not renderer.isValid():
            raise ValueError("The approved ModSim logo could not be rendered")
        return renderer

    def paintEvent(self, event: QPaintEvent) -> None:
        super().paintEvent(event)
        if self._view_box.isEmpty() or self.width() <= 0:
            return
        scale = min(self.width() / self._view_box.width(), self.height() / self._view_box.height())
        width, height = self._view_box.width() * scale, self._view_box.height() * scale
        target = QRectF((self.width() - width) / 2, (self.height() - height) / 2, width, height)
        elapsed = self._elapsed_ms
        pulse = (
            0.97 + 0.03 * math.cos((elapsed - _ASSEMBLY_DURATION_MS) * math.tau / 1_800.0)
            if elapsed is not None and elapsed >= _ASSEMBLY_DURATION_MS
            else 1.0
        )
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        try:
            for layer in self._layers:
                progress = (
                    min(1.0, max(0.0, (elapsed - layer.delay_ms) / layer.duration_ms))
                    if elapsed is not None
                    else 1.0
                )
                eased = 1.0 - (1.0 - progress) ** 3
                painter.save()
                painter.setOpacity((0.25 + 0.75 * eased) * pulse)
                painter.translate(layer.offset * ((1.0 - eased) * scale))
                layer.renderer.render(painter, target)
                painter.restore()
        finally:
            painter.end()
