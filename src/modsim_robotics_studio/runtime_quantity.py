# pyright: reportMissingTypeStubs=false, reportUnknownMemberType=false, reportUnknownArgumentType=false
"""Display runtime quantities in chosen units while retaining SI plot/form data."""

from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QFocusEvent, QKeyEvent, QValidator
from PySide6.QtWidgets import QDoubleSpinBox, QWidget

from modsim_robotics.units import QuantityKind, UnitError, from_si, parse_quantity


class QuantitySpinBox(QDoubleSpinBox):
    """Keep the spinbox value/range/step API canonical while converting its text.

    Args:
        kind: Physical quantity expected by this control.
        unit: Display and implicit input unit.
        parent: Optional Qt owner.
    """

    def __init__(self, *, kind: QuantityKind, unit: str, parent: QWidget | None = None) -> None:
        self.kind = kind
        self.unit = unit
        super().__init__(parent)
        from_si(0.0, unit, kind=kind)
        # Qt otherwise rounds its canonical numeric value to decimal places
        # before textFromValue runs. Keep the full double; only display rounds.
        self.setDecimals(323)

    def textFromValue(self, value: float) -> str:
        """Format the canonical numeric value for the selected unit."""
        # Put units in the formatter rather than QAbstractSpinBox.suffix:
        # Qt otherwise appends that implicit suffix to explicit input (rad deg).
        return f"{from_si(value, self.unit, kind=self.kind):.12g} {self.unit}"

    def valueFromText(self, text: str) -> float:
        """Read unit-bearing or implicit input as a canonical SI value."""
        if text == self.textFromValue(self.value()) + self.suffix():
            return self.value()
        try:
            return parse_quantity(text, kind=self.kind, default_unit=self.unit)
        except UnitError:
            # Qt calls this while a user is composing incomplete input.
            return self.value()

    def validate(self, text: str, position: int) -> tuple[QValidator.State, str, int]:
        """Allow unfinished drafts and accept complete in-range quantities."""
        try:
            value = parse_quantity(text, kind=self.kind, default_unit=self.unit)
        except UnitError:
            return (QValidator.State.Intermediate, text, position)
        state = (
            QValidator.State.Acceptable
            if self.minimum() <= value <= self.maximum()
            else QValidator.State.Intermediate
        )
        return (state, text, position)

    def focusOutEvent(self, event: QFocusEvent) -> None:
        """Keep invalid draft text when Qt would restore the previous number."""
        draft = self.text()
        try:
            self.value_si()
        except UnitError:
            editor = self.lineEdit()
            blocked = self.blockSignals(True)
            editor_blocked = editor.blockSignals(True)
            try:
                super().focusOutEvent(event)
                editor.setText(draft)
            finally:
                editor.blockSignals(editor_blocked)
                self.blockSignals(blocked)
            return
        super().focusOutEvent(event)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        """Keep an invalid draft visible when Enter would silently correct it."""
        if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            try:
                self.value_si()
            except UnitError:
                event.accept()
                return
        super().keyPressEvent(event)

    def value_si(self) -> float:
        """Read the displayed pending quantity in canonical units.

        Returns:
            The finite SI value within this control's canonical range.

        Raises:
            UnitError: Pending input is incomplete, incompatible, or out of range.
        """
        value = parse_quantity(self.text(), kind=self.kind, default_unit=self.unit)
        if not self.minimum() <= value <= self.maximum():
            raise UnitError("Quantity is outside the allowed range.")
        # Preserve the spinbox's precise number when text is merely its rendering.
        rendered = self.textFromValue(self.value()) + self.suffix()
        return self.value() if self.text() == rendered else value

    def set_unit(self, unit: str) -> bool:
        """Change presentation without changing the canonical value or range.

        Args:
            unit: Compatible display and implicit input unit.

        Returns:
            False keeps an unfinished draft and its old unit intact.

        Raises:
            UnitError: The requested unit is incompatible or conversion overflows.
        """
        from_si(0.0, unit, kind=self.kind)
        if unit == self.unit:
            return True
        try:
            canonical = self.value_si()
        except UnitError:
            self.setToolTip(f"Unfinished input retains its previous unit: {self.unit}.")
            return False
        from_si(canonical, unit, kind=self.kind)
        was_blocked = self.blockSignals(True)
        try:
            self.setValue(canonical)
            self.unit = unit
            editor = self.lineEdit()
            editor_blocked = editor.blockSignals(True)
            try:
                editor.setText(self.textFromValue(self.value()) + self.suffix())
            finally:
                editor.blockSignals(editor_blocked)
        finally:
            self.blockSignals(was_blocked)
        self.setToolTip(
            f"Enter {unit}, or include an explicit compatible unit. Runtime values use SI."
        )
        return True


class QuantityAxis(pg.AxisItem):
    """Convert axis labels only; curve coordinates and camera ranges remain SI.

    Args:
        orientation: PyQtGraph axis side, such as bottom or left.
        kind: Physical quantity represented by the canonical coordinates.
        unit: Compatible display unit for tick labels.
    """

    def __init__(self, orientation: str, *, kind: QuantityKind, unit: str) -> None:
        self.kind = kind
        self.unit = unit
        super().__init__(orientation=orientation)
        self.enableAutoSIPrefix(False)

    def tickStrings(self, values: list[float], scale: float, spacing: float) -> list[str]:
        """Format canonical coordinates using the configured display unit."""
        del scale, spacing
        return [f"{from_si(value, self.unit, kind=self.kind):.6g}" for value in values]

    def set_unit(self, unit: str) -> None:
        """Refresh tick text after validating a compatible display unit.

        Args:
            unit: Supported unit for this axis's physical quantity.

        Raises:
            UnitError: The unit is unknown or incompatible with this axis.
        """
        from_si(0.0, unit, kind=self.kind)
        self.unit = unit
        self.picture = None
        self.update()
