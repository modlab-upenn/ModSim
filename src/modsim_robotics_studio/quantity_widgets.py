"""Unit-aware authoring controls that retain canonical SI values between edits.

These widgets only convert at the input and display boundary. They do not write
Robot Pack models, change coordinate frames, or wrap angular values.
"""

from __future__ import annotations

import re

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QLineEdit, QWidget

from modsim_robotics.units import QuantityKind, UnitError, format_quantity, from_si, parse_quantity


class QuantityEdit(QLineEdit):
    """Edit one physical quantity using a visible unit and an SI-valued cache.

    Args:
        value_si: Initial canonical value in the quantity kind's SI unit.
        kind: Physical quantity expected by this field.
        unit: Initial display and implicit input unit.
        parent: Optional Qt owner.

    Raises:
        UnitError: The initial value is non-finite or the unit is incompatible.
    """

    def __init__(
        self,
        value_si: float,
        *,
        kind: QuantityKind,
        unit: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._kind = kind
        self._unit = unit
        self._canonical_value = 0.0
        self._cached_text = ""
        self.setMinimumWidth(75)
        self.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.set_value_si(value_si)

    @property
    def kind(self) -> QuantityKind:
        """Return the physical quantity represented by this editor."""
        return self._kind

    @property
    def unit(self) -> str:
        """Return the current display and implicit input unit."""
        return self._unit

    def value_si(self) -> float:
        """Read valid pending input as a canonical value without reformatting it.

        Returns:
            A finite SI value. Unchanged formatted text returns the original
            cached number, preserving precision across display-unit changes.

        Raises:
            UnitError: Pending text is empty, invalid, or has incompatible units.
        """
        text = self.text()
        if text != self._cached_text:
            value = parse_quantity(text, kind=self._kind, default_unit=self._unit)
            self._canonical_value = value
            self._cached_text = text
        return self._canonical_value

    def set_value_si(self, value_si: float) -> None:
        """Replace the quantity and render it without emitting edit signals.

        Args:
            value_si: Finite replacement value in canonical SI units.

        Raises:
            UnitError: The value or its display conversion is not finite.
        """
        from_si(value_si, self._unit, kind=self._kind)
        self._render(value_si)

    def set_unit(self, unit: str) -> bool:
        """Convert valid text to another display unit without changing its value.

        Args:
            unit: Requested compatible display and implicit input unit.

        Returns:
            True when the unit is applied. False preserves an invalid pending
            draft, including its previous implicit unit and complete text.

        Raises:
            UnitError: The requested unit is incompatible or conversion overflows.
        """
        from_si(0.0, unit, kind=self._kind)
        if unit == self._unit:
            return True
        try:
            value = self.value_si()
        except UnitError:
            self.refresh_tooltip(
                f"Pending input is still interpreted in {self._unit}. "
                "Complete or correct it before changing this field's unit."
            )
            return False
        from_si(value, unit, kind=self._kind)
        self._unit = unit
        self._render(value)
        return True

    def _render(self, value_si: float) -> None:
        text = format_quantity(value_si, self._unit, precision=12)
        self._canonical_value = float(value_si)
        self._cached_text = text
        previous = self.blockSignals(True)
        try:
            self.setText(text)
        finally:
            self.blockSignals(previous)
        self.setPlaceholderText(f"Value ({self._unit})")
        self.refresh_tooltip()

    def refresh_tooltip(self, detail: str | None = None) -> None:
        """Describe this component and its units without changing editable text.

        Args:
            detail: Optional pending-input guidance replacing the normal unit help.
                The component's accessible name is preserved as a prefix.
        """
        name = self.accessibleName().strip()
        prefix = f"{name} · " if name else ""
        message = detail or (
            f"Enter a value in {self._unit}, or include an explicit compatible unit. "
            "Stored values use SI units."
        )
        self.setToolTip(prefix + message)


class QuantityVectorEdit(QWidget):
    """Edit a physical triple with shared quantity and display-unit metadata.

    Args:
        value_si: Initial three components in canonical SI units.
        kind: Physical quantity of every component.
        unit: Initial display and implicit input unit.
        labels: Exactly three component labels, such as ``XYZ`` or ``RPY``.
        name: Quantity label used by assistive technologies.
        parent: Optional Qt owner.

    Raises:
        UnitError: Any value or unit is invalid, or the value is not a triple.
        ValueError: Component labels do not contain exactly three items.
    """

    def __init__(
        self,
        value_si: tuple[float, float, float],
        *,
        kind: QuantityKind,
        unit: str,
        labels: str | tuple[str, str, str] = "XYZ",
        name: str = "Value",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        if len(value_si) != 3:
            raise UnitError("a quantity vector must contain three values")
        if len(labels) != 3:
            raise ValueError("a quantity vector requires three component labels")
        self._kind = kind
        self._unit = unit
        self.fields: list[QuantityEdit] = []
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(5)
        for label, value in zip(labels, value_si, strict=True):
            field = QuantityEdit(value, kind=kind, unit=unit, parent=self)
            field.setAccessibleName(f"{name} {label}")
            field.refresh_tooltip()
            row.addWidget(field)
            self.fields.append(field)

    @property
    def kind(self) -> QuantityKind:
        """Return the physical quantity represented by every component."""
        return self._kind

    @property
    def unit(self) -> str:
        """Return the display and implicit input unit for the triple."""
        return self._unit

    def value_si(self) -> tuple[float, float, float]:
        """Read every valid pending component in canonical SI units.

        Returns:
            The finite three-component SI vector without wrapping angles.

        Raises:
            UnitError: Any component contains invalid or incompatible input.
        """
        return (self.fields[0].value_si(), self.fields[1].value_si(), self.fields[2].value_si())

    def set_value_si(self, value_si: tuple[float, float, float]) -> None:
        """Replace every component after validating the complete vector.

        Args:
            value_si: Three replacement canonical SI components.

        Raises:
            UnitError: The value is not a finite triple or cannot be displayed.
        """
        if len(value_si) != 3:
            raise UnitError("a quantity vector must contain three values")
        for value in value_si:
            from_si(value, self._unit, kind=self._kind)
        for field, value in zip(self.fields, value_si, strict=True):
            field.set_value_si(value)

    def set_unit(self, unit: str) -> bool:
        """Switch the whole triple atomically while preserving pending edits.

        Args:
            unit: Requested compatible display and implicit input unit.

        Returns:
            True when every component uses the requested unit. False leaves
            all displayed text and units unchanged if any draft is invalid.

        Raises:
            UnitError: The requested unit is incompatible or a value overflows.
        """
        from_si(0.0, unit, kind=self._kind)
        if unit == self._unit:
            return True
        try:
            values = self.value_si()
        except UnitError:
            for field in self.fields:
                field.refresh_tooltip(
                    f"Pending vector input is still interpreted in {self._unit}. "
                    "Complete or correct all components before changing its unit."
                )
            return False
        for value in values:
            from_si(value, unit, kind=self._kind)
        for field in self.fields:
            field.set_unit(unit)
        self._unit = unit
        return True

    def text(self) -> str:
        """Return canonical comma-separated numbers for legacy vector parsers.

        Returns:
            The three SI numbers in round-trip-safe floating-point notation.

        Raises:
            UnitError: Any pending component is invalid or incompatible.
        """
        return ", ".join(repr(value) for value in self.value_si())


class OptionalQuantityEdit(QLineEdit):
    """Edit a physical quantity while retaining an unset or automatic value.

    Args:
        value_si: Initial canonical value, or None to leave the field empty.
        kind: Semantic physical quantity represented by this field.
        unit: Display unit and unit for bare numeric input.
        parent: Optional Qt owner.

    Raises:
        UnitError: The initial unit or finite quantity is invalid.
    """

    def __init__(
        self,
        value_si: float | None,
        *,
        kind: QuantityKind,
        unit: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._kind = kind
        self._unit = unit
        self._canonical_value: float | None = None
        self._cached_text = ""
        self.setMinimumWidth(75)
        self.setAlignment(Qt.AlignmentFlag.AlignRight)
        from_si(0.0, unit, kind=kind)
        self.set_value_si(value_si)

    @property
    def kind(self) -> QuantityKind:
        """Return this field's semantic physical quantity."""
        return self._kind

    @property
    def unit(self) -> str:
        """Return the displayed unit, also used for bare numeric input."""
        return self._unit

    def value_si(self) -> float | None:
        """Read valid pending input without discarding its text.

        Returns:
            The canonical SI magnitude, or None when the field is empty.

        Raises:
            UnitError: Nonempty input is invalid or has incompatible units.
        """
        text = self.text()
        if text != self._cached_text:
            self._canonical_value = (
                parse_quantity(text, kind=self.kind, default_unit=self.unit)
                if text.strip()
                else None
            )
            self._cached_text = text
        return self._canonical_value

    def set_value_si(self, value_si: float | None) -> None:
        """Replace the canonical value, preserving None as an empty field.

        Args:
            value_si: Finite SI value, or None for an unspecified value.

        Raises:
            UnitError: A magnitude is invalid or cannot be displayed.
        """
        text = "" if value_si is None else format_quantity(value_si, self.unit, precision=12)
        if value_si is not None:
            from_si(value_si, self.unit, kind=self.kind)
        self._canonical_value = value_si
        self._cached_text = text
        blocked = self.blockSignals(True)
        try:
            self.setText(text)
        finally:
            self.blockSignals(blocked)
        self.setPlaceholderText(f"Unspecified ({self.unit})")
        self.refresh_tooltip()

    def set_unit(self, unit: str) -> bool:
        """Change display units without losing values or invalid drafts.

        Args:
            unit: Compatible target display unit.

        Returns:
            True when applied; False retains an unfinished draft in its old unit.

        Raises:
            UnitError: The requested unit is incompatible or conversion overflows.
        """
        from_si(0.0, unit, kind=self.kind)
        if unit == self.unit:
            return True
        try:
            value = self.value_si()
        except UnitError:
            self.refresh_tooltip(f"Pending input retains {self.unit} until corrected.")
            return False
        if value is not None:
            from_si(value, unit, kind=self.kind)
        self._unit = unit
        self.set_value_si(value)
        return True

    def refresh_tooltip(self, detail: str | None = None) -> None:
        """Describe the unit and the meaning of a blank value.

        Args:
            detail: Optional draft-preservation guidance.
        """
        prefix = f"{self.accessibleName()} · " if self.accessibleName() else ""
        self.setToolTip(
            prefix
            + (
                detail
                or f"Enter {self.unit} or an explicit compatible suffix. Blank is unspecified."
            )
        )


class QuantityListEdit(QLineEdit):
    """Edit comma-separated physical quantities with one display preference.

    Args:
        values_si: Initial canonical magnitudes; an empty tuple means no values.
        kind: Semantic quantity shared by all entries.
        unit: Display and bare-input unit.
        parent: Optional Qt owner.

    Raises:
        UnitError: A value or display unit is invalid.
    """

    def __init__(
        self,
        values_si: tuple[float, ...],
        *,
        kind: QuantityKind,
        unit: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._kind = kind
        self._unit = unit
        self._canonical_values: tuple[float, ...] = ()
        self._cached_text = ""
        from_si(0.0, unit, kind=kind)
        self.set_value_si(values_si)

    @property
    def kind(self) -> QuantityKind:
        """Return the semantic quantity of all entries."""
        return self._kind

    @property
    def unit(self) -> str:
        """Return the displayed and bare-input unit."""
        return self._unit

    def value_si(self) -> tuple[float, ...]:
        """Parse the list while retaining exact cached values when text is unchanged.

        Returns:
            Canonical SI values, or an empty tuple for a blank field.

        Raises:
            UnitError: Any nonempty entry is invalid or incompatible.
        """
        text = self.text()
        if text != self._cached_text:
            if not text.strip():
                values: tuple[float, ...] = ()
            elif "," in text or ";" in text:
                parts = re.split("[,;]", text)
                values = tuple(
                    parse_quantity(part, kind=self.kind, default_unit=self.unit) for part in parts
                )
            else:
                # Retain the legacy whitespace-only numeric list, while a
                # suffixed entry such as "90 deg" remains one quantity.
                parts = text.split()
                numeric = all(
                    re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", part)
                    for part in parts
                )
                entries = parts if numeric else [text]
                values = tuple(
                    parse_quantity(part, kind=self.kind, default_unit=self.unit) for part in entries
                )
            self._canonical_values = values
            self._cached_text = text
        return self._canonical_values

    def set_value_si(self, values_si: tuple[float, ...]) -> None:
        """Render canonical values without generating user edit signals.

        Args:
            values_si: Replacement finite canonical SI magnitudes.

        Raises:
            UnitError: A magnitude is invalid or cannot be displayed.
        """
        for value in values_si:
            from_si(value, self.unit, kind=self.kind)
        text = ", ".join(format_quantity(value, self.unit, precision=12) for value in values_si)
        self._canonical_values = tuple(values_si)
        self._cached_text = text
        blocked = self.blockSignals(True)
        try:
            self.setText(text)
        finally:
            self.blockSignals(blocked)
        self.setPlaceholderText(f"Comma-separated values ({self.unit})")
        self.refresh_tooltip()

    def set_unit(self, unit: str) -> bool:
        """Convert the whole list atomically while preserving invalid drafts.

        Args:
            unit: Compatible target display unit.

        Returns:
            True when applied; False preserves invalid text and the previous unit.

        Raises:
            UnitError: The target unit is incompatible or conversion overflows.
        """
        from_si(0.0, unit, kind=self.kind)
        if unit == self.unit:
            return True
        try:
            values = self.value_si()
        except UnitError:
            self.refresh_tooltip(f"Pending list retains {self.unit} until corrected.")
            return False
        for value in values:
            from_si(value, unit, kind=self.kind)
        self._unit = unit
        self.set_value_si(values)
        return True

    def refresh_tooltip(self, detail: str | None = None) -> None:
        """Describe the list's displayed units without editing its contents.

        Args:
            detail: Optional invalid-draft guidance.
        """
        prefix = f"{self.accessibleName()} · " if self.accessibleName() else ""
        self.setToolTip(
            prefix + (detail or f"Use comma-separated {self.unit} values or explicit suffixes.")
        )
