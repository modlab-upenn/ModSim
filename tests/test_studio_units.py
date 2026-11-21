"""Unit-aware authoring controls preserve SI values and unfinished drafts."""

from __future__ import annotations

import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication

from modsim_robotics.units import QuantityKind, UnitError
from modsim_robotics_studio.quantity_widgets import (
    OptionalQuantityEdit,
    QuantityEdit,
    QuantityListEdit,
    QuantityVectorEdit,
)


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


def test_scalar_converts_explicit_input_and_displays_unit(application: QApplication) -> None:
    field = QuantityEdit(0.05, kind=QuantityKind.LENGTH, unit="mm")
    assert field.text() == "50 mm"
    assert field.value_si() == 0.05
    field.setText("2.5 cm")
    assert field.value_si() == 0.025
    assert field.set_unit("m")
    assert field.unit == "m"
    assert field.text().endswith(" m")
    assert field.value_si() == 0.025


def test_implicit_input_uses_field_unit(application: QApplication) -> None:
    field = QuantityEdit(0.0, kind=QuantityKind.LENGTH, unit="mm")
    field.setText("50")
    assert field.value_si() == 0.05
    assert field.set_unit("cm")
    assert field.text() == "5 cm"
    field.setText("50")
    assert field.value_si() == 0.5


def test_unchanged_text_never_round_trips_through_display(application: QApplication) -> None:
    original = math.pi * 1.2345678901234567e-7
    field = QuantityEdit(original, kind=QuantityKind.LENGTH, unit="mm")
    for _ in range(25):
        for unit in ("m", "cm", "mm"):
            assert field.set_unit(unit)
            assert field.value_si() == original


def test_display_is_readable_while_canonical_value_retains_precision(
    application: QApplication,
) -> None:
    readable = QuantityEdit(0.003, kind=QuantityKind.LENGTH, unit="cm")
    assert readable.text() == "0.3 cm"
    assert readable.value_si() == 0.003
    field = QuantityEdit(0.12345678901234567, kind=QuantityKind.LENGTH, unit="m")
    assert field.text() == "0.123456789012 m"
    assert field.value_si() == 0.12345678901234567
    for _ in range(25):
        for unit in ("cm", "mm", "m"):
            assert field.set_unit(unit)
            assert field.value_si() == 0.12345678901234567


def test_component_tooltips_retain_names_and_units_after_refresh(
    application: QApplication,
) -> None:
    editor = QuantityVectorEdit(
        (0.003, 0.0, 0.0), kind=QuantityKind.LENGTH, unit="cm", name="Position"
    )
    assert editor.fields[0].toolTip().startswith("Position X · ")
    assert editor.set_unit("mm")
    assert editor.fields[0].toolTip().startswith("Position X · ")
    assert "mm" in editor.fields[0].toolTip()
    editor.fields[1].setText("unfinished")
    assert not editor.set_unit("m")
    assert editor.fields[1].toolTip().startswith("Position Y · ")
    assert "mm" in editor.fields[1].toolTip()


@pytest.mark.parametrize("draft", ["", "-", "1e-", "unfinished", "50 rad", "nan mm"])
def test_invalid_draft_keeps_text_and_implicit_unit(application: QApplication, draft: str) -> None:
    field = QuantityEdit(0.05, kind=QuantityKind.LENGTH, unit="mm")
    field.setText(draft)
    assert not field.set_unit("cm")
    assert field.text() == draft
    assert field.unit == "mm"
    assert "mm" in field.toolTip()
    with pytest.raises(UnitError):
        field.value_si()
    field.setText("25")
    assert field.value_si() == 0.025


def test_incompatible_unit_does_not_change_scalar(application: QApplication) -> None:
    field = QuantityEdit(0.05, kind=QuantityKind.LENGTH, unit="mm")
    previous = field.text()
    with pytest.raises(UnitError):
        field.set_unit("deg")
    assert field.text() == previous
    assert field.unit == "mm"
    assert field.value_si() == 0.05


def test_overflowing_display_change_is_atomic(application: QApplication) -> None:
    field = QuantityEdit(1e307, kind=QuantityKind.LENGTH, unit="m")
    previous = field.text()
    with pytest.raises(UnitError):
        field.set_unit("mm")
    assert field.text() == previous
    assert field.unit == "m"
    assert field.value_si() == 1e307


def test_programmatic_unit_and_value_changes_emit_no_edit_signals(
    application: QApplication,
) -> None:
    field = QuantityEdit(0.05, kind=QuantityKind.LENGTH, unit="mm")
    edited: list[str] = []
    changed: list[str] = []
    field.textEdited.connect(edited.append)
    field.textChanged.connect(changed.append)
    field.set_unit("cm")
    field.set_value_si(0.125)
    assert edited == []
    assert changed == []
    assert field.value_si() == 0.125


def test_vector_parses_each_component_and_returns_canonical_text(
    application: QApplication,
) -> None:
    editor = QuantityVectorEdit(
        (0.05, 0.0, 0.025), kind=QuantityKind.LENGTH, unit="mm", name="Position"
    )
    assert editor.fields[0].accessibleName() == "Position X"
    assert editor.fields[2].accessibleName() == "Position Z"
    editor.fields[1].setText("2 cm")
    assert editor.value_si() == (0.05, 0.02, 0.025)
    assert editor.text() == "0.05, 0.02, 0.025"
    assert editor.set_unit("cm")
    assert all(field.unit == "cm" for field in editor.fields)
    assert editor.value_si() == (0.05, 0.02, 0.025)


def test_vector_unit_switch_preserves_whole_invalid_draft(application: QApplication) -> None:
    editor = QuantityVectorEdit((0.05, 0.0, 0.025), kind=QuantityKind.LENGTH, unit="mm")
    editor.fields[0].setText("25")
    editor.fields[2].setText("1e-")
    previous = [field.text() for field in editor.fields]
    assert not editor.set_unit("cm")
    assert [field.text() for field in editor.fields] == previous
    assert all(field.unit == "mm" for field in editor.fields)
    assert editor.unit == "mm"
    editor.fields[2].setText("3")
    assert editor.value_si() == (0.025, 0.0, 0.003)
    assert editor.set_unit("cm")
    assert editor.unit == "cm"


def test_vector_setter_validates_every_component_before_rendering(
    application: QApplication,
) -> None:
    editor = QuantityVectorEdit((0.05, 0.0, 0.025), kind=QuantityKind.LENGTH, unit="mm")
    previous = [field.text() for field in editor.fields]
    with pytest.raises(UnitError):
        editor.set_value_si((1.0, math.inf, 2.0))
    assert [field.text() for field in editor.fields] == previous
    assert editor.value_si() == (0.05, 0.0, 0.025)
    editor.set_value_si((0.01, 0.02, 0.03))
    assert editor.value_si() == (0.01, 0.02, 0.03)


def test_angle_editing_preserves_multiple_revolutions(application: QApplication) -> None:
    editor = QuantityVectorEdit(
        (0.0, 0.0, 0.0), kind=QuantityKind.ANGLE, unit="deg", labels="RPY", name="Orientation"
    )
    editor.fields[2].setText("450 deg")
    angle = editor.value_si()[2]
    assert angle == pytest.approx(2.5 * math.pi)
    assert editor.set_unit("rad")
    assert editor.value_si()[2] == angle
    assert editor.set_unit("deg")
    assert editor.value_si()[2] == angle
    assert float(editor.fields[2].text().split()[0]) == pytest.approx(450.0)


def test_vector_display_overflow_does_not_partially_convert(application: QApplication) -> None:
    editor = QuantityVectorEdit((0.05, 1e307, 0.025), kind=QuantityKind.LENGTH, unit="m")
    previous = [field.text() for field in editor.fields]
    with pytest.raises(UnitError):
        editor.set_unit("mm")
    assert [field.text() for field in editor.fields] == previous
    assert all(field.unit == "m" for field in editor.fields)
    assert editor.unit == "m"


def test_optional_quantity_keeps_automatic_blank_and_exact_values(
    application: QApplication,
) -> None:
    field = OptionalQuantityEdit(None, kind=QuantityKind.TIME, unit="s")
    assert field.value_si() is None
    assert field.set_unit("ms")
    assert field.text() == "" and field.value_si() is None
    field.setText("250")
    assert field.value_si() == 0.25
    assert field.set_unit("s")
    assert field.value_si() == 0.25 and field.text() == "0.25 s"
    field.clear()
    assert field.value_si() is None
    field.set_value_si(1.2345678901234567)
    for _ in range(25):
        assert field.set_unit("ms")
        assert field.set_unit("s")
        assert field.value_si() == 1.2345678901234567


def test_optional_quantity_preserves_invalid_pending_text(application: QApplication) -> None:
    field = OptionalQuantityEdit(0.5, kind=QuantityKind.LENGTH, unit="m")
    field.setText("1e-")
    assert not field.set_unit("mm")
    assert field.text() == "1e-" and field.unit == "m"
    field.setText("25 mm")
    assert field.value_si() == 0.025


def test_quantity_list_converts_each_entry_without_losing_turns(application: QApplication) -> None:
    field = QuantityListEdit((), kind=QuantityKind.ANGLE, unit="deg")
    field.setText("0 deg, 90 deg, 450 deg")
    assert field.value_si() == pytest.approx((0.0, math.pi / 2, 2.5 * math.pi))
    original = field.value_si()
    for _ in range(25):
        assert field.set_unit("rad")
        assert field.set_unit("deg")
        assert field.value_si() == original
    field.setText("0 90 180")
    assert field.value_si() == pytest.approx((0.0, math.pi / 2, math.pi))
    field.setText("90 deg")
    assert field.value_si() == pytest.approx((math.pi / 2,))


def test_quantity_list_switch_is_atomic_for_invalid_entry(application: QApplication) -> None:
    field = QuantityListEdit((0.0, math.pi), kind=QuantityKind.ANGLE, unit="rad")
    field.setText("90 deg, unfinished")
    assert not field.set_unit("deg")
    assert field.unit == "rad" and field.text() == "90 deg, unfinished"
    field.clear()
    assert field.value_si() == ()
    assert field.set_unit("deg")
    assert field.text() == ""
