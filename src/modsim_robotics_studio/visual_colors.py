"""Categorical visualization colors, independent of Qt and application accents.

Hues retain their meaning across themes; light surfaces use darker ink. Colors
encode categories, never a progression of subtly different accent shades.
"""

from __future__ import annotations

from collections.abc import Iterable

_DARK = {
    "blue": "#5495ff",
    "orange": "#f68b38",
    "violet": "#ac7ef2",
    "green": "#63c66d",
    "magenta": "#ef70bf",
    "cyan": "#28cada",
    "red": "#ff6464",
    "gold": "#e4c329",
    "neutral": "#a4adba",
}
_LIGHT = {
    "blue": "#183b6b",
    "orange": "#ae5200",
    "violet": "#7541b5",
    "green": "#287538",
    "magenta": "#b52b83",
    "cyan": "#007d90",
    "red": "#c12c38",
    "gold": "#8a6b00",
    "neutral": "#5a6573",
}


def visual_colors(theme_id: str) -> dict[str, str]:
    return dict(_LIGHT if theme_id == "light" else _DARK)


def action_colors(theme_id: str) -> dict[str, str]:
    colors = visual_colors(theme_id)
    return {
        "pending": colors["neutral"],
        "waiting": colors["gold"],
        "navigating": colors["blue"],
        "aligning": colors["violet"],
        "approaching": colors["cyan"],
        "holding": colors["orange"],
        "retreating": colors["magenta"],
        "complete": colors["green"],
        "failed": colors["red"],
    }


def categorical_color(index: int, theme_id: str) -> str:
    palette = tuple(visual_colors(theme_id).values())
    return palette[index % len(palette)]


class CategoryColors:
    """Allocate distinct slots to visible categories, preserving surviving IDs.

    Shared by live/target topology and lattice widgets in one runtime window.
    Unlike hashing IDs modulo the palette, separate small assemblies cannot
    accidentally receive identical colors. Larger sets cycle the finite palette;
    labels and topology continue identifying each assembly.
    """

    def __init__(self) -> None:
        self._slots: dict[str, int] = {}

    def prepare(self, identifiers: Iterable[str]) -> None:
        identifiers = sorted(set(identifiers))
        counts = [0] * len(_DARK)
        pending: list[str] = []
        for identifier in identifiers:
            slot = self._slots.get(identifier)
            if slot is not None and counts[slot] == 0:
                counts[slot] += 1
            else:
                pending.append(identifier)
        for identifier in pending:
            slot = min(range(len(counts)), key=counts.__getitem__)
            self._slots[identifier] = slot
            counts[slot] += 1

    def color(self, identifier: str, theme_id: str) -> str:
        return categorical_color(self._slots[identifier], theme_id)
