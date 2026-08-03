"""Broad-phase candidate detection over connector world positions.

Docking tolerances are millimetres while worlds are metres, so almost every
connector pair is trivially far apart. A uniform spatial hash rejects those in
near-linear time instead of the quadratic scan a naive loop would perform, and
it needs no third-party spatial index.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator

from modsim.core.ids import ConnectorInstanceId
from modsim.core.transforms import Vec3, vec_norm, vec_sub

Cell = tuple[int, int, int]

_MIN_CELL_SIZE = 1e-6


class SpatialHash:
    """Uniform grid of connector positions supporting radius pair queries."""

    __slots__ = ("_cell_size", "_cells", "_points")

    def __init__(self, cell_size: float) -> None:
        if cell_size <= 0.0:
            raise ValueError("cell size must be positive")
        self._cell_size = max(cell_size, _MIN_CELL_SIZE)
        self._cells: dict[Cell, list[ConnectorInstanceId]] = {}
        self._points: dict[ConnectorInstanceId, Vec3] = {}

    def insert(self, key: ConnectorInstanceId, point: Vec3) -> None:
        """Add one connector position to the grid."""
        self._points[key] = point
        self._cells.setdefault(self._cell(point), []).append(key)

    def _cell(self, point: Vec3) -> Cell:
        size = self._cell_size
        return (
            int(point[0] // size),
            int(point[1] // size),
            int(point[2] // size),
        )

    def pairs_within(
        self, radius_m: float
    ) -> Iterator[tuple[ConnectorInstanceId, ConnectorInstanceId]]:
        """Yield each unordered pair of connectors closer than ``radius_m``.

        Pairs are yielded in sorted order so that a docking pass over the same
        world state always evaluates candidates in the same sequence.
        """
        seen: set[tuple[ConnectorInstanceId, ConnectorInstanceId]] = set()
        for key, point in self._points.items():
            base = self._cell(point)
            for offset_x in (-1, 0, 1):
                for offset_y in (-1, 0, 1):
                    for offset_z in (-1, 0, 1):
                        neighbours = self._cells.get(
                            (base[0] + offset_x, base[1] + offset_y, base[2] + offset_z)
                        )
                        if neighbours is None:
                            continue
                        for other in neighbours:
                            if other == key:
                                continue
                            pair = (key, other) if key < other else (other, key)
                            if pair in seen:
                                continue
                            if vec_norm(vec_sub(self._points[other], point)) <= radius_m:
                                seen.add(pair)
        for pair in sorted(seen):
            yield pair


def candidate_pairs(
    positions: Iterable[tuple[ConnectorInstanceId, Vec3]],
    radius_m: float,
) -> tuple[tuple[ConnectorInstanceId, ConnectorInstanceId], ...]:
    """Return every connector pair within ``radius_m``, in deterministic order."""
    if radius_m <= 0.0:
        return ()
    grid = SpatialHash(cell_size=radius_m)
    for key, point in positions:
        grid.insert(key, point)
    return tuple(grid.pairs_within(radius_m))
