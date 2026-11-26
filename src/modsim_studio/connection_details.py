"""Text for connection tooltips, shared by the graph and lattice presenters.

Pure functions over model-view DTOs, so the text is testable without Qt.
"""

from __future__ import annotations

from modsim.model_views.models import LatticeFace, SurfaceAttachmentDTO
from modsim.robot_packs.schema import SurfaceRealization

NO_FACE = "—"
"""Shown in place of a face label for a surface endpoint, which has none."""


def face_label(face: LatticeFace | None) -> str:
    """Return a lattice face label, or a dash for a surface endpoint."""
    return NO_FACE if face is None else face


def surface_detail_lines(
    realization: SurfaceRealization | None,
    attachment: SurfaceAttachmentDTO | None,
) -> tuple[str, ...]:
    """Return the realisation and live attachment of a point-to-surface edge.

    Point-to-point edges have neither and give no lines.
    """
    lines: list[str] = []
    if realization is not None:
        lines.append(f"Realisation: {realization.value}")
    if attachment is not None:
        x, y, z = attachment.normal_local
        lines.append(f"Normal (surface frame): ({x:+.3f}, {y:+.3f}, {z:+.3f})")
        spin = f"ψ={attachment.spin_rad:.3f} rad"
        if attachment.spin_reference_degenerate:
            spin += " (from y; x is degenerate here)"
        lines.append(
            f"θ={attachment.polar_rad:.3f} rad · φ={attachment.azimuth_rad:.3f} rad · {spin}"
        )
        lines.append(f"Seat gap: {attachment.seat_gap_m * 1000.0:.3f} mm")
        if attachment.magnet_height_m is not None:
            lines.append(f"Magnet height: {attachment.magnet_height_m * 1000.0:.3f} mm")
    return tuple(lines)
