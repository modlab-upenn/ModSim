"""Connection tooltips for point-to-surface edges (continuous_docking.md section 9).

Qt-free: the tooltip text is built by the presenters and only displayed by the
widgets.
"""

from __future__ import annotations

from modsim.backends.mock import MockBackendAdapter
from modsim.core.ids import ConnectorInstanceId
from modsim.model_views import ModelViewContext, ModelViewFactory
from modsim.model_views.models import CubicLatticeView, SurfaceAttachmentDTO
from modsim.robot_packs.schema import ModelViewSpec, RobotPack, SurfaceRealization
from modsim.runtime.inspection import RuntimeInspectorFrame, build_runtime_inspector_frame
from modsim.runtime.metrics import collect_metrics
from modsim.runtime.session import RuntimeSession
from modsim_studio.connection_details import NO_FACE, face_label, surface_detail_lines
from modsim_studio.runtime_presenter import (
    CubicLatticePresentation,
    RuntimeInspectorPresenter,
    RuntimePresentation,
)
from surface_fixtures import strut_pose, surface_pack, surface_scene

SHELL = ConnectorInstanceId("node_0/shell")
UPPER = ConnectorInstanceId("strut_0/upper")

_ATTACHMENT = SurfaceAttachmentDTO(
    normal_local=(0.0, 0.0, -1.0),
    polar_rad=3.14159,
    azimuth_rad=0.0,
    spin_rad=0.7,
    spin_reference_degenerate=False,
    seat_gap_m=0.0012,
    magnet_height_m=0.002,
)


def _docked_session(example_pack: RobotPack) -> tuple[RobotPack, RuntimeSession]:
    pack = surface_pack(example_pack)
    session = RuntimeSession.create(
        pack, surface_scene(strut_pose((0.0, 0.0, -1.0), gap_m=0.001)), MockBackendAdapter()
    )
    session.request_dock(UPPER, SHELL)
    session.step(0.0)
    assert session.world.connections
    return pack, session


def test_surface_detail_lines_show_the_realisation_and_attachment() -> None:
    lines = surface_detail_lines(SurfaceRealization.DRIVEN_FIXED, _ATTACHMENT)

    assert lines == (
        "Realisation: driven_fixed",
        "Normal (surface frame): (+0.000, +0.000, -1.000)",
        "θ=3.142 rad · φ=0.000 rad · ψ=0.700 rad",
        "Seat gap: 1.200 mm",
        "Magnet height: 2.000 mm",
    )
    assert surface_detail_lines(None, None) == ()


def test_a_surface_face_is_shown_as_a_dash() -> None:
    assert face_label(None) == NO_FACE == "—"
    assert face_label("positive_x") == "positive_x"


def test_the_topology_tooltip_carries_the_surface_fields(example_pack: RobotPack) -> None:
    pack, session = _docked_session(example_pack)
    frame = build_runtime_inspector_frame(session, pack.manifest.model_views[0], ModelViewFactory())

    presented = RuntimeInspectorPresenter().apply_frame(frame)

    assert isinstance(presented, RuntimePresentation)
    (edge,) = presented.edges
    assert edge.realization is SurfaceRealization.DRIVEN_FIXED
    assert edge.attachment is not None
    lines = edge.tooltip.splitlines()
    assert lines[1] == "Committed"
    assert "Realisation: driven_fixed" in lines
    assert "Seat gap: 1.000 mm" in lines


def test_the_lattice_tooltip_shows_a_dash_for_the_surface_face(example_pack: RobotPack) -> None:
    pack, session = _docked_session(example_pack)
    recipe = ModelViewSpec.model_validate(
        {
            "id": "surface_lattice",
            "builder": "cubic_lattice",
            "configuration": {
                "pitch_m": 0.1,
                "position_tolerance_m": 0.005,
                "orientation_tolerance_rad": 0.08726646259971647,
            },
        }
    )
    view = ModelViewFactory().build(recipe, ModelViewContext(pack, session.world))
    assert isinstance(view, CubicLatticeView)
    frame = RuntimeInspectorFrame(
        backend_name="mock",
        view=view,
        metrics=collect_metrics(session.world),
        events=(),
        event_start_sequence=0,
        next_event_sequence=0,
    )

    presented = RuntimeInspectorPresenter().apply_frame(frame)

    assert isinstance(presented, CubicLatticePresentation)
    (edge,) = presented.geometry.edges
    tooltip = edge.tooltip
    assert "None" not in tooltip
    assert f"{SHELL} (—)" in tooltip
    assert "Realisation: driven_fixed" in tooltip
    assert "Seat gap: 1.000 mm" in tooltip
