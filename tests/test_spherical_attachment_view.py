"""The spherical_attachment model view (docs/continuous_docking.md 9)."""

from __future__ import annotations

from itertools import combinations
from typing import Any

import pytest

from modsim.core.ids import ConnectorInstanceId, connection_id
from modsim.core.transforms import (
    Transform,
    Vec3,
    angle_between,
    quat_from_axis_angle,
    quat_rotate,
    vec_add,
    vec_cross,
    vec_dot,
    vec_norm,
    vec_normalize,
    vec_sub,
)
from modsim.model_views import (
    ModelViewBuildError,
    ModelViewContext,
    ModelViewFactory,
    SphericalAttachmentBuilder,
    SphericalAttachmentView,
)
from modsim.model_views.spherical import icosphere
from modsim.robot_packs.schema import ModelViewSpec, RobotPack
from modsim.runtime.session import RuntimeSession
from modsim.runtime.surface_slide import SurfaceSlideCommand
from surface_fixtures import CAPACITY, strut_pose, surface_pack, surface_scene

SHELL = ConnectorInstanceId("node_0/shell")


def upper(index: int) -> ConnectorInstanceId:
    return ConnectorInstanceId(f"strut_{index}/upper")


def recipe(**configuration: Any) -> ModelViewSpec:
    return ModelViewSpec(
        id="attachments", builder="spherical_attachment", configuration=configuration
    )


def docked(pack: RobotPack, *normals: Vec3, node_pose: Transform | None = None) -> RuntimeSession:
    session = RuntimeSession.create(
        pack,
        surface_scene(
            *(strut_pose(n, center=_center(node_pose)) for n in normals), node_pose=node_pose
        ),
        "mock",
    )
    for index in range(len(normals)):
        session.request_dock(upper(index), SHELL)
    session.step(0.002)
    assert len(session.world.connections) == len(normals)
    return session


def _center(node_pose: Transform | None) -> Vec3:
    return (0.0, 0.0, 0.0) if node_pose is None else node_pose.translation


def build(session: RuntimeSession, **configuration: Any) -> SphericalAttachmentView:
    view = ModelViewFactory().build(
        recipe(**configuration), ModelViewContext(session.world.pack, session.world)
    )
    assert isinstance(view, SphericalAttachmentView)
    return view


@pytest.fixture
def pack(example_pack: RobotPack) -> RobotPack:
    return surface_pack(example_pack)


# ----------------------------------------------------------------------
# the grid
# ----------------------------------------------------------------------


def test_icosphere_2_has_42_vertices_and_80_outward_triangles() -> None:
    vertices, faces = icosphere(1)

    assert len(vertices) == 42
    assert len(faces) == 80
    assert all(vec_norm(vertex) == pytest.approx(1.0, abs=1e-12) for vertex in vertices)
    edges = {tuple(sorted(pair)) for face in faces for pair in combinations(face, 2)}
    # Euler: V - E + F = 2 for a closed genus-0 triangulation.
    assert len(vertices) - len(edges) + len(faces) == 2
    for a, b, c in faces:
        va, vb, vc = vertices[a], vertices[b], vertices[c]
        outward = vec_dot(vec_cross(vec_sub(vb, va), vec_sub(vc, va)), vec_add(vec_add(va, vb), vc))
        assert outward > 0.0


def test_the_grid_has_twelve_five_fold_and_thirty_six_fold_vertices() -> None:
    vertices, faces = icosphere(1)
    valence = [sum(index in face for face in faces) for index in range(len(vertices))]

    assert valence[:12] == [5] * 12
    assert valence[12:] == [6] * 30


def test_the_view_carries_the_complete_catalog(pack: RobotPack) -> None:
    view = build(docked(pack, (0.0, 0.0, -1.0)))

    assert view.subdivision == "icosphere_2"
    assert len(view.vertices_local) == 42
    assert tuple(face.index for face in view.faces) == tuple(range(80))
    for face in view.faces:
        corners = [view.vertices_local[index] for index in face.vertices]
        assert all(angle_between(face.centroid_local, c) < 0.4 for c in corners)


# ----------------------------------------------------------------------
# assignment
# ----------------------------------------------------------------------


def _inside(view: SphericalAttachmentView, face_index: int, direction: Vec3) -> bool:
    a, b, c = (view.vertices_local[i] for i in view.faces[face_index].vertices)
    return all(vec_dot(vec_cross(p, q), direction) >= -1e-12 for p, q in ((a, b), (b, c), (c, a)))


@pytest.mark.parametrize(
    "normal",
    [
        (0.0, 0.0, -1.0),
        (1.0, 0.0, 0.0),
        vec_normalize((0.3, -0.5, 0.8)),
        vec_normalize((-0.7, 0.1, -0.2)),
    ],
)
def test_each_attachment_lands_in_the_triangle_that_contains_it(
    pack: RobotPack, normal: Vec3
) -> None:
    view = build(docked(pack, normal))
    (surface,) = view.surfaces
    (cell,) = surface.attachments
    face = view.faces[cell.face_index]

    assert cell.normal_local == pytest.approx(normal, abs=1e-9)
    assert _inside(view, cell.face_index, cell.normal_local)
    assert cell.face_residual_rad == pytest.approx(
        angle_between(cell.normal_local, face.centroid_local), abs=1e-12
    )
    nearest = min(range(42), key=lambda i: angle_between(cell.normal_local, view.vertices_local[i]))
    assert cell.vertex_index == nearest
    assert cell.vertex_residual_rad == pytest.approx(
        angle_between(cell.normal_local, view.vertices_local[nearest]), abs=1e-12
    )


def test_an_attachment_at_a_face_centre_has_zero_residual(pack: RobotPack) -> None:
    probe = build(docked(pack, (0.0, 0.0, -1.0)))
    target = probe.faces[37]

    view = build(docked(pack, target.centroid_local))
    (cell,) = view.surfaces[0].attachments

    assert cell.face_index == 37
    assert cell.face_residual_rad == pytest.approx(0.0, abs=1e-9)


def test_an_attachment_on_a_grid_vertex_names_that_vertex(pack: RobotPack) -> None:
    vertices, _ = icosphere(1)
    view = build(docked(pack, vertices[20]))
    (cell,) = view.surfaces[0].attachments

    assert cell.vertex_index == 20
    assert cell.vertex_residual_rad == pytest.approx(0.0, abs=1e-9)
    # A vertex is shared by several faces; the containing face is still one of them.
    assert 20 in view.faces[cell.face_index].vertices


def test_assignment_is_in_the_surface_frame_not_the_world_frame(pack: RobotPack) -> None:
    node_pose = Transform(
        translation=(0.4, -0.2, 0.1), rotation=quat_from_axis_angle((1.0, 1.0, 0.0), 0.9)
    )
    local = vec_normalize((0.2, 0.6, -0.7))
    world_normal = quat_rotate(node_pose.rotation, local)

    moved = build(docked(pack, world_normal, node_pose=node_pose))
    still = build(docked(pack, local))

    assert moved.surfaces[0].attachments[0].normal_local == pytest.approx(local, abs=1e-9)
    assert (
        moved.surfaces[0].attachments[0].face_index == still.surfaces[0].attachments[0].face_index
    )


def test_the_grid_orientation_rotates_the_catalog(pack: RobotPack) -> None:
    session = docked(pack, (0.0, 0.0, -1.0))
    turn = quat_from_axis_angle((0.0, 0.0, 1.0), 0.3)
    plain = build(session)
    rotated = build(session, grid_orientation_local_wxyz=list(turn))

    for before, after in zip(plain.vertices_local, rotated.vertices_local, strict=True):
        assert after == pytest.approx(quat_rotate(turn, before), abs=1e-12)
    assert rotated.grid_orientation_local_wxyz == pytest.approx(turn)


def test_surfaces_and_attachments_are_listed_in_sorted_order(pack: RobotPack) -> None:
    normals = [(0.0, 0.0, -1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)]
    view = build(docked(pack, *normals))
    (surface,) = view.surfaces

    assert surface.connector_id == str(SHELL)
    assert surface.capacity == CAPACITY
    ids = [cell.connection_id for cell in surface.attachments]
    assert ids == sorted(str(connection_id(upper(i), SHELL)) for i in range(4))
    assert {cell.point_connector for cell in surface.attachments} == {
        str(upper(i)) for i in range(4)
    }


def test_an_empty_surface_is_listed_with_no_attachments(pack: RobotPack) -> None:
    session = RuntimeSession.create(
        pack, surface_scene(strut_pose((0.0, 0.0, -1.0), gap_m=0.05)), "mock"
    )
    view = build(session)

    assert [(s.connector_id, s.attachments) for s in view.surfaces] == [(str(SHELL), ())]


def test_a_point_only_pack_has_no_surfaces(example_pack: RobotPack) -> None:
    from modsim.core.scene import SceneSpec

    session = RuntimeSession.create(
        example_pack, SceneSpec.grid("generic_cube", 2, spacing_m=0.1), "mock"
    )
    assert build(session).surfaces == ()


# ----------------------------------------------------------------------
# runtime behaviour
# ----------------------------------------------------------------------


def test_building_the_view_does_not_change_world_state(pack: RobotPack) -> None:
    session = docked(pack, (0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
    before = session.world.revision
    events = len(session.world.event_log)

    build(session)

    assert session.world.revision == before
    assert len(session.world.event_log) == events


def test_a_slide_refreshes_the_view_without_a_topology_change(pack: RobotPack) -> None:
    session = docked(pack, (0.0, 0.0, -1.0))
    factory = ModelViewFactory()
    context = ModelViewContext(session.world.pack, session.world)
    first = factory.build(recipe(), context)
    assert isinstance(first, SphericalAttachmentView)
    topology = session.world.revision.topology_revision

    (connection,) = session.world.connections
    session.set_surface_slide(connection, SurfaceSlideCommand(tangential_velocity_m_s=(0.05, 0.0)))
    for _ in range(10):
        session.step(0.05)
    second = factory.build(recipe(), context)
    assert isinstance(second, SphericalAttachmentView)

    assert session.world.revision.topology_revision == topology
    before = first.surfaces[0].attachments[0]
    after = second.surfaces[0].attachments[0]
    assert angle_between(before.normal_local, after.normal_local) > 0.3
    assert after.face_index != before.face_index


def test_the_factory_reuses_the_view_while_nothing_changes(pack: RobotPack) -> None:
    session = docked(pack, (0.0, 0.0, -1.0))
    factory = ModelViewFactory()
    context = ModelViewContext(session.world.pack, session.world)

    assert factory.build(recipe(), context) is factory.build(recipe(), context)


# ----------------------------------------------------------------------
# configuration
# ----------------------------------------------------------------------


@pytest.mark.parametrize(
    ("configuration", "message"),
    [
        ({"subdivision": "icosphere_3"}, "subdivision"),
        ({"pitch_m": 0.1}, "pitch_m"),
        ({"grid_orientation_local_wxyz": [1.0, 1.0, 0.0, 0.0]}, "unit quaternion"),
        ({"grid_orientation_local_wxyz": [1.0, 0.0, 0.0]}, "grid_orientation"),
    ],
)
def test_invalid_configuration_is_rejected(
    pack: RobotPack, configuration: dict[str, Any], message: str
) -> None:
    session = docked(pack, (0.0, 0.0, -1.0))
    with pytest.raises(ModelViewBuildError, match=message):
        build(session, **configuration)


def test_the_builder_needs_a_world(pack: RobotPack) -> None:
    descriptor = SphericalAttachmentBuilder().descriptor()

    assert descriptor.requires_world
    assert descriptor.view_type == "spherical_attachment"
    assert ModelViewFactory().available(ModelViewContext(pack)) == ()


def test_the_view_round_trips_through_json(pack: RobotPack) -> None:
    view = build(docked(pack, (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)))

    assert SphericalAttachmentView.model_validate_json(view.model_dump_json()) == view
