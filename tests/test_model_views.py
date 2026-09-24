"""Backend-neutral model-view factory and module-topology graph tests."""

from __future__ import annotations

import math
from collections.abc import Sequence

import pytest
from pydantic import ValidationError

from modsim.core.events import DockCandidateDetected, DockCommitted, UndockCommitted
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
    connection_id,
)
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, BodyState
from modsim.core.state import WorldState
from modsim.core.transforms import Transform, quat_from_axis_angle, quat_multiply, quat_rotate
from modsim.model_views import (
    CubicLatticeView,
    DuplicateModelViewBuilderError,
    ModelViewBuildError,
    ModelViewContext,
    ModelViewFactory,
    ModelViewSourceStamp,
    ModelViewUnavailableError,
    ModuleTopologyGraphBuilder,
    ModuleTopologyGraphView,
)
from modsim.robot_packs.schema import ModelViewMode, ModelViewSpec, RobotPack

MODULE_TYPE = "generic_cube"
ROOT_LINK = "base_link"


def topology_recipe(pack: RobotPack) -> ModelViewSpec:
    return pack.manifest.model_views[0]


def lattice_recipe(**configuration: object) -> ModelViewSpec:
    values: dict[str, object] = {
        "pitch_m": 0.1,
        "position_tolerance_m": 0.005,
        "orientation_tolerance_rad": math.radians(5.0),
    }
    values.update(configuration)
    return ModelViewSpec.model_validate(
        {
            "id": "cubic_lattice",
            "name": "Cubic Lattice",
            "builder": "cubic_lattice",
            "configuration": values,
        }
    )


def world_with_modules(pack: RobotPack, *module_ids: str) -> WorldState:
    placements = tuple(
        ModulePlacement(
            instance_id=ModuleInstanceId(module_id),
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((float(index), 0.0, 0.0)),
        )
        for index, module_id in enumerate(module_ids)
    )
    return WorldState.from_scene(pack, SceneSpec(placements=placements))


def connector(module_id: str, local_id: str) -> ConnectorInstanceId:
    return ConnectorInstanceId(f"{module_id}/{local_id}")


def commit(
    world: WorldState,
    first: ConnectorInstanceId,
    second: ConnectorInstanceId,
    *,
    time_s: float,
    orientation_rad: float,
    orientation_index: int | None,
) -> ConnectionId:
    identifier = connection_id(first, second)
    world.apply(
        DockCommitted(
            time_s=time_s,
            connection_id=identifier,
            connector_a=first,
            connector_b=second,
            constraint_handle=ConstraintHandle(f"constraint:{identifier}"),
            relative_transform=Transform.identity(),
            orientation_rad=orientation_rad,
            orientation_index=orientation_index,
        )
    )
    return identifier


def commit_parallel_connections(world: WorldState, *, reverse: bool = False) -> None:
    actions = (
        (connector("alpha", "front"), connector("beta", "rear"), 0.0, 0),
        (connector("alpha", "rear"), connector("beta", "front"), 3.14, 2),
    )
    selected: Sequence[tuple[ConnectorInstanceId, ConnectorInstanceId, float, int]]
    selected = tuple(reversed(actions)) if reverse else actions
    for first, second, orientation_rad, orientation_index in selected:
        commit(
            world,
            first,
            second,
            time_s=0.25,
            orientation_rad=orientation_rad,
            orientation_index=orientation_index,
        )


def test_factory_lists_built_in_builder_and_filters_unavailable_inputs(
    example_pack: RobotPack,
) -> None:
    factory = ModelViewFactory()
    descriptors = {descriptor.builder: descriptor for descriptor in factory.descriptors()}
    descriptor = descriptors["module_topology_graph"]

    assert set(descriptors) == {"cubic_lattice", "module_topology_graph"}
    assert descriptor.builder == "module_topology_graph"
    assert descriptor.view_type == "module_topology_graph"
    assert descriptor.modes == (ModelViewMode.RUNTIME,)
    assert descriptor.requires_world
    assert factory.available(ModelViewContext(example_pack)) == ()

    world = world_with_modules(example_pack, "alpha")
    assert {item.builder for item in factory.available(ModelViewContext(example_pack, world))} == {
        "cubic_lattice",
        "module_topology_graph",
    }


def test_runtime_builder_reports_a_missing_world_clearly(example_pack: RobotPack) -> None:
    factory = ModelViewFactory()
    with pytest.raises(ModelViewUnavailableError, match="requires inputs unavailable"):
        factory.build(topology_recipe(example_pack), ModelViewContext(example_pack))


def test_topology_view_includes_isolated_modules_in_sorted_order_and_is_json_safe(
    example_pack: RobotPack,
) -> None:
    world = world_with_modules(example_pack, "zeta", "alpha", "middle")
    view = ModelViewFactory().build(
        topology_recipe(example_pack),
        ModelViewContext(example_pack, world),
    )

    assert isinstance(view, ModuleTopologyGraphView)
    assert [node.id for node in view.nodes] == ["alpha", "middle", "zeta"]
    assert [node.assembly_id for node in view.nodes] == [
        "assembly:alpha",
        "assembly:middle",
        "assembly:zeta",
    ]
    assert view.edges == ()
    assert not view.directed
    assert view.allows_parallel_edges
    assert view.schema_version == "0.1"

    document = view.model_dump(mode="json")
    assert document["nodes"][0]["world_position_m"] == [1.0, 0.0, 0.0]
    assert ModuleTopologyGraphView.model_validate_json(view.model_dump_json()) == view


def test_parallel_docked_connections_are_distinct_deterministically_sorted_edges(
    example_pack: RobotPack,
) -> None:
    first_world = world_with_modules(example_pack, "beta", "alpha")
    second_world = world_with_modules(example_pack, "beta", "alpha")
    commit_parallel_connections(first_world)
    commit_parallel_connections(second_world, reverse=True)
    factory = ModelViewFactory()

    first = factory.build(
        topology_recipe(example_pack), ModelViewContext(example_pack, first_world)
    )
    second = factory.build(
        topology_recipe(example_pack), ModelViewContext(example_pack, second_world)
    )

    assert isinstance(first, ModuleTopologyGraphView)
    assert isinstance(second, ModuleTopologyGraphView)
    assert len(first.edges) == 2
    assert [edge.id for edge in first.edges] == sorted(edge.id for edge in first.edges)
    assert {(edge.source, edge.target) for edge in first.edges} == {("alpha", "beta")}
    assert [edge.model_dump(mode="json") for edge in first.edges] == [
        edge.model_dump(mode="json") for edge in second.edges
    ]
    assert [node.assembly_id for node in first.nodes] == ["assembly:alpha", "assembly:alpha"]


def test_undirected_edge_endpoints_do_not_depend_on_dock_command_order(
    example_pack: RobotPack,
) -> None:
    first_world = world_with_modules(example_pack, "alpha", "beta")
    second_world = world_with_modules(example_pack, "alpha", "beta")
    alpha = connector("alpha", "front")
    beta = connector("beta", "rear")
    commit(
        first_world,
        alpha,
        beta,
        time_s=0.25,
        orientation_rad=0.0,
        orientation_index=0,
    )
    commit(
        second_world,
        beta,
        alpha,
        time_s=0.25,
        orientation_rad=0.0,
        orientation_index=0,
    )
    factory = ModelViewFactory()

    first = factory.build(
        topology_recipe(example_pack), ModelViewContext(example_pack, first_world)
    )
    second = factory.build(
        topology_recipe(example_pack), ModelViewContext(example_pack, second_world)
    )

    assert isinstance(first, ModuleTopologyGraphView)
    assert isinstance(second, ModuleTopologyGraphView)
    assert first.edges == second.edges
    assert first.edges[0].source == "alpha"
    assert first.edges[0].connector_a == "alpha/front"


def test_topology_cache_regenerates_after_dock_and_undock(example_pack: RobotPack) -> None:
    world = world_with_modules(example_pack, "alpha", "beta")
    context = ModelViewContext(example_pack, world)
    recipe = topology_recipe(example_pack)
    factory = ModelViewFactory()

    initial = factory.build(recipe, context)
    assert factory.build(recipe, context) is initial

    first = connector("alpha", "front")
    second = connector("beta", "rear")
    identifier = commit(
        world,
        first,
        second,
        time_s=0.5,
        orientation_rad=0.0,
        orientation_index=0,
    )
    docked = factory.build(recipe, context)
    assert docked is not initial
    assert isinstance(docked, ModuleTopologyGraphView)
    assert len(docked.edges) == 1
    assert docked.source.topology_revision == 1

    world.apply(
        UndockCommitted(
            time_s=1.0,
            connection_id=identifier,
            connector_a=first,
            connector_b=second,
        )
    )
    released = factory.build(recipe, context)
    assert released is not docked
    assert isinstance(released, ModuleTopologyGraphView)
    assert released.edges == ()
    assert released.source.topology_revision == 2
    assert [node.assembly_id for node in released.nodes] == ["assembly:alpha", "assembly:beta"]


def test_pose_samples_invalidate_topology_nodes(example_pack: RobotPack) -> None:
    world = world_with_modules(example_pack, "alpha")
    context = ModelViewContext(example_pack, world)
    recipe = topology_recipe(example_pack)
    factory = ModelViewFactory()
    before = factory.build(recipe, context)

    moved_pose = Transform.from_translation((4.0, 5.0, 6.0))
    world.ingest(
        BackendStateSnapshot(
            time_s=2.0,
            link_states={
                ModuleInstanceId("alpha"): {ROOT_LINK: BodyState(pose=moved_pose)},
            },
        )
    )
    after = factory.build(recipe, context)

    assert isinstance(before, ModuleTopologyGraphView)
    assert isinstance(after, ModuleTopologyGraphView)
    assert after is not before
    assert after.nodes[0].world_position_m == (4.0, 5.0, 6.0)
    assert after.source.sample_sequence == 1
    assert after.source.world_time_s == 2.0


class CountingTopologyBuilder(ModuleTopologyGraphBuilder):
    """Topology builder that exposes whether generation actually ran."""

    def __init__(self) -> None:
        self.calls = 0

    def build(
        self,
        recipe: ModelViewSpec,
        context: ModelViewContext,
        source: ModelViewSourceStamp,
    ) -> ModuleTopologyGraphView:
        self.calls += 1
        return super().build(recipe, context, source)


def test_cache_ignores_unrelated_event_changes_but_updates_the_source_stamp(
    example_pack: RobotPack,
) -> None:
    builder = CountingTopologyBuilder()
    factory = ModelViewFactory(include_builtins=False)
    factory.register(builder)
    world = world_with_modules(example_pack, "alpha", "beta")
    context = ModelViewContext(example_pack, world)
    recipe = topology_recipe(example_pack)

    first = factory.build(recipe, context)
    world.apply(
        DockCandidateDetected(
            time_s=0.1,
            connector_a=connector("alpha", "front"),
            connector_b=connector("beta", "rear"),
        )
    )
    second = factory.build(recipe, context)

    assert builder.calls == 1
    assert second is not first
    assert second.source.event_revision == 1
    assert factory.build(recipe, context) is second

    factory.invalidate(recipe.id)
    assert factory.build(recipe, context) is not second
    assert builder.calls == 2


def test_cache_is_bounded_to_the_latest_context_results(example_pack: RobotPack) -> None:
    builder = CountingTopologyBuilder()
    factory = ModelViewFactory(cache_capacity=1, include_builtins=False)
    factory.register(builder)
    recipe = topology_recipe(example_pack)
    first_context = ModelViewContext(example_pack, world_with_modules(example_pack, "alpha"))
    second_context = ModelViewContext(example_pack, world_with_modules(example_pack, "beta"))

    factory.build(recipe, first_context)
    factory.build(recipe, second_context)
    factory.build(recipe, first_context)

    assert builder.calls == 3


def test_duplicate_builder_registration_is_rejected() -> None:
    factory = ModelViewFactory()
    with pytest.raises(DuplicateModelViewBuilderError, match="already registered"):
        factory.register(ModuleTopologyGraphBuilder())


def test_builder_registration_rejects_unreferenceable_ids_and_invalid_modes() -> None:
    class InvalidIdBuilder(ModuleTopologyGraphBuilder):
        @property
        def builder_id(self) -> str:
            return "Invalid-ID"

    class EmptyModesBuilder(ModuleTopologyGraphBuilder):
        @property
        def modes(self) -> tuple[ModelViewMode, ...]:
            return ()

    with pytest.raises(ValueError, match="lowercase snake_case"):
        ModelViewFactory(include_builtins=False).register(InvalidIdBuilder())
    with pytest.raises(ValueError, match="modes must not be empty"):
        ModelViewFactory(include_builtins=False).register(EmptyModesBuilder())


def test_topology_builder_rejects_configuration_it_does_not_implement(
    example_pack: RobotPack,
) -> None:
    recipe = topology_recipe(example_pack).model_copy(
        update={"configuration": {"unknown_option": True}}
    )
    context = ModelViewContext(example_pack, world_with_modules(example_pack, "alpha"))

    with pytest.raises(ModelViewBuildError, match="unknown_option"):
        ModelViewFactory().build(recipe, context)


def test_factory_builds_declared_default_runtime_views(example_pack: RobotPack) -> None:
    context = ModelViewContext(example_pack, world_with_modules(example_pack, "alpha"))
    factory = ModelViewFactory()

    all_runtime = factory.build_all(context)
    defaults = factory.build_default_runtime(context)

    assert len(all_runtime) == 1
    assert defaults == all_runtime
    assert defaults[0].id == "module_topology"


@pytest.mark.parametrize(
    ("configuration", "message"),
    (
        ({}, "pitch_m"),
        (
            {
                "pitch_m": 0.0,
                "position_tolerance_m": 0.001,
                "orientation_tolerance_rad": 0.1,
            },
            "pitch_m",
        ),
        (
            {
                "pitch_m": 0.1,
                "position_tolerance_m": 0.05,
                "orientation_tolerance_rad": 0.1,
            },
            "less than half",
        ),
        (
            {
                "pitch_m": 0.1,
                "position_tolerance_m": 0.001,
                "orientation_tolerance_rad": math.pi / 2.0,
            },
            "orientation_tolerance_rad",
        ),
        (
            {
                "pitch_m": 0.1,
                "position_tolerance_m": 0.001,
                "orientation_tolerance_rad": 0.1,
                "orientation_world_wxyz": [2.0, 0.0, 0.0, 0.0],
            },
            "unit quaternion",
        ),
        (
            {
                "pitch_m": 0.1,
                "position_tolerance_m": 0.001,
                "orientation_tolerance_rad": 0.1,
                "unknown_option": True,
            },
            "unknown_option",
        ),
    ),
)
def test_cubic_lattice_builder_rejects_invalid_configuration(
    example_pack: RobotPack,
    configuration: dict[str, object],
    message: str,
) -> None:
    recipe = ModelViewSpec.model_validate(
        {
            "id": "cubic_lattice",
            "builder": "cubic_lattice",
            "configuration": configuration,
        }
    )
    context = ModelViewContext(example_pack, world_with_modules(example_pack, "alpha"))

    with pytest.raises(ModelViewBuildError, match=message):
        ModelViewFactory().build(recipe, context)


def test_cubic_lattice_quantizes_pose_and_reports_residuals_as_immutable_json(
    example_pack: RobotPack,
) -> None:
    ninety_degrees = quat_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0)
    placements = (
        ModulePlacement(
            instance_id=ModuleInstanceId("aligned"),
            module_type_id=MODULE_TYPE,
            pose=Transform(
                translation=(0.101, -0.199, 0.3),
                rotation=ninety_degrees,
            ),
        ),
        ModulePlacement(
            instance_id=ModuleInstanceId("off_lattice"),
            module_type_id=MODULE_TYPE,
            pose=Transform(
                translation=(0.22, 0.0, 0.0),
                rotation=quat_from_axis_angle((0.0, 0.0, 1.0), 0.2),
            ),
        ),
    )
    world = WorldState.from_scene(example_pack, SceneSpec(placements=placements))
    view = ModelViewFactory().build(
        lattice_recipe(),
        ModelViewContext(example_pack, world),
    )

    assert isinstance(view, CubicLatticeView)
    aligned, off_lattice = view.nodes
    assert aligned.id == "aligned"
    assert aligned.cell == (1, -2, 3)
    assert aligned.pose_residual.translation_m == pytest.approx((0.001, 0.001, 0.0))
    assert aligned.pose_residual.position_m == pytest.approx(math.sqrt(2.0) * 0.001)
    assert aligned.pose_residual.orientation_rad == pytest.approx(0.0, abs=1e-12)
    assert aligned.position_within_tolerance
    assert aligned.orientation_within_tolerance
    assert not aligned.off_lattice
    assert view.orientation_catalog[aligned.orientation_index].id == aligned.orientation_id

    assert off_lattice.cell == (2, 0, 0)
    assert off_lattice.pose_residual.translation_m == pytest.approx((0.02, 0.0, 0.0))
    assert not off_lattice.position_within_tolerance
    assert not off_lattice.orientation_within_tolerance
    assert off_lattice.off_lattice

    document = view.model_dump(mode="json")
    assert document["nodes"][0]["cell"] == [1, -2, 3]
    assert CubicLatticeView.model_validate_json(view.model_dump_json()) == view
    with pytest.raises(ValidationError, match="frozen"):
        aligned.off_lattice = True


def test_cubic_lattice_half_cell_ties_round_away_from_zero(
    example_pack: RobotPack,
) -> None:
    world = WorldState.from_scene(
        example_pack,
        SceneSpec(
            placements=(
                ModulePlacement(
                    instance_id=ModuleInstanceId("negative"),
                    module_type_id=MODULE_TYPE,
                    pose=Transform.from_translation((-0.05, 0.0, 0.0)),
                ),
                ModulePlacement(
                    instance_id=ModuleInstanceId("positive"),
                    module_type_id=MODULE_TYPE,
                    pose=Transform.from_translation((0.05, 0.0, 0.0)),
                ),
            )
        ),
    )
    view = ModelViewFactory().build(
        lattice_recipe(),
        ModelViewContext(example_pack, world),
    )

    assert isinstance(view, CubicLatticeView)
    assert [(node.id, node.cell) for node in view.nodes] == [
        ("negative", (-1, 0, 0)),
        ("positive", (1, 0, 0)),
    ]
    assert all(node.off_lattice for node in view.nodes)


def test_cubic_lattice_classifies_all_24_proper_cube_orientations(
    example_pack: RobotPack,
) -> None:
    seed_world = world_with_modules(example_pack, "seed")
    seed = ModelViewFactory().build(
        lattice_recipe(),
        ModelViewContext(example_pack, seed_world),
    )
    assert isinstance(seed, CubicLatticeView)
    assert len(seed.orientation_catalog) == 24
    assert [item.index for item in seed.orientation_catalog] == list(range(24))
    assert len({item.id for item in seed.orientation_catalog}) == 24
    face_vectors = {
        "positive_x": (1.0, 0.0, 0.0),
        "negative_x": (-1.0, 0.0, 0.0),
        "positive_y": (0.0, 1.0, 0.0),
        "negative_y": (0.0, -1.0, 0.0),
        "positive_z": (0.0, 0.0, 1.0),
        "negative_z": (0.0, 0.0, -1.0),
    }
    for orientation in seed.orientation_catalog:
        assert quat_rotate(orientation.lattice_orientation_wxyz, (1.0, 0.0, 0.0)) == (
            pytest.approx(face_vectors[orientation.local_x_face])
        )
        assert quat_rotate(orientation.lattice_orientation_wxyz, (0.0, 1.0, 0.0)) == (
            pytest.approx(face_vectors[orientation.local_y_face])
        )
        assert quat_rotate(orientation.lattice_orientation_wxyz, (0.0, 0.0, 1.0)) == (
            pytest.approx(face_vectors[orientation.local_z_face])
        )

    placements = tuple(
        ModulePlacement(
            instance_id=ModuleInstanceId(f"cube_{orientation.index:02d}"),
            module_type_id=MODULE_TYPE,
            pose=Transform(
                translation=(orientation.index * 0.1, 0.0, 0.0),
                rotation=orientation.lattice_orientation_wxyz,
            ),
        )
        for orientation in seed.orientation_catalog
    )
    world = WorldState.from_scene(example_pack, SceneSpec(placements=placements))
    view = ModelViewFactory().build(
        lattice_recipe(),
        ModelViewContext(example_pack, world),
    )

    assert isinstance(view, CubicLatticeView)
    assert [node.orientation_index for node in view.nodes] == list(range(24))
    assert all(node.pose_residual.orientation_rad == pytest.approx(0.0) for node in view.nodes)
    assert all(not node.off_lattice for node in view.nodes)


def test_cubic_lattice_applies_a_rotated_translated_lattice_frame(
    example_pack: RobotPack,
) -> None:
    lattice_orientation = quat_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0)
    origin = (1.0, 2.0, 3.0)
    lattice_point = (0.1, -0.2, 0.3)
    rotated_point = quat_rotate(lattice_orientation, lattice_point)
    world_position = (
        origin[0] + rotated_point[0],
        origin[1] + rotated_point[1],
        origin[2] + rotated_point[2],
    )
    cube_orientation = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)
    world_orientation = quat_multiply(lattice_orientation, cube_orientation)
    world = WorldState.from_scene(
        example_pack,
        SceneSpec(
            placements=(
                ModulePlacement(
                    instance_id=ModuleInstanceId("rotated_frame_cube"),
                    module_type_id=MODULE_TYPE,
                    pose=Transform(
                        translation=world_position,
                        rotation=world_orientation,
                    ),
                ),
            )
        ),
    )
    view = ModelViewFactory().build(
        lattice_recipe(
            origin_world_m=list(origin),
            orientation_world_wxyz=list(lattice_orientation),
        ),
        ModelViewContext(example_pack, world),
    )

    assert isinstance(view, CubicLatticeView)
    assert view.nodes[0].cell == (1, -2, 3)
    assert view.nodes[0].pose_residual.position_m == pytest.approx(0.0, abs=1e-12)
    assert view.nodes[0].pose_residual.orientation_rad == pytest.approx(0.0, abs=1e-12)
    assert not view.nodes[0].off_lattice


def test_cubic_lattice_reports_only_position_valid_cell_occupancy_conflicts(
    example_pack: RobotPack,
) -> None:
    placements = (
        ModulePlacement(
            instance_id=ModuleInstanceId("alpha"),
            module_type_id=MODULE_TYPE,
            pose=Transform.identity(),
        ),
        ModulePlacement(
            instance_id=ModuleInstanceId("beta"),
            module_type_id=MODULE_TYPE,
            pose=Transform(
                translation=(0.001, 0.0, 0.0),
                rotation=quat_from_axis_angle((0.0, 0.0, 1.0), 0.2),
            ),
        ),
        ModulePlacement(
            instance_id=ModuleInstanceId("too_far"),
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((0.02, 0.0, 0.0)),
        ),
    )
    world = WorldState.from_scene(example_pack, SceneSpec(placements=placements))
    view = ModelViewFactory().build(
        lattice_recipe(),
        ModelViewContext(example_pack, world),
    )

    assert isinstance(view, CubicLatticeView)
    assert len(view.occupancy_conflicts) == 1
    assert view.occupancy_conflicts[0].cell == (0, 0, 0)
    assert view.occupancy_conflicts[0].module_ids == ("alpha", "beta")
    by_id = {node.id: node for node in view.nodes}
    assert by_id["alpha"].occupancy_conflict
    assert by_id["beta"].occupancy_conflict
    assert by_id["beta"].off_lattice
    assert not by_id["too_far"].occupancy_conflict


def test_cubic_lattice_connections_include_canonical_lattice_face_labels(
    example_pack: RobotPack,
) -> None:
    world = WorldState.from_scene(
        example_pack,
        SceneSpec(
            placements=(
                ModulePlacement(
                    instance_id=ModuleInstanceId("alpha"),
                    module_type_id=MODULE_TYPE,
                    pose=Transform.identity(),
                ),
                ModulePlacement(
                    instance_id=ModuleInstanceId("beta"),
                    module_type_id=MODULE_TYPE,
                    pose=Transform.from_translation((0.1, 0.0, 0.0)),
                ),
            )
        ),
    )
    commit(
        world,
        connector("beta", "rear"),
        connector("alpha", "front"),
        time_s=0.25,
        orientation_rad=0.0,
        orientation_index=0,
    )
    view = ModelViewFactory().build(
        lattice_recipe(),
        ModelViewContext(example_pack, world),
    )

    assert isinstance(view, CubicLatticeView)
    assert len(view.edges) == 1
    edge = view.edges[0]
    assert edge.source == "alpha"
    assert edge.target == "beta"
    assert edge.connector_a == "alpha/front"
    assert edge.connector_b == "beta/rear"
    assert edge.source_face == "positive_x"
    assert edge.target_face == "negative_x"
    assert edge.source_face_residual_rad == pytest.approx(0.0)
    assert edge.target_face_residual_rad == pytest.approx(0.0)


def test_cubic_lattice_cache_regenerates_after_a_pose_sample(
    example_pack: RobotPack,
) -> None:
    world = world_with_modules(example_pack, "alpha")
    context = ModelViewContext(example_pack, world)
    recipe = lattice_recipe()
    factory = ModelViewFactory()
    before = factory.build(recipe, context)

    assert factory.build(recipe, context) is before
    world.ingest(
        BackendStateSnapshot(
            time_s=1.0,
            link_states={
                ModuleInstanceId("alpha"): {
                    ROOT_LINK: BodyState(pose=Transform.from_translation((0.1, 0.0, 0.0)))
                }
            },
        )
    )
    after = factory.build(recipe, context)

    assert isinstance(after, CubicLatticeView)
    assert after is not before
    assert after.nodes[0].cell == (1, 0, 0)
    assert after.source.sample_sequence == 1
