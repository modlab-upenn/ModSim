"""Regression coverage for the committed 3D M-Blocks Robot Pack."""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim.core.entities import JointCommand
from modsim.core.events import AssemblyMerged, DockCandidateDetected, DockCommitted, DockFailed
from modsim.core.ids import ConnectorInstanceId, JointInstanceId, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.state import WorldState
from modsim.model_views import CubicLatticeView, ModelViewContext, ModelViewFactory
from modsim.robot_packs import RobotPackLoader, RobotPackValidator, ValidationProfile
from modsim.robot_packs.schema import AlignmentMode, ControlMode, PhysicalConstraintType
from modsim.runtime.reconfiguration import stage_docking_assembly_pair
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "mblocks_3d"
MODULE_0 = ModuleInstanceId("mblocks_3d_0")
MODULE_1 = ModuleInstanceId("mblocks_3d_1")
POS_X_0 = ConnectorInstanceId("mblocks_3d_0/pos_x")
POS_X_1 = ConnectorInstanceId("mblocks_3d_1/pos_x")
FLYWHEEL_0 = JointInstanceId("mblocks_3d_0/flywheel_spin")
PACK_ROOT = Path(__file__).parents[1] / "examples" / "robot_packs" / MODULE_TYPE


def test_mblocks_example_pack_is_simulation_ready() -> None:
    """The CAD-derived pack is self-contained and explicit about fidelity."""
    loaded = RobotPackLoader().load(PACK_ROOT)
    report = RobotPackValidator().validate(loaded, profile=ValidationProfile.SIMULATION)

    assert report.issues == ()
    assert loaded.pack.id == MODULE_TYPE
    assert (
        loaded.pack.manifest.metadata["fidelity"] == "cad_visual_paper_reference_one_plane_physics"
    )
    assert loaded.pack.manifest.metadata["nominal_lattice_pitch_m"] == pytest.approx(0.05)
    assert loaded.pack.manifest.metadata["nominal_physics_mass_kg"] == pytest.approx(0.15)
    assert loaded.pack.manifest.metadata["flywheel_axial_inertia_kg_m2"] == pytest.approx(8.4e-6)
    assert loaded.pack.manifest.metadata["underactuated_three_plane_carrier_modeled"] is False
    assert loaded.pack.manifest.assets.mujoco == {MODULE_TYPE: "assets/mujoco/mblocks_3d.xml"}

    module = loaded.pack.hardware_catalog.module_types[MODULE_TYPE]
    assert module.mass_kg == pytest.approx(0.15)
    assert {connector.id for connector in module.connectors} == {
        "pos_x",
        "neg_x",
        "pos_y",
        "neg_y",
        "pos_z",
        "neg_z",
        "edge_pos_x_pos_z",
        "edge_pos_x_neg_z",
        "edge_neg_x_pos_z",
        "edge_neg_x_neg_z",
        "edge_pos_x_pos_z_from_pos_x",
        "edge_pos_x_neg_z_from_pos_x",
        "edge_neg_x_pos_z_from_neg_x",
        "edge_neg_x_neg_z_from_neg_x",
    }

    expected = {
        "pos_x": ((0.025, 0.0, 0.0), (1.0, 0.0, 0.0)),
        "neg_x": ((-0.025, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        "pos_y": ((0.0, 0.025, 0.0), (0.0, 1.0, 0.0)),
        "neg_y": ((0.0, -0.025, 0.0), (0.0, -1.0, 0.0)),
        "pos_z": ((0.0, 0.0, 0.025), (0.0, 0.0, 1.0)),
        "neg_z": ((0.0, 0.0, -0.025), (0.0, 0.0, -1.0)),
        "edge_pos_x_pos_z": ((0.025, 0.0, 0.025), (0.0, 0.0, 1.0)),
        "edge_pos_x_neg_z": ((0.025, 0.0, -0.025), (0.0, 0.0, -1.0)),
        "edge_neg_x_pos_z": ((-0.025, 0.0, 0.025), (0.0, 0.0, 1.0)),
        "edge_neg_x_neg_z": ((-0.025, 0.0, -0.025), (0.0, 0.0, -1.0)),
        "edge_pos_x_pos_z_from_pos_x": ((0.025, 0.0, 0.025), (1.0, 0.0, 0.0)),
        "edge_pos_x_neg_z_from_pos_x": ((0.025, 0.0, -0.025), (1.0, 0.0, 0.0)),
        "edge_neg_x_pos_z_from_neg_x": ((-0.025, 0.0, 0.025), (-1.0, 0.0, 0.0)),
        "edge_neg_x_neg_z_from_neg_x": ((-0.025, 0.0, -0.025), (-1.0, 0.0, 0.0)),
    }
    for connector in module.connectors:
        assert connector.local_pose is not None
        position, axis = expected[connector.id]
        assert connector.local_pose.xyz_m == pytest.approx(position)
        assert connector.docking_axis == pytest.approx(axis)
        assert connector.approach_axis == pytest.approx(axis)

    edge_connectors = {
        connector.id: connector
        for connector in module.connectors
        if connector.id.startswith("edge_")
    }
    assert {
        connector_id: connector.metadata["directed_pair"]
        for connector_id, connector in edge_connectors.items()
    } == {
        "edge_pos_x_pos_z": "edge_pos_x_neg_z",
        "edge_pos_x_neg_z": "edge_pos_x_pos_z",
        "edge_neg_x_pos_z": "edge_neg_x_neg_z",
        "edge_neg_x_neg_z": "edge_neg_x_pos_z",
        "edge_pos_x_pos_z_from_pos_x": "edge_neg_x_pos_z_from_neg_x",
        "edge_pos_x_neg_z_from_pos_x": "edge_neg_x_neg_z_from_neg_x",
        "edge_neg_x_pos_z_from_neg_x": "edge_pos_x_pos_z_from_pos_x",
        "edge_neg_x_neg_z_from_neg_x": "edge_pos_x_neg_z_from_pos_x",
    }
    assert all(connector.metadata["edge_axis"] == "pos_y" for connector in edge_connectors.values())
    assert all(
        connector.local_pose is not None
        and connector.local_pose.rpy_rad == pytest.approx((0.0, 0.0, 0.0))
        for connector in edge_connectors.values()
    )

    joints = {joint.id: joint for joint in module.joints}
    flywheel = joints["flywheel_spin"]
    assert flywheel.axis == pytest.approx((0.0, 1.0, 0.0))
    assert flywheel.control_modes == (ControlMode.EFFORT,)
    assert flywheel.limits is not None
    assert flywheel.limits.max_velocity_rad_per_s == pytest.approx(2094.3951023931954)
    assert flywheel.limits.max_effort_nm == pytest.approx(2.6)

    connector_type = loaded.pack.hardware_catalog.connector_types["mblock_magnetic_face"]
    assert connector_type.gender.value == "genderless"
    assert connector_type.compatible_with == ("mblock_magnetic_face",)
    assert connector_type.effective_docking_policy.auto_latch
    assert connector_type.effective_docking_policy.alignment is AlignmentMode.MEASURED
    assert connector_type.supports_undocking

    hinge_type = loaded.pack.hardware_catalog.connector_types["mblock_magnetic_edge_hinge"]
    assert hinge_type.physical_connection is not None
    assert hinge_type.physical_connection.constraint is PhysicalConstraintType.HINGE
    assert hinge_type.physical_connection.hinge is not None
    assert hinge_type.physical_connection.hinge.axis == pytest.approx((0.0, 1.0, 0.0))
    assert hinge_type.physical_connection.hinge.anchor_separation_m == pytest.approx(0.04)
    assert hinge_type.acceptance_region is not None
    assert hinge_type.acceptance_region.position_tolerance_m == pytest.approx(0.003)
    assert not hinge_type.effective_docking_policy.auto_latch


def test_mblocks_lattice_recipe_builds_origin_module() -> None:
    """The pack's lattice recipe matches the built-in cubic-lattice contract."""
    loaded = RobotPackLoader().load(PACK_ROOT)
    recipe = next(
        recipe for recipe in loaded.pack.manifest.model_views if recipe.id == "mblocks_lattice"
    )
    world = WorldState.from_scene(
        loaded.pack,
        SceneSpec.grid(MODULE_TYPE, 1, spacing_m=0.05),
    )

    view = ModelViewFactory().build(recipe, ModelViewContext(loaded.pack, world))

    assert isinstance(view, CubicLatticeView)
    assert view.pitch_m == pytest.approx(0.05)
    assert view.origin_world_m == pytest.approx((0.0, 0.0, 0.0))
    assert len(view.orientation_catalog) == 24
    assert len(view.nodes) == 1
    assert view.nodes[0].cell == (0, 0, 0)
    assert not view.nodes[0].off_lattice
    assert not view.nodes[0].occupancy_conflict
    assert view.occupancy_conflicts == ()


@pytest.mark.mujoco
def test_mblocks_faces_compile_and_dock_in_mujoco() -> None:
    """Compile the actual meshes and commit one measured magnetic-face weld."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    loaded = RobotPackLoader().load(PACK_ROOT)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
        timestep_s=0.0005,
    )
    try:
        assert len(session.handles.bodies) == 6
        assert len(session.handles.connector_frames) == 28
        edge_ids = {
            connector.id
            for connector in loaded.pack.hardware_catalog.module_types[MODULE_TYPE].connectors
            if connector.id.startswith("edge_")
        }
        expected_edge_frames = {
            ConnectorInstanceId(f"{module_id}/{connector_id}")
            for module_id in (MODULE_0, MODULE_1)
            for connector_id in edge_ids
        }
        assert expected_edge_frames <= session.handles.connector_frames.keys()
        assert (MODULE_0, "actuator_carrier_link_1") in session.handles.bodies
        assert (MODULE_1, "flywheel_link_1") in session.handles.bodies

        stage_docking_assembly_pair(session, POS_X_0, POS_X_1, gap_m=0.0)
        session.request_dock(POS_X_0, POS_X_1)
        events = session.process_docking()

        assert any(isinstance(event, DockCandidateDetected) for event in events)
        assert [
            type(event) for event in events if not isinstance(event, DockCandidateDetected)
        ] == [DockCommitted, AssemblyMerged]
        assert not [event for event in events if isinstance(event, DockFailed)]
        assert len(session.world.connections) == 1
        assert session.world.assemblies.count == 1
    finally:
        session.shutdown()


@pytest.mark.mujoco
def test_mblocks_flywheel_effort_generates_opposing_body_rotation() -> None:
    """The +Y one-plane flywheel transfers angular momentum internally."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    loaded = RobotPackLoader().load(PACK_ROOT)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid(MODULE_TYPE, 1, spacing_m=0.05),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
        timestep_s=0.0005,
    )
    try:
        session.set_joint_commands((JointCommand(FLYWHEEL_0, ControlMode.EFFORT, 0.03),))
        for _ in range(40):
            session.step(0.0005)

        module = session.world.modules[MODULE_0]
        flywheel = module.joint_states["flywheel_spin"]
        exported_axis = (0.0, 1.0, 0.0)
        body_along_flywheel_axis = sum(
            angular * axis
            for angular, axis in zip(module.angular_velocity_rad_s, exported_axis, strict=True)
        )

        assert flywheel.velocity > 0.0
        assert body_along_flywheel_axis < 0.0
    finally:
        session.shutdown()
