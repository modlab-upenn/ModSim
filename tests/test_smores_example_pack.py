"""Regression coverage for the committed SMORES-EP Robot Pack."""

from __future__ import annotations

import pytest

from modsim.core.events import AssemblyMerged, DockCandidateDetected, DockCommitted, DockFailed
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.robot_packs import LoadedRobotPack, RobotPackValidator, ValidationProfile
from modsim.runtime.scenarios import stage_docking_pair
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "smores_ep"
MODULE_0 = ModuleInstanceId("smores_ep_0")
MODULE_1 = ModuleInstanceId("smores_ep_1")
BOTTOM_0 = ConnectorInstanceId("smores_ep_0/bottom")
BOTTOM_1 = ConnectorInstanceId("smores_ep_1/bottom")


def test_smores_example_pack_is_simulation_ready(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """The committed pack and every declared local asset form a clean bundle."""
    report = RobotPackValidator().validate(
        smores_loaded_pack,
        profile=ValidationProfile.SIMULATION,
    )

    assert report.issues == ()
    assert smores_loaded_pack.pack.id == MODULE_TYPE
    assert (
        smores_loaded_pack.pack.manifest.metadata["repository_inclusion"]
        == "authorized_for_collaborator_access"
    )
    module = smores_loaded_pack.pack.hardware_catalog.module_types[MODULE_TYPE]
    assert {connector.id for connector in module.connectors} == {
        "bottom",
        "left",
        "pan",
        "right",
    }

    bottom = next(connector for connector in module.connectors if connector.id == "bottom")
    assert bottom.parent_link == "base_link"
    assert bottom.local_pose is not None
    assert bottom.local_pose.xyz_m == pytest.approx((-0.010741577148, 0.0, 0.0))
    assert bottom.docking_axis == pytest.approx((-1.0, 0.0, 0.0))
    assert bottom.approach_axis == pytest.approx((-1.0, 0.0, 0.0))
    assert bottom.metadata["physical_location"] == "rear_base"


@pytest.mark.mujoco
def test_smores_rear_bottom_pair_compiles_and_docks_in_mujoco(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """Compile the actual meshes and commit one weld through the public runtime."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

    session = RuntimeSession.create(
        smores_loaded_pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.3),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
        timestep_s=0.002,
    )
    try:
        assert session.adapter.capabilities().name == "mujoco"
        assert len(session.handles.bodies) == 10
        assert len(session.handles.connector_frames) == 8
        assert (MODULE_0, "base_link") in session.handles.bodies
        assert (MODULE_1, "base_link") in session.handles.bodies

        stage_docking_pair(session, BOTTOM_0, BOTTOM_1, gap_m=0.0)
        session.request_dock(BOTTOM_0, BOTTOM_1)
        events = session.process_docking()

        assert [type(event) for event in events] == [
            DockCandidateDetected,
            DockCommitted,
            AssemblyMerged,
        ]
        assert not [event for event in events if isinstance(event, DockFailed)]
        assert len(session.world.connections) == 1
        assert session.world.assemblies.count == 1
        assert session.world.assemblies.largest_size == 2
        assert session.metrics().docking_success_count == 1
    finally:
        session.shutdown()
