"""MuJoCo backend tests.

Skipped entirely when the optional MuJoCo dependencies are not installed, so a
core-only environment still runs a green suite.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.backends.base import (
    BackendAdapter,
    BackendError,
    SupportsJointCommands,
    SupportsModuleKinematics,
)
from modsim.core.entities import ConnectorLifecycleState, JointCommand
from modsim.core.events import DockCommitted, DockFailed
from modsim.core.ids import ConnectorInstanceId, JointInstanceId, ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, quat_from_axis_angle, vec_norm, vec_sub
from modsim.robot_packs import LoadedRobotPack, RobotPack, RobotPackLoader
from modsim.robot_packs.schema import ControlMode
from modsim.runtime.reconfiguration import stage_docking_assembly_pair
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
from modsim_backend_mujoco.scene import (
    ENVIRONMENT_GEOM_GROUP,
    GROUND_GEOM,
    URDF_COLLISION_GEOM_GROUP,
    MuJoCoSceneError,
    actuator_name,
    body_name,
    build_scene,
    joint_name,
    site_name,
)

pytestmark = pytest.mark.mujoco

MODULE_TYPE = "generic_cube"
SPACING_M = 0.1
CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")
SMORES_MODULE_TYPE = "smores_ep"
SMORES_0 = ModuleInstanceId("smores_ep_0")
SMORES_1 = ModuleInstanceId("smores_ep_1")
LEFT_WHEEL_0 = JointInstanceId("smores_ep_0/joint_left_wheel")
RIGHT_WHEEL_0 = JointInstanceId("smores_ep_0/joint_right_wheel")
RIGHT_WHEEL_1 = JointInstanceId("smores_ep_1/joint_right_wheel")
TILT_0 = JointInstanceId("smores_ep_0/joint_tilt")
PAN_0 = JointInstanceId("smores_ep_0/joint_pan")


@pytest.fixture
def loaded_pack(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


def weightless_session(loaded: LoadedRobotPack, count: int = 2) -> RuntimeSession:
    """Return a MuJoCo session with gravity disabled.

    Gravity off is what makes a MuJoCo run comparable to the kinematic mock, so
    a difference in behaviour is attributable to the adapter rather than to the
    modules falling.
    """
    return RuntimeSession.create(
        loaded,
        SceneSpec.grid(MODULE_TYPE, count, spacing_m=SPACING_M),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )


def smores_ground_session(loaded: LoadedRobotPack, count: int = 1) -> RuntimeSession:
    """Return upright SMORES modules over the physics-oriented MJCF floor."""
    scene = SceneSpec(
        placements=tuple(
            ModulePlacement(
                instance_id=ModuleInstanceId(f"smores_ep_{index}"),
                module_type_id=SMORES_MODULE_TYPE,
                pose=Transform.from_translation((0.3 * index, 0.0, 0.05)),
            )
            for index in range(count)
        )
    )
    return RuntimeSession.create(
        loaded,
        scene,
        "mujoco",
        ground=True,
        timestep_s=0.002,
    )


def drive_smores_wheel_velocities(
    session: RuntimeSession,
    *,
    left_target_rad_s: float,
    right_target_rad_s: float,
    duration_s: float,
) -> None:
    """Run the provisional 40 Hz wheel-velocity loop through effort commands."""
    period_s = 0.025
    proportional_gain_nm_per_rad_s = 0.04
    steps = round(duration_s / period_s)
    for _ in range(steps):
        left = session.world.joint_state(LEFT_WHEEL_0)
        right = session.world.joint_state(RIGHT_WHEEL_0)
        left_effort = max(
            -0.04,
            min(0.04, proportional_gain_nm_per_rad_s * (left_target_rad_s - left.velocity)),
        )
        right_effort = max(
            -0.04,
            min(0.04, proportional_gain_nm_per_rad_s * (right_target_rad_s - right.velocity)),
        )
        session.set_joint_commands(
            (
                JointCommand(LEFT_WHEEL_0, ControlMode.EFFORT, left_effort),
                JointCommand(RIGHT_WHEEL_0, ControlMode.EFFORT, right_effort),
            )
        )
        session.step(period_s)


def yaw_rad(transform: Transform) -> float:
    """Return yaw from a scalar-first quaternion."""
    w, x, y, z = transform.rotation
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ----------------------------------------------------------------------
# scene compilation
# ----------------------------------------------------------------------


def test_each_placement_becomes_its_own_body(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 3, spacing_m=SPACING_M),
        loaded_pack.root,
    )

    # Three independent copies, not one body with stacked name prefixes.
    assert compiled.model.nbody == 4  # world plus three modules
    for index in range(3):
        module = ModuleInstanceId(f"{MODULE_TYPE}_{index}")
        assert (module, "base_link") in compiled.body_ids
        assert compiled.handles.bodies[(module, "base_link")] == body_name(module, "base_link")


def test_smores_mjcf_is_preferred_and_effort_actuators_are_indexed(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """The pack-local dynamics model supplies visuals, contacts, and motors."""
    session = smores_ground_session(smores_loaded_pack)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    model = adapter.model

    assert model.nmesh == 5
    assert model.nu == 4
    assert sum(model.body_mass) == pytest.approx(0.4386367977876995)
    assert len(session.handles.joints) == 4
    assert session.handles.joints[(SMORES_0, "joint_left_wheel")] == joint_name(
        SMORES_0, "joint_left_wheel"
    )
    assert model.actuator(actuator_name(SMORES_0, "joint_left_wheel")).ctrlrange == (
        pytest.approx((-0.04, 0.04))
    )
    assert model.actuator(actuator_name(SMORES_0, "joint_tilt")).ctrlrange == pytest.approx(
        (-0.1, 0.1)
    )

    ground = model.geom(GROUND_GEOM).id
    tire = model.geom(f"{SMORES_0}/left_tire_contact").id
    skid = model.geom(f"{SMORES_0}/rear_skid").id
    face = model.geom(f"{SMORES_0}/bottom_face_proxy").id
    assert model.geom_size[tire, :2] == pytest.approx((0.04, 0.01045))
    assert model.geom_size[skid, 0] == pytest.approx(0.004)
    assert int(model.geom_group[tire]) == URDF_COLLISION_GEOM_GROUP
    assert int(model.geom_contype[tire]) & int(model.geom_conaffinity[ground])
    assert not (int(model.geom_contype[face]) & int(model.geom_conaffinity[ground]))
    assert int(model.geom_contype[face]) & int(model.geom_conaffinity[face])


def test_urdf_visuals_are_retained_while_collision_proxies_stay_physical(
    copied_pack: Path,
) -> None:
    """ModSim must override MuJoCo's visual-discarding URDF default."""
    asset = copied_pack / "assets" / "urdf" / "generic_cube.urdf"
    asset.write_text(
        """<?xml version="1.0"?>
<robot name="generic_cube">
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual name="visual_shell">
      <geometry><box size="0.08 0.06 0.04"/></geometry>
      <material name="red"><color rgba="1 0 0 1"/></material>
    </visual>
    <collision name="collision_proxy">
      <geometry><box size="0.04 0.04 0.04"/></geometry>
    </collision>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    loaded = RobotPackLoader().load(copied_pack)

    compiled = build_scene(
        loaded.pack,
        SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
        loaded.root,
    )
    model = compiled.model
    visual = model.geom(f"{CUBE_0}/visual_shell").id
    collision = model.geom(f"{CUBE_0}/collision_proxy").id

    assert model.ngeom == 2
    assert visual >= 0
    assert collision >= 0
    assert int(model.geom_group[visual]) == 1
    assert int(model.geom_contype[visual]) == 0
    assert int(model.geom_conaffinity[visual]) == 0
    assert int(model.geom_group[collision]) == URDF_COLLISION_GEOM_GROUP
    assert int(model.geom_contype[collision]) == 1
    assert int(model.geom_conaffinity[collision]) == 1


def test_collision_only_urdf_remains_visible_and_physical(copied_pack: Path) -> None:
    """A legacy collision-only model must not vanish with hidden proxy group 3."""
    asset = copied_pack / "assets" / "urdf" / "generic_cube.urdf"
    asset.write_text(
        """<?xml version="1.0"?>
<robot name="generic_cube">
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision name="only_geometry">
      <geometry><box size="0.04 0.04 0.04"/></geometry>
    </collision>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    loaded = RobotPackLoader().load(copied_pack)

    compiled = build_scene(
        loaded.pack,
        SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
        loaded.root,
    )
    model = compiled.model
    collision = model.geom(f"{CUBE_0}/only_geometry").id

    assert model.ngeom == 1
    assert collision >= 0
    assert int(model.geom_group[collision]) == 0
    assert int(model.geom_contype[collision]) == 1
    assert int(model.geom_conaffinity[collision]) == 1


def test_every_connector_becomes_a_site(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=SPACING_M),
        loaded_pack.root,
    )

    assert compiled.model.nsite == 4
    assert FRONT_0 in compiled.site_ids
    assert compiled.handles.connector_frames[FRONT_0] == site_name(CUBE_0, "front")


def test_named_frame_only_connectors_are_refused(loaded_pack: LoadedRobotPack) -> None:
    data = loaded_pack.pack.model_dump(mode="python")
    connector = data["hardware_catalog"]["module_types"][MODULE_TYPE]["connectors"][0]
    connector["frame"] = "front_frame"
    connector["local_pose"] = None
    pack = RobotPack.model_validate(data)

    with pytest.raises(MuJoCoSceneError, match="cannot resolve named connector frames"):
        build_scene(
            pack,
            SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
            loaded_pack.root,
        )


def test_modules_get_six_degrees_of_freedom(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=SPACING_M),
        loaded_pack.root,
    )

    assert compiled.model.nq == 14  # two free joints, seven coordinates each


def test_a_weld_pool_is_reserved_and_inactive(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=SPACING_M),
        loaded_pack.root,
        weld_pool_size=5,
    )

    assert len(compiled.weld_pool) == 5
    assert not compiled.model.eq_active0.any()


def test_a_missing_asset_is_reported_clearly(loaded_pack: LoadedRobotPack, tmp_path: Path) -> None:
    with pytest.raises(MuJoCoSceneError, match="missing on disk"):
        build_scene(
            loaded_pack.pack,
            SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
            tmp_path,
        )


def test_a_non_positive_timestep_is_rejected(loaded_pack: LoadedRobotPack) -> None:
    with pytest.raises(MuJoCoSceneError, match="timestep must be positive"):
        build_scene(
            loaded_pack.pack,
            SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
            loaded_pack.root,
            timestep_s=0.0,
        )


# ----------------------------------------------------------------------
# adapter contract
# ----------------------------------------------------------------------


def test_the_adapter_satisfies_the_backend_protocol() -> None:
    assert isinstance(MuJoCoBackendAdapter(), BackendAdapter)


def test_effort_joint_commands_are_advertised() -> None:
    adapter = MuJoCoBackendAdapter()
    capabilities = adapter.capabilities()

    assert capabilities.supports_joint_commands
    assert capabilities.supported_joint_control_modes == frozenset({ControlMode.EFFORT})
    assert isinstance(adapter, SupportsJointCommands)


def test_loading_without_a_pack_root_is_refused(loaded_pack: LoadedRobotPack) -> None:
    adapter = MuJoCoBackendAdapter()
    with pytest.raises(BackendError, match="needs the Robot Pack directory"):
        adapter.load(loaded_pack.pack, SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M))


def test_using_the_adapter_before_loading_is_refused() -> None:
    adapter = MuJoCoBackendAdapter()
    with pytest.raises(BackendError, match="no scene loaded"):
        adapter.snapshot()


def test_stepping_backwards_is_refused(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    with pytest.raises(BackendError, match="step backwards"):
        session.adapter.step(-1.0)


# ----------------------------------------------------------------------
# physics
# ----------------------------------------------------------------------


def test_modules_fall_under_gravity(loaded_pack: LoadedRobotPack) -> None:
    session = RuntimeSession.create(
        loaded_pack, SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M), "mujoco"
    )
    for _ in range(20):
        session.step(0.01)

    height = session.world.modules[ModuleInstanceId(f"{MODULE_TYPE}_0")].pose.translation[2]
    assert height < -0.01


def test_gravity_can_be_disabled_for_comparison_with_the_mock(
    loaded_pack: LoadedRobotPack,
) -> None:
    session = weightless_session(loaded_pack, count=1)
    for _ in range(20):
        session.step(0.01)

    module = session.world.modules[ModuleInstanceId(f"{MODULE_TYPE}_0")]
    assert vec_norm(module.pose.translation) == pytest.approx(0.0, abs=1e-9)


def test_a_ground_plane_stops_modules_falling(loaded_pack: LoadedRobotPack) -> None:
    scene = SceneSpec(
        placements=(
            ModulePlacement(
                instance_id=CUBE_0,
                module_type_id=MODULE_TYPE,
                pose=Transform.from_translation((0.0, 0.0, 0.4)),
            ),
        )
    )
    session = RuntimeSession.create(loaded_pack, scene, "mujoco", ground=True)
    for _ in range(1500):
        session.step(0.002)

    # A 0.1 m cube resting on a plane at z = 0 sits with its centre at 0.05.
    assert session.world.modules[CUBE_0].pose.translation[2] == pytest.approx(0.05, abs=1e-3)


def test_ground_uses_a_visible_environment_geom_group(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
        loaded_pack.root,
        ground=True,
    )
    ground = compiled.model.geom(GROUND_GEOM).id

    assert ground >= 0
    assert int(compiled.model.geom_group[ground]) == ENVIRONMENT_GEOM_GROUP


def test_smores_provisional_contact_model_settles_upright(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """Two tires and the low-friction rear skid form a stable support triangle."""
    session = smores_ground_session(smores_loaded_pack)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)

    session.step(1.0)

    module = session.world.modules[SMORES_0]
    body_id = adapter.model.body(body_name(SMORES_0, "base_link")).id
    up_z = float(adapter.data.xmat[body_id].reshape(3, 3)[2, 2])
    assert module.pose.translation[2] == pytest.approx(0.04, abs=0.001)
    assert abs(module.pose.translation[0]) < 0.002
    assert abs(module.pose.translation[1]) < 0.002
    assert up_z > 0.999
    assert adapter.data.ncon >= 3


def test_smores_provisional_pan_tilt_drivetrain_is_stable_at_40_hz(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """Reflected armature makes the provisional sampled PD hold well behaved."""
    session = smores_ground_session(smores_loaded_pack)

    for _ in range(20):
        commands: list[JointCommand] = []
        for joint in (TILT_0, PAN_0):
            state = session.world.joint_state(joint)
            effort = max(-0.1, min(0.1, -state.position - 0.02 * state.velocity))
            commands.append(JointCommand(joint, ControlMode.EFFORT, effort))
        session.set_joint_commands(commands)
        session.step(0.025)

    module = session.world.modules[SMORES_0]
    tilt = session.world.joint_state(TILT_0)
    pan = session.world.joint_state(PAN_0)
    assert tilt.position == pytest.approx(0.02, abs=0.01)
    assert abs(tilt.velocity) < 0.1
    assert abs(pan.position) < 0.01
    assert abs(pan.velocity) < 0.1
    assert module.pose.translation[2] == pytest.approx(0.04, abs=0.002)


def test_smores_joint_commands_are_atomic_persistent_and_isolated(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """Semantic joint IDs map to the correct per-instance motor and snapshot."""
    session = smores_ground_session(smores_loaded_pack, count=2)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    assert isinstance(adapter, SupportsJointCommands)

    session.set_joint_commands(
        (
            JointCommand(LEFT_WHEEL_0, ControlMode.EFFORT, 0.01),
            JointCommand(RIGHT_WHEEL_1, ControlMode.EFFORT, -0.02),
        )
    )
    controls_before_bad_batch = adapter.data.ctrl.copy()
    with pytest.raises(BackendError, match="has no MuJoCo effort actuator"):
        adapter.set_joint_commands(
            (
                JointCommand(RIGHT_WHEEL_0, ControlMode.EFFORT, 0.03),
                JointCommand(
                    JointInstanceId("smores_ep_1/not_a_joint"),
                    ControlMode.EFFORT,
                    0.01,
                ),
            )
        )
    assert adapter.data.ctrl == pytest.approx(controls_before_bad_batch)

    session.step(0.01)

    assert session.world.joint_state(LEFT_WHEEL_0).effort == pytest.approx(0.01)
    assert session.world.joint_state(RIGHT_WHEEL_1).effort == pytest.approx(-0.02)
    assert session.world.joint_state(RIGHT_WHEEL_0).effort == pytest.approx(0.0)
    assert session.world.joint_state(LEFT_WHEEL_0).velocity > 0.0
    assert session.world.joint_state(RIGHT_WHEEL_1).velocity < 0.0

    session.clear_joint_commands((LEFT_WHEEL_0,))
    left_actuator = adapter.model.actuator(actuator_name(SMORES_0, "joint_left_wheel")).id
    right_1_actuator = adapter.model.actuator(actuator_name(SMORES_1, "joint_right_wheel")).id
    assert adapter.data.ctrl[left_actuator] == 0.0
    assert adapter.data.ctrl[right_1_actuator] == pytest.approx(-0.02)
    session.clear_joint_commands()
    assert adapter.data.ctrl == pytest.approx(0.0)


def test_smores_provisional_effort_loop_drives_forward_on_tire_contacts(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """Equal positive wheel targets produce +X differential-drive motion."""
    session = smores_ground_session(smores_loaded_pack)
    session.step(1.0)
    start = session.world.modules[SMORES_0].pose

    drive_smores_wheel_velocities(
        session,
        left_target_rad_s=1.0,
        right_target_rad_s=1.0,
        duration_s=2.0,
    )

    finish = session.world.modules[SMORES_0].pose
    assert finish.translation[0] - start.translation[0] > 0.01
    assert abs(finish.translation[1] - start.translation[1]) < 0.002
    assert abs(yaw_rad(finish) - yaw_rad(start)) < 0.01
    assert finish.translation[2] == pytest.approx(0.04, abs=0.002)


def test_smores_provisional_effort_loop_turns_with_opposite_wheel_targets(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """Opposite wheel targets yaw the module without a root-pose write."""
    session = smores_ground_session(smores_loaded_pack)
    session.step(1.0)
    start = session.world.modules[SMORES_0].pose

    drive_smores_wheel_velocities(
        session,
        left_target_rad_s=1.0,
        right_target_rad_s=-1.0,
        duration_s=2.0,
    )

    finish = session.world.modules[SMORES_0].pose
    planar_displacement = math.hypot(
        finish.translation[0] - start.translation[0],
        finish.translation[1] - start.translation[1],
    )
    assert abs(yaw_rad(finish) - yaw_rad(start)) > 0.05
    # The anisotropic tire model now permits the small lateral scrub a real
    # skid-steer pivot needs; the rear support point makes the turn trace a
    # short arc rather than rotating about an exact mathematical point.
    assert planar_displacement < 0.02
    assert finish.translation[2] == pytest.approx(0.04, abs=0.002)


def test_without_a_ground_plane_modules_keep_falling(loaded_pack: LoadedRobotPack) -> None:
    session = RuntimeSession.create(
        loaded_pack, SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M), "mujoco"
    )
    for _ in range(500):
        session.step(0.002)

    assert session.world.modules[CUBE_0].pose.translation[2] < -0.1


def test_module_twist_is_expressed_in_world_coordinates(loaded_pack: LoadedRobotPack) -> None:
    """A free joint stores angular velocity in the body frame; the API does not.

    The module is spawned rotated so that a body-frame leak would show up as a
    velocity about the wrong world axis.
    """
    rotated = SceneSpec(
        placements=(
            ModulePlacement(
                instance_id=CUBE_0,
                module_type_id=MODULE_TYPE,
                pose=Transform(rotation=quat_from_axis_angle((1.0, 0.0, 0.0), math.pi / 2)),
            ),
        )
    )
    session = RuntimeSession.create(loaded_pack, rotated, "mujoco", gravity=(0.0, 0.0, 0.0))
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    assert isinstance(adapter, SupportsModuleKinematics)

    adapter.set_module_twist(CUBE_0, linear_m_s=(0.1, 0.0, 0.0), angular_rad_s=(0.0, 0.0, 2.0))
    body = adapter.snapshot().link_states[CUBE_0]["base_link"]

    assert body.linear_velocity_m_s == pytest.approx((0.1, 0.0, 0.0), abs=1e-9)
    assert body.angular_velocity_rad_s == pytest.approx((0.0, 0.0, 2.0), abs=1e-9)


def test_a_driven_module_travels_at_the_commanded_speed(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.set_module_twist(CUBE_0, linear_m_s=(0.1, 0.0, 0.0))
    start = session.world.modules[CUBE_0].pose.translation

    for _ in range(500):
        session.step(0.002)

    travelled = vec_norm(vec_sub(session.world.modules[CUBE_0].pose.translation, start))
    assert travelled == pytest.approx(0.1, abs=1e-3)


def test_an_applied_wrench_accelerates_a_module(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.apply_module_wrench(CUBE_0, force_n=(1.0, 0.0, 0.0))

    for _ in range(500):
        session.step(0.002)
    speed = vec_norm(session.world.modules[CUBE_0].linear_velocity_m_s)

    # 1 N on 1 kg for 1 s.
    assert speed == pytest.approx(1.0, rel=0.05)

    adapter.clear_wrenches()
    for _ in range(500):
        session.step(0.002)
    assert vec_norm(session.world.modules[CUBE_0].linear_velocity_m_s) == pytest.approx(
        speed, rel=0.05
    )


def test_driving_an_unknown_module_is_refused(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    with pytest.raises(BackendError, match="no free joint"):
        adapter.set_module_twist(ModuleInstanceId("nobody"))


def test_simulation_time_advances(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    for _ in range(10):
        session.step(0.01)

    assert session.world.time_s == pytest.approx(0.1, abs=1e-6)


def test_connector_frames_come_from_measured_sites(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    snapshot = session.adapter.snapshot()

    # The adapter populates the optional measured-frame channel, which
    # WorldState prefers over recomposing link pose with the authored offset.
    assert snapshot.connector_frames[CUBE_0]["front"].translation == pytest.approx(
        (0.05, 0.0, 0.0), abs=1e-9
    )


def test_a_moved_module_carries_its_connectors(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.set_module_pose(CUBE_0, Transform.from_translation((0.0, 0.3, 0.0)))
    session.step(0.0)

    front = session.world.connector(FRONT_0)
    assert front.world_pose.translation == pytest.approx((0.05, 0.3, 0.0), abs=1e-6)


def test_a_spinning_module_reports_connector_velocity(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    body = adapter.model.body_jntadr[1]
    adapter.data.qvel[adapter.model.jnt_dofadr[body] + 5] = 4.0
    session.step(0.01)

    front = session.world.connector(FRONT_0)
    # The connector sits away from the spin axis, so the lever-arm term shows up.
    assert vec_norm(front.world_velocity_m_s) > 0.05


# ----------------------------------------------------------------------
# docking through runtime weld constraints
# ----------------------------------------------------------------------


def test_a_commanded_dock_commits_under_physics(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    assert session.adapter.capabilities().supports_runtime_constraints

    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    assert any(isinstance(event, DockCommitted) for event in events)
    assert not [event for event in events if isinstance(event, DockFailed)]
    assert len(session.world.connections) == 1
    assert session.world.assemblies.count == 1
    assert session.world.connector(FRONT_0).lifecycle_state is ConnectorLifecycleState.DOCKED


def test_nominal_snap_preserves_root_to_articulated_connector_transform(
    loaded_pack: LoadedRobotPack,
    tmp_path: Path,
) -> None:
    """Snapping a child-link connector must move the module root coherently."""
    root = tmp_path / "articulated_pack"
    asset = root / "assets" / "urdf" / "generic_cube.urdf"
    asset.parent.mkdir(parents=True)
    asset.write_text(
        """<?xml version="1.0"?>
<robot name="articulated_connector">
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual><geometry><box size="0.04 0.04 0.04"/></geometry></visual>
    <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
  </link>
  <link name="tool_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual><geometry><box size="0.02 0.02 0.02"/></geometry></visual>
    <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
  </link>
  <joint name="tool_joint" type="revolute">
    <parent link="base_link"/><child link="tool_link"/>
    <origin xyz="0.04 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    data = loaded_pack.pack.model_dump(mode="python")
    module = data["hardware_catalog"]["module_types"][MODULE_TYPE]
    module["joints"] = [
        {
            "id": "tool_joint",
            "source_joint_name": "tool_joint",
            "type": "revolute",
            "parent_link": "base_link",
            "child_link": "tool_link",
            "axis": (0.0, 0.0, 1.0),
            "control_modes": (),
            "limits": {
                "lower_position_rad": -1.0,
                "upper_position_rad": 1.0,
                "max_velocity_rad_per_s": 1.0,
                "max_effort_nm": 1.0,
            },
        }
    ]
    for connector in module["connectors"]:
        connector["parent_link"] = "tool_link"
        connector["local_pose"]["xyz_m"] = (
            0.01 if connector["id"] == "front" else -0.01,
            0.0,
            0.0,
        )
    data["hardware_catalog"]["connector_types"]["fixed_face"]["docking_policy"] = {
        "alignment": "nominal"
    }
    mapping = data["backend_mappings"]["urdf"]["module_types"][MODULE_TYPE]
    mapping["link_map"]["tool_link"] = "tool_link"
    mapping["joint_map"]["tool_joint"] = "tool_joint"
    articulated = LoadedRobotPack(
        root=root,
        manifest_path=root / "robot_pack.yaml",
        pack=RobotPack.model_validate(data),
    )
    session = RuntimeSession.create(
        articulated,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.5),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    moving_front = ConnectorInstanceId("generic_cube_1/front")
    stage_docking_assembly_pair(session, FRONT_0, moving_front, gap_m=0.0)
    root_before = session.world.modules[CUBE_1].pose

    session.request_dock(FRONT_0, moving_front)
    events = session.step(0.0)
    assert any(isinstance(event, DockCommitted) for event in events)
    session.step(0.0)

    root_after = session.world.modules[CUBE_1].pose
    assert root_after.is_close(root_before, position_tolerance_m=1e-8)


def test_a_compliant_connection_is_refused_instead_of_rigidly_welded(
    loaded_pack: LoadedRobotPack,
) -> None:
    data = loaded_pack.pack.model_dump(mode="python")
    connector_type = data["hardware_catalog"]["connector_types"]["fixed_face"]
    connector_type["physical_connection"] = {
        "constraint": "compliant",
        "compliance": {
            "translational_stiffness_n_per_m": 1000.0,
            "rotational_stiffness_nm_per_rad": 100.0,
        },
    }
    compliant = loaded_pack.with_pack(RobotPack.model_validate(data))
    session = weightless_session(compliant)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)

    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    failures = [event for event in events if isinstance(event, DockFailed)]
    assert len(failures) == 1
    assert "supports fixed and hinge physical connections" in failures[0].detail
    assert not session.world.connections
    assert adapter.weld_pool.in_use == 0


def test_detection_runs_before_any_command(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    proposals = session.proposals()

    assert proposals
    assert proposals[0].compatibility.compatible
    assert proposals[0].acceptance.satisfied
    # Nothing latches without a command, because the example pack's connector
    # type is not auto-latching.
    assert session.world.connector(FRONT_0).lifecycle_state is not ConnectorLifecycleState.DOCKED


def test_connector_geometry_matches_the_authored_layout(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    front = session.world.connector(FRONT_0)
    rear = session.world.connector(REAR_1)

    assert vec_norm(vec_sub(front.world_pose.translation, rear.world_pose.translation)) == (
        pytest.approx(0.0, abs=1e-9)
    )
    assert front.world_docking_axis == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    assert rear.world_docking_axis == pytest.approx((-1.0, 0.0, 0.0), abs=1e-9)
