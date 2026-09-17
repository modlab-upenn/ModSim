"""The mock backend as a kinematic reference for articulated modules.

The differential conformance suite compares the MuJoCo adapter against the mock
as a kinematic reference. That comparison used to be valid only for connectors
on the module root link, because the mock reported every link at the module root
pose and so mislocated any connector carried on an articulated child. The mock
now resolves link frames from the URDF joint origins at zero configuration,
which extends the reference to articulated hardware such as SMORES-EP, whose
``pan``, ``left``, and ``right`` faces sit on the wheels and tilt body.

These tests pin the resolution itself: it leaves single-link modules unchanged
and places SMORES-EP's articulated links off the module root. The differential
check that the resolved frames agree with MuJoCo lives in the SMORES conformance
suite.
"""

from __future__ import annotations

from pathlib import Path

from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.transforms import vec_norm, vec_sub
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime.session import RuntimeSession

SMORES_TYPE = "smores_ep"
SMORES_0 = ModuleInstanceId("smores_ep_0")
ARTICULATED_LINKS = ("tilt_body_1", "left_wheel_1", "right_wheel_1", "front_wheel_1")
SPACING_M = 0.3

# An articulated link is offset from the module root by real hardware distances
# (centimetres), so its resolved frame must sit clearly away from the root.
ARTICULATED_OFFSET_MIN_M = 1e-4


def test_single_link_module_keeps_every_link_at_the_root(example_pack_dir: Path) -> None:
    """A jointless module resolves its one link at the module root, as before."""
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded, SceneSpec.grid("generic_cube", 1, spacing_m=SPACING_M), "mock"
    )
    module = session.world.modules[ModuleInstanceId("generic_cube_0")]

    assert set(module.link_poses) == {"base_link"}
    assert module.link_poses["base_link"].is_close(module.pose)
    assert all(connector.resolved for connector in session.world.connectors.values())


def test_mock_places_smores_articulated_links_off_the_root(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    """SMORES-EP's wheel and tilt links resolve to frames offset from the root."""
    session = RuntimeSession.create(
        smores_loaded_pack, SceneSpec.grid(SMORES_TYPE, 1, spacing_m=SPACING_M), "mock"
    )
    module = session.world.modules[SMORES_0]

    assert {"base_link", *ARTICULATED_LINKS} <= set(module.link_poses)
    root_translation = module.link_poses["base_link"].translation
    for link in ARTICULATED_LINKS:
        offset = vec_norm(vec_sub(module.link_poses[link].translation, root_translation))
        assert offset >= ARTICULATED_OFFSET_MIN_M, (
            f"link '{link}' resolved onto the module root, so forward kinematics did not apply"
        )
    assert all(connector.resolved for connector in session.world.connectors.values())
