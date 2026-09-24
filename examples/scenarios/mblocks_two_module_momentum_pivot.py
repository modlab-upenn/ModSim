"""Two-module M-Blocks momentum-pivot physics bootstrap.

``moving_block`` begins on top of ``support_block``.  At runtime its initial
``neg_z`` face bond is replaced by the shared +Y edge hinge.  Braking the
positive-Y flywheel transfers angular momentum to the shell, which rolls 180
degrees around that edge until the two ``pos_x`` faces meet.  The target face
is committed from measured connector frames before the transient hinge is
released.

This is a deterministic constraint-transition approximation of the published
magnetic mechanism.  It deliberately models one actuator plane, not the
hardware's underactuated three-plane carrier or a continuous magnetic field.
"""

from modsim.core.ids import ModuleInstanceId, connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.runtime.momentum_pivot import MomentumPivotConfig, MomentumPivotScenario
from modsim.runtime.reconfiguration import ConnectorPairRef
from modsim.runtime.session import RuntimeSession

SOURCE_URL = "https://doi.org/10.1109/ICRA.2015.7139450"
MODULE_TYPE = "mblocks_3d"
SUPPORT_MODULE = ModuleInstanceId("support_block")
MOVING_MODULE = ModuleInstanceId("moving_block")
NOMINAL_PITCH_M = 0.05


def build_scene() -> SceneSpec:
    """Return two cubes resting at the initial face-connected geometry."""
    half_pitch = NOMINAL_PITCH_M / 2.0
    return SceneSpec.of(
        (
            ModulePlacement(
                instance_id=SUPPORT_MODULE,
                module_type_id=MODULE_TYPE,
                pose=Transform.from_translation((0.0, 0.0, half_pitch)),
            ),
            ModulePlacement(
                instance_id=MOVING_MODULE,
                module_type_id=MODULE_TYPE,
                pose=Transform.from_translation((0.0, 0.0, 3.0 * half_pitch)),
            ),
        )
    )


def build_config() -> MomentumPivotConfig:
    """Return the paper-reference one-plane controller configuration."""
    return MomentumPivotConfig(
        initial_face=_pair("pos_z", "neg_z"),
        edge_hinge=_pair("edge_pos_x_pos_z", "edge_pos_x_neg_z"),
        target_face=_pair("pos_x", "pos_x"),
    )


def build_scenario(session: RuntimeSession) -> MomentumPivotScenario:
    """Create the physical scenario in an already-loaded two-module session."""
    return MomentumPivotScenario.create(session, build_config())


def _pair(support_connector: str, moving_connector: str) -> ConnectorPairRef:
    return ConnectorPairRef(
        fixed_connector=connector_instance_id(SUPPORT_MODULE, support_connector),
        moving_connector=connector_instance_id(MOVING_MODULE, moving_connector),
    )


__all__ = [
    "MODULE_TYPE",
    "MOVING_MODULE",
    "NOMINAL_PITCH_M",
    "SOURCE_URL",
    "SUPPORT_MODULE",
    "build_config",
    "build_scenario",
    "build_scene",
]
