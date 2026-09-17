"""Run-to-run repeatability of docking decisions and reconfiguration.

The paper claims determinism: the event-sourced core derives assembly identity
from membership (``assembly:`` + min member id) and replays exactly, and MuJoCo
integrates deterministically for a fixed model and inputs. These tests turn that
claim into a measurement by running the *same* scenario from scratch several
times and requiring bit-stable results.

Two layers are covered:

* **Semantic determinism** on the kinematic mock -- the committed event-kind
  sequence and the resulting assembly partition and identifiers are identical
  across repetitions. No physics engine is involved, so this isolates the core.
* **Physical determinism** on MuJoCo -- a real coast-and-dock, and the full
  seven-module Driver-to-Snake reconfiguration, each produce an identical event
  sequence and identical final connector world poses across repetitions.
"""

from __future__ import annotations

import math
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import Any

import pytest

from conftest import with_connector_policy
from modsim.backends.base import SupportsModuleKinematics
from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, vec_norm, vec_sub
from modsim.robot_packs import LoadedRobotPack, RobotPack, RobotPackLoader
from modsim.runtime.reconfiguration import (
    ReconfigurationPhase,
    ReconfigurationPlan,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
)
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "generic_cube"
CONNECTOR_TYPE = "fixed_face"

SEMANTIC_REPEATS = 5
PHYSICAL_REPEATS = 3

CHAIN_COUNT = 6
CHAIN_SPACING_M = 0.1
STEP_S = 0.002

CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")
APPROACH_SPEED_M_S = 0.03
START_GAP_M = 0.15
REDOCK_COOLDOWN_S = 2.0
APPROACH_TIMEOUT_S = 20.0

# MuJoCo is deterministic for a fixed compiled model and single-threaded stepping,
# so repeated runs are expected to agree to machine precision. The bound is kept a
# little above that so a platform with slightly different FP rounding still passes;
# tighten it if the target build proves bit-exact.
POSE_REPEATABILITY_TOLERANCE_M = 1e-9


# ----------------------------------------------------------------------
# semantic determinism (mock backend, no physics)
# ----------------------------------------------------------------------


def _assembly_partition(session: RuntimeSession) -> set[frozenset[ModuleInstanceId]]:
    index = session.world.assemblies
    return {index.members(name) for name in index.assemblies}


def _assembly_ids(session: RuntimeSession) -> tuple[str, ...]:
    return tuple(sorted(str(name) for name in session.world.assemblies.assemblies))


def _semantic_run(example_pack_dir: Path) -> tuple[tuple[str, ...], tuple[str, ...],
                                                   set[frozenset[ModuleInstanceId]]]:
    """Run one scripted chain dock/undock; return event kinds, docked ids, partition."""
    scene = SceneSpec.grid(MODULE_TYPE, CHAIN_COUNT, spacing_m=CHAIN_SPACING_M)
    session = RuntimeSession.create(RobotPackLoader().load(example_pack_dir), scene, "mock")

    for proposal in session.proposals():
        session.request_dock(proposal.connector_a, proposal.connector_b)
    kinds = [event.kind for event in session.step(STEP_S)]
    # A chain of N cubes latches into one assembly before we release it.
    assert session.world.assemblies.count == 1
    docked_ids = _assembly_ids(session)

    for connection in sorted(session.world.connections):
        session.request_undock(connection)
    kinds.extend(event.kind for event in session.step(STEP_S))
    return tuple(kinds), docked_ids, _assembly_partition(session)


def test_semantic_reconfiguration_is_bit_stable(example_pack_dir: Path) -> None:
    """Repeating the scripted chain dock/undock yields identical semantics."""
    reference = _semantic_run(example_pack_dir)
    for _ in range(SEMANTIC_REPEATS - 1):
        kinds, docked_ids, partition = _semantic_run(example_pack_dir)
        assert kinds == reference[0], "event-kind sequence changed between runs"
        assert docked_ids == reference[1], "assembly identifiers changed between runs"
        assert partition == reference[2], "final partition changed between runs"


# ----------------------------------------------------------------------
# physical determinism (MuJoCo)
# ----------------------------------------------------------------------


def _latching_pack(example_pack_dir: Path) -> LoadedRobotPack:
    loaded = RobotPackLoader().load(example_pack_dir)
    return loaded.with_pack(
        with_connector_policy(
            loaded.pack,
            CONNECTOR_TYPE,
            auto_latch=True,
            redock_cooldown_s=REDOCK_COOLDOWN_S,
        )
    )


def _coast_and_dock(loaded: LoadedRobotPack) -> tuple[tuple[str, ...], dict[str, Transform]]:
    """Run one two-module coast-and-dock; return event kinds and final poses."""
    placements = (
        ModulePlacement(instance_id=CUBE_0, module_type_id=MODULE_TYPE),
        ModulePlacement(
            instance_id=CUBE_1,
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((START_GAP_M, 0.0, 0.0)),
        ),
    )
    session = RuntimeSession.create(
        loaded, SceneSpec(placements=placements), "mujoco", gravity=(0.0, 0.0, 0.0)
    )
    adapter = session.adapter
    assert isinstance(adapter, SupportsModuleKinematics)
    adapter.set_module_twist(CUBE_1, linear_m_s=(-APPROACH_SPEED_M_S, 0.0, 0.0))
    kinds: list[str] = []
    try:
        deadline = session.world.time_s + APPROACH_TIMEOUT_S
        while session.world.time_s < deadline and not session.world.connections:
            kinds.extend(event.kind for event in session.step(STEP_S))
        # A few settling steps so the recorded pose is past the latch transient.
        for _ in range(200):
            kinds.extend(event.kind for event in session.step(STEP_S))
        poses = {
            str(module_id): module.pose for module_id, module in session.world.modules.items()
        }
    finally:
        session.shutdown()
    return tuple(kinds), poses


def _assert_pose_maps_agree(
    reference: dict[str, Transform], other: dict[str, Transform]
) -> float:
    assert set(reference) == set(other), "module set changed between runs"
    max_offset = 0.0
    for name, pose in other.items():
        offset = vec_norm(vec_sub(pose.translation, reference[name].translation))
        max_offset = max(max_offset, offset)
    assert max_offset <= POSE_REPEATABILITY_TOLERANCE_M, (
        f"final poses differ by {max_offset:.3e} m between runs"
    )
    return max_offset


@pytest.mark.mujoco
def test_physical_dock_is_repeatable(example_pack_dir: Path) -> None:
    """A real coast-and-dock repeats with an identical event sequence and pose."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    loaded = _latching_pack(example_pack_dir)

    reference_kinds: tuple[str, ...] | None = None
    reference_poses: dict[str, Transform] | None = None
    for _ in range(PHYSICAL_REPEATS):
        kinds, poses = _coast_and_dock(loaded)
        if reference_kinds is None:
            reference_kinds, reference_poses = kinds, poses
            continue
        assert kinds == reference_kinds, "event-kind sequence changed between physical runs"
        assert reference_poses is not None
        _assert_pose_maps_agree(reference_poses, poses)


# ----------------------------------------------------------------------
# physical determinism of the full Driver-to-Snake reconfiguration
# ----------------------------------------------------------------------


def _driver_to_snake_plan() -> ReconfigurationPlan:
    path = Path(__file__).resolve().parents[1] / "examples/scenarios/smores_driver_to_snake.py"
    spec = spec_from_file_location("repeatability_smores_driver_to_snake", path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    plan = module.build_plan()
    assert isinstance(plan, ReconfigurationPlan)
    return plan


def _connector_spec(
    identifier: str,
    *,
    position: tuple[float, float, float],
    axis: tuple[float, float, float],
    rotation: tuple[float, float, float],
) -> dict[str, object]:
    return {
        "id": identifier,
        "connector_type": CONNECTOR_TYPE,
        "parent_link": "base_link",
        "frame": None,
        "local_pose": {"xyz_m": list(position), "rpy_rad": list(rotation)},
        "docking_axis": list(axis),
        "approach_axis": list(axis),
    }


def _four_face_pack(example_pack_dir: Path) -> LoadedRobotPack:
    """The generic cube pack with SMORES-like face names added in memory."""
    loaded = RobotPackLoader().load(example_pack_dir)
    data: dict[str, Any] = loaded.pack.model_dump(mode="python")
    module = data["hardware_catalog"]["module_types"][MODULE_TYPE]
    module["connectors"] = [
        _connector_spec("pan", position=(0.05, 0.0, 0.0), axis=(1.0, 0.0, 0.0),
                        rotation=(0.0, 0.0, 0.0)),
        _connector_spec("bottom", position=(-0.05, 0.0, 0.0), axis=(-1.0, 0.0, 0.0),
                        rotation=(0.0, 0.0, math.pi)),
        _connector_spec("right", position=(0.0, 0.05, 0.0), axis=(0.0, 1.0, 0.0),
                        rotation=(0.0, 0.0, math.pi / 2.0)),
        _connector_spec("left", position=(0.0, -0.05, 0.0), axis=(0.0, -1.0, 0.0),
                        rotation=(0.0, 0.0, -math.pi / 2.0)),
    ]
    return loaded.with_pack(RobotPack.model_validate(data))


def _seven_module_scene(plan: ReconfigurationPlan) -> SceneSpec:
    return SceneSpec.of(
        ModulePlacement(
            instance_id=module_id,
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((0.3 * index, 0.0, 0.0)),
        )
        for index, module_id in enumerate(plan.module_ids)
    )


def _run_driver_to_snake(loaded: LoadedRobotPack) -> tuple[tuple[str, ...], dict[str, Transform]]:
    plan = _driver_to_snake_plan()
    session = RuntimeSession.create(
        loaded, _seven_module_scene(plan), "mujoco", gravity=(0.0, 0.0, 0.0), timestep_s=STEP_S
    )
    kinds: list[str] = []
    try:
        scenario = ScriptedReconfigurationScenario.create(
            session,
            plan,
            ScriptedReconfigurationConfig(
                dt_s=STEP_S,
                gap_m=0.01,
                approach_speed_m_s=APPROACH_SPEED_M_S,
                initial_hold_s=0.002,
                separated_hold_s=0.002,
                connected_hold_s=0.002,
            ),
        )
        for _ in range(4000):
            kinds.extend(event.kind for event in scenario.step())
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break
        assert scenario.status.phase is ReconfigurationPhase.COMPLETE, (
            f"reconfiguration did not complete: {scenario.status}"
        )
        poses = {
            str(module_id): module.pose for module_id, module in session.world.modules.items()
        }
    finally:
        session.shutdown()
    return tuple(kinds), poses


@pytest.mark.mujoco
def test_driver_to_snake_reconfiguration_is_repeatable(example_pack_dir: Path) -> None:
    """The full seven-module reconfiguration repeats bit-stably across runs."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    loaded = _four_face_pack(example_pack_dir)

    reference_kinds: tuple[str, ...] | None = None
    reference_poses: dict[str, Transform] | None = None
    for _ in range(PHYSICAL_REPEATS):
        kinds, poses = _run_driver_to_snake(loaded)
        if reference_kinds is None:
            reference_kinds, reference_poses = kinds, poses
            continue
        assert kinds == reference_kinds, "reconfiguration event sequence changed between runs"
        assert reference_poses is not None
        _assert_pose_maps_agree(reference_poses, poses)


if __name__ == "__main__":  # pragma: no cover - manual entry point
    import sys

    sys.exit(pytest.main([__file__, "-s", "-p", "no:cov"]))
