"""Differential conformance on SMORES-EP, across the mock and MuJoCo backends.

The differential suite in ``test_backend_conformance`` runs two backends on one
Robot Pack and requires them to agree on the *semantic* picture: the same
connector frames, the same docking verdicts, the same committed event sequence,
and the same assemblies. It ran on the single-link generic cube because the mock
could only reference root-link connectors.

The mock now resolves link frames from the URDF joint tree, so it references
articulated hardware too. This suite is the same differential conformance on the
committed SMORES-EP pack, whose ``pan``, ``left``, and ``right`` faces sit on
the wheels and tilt body. Poses under dynamics may diverge once real forces act,
so every scene here is weightless, and the one docking check uses exact staging
so the backends are compared on the decisions ModSim makes, not on their
physics. A disagreement means an adapter is wrong.
"""

from __future__ import annotations

from collections.abc import Iterator

import pytest

from modsim.backends.registry import installed_backends
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.transforms import angle_between, vec_norm, vec_sub
from modsim.robot_packs import LoadedRobotPack
from modsim.runtime.reconfiguration import stage_docking_assembly_pair
from modsim.runtime.session import RuntimeSession

SMORES_TYPE = "smores_ep"
MODULE_COUNT = 2
CONNECTORS_PER_MODULE = 4
SPACING_M = 0.3
STEP_S = 0.002

POSITION_TOLERANCE_M = 1e-6
ANGLE_TOLERANCE_RAD = 1e-6
DRIFT_TOLERANCE_M = 1e-6

BOTTOM_0 = ConnectorInstanceId("smores_ep_0/bottom")
BOTTOM_1 = ConnectorInstanceId("smores_ep_1/bottom")

COMPARABLE_BACKENDS = ("mock", "mujoco")

# Event kinds, each connection's mating orientation index, and the assembly
# partition -- the backend-independent outcome of one staged dock.
_DockSummary = tuple[tuple[str, ...], dict[str, int | None], set[frozenset[ModuleInstanceId]]]


def _requires_both_backends() -> None:
    if "mujoco" not in installed_backends():
        pytest.skip("needs the MuJoCo backend to compare against the mock reference")


def _make_session(pack: LoadedRobotPack, backend: str) -> RuntimeSession:
    scene = SceneSpec.grid(SMORES_TYPE, MODULE_COUNT, spacing_m=SPACING_M)
    if backend == "mock":
        return RuntimeSession.create(pack, scene, backend)
    return RuntimeSession.create(pack, scene, backend, gravity=(0.0, 0.0, 0.0))


def _sessions(pack: LoadedRobotPack) -> Iterator[tuple[str, RuntimeSession]]:
    """Yield one live session per comparable, installed backend."""
    for backend in COMPARABLE_BACKENDS:
        yield backend, _make_session(pack, backend)


def _assembly_partition(session: RuntimeSession) -> set[frozenset[ModuleInstanceId]]:
    index = session.world.assemblies
    return {index.members(name) for name in index.assemblies}


# ----------------------------------------------------------------------
# per-backend invariants
# ----------------------------------------------------------------------


def test_backends_instantiate_the_same_smores_scene(smores_loaded_pack: LoadedRobotPack) -> None:
    """Both backends build the same modules, connectors, and free assemblies."""
    _requires_both_backends()
    for backend, session in _sessions(smores_loaded_pack):
        try:
            assert set(session.world.modules) == {
                ModuleInstanceId(f"{SMORES_TYPE}_{index}") for index in range(MODULE_COUNT)
            }, f"{backend} built a different module set"
            assert session.world.assemblies.count == MODULE_COUNT
            assert len(session.world.connectors) == MODULE_COUNT * CONNECTORS_PER_MODULE
            assert all(
                connector.resolved for connector in session.world.connectors.values()
            ), f"{backend} left a SMORES connector frame unresolved"
        finally:
            session.shutdown()


# ----------------------------------------------------------------------
# cross-backend agreement
# ----------------------------------------------------------------------


def test_backends_agree_on_smores_connector_frames(smores_loaded_pack: LoadedRobotPack) -> None:
    """MuJoCo and the mock reference resolve every connector to one world frame.

    This is the frame-agreement axis extended to connectors on articulated
    links, which the mock could not reference before it resolved link frames.
    """
    _requires_both_backends()
    reference_poses: dict[str, tuple[float, float, float]] | None = None
    reference_axes: dict[str, tuple[float, float, float]] = {}

    for backend, session in _sessions(smores_loaded_pack):
        try:
            translations = {
                str(connector.id): connector.world_pose.translation
                for connector in session.world.connectors.values()
            }
            axes = {
                str(connector.id): connector.world_docking_axis
                for connector in session.world.connectors.values()
            }
        finally:
            session.shutdown()
        if reference_poses is None:
            reference_poses, reference_axes = translations, axes
            continue

        assert set(translations) == set(reference_poses), (
            f"{backend} reports a different connector set"
        )
        for name, translation in translations.items():
            offset = vec_norm(vec_sub(translation, reference_poses[name]))
            assert offset == pytest.approx(0.0, abs=POSITION_TOLERANCE_M), (
                f"{backend} places connector '{name}' {offset:.3e} m from the reference"
            )
            angle = angle_between(axes[name], reference_axes[name])
            assert angle == pytest.approx(0.0, abs=ANGLE_TOLERANCE_RAD), (
                f"{backend} points connector '{name}' {angle:.3e} rad from the reference"
            )


def test_backends_agree_on_smores_initial_assemblies(smores_loaded_pack: LoadedRobotPack) -> None:
    """Both backends start SMORES-EP as the same free-module partition."""
    _requires_both_backends()
    reference: set[frozenset[ModuleInstanceId]] | None = None
    for _, session in _sessions(smores_loaded_pack):
        try:
            partition = _assembly_partition(session)
        finally:
            session.shutdown()
        if reference is None:
            reference = partition
            continue
        assert partition == reference


def test_smores_scene_does_not_drift_on_any_backend(smores_loaded_pack: LoadedRobotPack) -> None:
    """A weightless SMORES scene holds still on both backends."""
    _requires_both_backends()
    for backend, session in _sessions(smores_loaded_pack):
        try:
            before = {
                module_id: module.pose.translation
                for module_id, module in session.world.modules.items()
            }
            for _ in range(20):
                session.step(0.005)
            for module_id, module in session.world.modules.items():
                drift = vec_norm(vec_sub(module.pose.translation, before[module_id]))
                assert drift == pytest.approx(0.0, abs=DRIFT_TOLERANCE_M), (
                    f"{backend} drifted module '{module_id}' by {drift:.3e} m with no forces"
                )
        finally:
            session.shutdown()


# ----------------------------------------------------------------------
# docking-decision conformance
# ----------------------------------------------------------------------


def _staged_bottom_dock_summary(session: RuntimeSession) -> _DockSummary:
    """Exact-stage a bottom-bottom dock and return the semantic outcome.

    Staging aligns the connectors without physics, so the backends are compared
    on the docking decision -- event sequence, connection identity and mating
    orientation, and the resulting assembly -- rather than on approach dynamics.
    """
    stage_docking_assembly_pair(session, BOTTOM_0, BOTTOM_1, gap_m=0.0)
    session.request_dock(BOTTOM_0, BOTTOM_1)
    events = session.process_docking()
    kinds = tuple(event.kind for event in events)
    identity = {
        str(connection.id): connection.orientation_index
        for connection in session.world.connections.values()
    }
    return kinds, identity, _assembly_partition(session)


def test_backends_agree_on_a_staged_smores_dock(smores_loaded_pack: LoadedRobotPack) -> None:
    """A staged rear-base dock reaches the same decision on both backends."""
    _requires_both_backends()
    reference: _DockSummary | None = None
    reference_backend = ""

    for backend, session in _sessions(smores_loaded_pack):
        try:
            summary = _staged_bottom_dock_summary(session)
        finally:
            session.shutdown()
        assert summary[0][:2] == ("DockCandidateDetected", "DockCommitted"), (
            f"{backend} did not commit the staged dock: {summary[0]}"
        )
        if reference is None:
            reference, reference_backend = summary, backend
            continue
        assert summary == reference, (
            f"{backend} reached a different docking decision than {reference_backend}"
        )
