"""The maintained module adjacency must equal a full rebuild after every event."""

from __future__ import annotations

import random
from collections import defaultdict
from pathlib import Path

from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.state import WorldState
from modsim.core.transforms import Transform
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.session import RuntimeSession

PACK = Path(__file__).parents[1] / "examples/robot_packs/generic_cube"


def _rebuilt(world: WorldState) -> dict[ModuleInstanceId, frozenset[ModuleInstanceId]]:
    neighbours: dict[ModuleInstanceId, set[ModuleInstanceId]] = defaultdict(set)
    for module_id in world.modules:
        neighbours[module_id]
    for connection in world.connections.values():
        if connection.module_a != connection.module_b:
            neighbours[connection.module_a].add(connection.module_b)
            neighbours[connection.module_b].add(connection.module_a)
    return {module_id: frozenset(values) for module_id, values in neighbours.items()}


def _components(
    adjacency: dict[ModuleInstanceId, frozenset[ModuleInstanceId]],
) -> set[frozenset[ModuleInstanceId]]:
    seen: set[ModuleInstanceId] = set()
    result: set[frozenset[ModuleInstanceId]] = set()
    for start in adjacency:
        if start in seen:
            continue
        component, stack = {start}, [start]
        while stack:
            for neighbour in adjacency[stack.pop()]:
                if neighbour not in component:
                    component.add(neighbour)
                    stack.append(neighbour)
        seen |= component
        result.add(frozenset(component))
    return result


def _check(world: WorldState) -> None:
    expected = _rebuilt(world)
    assert world.adjacency() == expected
    for connection_id in world.connections:
        excluded = _rebuilt(world)
        connection = world.connections[connection_id]
        if connection.module_a != connection.module_b and not any(
            other.id != connection_id
            and {other.module_a, other.module_b} == {connection.module_a, connection.module_b}
            for other in world.connections.values()
        ):
            excluded[connection.module_a] -= {connection.module_b}
            excluded[connection.module_b] -= {connection.module_a}
        assert world.adjacency(exclude=connection_id) == excluded
    assemblies = world.assemblies
    actual = {frozenset(assemblies.members(assemblies.assembly_of(m))) for m in world.modules}
    assert actual == _components(expected)


def test_adjacency_and_assemblies_track_random_docks_and_undocks() -> None:
    rows, length = 3, 6
    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=ModuleInstanceId(f"c{r}_{i}"),
            module_type_id="generic_cube",
            pose=Transform.from_translation((0.1 * i, 0.5 * r, 1.0)),
        )
        for r in range(rows)
        for i in range(length)
    )
    session = RuntimeSession.create(RobotPackLoader().load(PACK), scene, "mock")
    try:
        world, manager = session.world, session.docking
        pairs = [
            (p.connector_a, p.connector_b)
            for p in session.proposals()
            if p.compatibility.compatible and p.acceptance.satisfied
        ]
        assert len(pairs) == rows * (length - 1)
        for pair in pairs:
            session.request_dock(*pair)
        session.process_docking()
        _check(world)
        rng = random.Random(7)
        for _ in range(60):
            docked = [pair for pair in pairs if world.connector(pair[0]).connection_id is not None]
            free = [pair for pair in pairs if world.connector(pair[0]).connection_id is None]
            if docked and (not free or rng.random() < 0.5):
                pair = rng.choice(docked)
                connection = world.connector(pair[0]).connection_id
                assert connection is not None
                manager.undock(world, session.adapter, connection)
            else:
                pair = rng.choice(free)
                for connector in pair:
                    world.connector(connector).available_at_s = 0.0
                proposal = manager._evaluate(world, *sorted(pair), requested=True)  # pyright: ignore[reportPrivateUsage]
                assert proposal is not None and proposal.viable
                manager.commit(world, session.adapter, proposal)
            _check(world)
    finally:
        session.shutdown()
