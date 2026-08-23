"""Derived index of physically connected components.

An assembly is a *derived view*, never canonical state: it is recomputed from
the set of module instances and the set of active connections. A disconnected
module is a valid assembly of size one, so every module always belongs to
exactly one assembly.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable, Mapping
from dataclasses import dataclass

from modsim.core.ids import AssemblyId, ModuleInstanceId, assembly_id

Adjacency = Mapping[ModuleInstanceId, frozenset[ModuleInstanceId]]


@dataclass(frozen=True, slots=True)
class MergeResult:
    """Outcome of merging two assemblies."""

    assembly_id: AssemblyId
    merged_from: tuple[AssemblyId, ...]

    @property
    def changed(self) -> bool:
        """Whether the merge actually joined two distinct assemblies."""
        return len(self.merged_from) > 1


@dataclass(frozen=True, slots=True)
class SplitResult:
    """Outcome of removing a connection from an assembly."""

    source_assembly_id: AssemblyId
    resulting: tuple[AssemblyId, ...]

    @property
    def changed(self) -> bool:
        """Whether the removal actually separated the assembly."""
        return len(self.resulting) > 1


class AssemblyIndex:
    """Connected-component index over module instances.

    Assembly identifiers are derived from membership rather than from a
    counter, so the same physical configuration always yields the same
    identifiers. That makes assembly identity comparable across runs and across
    processes, which matters for reconfiguration experiments that replay logs.
    """

    __slots__ = ("_members", "_owner")

    def __init__(self, module_ids: Iterable[ModuleInstanceId] = ()) -> None:
        self._owner: dict[ModuleInstanceId, AssemblyId] = {}
        self._members: dict[AssemblyId, frozenset[ModuleInstanceId]] = {}
        self.rebuild(module_ids, {})

    def rebuild(self, module_ids: Iterable[ModuleInstanceId], adjacency: Adjacency) -> None:
        """Recompute every component from scratch."""
        self._owner = {}
        self._members = {}
        remaining = set(module_ids)
        while remaining:
            seed = min(remaining)
            component = self._component(seed, adjacency)
            remaining -= component
            self._register(component)

    def assembly_of(self, module_id: ModuleInstanceId) -> AssemblyId:
        """Return the assembly containing ``module_id``."""
        try:
            return self._owner[module_id]
        except KeyError as error:
            raise KeyError(f"module '{module_id}' is not indexed") from error

    def members(self, assembly: AssemblyId) -> frozenset[ModuleInstanceId]:
        """Return every module in ``assembly``."""
        try:
            return self._members[assembly]
        except KeyError as error:
            raise KeyError(f"unknown assembly '{assembly}'") from error

    @property
    def assemblies(self) -> tuple[AssemblyId, ...]:
        """Return every assembly identifier in deterministic order."""
        return tuple(sorted(self._members))

    @property
    def count(self) -> int:
        """Return the number of physically connected components."""
        return len(self._members)

    @property
    def largest_size(self) -> int:
        """Return the module count of the largest assembly, or zero when empty."""
        return max((len(members) for members in self._members.values()), default=0)

    def merge_on_connection(
        self,
        module_a: ModuleInstanceId,
        module_b: ModuleInstanceId,
    ) -> MergeResult:
        """Join the assemblies owning ``module_a`` and ``module_b``."""
        left = self.assembly_of(module_a)
        right = self.assembly_of(module_b)
        if left == right:
            return MergeResult(assembly_id=left, merged_from=(left,))
        merged = self._members[left] | self._members[right]
        self._unregister(left)
        self._unregister(right)
        return MergeResult(
            assembly_id=self._register(merged),
            merged_from=tuple(sorted((left, right))),
        )

    def split_on_disconnection(
        self,
        module_a: ModuleInstanceId,
        module_b: ModuleInstanceId,
        adjacency: Adjacency,
    ) -> SplitResult:
        """Recompute the component that held ``module_a`` and ``module_b``.

        ``adjacency`` must already exclude the removed connection. Only the one
        affected component is walked, so the cost is proportional to that
        assembly rather than to the whole world.
        """
        source = self.assembly_of(module_a)
        if self.assembly_of(module_b) != source:
            raise ValueError(f"modules '{module_a}' and '{module_b}' are not in one assembly")
        self._unregister(source)
        first = self._component(module_a, adjacency)
        resulting = [self._register(first)]
        if module_b not in first:
            resulting.append(self._register(self._component(module_b, adjacency)))
        return SplitResult(source_assembly_id=source, resulting=tuple(sorted(resulting)))

    @staticmethod
    def _component(
        seed: ModuleInstanceId,
        adjacency: Adjacency,
    ) -> frozenset[ModuleInstanceId]:
        seen = {seed}
        queue: deque[ModuleInstanceId] = deque((seed,))
        while queue:
            current = queue.popleft()
            for neighbour in adjacency.get(current, frozenset()):
                if neighbour not in seen:
                    seen.add(neighbour)
                    queue.append(neighbour)
        return frozenset(seen)

    def _register(self, members: frozenset[ModuleInstanceId]) -> AssemblyId:
        identifier = assembly_id(members)
        self._members[identifier] = members
        for module_id in members:
            self._owner[module_id] = identifier
        return identifier

    def _unregister(self, identifier: AssemblyId) -> None:
        for module_id in self._members.pop(identifier):
            self._owner.pop(module_id, None)
