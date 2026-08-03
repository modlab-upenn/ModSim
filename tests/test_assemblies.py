"""Tests for the derived assembly index."""

from __future__ import annotations

from collections.abc import Iterable
from itertools import pairwise

import pytest

from modsim.core.assemblies import Adjacency, AssemblyIndex
from modsim.core.ids import ModuleInstanceId, assembly_id


def modules(*names: str) -> tuple[ModuleInstanceId, ...]:
    return tuple(ModuleInstanceId(name) for name in names)


def chain(*names: str) -> Adjacency:
    """Return adjacency for a linear chain of modules."""
    ids = modules(*names)
    links: dict[ModuleInstanceId, set[ModuleInstanceId]] = {name: set() for name in ids}
    for left, right in pairwise(ids):
        links[left].add(right)
        links[right].add(left)
    return {name: frozenset(values) for name, values in links.items()}


def partition(index: AssemblyIndex) -> set[frozenset[ModuleInstanceId]]:
    return {index.members(name) for name in index.assemblies}


def total_modules(index: AssemblyIndex) -> int:
    return sum(len(index.members(name)) for name in index.assemblies)


def test_disconnected_modules_are_assemblies_of_size_one() -> None:
    index = AssemblyIndex(modules("a", "b", "c"))
    assert index.count == 3
    assert index.largest_size == 1
    assert index.assembly_of(ModuleInstanceId("b")) == assembly_id(
        frozenset({ModuleInstanceId("b")})
    )


def test_assembly_ids_are_derived_from_membership_not_a_counter() -> None:
    first = AssemblyIndex(modules("a", "b"))
    first.merge_on_connection(ModuleInstanceId("a"), ModuleInstanceId("b"))

    second = AssemblyIndex(modules("b", "a"))
    second.merge_on_connection(ModuleInstanceId("b"), ModuleInstanceId("a"))

    assert first.assemblies == second.assemblies
    assert first.assemblies == ("assembly:a",)


def test_merge_joins_two_components() -> None:
    index = AssemblyIndex(modules("a", "b", "c"))
    result = index.merge_on_connection(ModuleInstanceId("a"), ModuleInstanceId("b"))

    assert result.changed
    assert result.merged_from == ("assembly:a", "assembly:b")
    assert index.count == 2
    assert index.largest_size == 2


def test_merging_within_one_assembly_is_a_no_op() -> None:
    index = AssemblyIndex(modules("a", "b"))
    index.merge_on_connection(ModuleInstanceId("a"), ModuleInstanceId("b"))
    result = index.merge_on_connection(ModuleInstanceId("a"), ModuleInstanceId("b"))

    assert not result.changed
    assert index.count == 1


def test_split_in_the_middle_of_a_chain_produces_two_assemblies() -> None:
    names: Iterable[str] = ("a", "b", "c", "d")
    index = AssemblyIndex(modules(*names))
    index.rebuild(modules(*names), chain(*names))
    assert index.count == 1

    remaining: Adjacency = {**chain("a", "b"), **chain("c", "d")}
    result = index.split_on_disconnection(ModuleInstanceId("b"), ModuleInstanceId("c"), remaining)

    assert result.changed
    assert result.resulting == ("assembly:a", "assembly:c")
    assert partition(index) == {
        frozenset(modules("a", "b")),
        frozenset(modules("c", "d")),
    }


def test_removing_a_loop_edge_does_not_split_the_assembly() -> None:
    names = ("a", "b", "c")
    ring: Adjacency = {
        ModuleInstanceId("a"): frozenset(modules("b", "c")),
        ModuleInstanceId("b"): frozenset(modules("a", "c")),
        ModuleInstanceId("c"): frozenset(modules("a", "b")),
    }
    index = AssemblyIndex(modules(*names))
    index.rebuild(modules(*names), ring)

    without_edge = chain("a", "b", "c")
    result = index.split_on_disconnection(
        ModuleInstanceId("a"), ModuleInstanceId("c"), without_edge
    )

    assert not result.changed
    assert index.count == 1


def test_merge_then_split_restores_the_original_partition() -> None:
    names = ("a", "b", "c", "d")
    index = AssemblyIndex(modules(*names))
    before = partition(index)

    index.merge_on_connection(ModuleInstanceId("b"), ModuleInstanceId("c"))
    assert partition(index) != before

    empty: Adjacency = {name: frozenset() for name in modules(*names)}
    index.split_on_disconnection(ModuleInstanceId("b"), ModuleInstanceId("c"), empty)
    assert partition(index) == before


def test_every_module_belongs_to_exactly_one_assembly_after_any_operation() -> None:
    names = ("a", "b", "c", "d", "e")
    index = AssemblyIndex(modules(*names))
    index.merge_on_connection(ModuleInstanceId("a"), ModuleInstanceId("b"))
    index.merge_on_connection(ModuleInstanceId("b"), ModuleInstanceId("c"))
    assert total_modules(index) == len(names)

    remaining = chain("a", "b")
    index.split_on_disconnection(ModuleInstanceId("b"), ModuleInstanceId("c"), remaining)
    assert total_modules(index) == len(names)
    assert len({index.assembly_of(name) for name in modules(*names)}) == index.count


def test_splitting_modules_that_share_no_assembly_is_an_error() -> None:
    index = AssemblyIndex(modules("a", "b"))
    with pytest.raises(ValueError, match="not in one assembly"):
        index.split_on_disconnection(ModuleInstanceId("a"), ModuleInstanceId("b"), {})


def test_unknown_module_lookup_is_an_error() -> None:
    index = AssemblyIndex(modules("a"))
    with pytest.raises(KeyError, match="not indexed"):
        index.assembly_of(ModuleInstanceId("missing"))
