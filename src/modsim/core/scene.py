"""Multi-module scene composition.

A Robot Pack describes module *types*. A scene says which module *instances*
exist in a world and where they start. Modular robotics work is rarely about a
single module, so scene composition is a first-class input to a runtime session
rather than something a backend improvises.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field

from modsim.core.ids import ModuleInstanceId
from modsim.core.transforms import Transform
from modsim.robot_packs.schema import RobotPack


class SceneError(ValueError):
    """Raised when a scene cannot be instantiated from a Robot Pack."""


@dataclass(frozen=True, slots=True)
class ModulePlacement:
    """One module instance and its initial world pose."""

    instance_id: ModuleInstanceId
    module_type_id: str
    pose: Transform = field(default_factory=Transform.identity)


@dataclass(frozen=True, slots=True)
class SceneSpec:
    """An ordered collection of module placements."""

    placements: tuple[ModulePlacement, ...] = ()

    @classmethod
    def of(cls, placements: Iterable[ModulePlacement]) -> SceneSpec:
        """Return a scene from any iterable of placements."""
        return cls(placements=tuple(placements))

    @classmethod
    def grid(
        cls,
        module_type_id: str,
        count: int,
        *,
        spacing_m: float,
        prefix: str | None = None,
        axis: int = 0,
    ) -> SceneSpec:
        """Return ``count`` instances of one module type spaced along one axis.

        This is the smallest useful multi-module scene and exists so that tests
        and examples do not each reinvent placement arithmetic.
        """
        if count < 1:
            raise SceneError("a scene requires at least one module placement")
        if axis not in (0, 1, 2):
            raise SceneError("axis must be 0, 1, or 2")
        name = prefix if prefix is not None else module_type_id
        placements: list[ModulePlacement] = []
        for index in range(count):
            offset = [0.0, 0.0, 0.0]
            offset[axis] = spacing_m * index
            placements.append(
                ModulePlacement(
                    instance_id=ModuleInstanceId(f"{name}_{index}"),
                    module_type_id=module_type_id,
                    pose=Transform.from_translation((offset[0], offset[1], offset[2])),
                )
            )
        return cls(placements=tuple(placements))

    @property
    def instance_ids(self) -> tuple[ModuleInstanceId, ...]:
        """Return every module instance identifier in placement order."""
        return tuple(placement.instance_id for placement in self.placements)

    def validate_against(self, pack: RobotPack) -> None:
        """Raise when the scene cannot be instantiated from ``pack``."""
        if not self.placements:
            raise SceneError("a scene requires at least one module placement")
        seen: set[ModuleInstanceId] = set()
        duplicates: list[ModuleInstanceId] = []
        unknown: list[str] = []
        for placement in self.placements:
            if placement.instance_id in seen:
                duplicates.append(placement.instance_id)
            seen.add(placement.instance_id)
            if placement.module_type_id not in pack.hardware_catalog.module_types:
                unknown.append(placement.module_type_id)
        if duplicates:
            raise SceneError(
                "module instance ids must be unique: " + ", ".join(sorted(set(duplicates)))
            )
        if unknown:
            available = ", ".join(sorted(pack.hardware_catalog.module_types))
            raise SceneError(
                f"unknown module type(s) {', '.join(sorted(set(unknown)))}; "
                f"pack '{pack.id}' defines: {available}"
            )

    def module_links(self, pack: RobotPack) -> dict[ModuleInstanceId, tuple[str, ...]]:
        """Return the link names each placement needs, root link first.

        A Robot Pack does not itself enumerate links; it names a root link and
        references parent and child links through joints and connectors. This
        collects that set so a backend can create one body per link without
        parsing the mechanical asset.
        """
        self.validate_against(pack)
        result: dict[ModuleInstanceId, tuple[str, ...]] = {}
        for placement in self.placements:
            module_type = pack.hardware_catalog.module_types[placement.module_type_id]
            names: list[str] = [module_type.root_link]
            candidates: Sequence[str] = [
                *(joint.parent_link for joint in module_type.joints),
                *(joint.child_link for joint in module_type.joints),
                *(connector.parent_link for connector in module_type.connectors),
            ]
            for name in candidates:
                if name not in names:
                    names.append(name)
            result[placement.instance_id] = tuple(names)
        return result
