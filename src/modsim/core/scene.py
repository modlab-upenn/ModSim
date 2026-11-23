"""Multi-module scene composition.

A Robot Pack describes module *types*. A scene says which module *instances*
exist in a world and where they start. Modular robotics work is rarely about a
single module, so scene composition is a first-class input to a runtime session
rather than something a backend improvises.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field, replace

from modsim.core.ids import ModuleInstanceId
from modsim.core.transforms import ZERO_VEC3, Transform, Vec3
from modsim.robot_packs.schema import PhysicalConstraintType, RobotPack


class SceneError(ValueError):
    """Raised when a scene cannot be instantiated from a Robot Pack."""


@dataclass(frozen=True, slots=True)
class ModulePlacement:
    """One module instance and its initial world pose."""

    instance_id: ModuleInstanceId
    module_type_id: str
    pose: Transform = field(default_factory=Transform.identity)


@dataclass(frozen=True, slots=True)
class WorldSettings:
    """Physical properties of the world itself, shared by every backend.

    These describe the world, not an engine, so every adapter reads them from
    the scene instead of from engine-specific constructor options. ``None``
    means "use the backend's own default". A backend that cannot honour a
    requested setting refuses the scene (see ``BackendCapabilities``) rather
    than silently ignoring it.
    """

    gravity_m_s2: Vec3 | None = None
    """World gravity vector. ``(0, 0, 0)`` asks for no gravity."""

    ground: bool = False
    """Add a static ground plane at ``ground_height_m``."""

    ground_height_m: float = 0.0

    timestep_s: float | None = None
    """Requested physics integration timestep.

    A hint: backends with a fixed internal rate may substep. ``step(dt_s)`` is
    always honoured to the nearest whole internal step.
    """

    def __post_init__(self) -> None:
        if self.gravity_m_s2 is not None and not all(
            math.isfinite(component) for component in self.gravity_m_s2
        ):
            raise SceneError("gravity_m_s2 must be finite")
        if not math.isfinite(self.ground_height_m):
            raise SceneError("ground_height_m must be finite")
        if self.timestep_s is not None and (
            not math.isfinite(self.timestep_s) or self.timestep_s <= 0.0
        ):
            raise SceneError("timestep_s must be finite and positive")

    @property
    def requests_gravity(self) -> bool:
        """Return whether a non-zero gravity vector was explicitly requested."""
        return self.gravity_m_s2 is not None and any(self.gravity_m_s2)


@dataclass(frozen=True, slots=True)
class ConnectionBudget:
    """Upper bound on simultaneously active connections, by constraint type.

    A sizing hint, not a semantic limit. Engines that must reserve constraint
    storage in advance (MuJoCo's equality pools, for example) size it from
    these values; engines that allocate on demand ignore them. ``None`` lets the
    backend derive a safe bound from the scene. Exceeding a stated bound makes
    the backend refuse the extra connection, exactly as if the engine were full.
    """

    fixed: int | None = None
    compliant: int | None = None
    hinge: int | None = None
    ball: int | None = None
    custom: int | None = None

    def __post_init__(self) -> None:
        for name in ("fixed", "compliant", "hinge", "ball", "custom"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, bool) or value < 0):
                raise SceneError(f"connection budget '{name}' must be a non-negative integer")

    def limit(self, constraint: PhysicalConstraintType) -> int | None:
        """Return the bound for one constraint type, or ``None`` if unstated."""
        value: int | None = getattr(self, PhysicalConstraintType(constraint).value)
        return value


@dataclass(frozen=True, slots=True)
class SceneSpec:
    """An ordered collection of module placements, plus world-level settings."""

    placements: tuple[ModulePlacement, ...] = ()
    world: WorldSettings = field(default_factory=WorldSettings)
    connection_budget: ConnectionBudget = field(default_factory=ConnectionBudget)

    def with_world(self, world: WorldSettings) -> SceneSpec:
        """Return this scene with different world settings."""
        return replace(self, world=world)

    def with_connection_budget(self, budget: ConnectionBudget) -> SceneSpec:
        """Return this scene with a different connection budget."""
        return replace(self, connection_budget=budget)

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
        origin: Vec3 = ZERO_VEC3,
    ) -> SceneSpec:
        """Return ``count`` instances of one module type spaced along one axis.

        This is the smallest useful multi-module scene and exists so that tests
        and examples do not each reinvent placement arithmetic. ``origin``
        shifts the whole row, which is how a scene is lifted clear of a ground
        plane.
        """
        if count < 1:
            raise SceneError("a scene requires at least one module placement")
        if axis not in (0, 1, 2):
            raise SceneError("axis must be 0, 1, or 2")
        name = prefix if prefix is not None else module_type_id
        placements: list[ModulePlacement] = []
        for index in range(count):
            offset = [origin[0], origin[1], origin[2]]
            offset[axis] += spacing_m * index
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
