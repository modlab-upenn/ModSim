"""Contracts for backend-neutral model-view generation."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Hashable
from dataclasses import dataclass
from typing import Generic, TypeVar

from pydantic import Field, field_validator

from modsim.core.state import WorldState
from modsim.model_views.models import ModelView, ModelViewDTO, ModelViewSourceStamp
from modsim.robot_packs.schema import ModelViewMode, ModelViewSpec, RobotPack


class ModelViewError(RuntimeError):
    """Base exception for model-view registration and generation failures."""


class DuplicateModelViewBuilderError(ModelViewError):
    """Raised when two builders claim the same stable identifier."""


class UnknownModelViewBuilderError(ModelViewError):
    """Raised when a recipe references an unregistered builder."""


class ModelViewUnavailableError(ModelViewError):
    """Raised when a builder's required inputs are absent."""


class ModelViewBuildError(ModelViewError):
    """Raised when a builder cannot generate a valid view from its recipe."""


@dataclass(frozen=True, slots=True)
class ModelViewContext:
    """Inputs available to model-view builders for one generation request."""

    pack: RobotPack
    world: WorldState | None = None

    def __post_init__(self) -> None:
        if self.world is not None and self.world.pack != self.pack:
            raise ValueError("model-view context world was created from a different Robot Pack")


class ModelViewDescriptor(ModelViewDTO):
    """Static builder capabilities exposed to clients and authoring tools."""

    builder: str = Field(min_length=1)
    name: str = Field(min_length=1)
    view_type: str = Field(min_length=1)
    modes: tuple[ModelViewMode, ...]
    requires_world: bool

    @field_validator("modes")
    @classmethod
    def require_unique_nonempty_modes(
        cls,
        value: tuple[ModelViewMode, ...],
    ) -> tuple[ModelViewMode, ...]:
        if not value:
            raise ValueError("model-view builder modes must not be empty")
        if len(set(value)) != len(value):
            raise ValueError("model-view builder modes must not contain duplicates")
        return value


ViewT_co = TypeVar("ViewT_co", bound=ModelView, covariant=True)


class ModelViewBuilder(ABC, Generic[ViewT_co]):
    """Abstract generator for one stable model-view representation."""

    @property
    @abstractmethod
    def builder_id(self) -> str:
        """Return the stable ID referenced by Robot Pack recipes."""

    @property
    @abstractmethod
    def display_name(self) -> str:
        """Return a human-readable builder name."""

    @property
    @abstractmethod
    def view_type(self) -> str:
        """Return the stable semantic result type produced by this builder."""

    @property
    @abstractmethod
    def modes(self) -> tuple[ModelViewMode, ...]:
        """Return the authoring/runtime modes this builder supports."""

    @property
    def requires_world(self) -> bool:
        """Return whether this builder needs live canonical world state."""
        return False

    def descriptor(self) -> ModelViewDescriptor:
        """Return the immutable public description of this builder."""
        return ModelViewDescriptor(
            builder=self.builder_id,
            name=self.display_name,
            view_type=self.view_type,
            modes=self.modes,
            requires_world=self.requires_world,
        )

    def is_available(self, context: ModelViewContext) -> bool:
        """Return whether every required input is present in ``context``."""
        return not self.requires_world or context.world is not None

    def cache_token(self, context: ModelViewContext) -> Hashable:
        """Return the source revision subset that changes this builder's output."""
        return None

    @abstractmethod
    def build(
        self,
        recipe: ModelViewSpec,
        context: ModelViewContext,
        source: ModelViewSourceStamp,
    ) -> ViewT_co:
        """Generate one immutable view from ``recipe`` and ``context``."""
