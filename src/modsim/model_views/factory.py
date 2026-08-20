"""Registration, selection, and bounded caching for model-view builders."""

from __future__ import annotations

import json
from collections import OrderedDict
from dataclasses import dataclass
from threading import RLock

from pydantic import TypeAdapter, ValidationError

from modsim.model_views.base import (
    DuplicateModelViewBuilderError,
    ModelViewBuilder,
    ModelViewBuildError,
    ModelViewContext,
    ModelViewDescriptor,
    ModelViewUnavailableError,
    UnknownModelViewBuilderError,
)
from modsim.model_views.models import ModelView, ModelViewSourceStamp
from modsim.model_views.topology import ModuleTopologyGraphBuilder
from modsim.robot_packs.schema import Identifier, ModelViewMode, ModelViewSpec


@dataclass(frozen=True, slots=True)
class _CacheEntry:
    pack: object
    world: object | None
    recipe_fingerprint: str
    source_token: object
    result: ModelView


_CacheSlot = tuple[int, int | None, str]
_IDENTIFIER_ADAPTER: TypeAdapter[str] = TypeAdapter(Identifier)


class ModelViewFactory:
    """Own independent builders and latest generated results for one client.

    A factory is intentionally an ordinary object rather than a singleton.
    Separate runtimes, tests, and frontends can therefore register different
    platform-specific builders without affecting each other.
    """

    def __init__(self, *, cache_capacity: int = 32, include_builtins: bool = True) -> None:
        if cache_capacity < 1:
            raise ValueError("cache_capacity must be at least 1")
        self._cache_capacity = cache_capacity
        self._builders: dict[str, ModelViewBuilder[ModelView]] = {}
        self._cache: OrderedDict[_CacheSlot, _CacheEntry] = OrderedDict()
        self._lock = RLock()
        if include_builtins:
            self.register(ModuleTopologyGraphBuilder())

    def register(self, builder: ModelViewBuilder[ModelView]) -> None:
        """Register ``builder`` or reject a duplicate stable builder ID."""
        identifier = builder.builder_id
        try:
            validated_identifier = _IDENTIFIER_ADAPTER.validate_python(identifier)
        except ValidationError as error:
            raise ValueError(
                f"invalid model-view builder ID '{identifier}'; expected lowercase snake_case"
            ) from error
        if identifier != validated_identifier:
            raise ValueError("model-view builder ID must not contain surrounding whitespace")
        # Fail invalid mode declarations while registering, before a UI opens
        # the builder catalog or a Robot Pack recipe attempts to use it.
        builder.descriptor()
        with self._lock:
            if identifier in self._builders:
                raise DuplicateModelViewBuilderError(
                    f"model-view builder '{identifier}' is already registered"
                )
            self._builders[identifier] = builder

    def descriptors(self) -> tuple[ModelViewDescriptor, ...]:
        """Return every registered builder descriptor in deterministic order."""
        with self._lock:
            builders = tuple(self._builders.values())
        return tuple(builder.descriptor() for builder in sorted(builders, key=_builder_key))

    def available(
        self,
        context: ModelViewContext,
        *,
        mode: ModelViewMode | None = None,
    ) -> tuple[ModelViewDescriptor, ...]:
        """Return builders compatible with available inputs and optional mode."""
        with self._lock:
            builders = tuple(self._builders.values())
        compatible = (
            builder
            for builder in builders
            if builder.is_available(context) and (mode is None or mode in builder.modes)
        )
        return tuple(builder.descriptor() for builder in sorted(compatible, key=_builder_key))

    def build(self, recipe: ModelViewSpec, context: ModelViewContext) -> ModelView:
        """Build one recipe, returning the cached latest result when valid."""
        builder = self._builder(recipe.builder)
        if not builder.is_available(context):
            raise ModelViewUnavailableError(
                f"model-view recipe '{recipe.id}' requires inputs unavailable to "
                f"builder '{builder.builder_id}'"
            )
        unsupported_modes = tuple(mode for mode in recipe.modes if mode not in builder.modes)
        if unsupported_modes:
            names = ", ".join(mode.value for mode in unsupported_modes)
            raise ModelViewBuildError(
                f"builder '{builder.builder_id}' does not support recipe mode(s): {names}"
            )

        source = _source_stamp(context)
        slot = (
            id(context.pack),
            id(context.world) if context.world is not None else None,
            recipe.id,
        )
        fingerprint = _recipe_fingerprint(recipe)
        source_token = builder.cache_token(context)
        with self._lock:
            cached = self._cache.get(slot)
            if (
                cached is not None
                and cached.pack is context.pack
                and cached.world is context.world
                and cached.recipe_fingerprint == fingerprint
                and cached.source_token == source_token
            ):
                result = cached.result
                if result.source != source:
                    result = result.model_copy(update={"source": source})
                    self._cache[slot] = _CacheEntry(
                        pack=context.pack,
                        world=context.world,
                        recipe_fingerprint=fingerprint,
                        source_token=source_token,
                        result=result,
                    )
                self._cache.move_to_end(slot)
                return result

        result = builder.build(recipe, context, source)
        self._validate_result(recipe, builder, source, result)
        with self._lock:
            self._cache[slot] = _CacheEntry(
                pack=context.pack,
                world=context.world,
                recipe_fingerprint=fingerprint,
                source_token=source_token,
                result=result,
            )
            self._cache.move_to_end(slot)
            while len(self._cache) > self._cache_capacity:
                self._cache.popitem(last=False)
        return result

    def build_all(
        self,
        context: ModelViewContext,
        *,
        mode: ModelViewMode = ModelViewMode.RUNTIME,
        defaults_only: bool = False,
    ) -> tuple[ModelView, ...]:
        """Build declared recipes enabled for ``mode`` in manifest order."""
        recipes = (
            recipe
            for recipe in context.pack.manifest.model_views
            if mode in recipe.modes and (not defaults_only or recipe.default)
        )
        return tuple(self.build(recipe, context) for recipe in recipes)

    def build_default_runtime(self, context: ModelViewContext) -> tuple[ModelView, ...]:
        """Build every Robot Pack recipe marked as a default runtime view."""
        return self.build_all(
            context,
            mode=ModelViewMode.RUNTIME,
            defaults_only=True,
        )

    def invalidate(self, view_id: str | None = None) -> None:
        """Drop all cached results, or every cached context for one recipe ID."""
        with self._lock:
            if view_id is None:
                self._cache.clear()
                return
            slots = tuple(slot for slot in self._cache if slot[2] == view_id)
            for slot in slots:
                self._cache.pop(slot, None)

    def _builder(self, identifier: str) -> ModelViewBuilder[ModelView]:
        with self._lock:
            try:
                return self._builders[identifier]
            except KeyError as error:
                available = ", ".join(sorted(self._builders)) or "none"
                raise UnknownModelViewBuilderError(
                    f"model-view builder '{identifier}' is not registered; available: {available}"
                ) from error

    @staticmethod
    def _validate_result(
        recipe: ModelViewSpec,
        builder: ModelViewBuilder[ModelView],
        source: ModelViewSourceStamp,
        result: ModelView,
    ) -> None:
        if result.id != recipe.id:
            raise ModelViewBuildError(
                f"builder '{builder.builder_id}' returned view ID '{result.id}' "
                f"for recipe '{recipe.id}'"
            )
        if result.builder != builder.builder_id:
            raise ModelViewBuildError(
                f"builder '{builder.builder_id}' returned mismatched builder ID '{result.builder}'"
            )
        if result.view_type != builder.view_type:
            raise ModelViewBuildError(
                f"builder '{builder.builder_id}' returned view type '{result.view_type}', "
                f"expected '{builder.view_type}'"
            )
        if result.source != source:
            raise ModelViewBuildError(
                f"builder '{builder.builder_id}' returned a mismatched source stamp"
            )


def _builder_key(builder: ModelViewBuilder[ModelView]) -> str:
    return builder.builder_id


def _recipe_fingerprint(recipe: ModelViewSpec) -> str:
    return json.dumps(
        recipe.model_dump(mode="json"),
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def _source_stamp(context: ModelViewContext) -> ModelViewSourceStamp:
    world = context.world
    if world is None:
        return ModelViewSourceStamp(
            pack_id=context.pack.id,
            pack_version=context.pack.version,
        )
    revision = world.revision
    return ModelViewSourceStamp(
        pack_id=context.pack.id,
        pack_version=context.pack.version,
        world_time_s=world.time_s,
        sample_sequence=revision.sample_sequence,
        topology_revision=revision.topology_revision,
        docking_revision=revision.docking_revision,
        event_revision=revision.event_revision,
    )
