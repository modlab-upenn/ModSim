"""Named backend selection.

Backends are chosen by name so that a script, a test, or the CLI can swap
physics engines without importing an adapter class. Resolution order is
explicit argument, then the ``MODSIM_BACKEND`` environment variable, then the
default.

Every entry loads its adapter lazily. That is what keeps the rule in
``docs/AGENTS.md`` true — importing ``modsim`` never imports ``mujoco``, VTK, or
any other optional engine dependency.
"""

from __future__ import annotations

import os
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from modsim.backends.base import BackendAdapter, BackendError

DEFAULT_BACKEND = "mock"
BACKEND_ENV_VAR = "MODSIM_BACKEND"

BackendFactory = Callable[..., BackendAdapter]


class BackendUnavailableError(BackendError):
    """Raised when a backend is known but its dependencies are not installed."""


class UnknownBackendError(BackendError):
    """Raised when a backend name is not registered."""


@dataclass(frozen=True, slots=True)
class BackendEntry:
    """One registered backend and how to construct it."""

    name: str
    summary: str
    loader: Callable[[], BackendFactory]
    install_hint: str | None = None

    def factory(self) -> BackendFactory:
        """Import and return the adapter factory, or explain what is missing."""
        try:
            return self.loader()
        except ImportError as error:
            hint = self.install_hint or f"install the '{self.name}' backend dependencies"
            raise BackendUnavailableError(
                f"backend '{self.name}' is not available: {error}. {hint}"
            ) from error

    @property
    def installed(self) -> bool:
        """Whether this backend's dependencies can be imported right now."""
        try:
            self.loader()
        except ImportError:
            return False
        return True


def _load_mock() -> BackendFactory:
    from modsim.backends.mock import MockBackendAdapter

    return MockBackendAdapter


def _load_mujoco() -> BackendFactory:
    from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter

    return MuJoCoBackendAdapter


_REGISTRY: dict[str, BackendEntry] = {
    "mock": BackendEntry(
        name="mock",
        summary="Kinematic reference backend. No physics, no dependencies.",
        loader=_load_mock,
    ),
    "mujoco": BackendEntry(
        name="mujoco",
        summary="MuJoCo rigid-body physics.",
        loader=_load_mujoco,
        install_hint="Install with: pip install 'modsim-robotics[mujoco]'",
    ),
}


def register_backend(entry: BackendEntry, *, replace: bool = False) -> None:
    """Register a backend so it can be selected by name."""
    if entry.name in _REGISTRY and not replace:
        raise BackendError(f"backend '{entry.name}' is already registered")
    _REGISTRY[entry.name] = entry


def backend_entry(name: str) -> BackendEntry:
    """Return one registered backend entry."""
    try:
        return _REGISTRY[name]
    except KeyError as error:
        known = ", ".join(available_backends())
        raise UnknownBackendError(f"unknown backend '{name}'; available: {known}") from error


def available_backends() -> tuple[str, ...]:
    """Return every registered backend name."""
    return tuple(sorted(_REGISTRY))


def installed_backends() -> tuple[str, ...]:
    """Return the backend names whose dependencies are importable."""
    return tuple(name for name in available_backends() if _REGISTRY[name].installed)


def describe_backends() -> tuple[tuple[str, bool, str], ...]:
    """Return ``(name, installed, summary)`` for every registered backend."""
    return tuple(
        (name, _REGISTRY[name].installed, _REGISTRY[name].summary) for name in available_backends()
    )


def resolve_backend_name(name: str | None = None) -> str:
    """Return the backend to use: explicit, then environment, then default."""
    if name:
        return name
    from_environment = os.environ.get(BACKEND_ENV_VAR, "").strip()
    return from_environment or DEFAULT_BACKEND


def create_backend(name: str | None = None, **options: Any) -> BackendAdapter:
    """Construct a backend adapter by name.

    ``options`` are forwarded to the adapter's constructor, so engine-specific
    settings such as MuJoCo's gravity or timestep stay out of the shared
    interface.
    """
    resolved = resolve_backend_name(name)
    factory = backend_entry(resolved).factory()
    try:
        return factory(**options)
    except TypeError as error:
        raise BackendError(
            f"backend '{resolved}' rejected the supplied options "
            f"({', '.join(sorted(options)) or 'none'}): {error}"
        ) from error
