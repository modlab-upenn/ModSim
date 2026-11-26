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

from modsim.backends.base import BackendAdapter, BackendCapabilities, BackendError
from modsim.robot_packs.schema import ControlMode, PhysicalConstraintType

DEFAULT_BACKEND = "mock"
BACKEND_ENV_VAR = "MODSIM_BACKEND"

BackendFactory = Callable[..., BackendAdapter]
ViewerRunner = Callable[..., Any]
"""Runs a session with the backend's native viewer open; see ``BackendEntry.viewer``."""


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
    viewer_loader: Callable[[], ViewerRunner] | None = None
    """Lazily import the backend's native viewer runner, if it has one.

    The runner is called as ``runner(session, duration_s=..., step_once=...)``
    and returns the events produced while the viewer was open.
    """
    capabilities: BackendCapabilities | None = None
    """What an adapter built with default options reports, declared up front.

    Lets callers validate a request against an optional engine without importing
    it, so a missing install is reported where the engine is actually needed.
    Must equal ``factory()().capabilities()``; when ``None`` that is used instead.
    """

    def factory(self) -> BackendFactory:
        """Import and return the adapter factory, or explain what is missing."""
        try:
            return self.loader()
        except ImportError as error:
            hint = self.install_hint or f"install the '{self.name}' backend dependencies"
            raise BackendUnavailableError(
                f"backend '{self.name}' is not available: {error}. {hint}"
            ) from error

    def viewer(self) -> ViewerRunner:
        """Import and return the native viewer runner, or explain why there is none."""
        if self.viewer_loader is None:
            raise BackendError(f"backend '{self.name}' has no native viewer")
        try:
            return self.viewer_loader()
        except ImportError as error:
            hint = self.install_hint or f"install the '{self.name}' backend dependencies"
            raise BackendUnavailableError(
                f"the '{self.name}' viewer is not available: {error}. {hint}"
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


def _load_mujoco_viewer() -> ViewerRunner:
    from modsim_backend_mujoco.viewer import run_with_viewer

    return run_with_viewer


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
        viewer_loader=_load_mujoco_viewer,
        capabilities=BackendCapabilities(
            name="mujoco",
            supports_runtime_constraints=True,
            supports_constraint_removal=True,
            supports_constraint_forces=True,
            supports_constraint_torques=True,
            supports_contact_forces=True,
            supports_module_pose_write=True,
            supports_joint_commands=True,
            supported_joint_control_modes=frozenset({ControlMode.EFFORT}),
            supports_external_viewer=True,
            supports_gravity=True,
            supports_ground_plane=True,
            supports_contact_dynamics=True,
            supported_constraint_types=frozenset(
                {PhysicalConstraintType.FIXED, PhysicalConstraintType.HINGE}
            ),
            connection_change_delay_substeps=1,
        ),
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


def backend_capabilities(name: str | None = None) -> BackendCapabilities:
    """Return what a backend can do, without loading a scene.

    Adapters must report capabilities that do not depend on a loaded scene.
    A backend that declares them in its registry entry is answered without
    importing its engine; otherwise this constructs an unloaded adapter with
    default options and asks it. Callers use this to refuse an unsupported
    request before any engine work.
    """
    entry = backend_entry(resolve_backend_name(name))
    if entry.capabilities is not None:
        return entry.capabilities
    return entry.factory()().capabilities()


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
