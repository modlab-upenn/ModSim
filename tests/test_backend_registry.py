"""Tests for named backend selection."""

from __future__ import annotations

import pytest

from modsim.backends.base import BackendAdapter, BackendError
from modsim.backends.mock import MockBackendAdapter
from modsim.backends.registry import (
    BACKEND_ENV_VAR,
    DEFAULT_BACKEND,
    BackendEntry,
    BackendUnavailableError,
    UnknownBackendError,
    available_backends,
    backend_entry,
    create_backend,
    describe_backends,
    installed_backends,
    register_backend,
    resolve_backend_name,
)


def test_builtin_backends_are_registered() -> None:
    assert "mock" in available_backends()
    assert "mujoco" in available_backends()


def test_the_mock_backend_is_always_installed() -> None:
    assert "mock" in installed_backends()


def test_create_backend_defaults_to_the_mock(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(BACKEND_ENV_VAR, raising=False)
    adapter = create_backend()

    assert isinstance(adapter, MockBackendAdapter)
    assert isinstance(adapter, BackendAdapter)
    assert adapter.capabilities().name == DEFAULT_BACKEND


def test_the_environment_variable_selects_a_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(BACKEND_ENV_VAR, "mock")
    assert resolve_backend_name() == "mock"

    monkeypatch.setenv(BACKEND_ENV_VAR, "mujoco")
    assert resolve_backend_name() == "mujoco"


def test_an_explicit_name_beats_the_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(BACKEND_ENV_VAR, "mujoco")
    assert resolve_backend_name("mock") == "mock"


def test_a_blank_environment_value_falls_back_to_the_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(BACKEND_ENV_VAR, "   ")
    assert resolve_backend_name() == DEFAULT_BACKEND


def test_unknown_backends_list_what_is_available() -> None:
    with pytest.raises(UnknownBackendError, match="available: mock"):
        create_backend("not_a_backend")


def test_rejected_options_name_the_backend() -> None:
    with pytest.raises(BackendError, match="backend 'mock' rejected the supplied options"):
        create_backend("mock", gravity=(0.0, 0.0, 0.0))


def test_a_missing_dependency_reports_an_install_hint() -> None:
    def fail() -> None:
        raise ImportError("no module named 'pretend_engine'")

    register_backend(
        BackendEntry(
            name="pretend",
            summary="Test-only backend.",
            loader=fail,  # pyright: ignore[reportArgumentType]
            install_hint="Install with: pip install pretend",
        ),
        replace=True,
    )
    try:
        entry = backend_entry("pretend")
        assert not entry.installed
        with pytest.raises(BackendUnavailableError, match="pip install pretend"):
            create_backend("pretend")
    finally:
        register_backend(
            BackendEntry(name="pretend", summary="", loader=fail),  # pyright: ignore[reportArgumentType]
            replace=True,
        )


def test_registering_a_duplicate_name_requires_replace() -> None:
    with pytest.raises(BackendError, match="already registered"):
        register_backend(
            BackendEntry(name="mock", summary="duplicate", loader=lambda: MockBackendAdapter)
        )


def test_describe_backends_reports_name_installed_and_summary() -> None:
    described = {name: (installed, summary) for name, installed, summary in describe_backends()}

    assert described["mock"][0] is True
    assert "Kinematic" in described["mock"][1]
