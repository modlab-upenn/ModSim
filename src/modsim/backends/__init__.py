"""Backend adapter contract and the built-in mock backend."""

from modsim.backends.base import (
    BackendAdapter,
    BackendCapabilities,
    BackendError,
    BackendHandleRegistry,
    ConnectionOutcome,
    ConnectionRequest,
    SupportsJointCommands,
)
from modsim.backends.mock import MOCK_BACKEND_NAME, MockBackendAdapter
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

__all__ = [
    "BACKEND_ENV_VAR",
    "DEFAULT_BACKEND",
    "MOCK_BACKEND_NAME",
    "BackendAdapter",
    "BackendCapabilities",
    "BackendEntry",
    "BackendError",
    "BackendHandleRegistry",
    "BackendUnavailableError",
    "ConnectionOutcome",
    "ConnectionRequest",
    "MockBackendAdapter",
    "SupportsJointCommands",
    "UnknownBackendError",
    "available_backends",
    "backend_entry",
    "create_backend",
    "describe_backends",
    "installed_backends",
    "register_backend",
    "resolve_backend_name",
]
