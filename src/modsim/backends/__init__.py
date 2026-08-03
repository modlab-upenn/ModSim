"""Backend adapter contract and the built-in mock backend."""

from modsim.backends.base import (
    BackendAdapter,
    BackendCapabilities,
    BackendError,
    BackendHandleRegistry,
    ConnectionOutcome,
    ConnectionRequest,
)
from modsim.backends.mock import MOCK_BACKEND_NAME, MockBackendAdapter

__all__ = [
    "MOCK_BACKEND_NAME",
    "BackendAdapter",
    "BackendCapabilities",
    "BackendError",
    "BackendHandleRegistry",
    "ConnectionOutcome",
    "ConnectionRequest",
    "MockBackendAdapter",
]
