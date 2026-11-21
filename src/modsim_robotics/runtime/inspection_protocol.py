"""Versioned newline JSON protocol for out-of-process runtime inspection.

Protocol records are deliberately ordinary strict Pydantic values. They carry
the same immutable :class:`RuntimeInspectorFrame` used at the Qt thread
boundary and can later be transported over a socket without exposing a live
runtime session or relying on Python pickle.
"""

from __future__ import annotations

import json
from enum import StrEnum
from typing import Annotated, Literal, TypeAlias, cast

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError, field_validator

from modsim_robotics.runtime.inspection import RuntimeInspectorFrame
from modsim_robotics.runtime.inspector_runner import RuntimeInspectorConfig
from modsim_robotics.units import to_si

RUNTIME_PROTOCOL_VERSION = 4
RUNTIME_PROTOCOL_PREFIX = "MODSIM_RUNTIME/4 "
MAX_RUNTIME_MESSAGE_BYTES = 8 * 1024 * 1024

_PREFIX_BYTES = RUNTIME_PROTOCOL_PREFIX.encode("ascii")
_LEGACY_PREFIX_BYTES = b"MODSIM_RUNTIME/3 "


class RuntimeProtocolError(ValueError):
    """Raised when a runtime protocol record is malformed or unsupported."""


class RuntimeFinishedReason(StrEnum):
    """Terminal reason reported before a runtime host process exits."""

    COMPLETED = "completed"
    STOPPED = "stopped"
    VIEWER_CLOSED = "viewer_closed"
    FAILED = "failed"


class _RuntimeProtocolDTO(BaseModel):
    model_config = ConfigDict(
        allow_inf_nan=False,
        extra="forbid",
        frozen=True,
        strict=True,
        str_strip_whitespace=True,
        validate_default=True,
    )

    protocol_version: Literal[4] = RUNTIME_PROTOCOL_VERSION

    @field_validator("protocol_version", mode="before")
    @classmethod
    def _require_integer_version(cls, value: object) -> int:
        if type(value) is not int:
            raise ValueError("protocol_version must be an integer")
        return value


class RuntimeHello(_RuntimeProtocolDTO):
    """First child-to-parent record announcing protocol readiness."""

    kind: Literal["hello"] = "hello"


class RuntimeInitialize(_RuntimeProtocolDTO):
    """Parent-to-child request containing the complete launch configuration."""

    kind: Literal["initialize"] = "initialize"
    config: RuntimeInspectorConfig


class RuntimeStop(_RuntimeProtocolDTO):
    """Parent-to-child cooperative shutdown request."""

    kind: Literal["stop"] = "stop"


class RuntimeSetPaused(_RuntimeProtocolDTO):
    """Parent-to-child request to pause or resume authoritative stepping."""

    kind: Literal["set_paused"] = "set_paused"
    paused: bool


class RuntimeStatus(_RuntimeProtocolDTO):
    """Concise child execution status suitable for the Inspector header."""

    kind: Literal["status"] = "status"
    message: str = Field(min_length=1, max_length=4096)


class RuntimeFrame(_RuntimeProtocolDTO):
    """One immutable, lossless runtime update."""

    kind: Literal["frame"] = "frame"
    frame: RuntimeInspectorFrame


class RuntimePlaybackState(_RuntimeProtocolDTO):
    """Child-to-parent acknowledgement of authoritative playback state."""

    kind: Literal["playback_state"] = "playback_state"
    paused: bool


class RuntimeError(_RuntimeProtocolDTO):
    """One fatal child error; full diagnostics remain on child stderr."""

    kind: Literal["error"] = "error"
    message: str = Field(min_length=1, max_length=8192)


class RuntimeFinished(_RuntimeProtocolDTO):
    """Terminal child status emitted after runtime/backend cleanup."""

    kind: Literal["finished"] = "finished"
    reason: RuntimeFinishedReason


RuntimeProtocolMessage: TypeAlias = Annotated[
    RuntimeHello
    | RuntimeInitialize
    | RuntimeStop
    | RuntimeSetPaused
    | RuntimeStatus
    | RuntimeFrame
    | RuntimePlaybackState
    | RuntimeError
    | RuntimeFinished,
    Field(discriminator="kind"),
]

_MESSAGE_ADAPTER: TypeAdapter[RuntimeProtocolMessage] = TypeAdapter(RuntimeProtocolMessage)


def _upgrade_legacy_planning(document: dict[str, object]) -> None:
    """Normalize only declared v3 telemetry fields before strict JSON validation."""
    raw_frame = document.get("frame")
    if not isinstance(raw_frame, dict):
        return
    frame = cast(dict[str, object], raw_frame)
    for name in ("planning", "lattice_planning"):
        raw_planning = frame.get(name)
        if not isinstance(raw_planning, dict):
            continue
        planning = cast(dict[str, object], raw_planning)
        mappings = [("planning_ms", "planning_time_s", "ms")]
        if name == "lattice_planning":
            mappings.extend(
                [
                    ("flywheel_rpm", "flywheel_angular_velocity_rad_s", "rpm"),
                    ("pivot_angle_deg", "pivot_angle_rad", "deg"),
                ]
            )
        for old, new, unit in mappings:
            if old in planning:
                if new in planning:
                    raise ValueError(f"Legacy record contains both {old} and {new}.")
                planning[new] = to_si(cast(float, planning.pop(old)), unit)


def encode_runtime_message(message: RuntimeProtocolMessage) -> bytes:
    """Encode one typed record as a prefixed UTF-8 JSON line."""
    try:
        payload = message.model_dump_json().encode("utf-8")
    except (TypeError, ValueError) as error:
        raise RuntimeProtocolError(f"runtime message could not be encoded: {error}") from error
    encoded = _PREFIX_BYTES + payload + b"\n"
    if len(encoded) > MAX_RUNTIME_MESSAGE_BYTES:
        raise RuntimeProtocolError(
            f"runtime message exceeds the {MAX_RUNTIME_MESSAGE_BYTES}-byte protocol limit"
        )
    return encoded


def decode_runtime_message(
    line: bytes | bytearray | memoryview | str,
) -> RuntimeProtocolMessage:
    """Validate and decode exactly one complete prefixed JSON line."""
    if isinstance(line, str):
        try:
            encoded = line.encode("utf-8")
        except UnicodeEncodeError as error:
            raise RuntimeProtocolError("runtime message is not valid UTF-8") from error
    else:
        encoded = bytes(line)
    if len(encoded) > MAX_RUNTIME_MESSAGE_BYTES:
        raise RuntimeProtocolError(
            f"runtime message exceeds the {MAX_RUNTIME_MESSAGE_BYTES}-byte protocol limit"
        )
    if not encoded.endswith(b"\n"):
        raise RuntimeProtocolError("runtime message must end with a newline")
    record = encoded[:-1]
    if record.endswith(b"\r"):
        record = record[:-1]
    if b"\n" in record or b"\r" in record:
        raise RuntimeProtocolError("runtime message must contain exactly one line")
    legacy = record.startswith(_LEGACY_PREFIX_BYTES)
    if not record.startswith(_PREFIX_BYTES) and not legacy:
        raise RuntimeProtocolError(f"runtime message must begin with '{RUNTIME_PROTOCOL_PREFIX}'")
    prefix = _LEGACY_PREFIX_BYTES if legacy else _PREFIX_BYTES
    payload = record[len(prefix) :]
    if not payload:
        raise RuntimeProtocolError("runtime message has an empty JSON payload")
    try:
        if legacy:
            document: object = json.loads(payload)
            if not isinstance(document, dict):
                raise ValueError("legacy message must be a JSON object")
            migrated = dict(cast(dict[str, object], document))
            if (
                type(migrated.get("protocol_version")) is not int
                or migrated["protocol_version"] != 3
            ):
                raise ValueError("legacy prefix requires protocol_version 3")
            migrated["protocol_version"] = RUNTIME_PROTOCOL_VERSION
            _upgrade_legacy_planning(migrated)
            payload = json.dumps(migrated, allow_nan=False).encode("utf-8")
        return _MESSAGE_ADAPTER.validate_json(payload)
    except (ValidationError, ValueError) as error:
        raise RuntimeProtocolError(f"invalid runtime protocol message: {error}") from error


class RuntimeProtocolFramer:
    """Incrementally split byte-stream chunks into validated protocol records."""

    __slots__ = ("_buffer",)

    def __init__(self) -> None:
        self._buffer = bytearray()

    @property
    def buffered_bytes(self) -> int:
        """Return bytes awaiting the next newline."""
        return len(self._buffer)

    def feed(self, data: bytes | bytearray | memoryview) -> tuple[RuntimeProtocolMessage, ...]:
        """Consume a stream chunk and return every complete message in order."""
        self._buffer.extend(data)
        messages: list[RuntimeProtocolMessage] = []
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                if len(self._buffer) >= MAX_RUNTIME_MESSAGE_BYTES:
                    self._buffer.clear()
                    raise RuntimeProtocolError(
                        "unterminated runtime message exceeds the protocol limit"
                    )
                break
            line_length = newline + 1
            if line_length > MAX_RUNTIME_MESSAGE_BYTES:
                self._buffer.clear()
                raise RuntimeProtocolError(
                    f"runtime message exceeds the {MAX_RUNTIME_MESSAGE_BYTES}-byte protocol limit"
                )
            line = bytes(self._buffer[:line_length])
            del self._buffer[:line_length]
            messages.append(decode_runtime_message(line))
        return tuple(messages)

    def finish(self) -> tuple[RuntimeProtocolMessage, ...]:
        """Validate end-of-stream, rejecting any unterminated record."""
        if self._buffer:
            buffered = len(self._buffer)
            self._buffer.clear()
            raise RuntimeProtocolError(
                f"runtime protocol stream ended with {buffered} unterminated byte(s)"
            )
        return ()


# Compatibility aliases keep message-oriented call sites readable while the
# shorter names form the public protocol vocabulary used by Qt controllers.
RuntimeHelloMessage = RuntimeHello
RuntimeInitMessage = RuntimeInitialize
RuntimeStopMessage = RuntimeStop
RuntimeSetPausedMessage = RuntimeSetPaused
RuntimeStatusMessage = RuntimeStatus
RuntimeFrameMessage = RuntimeFrame
RuntimePlaybackStateMessage = RuntimePlaybackState
RuntimeErrorMessage = RuntimeError
RuntimeFinishedMessage = RuntimeFinished
RuntimeProtocolDecoder = RuntimeProtocolFramer


__all__ = [
    "MAX_RUNTIME_MESSAGE_BYTES",
    "RUNTIME_PROTOCOL_PREFIX",
    "RUNTIME_PROTOCOL_VERSION",
    "RuntimeError",
    "RuntimeErrorMessage",
    "RuntimeFinished",
    "RuntimeFinishedMessage",
    "RuntimeFinishedReason",
    "RuntimeFrame",
    "RuntimeFrameMessage",
    "RuntimeHello",
    "RuntimeHelloMessage",
    "RuntimeInitMessage",
    "RuntimeInitialize",
    "RuntimePlaybackState",
    "RuntimePlaybackStateMessage",
    "RuntimeProtocolDecoder",
    "RuntimeProtocolError",
    "RuntimeProtocolFramer",
    "RuntimeProtocolMessage",
    "RuntimeSetPaused",
    "RuntimeSetPausedMessage",
    "RuntimeStatus",
    "RuntimeStatusMessage",
    "RuntimeStop",
    "RuntimeStopMessage",
    "decode_runtime_message",
    "encode_runtime_message",
]
