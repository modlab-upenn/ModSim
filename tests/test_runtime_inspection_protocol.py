"""Strict, transport-neutral Runtime Inspector protocol tests."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from modsim.runtime import (
    ReconfigurationPhase,
    ReconfigurationStatus,
    RuntimeDemo,
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
)
from modsim.runtime.inspection_protocol import (
    MAX_RUNTIME_MESSAGE_BYTES,
    RUNTIME_PROTOCOL_PREFIX,
    RuntimeError,
    RuntimeFinished,
    RuntimeFinishedReason,
    RuntimeFrame,
    RuntimeHello,
    RuntimeInitialize,
    RuntimePlaybackState,
    RuntimeProtocolError,
    RuntimeProtocolFramer,
    RuntimeSetPaused,
    RuntimeStatus,
    RuntimeStop,
    decode_runtime_message,
    encode_runtime_message,
)


def test_protocol_round_trips_every_message_and_nested_path(
    example_pack_dir: Path,
) -> None:
    config = RuntimeInspectorConfig(
        pack_path=example_pack_dir,
        demo=RuntimeDemo.DOCK_UNDOCK,
        backend="mock",
        viewer_enabled=True,
    )
    runner = RuntimeInspectorRunner.create(config)
    try:
        frame = runner.frame()
    finally:
        runner.shutdown()

    messages = (
        RuntimeHello(),
        RuntimeInitialize(config=config),
        RuntimeStop(),
        RuntimeStatus(message="Loading"),
        RuntimeFrame(frame=frame),
        RuntimeError(message="Backend failed"),
        RuntimeFinished(reason=RuntimeFinishedReason.VIEWER_CLOSED),
        RuntimeSetPaused(paused=True),
        RuntimePlaybackState(paused=True),
    )

    decoded = tuple(decode_runtime_message(encode_runtime_message(message)) for message in messages)

    assert decoded == messages
    initialize = decoded[1]
    assert isinstance(initialize, RuntimeInitialize)
    assert isinstance(initialize.config.pack_path, Path)
    assert initialize.config.viewer_enabled
    assert initialize.config.demo is RuntimeDemo.DOCK_UNDOCK
    transported = decoded[4]
    assert isinstance(transported, RuntimeFrame)
    assert transported.frame == frame


@pytest.mark.parametrize(
    "phase",
    [
        ReconfigurationPhase.APPROACHING,
        ReconfigurationPhase.STOPPED,
        ReconfigurationPhase.TIMED_OUT,
    ],
)
def test_reconfiguration_status_round_trips_as_a_strict_runtime_frame(
    example_pack_dir: Path,
    phase: ReconfigurationPhase,
) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(pack_path=example_pack_dir, backend="mock")
    )
    try:
        base_frame = runner.frame()
    finally:
        runner.shutdown()
    status = ReconfigurationStatus(
        phase=phase,
        time_s=2.0,
        plan_id=RuntimeDemo.SMORES_DRIVER_TO_SNAKE.value,
        plan_name="SMORES-EP Driver to Snake",
        action_index=1,
        action_count=4,
        detail="Moving module 7 toward module_6/pan",
    )
    message = RuntimeFrame(frame=base_frame.model_copy(update={"scenario": status}))

    encoded = encode_runtime_message(message)
    decoded = decode_runtime_message(encoded)

    assert isinstance(decoded, RuntimeFrame)
    assert isinstance(decoded.frame.scenario, ReconfigurationStatus)
    assert decoded.frame.scenario == status
    assert decoded.frame.scenario.phase is phase

    document = json.loads(encoded.removeprefix(RUNTIME_PROTOCOL_PREFIX.encode()).decode())
    document["frame"]["scenario"]["action_index"] = "1"
    malformed = (
        RUNTIME_PROTOCOL_PREFIX.encode()
        + json.dumps(document, separators=(",", ":")).encode()
        + b"\n"
    )
    with pytest.raises(RuntimeProtocolError, match="invalid runtime protocol"):
        decode_runtime_message(malformed)


def test_protocol_framer_preserves_fragmented_message_order() -> None:
    encoded = b"".join(
        (
            encode_runtime_message(RuntimeHello()),
            encode_runtime_message(RuntimeStatus(message="Ready")),
            encode_runtime_message(RuntimeFinished(reason=RuntimeFinishedReason.COMPLETED)),
        )
    )
    framer = RuntimeProtocolFramer()

    first = framer.feed(encoded[:7])
    second = framer.feed(encoded[7:-4])
    third = framer.feed(encoded[-4:])

    assert first == ()
    assert [type(message) for message in second + third] == [
        RuntimeHello,
        RuntimeStatus,
        RuntimeFinished,
    ]
    assert framer.buffered_bytes == 0
    assert framer.finish() == ()


@pytest.mark.parametrize(
    "record, expected",
    [
        (b'{"protocol_version":3,"kind":"hello"}\n', "must begin"),
        (
            b'MODSIM_RUNTIME/3 {"protocol_version":1,"kind":"hello"}\n',
            "invalid runtime protocol",
        ),
        (
            b'MODSIM_RUNTIME/3 {"protocol_version":3,"kind":"hello","extra":1}\n',
            "invalid runtime protocol",
        ),
        (b'MODSIM_RUNTIME/3 {"protocol_version":3,"kind":"hello"}', "end with"),
    ],
)
def test_protocol_rejects_malformed_or_unsupported_records(
    record: bytes,
    expected: str,
) -> None:
    with pytest.raises(RuntimeProtocolError, match=expected):
        decode_runtime_message(record)


def test_protocol_framer_bounds_unterminated_input_and_rejects_eof_fragment() -> None:
    framer = RuntimeProtocolFramer()
    with pytest.raises(RuntimeProtocolError, match="exceeds"):
        framer.feed(b"x" * MAX_RUNTIME_MESSAGE_BYTES)
    assert framer.buffered_bytes == 0

    framer.feed(b"MODSIM_RUNTIME/3 ")
    with pytest.raises(RuntimeProtocolError, match="unterminated"):
        framer.finish()
    assert framer.buffered_bytes == 0
