"""Strict, transport-neutral Runtime Inspector protocol tests."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from modsim.model_views import CubicLatticeView
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

_ROOT = Path(__file__).resolve().parents[1]
_MBLOCKS_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"


def test_protocol_round_trips_every_message_and_nested_path(
    example_pack_dir: Path,
) -> None:
    config = RuntimeInspectorConfig(
        pack_path=example_pack_dir,
        demo=RuntimeDemo.DOCK_UNDOCK,
        backend="mock",
        viewer_enabled=True,
        real_time_factor=4.0,
    )
    runner = RuntimeInspectorRunner.create(config)
    try:
        frame = runner.frame()
    finally:
        runner.shutdown()

    messages = (
        RuntimeHello(),
        RuntimeInitialize(config=config),
        RuntimeSetPaused(paused=True),
        RuntimePlaybackState(paused=True),
        RuntimeStop(),
        RuntimeStatus(message="Loading"),
        RuntimeFrame(frame=frame),
        RuntimeError(message="Backend failed"),
        RuntimeFinished(reason=RuntimeFinishedReason.VIEWER_CLOSED),
    )

    decoded = tuple(decode_runtime_message(encode_runtime_message(message)) for message in messages)

    assert decoded == messages
    initialize = decoded[1]
    assert isinstance(initialize, RuntimeInitialize)
    assert isinstance(initialize.config.pack_path, Path)
    assert initialize.config.viewer_enabled
    assert initialize.config.real_time_factor == pytest.approx(4.0)
    assert initialize.config.demo is RuntimeDemo.DOCK_UNDOCK
    transported = next(message for message in decoded if isinstance(message, RuntimeFrame))
    assert isinstance(transported, RuntimeFrame)
    assert transported.frame == frame


def test_protocol_defaults_a_legacy_initialize_without_real_time_factor(
    example_pack_dir: Path,
) -> None:
    encoded = encode_runtime_message(
        RuntimeInitialize(
            config=RuntimeInspectorConfig(
                pack_path=example_pack_dir,
                backend="mock",
            )
        )
    )
    document = json.loads(encoded.removeprefix(RUNTIME_PROTOCOL_PREFIX.encode()).decode())
    assert document["config"].pop("real_time_factor") == 1.0
    legacy = (
        RUNTIME_PROTOCOL_PREFIX.encode()
        + json.dumps(document, separators=(",", ":")).encode()
        + b"\n"
    )

    decoded = decode_runtime_message(legacy)

    assert isinstance(decoded, RuntimeInitialize)
    assert decoded.config.real_time_factor == 1.0


def test_reconfiguration_status_round_trips_as_a_strict_runtime_frame(
    example_pack_dir: Path,
) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(pack_path=example_pack_dir, backend="mock")
    )
    try:
        base_frame = runner.frame()
    finally:
        runner.shutdown()
    status = ReconfigurationStatus(
        phase=ReconfigurationPhase.APPROACHING,
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
    assert decoded.frame.scenario.phase is ReconfigurationPhase.APPROACHING

    document = json.loads(encoded.removeprefix(RUNTIME_PROTOCOL_PREFIX.encode()).decode())
    document["frame"]["scenario"]["action_index"] = "1"
    malformed = (
        RUNTIME_PROTOCOL_PREFIX.encode()
        + json.dumps(document, separators=(",", ":")).encode()
        + b"\n"
    )
    with pytest.raises(RuntimeProtocolError, match="invalid runtime protocol"):
        decode_runtime_message(malformed)


def test_protocol_round_trips_a_concrete_cubic_lattice_frame() -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=_MBLOCKS_PACK_PATH,
            demo=RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
            backend="mock",
            duration_s=0.01,
            dt_s=0.01,
        )
    )
    try:
        frame = runner.frame()
    finally:
        runner.shutdown()

    decoded = decode_runtime_message(encode_runtime_message(RuntimeFrame(frame=frame)))

    assert isinstance(decoded, RuntimeFrame)
    assert isinstance(decoded.frame.view, CubicLatticeView)
    assert isinstance(frame.view, CubicLatticeView)
    assert decoded.frame == frame
    assert decoded.frame.view.orientation_catalog == frame.view.orientation_catalog


def test_protocol_framer_preserves_fragmented_message_order() -> None:
    encoded = b"".join(
        (
            encode_runtime_message(RuntimeHello()),
            encode_runtime_message(RuntimeSetPaused(paused=True)),
            encode_runtime_message(RuntimePlaybackState(paused=True)),
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
        RuntimeSetPaused,
        RuntimePlaybackState,
        RuntimeStatus,
        RuntimeFinished,
    ]
    assert framer.buffered_bytes == 0
    assert framer.finish() == ()


@pytest.mark.parametrize("kind", ("set_paused", "playback_state"))
def test_protocol_pause_state_requires_a_strict_boolean(kind: str) -> None:
    malformed = (
        RUNTIME_PROTOCOL_PREFIX.encode()
        + json.dumps(
            {"protocol_version": 1, "kind": kind, "paused": 1},
            separators=(",", ":"),
        ).encode()
        + b"\n"
    )

    with pytest.raises(RuntimeProtocolError, match="invalid runtime protocol"):
        decode_runtime_message(malformed)


@pytest.mark.parametrize(
    "record, expected",
    [
        (b'{"protocol_version":1,"kind":"hello"}\n', "must begin"),
        (
            b'MODSIM_RUNTIME/1 {"protocol_version":2,"kind":"hello"}\n',
            "invalid runtime protocol",
        ),
        (
            b'MODSIM_RUNTIME/1 {"protocol_version":1,"kind":"hello","extra":1}\n',
            "invalid runtime protocol",
        ),
        (b'MODSIM_RUNTIME/1 {"protocol_version":1,"kind":"hello"}', "end with"),
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

    framer.feed(b"MODSIM_RUNTIME/1 ")
    with pytest.raises(RuntimeProtocolError, match="unterminated"):
        framer.finish()
    assert framer.buffered_bytes == 0
