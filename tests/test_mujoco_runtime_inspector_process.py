"""Protocol and lifecycle tests for the no-Qt MuJoCo viewer child."""

from __future__ import annotations

import io
import math
import os
import queue
from contextlib import suppress
from pathlib import Path
from threading import Event as ThreadEvent
from threading import enumerate as enumerate_threads
from typing import Any

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.runtime.inspection_protocol import (
    RuntimeError,
    RuntimeFinished,
    RuntimeFinishedReason,
    RuntimeFrame,
    RuntimeHello,
    RuntimeInitialize,
    RuntimePlaybackState,
    RuntimeProtocolFramer,
    RuntimeSetPaused,
    RuntimeStatus,
    RuntimeStop,
    encode_runtime_message,
)
from modsim.runtime.inspector_runner import RuntimeInspectorConfig
from modsim_backend_mujoco import runtime_inspector_process as runtime_process

pytestmark = pytest.mark.mujoco


class _TerminalClosingOutput(io.BytesIO):
    """Model the Qt parent closing stdin as soon as Finished is received."""

    def __init__(self, parent_control: io.BufferedWriter) -> None:
        super().__init__()
        self._parent_control = parent_control

    def write(self, buffer: Any, /) -> int:
        written = super().write(buffer)
        if b'"kind":"finished"' in bytes(buffer):
            self._parent_control.close()
        return written


def _config(pack_path: Path, **overrides: Any) -> RuntimeInspectorConfig:
    values: dict[str, Any] = {
        "pack_path": pack_path,
        "backend": "mujoco",
        "connector_gap_m": 0.005,
        "approach_m_s": 0.03,
        "duration_s": 0.05,
        "dt_s": 0.01,
        "publish_hz": 100.0,
        "viewer_enabled": True,
    }
    values.update(overrides)
    return RuntimeInspectorConfig(**values)


def _initialize(config: RuntimeInspectorConfig) -> bytes:
    return encode_runtime_message(RuntimeInitialize(config=config))


def _messages(output: io.BytesIO) -> tuple[object, ...]:
    framer = RuntimeProtocolFramer()
    messages = framer.feed(output.getvalue())
    framer.finish()
    return messages


def test_process_streams_frames_from_the_same_mujoco_runtime(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    read_descriptor, write_descriptor = os.pipe()
    control = os.fdopen(read_descriptor, "rb")
    parent_control = os.fdopen(write_descriptor, "wb")
    expected_real_time_factor = 3.0
    parent_control.write(
        _initialize(
            _config(
                example_pack_dir,
                real_time_factor=expected_real_time_factor,
            )
        )
    )
    parent_control.flush()
    output = _TerminalClosingOutput(parent_control)
    diagnostics = io.StringIO()

    def run_fake_viewer(
        session: object,
        *,
        duration_s: float,
        step_once: Any,
        real_time_factor: float,
        hold: bool,
        stop_requested: Any,
        pause_requested: Any,
        key_callback: Any,
        on_started: Any,
        after_step: Any,
        on_pause_changed: Any,
        on_scenario_complete: Any,
        on_stopped: Any,
    ) -> tuple[()]:
        del session
        assert hold
        assert real_time_factor == expected_real_time_factor
        assert not pause_requested()
        assert callable(key_callback)
        on_started()
        on_pause_changed(False)
        for _ in range(math.ceil(duration_s / 0.01)):
            if stop_requested():
                break
            step_once()
            after_step()
        on_scenario_complete()
        on_stopped()
        return ()

    monkeypatch.setattr(runtime_process, "run_with_viewer", run_fake_viewer)
    try:
        exit_code = runtime_process.run_runtime_inspector_process(
            control,
            output,
            diagnostics,
        )
    finally:
        with suppress(BrokenPipeError):
            parent_control.close()
        if not control.closed:
            control.close()

    messages = _messages(output)
    frames = tuple(message.frame for message in messages if isinstance(message, RuntimeFrame))
    assert exit_code == 0
    assert diagnostics.getvalue() == ""
    assert isinstance(messages[0], RuntimeHello)
    assert any(isinstance(message, RuntimeStatus) for message in messages)
    assert any(message == RuntimePlaybackState(paused=False) for message in messages)
    assert frames[0].view.edges == ()
    assert len(frames[-1].view.edges) == 1
    assert [row.kind for frame in frames for row in frame.events] == [
        "DockCandidateDetected",
        "DockCommitted",
        "AssemblyMerged",
    ]
    assert messages[-1] == RuntimeFinished(reason=RuntimeFinishedReason.VIEWER_CLOSED)
    assert not any(
        thread.name == "ModSimRuntimeControl" and thread.is_alive()
        for thread in enumerate_threads()
    )


@pytest.mark.parametrize(
    "control_tail",
    (b"", encode_runtime_message(RuntimeStop())),
    ids=("stdin-eof", "stop-message"),
)
def test_process_treats_parent_stop_as_cooperative(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
    control_tail: bytes,
) -> None:
    output = io.BytesIO()
    diagnostics = io.StringIO()

    def run_until_stopped(
        session: object,
        *,
        real_time_factor: float,
        stop_requested: Any,
        on_started: Any,
        on_stopped: Any,
        **callbacks: Any,
    ) -> tuple[()]:
        del session, callbacks
        assert real_time_factor == 1.0
        on_started()
        assert stop_requested()
        on_stopped()
        return ()

    monkeypatch.setattr(runtime_process, "run_with_viewer", run_until_stopped)
    exit_code = runtime_process.run_runtime_inspector_process(
        io.BytesIO(_initialize(_config(example_pack_dir)) + control_tail),
        output,
        diagnostics,
    )

    messages = _messages(output)
    assert exit_code == 0
    assert diagnostics.getvalue() == ""
    assert messages[-1] == RuntimeFinished(reason=RuntimeFinishedReason.STOPPED)


def test_process_rejects_a_non_mujoco_initialization(example_pack_dir: Path) -> None:
    output = io.BytesIO()
    diagnostics = io.StringIO()
    exit_code = runtime_process.run_runtime_inspector_process(
        io.BytesIO(_initialize(_config(example_pack_dir, backend="mock"))),
        output,
        diagnostics,
    )

    messages = _messages(output)
    assert exit_code == 1
    assert isinstance(messages[0], RuntimeHello)
    assert isinstance(messages[-2], RuntimeError)
    assert "requires the MuJoCo backend" in messages[-2].message
    assert messages[-1] == RuntimeFinished(reason=RuntimeFinishedReason.FAILED)
    assert "RuntimeInspectorSetupError" in diagnostics.getvalue()


def test_process_reports_invalid_post_initialization_control(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    invalid_control = encode_runtime_message(RuntimeHello())
    output = io.BytesIO()
    diagnostics = io.StringIO()

    def run_until_stopped(
        session: object,
        *,
        real_time_factor: float,
        stop_requested: Any,
        on_started: Any,
        on_stopped: Any,
        **callbacks: Any,
    ) -> tuple[()]:
        del session, callbacks
        assert real_time_factor == 1.0
        on_started()
        assert stop_requested()
        on_stopped()
        return ()

    monkeypatch.setattr(runtime_process, "run_with_viewer", run_until_stopped)
    exit_code = runtime_process.run_runtime_inspector_process(
        io.BytesIO(_initialize(_config(example_pack_dir)) + invalid_control),
        output,
        diagnostics,
    )

    messages = _messages(output)
    assert exit_code == 1
    assert isinstance(messages[-2], RuntimeError)
    assert "accepts only set_paused or stop messages" in messages[-2].message
    assert messages[-1] == RuntimeFinished(reason=RuntimeFinishedReason.FAILED)
    assert "RuntimeProtocolError" in diagnostics.getvalue()


def test_control_reader_applies_pause_requests_in_order_before_stop() -> None:
    class PlaybackSpy:
        def __init__(self) -> None:
            self.values: list[bool] = []

        def set_paused(self, paused: bool) -> None:
            self.values.append(paused)

    playback = PlaybackSpy()
    stop_requested = ThreadEvent()
    errors: queue.SimpleQueue[Exception] = queue.SimpleQueue()
    stream = io.BytesIO(
        encode_runtime_message(RuntimeSetPaused(paused=True))
        + encode_runtime_message(RuntimeSetPaused(paused=False))
        + encode_runtime_message(RuntimeStop())
    )

    runtime_process._watch_control_stream(
        stream,
        stop_requested,
        playback,  # type: ignore[arg-type]
        ThreadEvent(),
        errors,
    )

    assert playback.values == [True, False]
    assert stop_requested.is_set()
    assert errors.empty()
