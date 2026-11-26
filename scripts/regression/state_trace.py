"""Exact, order-stable dumps of ModSim runtime state for A/B regression checks.

Every float is written with ``float.hex`` so two traces are equal only if every
bit (including the sign of zero) is equal. Mappings are written in sorted key
order, so a change in iteration order alone never reads as a regression.

``install(path, every=N)`` patches ``RuntimeSession.step`` so that every Nth
step of every session in the process appends one record. Reading the state to
write it forces any lazily derived value (module roots, connector frames) to
resolve; :func:`install` with ``every=0`` records only the final state of each
session, which leaves laziness untouched during the run.
"""

from __future__ import annotations

import atexit
import hashlib
import math
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, TextIO

from modsim.runtime.session import RuntimeSession


def _f(value: Any) -> str:
    if isinstance(value, bool) or value is None:
        return repr(value)
    if isinstance(value, int):
        return str(value)
    number = float(value)
    return "nan" if math.isnan(number) else number.hex()


def _seq(values: Iterable[Any]) -> str:
    return "(" + ",".join(_f(v) for v in values) + ")"


def _pose(pose: Any) -> str:
    return _seq(pose.translation) + _seq(pose.rotation)


def _enum(value: Any) -> str:
    return getattr(value, "value", str(value))


def world_lines(session: RuntimeSession, *, include_events: bool) -> list[str]:
    """Every observable piece of kinematic and semantic world state."""
    world = session.world
    revision = world.revision
    lines = [
        f"time {_f(world.time_s)}",
        "revision "
        + " ".join(
            str(getattr(revision, name))
            for name in (
                "sample_sequence",
                "topology_revision",
                "docking_revision",
                "event_revision",
            )
        ),
    ]
    for module_id in sorted(world.modules):
        module = world.modules[module_id]
        lines.append(
            f"module {module_id} pose {_pose(module.pose)} "
            f"v {_seq(module.linear_velocity_m_s)} w {_seq(module.angular_velocity_rad_s)}"
        )
        links: Mapping[str, Any] = module.link_poses
        for link in sorted(links):
            lines.append(f"  link {link} {_pose(links[link])}")
        joints: Mapping[str, Any] = module.joint_states
        for joint in sorted(joints):
            state = joints[joint]
            lines.append(
                f"  joint {joint} {_f(state.position)} {_f(state.velocity)} {_f(state.effort)}"
            )
    for connector_id in sorted(world.connectors):
        c = world.connectors[connector_id]
        lines.append(
            f"connector {connector_id} {_enum(c.lifecycle_state)} conn={c.connection_id} "
            f"resolved={c.resolved} avail={_f(c.available_at_s)} pose {_pose(c.world_pose)} "
            f"dock {_seq(c.world_docking_axis)} appr {_seq(c.world_approach_axis)} "
            f"v {_seq(c.world_velocity_m_s)}"
        )
    for connection_id in sorted(world.connections):
        connection = world.connections[connection_id]
        lines.append(
            f"connection {connection_id} {connection.connector_a}<->{connection.connector_b}"
        )
    assemblies = world.assemblies
    members = sorted(
        tuple(sorted(assemblies.members(assemblies.assembly_of(m)))) for m in world.modules
    )
    lines.append("assemblies " + " | ".join(",".join(group) for group in dict.fromkeys(members)))
    events = world.event_log.events
    lines.append(f"events {len(events)}")
    if include_events:
        for event in events:
            lines.append(
                f"  event {_f(event.time_s)} {type(event).__name__} {_event_fields(event)}"
            )
    return lines


def _event_fields(event: Any) -> str:
    fields = []
    for name in sorted(
        getattr(event, "__dataclass_fields__", {}) or getattr(event, "model_fields", {})
    ):
        if name == "time_s":
            continue
        value = getattr(event, name, None)
        if isinstance(value, float):
            fields.append(f"{name}={_f(value)}")
        elif isinstance(value, tuple | list):
            fields.append(f"{name}={[str(v) for v in value]}")
        else:
            fields.append(f"{name}={_enum(value)}")
    return " ".join(fields)


class Tracer:
    """Append-only writer shared by every session in one process."""

    def __init__(self, path: Path, every: int) -> None:
        self.every = every
        self._out: TextIO = path.open("w")
        self._steps: dict[int, int] = {}
        self._sessions: dict[int, int] = {}
        self._last: dict[int, RuntimeSession] = {}
        self.records = 0
        atexit.register(self.close)

    def _tag(self, session: RuntimeSession) -> int:
        return self._sessions.setdefault(id(session), len(self._sessions))

    def write(self, session: RuntimeSession, label: str, *, include_events: bool = False) -> None:
        tag = self._tag(session)
        body = world_lines(session, include_events=include_events)
        digest = hashlib.sha256("\n".join(body).encode()).hexdigest()[:16]
        self._out.write(f"=== session {tag} {label} sha={digest}\n")
        self._out.write("\n".join(body) + "\n")
        self.records += 1

    def after_step(self, session: RuntimeSession) -> None:
        key = id(session)
        count = self._steps.get(key, 0) + 1
        self._steps[key] = count
        self._last[key] = session
        if self.every and count % self.every == 0:
            self.write(session, f"step {count}")

    def before_shutdown(self, session: RuntimeSession) -> None:
        self.write(session, f"final step {self._steps.get(id(session), 0)}", include_events=True)
        self._last.pop(id(session), None)

    def close(self) -> None:
        if not self._out.closed:
            self._out.close()


_TRACER: Tracer | None = None


def install(path: Path, every: int) -> Tracer:
    """Record every ``every``-th step (0 = final state only) of every session."""
    global _TRACER
    tracer = Tracer(path, every)
    _TRACER = tracer
    original_step = RuntimeSession.step
    original_shutdown = RuntimeSession.shutdown

    def step(self: RuntimeSession, *args: Any, **kwargs: Any) -> Any:
        result = original_step(self, *args, **kwargs)
        tracer.after_step(self)
        return result

    def shutdown(self: RuntimeSession) -> None:
        tracer.before_shutdown(self)
        original_shutdown(self)

    RuntimeSession.step = step  # type: ignore[method-assign]
    RuntimeSession.shutdown = shutdown  # type: ignore[method-assign]
    return tracer
