"""Driving point connectors over the surfaces they are attached to.

A ``driven_fixed`` point-to-surface connection is held by a fixed constraint
whose relative pose the runtime retargets each step. The commanded motion is a
backend-neutral :class:`SurfaceSlideCommand`: a tangential velocity in the
attachment frame plus a spin rate about the outward normal. The
:class:`SurfaceSlideDriver` integrates every command on the sphere, enforces
the surface's minimum separation (docs/continuous_docking.md amendment A2), and
sends one batched retarget per step.

Slide commands are controller intent, like joint commands: they are never
written to the event log. The live attachment remains measured state
(``WorldState.surface_attachment``).
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass, field

from modsim.backends.base import (
    BackendAdapter,
    BackendError,
    ConnectionRetarget,
    SupportsConnectionRetarget,
)
from modsim.core.ids import ConnectionId
from modsim.core.state import WorldState
from modsim.core.surface import central_angle
from modsim.core.transforms import (
    Transform,
    Vec3,
    quat_from_axis_angle,
    vec_add,
    vec_cross,
    vec_norm,
    vec_scale,
    vec_sub,
)
from modsim.robot_packs.schema import SurfaceDriveSpec, SurfaceRealization

_BISECTION_STEPS = 60
_MIN_MOTION = 1e-15


class SurfaceSlideError(ValueError):
    """Raised when a slide command is invalid or its connection cannot slide."""


@dataclass(frozen=True, slots=True)
class SurfaceSlideCommand:
    """Backend-neutral sliding motion for one point-to-surface connection.

    ``tangential_velocity_m_s`` is the velocity of the attachment point along
    the attachment frame's x and y axes (both tangent to the surface).
    ``spin_rate_rad_s`` is the rate of rotation about the outward normal.
    """

    tangential_velocity_m_s: tuple[float, float] = (0.0, 0.0)
    spin_rate_rad_s: float = 0.0


@dataclass(frozen=True, slots=True)
class SlideClamp:
    """A commanded slide that was shortened to respect a surface's minimum separation."""

    connection_id: ConnectionId
    neighbour: ConnectionId
    """The connection whose attachment the move would have come too close to."""
    allowed_fraction: float
    """How much of the commanded translation was applied, from 0 to 1."""


@dataclass(frozen=True, slots=True)
class SlideRefusal:
    """A commanded slide the backend refused; the command has been dropped."""

    connection_id: ConnectionId
    detail: str


@dataclass(frozen=True, slots=True)
class SlideReport:
    """What the last slide update did besides moving attachments as commanded."""

    clamps: tuple[SlideClamp, ...] = ()
    refusals: tuple[SlideRefusal, ...] = ()


def differential_slide_command(
    drive: SurfaceDriveSpec,
    left_rate_rad_s: float,
    right_rate_rad_s: float,
) -> SurfaceSlideCommand:
    """Return the slide produced by two wheel rates under a differential drive.

    Forward speed is the mean rim speed along the attachment frame's +x, and
    the spin rate about the outward normal is the rim-speed difference over
    the track, positive when the right wheel is faster. Phase 1 provides the
    function only; nothing feeds it measured wheel rates yet.
    """
    left = drive.wheel_radius_m * left_rate_rad_s
    right = drive.wheel_radius_m * right_rate_rad_s
    return SurfaceSlideCommand(
        tangential_velocity_m_s=((left + right) / 2.0, 0.0),
        spin_rate_rad_s=(right - left) / drive.track_m,
    )


def _finite(*values: object) -> bool:
    return all(
        not isinstance(value, bool) and isinstance(value, int | float) and math.isfinite(value)
        for value in values
    )


def _rotate_about_center(frame: Transform, center: Vec3, axis: Vec3, angle: float) -> Transform:
    rotation = Transform(rotation=quat_from_axis_angle(axis, angle))
    to_center = Transform(translation=center)
    return to_center.compose(rotation).compose(to_center.inverse()).compose(frame)


@dataclass(slots=True)
class _Target:
    frame: Transform
    """Commanded attachment frame A_q in the surface connector frame."""


@dataclass(slots=True)
class SurfaceSlideDriver:
    """Integrates slide commands and retargets ``driven_fixed`` connections."""

    commands: dict[ConnectionId, SurfaceSlideCommand] = field(
        default_factory=dict[ConnectionId, SurfaceSlideCommand]
    )
    _targets: dict[ConnectionId, _Target] = field(default_factory=dict[ConnectionId, _Target])
    last_report: SlideReport = field(default_factory=SlideReport)

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------

    def set(
        self,
        world: WorldState,
        adapter: BackendAdapter,
        connection: ConnectionId,
        command: SurfaceSlideCommand,
    ) -> None:
        """Validate and store one persistent slide command."""
        runtime = world.connections.get(connection)
        if runtime is None:
            raise SurfaceSlideError(f"no active connection '{connection}'")
        if runtime.realization is None:
            raise SurfaceSlideError(f"connection '{connection}' is not a surface connection")
        if runtime.realization is not SurfaceRealization.DRIVEN_FIXED:
            raise SurfaceSlideError(
                f"connection '{connection}' is realised as '{runtime.realization.value}', "
                "which cannot be driven; only driven_fixed connections slide on command"
            )
        velocity = command.tangential_velocity_m_s
        if len(velocity) != 2 or not _finite(*velocity, command.spin_rate_rad_s):
            raise SurfaceSlideError(
                f"slide command for '{connection}' must have two finite tangential velocity "
                "components and a finite spin rate"
            )
        capabilities = adapter.capabilities()
        if not capabilities.supports_connection_retarget or not isinstance(
            adapter, SupportsConnectionRetarget
        ):
            raise SurfaceSlideError(
                f"backend '{capabilities.name}' cannot retarget connections, so "
                f"'{connection}' cannot slide"
            )
        if connection not in self._targets:
            attachment = world.surface_attachment(connection)
            if attachment is None:
                raise SurfaceSlideError(
                    f"connection '{connection}' has no measured attachment in this sample"
                )
            self._targets[connection] = _Target(frame=attachment.frame_local)
        self.commands[connection] = command

    def clear(self, connections: Iterable[ConnectionId] | None = None) -> None:
        """Stop sliding the given connections, or every connection."""
        selected = tuple(self.commands) if connections is None else tuple(connections)
        for connection in selected:
            self.commands.pop(connection, None)
            self._targets.pop(connection, None)

    # ------------------------------------------------------------------
    # per-step update
    # ------------------------------------------------------------------

    def update(self, world: WorldState, adapter: BackendAdapter, dt_s: float) -> SlideReport:
        """Integrate every command over ``dt_s`` and send one retarget batch.

        Commands are processed in sorted connection-id order. A target that
        would come inside a surface's minimum separation is clamped to the
        boundary along its commanded direction; an attachment already inside
        the boundary may only move away. Connections that no longer exist are
        forgotten.
        """
        for connection in [c for c in self.commands if c not in world.connections]:
            self.clear((connection,))
        if not self.commands or dt_s <= 0.0:
            self.last_report = SlideReport()
            return self.last_report
        if not isinstance(adapter, SupportsConnectionRetarget):
            raise BackendError("slide commands are set but the backend cannot retarget")

        clamps: list[SlideClamp] = []
        planned: dict[ConnectionId, Transform] = {}
        for connection in sorted(self.commands):
            frame, clamp = self._advance(world, connection, dt_s, planned)
            planned[connection] = frame
            if clamp is not None:
                clamps.append(clamp)

        refusals = self._send(world, adapter, planned)
        for connection, frame in planned.items():
            if connection in self._targets:
                self._targets[connection].frame = frame
        self.last_report = SlideReport(clamps=tuple(clamps), refusals=refusals)
        return self.last_report

    def _advance(
        self,
        world: WorldState,
        connection: ConnectionId,
        dt_s: float,
        planned: dict[ConnectionId, Transform],
    ) -> tuple[Transform, SlideClamp | None]:
        command = self.commands[connection]
        runtime = world.connections[connection]
        spec = world.connector_type(runtime.connector_b).surface
        if spec is None:  # pragma: no cover - realisation implies a surface
            raise SurfaceSlideError(f"connection '{connection}' has no surface")
        current = self._targets[connection].frame
        center = spec.center_m
        normal = _normal(current, center)
        vx, vy = command.tangential_velocity_m_s
        tangent = vec_add(
            vec_scale(current.apply_direction((1.0, 0.0, 0.0)), vx),
            vec_scale(current.apply_direction((0.0, 1.0, 0.0)), vy),
        )
        speed = vec_norm(tangent)
        moved = current
        clamp: SlideClamp | None = None
        if speed * dt_s > _MIN_MOTION:
            axis = vec_scale(vec_cross(normal, tangent), 1.0 / speed)
            full_angle = speed * dt_s / spec.radius_m
            fraction, neighbour = self._allowed_fraction(
                world, connection, current, center, axis, full_angle, planned
            )
            moved = _rotate_about_center(current, center, axis, full_angle * fraction)
            if neighbour is not None:
                clamp = SlideClamp(connection, neighbour, fraction)
        if command.spin_rate_rad_s:
            spin = Transform(
                rotation=quat_from_axis_angle((0.0, 0.0, 1.0), command.spin_rate_rad_s * dt_s)
            )
            moved = moved.compose(spin)
        return moved, clamp

    def _allowed_fraction(
        self,
        world: WorldState,
        connection: ConnectionId,
        current: Transform,
        center: Vec3,
        axis: Vec3,
        full_angle: float,
        planned: dict[ConnectionId, Transform],
    ) -> tuple[float, ConnectionId | None]:
        runtime = world.connections[connection]
        surface = world.connector(runtime.connector_b)
        spec = world.connector_type(surface.id).surface
        if spec is None or spec.min_separation_rad <= 0.0:
            return 1.0, None
        separation = spec.min_separation_rad

        def normal_at(fraction: float) -> Vec3:
            frame = _rotate_about_center(current, center, axis, full_angle * fraction)
            return _normal(frame, center)

        allowed = 1.0
        blocker: ConnectionId | None = None
        for other in surface.connection_ids:
            if other == connection:
                continue
            neighbour = self._neighbour_normal(world, other, center, planned)
            if neighbour is None:
                continue
            start = central_angle(normal_at(0.0), neighbour)
            end = central_angle(normal_at(1.0), neighbour)
            if end >= separation:
                continue
            if start < separation:
                # Already inside the boundary: only moves that increase the
                # separation are allowed.
                if end < start:
                    allowed, blocker = 0.0, other
                continue
            # Bisect for the boundary along the commanded arc, keeping the
            # side that is still outside it.
            low, high = 0.0, 1.0
            for _ in range(_BISECTION_STEPS):
                middle = (low + high) / 2.0
                if central_angle(normal_at(middle), neighbour) >= separation:
                    low = middle
                else:
                    high = middle
            if low < allowed:
                allowed, blocker = low, other
        return allowed, blocker

    def _neighbour_normal(
        self,
        world: WorldState,
        other: ConnectionId,
        center: Vec3,
        planned: dict[ConnectionId, Transform],
    ) -> Vec3 | None:
        """Return another attachment's normal: its new target if already moved, else measured."""
        if other in planned:
            return _normal(planned[other], center)
        attachment = world.surface_attachment(other)
        return None if attachment is None else attachment.normal_local

    def _send(
        self,
        world: WorldState,
        adapter: SupportsConnectionRetarget,
        planned: dict[ConnectionId, Transform],
    ) -> tuple[SlideRefusal, ...]:
        updates: list[tuple[ConnectionId, ConnectionRetarget]] = []
        for connection, frame in planned.items():
            runtime = world.connections[connection]
            point = world.connector(runtime.connector_a)
            surface = world.connector(runtime.connector_b)
            updates.append(
                (
                    connection,
                    ConnectionRetarget(
                        handle=runtime.constraint_handle,
                        connector_a_local=point.local_pose,
                        connector_b_local=surface.local_pose.compose(frame),
                        relative_transform=runtime.relative_transform,
                    ),
                )
            )
        refusals: list[SlideRefusal] = []
        while updates:
            flags = adapter.retarget_physical_connections(tuple(u for _, u in updates))
            if len(flags) != len(updates):
                raise BackendError("retarget returned a different number of results")
            if all(flags):
                break
            # Nothing was applied. Drop the refused updates and resend the rest.
            kept: list[tuple[ConnectionId, ConnectionRetarget]] = []
            for (connection, update), accepted in zip(updates, flags, strict=True):
                if accepted:
                    kept.append((connection, update))
                    continue
                refusals.append(SlideRefusal(connection, "backend refused the retarget"))
                self.clear((connection,))
                planned.pop(connection, None)
            updates = kept
        return tuple(refusals)


def _normal(frame: Transform, center: Vec3) -> Vec3:
    offset = vec_sub(frame.translation, center)
    return vec_scale(offset, 1.0 / vec_norm(offset))
