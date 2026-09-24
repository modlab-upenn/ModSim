"""A dependency-free kinematic backend.

The mock backend exists so that docking semantics can be developed and tested
without a physics engine. It is deliberately *kinematic*, not dynamic: bodies
move at whatever twist they are given, nothing falls, and nothing collides.
What it does model faithfully is the part ModSim actually depends on — that a
physical connection makes two modules move as one rigid body, and that removing
it lets them move independently again.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from modsim.backends.base import (
    BackendCapabilities,
    BackendError,
    BackendHandleRegistry,
    ConnectionOutcome,
    ConnectionRequest,
)
from modsim.core.ids import ConstraintHandle, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, BodyState
from modsim.core.transforms import (
    ZERO_VEC3,
    Transform,
    Vec3,
    quat_from_axis_angle,
    vec_norm,
    vec_scale,
)
from modsim.robot_packs.schema import PhysicalConstraintType, RobotPack

MOCK_BACKEND_NAME = "mock"


@dataclass(slots=True)
class _Body:
    """One module's kinematic state in the mock world."""

    pose: Transform
    links: tuple[str, ...]
    linear_velocity_m_s: Vec3 = ZERO_VEC3
    angular_velocity_rad_s: Vec3 = ZERO_VEC3


@dataclass(slots=True)
class _Weld:
    """One realised rigid connection between two modules."""

    handle: ConstraintHandle
    module_a: ModuleInstanceId
    module_b: ModuleInstanceId
    relative_pose: Transform
    """Pose of module B expressed in module A's frame at commit time."""


class MockBackendAdapter:
    """Kinematic backend used for tests, CI, and semantics development."""

    __slots__ = ("_bodies", "_forces", "_next_failure", "_time_s", "_welds")

    def __init__(self) -> None:
        self._bodies: dict[ModuleInstanceId, _Body] = {}
        self._welds: dict[ConstraintHandle, _Weld] = {}
        self._forces: dict[ConstraintHandle, float] = {}
        self._time_s = 0.0
        self._next_failure: str | None = None

    # ------------------------------------------------------------------
    # BackendAdapter protocol
    # ------------------------------------------------------------------

    def capabilities(self) -> BackendCapabilities:
        """Return the mock backend's capability set."""
        return BackendCapabilities(
            name=MOCK_BACKEND_NAME,
            supports_runtime_constraints=True,
            supports_constraint_removal=True,
            supports_constraint_forces=True,
            supports_module_pose_write=True,
        )

    def load(
        self,
        pack: RobotPack,
        scene: SceneSpec,
        *,
        root: Path | None = None,
    ) -> BackendHandleRegistry:
        """Instantiate every placement as one rigid body per module.

        ``root`` is ignored: the mock reads no mechanical assets, which is
        exactly why it can run from an in-memory Robot Pack.
        """
        del root
        links_by_module = scene.module_links(pack)
        self._bodies = {
            placement.instance_id: _Body(
                pose=placement.pose,
                links=links_by_module[placement.instance_id],
            )
            for placement in scene.placements
        }
        self._welds = {}
        self._forces = {}
        self._time_s = 0.0
        bodies: dict[tuple[ModuleInstanceId, str], str] = {}
        for module_id, body in self._bodies.items():
            for link in body.links:
                bodies[(module_id, link)] = f"{module_id}:{link}"
        return BackendHandleRegistry(bodies=bodies)

    def step(self, dt_s: float) -> None:
        """Advance every welded group rigidly by its representative's twist."""
        if dt_s < 0.0:
            raise BackendError("cannot step backwards")
        for members in self._groups():
            representative = min(members)
            body = self._bodies[representative]
            delta = self._integrate(body, dt_s)
            for module_id in members:
                member = self._bodies[module_id]
                member.pose = delta.compose(member.pose)
                if module_id != representative:
                    member.linear_velocity_m_s = body.linear_velocity_m_s
                    member.angular_velocity_rad_s = body.angular_velocity_rad_s
        self._time_s += dt_s

    def snapshot(self) -> BackendStateSnapshot:
        """Return the current kinematic state.

        Every link of a module reports the module pose, because the mock has no
        articulated kinematics. Modules with internal joints therefore need a
        real backend before their non-root connectors mean anything.
        """
        link_states: dict[ModuleInstanceId, dict[str, BodyState]] = {}
        for module_id, body in self._bodies.items():
            state = BodyState(
                pose=body.pose,
                linear_velocity_m_s=body.linear_velocity_m_s,
                angular_velocity_rad_s=body.angular_velocity_rad_s,
            )
            link_states[module_id] = dict.fromkeys(body.links, state)
        return BackendStateSnapshot(
            time_s=self._time_s,
            link_states=link_states,
            constraint_forces_n=dict(self._forces),
        )

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Weld two modules together, honouring an injected failure if armed."""
        if request.physical_connection.constraint is PhysicalConstraintType.HINGE:
            return ConnectionOutcome.refused(
                "mock backend does not implement hinge constraints; use a physics backend"
            )
        if self._next_failure is not None:
            detail, self._next_failure = self._next_failure, None
            return ConnectionOutcome.refused(detail)
        for module_id in (request.module_a, request.module_b):
            if module_id not in self._bodies:
                return ConnectionOutcome.refused(f"unknown module '{module_id}'")

        handle = ConstraintHandle(f"weld:{request.connection_id}")
        if handle in self._welds:
            return ConnectionOutcome.refused(f"constraint '{handle}' already exists")

        # A nominal-alignment request asks the backend to move module B so the
        # connector frames coincide exactly, which is the whole point of the
        # nominal mode. A measured request freezes whatever pose exists now.
        if request.snap_to_nominal:
            self._snap(request)
        body_a, body_b = self._bodies[request.module_a], self._bodies[request.module_b]
        self._welds[handle] = _Weld(
            handle=handle,
            module_a=request.module_a,
            module_b=request.module_b,
            relative_pose=body_b.pose.relative_to(body_a.pose),
        )
        self._forces[handle] = 0.0
        return ConnectionOutcome.accepted(handle)

    def remove_physical_connection(self, handle: ConstraintHandle) -> bool:
        """Remove one weld, returning whether it existed."""
        self._forces.pop(handle, None)
        return self._welds.pop(handle, None) is not None

    def shutdown(self) -> None:
        """Clear all mock state."""
        self._bodies.clear()
        self._welds.clear()
        self._forces.clear()

    # ------------------------------------------------------------------
    # test controls
    # ------------------------------------------------------------------

    def set_module_pose(self, module_id: ModuleInstanceId, pose: Transform) -> None:
        """Teleport one module and clear its motion."""
        body = self._body(module_id)
        body.pose = pose
        body.linear_velocity_m_s = ZERO_VEC3
        body.angular_velocity_rad_s = ZERO_VEC3

    def set_module_twist(
        self,
        module_id: ModuleInstanceId,
        *,
        linear_m_s: Vec3 = ZERO_VEC3,
        angular_rad_s: Vec3 = ZERO_VEC3,
    ) -> None:
        """Set one module's linear and angular velocity."""
        body = self._body(module_id)
        body.linear_velocity_m_s = linear_m_s
        body.angular_velocity_rad_s = angular_rad_s

    def fail_next_connection(self, detail: str = "injected backend failure") -> None:
        """Make the next connection request fail, to exercise the failure path."""
        self._next_failure = detail

    def set_constraint_force(self, handle: ConstraintHandle, force_n: float) -> None:
        """Set the reported constraint force for one weld."""
        if handle not in self._welds:
            raise BackendError(f"unknown constraint '{handle}'")
        self._forces[handle] = force_n

    @property
    def welds(self) -> tuple[ConstraintHandle, ...]:
        """Return every active weld handle."""
        return tuple(sorted(self._welds))

    @property
    def time_s(self) -> float:
        """Return the mock simulation time."""
        return self._time_s

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------

    def _body(self, module_id: ModuleInstanceId) -> _Body:
        try:
            return self._bodies[module_id]
        except KeyError as error:
            raise BackendError(f"unknown module '{module_id}'") from error

    def _snap(self, request: ConnectionRequest) -> None:
        """Move module B so the requested connector-frame relative pose holds."""
        body_a, body_b = self._bodies[request.module_a], self._bodies[request.module_b]
        connector_a_world = body_a.pose.compose(request.connector_a_local)
        target_connector_b = connector_a_world.compose(request.relative_transform)
        body_b.pose = target_connector_b.compose(request.connector_b_local.inverse())

    @staticmethod
    def _integrate(body: _Body, dt_s: float) -> Transform:
        """Return the world-frame delta produced by one step of a body's twist."""
        translation = vec_scale(body.linear_velocity_m_s, dt_s)
        speed = vec_norm(body.angular_velocity_rad_s)
        if speed < 1e-12:
            return Transform(translation=translation)
        rotation = quat_from_axis_angle(body.angular_velocity_rad_s, speed * dt_s)
        pivot = body.pose.translation
        # Rotate about the body's own origin, then translate.
        about_origin = Transform(rotation=rotation)
        to_pivot = Transform(translation=pivot)
        return Transform(translation=translation).compose(
            to_pivot.compose(about_origin).compose(to_pivot.inverse())
        )

    def _groups(self) -> tuple[frozenset[ModuleInstanceId], ...]:
        """Return welded module groups as connected components."""
        neighbours: dict[ModuleInstanceId, set[ModuleInstanceId]] = {
            module_id: set() for module_id in self._bodies
        }
        for weld in self._welds.values():
            if weld.module_a == weld.module_b:
                continue
            neighbours[weld.module_a].add(weld.module_b)
            neighbours[weld.module_b].add(weld.module_a)
        groups: list[frozenset[ModuleInstanceId]] = []
        remaining = set(self._bodies)
        while remaining:
            seed = min(remaining)
            component = {seed}
            queue = [seed]
            while queue:
                current = queue.pop()
                for neighbour in neighbours[current]:
                    if neighbour not in component:
                        component.add(neighbour)
                        queue.append(neighbour)
            remaining -= component
            groups.append(frozenset(component))
        return tuple(sorted(groups, key=min))
