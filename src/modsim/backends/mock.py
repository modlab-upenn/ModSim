"""A dependency-free kinematic backend.

The mock backend exists so that docking semantics can be developed and tested
without a physics engine. It is deliberately *kinematic*, not dynamic: bodies
move at whatever twist they are given, nothing falls, and nothing collides.
What it does model faithfully is the part ModSim actually depends on — that a
physical connection makes two modules move as one rigid body, and that removing
it lets them move independently again.

Because the mock never moves a joint, every link of a module sits at a fixed
transform from the module root for the whole run. The mock resolves that
transform from the URDF joint origins at zero configuration, so it reports the
true world frame of a connector carried on a wheel or a tilt body rather than
collapsing every link onto the module root. This is what makes the mock a
genuine kinematic reference for articulated modules, not only single-link ones.
When no URDF is reachable it degrades to the root frame for every link, which is
exact for a single-link module and is the only case that pre-dates this
resolution.
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
    quat_from_rpy,
    vec_add,
    vec_cross,
    vec_norm,
    vec_scale,
    vec_sub,
)
from modsim.importers.urdf import URDFImportError, URDFImporter
from modsim.robot_packs.schema import AssetCatalogKind, ModuleType, RobotPack

MOCK_BACKEND_NAME = "mock"


def _resolve_link_transforms(
    pack: RobotPack,
    module_type: ModuleType,
    root: Path | None,
) -> dict[str, Transform]:
    """Return each link's transform from the module root at zero joint config.

    The mock has no articulated dynamics, so its joints never move and the fixed
    transform implied by the URDF joint origins is the correct pose of every
    child link for the mock's whole lifetime. When no URDF is reachable the map
    holds only the root link, and every other link falls back to the root frame,
    which is exact for a single-link module.
    """
    transforms: dict[str, Transform] = {module_type.root_link: Transform.identity()}
    if root is None:
        return transforms
    relative = pack.manifest.assets.mechanical_catalog(AssetCatalogKind.URDF).get(
        module_type.asset_ref
    )
    if relative is None:
        return transforms
    try:
        asset = URDFImporter().load(root / relative)
    except (URDFImportError, OSError):
        return transforms

    # Resolve links outward from the root: a child's transform is its parent's
    # composed with the joint origin. Repeated passes handle any joint order and
    # any tree depth without assuming the URDF lists joints parent-first.
    remaining = list(asset.joints)
    progressed = True
    while progressed:
        progressed = False
        for joint in remaining:
            if joint.child_link in transforms or joint.parent_link not in transforms:
                continue
            origin = Transform(
                translation=joint.origin_xyz_m,
                rotation=quat_from_rpy(joint.origin_rpy_rad),
            )
            transforms[joint.child_link] = transforms[joint.parent_link].compose(origin)
            progressed = True
    return transforms


@dataclass(slots=True)
class _Body:
    """One module's kinematic state in the mock world."""

    pose: Transform
    links: tuple[str, ...]
    module_type_id: str
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

    __slots__ = ("_bodies", "_forces", "_link_transforms", "_next_failure", "_time_s", "_welds")

    def __init__(self) -> None:
        self._bodies: dict[ModuleInstanceId, _Body] = {}
        self._welds: dict[ConstraintHandle, _Weld] = {}
        self._forces: dict[ConstraintHandle, float] = {}
        self._link_transforms: dict[str, dict[str, Transform]] = {}
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

        ``root`` is the Robot Pack directory. The mock reads no meshes, but it
        does read the URDF joint tree so it can place articulated child links;
        without ``root`` it falls back to the module root frame for every link.
        """
        links_by_module = scene.module_links(pack)
        self._link_transforms = {}
        bodies: dict[ModuleInstanceId, _Body] = {}
        for placement in scene.placements:
            module_type = pack.hardware_catalog.module_types[placement.module_type_id]
            if placement.module_type_id not in self._link_transforms:
                self._link_transforms[placement.module_type_id] = _resolve_link_transforms(
                    pack, module_type, root
                )
            bodies[placement.instance_id] = _Body(
                pose=placement.pose,
                links=links_by_module[placement.instance_id],
                module_type_id=placement.module_type_id,
            )
        self._bodies = bodies
        self._welds = {}
        self._forces = {}
        self._time_s = 0.0
        handles: dict[tuple[ModuleInstanceId, str], str] = {}
        for module_id, body in self._bodies.items():
            for link in body.links:
                handles[(module_id, link)] = f"{module_id}:{link}"
        return BackendHandleRegistry(bodies=handles)

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

        Every module is one rigid body: the mock never moves a joint, so each
        link sits at its fixed transform from the module root and inherits the
        module's twist through the rigid ``omega x r`` term. Reporting those
        per-link frames is what lets a connector on an articulated link resolve
        to the same place a real backend would put it.
        """
        link_states: dict[ModuleInstanceId, dict[str, BodyState]] = {}
        for module_id, body in self._bodies.items():
            transforms = self._link_transforms.get(body.module_type_id, {})
            root_origin = body.pose.translation
            states: dict[str, BodyState] = {}
            for link in body.links:
                local = transforms.get(link)
                world_pose = body.pose if local is None else body.pose.compose(local)
                lever = vec_sub(world_pose.translation, root_origin)
                linear = vec_add(
                    body.linear_velocity_m_s,
                    vec_cross(body.angular_velocity_rad_s, lever),
                )
                states[link] = BodyState(
                    pose=world_pose,
                    linear_velocity_m_s=linear,
                    angular_velocity_rad_s=body.angular_velocity_rad_s,
                )
            link_states[module_id] = states
        return BackendStateSnapshot(
            time_s=self._time_s,
            link_states=link_states,
            constraint_forces_n=dict(self._forces),
        )

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Weld two modules together, honouring an injected failure if armed."""
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
        self._link_transforms.clear()

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

    def _root_to_link(self, module_type_id: str, link: str) -> Transform:
        """Return one link's fixed transform from its module root."""
        return self._link_transforms.get(module_type_id, {}).get(link, Transform.identity())

    def _snap(self, request: ConnectionRequest) -> None:
        """Move module B so the requested connector-frame relative pose holds.

        Both connectors are expressed on their parent links, which may be
        articulated children rather than the module root, so the root-to-link
        transforms are folded in before solving for module B's root pose.
        """
        body_a = self._bodies[request.module_a]
        body_b = self._bodies[request.module_b]
        root_to_a = self._root_to_link(body_a.module_type_id, request.link_a)
        root_to_b = self._root_to_link(body_b.module_type_id, request.link_b)
        connector_a_world = body_a.pose.compose(root_to_a).compose(request.connector_a_local)
        target_connector_b = connector_a_world.compose(request.relative_transform)
        offset_b = root_to_b.compose(request.connector_b_local)
        body_b.pose = target_connector_b.compose(offset_b.inverse())

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
