"""Compile a ModSim scene into a single MuJoCo model.

A Robot Pack describes one module type per mechanical asset. A ModSim scene
places many instances of those types in one world. MuJoCo needs all of them in
one compiled model, so this module composes per-instance copies of each asset
into a single ``MjSpec`` and compiles it.

Names are prefixed per module instance, which is what lets the adapter map
ModSim identifiers onto MuJoCo ids without guessing.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import mujoco

from modsim.backends.base import BackendError, BackendHandleRegistry
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, connector_instance_id
from modsim.core.scene import SceneSpec
from modsim.core.transforms import Transform, Vec3
from modsim.robot_packs.schema import (
    AssetCatalogKind,
    ControlMode,
    JointSpec,
    JointType,
    ModuleType,
    RobotPack,
)

INSTANCE_SEPARATOR = "/"
CONNECTOR_NAMESPACE = "connector"
ACTUATOR_NAMESPACE = "actuator"
WORLD_BODY = "world"
GROUND_GEOM = "modsim_ground"
GROUND_HALF_EXTENT_M = 10.0
GROUND_THICKNESS_M = 0.05
ENVIRONMENT_GEOM_GROUP = 2
URDF_COLLISION_GEOM_GROUP = 3
DEFAULT_GRAVITY: Vec3 = (0.0, 0.0, -9.81)
MIN_WELD_POOL = 8
MIN_HINGE_POOL = 8
GROUND_CONTACT_CLASS_SUFFIX = "_ground_contact"
TIRE_GROUND_CONTACT_CLASS = "tire_ground_contact"


class MuJoCoSceneError(BackendError):
    """Raised when a Robot Pack and scene cannot be compiled into a MuJoCo model."""


def body_name(module_id: ModuleInstanceId, link: str) -> str:
    """Return the compiled body name for one module's link."""
    return f"{module_id}{INSTANCE_SEPARATOR}{link}"


def site_name(module_id: ModuleInstanceId, connector_id: str) -> str:
    """Return the compiled site name for one module's connector.

    Connector sites live in their own namespace so that a connector may share a
    name with a link without colliding in MuJoCo's flat name table.
    """
    return f"{module_id}{INSTANCE_SEPARATOR}{CONNECTOR_NAMESPACE}{INSTANCE_SEPARATOR}{connector_id}"


def joint_name(module_id: ModuleInstanceId, source_joint_name: str) -> str:
    """Return the compiled joint name for one source mechanical joint."""
    return f"{module_id}{INSTANCE_SEPARATOR}{source_joint_name}"


def actuator_name(module_id: ModuleInstanceId, joint_id: str) -> str:
    """Return the compiled effort-actuator name for one semantic joint."""
    return f"{module_id}{INSTANCE_SEPARATOR}{ACTUATOR_NAMESPACE}{INSTANCE_SEPARATOR}{joint_id}"


@dataclass(frozen=True, slots=True)
class CompiledScene:
    """A compiled MuJoCo model plus the ModSim identifier maps it needs."""

    model: mujoco.MjModel
    handles: BackendHandleRegistry
    body_ids: dict[tuple[ModuleInstanceId, str], int] = field(
        default_factory=dict[tuple[ModuleInstanceId, str], int]
    )
    site_ids: dict[ConnectorInstanceId, int] = field(default_factory=dict[ConnectorInstanceId, int])
    joint_ids: dict[tuple[ModuleInstanceId, str], int] = field(
        default_factory=dict[tuple[ModuleInstanceId, str], int]
    )
    actuator_ids: dict[tuple[ModuleInstanceId, str], int] = field(
        default_factory=dict[tuple[ModuleInstanceId, str], int]
    )
    weld_pool: tuple[int, ...] = ()
    """Ids of pre-allocated, inactive weld equality constraints.

    MuJoCo fixes model topology at compile time, so runtime docking cannot
    create constraints; it can only claim one of these. The pool is reserved
    here so docking can activate a slot without recompiling the model.
    """
    hinge_pool: tuple[tuple[int, int], ...] = ()
    """Pairs of inactive connect equalities reserved for runtime hinges."""
    contact_exclusion_pool: tuple[int, ...] = ()
    """Ids reserved to suppress collision between each welded body pair."""
    static_contact_exclusions: tuple[int, ...] = ()
    """Authored exclusion signatures retained beside the runtime pool."""
    anisotropic_ground_geoms: tuple[int, ...] = ()
    """Tire geoms whose first contact tangent follows their local cylinder axis."""


def _asset_path(
    pack: RobotPack,
    root: Path,
    module_type: ModuleType,
) -> tuple[AssetCatalogKind, Path]:
    """Return the mechanical asset backing one module type.

    A pack that ships a MuJoCo asset for the module's ``asset_ref`` is preferred,
    since a hand-authored MJCF carries solver settings a URDF cannot express.
    Otherwise MuJoCo parses the URDF directly.
    """
    assets = pack.manifest.assets
    for catalog in (AssetCatalogKind.MUJOCO, AssetCatalogKind.URDF):
        relative = assets.mechanical_catalog(catalog).get(module_type.asset_ref)
        if relative is not None:
            path = root / relative
            if not path.is_file():
                raise MuJoCoSceneError(f"declared asset is missing on disk: {path}")
            return catalog, path
    raise MuJoCoSceneError(
        f"module type '{module_type.id}' references asset '{module_type.asset_ref}', "
        "which is not declared in the urdf or mujoco catalog"
    )


def _module_spec(path: Path, catalog: AssetCatalogKind) -> mujoco.MjSpec:
    """Parse one mechanical asset into a fresh spec.

    A spec is re-parsed for every placement rather than reused, because
    ``attach`` mutates its argument and reusing one spec stacks prefixes onto a
    single body instead of producing independent copies.

    MuJoCo's URDF compiler discards ``<visual>`` geometry unless
    ``discardvisual`` is explicitly disabled. Robot Packs distinguish visual
    and collision geometry, so ModSim opts into retaining visuals when the URDF
    does not state a preference. Collision-only geoms are moved to group 3,
    MuJoCo's hidden-by-default debug group, without changing their contact
    masks. A pack can therefore use lightweight collision proxies while the
    viewer draws the detailed visual mesh.
    """
    try:
        if catalog is AssetCatalogKind.URDF:
            spec = _urdf_spec(path)
            _group_urdf_geometry(spec)
            return spec
        return mujoco.MjSpec.from_file(str(path))
    except (ET.ParseError, OSError, ValueError, RuntimeError) as error:
        raise MuJoCoSceneError(f"MuJoCo could not parse '{path}': {error}") from error


def _urdf_spec(path: Path) -> mujoco.MjSpec:
    """Parse a URDF while retaining its authored visual geometry by default."""
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"expected a URDF <robot> root, found <{root.tag}>")

    mujoco_options = root.find("mujoco")
    if mujoco_options is None:
        mujoco_options = ET.SubElement(root, "mujoco")
    compiler = mujoco_options.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_options, "compiler")

    # Respect an explicit author choice, but avoid MuJoCo's surprising URDF
    # default of throwing visual geometry away.
    if compiler.get("discardvisual") is None:
        compiler.set("discardvisual", "false")

    # Parsing from a string loses the source filename used to resolve relative
    # mesh paths. Re-establish that base without modifying the Robot Pack URDF.
    mesh_directory = compiler.get("meshdir")
    if mesh_directory is None:
        resolved_mesh_directory = path.parent
    else:
        configured = Path(mesh_directory)
        resolved_mesh_directory = (
            configured if configured.is_absolute() else path.parent / configured
        )
    compiler.set("meshdir", str(resolved_mesh_directory.resolve()))

    return mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))


def _group_urdf_geometry(spec: mujoco.MjSpec) -> None:
    """Hide URDF collision proxies by default when separate visuals exist."""
    has_visual_geometry = any(geom.contype == 0 and geom.conaffinity == 0 for geom in spec.geoms)
    if not has_visual_geometry:
        # A collision-only URDF must remain visible instead of disappearing.
        return
    for geom in spec.geoms:
        if geom.contype != 0 or geom.conaffinity != 0:
            geom.group = URDF_COLLISION_GEOM_GROUP


def build_scene(
    pack: RobotPack,
    scene: SceneSpec,
    root: Path,
    *,
    gravity: Vec3 = DEFAULT_GRAVITY,
    timestep_s: float | None = None,
    weld_pool_size: int | None = None,
    hinge_pool_size: int | None = None,
    ground: bool = False,
    ground_height_m: float = 0.0,
) -> CompiledScene:
    """Compose and compile every placement into one MuJoCo model.

    ``ground`` is off by default because a scene positions modules explicitly
    and a floor at the origin would intersect any module placed there. Enable it
    for scenes that need something to rest on, and place modules above
    ``ground_height_m``.
    """
    scene.validate_against(pack)
    spec = mujoco.MjSpec()
    spec.modelname = f"modsim_{pack.id}"
    spec.option.gravity = list(gravity)
    if timestep_s is not None:
        if timestep_s <= 0.0:
            raise MuJoCoSceneError("timestep must be positive")
        spec.option.timestep = timestep_s
    if ground:
        _add_ground(spec, ground_height_m)

    module_types: dict[ModuleInstanceId, ModuleType] = {}
    for placement in scene.placements:
        module_type = pack.hardware_catalog.module_types[placement.module_type_id]
        module_types[placement.instance_id] = module_type
        asset_catalog, asset = _asset_path(pack, root, module_type)
        frame = spec.worldbody.add_frame(
            pos=list(placement.pose.translation),
            quat=list(placement.pose.rotation),
        )
        spec.attach(
            _module_spec(asset, asset_catalog),
            prefix=f"{placement.instance_id}{INSTANCE_SEPARATOR}",
            frame=frame,
        )

    _add_freejoints(spec, module_types)
    _add_effort_actuators(spec, module_types)
    _add_connector_sites(spec, module_types)
    anisotropic_ground_geom_names = _add_ground_contact_pairs(spec) if ground else ()
    pool_names = _reserve_weld_pool(spec, module_types, weld_pool_size)
    hinge_pool_names = _reserve_hinge_pool(spec, module_types, hinge_pool_size)
    exclusion_pool_names = _reserve_contact_exclusion_pool(
        spec,
        module_types,
        len(pool_names),
    )

    try:
        model = spec.compile()
    except (ValueError, RuntimeError) as error:
        raise MuJoCoSceneError(f"MuJoCo failed to compile the scene: {error}") from error

    return _index(
        model,
        pack,
        scene,
        module_types,
        pool_names,
        hinge_pool_names,
        exclusion_pool_names,
        anisotropic_ground_geom_names,
    )


def _add_ground(spec: mujoco.MjSpec, height_m: float) -> None:
    """Add an infinite ground plane so modules have something to rest on."""
    geom = spec.worldbody.add_geom()
    geom.name = GROUND_GEOM
    geom.type = mujoco.mjtGeom.mjGEOM_PLANE
    geom.size = [GROUND_HALF_EXTENT_M, GROUND_HALF_EXTENT_M, GROUND_THICKNESS_M]
    geom.pos = [0.0, 0.0, height_m]
    geom.group = ENVIRONMENT_GEOM_GROUP


def _add_ground_contact_pairs(spec: mujoco.MjSpec) -> tuple[str, ...]:
    """Instantiate pack-authored ground-contact defaults per module geom.

    An MJCF module cannot name the composed world's shared ground geom. Assets
    opt into scene-level pairs by placing a geom in a default class whose name
    ends in ``_ground_contact`` and defining the corresponding ``<pair>``
    defaults there. This keeps friction coefficients in the mechanical asset
    while the composer supplies the per-instance, namespaced geom operands.

    Tire pairs use distinct lateral and rolling coefficients. MuJoCo expresses
    those along the contact frame's first two tangents, so their geom ids are
    also returned for the adapter to align tangent one with the tire axle.
    """
    anisotropic: list[str] = []
    pair_index = 0
    for geom in tuple(spec.geoms):
        class_name = geom.classname.name.rsplit(INSTANCE_SEPARATOR, 1)[-1]
        if not class_name.endswith(GROUND_CONTACT_CLASS_SUFFIX):
            continue
        spec.add_pair(
            default=geom.classname,
            name=f"modsim_ground_pair_{pair_index}",
            geomname1=GROUND_GEOM,
            geomname2=geom.name,
        )
        pair_index += 1
        if class_name == TIRE_GROUND_CONTACT_CLASS:
            anisotropic.append(geom.name)
    return tuple(anisotropic)


def _add_freejoints(spec: mujoco.MjSpec, module_types: dict[ModuleInstanceId, ModuleType]) -> None:
    """Give every module's root link six degrees of freedom.

    A URDF root link attaches rigidly to the world. Modules must be able to move
    and, once docked, be held together by a constraint rather than by the model
    tree, so each gets a free joint.
    """
    for module_id, module_type in module_types.items():
        name = body_name(module_id, module_type.root_link)
        body = spec.body(name)
        if body is None:
            raise MuJoCoSceneError(f"compiled scene has no body named '{name}'")
        body.add_freejoint()


def _add_effort_actuators(
    spec: mujoco.MjSpec,
    module_types: dict[ModuleInstanceId, ModuleType],
) -> None:
    """Add a direct motor for every joint that advertises effort control.

    The Robot Pack is the semantic source of truth for which joints are
    controllable and for their command bounds. Mechanical MJCF files therefore
    remain reusable assets rather than silently defining a second actuator API.
    The motor has unit gear, so its control input and output are both expressed
    in the joint's declared SI effort unit (N m or N).
    """
    for module_id, module_type in module_types.items():
        for joint in module_type.joints:
            if ControlMode.EFFORT not in joint.control_modes:
                continue
            target = joint_name(module_id, joint.source_joint_name)
            if spec.joint(target) is None:
                raise MuJoCoSceneError(
                    f"joint '{joint.id}' on module type '{module_type.id}' maps to "
                    f"source joint '{joint.source_joint_name}', but the selected "
                    "MuJoCo asset does not define it"
                )
            maximum = _maximum_effort(joint)
            actuator = spec.add_actuator(name=actuator_name(module_id, joint.id))
            actuator.set_to_motor()
            actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
            actuator.target = target
            actuator.ctrllimited = True
            actuator.ctrlrange = [-maximum, maximum]
            actuator.forcelimited = True
            actuator.forcerange = [-maximum, maximum]


def _maximum_effort(joint: JointSpec) -> float:
    """Return the effort limit matching one scalar joint's physical unit."""
    if joint.limits is None:
        raise MuJoCoSceneError(
            f"effort-controlled joint '{joint.id}' does not declare effort limits"
        )
    maximum = (
        joint.limits.max_effort_nm
        if joint.type in {JointType.REVOLUTE, JointType.CONTINUOUS}
        else joint.limits.max_effort_n
    )
    if maximum is None:
        unit = "max_effort_nm" if joint.type is not JointType.PRISMATIC else "max_effort_n"
        raise MuJoCoSceneError(f"effort-controlled joint '{joint.id}' does not declare '{unit}'")
    return float(maximum)


def _add_connector_sites(
    spec: mujoco.MjSpec,
    module_types: dict[ModuleInstanceId, ModuleType],
) -> None:
    """Materialise every connector as a MuJoCo site on its parent link.

    Sites make connector frames first-class in the physics model, so the adapter
    reports measured frames instead of recomposing them from link poses. That
    removes any chance of ModSim and MuJoCo disagreeing about where a connector
    is.
    """
    for module_id, module_type in module_types.items():
        for connector in module_type.connectors:
            parent = spec.body(body_name(module_id, connector.parent_link))
            if parent is None:
                raise MuJoCoSceneError(
                    f"connector '{connector.id}' names parent link "
                    f"'{connector.parent_link}', which the asset does not define"
                )
            if connector.local_pose is None:
                raise MuJoCoSceneError(
                    f"connector '{connector.id}' on module type '{module_type.id}' "
                    f"uses named frame '{connector.frame}', but the MuJoCo backend "
                    "cannot resolve named connector frames yet; author an explicit "
                    "local_pose"
                )
            local = Transform.from_pose_spec(connector.local_pose)
            parent.add_site(
                name=site_name(module_id, connector.id),
                pos=list(local.translation),
                quat=list(local.rotation),
            )


def _reserve_weld_pool(
    spec: mujoco.MjSpec,
    module_types: dict[ModuleInstanceId, ModuleType],
    requested: int | None,
) -> tuple[str, ...]:
    """Pre-allocate inactive weld constraints for runtime docking."""
    connectors = sum(len(module_type.connectors) for module_type in module_types.values())
    size = requested if requested is not None else max(MIN_WELD_POOL, connectors // 2)
    if size < 0:
        raise MuJoCoSceneError("weld pool size must not be negative")
    if not module_types:
        return ()
    anchor = next(iter(module_types))
    anchor_body = body_name(anchor, module_types[anchor].root_link)

    names: list[str] = []
    for index in range(size):
        equality = spec.add_equality()
        equality.name = f"modsim_weld_{index}"
        equality.type = mujoco.mjtEq.mjEQ_WELD
        equality.objtype = mujoco.mjtObj.mjOBJ_BODY
        # Placeholder operands, rewritten when a slot is claimed. A slot is
        # anchored to the world body because MuJoCo rejects an equality whose
        # two operands are the same element, which a single-module scene would
        # otherwise produce.
        equality.name1 = anchor_body
        equality.name2 = WORLD_BODY
        equality.active = False
        names.append(equality.name)
    return tuple(names)


def _reserve_hinge_pool(
    spec: mujoco.MjSpec,
    module_types: dict[ModuleInstanceId, ModuleType],
    requested: int | None,
) -> tuple[tuple[str, str], ...]:
    """Pre-allocate two inactive point constraints for every runtime hinge."""
    connectors = sum(len(module_type.connectors) for module_type in module_types.values())
    size = requested if requested is not None else max(MIN_HINGE_POOL, connectors // 2)
    if size < 0:
        raise MuJoCoSceneError("hinge pool size must not be negative")
    if not module_types:
        return ()
    anchor = next(iter(module_types))
    anchor_body = body_name(anchor, module_types[anchor].root_link)

    pairs: list[tuple[str, str]] = []
    for slot_index in range(size):
        names: list[str] = []
        for point_index in range(2):
            equality = spec.add_equality()
            equality.name = f"modsim_hinge_{slot_index}_{point_index}"
            equality.type = mujoco.mjtEq.mjEQ_CONNECT
            equality.objtype = mujoco.mjtObj.mjOBJ_BODY
            # Placeholder operands are rewritten when the slot is claimed.
            # Anchoring to world avoids illegal same-body equality operands in
            # a scene that contains only one module.
            equality.name1 = anchor_body
            equality.name2 = WORLD_BODY
            equality.active = False
            names.append(equality.name)
        pairs.append((names[0], names[1]))
    return tuple(pairs)


def _reserve_contact_exclusion_pool(
    spec: mujoco.MjSpec,
    module_types: dict[ModuleInstanceId, ModuleType],
    size: int,
) -> tuple[str, ...]:
    """Reserve one mutable collision-exclusion entry per weld slot."""
    if not module_types or size == 0:
        return ()
    anchor = next(iter(module_types))
    anchor_body = body_name(anchor, module_types[anchor].root_link)
    names: list[str] = []
    for index in range(size):
        exclusion = spec.add_exclude(
            name=f"modsim_contact_exclusion_{index}",
            bodyname1=anchor_body,
            bodyname2=WORLD_BODY,
        )
        names.append(exclusion.name)
    return tuple(names)


def _index(
    model: mujoco.MjModel,
    pack: RobotPack,
    scene: SceneSpec,
    module_types: dict[ModuleInstanceId, ModuleType],
    pool_names: tuple[str, ...],
    hinge_pool_names: tuple[tuple[str, str], ...],
    exclusion_pool_names: tuple[str, ...],
    anisotropic_ground_geom_names: tuple[str, ...],
) -> CompiledScene:
    """Resolve every ModSim identifier to a compiled MuJoCo id."""
    body_ids: dict[tuple[ModuleInstanceId, str], int] = {}
    site_ids: dict[ConnectorInstanceId, int] = {}
    joint_ids: dict[tuple[ModuleInstanceId, str], int] = {}
    actuator_ids: dict[tuple[ModuleInstanceId, str], int] = {}
    bodies: dict[tuple[ModuleInstanceId, str], str] = {}
    joints: dict[tuple[ModuleInstanceId, str], str] = {}
    frames: dict[ConnectorInstanceId, str] = {}

    for module_id, links in scene.module_links(pack).items():
        for link in links:
            name = body_name(module_id, link)
            identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if identifier < 0:
                raise MuJoCoSceneError(f"compiled model has no body named '{name}'")
            body_ids[(module_id, link)] = identifier
            bodies[(module_id, link)] = name

    for module_id, module_type in module_types.items():
        for joint in module_type.joints:
            # BackendStateSnapshot.JointState and the current command API are
            # scalar. Fixed joints have no native MuJoCo joint after compile,
            # while floating/planar state needs a future multi-DOF contract.
            if joint.type not in {
                JointType.REVOLUTE,
                JointType.CONTINUOUS,
                JointType.PRISMATIC,
            }:
                continue
            key = (module_id, joint.id)
            name = joint_name(module_id, joint.source_joint_name)
            identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if identifier < 0:
                raise MuJoCoSceneError(f"compiled model has no joint named '{name}'")
            joint_ids[key] = identifier
            joints[key] = name
            if ControlMode.EFFORT in joint.control_modes:
                motor = actuator_name(module_id, joint.id)
                motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor)
                if motor_id < 0:
                    raise MuJoCoSceneError(f"compiled model has no actuator named '{motor}'")
                actuator_ids[key] = motor_id

        for connector in module_type.connectors:
            name = site_name(module_id, connector.id)
            identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
            if identifier < 0:
                raise MuJoCoSceneError(f"compiled model has no site named '{name}'")
            instance = connector_instance_id(module_id, connector.id)
            site_ids[instance] = identifier
            frames[instance] = name

    pool: list[int] = []
    for name in pool_names:
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, name)
        if identifier >= 0:
            pool.append(identifier)

    hinge_pool: list[tuple[int, int]] = []
    for names in hinge_pool_names:
        identifiers = tuple(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, name) for name in names
        )
        if all(identifier >= 0 for identifier in identifiers):
            hinge_pool.append((identifiers[0], identifiers[1]))

    exclusion_pool: list[int] = []
    for name in exclusion_pool_names:
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EXCLUDE, name)
        if identifier >= 0:
            exclusion_pool.append(identifier)

    reserved_exclusions = set(exclusion_pool)
    static_exclusions = tuple(
        int(signature)
        for identifier, signature in enumerate(model.exclude_signature)
        if identifier not in reserved_exclusions
    )
    # The collision filter searches this array in signature order. Runtime
    # slots start inactive (zero) and are re-sorted with authored entries every
    # time a connection is claimed or released.
    model.exclude_signature[:] = sorted((*static_exclusions, *(0 for _ in exclusion_pool)))

    anisotropic_ground_geoms: list[int] = []
    for name in anisotropic_ground_geom_names:
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if identifier >= 0:
            anisotropic_ground_geoms.append(identifier)

    return CompiledScene(
        model=model,
        handles=BackendHandleRegistry(bodies=bodies, joints=joints, connector_frames=frames),
        body_ids=body_ids,
        site_ids=site_ids,
        joint_ids=joint_ids,
        actuator_ids=actuator_ids,
        weld_pool=tuple(pool),
        hinge_pool=tuple(hinge_pool),
        contact_exclusion_pool=tuple(exclusion_pool),
        static_contact_exclusions=static_exclusions,
        anisotropic_ground_geoms=tuple(anisotropic_ground_geoms),
    )
