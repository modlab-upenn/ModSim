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
from modsim.robot_packs.schema import AssetCatalogKind, ModuleType, RobotPack

INSTANCE_SEPARATOR = "/"
CONNECTOR_NAMESPACE = "connector"
WORLD_BODY = "world"
GROUND_GEOM = "modsim_ground"
GROUND_HALF_EXTENT_M = 10.0
GROUND_THICKNESS_M = 0.05
ENVIRONMENT_GEOM_GROUP = 2
URDF_COLLISION_GEOM_GROUP = 3
DEFAULT_GRAVITY: Vec3 = (0.0, 0.0, -9.81)
MIN_WELD_POOL = 8


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


@dataclass(frozen=True, slots=True)
class CompiledScene:
    """A compiled MuJoCo model plus the ModSim identifier maps it needs."""

    model: mujoco.MjModel
    handles: BackendHandleRegistry
    body_ids: dict[tuple[ModuleInstanceId, str], int] = field(
        default_factory=dict[tuple[ModuleInstanceId, str], int]
    )
    site_ids: dict[ConnectorInstanceId, int] = field(default_factory=dict[ConnectorInstanceId, int])
    weld_pool: tuple[int, ...] = ()
    """Ids of pre-allocated, inactive weld equality constraints.

    MuJoCo fixes model topology at compile time, so runtime docking cannot
    create constraints; it can only claim one of these. The pool is reserved
    here so docking can activate a slot without recompiling the model.
    """


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
    _add_connector_sites(spec, module_types)
    pool_names = _reserve_weld_pool(spec, module_types, weld_pool_size)

    try:
        model = spec.compile()
    except (ValueError, RuntimeError) as error:
        raise MuJoCoSceneError(f"MuJoCo failed to compile the scene: {error}") from error

    return _index(model, pack, scene, module_types, pool_names)


def _add_ground(spec: mujoco.MjSpec, height_m: float) -> None:
    """Add an infinite ground plane so modules have something to rest on."""
    geom = spec.worldbody.add_geom()
    geom.name = GROUND_GEOM
    geom.type = mujoco.mjtGeom.mjGEOM_PLANE
    geom.size = [GROUND_HALF_EXTENT_M, GROUND_HALF_EXTENT_M, GROUND_THICKNESS_M]
    geom.pos = [0.0, 0.0, height_m]
    geom.group = ENVIRONMENT_GEOM_GROUP


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


def _index(
    model: mujoco.MjModel,
    pack: RobotPack,
    scene: SceneSpec,
    module_types: dict[ModuleInstanceId, ModuleType],
    pool_names: tuple[str, ...],
) -> CompiledScene:
    """Resolve every ModSim identifier to a compiled MuJoCo id."""
    body_ids: dict[tuple[ModuleInstanceId, str], int] = {}
    site_ids: dict[ConnectorInstanceId, int] = {}
    bodies: dict[tuple[ModuleInstanceId, str], str] = {}
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

    return CompiledScene(
        model=model,
        handles=BackendHandleRegistry(bodies=bodies, connector_frames=frames),
        body_ids=body_ids,
        site_ids=site_ids,
        weld_pool=tuple(pool),
    )
