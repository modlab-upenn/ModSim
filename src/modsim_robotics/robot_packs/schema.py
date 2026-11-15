"""Typed Robot Pack configuration models."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import StrEnum
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Annotated, Literal, Self

from pydantic import (
    AfterValidator,
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    StrictBool,
    StrictFloat,
    StringConstraints,
    field_validator,
    model_validator,
)

SCHEMA_VERSION: Literal["0.1"] = "0.1"


def _validate_pack_relative_path(value: str) -> str:
    """Require a portable path that cannot leave a Robot Pack."""
    path = PurePosixPath(value)
    windows_path = PureWindowsPath(value)
    if not value or value == ".":
        raise ValueError("path must not be empty")
    if len(value) > 1024:
        raise ValueError("path must not exceed 1024 characters")
    if any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise ValueError("control characters are not allowed in paths")
    if value.startswith("~"):
        raise ValueError("home-relative paths are not allowed")
    if "\\" in value:
        raise ValueError("Robot Pack paths must use POSIX '/' separators")
    if path.is_absolute() or windows_path.is_absolute() or windows_path.drive:
        raise ValueError("absolute paths are not allowed")
    if ".." in path.parts:
        raise ValueError("parent traversal ('..') is not allowed")
    if any(len(part) > 255 for part in path.parts):
        raise ValueError("path components must not exceed 255 characters")
    return path.as_posix()


def _validate_unit_vector(value: tuple[float, float, float]) -> tuple[float, float, float]:
    magnitude = math.sqrt(sum(component * component for component in value))
    if not math.isclose(magnitude, 1.0, rel_tol=1e-6, abs_tol=1e-6):
        raise ValueError("direction vector must have unit length")
    return value


Identifier = Annotated[
    str,
    StringConstraints(
        min_length=1,
        max_length=64,
        pattern=r"^[a-z][a-z0-9]*(?:_[a-z0-9]+)*$",
        strip_whitespace=True,
    ),
]
NonEmptyString = Annotated[str, StringConstraints(min_length=1, strip_whitespace=True)]
MetadataKey = Annotated[
    str,
    StringConstraints(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z][A-Za-z0-9_.:-]*$",
        strip_whitespace=True,
    ),
]
Metadata = dict[MetadataKey, JsonValue]
SemanticVersion = Annotated[
    str,
    StringConstraints(
        pattern=r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"
        r"(?:-[0-9A-Za-z.-]+)?(?:\+[0-9A-Za-z.-]+)?$",
        strip_whitespace=True,
    ),
]
PackRelativePath = Annotated[
    str,
    StringConstraints(min_length=1, strip_whitespace=True),
    AfterValidator(_validate_pack_relative_path),
]
FiniteFloat = StrictFloat
PositiveFloat = Annotated[StrictFloat, Field(gt=0.0)]
Vector3 = tuple[StrictFloat, StrictFloat, StrictFloat]
UnitVector3 = Annotated[Vector3, AfterValidator(_validate_unit_vector)]


class StrictModel(BaseModel):
    """Shared strict configuration for Robot Pack documents."""

    model_config = ConfigDict(
        allow_inf_nan=False,
        extra="forbid",
        frozen=True,
        str_strip_whitespace=True,
        validate_default=True,
    )


class JointType(StrEnum):
    """Supported mechanical joint categories."""

    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FLOATING = "floating"
    PLANAR = "planar"


class ControlMode(StrEnum):
    """Actuator command modes exposed by a joint."""

    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"


class ConnectorGender(StrEnum):
    """Mechanical connector gender model."""

    MALE = "male"
    FEMALE = "female"
    HERMAPHRODITIC = "hermaphroditic"
    GENDERLESS = "genderless"


class OrientationMode(StrEnum):
    """How relative connector orientations are accepted."""

    DISCRETE = "discrete"
    CONTINUOUS = "continuous"


class AcceptanceShape(StrEnum):
    """Simplified acceptance-region shape."""

    BOX = "box"
    SPHERE = "sphere"
    CYLINDER = "cylinder"


class PhysicalConstraintType(StrEnum):
    """Backend-independent physical connection intent."""

    FIXED = "fixed"
    COMPLIANT = "compliant"
    HINGE = "hinge"
    BALL = "ball"
    CUSTOM = "custom"


class AlignmentMode(StrEnum):
    """How a committed connection's relative pose is chosen."""

    MEASURED = "measured"
    NOMINAL = "nominal"


class CapabilityKind(StrEnum):
    """High-level capability category."""

    PRIMITIVE_ACTION = "primitive_action"
    BEHAVIOR = "behavior"


class ModelViewMode(StrEnum):
    """Robot Pack contexts in which a model-view recipe is intended for use."""

    AUTHORING = "authoring"
    RUNTIME = "runtime"


class PoseSpec(StrictModel):
    """A local pose expressed in SI units."""

    xyz_m: Vector3 = (0.0, 0.0, 0.0)
    rpy_rad: Vector3 = (0.0, 0.0, 0.0)


class JointLimits(StrictModel):
    """Explicit angular or linear limits for a one-degree-of-freedom joint."""

    lower_position_rad: FiniteFloat | None = None
    upper_position_rad: FiniteFloat | None = None
    lower_position_m: FiniteFloat | None = None
    upper_position_m: FiniteFloat | None = None
    max_velocity_rad_per_s: PositiveFloat | None = None
    max_velocity_m_per_s: PositiveFloat | None = None
    max_effort_nm: PositiveFloat | None = None
    max_effort_n: PositiveFloat | None = None

    @model_validator(mode="after")
    def validate_range(self) -> Self:
        ranges = (
            ("angular", self.lower_position_rad, self.upper_position_rad),
            ("linear", self.lower_position_m, self.upper_position_m),
        )
        for label, lower, upper in ranges:
            if lower is not None and upper is not None and lower > upper:
                raise ValueError(f"{label} lower limit must not exceed upper limit")
        return self


class JointSpec(StrictModel):
    """Semantic joint metadata linked to a source mechanical joint."""

    id: Identifier
    source_joint_name: NonEmptyString
    type: JointType
    parent_link: NonEmptyString
    child_link: NonEmptyString
    axis: UnitVector3 | None = Field(
        default=None,
        description="Unit axis expressed in the source joint frame.",
    )
    control_modes: tuple[ControlMode, ...] = ()
    limits: JointLimits | None = None

    @model_validator(mode="after")
    def validate_joint(self) -> Self:
        if self.parent_link == self.child_link:
            raise ValueError("parent_link and child_link must differ")
        if len(set(self.control_modes)) != len(self.control_modes):
            raise ValueError("control_modes must not contain duplicates")

        axis_required = {
            JointType.REVOLUTE,
            JointType.CONTINUOUS,
            JointType.PRISMATIC,
            JointType.PLANAR,
        }
        if self.type in axis_required and self.axis is None:
            raise ValueError(f"{self.type.value} joints require a unit axis")
        if self.type in {JointType.FIXED, JointType.FLOATING} and self.axis is not None:
            raise ValueError(f"{self.type.value} joints must not define an axis")

        scalar_control_joints = {
            JointType.REVOLUTE,
            JointType.CONTINUOUS,
            JointType.PRISMATIC,
        }
        if self.type not in scalar_control_joints and self.control_modes:
            raise ValueError(
                f"{self.type.value} joints do not support scalar control modes in schema 0.1"
            )
        if self.type not in scalar_control_joints and self.limits is not None:
            raise ValueError(f"{self.type.value} joints do not support scalar limits in schema 0.1")

        if self.limits is not None:
            angular_values = (
                self.limits.lower_position_rad,
                self.limits.upper_position_rad,
                self.limits.max_velocity_rad_per_s,
                self.limits.max_effort_nm,
            )
            linear_values = (
                self.limits.lower_position_m,
                self.limits.upper_position_m,
                self.limits.max_velocity_m_per_s,
                self.limits.max_effort_n,
            )
            if self.type in {JointType.REVOLUTE, JointType.CONTINUOUS} and any(
                value is not None for value in linear_values
            ):
                raise ValueError("angular joints must not define linear limits")
            if self.type is JointType.PRISMATIC and any(
                value is not None for value in angular_values
            ):
                raise ValueError("prismatic joints must not define angular limits")
            if self.type is JointType.CONTINUOUS and (
                self.limits.lower_position_rad is not None
                or self.limits.upper_position_rad is not None
            ):
                raise ValueError("continuous joints must not define position bounds")
        return self


class ConnectorSpec(StrictModel):
    """A connector instance attached to a module link or frame."""

    id: Identifier
    connector_type: Identifier
    parent_link: NonEmptyString
    frame: NonEmptyString | None = None
    local_pose: PoseSpec | None = None
    docking_axis: UnitVector3 | None = Field(
        default=None,
        description="Unit docking axis expressed in the connector parent-link frame.",
    )
    approach_axis: UnitVector3 | None = Field(
        default=None,
        description="Unit approach axis expressed in the connector parent-link frame.",
    )
    metadata: Metadata = Field(default_factory=dict, max_length=128)

    @model_validator(mode="after")
    def require_location(self) -> Self:
        if self.frame is None and self.local_pose is None:
            raise ValueError("connector requires either frame or local_pose")
        return self


class AllowedOrientations(StrictModel):
    """Allowed relative rotations about a docking axis."""

    mode: OrientationMode
    values_rad: tuple[StrictFloat, ...] = ()

    @model_validator(mode="after")
    def validate_values(self) -> Self:
        if self.mode is OrientationMode.DISCRETE and not self.values_rad:
            raise ValueError("discrete orientation mode requires at least one value")
        if self.mode is OrientationMode.CONTINUOUS and self.values_rad:
            raise ValueError("continuous orientation mode must not define discrete values")
        if len(set(self.values_rad)) != len(self.values_rad):
            raise ValueError("orientation values must not contain duplicates")
        return self


class AcceptanceRegion(StrictModel):
    """Geometric and kinematic tolerances required before docking."""

    shape: AcceptanceShape
    position_tolerance_m: PositiveFloat
    orientation_tolerance_rad: Annotated[StrictFloat, Field(gt=0.0, le=math.pi)]
    max_relative_velocity_m_s: PositiveFloat


class ComplianceSpec(StrictModel):
    """Optional compliant-connection parameters."""

    translational_stiffness_n_per_m: PositiveFloat | None = None
    rotational_stiffness_nm_per_rad: PositiveFloat | None = None

    @model_validator(mode="after")
    def require_stiffness(self) -> Self:
        if (
            self.translational_stiffness_n_per_m is None
            and self.rotational_stiffness_nm_per_rad is None
        ):
            raise ValueError("compliance requires at least one stiffness value")
        return self


class HingeConstraintSpec(StrictModel):
    """Geometry required to realise a hinge between two connector frames."""

    axis: UnitVector3 = Field(
        description=("Shared unit hinge axis expressed in each endpoint connector frame."),
    )
    anchor_separation_m: PositiveFloat = Field(
        description="Distance between the two hinge anchors along the local hinge axis.",
    )


class PhysicalConnectionSpec(StrictModel):
    """Physical constraint requested from a simulation backend."""

    constraint: PhysicalConstraintType
    compliance: ComplianceSpec | None = None
    hinge: HingeConstraintSpec | None = None

    @model_validator(mode="after")
    def validate_constraint_parameters(self) -> Self:
        if self.constraint is PhysicalConstraintType.COMPLIANT and self.compliance is None:
            raise ValueError("compliant constraints require compliance parameters")
        if self.constraint is not PhysicalConstraintType.COMPLIANT and self.compliance is not None:
            raise ValueError("compliance parameters require a compliant constraint")
        if self.constraint is PhysicalConstraintType.HINGE and self.hinge is None:
            raise ValueError("hinge constraints require hinge parameters")
        if self.constraint is not PhysicalConstraintType.HINGE and self.hinge is not None:
            raise ValueError("hinge parameters require a hinge constraint")
        return self


class ConnectorLimits(StrictModel):
    """Known connector load limits. Null values represent unknown limits."""

    max_normal_force_n: PositiveFloat | None = None
    max_shear_force_n: PositiveFloat | None = None
    max_bending_moment_nm: PositiveFloat | None = None


class DockingPolicySpec(StrictModel):
    """Runtime docking behaviour for a connector type.

    The runtime docking engine consumes this policy. It does not change the
    mechanical description of a connector, so an omitted policy is equivalent
    to this model's defaults.
    """

    auto_latch: StrictBool = Field(
        default=False,
        description=(
            "Latch as soon as acceptance is satisfied, without an explicit dock command. "
            "Passive connectors such as permanent magnets are usually auto-latching."
        ),
    )
    alignment: AlignmentMode = Field(
        default=AlignmentMode.MEASURED,
        description=(
            "Whether a committed connection freezes the measured relative pose or snaps "
            "to the nominal mating pose implied by the docking axis and orientation set."
        ),
    )
    redock_cooldown_s: PositiveFloat | None = Field(
        default=None,
        description="Minimum time a connector must stay free after undocking or a failed dock.",
    )
    break_force_n: PositiveFloat | None = Field(
        default=None,
        description=(
            "Constraint force above which the connection releases on its own. "
            "Null means the connection never breaks under load."
        ),
    )


class ConnectorTypeSpec(StrictModel):
    """Reusable mechanical and semantic connector definition."""

    id: Identifier
    name: NonEmptyString | None = None
    active: StrictBool = False
    gender: ConnectorGender = ConnectorGender.GENDERLESS
    compatible_with: tuple[Identifier, ...] = ()
    allowed_orientations: AllowedOrientations | None = None
    acceptance_region: AcceptanceRegion | None = None
    physical_connection: PhysicalConnectionSpec | None = None
    limits: ConnectorLimits | None = None
    supports_undocking: StrictBool = False
    docking_policy: DockingPolicySpec | None = None
    metadata: Metadata = Field(default_factory=dict, max_length=128)

    @property
    def effective_docking_policy(self) -> DockingPolicySpec:
        """Return the declared docking policy or the documented defaults."""
        return self.docking_policy if self.docking_policy is not None else DockingPolicySpec()

    @field_validator("compatible_with")
    @classmethod
    def require_unique_compatibility(cls, value: tuple[Identifier, ...]) -> tuple[Identifier, ...]:
        if len(set(value)) != len(value):
            raise ValueError("compatible_with must not contain duplicates")
        return value


class ModuleType(StrictModel):
    """A reusable robot module type backed by a mechanical asset."""

    id: Identifier
    name: NonEmptyString | None = None
    asset_ref: Identifier
    root_link: NonEmptyString
    mass_kg: PositiveFloat | None = None
    joints: tuple[JointSpec, ...] = ()
    connectors: tuple[ConnectorSpec, ...] = ()
    capabilities: tuple[Identifier, ...] = ()

    @model_validator(mode="after")
    def require_unique_child_ids(self) -> Self:
        joint_ids = [joint.id for joint in self.joints]
        connector_ids = [connector.id for connector in self.connectors]
        if len(set(joint_ids)) != len(joint_ids):
            raise ValueError("joint IDs must be unique within a module type")
        if len(set(connector_ids)) != len(connector_ids):
            raise ValueError("connector IDs must be unique within a module type")
        if len(set(self.capabilities)) != len(self.capabilities):
            raise ValueError("capability references must not contain duplicates")
        return self


class CapabilitySpec(StrictModel):
    """A capability advertised by one or more module types."""

    id: Identifier
    name: NonEmptyString
    kind: CapabilityKind
    description: str | None = None
    required_connector_types: tuple[Identifier, ...] = ()

    @field_validator("required_connector_types")
    @classmethod
    def require_unique_connector_requirements(
        cls, value: tuple[Identifier, ...]
    ) -> tuple[Identifier, ...]:
        if len(set(value)) != len(value):
            raise ValueError("required_connector_types must not contain duplicates")
        return value


class ModelViewSpec(StrictModel):
    """Named recipe for generating a model view from available ModSim state."""

    id: Identifier
    name: NonEmptyString | None = None
    builder: Identifier
    modes: tuple[ModelViewMode, ...] = (ModelViewMode.RUNTIME,)
    default: StrictBool = False
    configuration: Metadata = Field(default_factory=dict, max_length=128)

    @field_validator("modes")
    @classmethod
    def require_unique_nonempty_modes(
        cls,
        value: tuple[ModelViewMode, ...],
    ) -> tuple[ModelViewMode, ...]:
        if not value:
            raise ValueError("model-view modes must not be empty")
        if len(set(value)) != len(value):
            raise ValueError("model-view modes must not contain duplicates")
        return value


class HardwareCatalog(StrictModel):
    """All hardware types defined by a Robot Pack."""

    module_types: dict[Identifier, ModuleType] = Field(min_length=1)
    connector_types: dict[Identifier, ConnectorTypeSpec]

    @model_validator(mode="after")
    def require_matching_catalog_ids(self) -> Self:
        for item_id, item in self.module_types.items():
            if item.id != item_id:
                raise ValueError(f"module type key '{item_id}' does not match id '{item.id}'")
        for item_id, item in self.connector_types.items():
            if item.id != item_id:
                raise ValueError(f"connector type key '{item_id}' does not match id '{item.id}'")
        return self


class CapabilityCatalog(StrictModel):
    """Capabilities defined by a Robot Pack."""

    capabilities: dict[Identifier, CapabilitySpec] = Field(default_factory=dict)

    @model_validator(mode="after")
    def require_matching_catalog_ids(self) -> Self:
        for item_id, item in self.capabilities.items():
            if item.id != item_id:
                raise ValueError(f"capability key '{item_id}' does not match id '{item.id}'")
        return self


class AssetCatalogKind(StrEnum):
    """Mechanical asset catalogs that a backend mapping may target."""

    URDF = "urdf"
    MUJOCO = "mujoco"
    ISAAC_USD = "isaac_usd"


class ModuleBackendMapping(StrictModel):
    """Backend names and handles associated with one module type."""

    asset_ref: Identifier | None = None
    link_map: dict[NonEmptyString, NonEmptyString] = Field(default_factory=dict)
    joint_map: dict[Identifier, NonEmptyString] = Field(default_factory=dict)
    connector_frame_map: dict[Identifier, NonEmptyString] = Field(default_factory=dict)
    constraint_map: dict[Identifier, NonEmptyString] = Field(default_factory=dict)


class BackendMapping(StrictModel):
    """Mapping document for one backend, scoped by module type."""

    schema_version: Literal["0.1"] = SCHEMA_VERSION
    backend: Identifier
    asset_catalog: AssetCatalogKind
    module_types: dict[Identifier, ModuleBackendMapping] = Field(min_length=1)


class AssetManifest(StrictModel):
    """Typed, keyed local assets available to module and backend mappings."""

    urdf: dict[Identifier, PackRelativePath] = Field(min_length=1)
    mesh_directories: dict[Identifier, PackRelativePath] = Field(default_factory=dict)
    mujoco: dict[Identifier, PackRelativePath] = Field(default_factory=dict)
    isaac_usd: dict[Identifier, PackRelativePath] = Field(default_factory=dict)
    thumbnails: dict[Identifier, PackRelativePath] = Field(default_factory=dict)

    @model_validator(mode="after")
    def require_unique_asset_paths(self) -> Self:
        paths = [
            path
            for catalog in (
                self.urdf,
                self.mesh_directories,
                self.mujoco,
                self.isaac_usd,
                self.thumbnails,
            )
            for path in catalog.values()
        ]
        if len(set(paths)) != len(paths):
            raise ValueError("each declared asset must use a distinct path")
        return self

    def mechanical_catalog(self, kind: AssetCatalogKind) -> dict[str, str]:
        """Return the mechanical asset catalog selected by a mapping document."""
        catalogs = {
            AssetCatalogKind.URDF: self.urdf,
            AssetCatalogKind.MUJOCO: self.mujoco,
            AssetCatalogKind.ISAAC_USD: self.isaac_usd,
        }
        return catalogs[kind]

    def paths_with_kinds(self) -> tuple[tuple[str, str, str, str], ...]:
        """Return ``(catalog, asset-id, path, expected-kind)`` entries."""
        entries: list[tuple[str, str, str, str]] = []
        for catalog_name, catalog, expected_kind in (
            ("urdf", self.urdf, "file"),
            ("mesh_directories", self.mesh_directories, "directory"),
            ("mujoco", self.mujoco, "file"),
            ("isaac_usd", self.isaac_usd, "file"),
            ("thumbnails", self.thumbnails, "file"),
        ):
            entries.extend(
                (catalog_name, asset_id, path, expected_kind) for asset_id, path in catalog.items()
            )
        return tuple(entries)


class SpecFileManifest(StrictModel):
    """Split specification documents referenced by the root manifest."""

    modules: PackRelativePath
    connectors: PackRelativePath
    capabilities: PackRelativePath

    @model_validator(mode="after")
    def require_unique_document_paths(self) -> Self:
        paths = (self.modules, self.connectors, self.capabilities)
        if len(set(paths)) != len(paths):
            raise ValueError("specification documents must use distinct paths")
        return self


class RobotPackManifest(StrictModel):
    """Contents of a Robot Pack's root ``robot_pack.yaml``."""

    schema_version: Literal["0.1"] = SCHEMA_VERSION
    id: Identifier
    name: NonEmptyString
    version: SemanticVersion
    description: str | None = None
    metadata: Metadata = Field(default_factory=dict, max_length=128)
    model_views: tuple[ModelViewSpec, ...] = ()
    assets: AssetManifest
    specs: SpecFileManifest
    mappings: dict[Identifier, PackRelativePath] = Field(default_factory=dict)

    @model_validator(mode="after")
    def require_unique_referenced_documents(self) -> Self:
        model_view_ids = [model_view.id for model_view in self.model_views]
        if len(set(model_view_ids)) != len(model_view_ids):
            raise ValueError("model-view IDs must be unique within a Robot Pack")

        spec_paths = {
            self.specs.modules,
            self.specs.connectors,
            self.specs.capabilities,
            "robot_pack.yaml",
        }
        mapping_paths = tuple(self.mappings.values())
        if len(set(mapping_paths)) != len(mapping_paths):
            raise ValueError("backend mapping documents must use distinct paths")
        collisions = spec_paths.intersection(mapping_paths)
        if collisions:
            paths = ", ".join(sorted(collisions))
            raise ValueError(f"specification and mapping paths overlap: {paths}")

        document_paths = {PurePosixPath(path) for path in (*spec_paths, *mapping_paths)}
        declared_assets = {
            PurePosixPath(path): f"{catalog}/{asset_id}"
            for catalog, asset_id, path, _ in self.assets.paths_with_kinds()
        }
        role_collisions = sorted(
            path.as_posix() for path in document_paths.intersection(declared_assets)
        )
        if role_collisions:
            raise ValueError("document and asset paths overlap: " + ", ".join(role_collisions))

        directory_assets = {PurePosixPath(path) for path in self.assets.mesh_directories.values()}
        for directory in directory_assets:
            contained_documents = sorted(
                path.as_posix()
                for path in document_paths
                if path != directory and path.is_relative_to(directory)
            )
            if contained_documents:
                raise ValueError(
                    f"asset directory '{directory}' contains Robot Pack documents: "
                    + ", ".join(contained_documents)
                )
            contained_assets = sorted(
                path.as_posix()
                for path in declared_assets
                if path != directory and path.is_relative_to(directory)
            )
            if contained_assets:
                raise ValueError(
                    f"asset directory '{directory}' contains separately declared assets: "
                    + ", ".join(contained_assets)
                )
        return self


class ModuleTypesDocument(StrictModel):
    """On-disk module-type catalog document."""

    schema_version: Literal["0.1"] = SCHEMA_VERSION
    module_types: dict[Identifier, ModuleType] = Field(min_length=1)


class ConnectorTypesDocument(StrictModel):
    """On-disk connector-type catalog document."""

    schema_version: Literal["0.1"] = SCHEMA_VERSION
    connector_types: dict[Identifier, ConnectorTypeSpec]


class CapabilitiesDocument(StrictModel):
    """On-disk capability catalog document."""

    schema_version: Literal["0.1"] = SCHEMA_VERSION
    capabilities: dict[Identifier, CapabilitySpec] = Field(default_factory=dict)


class RobotPack(StrictModel):
    """Fully loaded, backend-independent Robot Pack aggregate."""

    manifest: RobotPackManifest
    hardware_catalog: HardwareCatalog
    capability_catalog: CapabilityCatalog
    backend_mappings: dict[Identifier, BackendMapping] = Field(default_factory=dict)

    @model_validator(mode="after")
    def require_matching_backend_ids(self) -> Self:
        for backend_id, mapping in self.backend_mappings.items():
            if mapping.backend != backend_id:
                raise ValueError(
                    f"backend mapping key '{backend_id}' does not match backend '{mapping.backend}'"
                )
        manifest_backends = set(self.manifest.mappings)
        aggregate_backends = set(self.backend_mappings)
        if manifest_backends != aggregate_backends:
            missing = sorted(manifest_backends - aggregate_backends)
            undeclared = sorted(aggregate_backends - manifest_backends)
            details: list[str] = []
            if missing:
                details.append(f"missing mappings: {', '.join(missing)}")
            if undeclared:
                details.append(f"undeclared mappings: {', '.join(undeclared)}")
            raise ValueError("; ".join(details))
        return self

    @property
    def id(self) -> str:
        """Return the stable pack identifier."""
        return self.manifest.id

    @property
    def version(self) -> str:
        """Return the Robot Pack release version."""
        return self.manifest.version


@dataclass(frozen=True, slots=True)
class LoadedRobotPack:
    """A Robot Pack plus explicit filesystem context."""

    root: Path
    manifest_path: Path
    pack: RobotPack

    def with_pack(self, pack: RobotPack) -> LoadedRobotPack:
        """Return a new loaded bundle with updated semantic data."""
        return replace(self, pack=pack)
