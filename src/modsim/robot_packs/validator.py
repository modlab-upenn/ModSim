"""Cross-document and local-filesystem Robot Pack validation."""

from __future__ import annotations

import os
import stat
from enum import StrEnum
from pathlib import Path

from pydantic import ValidationError

from modsim.robot_packs.issues import (
    Severity,
    ValidationIssue,
    ValidationReport,
)
from modsim.robot_packs.schema import (
    AssetCatalogKind,
    ControlMode,
    JointType,
    LoadedRobotPack,
    PhysicalConstraintType,
    RobotPack,
)

ROBOT_PACK_MANIFEST = "robot_pack.yaml"


class ValidationProfile(StrEnum):
    """How incomplete, authoring-time connector data is classified."""

    AUTHORING = "authoring"
    SIMULATION = "simulation"


class RobotPackValidator:
    """Validate assets and semantic references across a loaded Robot Pack."""

    def validate(
        self,
        loaded: LoadedRobotPack,
        *,
        profile: ValidationProfile = ValidationProfile.AUTHORING,
    ) -> ValidationReport:
        """Return all deterministic validation issues for a loaded pack."""
        issues: list[ValidationIssue] = []
        try:
            pack = RobotPack.model_validate(loaded.pack.model_dump(mode="python"))
        except ValidationError as error:
            return ValidationReport.from_issues(
                [
                    ValidationIssue(
                        severity=Severity.ERROR,
                        code="schema.invalid_in_memory",
                        message=str(item["msg"]),
                        document="<in-memory>",
                        path=tuple(item["loc"]),
                        suggested_fix=(
                            "Rebuild the model through Pydantic validation instead of "
                            "mutating nested dictionaries."
                        ),
                    )
                    for item in error.errors(
                        include_url=False,
                        include_context=False,
                        include_input=False,
                    )
                ]
            )
        loaded = LoadedRobotPack(
            root=loaded.root,
            manifest_path=loaded.manifest_path,
            pack=pack,
        )
        manifest = pack.manifest

        self._validate_assets(loaded, issues)

        module_document = manifest.specs.modules
        connector_document = manifest.specs.connectors
        capability_document = manifest.specs.capabilities
        module_types = pack.hardware_catalog.module_types
        connector_types = pack.hardware_catalog.connector_types
        capabilities = pack.capability_catalog.capabilities
        completeness_severity = (
            Severity.WARNING if profile is ValidationProfile.AUTHORING else Severity.ERROR
        )

        for module_id, module in module_types.items():
            module_ref = f"module_type:{module_id}"
            if module.asset_ref not in manifest.assets.urdf:
                issues.append(
                    self._issue(
                        severity=Severity.ERROR,
                        code="reference.asset_unknown",
                        message=(
                            f"URDF asset '{module.asset_ref}' is not declared in robot_pack.yaml"
                        ),
                        document=module_document,
                        path=("module_types", module_id, "asset_ref"),
                        entity_ref=module_ref,
                        suggested_fix="Declare the ID in the root assets.urdf catalog.",
                    )
                )

            for index, joint in enumerate(module.joints):
                required_limit_fields: list[str] = []
                if ControlMode.POSITION in joint.control_modes:
                    if joint.type is JointType.REVOLUTE:
                        required_limit_fields.extend(("lower_position_rad", "upper_position_rad"))
                    elif joint.type is JointType.PRISMATIC:
                        required_limit_fields.extend(("lower_position_m", "upper_position_m"))
                if ControlMode.VELOCITY in joint.control_modes:
                    if joint.type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
                        required_limit_fields.append("max_velocity_rad_per_s")
                    elif joint.type is JointType.PRISMATIC:
                        required_limit_fields.append("max_velocity_m_per_s")
                if ControlMode.EFFORT in joint.control_modes:
                    if joint.type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
                        required_limit_fields.append("max_effort_nm")
                    elif joint.type is JointType.PRISMATIC:
                        required_limit_fields.append("max_effort_n")

                if required_limit_fields and joint.limits is None:
                    issues.append(
                        self._issue(
                            severity=completeness_severity,
                            code="joint.limits_missing",
                            message=(
                                "controlled joint is missing required limit metadata: "
                                + ", ".join(required_limit_fields)
                            ),
                            document=module_document,
                            path=("module_types", module_id, "joints", index, "limits"),
                            entity_ref=f"{module_ref}/joint:{joint.id}",
                            suggested_fix=(
                                "Add unit-bearing limits; use explicit null values for unknowns."
                            ),
                        )
                    )
                elif joint.limits is not None:
                    for field_name in required_limit_fields:
                        if getattr(joint.limits, field_name) is None:
                            issues.append(
                                self._issue(
                                    severity=completeness_severity,
                                    code="joint.limit_unknown",
                                    message=(f"control mode requires joint limit '{field_name}'"),
                                    document=module_document,
                                    path=(
                                        "module_types",
                                        module_id,
                                        "joints",
                                        index,
                                        "limits",
                                        field_name,
                                    ),
                                    entity_ref=f"{module_ref}/joint:{joint.id}",
                                    suggested_fix=(
                                        "Set the unit-bearing limit to a measured or "
                                        "conservative numeric value."
                                    ),
                                )
                            )

            for index, connector in enumerate(module.connectors):
                connector_ref = f"{module_ref}/connector:{connector.id}"
                if connector.connector_type not in connector_types:
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="reference.connector_type_unknown",
                            message=(f"connector type '{connector.connector_type}' is not defined"),
                            document=module_document,
                            path=(
                                "module_types",
                                module_id,
                                "connectors",
                                index,
                                "connector_type",
                            ),
                            entity_ref=connector_ref,
                            suggested_fix=(
                                "Add the connector type to the connector catalog or "
                                "correct the reference."
                            ),
                        )
                    )
                for field_name in ("docking_axis", "approach_axis"):
                    if getattr(connector, field_name) is None:
                        issues.append(
                            self._issue(
                                severity=completeness_severity,
                                code=f"connector.{field_name}_missing",
                                message=f"connector is missing '{field_name}'",
                                document=module_document,
                                path=(
                                    "module_types",
                                    module_id,
                                    "connectors",
                                    index,
                                    field_name,
                                ),
                                entity_ref=connector_ref,
                                suggested_fix=(
                                    "Define a unit axis in the connector's parent-link frame."
                                ),
                            )
                        )

            for index, capability_id in enumerate(module.capabilities):
                if capability_id not in capabilities:
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="reference.capability_unknown",
                            message=f"capability '{capability_id}' is not defined",
                            document=module_document,
                            path=(
                                "module_types",
                                module_id,
                                "capabilities",
                                index,
                            ),
                            entity_ref=module_ref,
                        )
                    )
                    continue
                capability = capabilities[capability_id]
                module_connector_types = {
                    connector.connector_type for connector in module.connectors
                }
                missing_connector_types = sorted(
                    set(capability.required_connector_types) - module_connector_types
                )
                if missing_connector_types:
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="capability.connector_requirement_missing",
                            message=(
                                f"capability '{capability_id}' requires connector type(s) "
                                + ", ".join(missing_connector_types)
                            ),
                            document=module_document,
                            path=(
                                "module_types",
                                module_id,
                                "capabilities",
                                index,
                            ),
                            entity_ref=module_ref,
                            suggested_fix=(
                                "Add connector instances of the required types or remove "
                                "the capability."
                            ),
                        )
                    )

        for connector_type_id, connector_type in connector_types.items():
            connector_ref = f"connector_type:{connector_type_id}"
            for index, compatible_id in enumerate(connector_type.compatible_with):
                if compatible_id not in connector_types:
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="reference.compatibility_target_unknown",
                            message=(f"compatible connector type '{compatible_id}' is not defined"),
                            document=connector_document,
                            path=(
                                "connector_types",
                                connector_type_id,
                                "compatible_with",
                                index,
                            ),
                            entity_ref=connector_ref,
                        )
                    )

            missing_fields = (
                (
                    "compatible_with",
                    not connector_type.compatible_with,
                    "connector.compatibility_missing",
                    "Define at least one compatible connector type.",
                ),
                (
                    "allowed_orientations",
                    connector_type.allowed_orientations is None,
                    "connector.allowed_orientations_missing",
                    "Define discrete or continuous allowed orientations.",
                ),
                (
                    "acceptance_region",
                    connector_type.acceptance_region is None,
                    "connector.acceptance_region_missing",
                    "Define position, orientation, and relative-velocity tolerances.",
                ),
                (
                    "physical_connection",
                    connector_type.physical_connection is None,
                    "connector.physical_connection_missing",
                    "Define the backend-independent physical connection intent.",
                ),
            )
            for field_name, is_missing, code, suggested_fix in missing_fields:
                if is_missing:
                    issues.append(
                        self._issue(
                            severity=completeness_severity,
                            code=code,
                            message=f"connector type is missing '{field_name}'",
                            document=connector_document,
                            path=("connector_types", connector_type_id, field_name),
                            entity_ref=connector_ref,
                            suggested_fix=suggested_fix,
                        )
                    )

            if connector_type.limits is None:
                issues.append(
                    self._issue(
                        severity=completeness_severity,
                        code="connector.limits_missing",
                        message="connector load limits are not defined",
                        document=connector_document,
                        path=("connector_types", connector_type_id, "limits"),
                        entity_ref=connector_ref,
                        suggested_fix=(
                            "Define known limits; use explicit null values for unknown limits."
                        ),
                    )
                )
            else:
                for field_name in (
                    "max_normal_force_n",
                    "max_shear_force_n",
                    "max_bending_moment_nm",
                ):
                    if getattr(connector_type.limits, field_name) is None:
                        issues.append(
                            self._issue(
                                severity=completeness_severity,
                                code="connector.limit_unknown",
                                message=f"connector limit '{field_name}' is unknown",
                                document=connector_document,
                                path=(
                                    "connector_types",
                                    connector_type_id,
                                    "limits",
                                    field_name,
                                ),
                                entity_ref=connector_ref,
                            )
                        )

            if (
                connector_type.physical_connection is not None
                and connector_type.physical_connection.constraint
                in {
                    PhysicalConstraintType.HINGE,
                    PhysicalConstraintType.BALL,
                    PhysicalConstraintType.CUSTOM,
                }
            ):
                issues.append(
                    self._issue(
                        severity=completeness_severity,
                        code="connector.physical_connection_unsupported",
                        message=(
                            "schema 0.1 does not yet carry the parameters needed for "
                            f"'{connector_type.physical_connection.constraint.value}' constraints"
                        ),
                        document=connector_document,
                        path=(
                            "connector_types",
                            connector_type_id,
                            "physical_connection",
                            "constraint",
                        ),
                        entity_ref=connector_ref,
                        suggested_fix=(
                            "Use a fixed or compliant connection for Phase 1, or keep "
                            "this pack in the authoring profile."
                        ),
                    )
                )

        for capability_id, capability in capabilities.items():
            for index, connector_type_id in enumerate(capability.required_connector_types):
                if connector_type_id not in connector_types:
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="reference.connector_type_unknown",
                            message=(
                                f"required connector type '{connector_type_id}' is not defined"
                            ),
                            document=capability_document,
                            path=(
                                "capabilities",
                                capability_id,
                                "required_connector_types",
                                index,
                            ),
                            entity_ref=f"capability:{capability_id}",
                        )
                    )

        if profile is ValidationProfile.SIMULATION and not pack.backend_mappings:
            issues.append(
                self._issue(
                    severity=Severity.ERROR,
                    code="mapping.none",
                    message="simulation profile requires at least one backend mapping",
                    document=ROBOT_PACK_MANIFEST,
                    path=("mappings",),
                    suggested_fix="Declare and define a backend mapping document.",
                )
            )

        for backend_id, mapping in pack.backend_mappings.items():
            mapping_document = manifest.mappings[backend_id]
            asset_catalog = manifest.assets.mechanical_catalog(mapping.asset_catalog)
            missing_mapped_modules = sorted(set(module_types) - set(mapping.module_types))
            for module_id in missing_mapped_modules:
                issues.append(
                    self._issue(
                        severity=completeness_severity,
                        code="mapping.module_missing",
                        message=(f"backend '{backend_id}' does not map module type '{module_id}'"),
                        document=mapping_document,
                        path=("module_types", module_id),
                        entity_ref=f"backend:{backend_id}/module_type:{module_id}",
                        suggested_fix="Add a backend mapping for this module type.",
                    )
                )
            for module_id, module_mapping in mapping.module_types.items():
                mapping_path = ("module_types", module_id)
                if (
                    module_mapping.asset_ref is not None
                    and module_mapping.asset_ref not in asset_catalog
                ):
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="reference.asset_unknown",
                            message=(
                                f"asset_ref '{module_mapping.asset_ref}' is not declared "
                                f"in assets.{mapping.asset_catalog.value}"
                            ),
                            document=mapping_document,
                            path=(*mapping_path, "asset_ref"),
                            entity_ref=f"backend:{backend_id}/module_type:{module_id}",
                            suggested_fix=(
                                "Declare the asset ID in the selected root asset catalog."
                            ),
                        )
                    )
                if module_id not in module_types:
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="reference.mapping_module_unknown",
                            message=f"mapped module type '{module_id}' is not defined",
                            document=mapping_document,
                            path=mapping_path,
                            entity_ref=f"backend:{backend_id}/module_type:{module_id}",
                        )
                    )
                    continue

                module = module_types[module_id]
                joint_ids = {joint.id for joint in module.joints}
                connector_ids = {connector.id for connector in module.connectors}
                if (
                    module_mapping.asset_ref is None
                    and mapping.asset_catalog is not AssetCatalogKind.URDF
                ):
                    issues.append(
                        self._issue(
                            severity=completeness_severity,
                            code="mapping.asset_ref_missing",
                            message=(
                                f"backend '{backend_id}' needs an explicit "
                                f"assets.{mapping.asset_catalog.value} reference"
                            ),
                            document=mapping_document,
                            path=(*mapping_path, "asset_ref"),
                            entity_ref=f"backend:{backend_id}/module_type:{module_id}",
                        )
                    )
                if module.root_link not in module_mapping.link_map:
                    issues.append(
                        self._issue(
                            severity=completeness_severity,
                            code="mapping.root_link_missing",
                            message=f"root link '{module.root_link}' is not mapped",
                            document=mapping_document,
                            path=(*mapping_path, "link_map", module.root_link),
                            entity_ref=f"module_type:{module_id}",
                        )
                    )
                missing_joints = sorted(joint_ids - set(module_mapping.joint_map))
                for joint_id in missing_joints:
                    issues.append(
                        self._issue(
                            severity=completeness_severity,
                            code="mapping.joint_missing",
                            message=f"joint '{joint_id}' is not mapped",
                            document=mapping_document,
                            path=(*mapping_path, "joint_map", joint_id),
                            entity_ref=f"module_type:{module_id}",
                        )
                    )
                frame_only_connectors = {
                    connector.id
                    for connector in module.connectors
                    if connector.frame is not None and connector.local_pose is None
                }
                for connector_id in sorted(
                    frame_only_connectors - set(module_mapping.connector_frame_map)
                ):
                    issues.append(
                        self._issue(
                            severity=completeness_severity,
                            code="mapping.connector_frame_missing",
                            message=f"connector frame '{connector_id}' is not mapped",
                            document=mapping_document,
                            path=(
                                *mapping_path,
                                "connector_frame_map",
                                connector_id,
                            ),
                            entity_ref=f"module_type:{module_id}",
                        )
                    )
                for joint_id in module_mapping.joint_map:
                    if joint_id not in joint_ids:
                        issues.append(
                            self._issue(
                                severity=Severity.ERROR,
                                code="reference.mapping_joint_unknown",
                                message=f"mapped joint '{joint_id}' is not defined",
                                document=mapping_document,
                                path=(*mapping_path, "joint_map", joint_id),
                                entity_ref=f"module_type:{module_id}",
                            )
                        )
                for connector_id in module_mapping.connector_frame_map:
                    if connector_id not in connector_ids:
                        issues.append(
                            self._issue(
                                severity=Severity.ERROR,
                                code="reference.mapping_connector_unknown",
                                message=(f"mapped connector '{connector_id}' is not defined"),
                                document=mapping_document,
                                path=(
                                    *mapping_path,
                                    "connector_frame_map",
                                    connector_id,
                                ),
                                entity_ref=f"module_type:{module_id}",
                            )
                        )

        return ValidationReport.from_issues(issues)

    def _validate_assets(self, loaded: LoadedRobotPack, issues: list[ValidationIssue]) -> None:
        manifest = loaded.pack.manifest
        for catalog, asset_id, relative_path, expected_kind in manifest.assets.paths_with_kinds():
            self._check_asset(
                loaded.root,
                relative_path,
                expected_kind=expected_kind,
                document=ROBOT_PACK_MANIFEST,
                path=("assets", catalog, asset_id),
                entity_ref=f"asset:{catalog}/{asset_id}",
                issues=issues,
            )

    def _check_asset(
        self,
        root: Path,
        relative_path: str,
        *,
        expected_kind: str,
        document: str,
        path: tuple[str | int, ...],
        issues: list[ValidationIssue],
        entity_ref: str | None = None,
    ) -> None:
        root = root.resolve(strict=False)
        unresolved_candidate = root / relative_path
        try:
            candidate = unresolved_candidate.resolve(strict=False)
        except (OSError, RuntimeError, ValueError) as error:
            issues.append(
                self._issue(
                    severity=Severity.ERROR,
                    code="path.invalid",
                    message=f"cannot resolve asset '{relative_path}': {error}",
                    document=document,
                    path=path,
                    entity_ref=entity_ref,
                )
            )
            return
        if not self._is_within(candidate, root):
            issues.append(
                self._issue(
                    severity=Severity.ERROR,
                    code="path.outside_pack",
                    message=f"asset '{relative_path}' resolves outside the Robot Pack",
                    document=document,
                    path=path,
                    entity_ref=entity_ref,
                )
            )
            return

        cursor = root
        for component in Path(relative_path).parts:
            cursor /= component
            try:
                if stat.S_ISLNK(cursor.lstat().st_mode):
                    issues.append(
                        self._issue(
                            severity=Severity.ERROR,
                            code="asset.symlink_not_allowed",
                            message=f"asset path contains a symbolic link: '{relative_path}'",
                            document=document,
                            path=path,
                            entity_ref=entity_ref,
                            suggested_fix=(
                                "Replace the symlink with files physically contained "
                                "inside the Robot Pack."
                            ),
                        )
                    )
                    return
            except FileNotFoundError:
                break
            except OSError as error:
                issues.append(
                    self._issue(
                        severity=Severity.ERROR,
                        code="asset.inspect_error",
                        message=f"cannot inspect asset '{relative_path}': {error}",
                        document=document,
                        path=path,
                        entity_ref=entity_ref,
                    )
                )
                return

        if not candidate.exists():
            issues.append(
                self._issue(
                    severity=Severity.ERROR,
                    code="asset.missing",
                    message=f"asset '{relative_path}' does not exist",
                    document=document,
                    path=path,
                    entity_ref=entity_ref,
                    suggested_fix="Add the asset inside the Robot Pack or correct the path.",
                )
            )
            return
        if expected_kind == "file" and not candidate.is_file():
            issues.append(
                self._issue(
                    severity=Severity.ERROR,
                    code="asset.expected_file",
                    message=f"asset '{relative_path}' is not a regular file",
                    document=document,
                    path=path,
                    entity_ref=entity_ref,
                )
            )
        if expected_kind == "directory" and not candidate.is_dir():
            issues.append(
                self._issue(
                    severity=Severity.ERROR,
                    code="asset.expected_directory",
                    message=f"asset '{relative_path}' is not a directory",
                    document=document,
                    path=path,
                    entity_ref=entity_ref,
                )
            )
            return

        unsafe_entry = self._find_unsafe_asset_entry(candidate)
        if unsafe_entry is not None:
            code, message = unsafe_entry
            issues.append(
                self._issue(
                    severity=Severity.ERROR,
                    code=code,
                    message=message,
                    document=document,
                    path=path,
                    entity_ref=entity_ref,
                    suggested_fix=(
                        "Keep only regular files and directories physically inside the Robot Pack."
                    ),
                )
            )

    @classmethod
    def _find_unsafe_asset_entry(cls, path: Path) -> tuple[str, str] | None:
        try:
            mode = path.lstat().st_mode
        except OSError as error:
            return ("asset.inspect_error", f"cannot inspect asset entry '{path}': {error}")
        if stat.S_ISLNK(mode):
            return (
                "asset.symlink_not_allowed",
                f"asset tree contains a symbolic link: {path}",
            )
        if stat.S_ISREG(mode):
            return None
        if not stat.S_ISDIR(mode):
            return (
                "asset.special_file",
                f"asset tree contains a non-regular entry: {path}",
            )
        try:
            with os.scandir(path) as iterator:
                children = sorted(
                    (Path(entry.path) for entry in iterator),
                    key=lambda child: child.name,
                )
        except OSError as error:
            return (
                "asset.inspect_error",
                f"cannot inspect asset directory '{path}': {error}",
            )
        for child in children:
            issue = cls._find_unsafe_asset_entry(child)
            if issue is not None:
                return issue
        return None

    @staticmethod
    def _is_within(path: Path, root: Path) -> bool:
        try:
            path.relative_to(root)
        except ValueError:
            return False
        return True

    @staticmethod
    def _issue(
        *,
        severity: Severity,
        code: str,
        message: str,
        document: str,
        path: tuple[str | int, ...],
        entity_ref: str | None = None,
        suggested_fix: str | None = None,
    ) -> ValidationIssue:
        return ValidationIssue(
            severity=severity,
            code=code,
            message=message,
            document=document,
            path=path,
            entity_ref=entity_ref,
            suggested_fix=suggested_fix,
        )


def validate_robot_pack(
    loaded: LoadedRobotPack,
    *,
    profile: ValidationProfile = ValidationProfile.AUTHORING,
) -> ValidationReport:
    """Validate a Robot Pack using the default validator."""
    return RobotPackValidator().validate(loaded, profile=profile)
