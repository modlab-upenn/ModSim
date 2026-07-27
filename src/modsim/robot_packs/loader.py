"""Robot Pack manifest and split-document loader."""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path
from typing import TypeVar, cast

from pydantic import BaseModel, ValidationError
from ruamel.yaml import YAML
from ruamel.yaml.constructor import DuplicateKeyError
from ruamel.yaml.error import YAMLError

from modsim.robot_packs.errors import RobotPackLoadError
from modsim.robot_packs.issues import Severity, ValidationIssue
from modsim.robot_packs.schema import (
    SCHEMA_VERSION,
    BackendMapping,
    CapabilitiesDocument,
    CapabilityCatalog,
    ConnectorTypesDocument,
    HardwareCatalog,
    LoadedRobotPack,
    ModuleTypesDocument,
    RobotPack,
    RobotPackManifest,
)

ModelT = TypeVar("ModelT", bound=BaseModel)


class RobotPackLoader:
    """Load a Robot Pack directory into one typed aggregate."""

    manifest_filename = "robot_pack.yaml"

    def load(self, path: str | Path) -> LoadedRobotPack:
        """Load a pack directory or its root manifest."""
        supplied_path = Path(os.path.abspath(Path(path)))
        looks_like_manifest = supplied_path.suffix.lower() in {".yaml", ".yml"}
        if looks_like_manifest and supplied_path.name != self.manifest_filename:
            raise self._error(
                code="pack.manifest_name_invalid",
                message=(f"Robot Pack manifest must be named '{self.manifest_filename}'"),
                document=supplied_path.name,
                source=supplied_path,
                suggested_fix=(
                    f"Rename the file to {self.manifest_filename} or pass its parent directory."
                ),
            )
        supplied_is_directory = supplied_path.is_dir() or (
            not supplied_path.exists() and not looks_like_manifest
        )
        if not supplied_is_directory and supplied_path.name != self.manifest_filename:
            raise self._error(
                code="pack.manifest_name_invalid",
                message=(f"Robot Pack manifest must be named '{self.manifest_filename}'"),
                document=supplied_path.name,
                source=supplied_path,
                suggested_fix=(f"Pass a pack directory or its {self.manifest_filename} file."),
            )

        lexical_root = supplied_path if supplied_is_directory else supplied_path.parent
        lexical_manifest = (
            lexical_root / self.manifest_filename if supplied_is_directory else supplied_path
        )
        if lexical_root.is_symlink():
            raise self._error(
                code="pack.root_symlink_not_allowed",
                message="Robot Pack root must not be a symbolic link",
                document=self.manifest_filename,
                source=lexical_root,
            )
        if lexical_manifest.is_symlink():
            raise self._error(
                code="document.symlink_not_allowed",
                message="Robot Pack manifest must not be a symbolic link",
                document=self.manifest_filename,
                source=lexical_manifest,
            )
        try:
            root = lexical_root.resolve(strict=False)
        except (OSError, RuntimeError, ValueError) as error:
            raise self._error(
                code="path.invalid",
                message=f"cannot resolve Robot Pack root: {error}",
                document=self.manifest_filename,
                source=lexical_root,
            ) from error
        manifest_path = root / self.manifest_filename
        if manifest_path.is_symlink():
            raise self._error(
                code="document.symlink_not_allowed",
                message="Robot Pack manifest must not be a symbolic link",
                document=self.manifest_filename,
                source=manifest_path,
            )

        if not manifest_path.exists():
            raise self._error(
                code="pack.manifest_missing",
                message=f"Robot Pack manifest '{self.manifest_filename}' does not exist",
                document=self.manifest_filename,
                source=manifest_path,
                suggested_fix=f"Create {self.manifest_filename} at the Robot Pack root.",
            )
        if not manifest_path.is_file():
            raise self._error(
                code="document.expected_file",
                message="Robot Pack manifest is not a regular file",
                document=self.manifest_filename,
                source=manifest_path,
            )

        manifest_data = self._load_mapping(manifest_path, root)
        manifest = self._validate_model(
            RobotPackManifest,
            manifest_data,
            document=self.manifest_filename,
            source=manifest_path,
        )

        modules_path = self._resolve_document(root, manifest.specs.modules)
        connectors_path = self._resolve_document(root, manifest.specs.connectors)
        capabilities_path = self._resolve_document(root, manifest.specs.capabilities)

        modules_data = self._inject_catalog_ids(
            self._load_mapping(modules_path, root),
            catalog_key="module_types",
            document=manifest.specs.modules,
            source=modules_path,
        )
        connector_data = self._inject_catalog_ids(
            self._load_mapping(connectors_path, root),
            catalog_key="connector_types",
            document=manifest.specs.connectors,
            source=connectors_path,
        )
        capabilities_data = self._inject_catalog_ids(
            self._load_mapping(capabilities_path, root),
            catalog_key="capabilities",
            document=manifest.specs.capabilities,
            source=capabilities_path,
        )

        module_document = self._validate_model(
            ModuleTypesDocument,
            modules_data,
            document=manifest.specs.modules,
            source=modules_path,
        )
        connector_document = self._validate_model(
            ConnectorTypesDocument,
            connector_data,
            document=manifest.specs.connectors,
            source=connectors_path,
        )
        capability_document = self._validate_model(
            CapabilitiesDocument,
            capabilities_data,
            document=manifest.specs.capabilities,
            source=capabilities_path,
        )

        backend_mappings: dict[str, BackendMapping] = {}
        for backend_id, relative_path in manifest.mappings.items():
            mapping_path = self._resolve_document(root, relative_path)
            mapping = self._validate_model(
                BackendMapping,
                self._load_mapping(mapping_path, root),
                document=relative_path,
                source=mapping_path,
            )
            if mapping.backend != backend_id:
                raise self._error(
                    code="schema.invalid",
                    message=(
                        f"mapping key '{backend_id}' does not match document backend "
                        f"'{mapping.backend}'"
                    ),
                    document=relative_path,
                    path=("backend",),
                    source=mapping_path,
                )
            backend_mappings[backend_id] = mapping

        pack = RobotPack(
            manifest=manifest,
            hardware_catalog=HardwareCatalog(
                module_types=module_document.module_types,
                connector_types=connector_document.connector_types,
            ),
            capability_catalog=CapabilityCatalog(capabilities=capability_document.capabilities),
            backend_mappings=backend_mappings,
        )
        return LoadedRobotPack(root=root, manifest_path=manifest_path, pack=pack)

    def _load_mapping(self, path: Path, root: Path) -> dict[str, object]:
        document = self._relative_document(path, root)
        yaml = YAML(typ="safe", pure=True)
        yaml.allow_duplicate_keys = False
        try:
            with path.open("r", encoding="utf-8") as stream:
                raw_data = cast(
                    object,
                    yaml.load(stream),  # pyright: ignore[reportUnknownMemberType]
                )
        except OSError as error:
            raise self._error(
                code="yaml.read_error",
                message=str(error),
                document=document,
                source=path,
            ) from error
        except DuplicateKeyError as error:
            raise self._error(
                code="yaml.duplicate_key",
                message=str(error),
                document=document,
                source=path,
            ) from error
        except YAMLError as error:
            raise self._error(
                code="yaml.parse_error",
                message=str(error),
                document=document,
                source=path,
            ) from error

        if not isinstance(raw_data, Mapping):
            raise self._error(
                code="schema.invalid",
                message="YAML document root must be a mapping",
                document=document,
                source=path,
            )
        raw_mapping = cast(Mapping[object, object], raw_data)
        data: dict[str, object] = {str(key): value for key, value in raw_mapping.items()}
        version = data.get("schema_version")
        if version != SCHEMA_VERSION:
            raise self._error(
                code="schema.version_unsupported",
                message=(f"expected schema_version '{SCHEMA_VERSION}', got {version!r}"),
                document=document,
                path=("schema_version",),
                source=path,
                suggested_fix=f"Set schema_version to '{SCHEMA_VERSION}'.",
            )
        return data

    def _resolve_document(self, root: Path, relative_path: str) -> Path:
        unresolved_candidate = root / relative_path
        cursor = root
        for component in Path(relative_path).parts:
            cursor /= component
            if cursor.is_symlink():
                raise self._error(
                    code="document.symlink_not_allowed",
                    message="referenced YAML documents must not use symbolic links",
                    document=relative_path,
                    source=cursor,
                )
            if not cursor.exists():
                break
        try:
            candidate = unresolved_candidate.resolve(strict=False)
        except (OSError, RuntimeError, ValueError) as error:
            raise self._error(
                code="path.invalid",
                message=f"cannot resolve referenced YAML document: {error}",
                document=relative_path,
                source=unresolved_candidate,
            ) from error
        if not self._is_within(candidate, root):
            raise self._error(
                code="path.outside_pack",
                message="referenced document resolves outside the Robot Pack",
                document=relative_path,
                source=candidate,
            )
        if not candidate.exists():
            raise self._error(
                code="document.missing",
                message="referenced YAML document does not exist",
                document=relative_path,
                source=candidate,
            )
        if not candidate.is_file():
            raise self._error(
                code="document.expected_file",
                message="referenced YAML document is not a regular file",
                document=relative_path,
                source=candidate,
            )
        return candidate

    def _inject_catalog_ids(
        self,
        data: dict[str, object],
        *,
        catalog_key: str,
        document: str,
        source: Path,
    ) -> dict[str, object]:
        raw_catalog = data.get(catalog_key)
        if not isinstance(raw_catalog, Mapping):
            return data

        raw_catalog_mapping = cast(Mapping[object, object], raw_catalog)
        catalog: dict[str, object] = {}
        for raw_key, raw_value in raw_catalog_mapping.items():
            item_id = str(raw_key)
            if not isinstance(raw_value, Mapping):
                catalog[item_id] = raw_value
                continue
            raw_item_mapping = cast(Mapping[object, object], raw_value)
            item: dict[str, object] = {str(key): value for key, value in raw_item_mapping.items()}
            explicit_id = item.get("id")
            if "id" in item and explicit_id != item_id:
                raise self._error(
                    code="schema.invalid",
                    message=(f"catalog key '{item_id}' does not match explicit id '{explicit_id}'"),
                    document=document,
                    path=(catalog_key, item_id, "id"),
                    source=source,
                )
            item["id"] = item_id
            catalog[item_id] = item
        return {**data, catalog_key: catalog}

    def _validate_model(
        self,
        model_type: type[ModelT],
        data: dict[str, object],
        *,
        document: str,
        source: Path,
    ) -> ModelT:
        try:
            return model_type.model_validate(data)
        except ValidationError as error:
            issues = tuple(
                ValidationIssue(
                    severity=Severity.ERROR,
                    code="schema.invalid",
                    message=str(item["msg"]),
                    document=document,
                    path=tuple(item["loc"]),
                )
                for item in error.errors(
                    include_url=False,
                    include_context=False,
                    include_input=False,
                )
            )
            raise RobotPackLoadError(issues, source=source) from error

    @staticmethod
    def _relative_document(path: Path, root: Path) -> str:
        try:
            return path.relative_to(root).as_posix()
        except ValueError:
            return path.name

    @staticmethod
    def _is_within(path: Path, root: Path) -> bool:
        try:
            path.relative_to(root)
        except ValueError:
            return False
        return True

    @staticmethod
    def _error(
        *,
        code: str,
        message: str,
        document: str,
        source: Path,
        path: tuple[str | int, ...] = (),
        suggested_fix: str | None = None,
    ) -> RobotPackLoadError:
        return RobotPackLoadError(
            ValidationIssue(
                severity=Severity.ERROR,
                code=code,
                message=message,
                document=document,
                path=path,
                suggested_fix=suggested_fix,
            ),
            source=source,
        )


def load_robot_pack(path: str | Path) -> LoadedRobotPack:
    """Load a Robot Pack using the default loader."""
    return RobotPackLoader().load(path)
