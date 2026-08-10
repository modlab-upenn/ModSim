"""GUI-independent document model for ModSim Studio."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import cast

from pydantic import JsonValue

from modsim.importers import DraftPackBuilder, ImportedRobotAsset, URDFImporter
from modsim.robot_packs import (
    ConnectorSpec,
    ConnectorTypeSpec,
    JointSpec,
    LoadedRobotPack,
    ModuleType,
    RobotPack,
    RobotPackLoader,
    RobotPackValidator,
    RobotPackWriter,
    ValidationProfile,
    ValidationReport,
)


@dataclass(frozen=True, slots=True)
class StudioProject:
    """An editable in-memory Robot Pack and its imported visual assets."""

    loaded: LoadedRobotPack
    imported_assets: dict[str, ImportedRobotAsset]
    dirty: bool = False

    @classmethod
    def open(cls, path: str | Path) -> StudioProject:
        """Open a Robot Pack and import each declared URDF for visualization."""
        loaded = RobotPackLoader().load(path)
        roots = tuple(
            loaded.root / relative
            for relative in loaded.pack.manifest.assets.mesh_directories.values()
        )
        importer = URDFImporter()
        assets = {
            asset_id: importer.load(loaded.root / relative, asset_roots=roots)
            for asset_id, relative in loaded.pack.manifest.assets.urdf.items()
        }
        return cls(loaded=loaded, imported_assets=assets)

    @classmethod
    def create_from_urdf(
        cls,
        urdf_path: str | Path,
        destination: str | Path,
        *,
        asset_roots: tuple[str | Path, ...] = (),
    ) -> StudioProject:
        """Create, write, and open a new draft Robot Pack."""
        result = DraftPackBuilder().build(
            urdf_path,
            destination,
            asset_roots=asset_roots,
        )
        asset_id = next(iter(result.loaded.pack.manifest.assets.urdf))
        return cls(
            loaded=result.loaded,
            imported_assets={asset_id: result.imported_asset},
        )

    @property
    def pack(self) -> RobotPack:
        """Return the current strict aggregate."""
        return self.loaded.pack

    def validate(self, profile: ValidationProfile) -> ValidationReport:
        """Run structural and semantic validation against the in-memory document."""
        return RobotPackValidator().validate(self.loaded, profile=profile)

    def yaml_preview(self) -> str:
        """Render the canonical split YAML that an export will write."""
        return RobotPackWriter().render_documents(self.loaded)

    def update_manifest(
        self,
        *,
        name: str,
        version: str,
        description: str | None,
        metadata: dict[str, JsonValue] | None = None,
    ) -> StudioProject:
        """Apply editable manifest metadata through strict revalidation."""
        data = self.pack.model_dump(mode="python")
        manifest = data["manifest"]
        if not isinstance(manifest, dict):
            raise TypeError("invalid in-memory manifest")
        typed_manifest = cast(dict[str, object], manifest)
        typed_manifest["name"] = name
        typed_manifest["version"] = version
        typed_manifest["description"] = description or None
        typed_manifest["metadata"] = self.pack.manifest.metadata if metadata is None else metadata
        return self._with_pack(RobotPack.model_validate(data))

    def update_module_metadata(
        self,
        module_id: str,
        name: str | None,
        root_link: str,
        mass_kg: float | None,
    ) -> StudioProject:
        """Update editable module metadata through strict revalidation."""
        module = self._module(module_id)
        updated_module = ModuleType.model_validate(
            {
                **module.model_dump(mode="python"),
                "name": name,
                "root_link": root_link,
                "mass_kg": mass_kg,
            }
        )
        try:
            imported_asset = self.imported_assets[module.asset_ref]
        except KeyError as error:
            raise KeyError(
                f"module type '{module_id}' references unavailable imported URDF "
                f"asset '{module.asset_ref}'"
            ) from error
        if updated_module.root_link not in {link.name for link in imported_asset.links}:
            raise ValueError(
                f"root link '{updated_module.root_link}' is not present in imported "
                f"URDF asset '{module.asset_ref}'"
            )

        data = self.pack.model_dump(mode="python")
        self._replace_module_data(data, module_id, updated_module)
        return self._with_pack(RobotPack.model_validate(data))

    def add_connector(
        self,
        module_id: str,
        connector: ConnectorSpec,
    ) -> StudioProject:
        """Add one connector using an existing type and imported URDF link."""
        if connector.id in {existing.id for existing in self._module(module_id).connectors}:
            raise ValueError(f"connector '{connector.id}' already exists on module '{module_id}'")
        self._require_connector_type(connector.connector_type)
        self._require_module_link(module_id, connector.parent_link)
        module = self._module(module_id)
        updated_module = ModuleType.model_validate(
            {
                **module.model_dump(mode="python"),
                "connectors": (*module.connectors, connector),
            }
        )
        data = self.pack.model_dump(mode="python")
        self._replace_module_data(data, module_id, updated_module)
        return self._with_pack(RobotPack.model_validate(data))

    def update_connector(
        self,
        module_id: str,
        connector: ConnectorSpec,
    ) -> StudioProject:
        """Replace an existing connector by ID."""
        module = self._module(module_id)
        if connector.id not in {existing.id for existing in module.connectors}:
            raise KeyError(connector.id)
        self._require_connector_type(connector.connector_type)
        self._require_module_link(module_id, connector.parent_link)
        updated = tuple(
            connector if existing.id == connector.id else existing for existing in module.connectors
        )
        updated_module = ModuleType.model_validate(
            {**module.model_dump(mode="python"), "connectors": updated}
        )
        data = self.pack.model_dump(mode="python")
        self._replace_module_data(data, module_id, updated_module)
        return self._with_pack(RobotPack.model_validate(data))

    def add_connector_type(self, connector_type: ConnectorTypeSpec) -> StudioProject:
        """Add one explicit reusable connector type definition."""
        if connector_type.id in self.pack.hardware_catalog.connector_types:
            raise ValueError(f"connector type '{connector_type.id}' already exists")
        data = self.pack.model_dump(mode="python")
        connector_types = self._connector_type_data(data)
        connector_types[connector_type.id] = connector_type.model_dump(mode="python")
        return self._with_pack(RobotPack.model_validate(data))

    def update_connector_type(
        self,
        connector_type: ConnectorTypeSpec,
    ) -> StudioProject:
        """Replace a reusable connector type definition by ID."""
        if connector_type.id not in self.pack.hardware_catalog.connector_types:
            raise KeyError(connector_type.id)
        data = self.pack.model_dump(mode="python")
        connector_types = self._connector_type_data(data)
        connector_types[connector_type.id] = connector_type.model_dump(mode="python")
        return self._with_pack(RobotPack.model_validate(data))

    def remove_connector_type(self, connector_type_id: str) -> StudioProject:
        """Remove an unreferenced connector type without leaving dangling YAML references."""
        if connector_type_id not in self.pack.hardware_catalog.connector_types:
            raise KeyError(connector_type_id)
        references = [
            f"module '{module.id}' connector '{connector.id}'"
            for module in self.pack.hardware_catalog.module_types.values()
            for connector in module.connectors
            if connector.connector_type == connector_type_id
        ]
        if references:
            raise ValueError(
                f"connector type '{connector_type_id}' is still referenced by "
                + ", ".join(references)
            )
        data = self.pack.model_dump(mode="python")
        connector_types = self._connector_type_data(data)
        del connector_types[connector_type_id]
        for connector_type in connector_types.values():
            if not isinstance(connector_type, dict):
                continue
            typed_connector_type = cast(dict[str, object], connector_type)
            compatible_with = typed_connector_type.get("compatible_with")
            if isinstance(compatible_with, (list, tuple)):
                typed_compatible = cast(list[object] | tuple[object, ...], compatible_with)
                typed_connector_type["compatible_with"] = tuple(
                    value for value in typed_compatible if value != connector_type_id
                )
        capability_catalog = data.get("capability_catalog")
        if isinstance(capability_catalog, dict):
            typed_catalog = cast(dict[str, object], capability_catalog)
            capabilities = typed_catalog.get("capabilities")
            if isinstance(capabilities, dict):
                typed_capabilities = cast(dict[str, object], capabilities)
                for capability in typed_capabilities.values():
                    if not isinstance(capability, dict):
                        continue
                    typed_capability = cast(dict[str, object], capability)
                    required = typed_capability.get("required_connector_types")
                    if isinstance(required, (list, tuple)):
                        typed_required = cast(list[object] | tuple[object, ...], required)
                        typed_capability["required_connector_types"] = tuple(
                            value for value in typed_required if value != connector_type_id
                        )
        return self._with_pack(RobotPack.model_validate(data))

    def remove_connector(self, module_id: str, connector_id: str) -> StudioProject:
        """Remove an existing connector annotation."""
        module = self._module(module_id)
        if connector_id not in {connector.id for connector in module.connectors}:
            raise KeyError(connector_id)
        updated_module = ModuleType.model_validate(
            {
                **module.model_dump(mode="python"),
                "connectors": tuple(
                    connector for connector in module.connectors if connector.id != connector_id
                ),
            }
        )
        data = self.pack.model_dump(mode="python")
        self._replace_module_data(data, module_id, updated_module)
        backend_mappings = data.get("backend_mappings")
        if isinstance(backend_mappings, dict):
            typed_backend_mappings = cast(dict[str, object], backend_mappings)
            for mapping in typed_backend_mappings.values():
                if not isinstance(mapping, dict):
                    continue
                typed_mapping = cast(dict[str, object], mapping)
                module_mappings = typed_mapping.get("module_types")
                if not isinstance(module_mappings, dict):
                    continue
                typed_module_mappings = cast(dict[str, object], module_mappings)
                module_mapping = typed_module_mappings.get(module_id)
                if not isinstance(module_mapping, dict):
                    continue
                typed_module_mapping = cast(dict[str, object], module_mapping)
                connector_frames = typed_module_mapping.get("connector_frame_map")
                if isinstance(connector_frames, dict):
                    typed_connector_frames = cast(dict[str, object], connector_frames)
                    typed_connector_frames.pop(connector_id, None)
        return self._with_pack(RobotPack.model_validate(data))

    def update_joint(self, module_id: str, joint: JointSpec) -> StudioProject:
        """Replace an imported joint's semantic metadata by ID."""
        module = self._module(module_id)
        if joint.id not in {existing.id for existing in module.joints}:
            raise KeyError(joint.id)
        updated_module = ModuleType.model_validate(
            {
                **module.model_dump(mode="python"),
                "joints": tuple(
                    joint if existing.id == joint.id else existing for existing in module.joints
                ),
            }
        )
        data = self.pack.model_dump(mode="python")
        self._replace_module_data(data, module_id, updated_module)
        return self._with_pack(RobotPack.model_validate(data))

    def export(self, destination: str | Path) -> StudioProject:
        """Export the edited document to a new Robot Pack directory."""
        exported = RobotPackWriter().write(self.loaded, destination)
        return replace(self, loaded=exported, dirty=False)

    def save(self) -> StudioProject:
        """Atomically save edits back to the currently open Robot Pack."""
        saved = RobotPackWriter().update(self.loaded)
        return replace(self, loaded=saved, dirty=False)

    def _module(self, module_id: str) -> ModuleType:
        try:
            return self.pack.hardware_catalog.module_types[module_id]
        except KeyError as error:
            raise KeyError(f"unknown module type '{module_id}'") from error

    def _require_connector_type(self, connector_type_id: str) -> None:
        if connector_type_id not in self.pack.hardware_catalog.connector_types:
            raise ValueError(f"unknown connector type '{connector_type_id}'; create the type first")

    def _require_module_link(self, module_id: str, link_name: str) -> None:
        module = self._module(module_id)
        try:
            imported_asset = self.imported_assets[module.asset_ref]
        except KeyError as error:
            raise KeyError(
                f"module type '{module_id}' references unavailable imported URDF "
                f"asset '{module.asset_ref}'"
            ) from error
        if link_name not in {link.name for link in imported_asset.links}:
            raise ValueError(
                f"parent link '{link_name}' is not present in imported URDF "
                f"asset '{module.asset_ref}'"
            )

    def _with_pack(self, pack: RobotPack) -> StudioProject:
        return replace(
            self,
            loaded=self.loaded.with_pack(pack),
            dirty=True,
        )

    @staticmethod
    def _connector_type_data(pack_data: dict[str, object]) -> dict[str, object]:
        hardware = pack_data["hardware_catalog"]
        if not isinstance(hardware, dict):
            raise TypeError("invalid in-memory hardware catalog")
        typed_hardware = cast(dict[str, object], hardware)
        connector_types = typed_hardware["connector_types"]
        if not isinstance(connector_types, dict):
            raise TypeError("invalid in-memory connector catalog")
        return cast(dict[str, object], connector_types)

    @staticmethod
    def _replace_module_data(
        pack_data: dict[str, object],
        module_id: str,
        module: ModuleType,
    ) -> None:
        hardware = pack_data["hardware_catalog"]
        if not isinstance(hardware, dict):
            raise TypeError("invalid in-memory hardware catalog")
        typed_hardware = cast(dict[str, object], hardware)
        modules = typed_hardware["module_types"]
        if not isinstance(modules, dict):
            raise TypeError("invalid in-memory module catalog")
        typed_modules = cast(dict[str, object], modules)
        typed_modules[module_id] = module.model_dump(mode="python")
