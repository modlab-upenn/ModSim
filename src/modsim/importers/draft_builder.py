"""Create a strict draft Robot Pack from an imported URDF."""

from __future__ import annotations

import os
import re
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from xml.etree import ElementTree

from modsim.importers.urdf import ImportedJoint, ImportedRobotAsset, URDFImporter
from modsim.robot_packs import (
    AssetCatalogKind,
    AssetManifest,
    BackendMapping,
    CapabilityCatalog,
    HardwareCatalog,
    JointLimits,
    JointSpec,
    JointType,
    LoadedRobotPack,
    ModuleBackendMapping,
    ModuleType,
    RobotPack,
    RobotPackManifest,
    RobotPackWriter,
    SpecFileManifest,
)


@dataclass(frozen=True, slots=True)
class DraftBuildResult:
    """A generated Robot Pack plus the mechanical import that produced it."""

    loaded: LoadedRobotPack
    imported_asset: ImportedRobotAsset
    warnings: tuple[str, ...]


class DraftPackBuilder:
    """Build a new, non-overwriting Robot Pack folder from one URDF."""

    def __init__(self, *, importer: URDFImporter | None = None) -> None:
        self._importer = importer or URDFImporter()

    def build(
        self,
        urdf_path: str | Path,
        destination: str | Path,
        *,
        asset_roots: tuple[str | Path, ...] = (),
        pack_id: str | None = None,
        name: str | None = None,
        version: str = "0.1.0",
    ) -> DraftBuildResult:
        """Import a URDF and atomically write a new draft Robot Pack."""
        imported = self._importer.load(urdf_path, asset_roots=asset_roots)
        destination_path = Path(os.path.abspath(Path(destination)))
        if destination_path.exists() or destination_path.is_symlink():
            raise ValueError(f"draft destination already exists: {destination_path}")
        destination_path.parent.mkdir(parents=True, exist_ok=True)

        identifier = self._identifier(pack_id or imported.name)
        display_name = (name or imported.name).strip()
        if not display_name:
            display_name = identifier.replace("_", " ").title()
        warnings = list(imported.warnings)
        joint_specs, joint_id_by_source = self._joint_specs(imported, warnings)
        mass_values = [link.mass_kg for link in imported.links if link.mass_kg is not None]
        module_mass = float(sum(mass_values)) if mass_values else None
        root_link = imported.root_links[0]

        with tempfile.TemporaryDirectory(
            prefix=f".{destination_path.name}.source.",
            dir=destination_path.parent,
        ) as staging_name:
            staging_root = Path(staging_name)
            asset_manifest = self._stage_assets(
                imported,
                staging_root=staging_root,
                asset_roots=tuple(Path(os.path.abspath(Path(root))) for root in asset_roots),
                warnings=warnings,
                asset_id=identifier,
            )
            module = ModuleType(
                id=identifier,
                name=display_name,
                asset_ref=identifier,
                root_link=root_link,
                mass_kg=module_mass,
                joints=joint_specs,
                connectors=(),
                capabilities=(),
            )
            mapping = BackendMapping(
                backend="urdf",
                asset_catalog=AssetCatalogKind.URDF,
                module_types={
                    identifier: ModuleBackendMapping(
                        asset_ref=identifier,
                        link_map={link.name: link.name for link in imported.links},
                        joint_map={
                            joint_id_by_source[joint.name]: joint.name for joint in imported.joints
                        },
                    )
                },
            )
            pack = RobotPack(
                manifest=RobotPackManifest(
                    id=identifier,
                    name=display_name,
                    version=version,
                    description=(
                        "Draft generated from URDF. Add connector and capability semantics "
                        "in ModSim Studio."
                    ),
                    assets=asset_manifest,
                    specs=SpecFileManifest(
                        modules="specs/module_types.yaml",
                        connectors="specs/connector_types.yaml",
                        capabilities="specs/capabilities.yaml",
                    ),
                    mappings={"urdf": "mappings/urdf_mapping.yaml"},
                ),
                hardware_catalog=HardwareCatalog(
                    module_types={identifier: module},
                    connector_types={},
                ),
                capability_catalog=CapabilityCatalog(),
                backend_mappings={"urdf": mapping},
            )
            staged = LoadedRobotPack(
                root=staging_root,
                manifest_path=staging_root / "robot_pack.yaml",
                pack=pack,
            )
            loaded = RobotPackWriter().write(staged, destination_path)
        reloaded_asset = self._importer.load(
            loaded.root / loaded.pack.manifest.assets.urdf[identifier]
        )
        return DraftBuildResult(
            loaded=loaded,
            imported_asset=reloaded_asset,
            warnings=tuple(warnings),
        )

    def _stage_assets(
        self,
        imported: ImportedRobotAsset,
        *,
        staging_root: Path,
        asset_roots: tuple[Path, ...],
        warnings: list[str],
        asset_id: str,
    ) -> AssetManifest:
        urdf_relative = PurePosixPath("assets", "urdf", f"{asset_id}.urdf")
        urdf_target = staging_root / Path(urdf_relative)
        mesh_root = staging_root / "assets" / "meshes"
        urdf_target.parent.mkdir(parents=True, exist_ok=True)

        mesh_sources = {
            visual.geometry.mesh_filename: visual.geometry.resolved_mesh_path
            for link in imported.links
            for visual in (*link.visuals, *link.collisions)
            if visual.geometry.mesh_filename is not None
        }
        unresolved = sorted(filename for filename, source in mesh_sources.items() if source is None)
        if unresolved:
            raise ValueError(
                "cannot create a self-contained Robot Pack; unresolved mesh references: "
                + ", ".join(unresolved)
            )

        rewrite_paths: dict[str, str] = {}
        used_relative_paths: dict[PurePosixPath, Path] = {}
        for filename, optional_source in sorted(mesh_sources.items()):
            if optional_source is None:
                continue
            source = optional_source
            relative = self._mesh_relative_path(
                source,
                imported=imported,
                asset_roots=asset_roots,
            )
            relative = self._deduplicate_relative_path(
                relative,
                source=source,
                used=used_relative_paths,
            )
            target = mesh_root / Path(relative)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            rewrite_paths[filename] = (PurePosixPath("..", "meshes") / relative).as_posix()
            used_relative_paths[relative] = source
            if source.suffix.lower() in {".obj", ".dae"}:
                warnings.append(
                    f"Imported {source.name}; verify external material and texture references "
                    "in Studio."
                )

        self._rewrite_urdf(imported.source_path, urdf_target, rewrite_paths)
        mesh_directories = {"meshes": "assets/meshes"} if used_relative_paths else {}
        return AssetManifest(
            urdf={asset_id: urdf_relative.as_posix()},
            mesh_directories=mesh_directories,
        )

    @staticmethod
    def _rewrite_urdf(
        source: Path,
        destination: Path,
        rewrite_paths: dict[str, str],
    ) -> None:
        try:
            tree = ElementTree.parse(source)
        except (OSError, ElementTree.ParseError) as error:
            raise ValueError(f"cannot rewrite imported URDF: {error}") from error
        for element in tree.iter():
            if element.tag.rsplit("}", maxsplit=1)[-1] != "mesh":
                continue
            filename = element.attrib.get("filename")
            if filename in rewrite_paths:
                element.set("filename", rewrite_paths[filename])
        tree.write(destination, encoding="utf-8", xml_declaration=True)

    @classmethod
    def _joint_specs(
        cls,
        imported: ImportedRobotAsset,
        warnings: list[str],
    ) -> tuple[tuple[JointSpec, ...], dict[str, str]]:
        used_ids: set[str] = set()
        result: list[JointSpec] = []
        ids: dict[str, str] = {}
        for joint in imported.joints:
            joint_id = cls._unique_identifier(joint.name, used_ids)
            used_ids.add(joint_id)
            ids[joint.name] = joint_id
            limits = cls._joint_limits(joint, warnings)
            result.append(
                JointSpec(
                    id=joint_id,
                    source_joint_name=joint.name,
                    type=JointType(joint.type),
                    parent_link=joint.parent_link,
                    child_link=joint.child_link,
                    axis=joint.axis,
                    limits=limits,
                )
            )
        return tuple(result), ids

    @staticmethod
    def _joint_limits(
        joint: ImportedJoint,
        warnings: list[str],
    ) -> JointLimits | None:
        source = joint.limits
        if source is None or joint.type in {"fixed", "floating", "planar"}:
            return None
        velocity = source.velocity
        effort = source.effort
        if velocity is not None and velocity <= 0.0:
            warnings.append(f"Joint '{joint.name}' has non-positive velocity; it was left unknown.")
            velocity = None
        if effort is not None and effort <= 0.0:
            warnings.append(f"Joint '{joint.name}' has non-positive effort; it was left unknown.")
            effort = None
        if joint.type in {"revolute", "continuous"}:
            return JointLimits(
                lower_position_rad=(source.lower if joint.type == "revolute" else None),
                upper_position_rad=(source.upper if joint.type == "revolute" else None),
                max_velocity_rad_per_s=velocity,
                max_effort_nm=effort,
            )
        if joint.type == "prismatic":
            return JointLimits(
                lower_position_m=source.lower,
                upper_position_m=source.upper,
                max_velocity_m_per_s=velocity,
                max_effort_n=effort,
            )
        return None

    @classmethod
    def _mesh_relative_path(
        cls,
        source: Path,
        *,
        imported: ImportedRobotAsset,
        asset_roots: tuple[Path, ...],
    ) -> PurePosixPath:
        roots = (*asset_roots, imported.source_path.parent)
        for root in roots:
            try:
                relative = source.relative_to(root.resolve(strict=False))
            except ValueError:
                continue
            if ".." not in relative.parts and not relative.is_absolute():
                return PurePosixPath(*relative.parts)
        return PurePosixPath(source.name)

    @staticmethod
    def _deduplicate_relative_path(
        candidate: PurePosixPath,
        *,
        source: Path,
        used: dict[PurePosixPath, Path],
    ) -> PurePosixPath:
        existing = used.get(candidate)
        if existing is None or existing == source:
            return candidate
        index = 2
        while True:
            renamed = candidate.with_name(f"{candidate.stem}_{index}{candidate.suffix}")
            if renamed not in used:
                return renamed
            index += 1

    @classmethod
    def _unique_identifier(cls, value: str, used: set[str]) -> str:
        base = cls._identifier(value)
        if base not in used:
            return base
        index = 2
        while f"{base}_{index}" in used:
            index += 1
        return f"{base}_{index}"

    @staticmethod
    def _identifier(value: str) -> str:
        normalized = re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")
        if not normalized:
            normalized = "robot_module"
        if not normalized[0].isalpha():
            normalized = f"robot_{normalized}"
        return normalized[:64].rstrip("_")
