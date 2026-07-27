"""Deterministic, non-destructive Robot Pack writer."""

from __future__ import annotations

import os
import shutil
import stat
import tempfile
from collections.abc import Mapping
from contextlib import suppress
from io import StringIO
from pathlib import Path

from pydantic import BaseModel, ValidationError
from ruamel.yaml import YAML
from ruamel.yaml.nodes import ScalarNode
from ruamel.yaml.representer import RoundTripRepresenter

from modsim.robot_packs.errors import RobotPackWriteError
from modsim.robot_packs.schema import LoadedRobotPack, RobotPack


def _represent_none(representer: RoundTripRepresenter, _: None) -> ScalarNode:
    """Write explicit ``null`` instead of an empty YAML value."""
    return representer.represent_scalar(  # pyright: ignore[reportUnknownMemberType]
        "tag:yaml.org,2002:null", "null"
    )


class RobotPackWriter:
    """Atomically export a loaded Robot Pack to a new directory."""

    manifest_filename = "robot_pack.yaml"

    def document_data(
        self,
        loaded: LoadedRobotPack,
    ) -> tuple[tuple[str, dict[str, object]], ...]:
        """Return the canonical split-document data for previews and tooling."""
        return self._documents(self._revalidated_snapshot(loaded))

    def render_documents(self, loaded: LoadedRobotPack) -> str:
        """Render all canonical YAML documents as one labeled preview."""
        sections: list[str] = []
        for relative_path, data in self.document_data(loaded):
            stream = StringIO()
            yaml = self._configured_yaml()
            yaml.dump(  # pyright: ignore[reportUnknownMemberType]
                dict(data), stream
            )
            sections.append(f"# {relative_path}\n{stream.getvalue().rstrip()}")
        return "\n\n".join(sections) + "\n"

    def write(
        self,
        loaded: LoadedRobotPack,
        destination: str | Path,
    ) -> LoadedRobotPack:
        """Export canonical YAML and assets without modifying an existing path."""
        supplied_destination = Path(os.path.abspath(Path(destination)))
        if supplied_destination.is_symlink():
            raise RobotPackWriteError(
                f"refusing symbolic-link export destination: {supplied_destination}"
            )
        try:
            destination_root = supplied_destination.resolve(strict=False)
        except (OSError, RuntimeError, ValueError) as error:
            raise RobotPackWriteError(
                f"cannot resolve export destination '{supplied_destination}': {error}"
            ) from error
        if destination_root.exists():
            raise RobotPackWriteError(
                "Phase 1 exports only to a new directory; refusing existing destination: "
                f"{destination_root}"
            )

        snapshot = self._revalidated_snapshot(loaded)
        documents = self._documents(snapshot)
        document_targets = [
            (self._safe_target(destination_root, relative_path), data)
            for relative_path, data in documents
        ]
        asset_pairs: list[tuple[Path, Path, str]] = []
        for _, _, relative_path, expected_kind in snapshot.pack.manifest.assets.paths_with_kinds():
            source = self._safe_source(snapshot.root, relative_path)
            self._require_copyable_asset(source, expected_kind=expected_kind)
            target = self._safe_target(destination_root, relative_path)
            asset_pairs.append((source, target, expected_kind))

        self._reject_copy_overlap(
            destination_root,
            asset_pairs,
            tuple(target for target, _ in document_targets),
        )
        destination_root.parent.mkdir(parents=True, exist_ok=True)
        staging_root = Path(
            tempfile.mkdtemp(
                prefix=f".{destination_root.name}.",
                suffix=".tmp",
                dir=destination_root.parent,
            )
        )
        try:
            for source, final_target, _ in asset_pairs:
                relative_path = final_target.relative_to(destination_root)
                self._copy_asset(source, staging_root / relative_path)
            for final_target, data in document_targets:
                relative_path = final_target.relative_to(destination_root)
                self._atomic_dump(staging_root / relative_path, data)

            self._require_copyable_asset(staging_root, expected_kind="directory")
            staged = self._reload_staged_pack(staging_root)
            if staged.pack != snapshot.pack:
                raise RobotPackWriteError(
                    "staged Robot Pack does not match the source semantic model"
                )
            os.replace(staging_root, destination_root)
        except RobotPackWriteError:
            self._remove_staging_directory(staging_root)
            raise
        except OSError as error:
            self._remove_staging_directory(staging_root)
            raise RobotPackWriteError(
                f"failed to export Robot Pack to {destination_root}: {error}"
            ) from error

        return LoadedRobotPack(
            root=destination_root,
            manifest_path=destination_root / self.manifest_filename,
            pack=snapshot.pack,
        )

    def update(self, loaded: LoadedRobotPack) -> LoadedRobotPack:
        """Atomically replace an existing pack after staging a complete verified copy.

        This operation is intended for an explicit editor ``Save`` action. A
        hidden sibling directory temporarily holds the staged pack and the old
        pack so a failed publish can be rolled back.
        """
        source_root = loaded.root.resolve(strict=False)
        if not source_root.is_dir():
            raise RobotPackWriteError(f"cannot update missing Robot Pack directory: {source_root}")
        update_container = Path(
            tempfile.mkdtemp(
                prefix=f".{source_root.name}.update.",
                dir=source_root.parent,
            )
        )
        staged_root = update_container / "staged"
        backup_root = update_container / "backup"
        published = False
        try:
            staged = self.write(loaded, staged_root)
            os.replace(source_root, backup_root)
            try:
                os.replace(staged.root, source_root)
                published = True
            except OSError:
                os.replace(backup_root, source_root)
                raise
            self._remove_staging_directory(backup_root)
            update_container.rmdir()
        except RobotPackWriteError:
            if not published:
                self._remove_staging_directory(update_container)
            raise
        except OSError as error:
            if not published and backup_root.exists() and not source_root.exists():
                with suppress(OSError):
                    os.replace(backup_root, source_root)
            if update_container.exists():
                self._remove_staging_directory(update_container)
            raise RobotPackWriteError(
                f"failed to update Robot Pack at {source_root}: {error}"
            ) from error
        return LoadedRobotPack(
            root=source_root,
            manifest_path=source_root / self.manifest_filename,
            pack=staged.pack,
        )

    @staticmethod
    def _revalidated_snapshot(loaded: LoadedRobotPack) -> LoadedRobotPack:
        try:
            pack = RobotPack.model_validate(loaded.pack.model_dump(mode="python"))
        except ValidationError as error:
            first_error = error.errors(
                include_url=False,
                include_context=False,
                include_input=False,
            )[0]
            location = "/".join(str(part) for part in first_error["loc"])
            raise RobotPackWriteError(
                f"refusing invalid in-memory Robot Pack at {location}: {first_error['msg']}"
            ) from error
        return LoadedRobotPack(
            root=loaded.root.resolve(strict=False),
            manifest_path=loaded.manifest_path.resolve(strict=False),
            pack=pack,
        )

    def _documents(self, loaded: LoadedRobotPack) -> tuple[tuple[str, dict[str, object]], ...]:
        pack = loaded.pack
        manifest = pack.manifest
        manifest_backends = set(manifest.mappings)
        aggregate_backends = set(pack.backend_mappings)
        undeclared_backends = sorted(aggregate_backends - manifest_backends)
        if undeclared_backends:
            raise RobotPackWriteError(
                "aggregate contains backend mappings not declared by the manifest: "
                + ", ".join(undeclared_backends)
            )
        module_types = {
            item_id: self._without_catalog_id(item)
            for item_id, item in pack.hardware_catalog.module_types.items()
        }
        connector_types = {
            item_id: self._without_catalog_id(item)
            for item_id, item in pack.hardware_catalog.connector_types.items()
        }
        capabilities = {
            item_id: self._without_catalog_id(item)
            for item_id, item in pack.capability_catalog.capabilities.items()
        }

        documents: list[tuple[str, dict[str, object]]] = [
            (
                self.manifest_filename,
                self._model_data(manifest),
            ),
            (
                manifest.specs.modules,
                {
                    "schema_version": manifest.schema_version,
                    "module_types": module_types,
                },
            ),
            (
                manifest.specs.connectors,
                {
                    "schema_version": manifest.schema_version,
                    "connector_types": connector_types,
                },
            ),
            (
                manifest.specs.capabilities,
                {
                    "schema_version": manifest.schema_version,
                    "capabilities": capabilities,
                },
            ),
        ]
        for backend_id, relative_path in manifest.mappings.items():
            mapping = pack.backend_mappings.get(backend_id)
            if mapping is None:
                raise RobotPackWriteError(
                    f"manifest references missing backend mapping '{backend_id}'"
                )
            documents.append((relative_path, self._model_data(mapping)))
        return tuple(documents)

    @staticmethod
    def _model_data(model: BaseModel) -> dict[str, object]:
        return model.model_dump(mode="json", exclude_none=False)

    @classmethod
    def _without_catalog_id(cls, model: BaseModel) -> dict[str, object]:
        data = cls._model_data(model)
        data.pop("id", None)
        return data

    @classmethod
    def _safe_source(cls, root: Path, relative_path: str) -> Path:
        root = root.resolve(strict=False)
        unresolved_source = root / relative_path
        cursor = root
        for component in Path(relative_path).parts:
            cursor /= component
            try:
                mode = cursor.lstat().st_mode
            except FileNotFoundError as error:
                raise RobotPackWriteError(
                    f"cannot copy missing Robot Pack asset: {relative_path}"
                ) from error
            except OSError as error:
                raise RobotPackWriteError(
                    f"cannot inspect Robot Pack asset '{relative_path}': {error}"
                ) from error
            if stat.S_ISLNK(mode):
                raise RobotPackWriteError(
                    f"refusing symbolic link in Robot Pack asset path: {relative_path}"
                )
        source = unresolved_source.resolve(strict=False)
        if not cls._is_within(source, root):
            raise RobotPackWriteError(
                f"asset resolves outside the source Robot Pack: {relative_path}"
            )
        return source

    @classmethod
    def _require_copyable_asset(cls, source: Path, *, expected_kind: str) -> None:
        try:
            mode = source.lstat().st_mode
        except OSError as error:
            raise RobotPackWriteError(
                f"cannot inspect Robot Pack asset '{source}': {error}"
            ) from error
        if expected_kind == "file" and not stat.S_ISREG(mode):
            raise RobotPackWriteError(f"asset is not a regular file: {source}")
        if expected_kind == "directory" and not stat.S_ISDIR(mode):
            raise RobotPackWriteError(f"asset is not a directory: {source}")
        cls._inspect_local_tree(source)

    @classmethod
    def _inspect_local_tree(cls, source: Path) -> None:
        try:
            mode = source.lstat().st_mode
        except OSError as error:
            raise RobotPackWriteError(
                f"cannot inspect Robot Pack asset '{source}': {error}"
            ) from error
        if stat.S_ISLNK(mode):
            raise RobotPackWriteError(f"refusing symbolic link inside asset tree: {source}")
        if stat.S_ISREG(mode):
            return
        if not stat.S_ISDIR(mode):
            raise RobotPackWriteError(f"refusing non-regular entry inside asset tree: {source}")
        try:
            with os.scandir(source) as iterator:
                children = sorted(
                    (Path(entry.path) for entry in iterator),
                    key=lambda path: path.name,
                )
        except OSError as error:
            raise RobotPackWriteError(
                f"cannot inspect asset directory '{source}': {error}"
            ) from error
        for child in children:
            cls._inspect_local_tree(child)

    @classmethod
    def _reject_copy_overlap(
        cls,
        destination_root: Path,
        asset_pairs: list[tuple[Path, Path, str]],
        document_targets: tuple[Path, ...],
    ) -> None:
        all_targets = (*document_targets, *(target for _, target, _ in asset_pairs))
        for source, target, expected_kind in asset_pairs:
            if expected_kind != "directory":
                continue
            if cls._is_within(destination_root, source):
                raise RobotPackWriteError(
                    "destination is inside source asset directory; refusing recursive copy: "
                    f"{destination_root}"
                )
            for planned_target in all_targets:
                if cls._is_within(planned_target, source) or cls._is_within(source, planned_target):
                    raise RobotPackWriteError(
                        "source asset directory overlaps a planned output path: "
                        f"{source} and {planned_target}"
                    )
            if cls._is_within(target, source) or cls._is_within(source, target):
                raise RobotPackWriteError(
                    f"source and target asset directories overlap: {source} and {target}"
                )

    @classmethod
    def _safe_target(cls, root: Path, relative_path: str) -> Path:
        target = (root / relative_path).resolve(strict=False)
        if not cls._is_within(target, root):
            raise RobotPackWriteError(
                f"path resolves outside the destination Robot Pack: {relative_path}"
            )
        return target

    @staticmethod
    def _is_within(path: Path, root: Path) -> bool:
        try:
            path.relative_to(root)
        except ValueError:
            return False
        return True

    @classmethod
    def _copy_asset(cls, source: Path, target: Path) -> None:
        try:
            mode = source.lstat().st_mode
        except OSError as error:
            raise RobotPackWriteError(
                f"cannot inspect Robot Pack asset '{source}': {error}"
            ) from error
        if stat.S_ISLNK(mode):
            raise RobotPackWriteError(f"refusing symbolic link inside asset tree: {source}")
        if stat.S_ISREG(mode):
            target.parent.mkdir(parents=True, exist_ok=True)
            try:
                shutil.copy2(source, target, follow_symlinks=False)
            except OSError as error:
                raise RobotPackWriteError(
                    f"failed to copy asset '{source}' to '{target}': {error}"
                ) from error
            return
        if not stat.S_ISDIR(mode):
            raise RobotPackWriteError(f"refusing non-regular entry inside asset tree: {source}")

        target.mkdir(parents=True, exist_ok=False)
        try:
            with os.scandir(source) as iterator:
                children = sorted(
                    (Path(entry.path) for entry in iterator),
                    key=lambda path: path.name,
                )
        except OSError as error:
            raise RobotPackWriteError(f"cannot read asset directory '{source}': {error}") from error
        for child in children:
            cls._copy_asset(child, target / child.name)

    @staticmethod
    def _reload_staged_pack(staging_root: Path) -> LoadedRobotPack:
        from modsim.robot_packs.loader import RobotPackLoader

        try:
            return RobotPackLoader().load(staging_root)
        except Exception as error:
            raise RobotPackWriteError(f"failed to reload staged Robot Pack: {error}") from error

    @staticmethod
    def _remove_staging_directory(staging_root: Path) -> None:
        if staging_root.exists():
            shutil.rmtree(staging_root)

    @staticmethod
    def _atomic_dump(path: Path, data: Mapping[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        yaml = RobotPackWriter._configured_yaml()

        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=path.parent,
                prefix=f".{path.name}.",
                suffix=".tmp",
                delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                yaml.dump(  # pyright: ignore[reportUnknownMemberType]
                    dict(data), stream
                )
                stream.flush()
                os.fsync(stream.fileno())
            temporary_path.replace(path)
        except (OSError, ValueError) as error:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()
            raise RobotPackWriteError(f"failed to write {path}: {error}") from error

    @staticmethod
    def _configured_yaml() -> YAML:
        yaml = YAML(typ="rt")
        yaml.allow_duplicate_keys = False
        yaml.default_flow_style = False
        yaml.preserve_quotes = True
        yaml.width = 100
        yaml.indent(mapping=2, sequence=4, offset=2)
        yaml.representer.add_representer(type(None), _represent_none)
        return yaml


def write_robot_pack(
    loaded: LoadedRobotPack,
    destination: str | Path,
) -> LoadedRobotPack:
    """Write a Robot Pack using the default writer."""
    return RobotPackWriter().write(loaded, destination)


def update_robot_pack(loaded: LoadedRobotPack) -> LoadedRobotPack:
    """Update an existing Robot Pack using the default writer."""
    return RobotPackWriter().update(loaded)
