"""Explicit initialization of per-user ModSim data and the bundled starter pack.

Importing ModSim and reading command help never create user directories. The
``setup`` command copies the small starter pack into an ordinary, self-contained
directory that the existing Robot Pack loader and backends can use.
"""

from __future__ import annotations

import errno
import os
import tempfile
import tomllib
from dataclasses import dataclass
from importlib.resources import files
from importlib.resources.abc import Traversable
from pathlib import Path
from typing import cast

from ruamel.yaml import YAML
from ruamel.yaml.error import YAMLError

from modsim_robotics.robot_packs import RobotPackLoader, RobotPackLoadError, RobotPackValidator
from modsim_robotics.robot_packs.issues import RobotPackValidationError
from modsim_robotics.robot_packs.schema import RobotPackManifest
from modsim_robotics.robot_packs.validator import ValidationProfile

MODSIM_PATH_ENV_VAR = "MODSIM_PATH"
MODSIM_HOME_ENV_VAR = "MODSIM_HOME"
STARTER_PACK_ID = "generic_cube"
STARTER_PACK_VERSION = "0.1.0"
_STARTER_PACK_LIBRARY_PATH = Path("modlab") / STARTER_PACK_ID / STARTER_PACK_VERSION


class UserDataError(RuntimeError):
    """Raised when user storage or a Robot Pack selection cannot be used."""


@dataclass(frozen=True)
class InstalledRobotPack:
    """Manifest identity and local directory of one installed Robot Pack."""

    id: str
    version: str
    path: Path

    def as_dict(self) -> dict[str, str]:
        """Return the JSON-safe identity and location used by pack listings."""
        return {"id": self.id, "version": self.version, "path": str(self.path)}


@dataclass(frozen=True)
class SetupResult:
    """Paths prepared by setup, and whether it installed a new starter pack."""

    home: Path
    starter_pack: Path
    created: bool

    def as_dict(self) -> dict[str, str | bool]:
        """Return the JSON-safe setup result used by the CLI."""
        return {
            "home": str(self.home),
            "starter_pack": str(self.starter_pack),
            "created": self.created,
        }


def resolve_user_home(home: str | Path | None = None) -> Path:
    """Resolve a home, MODSIM_PATH, legacy MODSIM_HOME, or ~/.modsim without writes."""
    if home is None:
        for variable in (MODSIM_PATH_ENV_VAR, MODSIM_HOME_ENV_VAR):
            configured = os.environ.get(variable)
            if configured is not None:
                if not configured.strip():
                    raise UserDataError(f"{variable} must name a directory, not an empty value.")
                home = configured
                break
        else:
            home = Path.home() / ".modsim"
    return Path(home).expanduser().resolve(strict=False)


def resolve_robot_pack(
    pack_path: str | Path | None = None,
    *,
    home: str | Path | None = None,
) -> Path:
    """Select a library ID, ID@VERSION, or explicit path without creating files.

    Path objects and path-like strings bypass the library and retain symlink
    identity for loader validation. A bare name must identify one installed
    location; neither a default robot nor a latest version is selected.
    """
    if pack_path is None:
        raise UserDataError(
            "Select a Robot Pack by name (ID or ID@VERSION) or an explicit path. "
            "Use 'modsim-robotics pack list' to see installed packs."
        )
    if isinstance(pack_path, Path) or (
        pack_path in {".", ".."}
        or pack_path.startswith(("./", "../", "~"))
        or Path(pack_path).is_absolute()
        or "/" in pack_path
        or "\\" in pack_path
    ):
        return Path(pack_path).expanduser().absolute()
    pack_id, separator, version = pack_path.partition("@")
    if not pack_id or (separator and (not version or "@" in version)):
        raise UserDataError("Select a Robot Pack with ID, ID@VERSION, or an explicit path.")
    matches = [
        pack
        for pack in list_robot_packs(home=home)
        if pack.id == pack_id and (not separator or pack.version == version)
    ]
    if not matches:
        raise UserDataError(
            f"No installed Robot Pack matches '{pack_path}'. "
            "Use 'modsim-robotics pack list' or supply an explicit path."
        )
    if len(matches) > 1:
        choices = "\n".join(f"  {pack.id}@{pack.version}: {pack.path}" for pack in matches)
        hint = "Use an explicit path." if separator else "Use ID@VERSION or an explicit path."
        raise UserDataError(f"Robot Pack '{pack_path}' is ambiguous. {hint}\n{choices}")
    return matches[0].path


def list_robot_packs(*, home: str | Path | None = None) -> tuple[InstalledRobotPack, ...]:
    """List safe manifest identities without loading specs or mechanical assets."""
    try:
        root = resolve_user_home(home)
        library = _robot_pack_library(root)
        if not library.exists():
            raise UserDataError(
                f"Robot Pack library '{library}' does not exist. "
                "Run 'modsim-robotics setup' for the selected home first."
            )
        if not library.is_dir():
            raise UserDataError(f"Robot Pack library '{library}' must be a directory.")
        installed: list[InstalledRobotPack] = []
        for directory, subdirectories, _files in os.walk(library, followlinks=False):
            pack_root = Path(directory)
            subdirectories[:] = sorted(
                name
                for name in subdirectories
                if not name.startswith(".") and not (pack_root / name).is_symlink()
            )
            manifest = pack_root / "robot_pack.yaml"
            if manifest.exists() or manifest.is_symlink():
                # A manifest marks a pack boundary even when it is unreadable or invalid.
                subdirectories.clear()
                pack = _read_installed_pack(pack_root)
                if pack is not None:
                    installed.append(pack)
        return tuple(
            sorted(installed, key=lambda pack: (pack.id, pack.version, pack.path.as_posix()))
        )
    except (OSError, RuntimeError, ValueError) as error:
        if isinstance(error, UserDataError):
            raise
        raise UserDataError(f"Cannot list installed Robot Packs: {error}") from error


def _read_installed_pack(root: Path) -> InstalledRobotPack | None:
    manifest = root / "robot_pack.yaml"
    try:
        if root.is_symlink() or manifest.is_symlink() or not manifest.is_file():
            return None
        yaml = YAML(typ="safe", pure=True)
        yaml.allow_duplicate_keys = False
        with manifest.open(encoding="utf-8") as stream:
            document = cast(
                object,
                yaml.load(stream),  # pyright: ignore[reportUnknownMemberType]
            )
        metadata = RobotPackManifest.model_validate(document)
        return InstalledRobotPack(metadata.id, metadata.version, root)
    except (OSError, ValueError, YAMLError):
        # Broken unrelated packs must not prevent discovery of valid identities.
        return None


def _robot_pack_library(root: Path) -> Path:
    config = _managed_path(root, "config.toml")
    try:
        with config.open("rb") as stream:
            settings: dict[str, object] = tomllib.load(stream)
    except FileNotFoundError as error:
        raise UserDataError(
            f"ModSim is not set up at '{root}': '{config}' is missing. "
            "Run 'modsim-robotics setup' for the selected home first."
        ) from error
    except (OSError, ValueError) as error:
        raise UserDataError(f"Cannot read ModSim configuration '{config}': {error}") from error
    version = settings.get("schema_version")
    if type(version) is not int or version != 1:
        raise UserDataError(f"Configuration '{config}' requires schema_version = 1.")
    paths = settings.get("paths", {})
    if not isinstance(paths, dict):
        raise UserDataError(f"Configuration '{config}' requires [paths] to be a TOML table.")
    configured = cast(dict[str, object], paths).get("robot_packs", "robot_packs")
    if not isinstance(configured, str) or not configured.strip():
        raise UserDataError(
            f"Configuration '{config}' requires paths.robot_packs to be a non-empty path string."
        )
    selected = Path(configured).expanduser()
    if not selected.is_absolute():
        return _managed_path(root, selected).absolute()
    if selected.is_relative_to(root):
        return _managed_path(root, selected.relative_to(root)).absolute()
    if selected.is_symlink():
        raise UserDataError(f"Refusing symbolic-link storage path: '{selected}'.")
    return selected.absolute()


def initialize_user_home(home: str | Path | None = None) -> SetupResult:
    """Prepare user storage and install Generic Cube without overwriting a pack.

    Existing config and pack files are preserved. A pack is first copied and
    validated in a sibling staging directory, then published with a rename.
    No network access or optional simulator dependency is needed.
    """
    try:
        root = resolve_user_home(home)
        root.mkdir(parents=True, exist_ok=True)
        for relative in ("catalog", "cache/downloads", "workspaces"):
            _managed_path(root, relative).mkdir(parents=True, exist_ok=True)
        config = _managed_path(root, "config.toml")
        try:
            with config.open("x", encoding="utf-8") as stream:
                stream.write(
                    "# ModSim user data layout\nschema_version = 1\n\n"
                    "[paths]\nrobot_packs = 'robot_packs'\n"
                )
        except FileExistsError:
            if not config.is_file():
                raise UserDataError(f"Expected a configuration file at '{config}'.") from None
        library = _robot_pack_library(root)
        library.mkdir(parents=True, exist_ok=True)
        relative_pack = _STARTER_PACK_LIBRARY_PATH
        destination = _managed_path(library, relative_pack)
        if destination.exists():
            _validate_starter(destination)
            return SetupResult(root, destination, created=False)

        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix=".generic_cube-", dir=destination.parent
        ) as temporary:
            staged = Path(temporary) / STARTER_PACK_ID
            _copy_resource_tree(_starter_source(), staged)
            _validate_starter(staged)
            _managed_path(library, relative_pack)
            if destination.exists():
                _validate_starter(destination)
                return SetupResult(root, destination, created=False)
            try:
                staged.rename(destination)
            except OSError as error:
                if error.errno not in (errno.EEXIST, errno.ENOTEMPTY):
                    raise
                # Another setup may have completed while this copy was staged.
                _managed_path(library, relative_pack)
                _validate_starter(destination)
                return SetupResult(root, destination, created=False)
        return SetupResult(root, destination, created=True)
    except (OSError, RuntimeError, ValueError) as error:
        if isinstance(error, UserDataError):
            raise
        raise UserDataError(f"Cannot initialize ModSim user data: {error}") from error


def _managed_path(root: Path, relative: str | Path) -> Path:
    """Reject symlinks in each managed descendant of the resolved home."""
    path = root
    for part in Path(relative).parts:
        path /= part
        if path.is_symlink():
            raise UserDataError(f"Refusing symbolic-link storage path: '{path}'.")
    return path


def _starter_source() -> Traversable:
    bundled = files("modsim_robotics").joinpath("data", "robot_packs", STARTER_PACK_ID)
    if bundled.is_dir():
        return bundled
    # Hatch includes the authoritative example as wheel data. An editable source
    # checkout reads that same example directly, without a second tracked copy.
    source = Path(__file__).resolve().parents[2] / "examples" / "robot_packs" / STARTER_PACK_ID
    if source.is_dir():
        return source
    raise UserDataError("The bundled Generic Cube pack is missing. Reinstall ModSim from a wheel.")


def _copy_resource_tree(source: Traversable, target: Path) -> None:
    if isinstance(source, Path) and source.is_symlink():
        raise UserDataError(f"Refusing symbolic link in the bundled starter pack: '{source}'.")
    if source.is_dir():
        target.mkdir()
        for child in sorted(source.iterdir(), key=lambda entry: entry.name):
            _copy_resource_tree(child, target / child.name)
    elif source.is_file():
        target.write_bytes(source.read_bytes())
    else:
        raise UserDataError(f"The bundled starter pack contains an unsupported entry: '{source}'.")


def _validate_starter(root: Path) -> None:
    try:
        loaded = RobotPackLoader().load(root)
        if loaded.pack.id != STARTER_PACK_ID or loaded.pack.version != STARTER_PACK_VERSION:
            raise UserDataError(
                f"Starter pack '{root}' must be {STARTER_PACK_ID}@{STARTER_PACK_VERSION}. "
                "Move your edited pack into a workspace before running setup again."
            )
        RobotPackValidator().validate(
            loaded, profile=ValidationProfile.SIMULATION
        ).raise_for_errors()
    except (RobotPackLoadError, RobotPackValidationError) as error:
        raise UserDataError(
            f"Starter pack '{root}' is invalid and was not replaced: {error}. "
            "Move it into a workspace or choose another --home directory."
        ) from error
