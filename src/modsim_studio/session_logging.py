"""Per-launch file logging for ModSim Studio."""

from __future__ import annotations

import logging
import os
from pathlib import Path
from threading import RLock

LOGGER_NAME = "modsim"
LOG_PATH_ENV_VAR = "MODSIM_STUDIO_LOG"
DEFAULT_LOG_RELATIVE_PATH = Path(".modsim") / "logs" / "studio.log"
PACK_LOG_DIRECTORY = Path(".modsim") / "logs"

_MANIFEST_SUFFIXES = frozenset({".yaml", ".yml"})
_SESSION_HANDLER_LOCK = RLock()


class _StudioSessionFileHandler(logging.FileHandler):
    """File handler owned exclusively by this module."""


def resolve_session_log_path(initial_path: str | Path | None = None) -> Path:
    """Return the absolute log path for a Studio launch.

    ``MODSIM_STUDIO_LOG`` takes precedence. Otherwise, a pack launch uses a
    sidecar below the pack's parent directory so an atomic pack Save cannot
    replace the active log. When Studio is opened with a manifest file, its
    parent is the pack directory. Launches without an initial pack use the
    current working directory.
    """
    override = os.environ.get(LOG_PATH_ENV_VAR)
    if override:
        return Path(override).expanduser().resolve(strict=False)

    if initial_path is None:
        return (Path.cwd() / DEFAULT_LOG_RELATIVE_PATH).resolve(strict=False)

    candidate = Path(initial_path).expanduser()
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    if candidate.is_file() or candidate.suffix.lower() in _MANIFEST_SUFFIXES:
        pack_directory = candidate.parent
    else:
        pack_directory = candidate
    return (
        pack_directory.parent / PACK_LOG_DIRECTORY / pack_directory.name / "studio.log"
    ).resolve(strict=False)


def configure_session_logging(initial_path: str | Path | None = None) -> logging.Logger:
    """Configure and return the dedicated ``modsim`` session logger.

    The first configuration for a session truncates the selected log. Repeated
    configuration for the same path reuses the live handler instead of adding a
    duplicate or truncating messages already written during that launch.
    """
    log_path = resolve_session_log_path(initial_path)
    logger = logging.getLogger(LOGGER_NAME)

    with _SESSION_HANDLER_LOCK:
        logger.setLevel(logging.DEBUG)
        logger.disabled = False
        logger.propagate = False

        owned_handlers = _owned_handlers(logger)
        matching_handler = next(
            (
                handler
                for handler in owned_handlers
                if handler.stream is not None
                and Path(handler.baseFilename).resolve(strict=False) == log_path
            ),
            None,
        )
        if matching_handler is not None:
            for handler in owned_handlers:
                if handler is not matching_handler:
                    _remove_and_close(logger, handler)
            return logger

        for handler in owned_handlers:
            _remove_and_close(logger, handler)

        log_path.parent.mkdir(parents=True, exist_ok=True)
        handler = _StudioSessionFileHandler(
            log_path,
            mode="w",
            encoding="utf-8",
            delay=False,
        )
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(
            logging.Formatter(
                fmt="%(asctime)s %(levelname)s %(name)s %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            )
        )
        logger.addHandler(handler)

    return logger


def shutdown_session_logging() -> None:
    """Remove and close handlers created by :func:`configure_session_logging`."""
    logger = logging.getLogger(LOGGER_NAME)
    with _SESSION_HANDLER_LOCK:
        for handler in _owned_handlers(logger):
            _remove_and_close(logger, handler)


def _owned_handlers(logger: logging.Logger) -> tuple[_StudioSessionFileHandler, ...]:
    return tuple(
        handler for handler in logger.handlers if isinstance(handler, _StudioSessionFileHandler)
    )


def _remove_and_close(
    logger: logging.Logger,
    handler: _StudioSessionFileHandler,
) -> None:
    logger.removeHandler(handler)
    handler.close()


__all__ = [
    "DEFAULT_LOG_RELATIVE_PATH",
    "LOGGER_NAME",
    "LOG_PATH_ENV_VAR",
    "PACK_LOG_DIRECTORY",
    "configure_session_logging",
    "resolve_session_log_path",
    "shutdown_session_logging",
]
