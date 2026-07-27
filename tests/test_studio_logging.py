"""Focused tests for ModSim Studio session logging."""

from __future__ import annotations

import logging
import re
from collections.abc import Iterator
from io import StringIO
from pathlib import Path

import pytest

from modsim_studio.project import StudioProject
from modsim_studio.session_logging import (
    LOGGER_NAME,
    configure_session_logging,
    resolve_session_log_path,
    shutdown_session_logging,
)


@pytest.fixture(autouse=True)
def isolate_session_logger() -> Iterator[None]:
    """Remove session handlers and restore shared logger settings after each test."""
    logger = logging.getLogger(LOGGER_NAME)
    original_level = logger.level
    original_disabled = logger.disabled
    original_propagate = logger.propagate
    shutdown_session_logging()
    yield
    shutdown_session_logging()
    logger.setLevel(original_level)
    logger.disabled = original_disabled
    logger.propagate = original_propagate


def test_default_path_uses_initial_pack_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("MODSIM_STUDIO_LOG", raising=False)
    pack_directory = tmp_path / "robot_pack"
    pack_directory.mkdir()

    assert resolve_session_log_path(pack_directory) == (
        pack_directory.parent / ".modsim" / "logs" / pack_directory.name / "studio.log"
    )


def test_manifest_initial_path_uses_its_parent(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("MODSIM_STUDIO_LOG", raising=False)
    manifest = tmp_path / "robot_pack" / "robot_pack.yaml"
    manifest.parent.mkdir()
    manifest.write_text("id: test\n", encoding="utf-8")

    assert resolve_session_log_path(manifest) == (
        manifest.parent.parent / ".modsim" / "logs" / manifest.parent.name / "studio.log"
    )


def test_launch_without_initial_pack_uses_current_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("MODSIM_STUDIO_LOG", raising=False)
    monkeypatch.chdir(tmp_path)

    assert resolve_session_log_path() == tmp_path / ".modsim" / "logs" / "studio.log"


def test_environment_override_takes_precedence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("MODSIM_STUDIO_LOG", "custom/session.log")

    assert resolve_session_log_path(tmp_path / "ignored_pack") == (
        tmp_path / "custom" / "session.log"
    )


def test_configuration_truncates_and_formats_debug_log(tmp_path: Path) -> None:
    pack_path = tmp_path / "robot_pack"
    pack_path.mkdir()
    log_path = tmp_path / ".modsim" / "logs" / "robot_pack" / "studio.log"
    log_path.parent.mkdir(parents=True)
    log_path.write_text("previous launch\n", encoding="utf-8")

    logger = configure_session_logging(pack_path)

    assert logger.name == LOGGER_NAME
    assert logger.level == logging.DEBUG
    assert not logger.disabled
    assert not logger.propagate
    assert log_path.read_text(encoding="utf-8") == ""

    logger.debug("session ready")
    shutdown_session_logging()

    line = log_path.read_text(encoding="utf-8")
    assert re.fullmatch(
        r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2} DEBUG modsim session ready\n",
        line,
    )


def test_repeated_configuration_reuses_handler_without_truncation(tmp_path: Path) -> None:
    pack_path = tmp_path / "robot_pack"
    pack_path.mkdir()
    logger = configure_session_logging(pack_path)
    logger.info("before repeated configuration")

    configured_again = configure_session_logging(pack_path)
    configured_again.info("after repeated configuration")
    shutdown_session_logging()

    log_path = tmp_path / ".modsim" / "logs" / "robot_pack" / "studio.log"
    content = log_path.read_text(encoding="utf-8")
    assert configured_again is logger
    assert content.count("before repeated configuration") == 1
    assert content.count("after repeated configuration") == 1


def test_shutdown_closes_only_module_owned_handlers(tmp_path: Path) -> None:
    logger = logging.getLogger(LOGGER_NAME)
    external_stream = StringIO()
    external_handler = logging.StreamHandler(external_stream)
    logger.addHandler(external_handler)
    pack_path = tmp_path / "robot_pack"
    pack_path.mkdir()

    try:
        configure_session_logging(pack_path)
        expected_log_path = tmp_path / ".modsim" / "logs" / "robot_pack" / "studio.log"
        owned_handler = next(
            handler
            for handler in logger.handlers
            if isinstance(handler, logging.FileHandler)
            and Path(handler.baseFilename) == expected_log_path
        )

        shutdown_session_logging()

        assert owned_handler not in logger.handlers
        assert owned_handler.stream is None
        assert external_handler in logger.handlers
        assert external_handler.stream is external_stream

        logger.warning("external handler remains")
        assert "external handler remains" in external_stream.getvalue()
    finally:
        logger.removeHandler(external_handler)
        external_handler.close()


def test_atomic_pack_save_does_not_replace_active_sidecar_log(copied_pack: Path) -> None:
    logger = configure_session_logging(copied_pack)
    log_path = resolve_session_log_path(copied_pack)
    logger.info("before atomic save")

    project = StudioProject.open(copied_pack).update_manifest(
        name="Sidecar Log Save",
        version="0.1.1",
        description=None,
    )
    project.save()
    logger.info("after atomic save")
    shutdown_session_logging()

    content = log_path.read_text(encoding="utf-8")
    assert "before atomic save" in content
    assert "after atomic save" in content
