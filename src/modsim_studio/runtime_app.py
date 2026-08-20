"""Process entry point for the standalone ModSim Runtime Inspector."""

from __future__ import annotations

import logging
import sys
from types import TracebackType

from PySide6.QtWidgets import QApplication

from modsim import __version__
from modsim_studio.runtime_window import RuntimeInspectorWindow
from modsim_studio.runtime_worker import RuntimeInspectorConfig
from modsim_studio.session_logging import (
    configure_session_logging,
    resolve_session_log_path,
    shutdown_session_logging,
)


def main(config: RuntimeInspectorConfig) -> int:
    """Open a standalone inspector and auto-start ``config``."""
    log_path = resolve_session_log_path(config.pack_path)
    logger = configure_session_logging(config.pack_path)
    logger.info("ModSim Runtime Inspector %s session started", __version__)
    logger.info("Session log: %s", log_path)
    logger.info("Robot Pack: %s", config.pack_path)
    logger.info("Backend: %s", config.backend)

    previous_excepthook = sys.excepthook

    def log_unhandled_exception(
        exception_type: type[BaseException],
        exception: BaseException,
        traceback: TracebackType | None,
    ) -> None:
        logger.critical(
            "Unhandled Runtime Inspector exception",
            exc_info=(exception_type, exception, traceback),
        )
        previous_excepthook(exception_type, exception, traceback)

    sys.excepthook = log_unhandled_exception
    window: RuntimeInspectorWindow | None = None
    try:
        existing_application = QApplication.instance()
        application = (
            existing_application
            if isinstance(existing_application, QApplication)
            else QApplication(sys.argv)
        )
        application.setApplicationName("ModSim Runtime Inspector")
        application.setOrganizationName("ModSim")
        window = RuntimeInspectorWindow(config)
        window.show()
        qt_exit_code = application.exec()
        logger.info("Runtime Inspector event loop exited with code %d", qt_exit_code)
        return qt_exit_code if qt_exit_code != 0 else window.exit_code
    except Exception:
        logger.exception("Runtime Inspector failed during startup or event-loop execution")
        raise
    finally:
        if window is not None and not window.stop_and_wait():
            logging.getLogger("modsim.runtime_inspector").critical(
                "Runtime worker did not stop within the shutdown timeout"
            )
        sys.excepthook = previous_excepthook
        logger.info("ModSim Runtime Inspector session ended")
        shutdown_session_logging()


__all__ = ["main"]
