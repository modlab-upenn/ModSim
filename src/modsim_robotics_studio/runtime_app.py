"""Process entry point for the standalone ModSim Runtime Inspector."""

from __future__ import annotations

import logging
import sys
from types import TracebackType
from typing import TYPE_CHECKING

from PySide6.QtWidgets import QApplication

from modsim_robotics import __version__
from modsim_robotics_studio._branding import application_icon, configure_application_identity
from modsim_robotics_studio._desktop import configure_native_process_identity
from modsim_robotics_studio._loading import StartupSplash
from modsim_robotics_studio.appearance import theme_manager
from modsim_robotics_studio.session_logging import (
    configure_session_logging,
    resolve_session_log_path,
    shutdown_session_logging,
)

if TYPE_CHECKING:
    from modsim_robotics.runtime import RuntimeInspectorConfig
    from modsim_robotics_studio.runtime_window import RuntimeInspectorWindow


def main(config: RuntimeInspectorConfig) -> int:
    """Play the startup intro, then show the inspector and auto-start ``config``.

    The branded splash stays visible for at least three seconds. Runtime
    execution begins after the Inspector is shown, never behind that intro.
    Closing startup returns 0 without starting a runtime.
    """
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
    splash: StartupSplash | None = None
    try:
        configure_native_process_identity()
        existing_application = QApplication.instance()
        application = (
            existing_application
            if isinstance(existing_application, QApplication)
            else QApplication(sys.argv)
        )
        configure_application_identity(application, "ModSim Runtime Inspector")
        manager = theme_manager()
        application.setWindowIcon(application_icon(manager.theme.id))
        splash = StartupSplash("Runtime Inspector")
        splash.show()
        splash.set_stage("Starting ModSim")
        if not splash.play_intro():
            return 0
        splash.set_stage("Loading runtime workspace…")
        # Paint the branded startup surface before importing the plotting widgets.
        application.processEvents()
        from modsim_robotics_studio.runtime_window import RuntimeInspectorWindow

        splash.set_stage("Preparing runtime controls…")
        application.processEvents()
        window = RuntimeInspectorWindow(config)
        if not splash.present(window):
            window.close()
            return 0
        qt_exit_code = application.exec()
        logger.info("Runtime Inspector event loop exited with code %d", qt_exit_code)
        return qt_exit_code if qt_exit_code != 0 else window.exit_code
    except Exception:
        logger.exception("Runtime Inspector failed during startup or event-loop execution")
        raise
    finally:
        if splash is not None:
            splash.close()
        if window is not None and not window.stop_and_wait():
            logging.getLogger("modsim_robotics.runtime_inspector").critical(
                "Runtime worker did not stop within the shutdown timeout"
            )
        sys.excepthook = previous_excepthook
        logger.info("ModSim Runtime Inspector session ended")
        shutdown_session_logging()


__all__ = ["main"]
