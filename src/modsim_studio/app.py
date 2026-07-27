"""ModSim Studio process entry point."""

from __future__ import annotations

import sys
from pathlib import Path
from types import TracebackType

from PySide6.QtWidgets import QApplication

from modsim import __version__
from modsim_studio.main_window import MainWindow
from modsim_studio.session_logging import (
    configure_session_logging,
    resolve_session_log_path,
    shutdown_session_logging,
)


def main(initial_path: str | Path | None = None) -> int:
    """Start the standalone Qt application."""
    log_path = resolve_session_log_path(initial_path)
    logger = configure_session_logging(initial_path)
    logger.info("ModSim Studio %s session started", __version__)
    logger.info("Session log: %s", log_path)
    logger.info("Python runtime: %s", sys.version.split()[0])

    previous_excepthook = sys.excepthook

    def log_unhandled_exception(
        exception_type: type[BaseException],
        exception: BaseException,
        traceback: TracebackType | None,
    ) -> None:
        logger.critical(
            "Unhandled exception",
            exc_info=(exception_type, exception, traceback),
        )
        previous_excepthook(exception_type, exception, traceback)

    sys.excepthook = log_unhandled_exception
    try:
        existing_application = QApplication.instance()
        application = (
            existing_application
            if isinstance(existing_application, QApplication)
            else QApplication(sys.argv)
        )
        application.setApplicationName("ModSim Studio")
        application.setOrganizationName("ModSim")
        window = MainWindow(
            initial_path=initial_path,
            session_logger=logger,
            session_log_path=log_path,
        )
        window.show()
        exit_code = application.exec()
        logger.info("Qt event loop exited with code %d", exit_code)
        return exit_code
    except Exception:
        logger.exception("Studio failed during startup or event-loop execution")
        raise
    finally:
        sys.excepthook = previous_excepthook
        logger.info("ModSim Studio session ended")
        shutdown_session_logging()


def cli() -> None:
    """Console-script wrapper."""
    initial_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    raise SystemExit(main(initial_path))


if __name__ == "__main__":
    cli()
