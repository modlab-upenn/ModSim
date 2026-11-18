"""ModSim Studio process entry point."""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from types import TracebackType

from PySide6.QtWidgets import QApplication, QMainWindow

from modsim_robotics import __version__
from modsim_robotics.user_data import UserDataError, resolve_robot_pack
from modsim_robotics_studio._branding import application_icon, configure_application_identity
from modsim_robotics_studio._desktop import configure_native_process_identity
from modsim_robotics_studio._loading import StartupSplash
from modsim_robotics_studio.appearance import theme_manager
from modsim_robotics_studio.session_logging import (
    configure_session_logging,
    resolve_session_log_path,
    shutdown_session_logging,
)


def _create_window(
    *,
    initial_path: str | Path | None,
    session_logger: logging.Logger,
    session_log_path: Path,
) -> QMainWindow:
    from modsim_robotics_studio.main_window import MainWindow

    return MainWindow(
        initial_path=initial_path,
        session_logger=session_logger,
        session_log_path=session_log_path,
    )


def main(initial_path: str | Path | None = None, *, home: str | Path | None = None) -> int:
    """Start Studio with a blank workspace or an explicitly selected pack.

    Creates the session log, shows the timed cover intro, prepares the
    Builder, and reveals it after at least three seconds of splash display.
    The Qt event loop remains responsive during the branding waits.

    Args:
        initial_path: Installed pack ID, version-qualified ID, or explicit pack
            directory/manifest path. None opens the welcome screen without
            resolving a pack or reading the installed library.
        home: ModSim home for named pack lookup. It has no effect on blank
            launches or explicit pack paths.

    Returns:
        The Qt event-loop exit status, 0 for cancelled startup, or 2 if the
        supplied pack selector cannot be resolved or a deferred Studio
        dependency cannot be imported.
    """
    if initial_path is not None:
        try:
            initial_path = resolve_robot_pack(initial_path, home=home)
        except UserDataError as error:
            print(str(error), file=sys.stderr)
            return 2
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
    splash: StartupSplash | None = None
    try:
        configure_native_process_identity()
        existing_application = QApplication.instance()
        application = (
            existing_application
            if isinstance(existing_application, QApplication)
            else QApplication(sys.argv)
        )
        configure_application_identity(application, "ModSim Studio")
        application.setWindowIcon(application_icon(theme_manager().theme.id))
        splash = StartupSplash("Robot Pack Builder")
        splash.show()
        splash.set_stage("Starting ModSim")
        if not splash.play_intro():
            return 0
        splash.set_stage("Opening RobotPack…" if initial_path is not None else "Loading Studio…")
        window = _create_window(
            initial_path=initial_path,
            session_logger=logger,
            session_log_path=log_path,
        )
        if not splash.present(window):
            window.close()
            return 0
        splash = None
        exit_code = application.exec()
        logger.info("Qt event loop exited with code %d", exit_code)
        return exit_code
    except ImportError as error:
        logger.exception("A Studio dependency could not be loaded")
        print(
            "Could not load ModSim Studio dependencies. "
            "Install with: pip install 'modsim-robotics[studio]'\n"
            f"{error}",
            file=sys.stderr,
        )
        return 2
    except Exception:
        logger.exception("Studio failed during startup or event-loop execution")
        raise
    finally:
        if splash is not None:
            splash.close()
        sys.excepthook = previous_excepthook
        logger.info("ModSim Studio session ended")
        shutdown_session_logging()


def cli() -> None:
    """Console-script wrapper."""
    from modsim_robotics_studio.launcher import cli as launch

    launch()


if __name__ == "__main__":
    cli()
