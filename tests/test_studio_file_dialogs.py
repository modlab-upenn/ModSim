"""Real Qt pickers expose hidden RobotPack paths without disturbing selection."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyvistaqt")

from PySide6.QtCore import QDir, QSettings, Qt, QTimer
from PySide6.QtGui import QAction, QColor, QPalette
from PySide6.QtTest import QTest
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDialog,
    QFileDialog,
    QFileSystemModel,
    QLineEdit,
    QListView,
    QStyle,
    QStyleOptionFrame,
    QToolButton,
    QTreeView,
    QWidget,
)

from modsim_robotics_studio import main_window
from modsim_robotics_studio.appearance import theme_manager


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.fixture
def preferences(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> QSettings:
    settings = QSettings(str(tmp_path / "file-dialogs.ini"), QSettings.Format.IniFormat)

    def isolated_settings(*_args: object, **_kwargs: object) -> QSettings:
        return QSettings(settings.fileName(), QSettings.Format.IniFormat)

    monkeypatch.setattr(main_window, "QSettings", isolated_settings)
    return settings


def _run_picker(
    parent: QWidget,
    action: Callable[[QFileDialog], bool],
    directory: Path,
    *,
    file_mode: QFileDialog.FileMode = QFileDialog.FileMode.ExistingFile,
    accept_mode: QFileDialog.AcceptMode = QFileDialog.AcceptMode.AcceptOpen,
    name_filter: str = "URDF files (*.urdf)",
) -> str:
    errors: list[BaseException] = []
    completed = False
    deadline = time.monotonic() + 10.0
    timer = QTimer(parent)
    timer.setInterval(20)
    watchdog = QTimer(parent)
    watchdog.setSingleShot(True)

    def reject_on_timeout() -> None:
        dialog = QApplication.activeModalWidget()
        detail = ""
        if isinstance(dialog, QFileDialog):
            model = dialog.findChild(QFileSystemModel)
            detail = (
                f": entries={_visible_names(dialog)}, dialog_filter={dialog.filter()}, "
                f"model_filter={model.filter() if model is not None else None}"
            )
        errors.append(
            AssertionError("File picker interaction exceeded its ten-second deadline" + detail)
        )
        for widget in QApplication.topLevelWidgets():
            if isinstance(widget, QDialog) and widget.isVisible():
                widget.reject()
        timer.stop()

    def tick() -> None:
        nonlocal completed
        dialog = QApplication.activeModalWidget()
        if not isinstance(dialog, QFileDialog):
            return
        try:
            if time.monotonic() >= deadline:
                raise AssertionError(
                    "File picker did not complete the scripted interaction: "
                    f"{dialog.directory().absolutePath()}, {_visible_names(dialog)}"
                )
            completed = action(dialog)
            if completed and dialog.isVisible():
                raise AssertionError("Scripted accept/reject did not close the file picker")
        except BaseException as error:
            errors.append(error)
            completed = True
            dialog.reject()
        if completed:
            timer.stop()

    timer.timeout.connect(tick)
    timer.start()
    watchdog.timeout.connect(reject_on_timeout)
    watchdog.start(10_000)
    try:
        result = main_window._select_path(  # pyright: ignore[reportPrivateUsage]
            parent,
            "Select a RobotPack path",
            str(directory),
            name_filter,
            file_mode=file_mode,
            accept_mode=accept_mode,
        )
    finally:
        timer.stop()
        timer.deleteLater()
        watchdog.stop()
        watchdog.deleteLater()
    if errors:
        raise errors[0]
    assert completed
    return result


def _checkbox(dialog: QFileDialog) -> QCheckBox:
    checkbox = dialog.findChild(QCheckBox, "showHiddenFiles")
    assert checkbox is not None
    assert checkbox.text() == "Show hidden files and folders"
    return checkbox


def _visible_names(dialog: QFileDialog) -> set[str]:
    model = dialog.findChild(QFileSystemModel)
    assert model is not None
    root = model.index(dialog.directory().absolutePath())
    return {model.fileName(model.index(row, 0, root)) for row in range(model.rowCount(root))}


def test_hidden_paths_are_visible_navigable_and_preference_survives_dialogs(
    application: QApplication, preferences: QSettings, tmp_path: Path
) -> None:
    folder = tmp_path / ".modsim"
    folder.mkdir()
    (tmp_path / ".settings").mkdir()
    (folder / "inside.urdf").write_text("robot")
    (tmp_path / ".hidden.urdf").write_text("robot")
    (tmp_path / "visible.urdf").write_text("robot")
    parent = QWidget()
    stage = 0

    def inspect(dialog: QFileDialog) -> bool:
        nonlocal stage
        names = _visible_names(dialog)
        checkbox = _checkbox(dialog)
        if stage == 0:
            if "visible.urdf" not in names:
                return False
            assert not checkbox.isChecked()
            assert names.isdisjoint({".hidden.urdf", ".modsim", ".settings"})
            assert dialog.filter() & QDir.Filter.AllDirs
            assert (dialog.filter() & QDir.Filter.NoDotAndDotDot) == QDir.Filter.NoDotAndDotDot
            checkbox.click()
            stage = 1
            return False
        if stage == 1:
            if not {".hidden.urdf", ".modsim", ".settings"} <= names:
                return False
            assert dialog.directory().absolutePath() == str(tmp_path)
            assert dialog.filter() & QDir.Filter.Hidden
            assert dialog.filter() & QDir.Filter.AllDirs
            assert (dialog.filter() & QDir.Filter.NoDotAndDotDot) == QDir.Filter.NoDotAndDotDot
            dialog.setDirectory(str(folder))
            stage = 2
            return False
        if "inside.urdf" not in names:
            return False
        assert dialog.directory().absolutePath() == str(folder)
        dialog.reject()
        return True

    try:
        assert _run_picker(parent, inspect, tmp_path) == ""
        assert stage == 2
        assert preferences.value("file_dialogs/show_hidden", False, type=bool)
        turned_off = False

        def reopen(dialog: QFileDialog) -> bool:
            nonlocal turned_off
            names = _visible_names(dialog)
            if not turned_off:
                if not {".hidden.urdf", ".settings"} <= names:
                    return False
                assert _checkbox(dialog).isChecked()
                action = dialog.findChild(QAction, "qt_show_hidden_action")
                if action is not None:
                    action.trigger()
                else:
                    _checkbox(dialog).click()
                turned_off = True
                return False
            assert not _checkbox(dialog).isChecked()
            assert not dialog.filter() & QDir.Filter.Hidden
            if names.intersection({".hidden.urdf", ".settings"}):
                return False
            assert dialog.filter() & QDir.Filter.AllDirs
            assert (dialog.filter() & QDir.Filter.NoDotAndDotDot) == QDir.Filter.NoDotAndDotDot
            dialog.reject()
            return True

        assert _run_picker(parent, reopen, tmp_path) == ""
        assert not preferences.value("file_dialogs/show_hidden", True, type=bool)
    finally:
        parent.close()
        parent.deleteLater()


@pytest.mark.parametrize(
    ("file_mode", "accept_mode"),
    (
        (QFileDialog.FileMode.ExistingFile, QFileDialog.AcceptMode.AcceptOpen),
        (QFileDialog.FileMode.Directory, QFileDialog.AcceptMode.AcceptOpen),
        (QFileDialog.FileMode.AnyFile, QFileDialog.AcceptMode.AcceptSave),
    ),
)
def test_picker_modes_keep_directory_and_typed_selection_when_showing_hidden(
    application: QApplication,
    preferences: QSettings,
    tmp_path: Path,
    file_mode: QFileDialog.FileMode,
    accept_mode: QFileDialog.AcceptMode,
) -> None:
    existing = tmp_path / "robot.urdf"
    existing.write_text("robot")
    parent = QWidget()
    target = tmp_path if file_mode == QFileDialog.FileMode.Directory else existing
    if accept_mode == QFileDialog.AcceptMode.AcceptSave:
        target = tmp_path / "new_robot_pack"
    name_filter = (
        "Robot Pack folder (*)"
        if accept_mode == QFileDialog.AcceptMode.AcceptSave
        else "URDF files (*.urdf)"
        if file_mode == QFileDialog.FileMode.ExistingFile
        else ""
    )
    selected = False

    def accept(dialog: QFileDialog) -> bool:
        nonlocal selected
        assert dialog.testOption(QFileDialog.Option.DontUseNativeDialog)
        assert dialog.fileMode() == file_mode and dialog.acceptMode() == accept_mode
        if name_filter:
            assert dialog.nameFilters() == [name_filter]
        if not selected:
            filename = dialog.findChild(QLineEdit, "fileNameEdit")
            assert filename is not None
            if file_mode != QFileDialog.FileMode.Directory:
                filename.clear()
                QTest.keyClicks(filename, target.name)
            typed = filename.text()
            current = dialog.directory().absolutePath()
            _checkbox(dialog).click()
            assert dialog.directory().absolutePath() == current
            assert filename.text() == typed
            selected = True
        dialog.accept()
        return True

    try:
        result = _run_picker(
            parent,
            accept,
            tmp_path,
            file_mode=file_mode,
            accept_mode=accept_mode,
            name_filter=name_filter,
        )
        assert result == str(target)
        if accept_mode == QFileDialog.AcceptMode.AcceptSave:
            assert not target.exists()
    finally:
        parent.close()
        parent.deleteLater()


def _rename_editor(view: QListView | QTreeView) -> QLineEdit | None:
    return next((editor for editor in view.findChildren(QLineEdit) if editor.isVisible()), None)


def _assert_readable_editor(editor: QLineEdit) -> None:
    option = QStyleOptionFrame()
    editor.initStyleOption(option)
    contents = editor.style().subElementRect(QStyle.SubElement.SE_LineEditContents, option, editor)
    assert contents.height() >= editor.fontMetrics().height(), (
        f"Rename text is clipped: editor={editor.height()}px, "
        f"contents={contents.height()}px, font={editor.fontMetrics().height()}px"
    )
    palette = editor.palette()
    assert palette.color(QPalette.ColorRole.Text) != palette.color(QPalette.ColorRole.Base)
    assert palette.color(QPalette.ColorRole.HighlightedText) != palette.color(
        QPalette.ColorRole.Highlight
    )


def _luminance(color: QColor) -> float:
    channels = (color.redF(), color.greenF(), color.blueF())
    linear = tuple(
        value / 12.92 if value <= 0.04045 else ((value + 0.055) / 1.055) ** 2.4
        for value in channels
    )
    return sum(
        value * weight for value, weight in zip(linear, (0.2126, 0.7152, 0.0722), strict=True)
    )


def _assert_visible_navigation_glyph(button: QToolButton) -> None:
    image = button.grab().toImage()
    ratio = image.devicePixelRatio()
    background = _luminance(image.pixelColor(int(3 * ratio), image.height() // 2))
    radius = max(2, int(3 * ratio))
    center_x, center_y = image.width() // 2, image.height() // 2
    contrasts: list[float] = []
    for y in range(center_y - radius, center_y + radius + 1):
        for x in range(center_x - radius, center_x + radius + 1):
            foreground = _luminance(image.pixelColor(x, y))
            contrasts.append(
                (max(background, foreground) + 0.05) / (min(background, foreground) + 0.05)
            )
    assert max(contrasts) >= 3.0, "Enabled navigation arrow is unreadable against its button"


def test_picker_navigation_arrows_are_visible_and_keep_directory_history(
    application: QApplication, preferences: QSettings, tmp_path: Path
) -> None:
    library = tmp_path / "robot_library"
    library.mkdir()
    robot = library / "robot.urdf"
    robot.write_text("robot", encoding="utf-8")
    parent = QWidget()
    manager = theme_manager()
    previous_theme = manager.theme.id
    manager.set_theme("midnight", persist=False)
    stage = 0

    def navigate(dialog: QFileDialog) -> bool:
        nonlocal stage
        expected = library if stage in {0, 2, 4} else tmp_path
        if dialog.directory().absolutePath() != str(expected):
            return False
        if stage == 4:
            if robot.name not in _visible_names(dialog):
                return False
            filename = dialog.findChild(QLineEdit, "fileNameEdit")
            assert filename is not None
            filename.clear()
            QTest.keyClicks(filename, robot.name)
            dialog.accept()
            return True
        name = ("toParentButton", "backButton", "forwardButton", "backButton")[stage]
        button = dialog.findChild(QToolButton, name)
        assert button is not None and button.isEnabled()
        _assert_visible_navigation_glyph(button)
        button.click()
        stage += 1
        return False

    try:
        assert _run_picker(parent, navigate, library) == str(robot)
    finally:
        manager.set_theme(previous_theme, persist=False)
        parent.close()
        parent.deleteLater()


@pytest.mark.parametrize("mode", ["list", "detail", "icon"])
def test_picker_inline_rename_is_readable_and_commits_file_and_folder(
    application: QApplication,
    preferences: QSettings,
    tmp_path: Path,
    mode: str,
) -> None:
    original = tmp_path / "robot_description.urdf"
    original.write_text("robot description", encoding="utf-8")
    folder = tmp_path / "example_robot_pack"
    folder.mkdir()
    renamed_file = tmp_path / "renamed_robot.urdf"
    renamed_folder = tmp_path / "renamed_robot_pack"
    parent = QWidget()
    manager = theme_manager()
    previous_theme = manager.theme.id
    manager.set_theme("midnight", persist=False)
    stage = 0

    def rename(dialog: QFileDialog) -> bool:
        nonlocal stage
        model = dialog.findChild(QFileSystemModel)
        assert model is not None
        if stage == 0:
            dialog.setViewMode(
                QFileDialog.ViewMode.Detail if mode == "detail" else QFileDialog.ViewMode.List
            )
        view = (
            dialog.findChild(QTreeView, "treeView")
            if mode == "detail"
            else dialog.findChild(QListView, "listView")
        )
        assert view is not None
        if isinstance(view, QListView) and mode == "icon" and stage == 0:
            view.setViewMode(QListView.ViewMode.IconMode)
        if stage in {0, 2, 4}:
            path = folder if stage == 4 else original
            index = model.index(str(path))
            if not index.isValid() or not view.visualRect(index).isValid():
                return False
            view.setCurrentIndex(index)
            view.setFocus()
            QTest.keyClick(view, Qt.Key.Key_F2)
            stage += 1
            return False
        if stage in {1, 3, 5}:
            editor = _rename_editor(view)
            if editor is None:
                return False
            _assert_readable_editor(editor)
            editor.selectAll()
            dialog.grab().save(str(tmp_path / f"{mode}-rename-selected.png"))
            editor.deselect()
            dialog.grab().save(str(tmp_path / f"{mode}-rename-unselected.png"))
            if stage == 1:
                QTest.keyClick(editor, Qt.Key.Key_Escape)
                assert dialog.isVisible()
                assert original.is_file() and not renamed_file.exists()
                stage = 2
                return False
            editor.selectAll()
            QTest.keyClicks(editor, renamed_file.name if stage == 3 else renamed_folder.name)
            QTest.keyClick(editor, Qt.Key.Key_Return)
            stage += 1
            return False
        if stage == 6:
            if not renamed_file.is_file() or not renamed_folder.is_dir():
                return False
            assert not original.exists() and not folder.exists()
            assert renamed_file.read_text(encoding="utf-8") == "robot description"
            dialog.selectFile(str(renamed_file))
            dialog.accept()
            return True
        return False

    try:
        assert _run_picker(parent, rename, tmp_path) == str(renamed_file)
    finally:
        manager.set_theme(previous_theme, persist=False)
        parent.close()
        parent.deleteLater()


@pytest.mark.parametrize("scale", ["1.5", "2"])
def test_picker_inline_rename_at_scaled_display(scale: str, tmp_path: Path) -> None:
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["QT_SCALE_FACTOR"] = scale
    environment["MPLCONFIGDIR"] = str(tmp_path / "matplotlib")
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            str(Path(__file__).resolve()),
            "-k",
            "test_picker_inline_rename_is_readable_and_commits_file_and_folder or "
            "test_picker_navigation_arrows_are_visible_and_keep_directory_history",
            "--basetemp",
            str(tmp_path / "scaled-picker"),
            "--no-cov",
        ],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
