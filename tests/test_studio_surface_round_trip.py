"""Studio keeps continuous-docking fields through edit, save and reload.

The connector-type editor rebuilds ``physical_connection``, ``limits`` and
``docking_policy`` from its widgets, which have no controls for
``surface_realizations``, ``ball`` anchors, ``max_torsion_moment_nm`` or
``break_limits``. These tests prove that editing an unrelated field and saving
never drops them, through both the project API and the real editor panels.
"""

from __future__ import annotations

import os
from collections.abc import Iterator
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyvistaqt")

from PySide6.QtCore import QCoreApplication, QEvent, Qt, Signal
from PySide6.QtWidgets import QApplication, QLineEdit, QPushButton, QTreeWidgetItem, QWidget

from modsim.robot_packs import LoadedRobotPack, RobotPack, RobotPackLoader, RobotPackWriter
from modsim_studio import StudioProject, main_window
from surface_fixtures import surface_pack

SURFACE_TYPES = ("shell", "magnet")


def _write_surface_pack(example_pack_dir: Path, destination: Path) -> RobotPack:
    loaded = RobotPackLoader().load(example_pack_dir)
    pack = surface_pack(
        loaded.pack,
        realizations=("driven_fixed", "ball", "static_fixed"),
        magnet={
            "docking_policy": {
                "alignment": "nominal",
                "redock_cooldown_s": 0.2,
                "break_limits": {
                    "max_normal_force_n": 79.0,
                    "max_shear_force_n": 49.0,
                    "max_bending_moment_nm": 1.6,
                    "max_torsion_moment_nm": 1.3,
                },
            }
        },
    )
    RobotPackWriter().write(LoadedRobotPack(loaded.root, loaded.manifest_path, pack), destination)
    return RobotPackLoader().load(destination).pack


def _assert_surface_fields_kept(original: RobotPack, reloaded: RobotPack) -> None:
    for type_id in SURFACE_TYPES:
        before = original.hardware_catalog.connector_types[type_id]
        after = reloaded.hardware_catalog.connector_types[type_id]
        assert after.model_dump(exclude={"name"}) == before.model_dump(exclude={"name"}), type_id
    for module_id in ("node", "strut"):
        assert (
            reloaded.hardware_catalog.module_types[module_id].connectors
            == original.hardware_catalog.module_types[module_id].connectors
        )


def test_project_api_load_edit_save_keeps_surface_fields(
    example_pack_dir: Path, tmp_path: Path
) -> None:
    original = _write_surface_pack(example_pack_dir, tmp_path / "surface")
    project = StudioProject.open(tmp_path / "surface")
    magnet = project.pack.hardware_catalog.connector_types["magnet"]

    saved = project.update_connector_type(magnet.model_copy(update={"name": "Renamed"})).save()

    reloaded = StudioProject.open(saved.loaded.root).pack
    assert reloaded.hardware_catalog.connector_types["magnet"].name == "Renamed"
    _assert_surface_fields_kept(original, reloaded)


# ----------------------------------------------------------------------
# the real editor panels
# ----------------------------------------------------------------------


class _ViewportStub(QWidget):
    """Keep this test about the property panels; VTK is covered elsewhere."""

    entity_selected = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

    def render_module(self, *_args: object) -> None:
        pass

    def set_layer_visibility(self, **_kwargs: bool) -> None:
        pass

    def select_entity(self, *_args: str) -> None:
        pass

    def fit_module(self) -> None:
        pass


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.fixture
def surface_window(
    application: QApplication,
    example_pack_dir: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[tuple[main_window.MainWindow, RobotPack, Path]]:
    destination = tmp_path / "surface"
    original = _write_surface_pack(example_pack_dir, destination)
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)
    window = main_window.MainWindow(initial_path=destination)
    window.show()
    application.processEvents()
    yield window, original, destination
    window.project = None
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)


# The project tree tags each item with (kind, id, module) in these data roles.
_KIND_ROLE = int(Qt.ItemDataRole.UserRole)
_ID_ROLE = _KIND_ROLE + 1
_MODULE_ROLE = _KIND_ROLE + 2


def _select(
    application: QApplication,
    window: main_window.MainWindow,
    kind: str,
    entity_id: str,
    module_id: str = "",
) -> None:
    """Select one entity in the project tree, as a user click would."""
    pending: list[QTreeWidgetItem] = []
    for index in range(window.project_tree.topLevelItemCount()):
        item = window.project_tree.topLevelItem(index)
        if item is not None:
            pending.append(item)
    while pending:
        item = pending.pop()
        if (
            item.data(0, _KIND_ROLE) == kind
            and item.data(0, _ID_ROLE) == entity_id
            and item.data(0, _MODULE_ROLE) == module_id
        ):
            window.project_tree.setCurrentItem(item)
            application.processEvents()
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            return
        pending.extend(item.child(index) for index in range(item.childCount()))
    raise AssertionError(f"no {kind} '{entity_id}' in the project tree")


def _click(window: main_window.MainWindow, label: str) -> None:
    buttons = [b for b in window.properties.findChildren(QPushButton) if b.text() == label]
    assert len(buttons) == 1, f"expected one '{label}' button, found {len(buttons)}"
    buttons[0].click()


def test_editor_apply_and_save_keep_surface_fields(
    application: QApplication,
    surface_window: tuple[main_window.MainWindow, RobotPack, Path],
) -> None:
    window, original, destination = surface_window

    for type_id in SURFACE_TYPES:
        _select(application, window, "connector_type", type_id)
        current = original.hardware_catalog.connector_types[type_id].name
        name = next(
            field for field in window.properties.findChildren(QLineEdit) if field.text() == current
        )
        name.setText(f"{current} (edited)")
        _click(window, "Apply connector type")
        application.processEvents()

    _select(application, window, "connector", "upper", "strut")
    _click(window, "Apply connector")
    application.processEvents()
    assert window.project is not None and window.project.dirty

    window.save_action.trigger()
    application.processEvents()

    reloaded = RobotPackLoader().load(destination).pack
    for type_id in SURFACE_TYPES:
        assert reloaded.hardware_catalog.connector_types[type_id].name == (
            f"{original.hardware_catalog.connector_types[type_id].name} (edited)"
        )
    _assert_surface_fields_kept(original, reloaded)
