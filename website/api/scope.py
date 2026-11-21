"""Source-file scope shared by API generation and contributor documentation checks."""

API_MODULES = {
    "runtime/session.py",
    "runtime/metrics.py",
    "core/scene.py",
    "core/state.py",
    "core/entities.py",
    "core/events.py",
    "core/ids.py",
    "core/snapshot.py",
    "core/transforms.py",
    "backends/base.py",
    "robot_packs/schema.py",
    "units/definitions.py",
    "units/conversion.py",
    "units/preferences.py",
    "units/fields.py",
    "unit_settings.py",
}

# Resolve helper imports statically without publishing their reference pages yet.
PARSED_MODULES = API_MODULES | {
    "units/__init__.py",
    "user_data.py",
    "robot_packs/__init__.py",
    "robot_packs/errors.py",
    "robot_packs/issues.py",
    "robot_packs/loader.py",
    "robot_packs/validator.py",
    "robot_packs/writer.py",
    "core/assemblies.py",
    "backends/registry.py",
    "connectors/docking.py",
    "connectors/acceptance.py",
    "connectors/broadphase.py",
    "connectors/compatibility.py",
    "connectors/guards.py",
}
