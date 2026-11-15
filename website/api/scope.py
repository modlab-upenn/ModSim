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
}

# Resolve helper imports statically without publishing their reference pages yet.
PARSED_MODULES = API_MODULES | {
    "core/assemblies.py",
    "backends/registry.py",
    "connectors/docking.py",
    "connectors/acceptance.py",
    "connectors/broadphase.py",
    "connectors/compatibility.py",
    "connectors/guards.py",
}
