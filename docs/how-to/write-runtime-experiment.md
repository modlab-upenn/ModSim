---
title: Write a runtime experiment
sidebar_label: Write a runtime experiment
---

Create a scene and handle events directly when the named demonstrations in
Studio or the CLI are no longer enough. This example loads the saved workshop
cube, creates two instances, docks their touching faces, advances the world, and releases the
connection. It runs without Studio or a display.

Complete [Validate and save](validate-and-save.md) first. The base installation
can run the example with the mock backend; install the
[MuJoCo component](installation.md#install-mujoco) to run the same experiment
with physics.

This core-runtime workflow does not require a model-view recipe. The Inspector
uses a generated view for presentation; the experiment below works directly
with the canonical runtime state and events. Use this route for custom scene
layouts or control loops beyond Studio's current demonstration presets.

## Save the experiment

Copy this complete script into `experiment.py` in an ordinary working folder.
It expects the `workshop_cube` module and `front`/`rear` connectors authored in
the preceding guides.

```python
import argparse
from pathlib import Path

from modsim_robotics import RobotPackLoader, RuntimeSession, SceneSpec
from modsim_robotics.core.events import DockCommitted, Event
from modsim_robotics.core.ids import ConnectorInstanceId


def on_event(event: Event) -> None:
    print(f"{event.time_s:.3f}s  {event.kind}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("pack", type=Path)
    parser.add_argument("--backend", choices=("mock", "mujoco"), default="mock")
    args = parser.parse_args()

    loaded = RobotPackLoader().load(args.pack.expanduser())
    scene = SceneSpec.grid("workshop_cube", count=2, spacing_m=0.1)
    options = {"gravity": (0.0, 0.0, 0.0)} if args.backend == "mujoco" else {}
    session = RuntimeSession.create(loaded, scene, args.backend, **options)
    try:
        session.request_dock(
            ConnectorInstanceId("workshop_cube_0/front"),
            ConnectorInstanceId("workshop_cube_1/rear"),
        )
        dock_events = session.process_docking()
        for event in dock_events:
            on_event(event)

        connection = next(
            (event.connection_id for event in dock_events if isinstance(event, DockCommitted)),
            None,
        )
        if connection is None:
            raise RuntimeError("Docking did not commit; inspect the reported events.")

        for _ in range(100):
            for event in session.step(0.002):
                on_event(event)

        session.request_undock(connection)
        for event in session.process_docking():
            on_event(event)
        print(session.metrics().as_dict())
    finally:
        session.shutdown()


if __name__ == "__main__":
    main()
```

Run it against the saved workspace pack:

```bash
python experiment.py "~/.modsim/workspaces/workshop_cube"
python experiment.py "~/.modsim/workspaces/workshop_cube" --backend mujoco
```

Replace the pack path if your ModSim home is elsewhere. This script accepts an
explicit directory; it does not resolve a library ID. Keep the `LoadedRobotPack`
object rather than passing only its `.pack` model to the runtime so the backend
receives the filesystem root for mechanical assets.

## Read the sequence

`SceneSpec.grid()` places two cube centers 0.1 m apart along world X. Their
authored front/rear faces already coincide, so this experiment does not need
an approach controller. The instances are named `workshop_cube_0` and
`workshop_cube_1`.

`request_dock()` queues intent. `process_docking()` evaluates the aligned pair
against the current backend snapshot without advancing physics. This staging
pass establishes the requested constraint before the first integration step.
Ordinary moving scenes can queue requests and let `step()` process them after
the next measured snapshot instead.

The returned events include `DockCommitted` only after backend acceptance.
The example uses that event's connection ID when requesting release. Each
integration step advances physics, ingests measurements, handles overloads,
and processes connectors. The small `on_event()` function is called by the
experiment for each returned event; it is a place to add your own recording
or decisions.

After release, the metrics should show one successful dock, one successful
undock, zero active connections, and two assemblies. The cubes have not been
commanded to move apart. Removal of the constraint changes connectivity; a
motion controller would supply separation.

The `finally` block releases resources even when an event check or an experiment
step fails. The retained `WorldState` records measured state and committed
connections; planner targets or visualization layouts are separate inputs.

## Extend the scene deliberately

Use `SceneSpec.of()` with `ModulePlacement` and `Transform` objects for custom
poses instead of a row. Check that your connector frames can actually reach
the authored acceptance conditions. A pack can describe several module types;
choose each placement's type explicitly.

For a robot with actuated joints, follow [Configure joint control](configure-joints.md)
before submitting effort commands. Set finite targets through the session's
joint-command API, observe measured joint state, and clear persistent commands
before shutdown. The cube example has no joints to command.

Use [Troubleshoot a RobotPack](troubleshoot-robot-pack.md) when events explain
a rejected dock. For ownership and step ordering, continue with
[the runtime explanation](../explanation/runtime-model.mdx).

Previous: [Run the RobotPack](run-robot-pack.md).
Return to the [workflow overview](index.md).
