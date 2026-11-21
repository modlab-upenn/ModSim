---
title: Development setup
sidebar_label: Development setup
---

Use a source checkout when you want to change ModSim, run its development checks,
or prepare a wheel for someone else. To run an installed release without changing
the application, follow [Installation](../how-to/installation.md).

## Fork and create a branch

Fork `modlab-upenn/ModSim` on GitHub, then clone your fork and add the upstream
repository. Replace `YOUR-USERNAME` with your GitHub account name.

```bash
git clone https://github.com/YOUR-USERNAME/ModSim.git
cd ModSim
git remote add upstream https://github.com/modlab-upenn/ModSim.git
```

This documentation setup currently lives on `refactor`. Start a focused branch
from the corresponding upstream code while the setup is being developed:

```bash
git fetch upstream refactor
git switch -c feature/my-change upstream/refactor
```

When these changes reach the default branch, base new work on that branch. Read
the [current architecture contracts](index.md#current-architecture-contracts)
before changing their behavior.

Keep your AI agent's instructions, settings, memory, and session history local.
Common agent files and directories are excluded by `.gitignore`, including
personal `AGENTS.md` files. Shared project guidance belongs in these contributor
pages and the architecture contracts so it is available with any development tool.

## Install the development environment

Install `uv`, then run these commands from the repository root:

```bash
uv sync --locked --extra dev --python 3.12
uv run --no-sync modsim-robotics --version
```

The source package is installed in the development environment so edits to
`src/` are used by subsequent runs. This is the contributor workflow; it differs
from installing a built wheel into a user's environment.

The distribution name is `modsim-robotics`; Python imports use `modsim_robotics`
from `src/modsim_robotics/`. The desktop and simulator-adapter namespaces live in
`src/modsim_robotics_studio/` and `src/modsim_robotics_backend_mujoco/`. Run the CLI
with `modsim-robotics` or `python -m modsim_robotics`; the standalone Studio
command is `modsim-robotics-studio`.

The default project installation includes Studio and its desktop dependencies.
Add the optional MuJoCo backend when the change needs physics:

```bash
uv sync --locked --extra dev --extra mujoco --python 3.12
```

The `mujoco` extra supplies physics dependencies. Studio's Robot Pack and Runtime
workspaces and the standalone Inspector are available without an extra. The
legacy `studio` extra remains an empty compatibility alias for old install
commands. GUI tests need a display or Xvfb. Documentation editing
has its own Node environment, described in [Local development](local-development.md).

## Exercise the Studio workflow

Start the editable application from this environment:

```bash
uv run --no-sync modsim-robotics studio
```

Use **Open Robot Pack** to open a pack, then switch between **Robot Pack** and
**Runtime** in the same window. Use a temporary copy of the Generic Cube example
when testing recipe edits and Save. The [runtime how-to](../how-to/run-robot-pack.md)
describes the controls and their units.

For a desktop or runtime change, check the full handoff:

1. Run **Authoring Profile** (**F6**) and **Simulation Metadata (Structural)**
   (**F7**) in the Robot Pack workspace. These are pack checks; they do not
   establish that a selected Inspector launch will work.
2. In **Runtime**, select **Mock**, **Dock undock**, the module type, and a runtime
   model view. For a pack without a recipe, **Add topology view** must create an
   unsaved edit without writing the pack automatically.
3. Choose **Validate runtime**, or **Validate → Runtime Launch** (**F8**), and
   inspect the launch-specific report. **Start simulation** rechecks the inputs
   and asks **Save and start** when the pack has unsaved edits.
4. Check incoming state/events, **Pause**/**Resume**, **Stop**, retained results,
   and **Back to setup**. Switch back to Robot Pack to confirm that in-memory
   edits and the authoring dock arrangement are preserved.
5. With MuJoCo installed, repeat with **MuJoCo** and its **Native 3D viewer**
   enabled. One child process owns physics and the companion viewer; Studio
   receives immutable frames from that same simulation. Stop or close the native
   viewer to release the execution resources before configuring the next run.
6. Close Studio during a run and check that the worker or viewer child stops.
   Saving, exporting, and replacing the open pack must stay disabled while a run
   owns those resources; authoring edits can remain in memory for the next launch.

Runtime settings last for the current Studio session. Switching backends preserves
the scenario settings; changing the demonstration or choosing **Reset demonstration
settings** applies preset defaults. The authored recipe is persisted by Save.

The focused tests for this handoff are:

```bash
uv run --no-sync pytest tests/test_studio_runtime_launch.py tests/test_studio_runtime_workspace.py tests/test_runtime_brand_loading.py
```

Keep the core/backend boundary intact when extending this workflow. Launch
preflight belongs in `modsim_robotics_studio/runtime_launch.py`, form state and
embedded execution in `runtime_workspace.py`, and pack editing in `StudioProject`.
`RuntimeInspectorRunner` retains simulation ownership through the existing
worker/process controllers. Generic pack validation and core runtime APIs must
remain usable without Qt or an Inspector model-view recipe.

## Make and check a focused change

Keep core code independent of simulator and desktop dependencies. Update the
relevant documentation when behavior changes and keep RobotPack YAML semantics
separate from imported mechanical assets.

Public API changes must follow the
[API documentation contract](api-documentation.md). Update annotations,
parameter descriptions, return values, relevant errors, and examples beside
the code. Register new public modules in `website/api/scope.py` so they appear
in the published reference.

For a core change, run the checks appropriate to its scope:

```bash
uv run --no-sync ruff check .
uv run --no-sync ruff format --check .
uv run --no-sync pyright
uv run --no-sync pytest
git diff --check
```

For desktop and simulator-adapter changes, also use their dedicated type-check
configurations and relevant tests:

```bash
uv run --no-sync pyright --project pyright-mujoco.json
uv run --no-sync pyright --project pyright-studio.json
```

The GitHub Actions matrix provides separate core, MuJoCo, and Studio jobs. State
which checks you ran in the pull request so reviewers can relate them to the
change.

## Check the API documentation

Follow [Local development](local-development.md) and
[Documentation generation](documentation-generation.md) for the Node and
Graphviz prerequisites. Install the Python documentation tools alongside the
development environment, then check the source contract and rendered manual:

```bash
uv sync --locked --group docs --inexact --python 3.12
npm --prefix website run docs:check
npm --prefix website run typecheck
npm --prefix website run docs:build
```

`docs:check` validates the published API scope. The production build regenerates
the API Markdown and diagrams before compiling the manual. Review the affected
pages in the local preview and keep generated output out of the commit. CI
runs the documentation policy and build checks; public documentation deployment
remains a separate setup step.

## Build an installable distribution

```bash
uv build
```

This writes a wheel and a source distribution to `dist/`. The wheel can be
supplied to someone following the no-checkout installation guide. Public
distribution includes the small Generic Cube starter; larger example assets
are not part of that starter workflow.

The full test suite runs from a source checkout, where its example fixtures are
available. Tests are not included in the source distribution. The artifact
checks below exercise the installable end-user workflow instead.

Check both built artifacts in fresh environments outside the checkout before
sharing them. For the current package version:

```bash
uv run --no-sync python tools/release/smoke_install.py dist/modsim_robotics-0.1.0-py3-none-any.whl
uv run --no-sync python tools/release/smoke_install.py dist/modsim_robotics-0.1.0.tar.gz
```

Use the actual filenames produced by the build. Each check installs the artifact
in a temporary environment, prepares the starter, validates it, runs mock
docking, and checks repeated setup. The temporary environment is removed when
the check finishes. Preparing local files does not publish them.

## Open a pull request

Review your diff, commit the focused change, push the feature branch to your
fork, and open a pull request against its integration branch. Describe the
problem, resulting behavior, and verification. Keep generated build output and
local ModSim home data out of the commit.

Maintainers preparing a public version should continue with the
[release process](release-process.md).
