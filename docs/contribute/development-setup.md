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
from `src/modsim_robotics/`. The optional packages live in
`src/modsim_robotics_studio/` and `src/modsim_robotics_backend_mujoco/`. Run the CLI
with `modsim-robotics` or `python -m modsim_robotics`; the standalone Studio
command is `modsim-robotics-studio`.

Add optional environments only when the change needs them:

```bash
uv sync --locked --extra dev --extra mujoco --extra studio --python 3.12
```

The `mujoco` extra supplies physics dependencies. The `studio` extra supplies the
desktop applications. GUI tests need a display or Xvfb. Documentation editing
has its own Node environment, described in [Local development](local-development.md).

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

For changes to optional packages, also use their dedicated type-check
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
