---
title: Installation
sidebar_label: Installation
slug: /installation
---

To use ModSim we recommend creating a virtual environment and installing the modsim package with it dependancies.

Getting started requires ModSim Studio for inspection, authoring, and its
integrated runtime workspace. The installation commands below include its
desktop dependencies; use a working
desktop display to open and edit RobotPacks.

## Setup a virtual environment

A virtual environment keeps ModSim's dependencies separate from other Python
projects. It is recommended if you do not already have a suitable environment;
it is not an additional ModSim setup requirement. The examples use Python 3.12.

### macOS and Linux

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install "modsim-robotics[studio]"
```

### Windows PowerShell

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install "modsim-robotics[studio]"
```
ModSim requires Python 3.11 or newer. Install it from PyPI in your Python
environment:

## Install the modsim package

```bash
pip install "modsim-robotics[studio]"
```

This installs Studio, the core library, command-line tools, mock backend, and
bundled **Generic Cube** RobotPack.

The package and terminal command are named `modsim-robotics`. 

## Install MuJoCo

When you reach the physics simulation step in Getting started, add MuJoCo to
the same Python environment:

```bash
pip install "modsim-robotics[studio,mujoco]"
```

This adds the physics dependencies to your Studio installation. You do not need
to create another environment, reinstall Studio separately, or repeat RobotPack
setup. Studio inspection and authoring do not require MuJoCo. The **Mock**
backend also runs supported kinematic demonstrations in Studio without it;
choose **MuJoCo** for the physics examples.

If you came from Getting started, return to
[the docking example](../tutorials/getting-started.md#5-run-the-docking-example)
after installing.

## Verify the installation

```bash
modsim-robotics --version
```

This prints the installed ModSim version.

## After installation

To prepare a local storage for the RobotPacks and ModSim Configurations, run:

```bash
modsim-robotics setup
```

By default, this prepares `~/.modsim` and prints the starter pack's location.
It seeds the Generic Cube RobotPack and a `config.toml` containing:

```toml
schema_version = 1

[paths]
robot_packs = "robot_packs"
```

The configured library path is relative to the ModSim home, or it can be an
absolute directory. Setup places Generic Cube under
`<library>/modlab/generic_cube/0.1.0/`. Select it by name when opening Studio:

```bash
modsim-robotics studio generic_cube
```

Studio opens the pack in **Robot Pack**. Its **Runtime** workspace lets you
choose a backend, demonstration, and model view, then validate and start a run
without another terminal command. Follow
[Getting started](../tutorials/getting-started.md#5-run-the-docking-example)
for the first launch.

Studio does not initialize the home automatically. Run setup explicitly once
before using the bundled starter. Running setup again preserves an existing
valid pack and configuration. Older configurations without `[paths].robot_packs`
use the standard library directory; the old `[defaults].robot_pack` setting is
ignored.

The configuration says where packs are stored, not which robot to open. Supply a
selector to open an existing pack in Studio. If a name matches more than one
installed pack, use a version such as `generic_cube@0.1.0` or an explicit path;
ModSim does not silently choose the latest version or the only installed robot.

To author a new pack, open the blank Builder instead:

```bash
modsim-robotics studio
```

Choose **Create from URDF** on its welcome screen. This blank launch does not
require an installed pack or library setup; see [Import a URDF](import-urdf.md).

### Keep the ModSim path in your shell

`MODSIM_PATH` points to the ModSim data directory, not to the command executable.
It is separate from your system's `PATH` variable. Setup prints these instructions;
it does not change your shell profile or environment for you.
Use the same home printed by setup; the examples below assume `~/.modsim`.

For Bash, add this line to `~/.bashrc`. For Zsh, add it to `~/.zshrc`:

```bash
export MODSIM_PATH="$HOME/.modsim"
```

Reload the appropriate file with `source ~/.bashrc` for Bash or
`source ~/.zshrc` for Zsh, or open a new terminal.

In Windows PowerShell, set the current session and persist the value for future
sessions:

```powershell
$env:MODSIM_PATH = "$HOME\.modsim"
[Environment]::SetEnvironmentVariable("MODSIM_PATH", $env:MODSIM_PATH, "User")
```

To use another data directory, run `modsim-robotics setup --home PATH` and use that
same directory as your persisted `MODSIM_PATH`. An explicit `--home PATH` takes
precedence over `MODSIM_PATH`; the older `MODSIM_HOME` setting remains a fallback
when `MODSIM_PATH` is unset. With none set, ModSim uses `~/.modsim`.

You can also select by path, independently of the library configuration:

```bash
modsim-robotics studio --pack /absolute/path/to/another_robot_pack
```

An absolute path or a relative path beginning with `./` bypasses library lookup
without changing `config.toml`. The `--pack` option also accepts names such as
`generic_cube`. Omitting it opens the blank Builder. Commands that inspect,
validate, generate views, dock, or run a pack still require an explicit selector.

For terminal inspection, validation, and docking, use the separate
[command-line guide](cli.md).

Continue with [Getting started](../tutorials/getting-started.md#explore-and-simulate-generic-cube)
to run your first simulation. Want to contribute to ModSim? Check out
[Contribute](../contribute).
