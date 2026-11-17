# ModSim

[![CI](https://github.com/modlab-upenn/ModSim/actions/workflows/ci.yml/badge.svg?branch=main&event=push)](https://github.com/modlab-upenn/ModSim/actions/workflows/ci.yml)
[![Python 3.11+](https://img.shields.io/badge/Python-3.11%2B-3776AB?logo=python&logoColor=white)](https://github.com/modlab-upenn/ModSim/blob/refactor/pyproject.toml)
[![Pre-alpha](https://img.shields.io/badge/status-pre--alpha-F59E0B)](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/releases/0.1.0.md)

ModSim is an open-source Python framework for modular reconfigurable robots and
multi-robot systems. Build and validate **RobotPacks** from URDF assets in
**ModSim Studio**, then explore docking, undocking, and reconfiguration with a
backend-neutral runtime and **MuJoCo** physics.

**[Documentation](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/index.md)**
· **[Project website](https://modlab-upenn.github.io/ModSim/)**

## Getting started

ModSim requires **Python 3.11+**. Follow
[Installation](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/how-to/installation.md)
to set up Studio and the bundled Generic Cube, then complete
[Getting started](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/tutorials/getting-started.md)
to inspect a RobotPack and run your first docking example.

## Documentation

<!-- Use published Docusaurus URLs here when the new manual is deployed. -->

| Guide | What you will find |
| --- | --- |
| [Create a RobotPack](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/how-to/index.md) | Prepare a URDF, author connectors, validate, and run a simulation. |
| [Architecture and UML](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/explanation/index.md) | Source map, class responsibilities, runtime flows, and subsystem boundaries. |
| [Python API](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/reference/python-api.mdx) | Reference overview and how the API documentation is generated. |
| [CLI](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/how-to/cli.md) | Terminal workflows and headless execution. |
| [RobotPack format](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/robot_pack_spec.md) | YAML schema, semantics, and persistence contract. |
| Robot-specific examples | [SMORES-EP](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/planning.md) and [M-Blocks](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/mblocks_planning.md). |

For the full API reference and System Architecture, use the
[Developer Documentation](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/contribute/local-development.md).

## Contributing

Contributions to the repository and documentation are
welcome. Read [Contribute](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/contribute/index.md)
and [Development setup](https://github.com/modlab-upenn/ModSim/blob/refactor/docs/contribute/development-setup.md)
for the workflow, required checks, and pull request process.

## License

ModSim code is licensed under the [MIT License](https://github.com/modlab-upenn/ModSim/blob/refactor/LICENSE).
