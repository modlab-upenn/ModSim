---
title: Contribute
sidebar_label: Contribute
slug: /contribute
---

Contribute to ModSim by improving the documentation, adding robot descriptions,
fixing bugs, or extending the runtime, backends, and planners. This section
collects the development workflows and contribution process for those changes.

For documentation changes, begin with [Local development](local-development.md).
It explains how to run the documentation application, edit Markdown, and check
the rendered pages.

For Python API pages and code-derived UML diagrams, follow
[Documentation generation](documentation-generation.md). Edit Python docstrings
and the diagram scope, then regenerate their output as part of the local build.

For code changes, follow [Development setup](development-setup.md) to create a
branch, install the editable package, run checks, and prepare a pull request.
Public Python API changes must also follow
[API documentation](api-documentation.md), including source docstrings, type
annotations, scope registration, and the required documentation checks.
Maintainers can then use the [Release process](release-process.md) to prepare
checked distributions and configure publication.

## Work on a contribution

1. Describe the change and the behavior or documentation it should improve.
2. Read the current contract for the part of ModSim you are changing. The
   repository's `AGENTS.md` lists the relevant contracts and architectural rules.
3. Make a focused change, updating the relevant documentation alongside code.
4. Run the checks appropriate to the change and review its diff.
5. Open a pull request explaining the result and how it was verified.

Keep core code independent of desktop and simulator dependencies. Robot Pack
YAML describes modular-robot semantics, `WorldState` holds canonical runtime
state, and model views are generated from that state.
