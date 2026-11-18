# ModSim documentation development

This directory contains the local Docusaurus application. Authored pages live in
[`../docs`](../docs/); the application reads them directly.

Follow the [local development guide](../docs/contribute/local-development.md) for installation,
live editing, and production preview commands. Track the remaining setup work in
[SETUP_PLAN.md](SETUP_PLAN.md). The Documentation sidebar starts with Overview,
[Installation](../docs/how-to/installation.md), then Tutorials /
[Getting started](../docs/tutorials/getting-started.md), which uses Studio's
**Robot Pack** and **Runtime** workspaces for inspection, authoring, configuration,
and live simulation views. The
[How-to guides](../docs/how-to/index.md) take a new URDF through Studio import,
connector authoring, optional joints, validation and saving, runtime execution,
Python experiments, and troubleshooting. The [CLI workflow](../docs/how-to/cli.md)
provides the parallel terminal route. The separate
[Contribute](../docs/contribute/index.md) section contains code development,
local documentation development, and release instructions.

The [runtime how-to](../docs/how-to/run-robot-pack.md) is the main desktop launch
guide. Keep its backend/model-view selection, validation, save-before-start,
and playback steps aligned with the tutorial and
[Studio architecture](../docs/explanation/studio-and-inspector.md). The native
MuJoCo 3D viewer remains a companion window for the same simulation.

The [Explanation overview](../docs/explanation/index.md) introduces the system's
boundaries and reading order. Its chapters cover all source modules, the
RobotPack pipeline, runtime state, docking, backends, Studio and Inspector,
planning and scenarios, and generated model views. Focused Mermaid class,
sequence, state, and flow diagrams explain structure alongside behavior.
Reference contains the
[Python API landing page](../docs/reference/python-api.mdx) and generated module
pages in the same Docusaurus manual. The API uses the existing Documentation
sidebar, theme, breadcrumbs, and table of contents under **Reference → Python API**.

The shared palette, self-hosted fonts, and documentation layout are described in
[THEME.md](THEME.md). Edit `static/theme/tokens.css` for shared brand values and
`src/css/custom.css` for Docusaurus-specific presentation.
The Mermaid theme adapter in `src/theme/Mermaid/index.tsx` uses the same tokens
and fonts, with scrollable diagrams, a fit-width toggle, expanded viewing, and
SVG download in both color modes. Edit diagram source in its Markdown page; Pyreverse is a separate
code-derived diagnostic tool, not the source of the architecture chapters.

From the repository root:

```bash
npm --prefix website ci
uv sync --locked --group docs --inexact --python 3.12
npm --prefix website run docs:dev
```

Install Graphviz as described in
[Documentation generation](../docs/contribute/documentation-generation.md).
The start and build commands check the published API documentation contracts,
then regenerate Python API pages and scoped UML.
Use `docs:api` or `docs:uml` for one generator, or `docs:generate` for both. Set
`MODSIM_DOCS_PYTHON` when your documentation interpreter is outside `.venv`.

Authored API configuration lives in `website/api/`; UML generation lives in
`tools/docs/`. AutoAPI parses the selected Python source without importing
ModSim or its optional dependencies. Sphinx and sphinx-markdown-builder produce
Markdown module pages, which the generator materializes under the ignored
`docs/reference/api/` directory. The same docs plugin renders them at routes
such as `/ModSim/docs/reference/api/modsim_robotics/runtime/session/`.

Intermediate API files stay under `website/.generated/api/`. UML SVG and DOT
assets are staged under `website/.generated/site/`, which is mounted as a
public static directory. Generated content is neither committed nor edited
manually; the API Markdown mirror is also excluded from Python distributions.
Edit source signatures/docstrings, generator configuration, or authored pages
instead.

Contributors must follow the
[API documentation rules](../docs/contribute/api-documentation.md).
`npm --prefix website run docs:check` validates the current published API scope;
the documentation CI job runs that check and builds the complete site on pull
requests and pushes to `main`. Publishing the built website remains a separate
setup stage.

Authored pages are read directly from `docs/`, so saving them triggers live
reload. After changing Python source, run
`npm --prefix website run docs:api` in another terminal and refresh the page.
Docusaurus watches the regenerated Markdown; it does not rerun Python
extraction on source changes.

The local documentation opens at <http://localhost:3000/ModSim/docs/>. The
development server binds to `127.0.0.1`; stop it with **Ctrl+C**.

Node and npm dependencies are isolated here. This scaffold uses Node 22 and npm 10,
with the tested Node version recorded in `.nvmrc`. Python runtime dependencies,
GitHub Pages settings, and the public website are outside this local setup step.

## Dependency maintenance

`package.json` overrides three transitive dependencies to patched releases:
Tinypool 2.1.2 for Docusaurus workers, serialize-javascript 7.1.3 for webpack
serialization, and uuid 11.1.1 for SockJS. The selected releases retain the APIs
used by their callers and support this project's Node version. Review these
overrides when updating Docusaurus and remove them once its dependencies include
the fixes.

The classic preset also installs deepmerge 4.3.1 through its SVG optimization
plugin. npm reports five low-severity findings stemming from one advisory for
that package; the registry has no patched release at the time of setup. Track it
during dependency updates.

Upstream advisories: [Tinypool](https://github.com/advisories/GHSA-85c8-ppgw-ccpr),
[serialize-javascript](https://github.com/advisories/GHSA-m5mm-p8h7-m29v),
[uuid](https://github.com/advisories/GHSA-w5hq-g745-h8pq), and
[deepmerge](https://github.com/advisories/GHSA-gcqx-w48m-w2h4).
