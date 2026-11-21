# ModSim website and documentation development

This directory contains one Docusaurus application for the project homepage
and manual. The homepage is `src/pages/index.tsx`; authored manual pages live
in [`../docs`](../docs/) and are read directly. Both share navigation, theme,
fonts, logos, and one static build.

Open <http://localhost:3000/ModSim/> for the homepage and
<http://localhost:3000/ModSim/docs/> for the manual after starting the local server.
The interactive React/SVG demos live in `src/components/home/`. Existing videos
are kept under `static/media/`, with source and hash provenance. The legacy
`quickstart.html` URL forwards to the appropriate current guide, including old
section bookmarks.

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
ModSim, Studio, or simulator libraries. The default Python project installation
includes Studio; the generators do not initialize its GUI. Sphinx and
sphinx-markdown-builder produce Markdown module pages, which the generator
materializes under the ignored
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
requests and pushes to `main`. The separate **Website and Pages** workflow
builds relevant branch pushes and pull requests, uploads downloadable previews,
and publishes only after explicit opt-in on the configured deployment branch.
Follow [Website preview and GitHub Pages](../docs/contribute/github-pages.md)
for an isolated hosted `refactor` preview, repository settings, main-branch
cutover, and rollback. The existing public site is unchanged by local work or
build-only runs.

Authored pages are read directly from `docs/`, so saving them triggers live
reload. After changing Python source, run
`npm --prefix website run docs:api` in another terminal and refresh the page.
Docusaurus watches the regenerated Markdown; it does not rerun Python
extraction on source changes.

The development server binds to `127.0.0.1`; stop it with **Ctrl+C**.

Node and npm dependencies are isolated here. This scaffold uses Node 22 and npm 10,
with the tested Node version recorded in `.nvmrc`. Python dependencies are locked
in the repository's `uv.lock`. Local builds do not change GitHub Pages settings.

To check publication guards and base-aware legacy links, run
`npm --prefix website run site:check`. To keep two production previews separate,
build the alternate address with `docs:build -- --out-dir .generated/preview-build`
and serve it with `docs:serve -- --dir .generated/preview-build --port 3198`,
keeping the same `MODSIM_SITE_URL` and `MODSIM_BASE_URL` set for both commands.
The main build in `website/build/` remains available at its usual address.

For a preview repository, set `MODSIM_SITE_URL` and `MODSIM_BASE_URL` before both
building and serving. Defaults are `https://modlab-upenn.github.io` and `/ModSim/`.
The Pages workflow derives these values from its own repository, so a separate
`ModSim-preview` project uses `/ModSim-preview/`. Run the publication-setting
checks with `npm --prefix website run site:check`.

## RobotPack Library link

The homepage introduces the community RobotPack Library. Its optional external
link is configured at build time, separately from the ModSim website address.
For the library currently running locally, use:

```bash
MODSIM_ROBOTPACK_LIBRARY_URL=http://localhost:3214 npm --prefix website run docs:dev
```

Set the same variable before `docs:build` to include that address in a local
production preview. For GitHub Pages, set the Actions repository variable
`MODSIM_ROBOTPACK_LIBRARY_URL` to the library's public HTTP(S) URL before building.
When unset, the section retains its explanation and guide links but omits the
external library link. No localhost address is included by default in a public
build. Restart the development server or rebuild after changing the setting.

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
