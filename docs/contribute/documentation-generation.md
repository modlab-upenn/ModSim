---
title: Documentation generation
sidebar_label: API and diagrams
---

The manual combines authored pages and Mermaid diagrams with API pages extracted
from Python source. The Explanation chapters use curated UML, sequence diagrams,
state diagrams, and flowcharts to explain responsibilities and behavior. A
separate Pyreverse generator provides a small code-derived diagnostic view.
Keep prose, diagram source, and generator configuration in Git; regenerate
derived output locally.

## Install the generation tools

Use the repository checkout and the Node environment from
[Local development](local-development.md). From the repository root, install
the locked Python documentation group:

```bash
uv sync --locked --group docs --inexact --python 3.12
```

The `--inexact` option preserves any Studio, MuJoCo, or development tools already
installed in the shared environment. The docs group contains Sphinx,
Sphinx-AutoAPI, sphinx-markdown-builder, and Pylint's Pyreverse tool. These are
documentation dependencies; they are separate from ModSim's package runtime
requirements and Ruff checks.

Install Graphviz so that its `dot` executable is on `PATH`. On Ubuntu/Debian:

```bash
sudo apt install graphviz
```

On macOS with Homebrew:

```bash
brew install graphviz
```

For Windows, use the installer linked from
[Graphviz's download page](https://graphviz.org/download/) and add its `bin`
directory to `PATH`. Open a new terminal after changing `PATH`.

Check the renderer before generating diagrams:

```bash
dot -V
```

Installing the Python package named `graphviz` does not install this native
executable. The generator reports a missing prerequisite instead of silently
omitting a diagram.

For this checkout's initial verification, a Linux renderer is also available in
the ignored `.local-tools/graphviz/` directory. The UML script uses its
`usr/bin/dot` and matching libraries when `dot` is absent from `PATH`; no system
packages are changed. A fresh checkout should install Graphviz as above.

## Generate and preview

Generate both the API reference and scoped UML diagrams:

```bash
npm --prefix website run docs:generate
npm --prefix website run docs:dev
```

Development and production build commands also generate this content before
starting Docusaurus. To rerun one generator through the shared launcher:

```bash
npm --prefix website run docs:api
npm --prefix website run docs:uml
```

The launcher uses the repository's `.venv` Python. Set `MODSIM_DOCS_PYTHON` to
another documentation interpreter when using a different environment. To invoke
an individual generator directly from the repository root:

```bash
uv run --no-sync python website/api/build.py
uv run --no-sync python tools/docs/generate_diagrams.py
```

API extraction produces Markdown that Docusaurus renders as ordinary pages in
the same manual. Pyreverse images are available as static assets; the authored
architecture diagrams render directly from their pages' Mermaid blocks:

| Output | Site location |
| --- | --- |
| Generated Markdown in `docs/reference/api/` | `/ModSim/docs/reference/api/` |
| UML SVG and DOT in `website/.generated/site/generated/diagrams/` | `/ModSim/generated/diagrams/` |

Intermediate Sphinx files stay under `website/.generated/api/`; Pyreverse
staging also stays under `website/.generated/`. The generated
`docs/reference/api/` directory is an ignored, temporary mirror for the docs
plugin. It contains no authored pages and is excluded from Python
distributions. Do not edit or commit generated Markdown, HTML, DOT, or SVG files.

Docusaurus reads authored pages directly from `docs/`, so saving an authored
page still triggers live reload. It also watches the generated API Markdown.
After changing a Python signature or docstring, run `docs:api` in another
terminal and refresh the page; the development server does not rerun Python
extraction automatically. Use `docs:uml` after changing diagram inputs. Stop
and restart the development server if a regenerated static image remains stale.

## Update the API reference

Edit docstrings and signatures in `src/`, then regenerate. The first reference
covers a deliberately small runtime slice described on the
[Python API landing page](../reference/python-api.mdx). Extraction reads source
statically rather than importing Studio or simulator modules.

Follow the [API documentation contract](api-documentation.md) when changing a
public API. Use Google-style `Args`, `Returns`, `Raises`, and `Examples` sections
where applicable; Sphinx Napoleon converts them for the renderer. Keep parameter
and return types in the Python annotations. The checked published scope and
its static helper dependencies are defined in `website/api/scope.py`.

Sphinx configuration and the authored API introduction live in `website/api/`.
AutoAPI parses the selected modules, then sphinx-markdown-builder converts
Sphinx's reference output to Markdown. The generator places the module pages
under `docs/reference/api/`, where the same Docusaurus docs plugin loads them
alongside the manual. They use its theme, Documentation sidebar, breadcrumbs,
and table of contents under **Reference → Python API**. For example, the
runtime session module is served at
`/ModSim/docs/reference/api/modsim_robotics/runtime/session/`.

The module template in `website/api/templates/python/module.rst` orders the
public entries. `website/api/markdown.py` renders resolved API nodes as short
headings, highlighted Python signatures, parameter tables, return values,
exceptions, and examples. It reads argument names, types, and defaults from
AutoAPI and Sphinx metadata; descriptions come from the source docstrings.
Change these generator sources to adjust API presentation, then regenerate.

The Sphinx build treats warnings as errors so unresolved documentation problems
fail generation before Docusaurus builds the site. To change extraction scope
or formatting, edit the configuration and build script in `website/api/`, then
regenerate. Edit the API landing page in `docs/reference/python-api.mdx` for
authored guidance. Source docstrings remain the place to change individual API
descriptions.

## Update the code-derived UML scope

The initial class and package diagrams cover exactly these files:

```text
src/modsim_robotics/core/assemblies.py
src/modsim_robotics/core/entities.py
src/modsim_robotics/core/state.py
```

`tools/docs/generate_diagrams.py` passes that ordered list to Pyreverse. It
requests DOT output, includes private fields when inferring relationships, and
hides member lists and disconnected classes to keep the overview small. Class
labels are short names; DOT node IDs retain their qualified module paths.
Graphviz then renders each DOT file to SVG using
[`dot -Tsvg`](https://graphviz.org/doc/info/command.html).
The script changes Pyreverse's edge-label color for readable contrast; source
relationships are retained.

These generated SVGs are diagnostic assets at the site locations above. The
[runtime state chapter](../explanation/runtime-model.mdx) now uses authored
Mermaid diagrams with selected members and explicit ownership labels; it does
not embed the Pyreverse images. Static inference alone cannot explain ordering,
state authority, or the reason for a dependency.

Change the generator's scope when inspecting another class relationship, then
review its output against source. Keep each scope small. Pyreverse's
[scope and display options](https://pylint.pycqa.org/en/latest/additional_tools/pyreverse/configuration.html)
describe the available controls.

Repeated generation from unchanged sources with the same locked Python tools,
Graphviz version, and fonts should produce the same output. Graphviz layouts
can differ between renderer versions or platforms; review meaning and
readability rather than treating SVG byte identity across machines as a
compatibility guarantee.

## Add authored diagrams

Use fenced `mermaid` blocks beside the prose they explain. Choose a diagram for
the question a reader needs answered:

| Diagram | Use it to explain |
| --- | --- |
| `flowchart` | System boundaries, dependencies, data paths, or a decision pipeline |
| `classDiagram` | Class responsibilities, selected operations, ownership, and relationships |
| `sequenceDiagram` | Calls and messages in time order, including process boundaries and failure paths |
| `stateDiagram-v2` | Legal lifecycle states, transitions, and their triggers |

The [architecture overview](../explanation/index.md) provides the reading order;
subsystem chapters pair structural diagrams with behavioral sequences. Keep a
diagram focused on one question rather than drawing every method in a module.
Use actual class and method names, distinguish owned state from generated views,
and explain arrows and omissions in the surrounding text. Check each changed
relationship or sequence against source and the relevant current contract.
Architecture meaning is reviewed by people; API extraction and Pyreverse do
not automatically update these authored diagrams.

The shared renderer in `website/src/theme/Mermaid/index.tsx` reads the site's
light/dark tokens and fonts. It preserves readable native SVG dimensions within
a scrollable panel and provides **Fit width**, **Expand**, and **Download SVG**
controls. Use actual size to read labels and fit width to see the whole
structure. Avoid per-diagram colors, inline theme overrides, and screenshots of diagrams. Update
shared colors in `website/static/theme/tokens.css` and presentation in
`website/src/css/custom.css`; see `website/THEME.md` for the theme contract.

Saving an authored Mermaid block triggers Docusaurus live reload without
running `docs:uml`. Review the rendered page in both themes and at narrow widths;
check that labels, arrow directions, and expanded viewing remain readable.
Pyreverse diagrams remain a separate view of inferred implementation structure.

PlantUML remains an available future choice for diagrams needing richer UML
notation; it is not a dependency of this first setup.

## Verify the combined output

```bash
npm --prefix website run docs:check
npm --prefix website run typecheck
npm --prefix website run docs:build
npm --prefix website run docs:serve
```

The policy check validates source documentation in the published API slice;
the build regenerates the reference and validates the manual. CI runs these
checks as part of contribution review. Human review still checks the meanings
of descriptions and the accuracy of examples.

Review a generated API module from **Reference → Python API**, its sidebar and
object links, the authored architecture diagrams, and any changed Pyreverse output in the
local production preview. Stop the server with **Ctrl+C** when finished. This
produces local static files; publication remains a separate workflow.
