---
title: Local documentation development
sidebar_label: Local development
---

The combined website reads manual pages from `docs/` and the homepage from
`website/src/pages/index.tsx`. Its configuration, navigation, browser demos,
and Node dependencies live in `website/`. You can edit and preview the site
without running a simulator. Python documentation tools generate
the API reference and UML diagrams before the application starts or builds.

## Install the documentation dependencies

Use Node 22 and npm 10. The exact Node version used for the initial setup is
recorded in `website/.nvmrc`. If you use nvm, select it from the repository root:

```bash
cd website
nvm install
nvm use
cd ..
```

Check your environment and install the versions recorded in the lockfile:

```bash
node --version
npm --version
npm --prefix website ci
```

`npm ci` installs `website/node_modules/` from `website/package-lock.json`. Run it
after the lockfile changes or when setting up another checkout. Follow
[Documentation generation](documentation-generation.md) to install the Python
`docs` dependency group and Graphviz before starting the complete site. Project
sync also installs the default Studio dependencies. API extraction and diagram
generation read Python source without importing Studio or MuJoCo or opening a GUI;
the MuJoCo extra is only needed when running its physics backend.

## Start the live preview

Run this command from the repository root:

```bash
npm --prefix website run docs:dev
```

Open <http://localhost:3000/ModSim/> for the homepage or
<http://localhost:3000/ModSim/docs/> for the manual. If port 3000 is occupied, select another
port explicitly and use the URL printed by the server:

```bash
npm --prefix website run docs:dev -- --port 3001
```

Keep the command running while editing. **Ctrl+C** stops the server.
The command generates the API pages and UML assets once before starting.
After changing Python signatures or docstrings, run
`npm --prefix website run docs:api` in another terminal and refresh the page.
Use `docs:uml` after changes to diagram inputs, or `docs:generate` for both.

## Edit a page and understand the configuration

Open `docs/index.md`, change a sentence, and save it. Docusaurus reloads the page.
Homepage content lives in `website/src/pages/index.tsx`, with its layout in
`index.module.css`. The React/SVG demonstrations and recorded replay data live
under `website/src/components/home/`; videos and their provenance are in
`website/static/media/`. These share the manual's theme and navigation.
Use ordinary `.md` files for prose; `.mdx` files will be available when a page needs
React components. The `markdown.format: 'detect'` setting makes that distinction
explicit.

The Documentation sidebar contains Overview, Installation, Tutorials, How-to
guides, Explanation, and Reference.
Contributor guides live in the separate Contribute sidebar. When adding an
authored page:

1. Check the docs `include` list in `website/docusaurus.config.ts`. How-to and
   Explanation pages are included by directory patterns; add an entry only
   when the new path is not already covered.
2. Add its document ID to `website/sidebars.ts` to place it in the navigation.

A document ID is its path relative to `docs/` with the extension removed. For
example, `docs/tutorials/getting-started.md` has the ID
`tutorials/getting-started`. Add the ID to the sidebar for its intended readers.
Generated API pages are included by their directory pattern and appear under
**Reference → Python API**; they use the same docs plugin, theme, and sidebar as
the authored manual. Do not add these generated module pages individually.

`path: '../docs'` selects the content directory on disk. `routeBasePath: 'docs'`
selects its URL prefix. The site's `baseUrl: '/ModSim/'` adds the repository
prefix, so the documentation overview appears at `/ModSim/docs/`. The overview's
front matter sets `slug: /` to make it the documentation landing page.

For links between pages, use relative Markdown links such as
`[Overview](../index.md)`. Docusaurus resolves these links to the correct route.
Absolute browser paths need care because `/docs/` omits the `/ModSim/` prefix.

The existing contract files retain their current paths. Historical content under
`docs/archive/` and `docs/technical_report/` is explicitly excluded from the current
documentation application.

## Check and preview the production output

Run the TypeScript check and production build:

```bash
npm --prefix website run typecheck
npm --prefix website run docs:build
```

The TypeScript check covers website configuration and components. The production
build compiles the included pages, validates internal links, and writes the
static output into `website/build/`.
The build first regenerates the Python API and UML diagrams, then includes
the API Markdown as native documentation pages and the UML images as static
assets.

Preview that built output with:

```bash
npm --prefix website run docs:serve
```

Open the printed URL with `/ModSim/`, visit the manual under `/ModSim/docs/`,
follow the navigation links, and refresh a
nested page. Stop the server with **Ctrl+C**. Unlike the live development server,
this preview shows the last production build; rebuild after changing content.

These build and serve commands operate locally. The
[GitHub Pages guide](github-pages.md) explains the build-only branch artifacts,
isolated branch hosting, required repository settings, and explicit publication.

## Know which files to edit

| Location | Purpose |
| --- | --- |
| `docs/**/*.md`, `docs/**/*.mdx`, except `docs/reference/api/` | Authored documentation pages |
| `docs/reference/api/` | Ignored generated API Markdown loaded by the same docs plugin |
| `website/docusaurus.config.ts` | Content inclusion, routes, theme, and link checks |
| `website/sidebars.ts` | Sidebar sections and ordering |
| `website/src/` | Application components and styles |
| `website/src/pages/index.tsx` | Project homepage |
| `website/src/components/home/` | Browser demonstrations and recorded replay data |
| `website/static/media/` | Existing project recordings and provenance |
| `website/site-config.ts` | Production/preview origin and base-path validation |
| `website/package.json` | Commands and direct dependencies |
| `website/package-lock.json` | Exact dependency resolution for `npm ci` |
| `website/api/` | AutoAPI and Markdown generation sources, build script, and configuration |
| `tools/docs/` | Scoped UML generation from Python source |
| `website/.generated/` | Ignored API and diagram staging output |
| `website/SETUP_PLAN.md` | Setup checklist, evidence, and review status |
| `website/.docusaurus/`, `website/build/` | Generated application files and output |

Keep source files and lockfiles in Git. Generated directories, `node_modules/`,
`website/.generated/`, and `docs/reference/api/` are ignored. The generated API
mirror is temporary output and is excluded from Python distributions. Edit the
Python source, generator configuration, and authored introductions to change
the reference, then regenerate it. Saving an authored page triggers live reload
directly; saving a Python source file requires the `docs:api` step first.
Authored Mermaid diagrams live in their Markdown pages and reload with the
prose. Their shared theme and expanded viewer live in
`website/src/theme/Mermaid/index.tsx`; they do not require Pyreverse generation.

If a configuration change leaves a stale cache, stop the server and clear the
generated application cache before restarting:

```bash
npm --prefix website run docs:clear
```

For the stage checklist, open `website/SETUP_PLAN.md` in your editor. Technical
completion and your walkthrough review are tracked separately there.
