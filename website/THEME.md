# ModSim documentation theme

The documentation follows **ModSim brand asset release 5.0 / Final Honeycomb
System**. Docusaurus light mode uses Light / Glacier & Steel; dark mode uses
Midnight / Deep Navy & Cyan. Blue Charcoal remains the additional Dark theme
in Studio and the Runtime Inspector. The existing documentation layout,
content routes, and light/dark switch remain in place.

The contributor-facing identity and artwork rules are in
[`docs/contribute/branding.md`](../docs/contribute/branding.md). The production
palettes and supplied guidelines are packaged under
`src/modsim_robotics_studio/assets/branding/`. Brand asset version 5.0 is
independent of the Python distribution version.

## Theme files

| File | Responsibility |
| --- | --- |
| `static/branding/{light,midnight}/` | Theme-matched production assets with pack-relative filenames and local provenance |
| `static/theme/tokens.css` | Framework-independent `--modsim-*` roles for both documentation modes |
| `static/theme/fonts.css` | Local font declarations imported by the token stylesheet |
| `src/css/custom.css` | Docusaurus and Infima mappings, layout, navigation, and content styling |
| `src/theme/prism.ts` | Code-block syntax colors expressed through the shared CSS variables |
| `src/theme/Mermaid/index.tsx` | Token-based diagrams, expanded inspection, and SVG download |
| `static/img/modsim-logo-light.svg`, `static/img/modsim-logo-dark.svg` | Compatibility paths for the matching outlined wordmarks |
| `static/favicon.svg` | Production module mark used as the browser icon |
| `docusaurus.config.ts` | Theme wiring, asset selection, and content inclusion |
| `scripts/serve.mjs` | Local production preview with base-aware routes and font asset handling |

Each theme's `theme/colors.json` supplies its exact production colors. The web
token stylesheet adapts those values into reading, navigation, control, and
syntax roles. `custom.css` translates the shared roles into Infima variables
and stable Docusaurus theme classes. Prefer these classes to generated CSS
module names, which can change between builds.

## Palette roles

Light uses a white reading and navbar surface with an icy-blue navigation
background. Midnight uses a deep navy reading canvas and a raised navy
navigation surface. Keep body text, examples, API tables, and diagrams on calm
surfaces; honeycomb belongs to branding and cover artwork.

| Web role | Light | Midnight |
| --- | --- | --- |
| Canvas | `#E8F3FA` | `#071427` |
| Surface | `#FFFFFF` | `#0D2339` |
| Raised surface | `#D4E7F2` | `#143550` |
| Main text | `#0B1E3A` | `#F2F6FB` |
| Muted text | `#425E7C` | `#B4CDDF` |
| Primary action | `#087F79` | `#00BFAF` |
| Action text | `#FFFFFF` | `#071427` |
| Decorative border | `#9AB9CE` | `#245373` |
| Control border | `#526D82` | `#6896B7` |
| Focus | `#1169A2` | `#50DEF1` |
| Link | `#126A9C` | `#2AA8FF` |
| Disabled control | `#829AAC` | `#466A88` |

Use `--modsim-control-border` for functional boundaries and
`--modsim-border` for decorative separators. Use `--modsim-focus` for keyboard
focus, `--modsim-link` for links, and `--modsim-disabled` for inactive controls.
Brand blue, cyan, and secondary accents remain available through
`--modsim-blue`, `--modsim-cyan`, and `--modsim-secondary-accent`. Pair action
fills with their `accent_text` value, especially in Light where the decorative
bright teal is different from the darker primary-action fill.

Light values are defined on `:root`; dark overrides use
`[data-theme='dark']`. Keep both modes complete when adding a role. Syntax and
diagnostic colors serve distinct meanings and must remain readable on the
chosen surfaces. Do not extract colors from lighting variations in cover art.

## Logos and typography

The navbar uses the supplied horizontal SVG wordmark with a separate **Docs**
label. Its lettering is outlined artwork, not a guessed font. Preserve its
proportions and clear space; switch to the corresponding production asset
with the color mode. Use supplied theme variants rather than recoloring or
inverting a logo. The source guideline recommends at least 160 px for the
horizontal lockup and 24 px for the standalone symbol.

Archivo supplies headings, IBM Plex Sans body text, and IBM Plex Mono code.
These existing fonts remain self-hosted with their licenses. The brand guide
allows an application's sans-serif for supporting copy; it does not identify
the typeface behind the logo. No third-party font request or image-hosting
service is required by this theme.

## Artwork and layout

Use the approved theme-matched robot covers where a cover is appropriate. The
two SMORES configurations and illustrated simulation window are brand art,
not hardware verification or a screenshot of the application. Preserve the
source files and provenance. A cover already contains its logo; do not paste
a second logo over it. Desktop loading and welcome screens blend live controls
over the full-size cover with a soft lower fade. Documentation reading surfaces
remain plain rather than inheriting that cover treatment.

The desktop documentation layout retains its 300 px sidebar, bounded reading
column, generous spacing, and right-hand table of contents. Mobile navigation
uses the Docusaurus responsive sidebar. Navigation is controlled by
`sidebars.ts`; the theme does not invent search or version controls, or add page
headings merely to populate a table of contents.

The local production preview uses `scripts/serve.mjs`. It serves actual files
before normalizing page routes so self-hosted `.woff2` assets work under the
configured base URL.

## Make and review a theme change

1. Read the production JSON palette and source guidelines for the selected pack.
2. Update shared roles in `static/theme/tokens.css`, preserving their meaning.
3. Update Docusaurus selectors in `src/css/custom.css` only where adaptation is needed.
4. Refresh matching fixed-color assets and their provenance together.
5. Preview an existing page in both modes before editing its authored wording.

Keep prose, heading structure, and examples in `docs/`. Preserve manual edits
when adjusting presentation. The brand guide page must be registered in the
explicit content include list and the Contribute sidebar.

From the repository root:

```bash
npm --prefix website run docs:dev
```

Check API policy, configured types, and production output with:

```bash
npm --prefix website run docs:check
npm --prefix website run typecheck
npm --prefix website run docs:build
npm --prefix website run docs:serve
```

Review the navbar and sidebar, focus states, body text, links, inline code,
code blocks, API tables, a Mermaid diagram, and the table of contents. Check
desktop and narrow widths in both modes. Record actual verification results
and user visual review in `SETUP_PLAN.md`.

## Architecture diagrams

The Mermaid adapter reads resolved shared tokens after Docusaurus applies the
color mode. It supplies colors for boxes, relationships, actors, messages,
notes, and labels. Diagrams follow the manual rather than embedding an unrelated
palette or an opaque white SVG background.

Diagrams retain readable native dimensions in a scrollable card. **Fit width**
shows the complete structure; **Actual size** restores native labels.
**Expand** opens a wider view and **Download SVG** exports the current themed
graphic. Keep captions and responsibility tables alongside the diagrams.
Prefer several focused diagrams to a repository-wide graph with tiny labels.

Authored Mermaid conveys architectural meaning. Verify names and relationships
against current source. The optional Pyreverse output remains a scoped static
analysis aid rather than the primary architecture illustration.

## Reuse and deployment scope

`tokens.css`, `fonts.css`, and the production asset subset can support a future
homepage or API theme. Each consumer supplies its own adapter, preserves valid
relative font paths, and uses the same `data-theme` convention. Static assets
remain local to the site and work with the existing GitHub Pages base URL.

This repository-local documentation update does not deploy a public website or
modify the separate website branch. Homepage migration and publication remain
separate steps.
