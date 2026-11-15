# ModSim Documentation Theme

The local documentation uses ModSim's website palette, typography, and glyph with
a documentation layout informed by the Hasura quickstart. The theme changes
presentation while authored pages remain in `docs/` for manual editing.

## Theme files

| File | Responsibility |
| --- | --- |
| `static/theme/tokens.css` | Framework-independent `--modsim-*` design tokens for light and dark modes |
| `static/theme/fonts.css` | Local font declarations imported by the token stylesheet |
| `src/css/custom.css` | Docusaurus and Infima mappings, layout, navigation, and content styling |
| `src/theme/prism.ts` | Code-block syntax colors expressed through the shared CSS variables |
| `src/theme/Mermaid/index.tsx` | Architecture diagrams using the shared tokens, expanded inspection, and SVG download |
| `static/img/modsim-logo-light.svg` | Logo for the light documentation theme |
| `static/img/modsim-logo-dark.svg` | Logo for the dark documentation theme |
| `static/favicon.svg` | Original ModSim glyph used as the browser icon |
| `docusaurus.config.ts` | Theme wiring, logo selection, and Prism configuration |
| `scripts/serve.mjs` | Local production preview with base-aware routes and font asset handling |

The token stylesheet is the shared visual source. The Docusaurus stylesheet is
the adapter that translates those tokens into Infima variables and stable theme
classes. Prefer these classes to generated CSS module names, which can change
between builds or framework releases.

## Palette and typography

The palette comes from ModSim's existing website source. Its light mode uses a
warm `#F7F6F2` page background, `#142030` text, and `#1F4E8C` blue accents. Dark
mode uses a `#0B1521` background, `#E5ECF2` text, and `#79AEEA` blue accents.
Teal provides the complementary accent in both modes. The token file contains
the complete surface, border, muted-text, and code-color definitions.

Archivo supplies display typography, IBM Plex Sans supplies body text, and IBM
Plex Mono supplies code. Font files are self-hosted so local documentation does
not depend on Google Fonts requests. Keep the accompanying licenses with font
assets when updating them.

Light values are defined on `:root`; dark overrides use
`[data-theme='dark']`. Keep both modes complete when adding or changing a token.

## Layout behavior

The desktop layout uses a 300 px documentation sidebar, a bounded reading
column, generous spacing, and a right-hand table of contents for pages with
headings. The navbar stays compact. Mobile navigation uses Docusaurus's
responsive sidebar and keeps the content readable without a desktop-width rail.

The actual sidebar content remains controlled by `sidebars.ts`. Search and
version controls should appear when their corresponding functionality is
implemented. A page with no section headings does not need an invented table of
contents or extra headings added solely for the layout.

The local production preview uses `scripts/serve.mjs`. Docusaurus 3.10.2's preview
command misidentifies `.woff2` paths under a base URL and adds a trailing slash.
The wrapper uses the same route configuration and static file handler, serving
actual files before normalizing page routes so the self-hosted fonts load.

## Make a theme change

1. Change shared colors, fonts, or radii in `static/theme/tokens.css`.
2. Change Docusaurus-specific spacing or selectors in `src/css/custom.css`.
3. Update `src/theme/prism.ts` only when adding or changing syntax-token mappings.
4. Update logo assets and configuration together when changing the glyph.
5. Preview an existing page in both modes before changing authored content.

Keep page wording, heading structure, and examples in `docs/`. Preserve manual
edits there while adjusting the presentation in the theme files.

From the repository root, start the live preview with:

```bash
npm --prefix website run docs:dev
```

Check the configured types and production output with:

```bash
npm --prefix website run typecheck
npm --prefix website run docs:build
npm --prefix website run docs:serve
```

Review the navbar and sidebar, focus states, body text, links, inline code, code
blocks, and any table of contents. Check narrow and desktop widths in both
color modes. Record actual verification results and the user's visual review in
the S2 theme checklist in [SETUP_PLAN.md](SETUP_PLAN.md).

## Reuse the visual tokens

`tokens.css` and `fonts.css` can be reused by a future homepage or API theme.
Each consumer supplies its own adapter and uses the same `data-theme` convention.
Keep the relative font paths valid when moving or copying the shared files.

The existing website branch is a visual reference for this local work. It is
not changed or automatically restyled by these files; homepage migration and
deployment remain later setup stages.

## References

- [Existing ModSim website](https://modlab-upenn.github.io/ModSim/)
- [Hasura quickstart layout](https://hasura.io/docs/3.0/quickstart/)
- [Docusaurus styling and layout](https://docusaurus.io/docs/styling-layout)
- [Docusaurus code-block theming](https://docusaurus.io/docs/markdown-features/code-blocks)

## Architecture diagrams

The Mermaid theme adapter reads the resolved colors from `tokens.css` after the
Docusaurus color mode is applied. It supplies hex colors to Mermaid's base theme
for class boxes, relationships, actors, sequence messages, notes, and labels.
Diagrams therefore change with the same light/dark theme as the manual instead
of using a separate neutral palette or an opaque white SVG background.

Diagrams retain readable native dimensions inside a scrollable card. **Fit width**
shows the complete structure; **Actual size** restores readable native labels.
**Expand** opens a modal view for wider class or sequence diagrams;
**Download SVG** exports the current themed vector graphic. The captions and responsibility tables in
the authored page remain necessary context for the diagram. Prefer several
focused diagrams to a whole-repository graph with tiny labels.

Use authored Mermaid for architectural meaning and verify names/relationships
against current source. The optional Pyreverse output remains a scoped static
analysis aid; it is not the primary illustration for the architecture chapters.
