# Documentation branding assets

This is the documentation site's curated subset of **ModSim brand asset release
5.0 / Final Honeycomb System**, prepared 2026-11-17. Files are copied unchanged
from the [packaged production assets](../../../src/modsim_robotics_studio/assets/branding/README.md),
whose inventory verifies the supplied theme packs. This asset release number is
independent of the Python package version.

The documentation's light mode uses **Light / Glacier & Steel**. Its dark mode
uses **Midnight / Deep Navy & Cyan**. Blue Charcoal remains a separate desktop
application theme; the site's two-mode switch does not select it.

Each theme directory preserves the production pack's relative paths:

| Path | Use |
| --- | --- |
| `logos/svg/modsim-horizontal.svg` | Original outlined symbol and wordmark, without a subtitle |
| `icons/svg/modsim-symbol.svg` | Compact symbol for constrained interface slots |
| `backgrounds/robot-cover-1920x1080.png` | Approved robot cover, including its logo and illustrative simulation window |
| `patterns/module-pattern.svg` | Supporting honeycomb motif for decorative areas |

The existing `../img/modsim-logo-light.svg` and
`../img/modsim-logo-dark.svg` paths contain the corresponding complete
horizontal logos. Their aspect ratio is **5:1**; render them proportionally,
with a suggested minimum width of 160 px. The shared `../favicon.svg` is the
supplied Light app icon, copied without modification.

The cover already contains a logo. Preserve the original artwork and avoid
adding a second logo over it. Keep patterns and scene artwork away from reading
surfaces. The pictured robots and floating simulation window are illustrative
brand artwork, not an application screenshot or CAD validation.

[`asset-manifest.json`](asset-manifest.json) records each copied file's source,
byte count, and SHA-256, including the existing logo and favicon paths. The
manifest itself is excluded from its file inventory. Copy replacements from the
approved production packs, verify their source hashes, and update this inventory
when changing assets. No source files are read from the user's Documents folder
by the built website.

The site's semantic colors are mapped in `../theme/tokens.css`; Docusaurus
presentation remains in `../../src/css/custom.css`. Use base-aware asset URLs
when referencing these files from components.
