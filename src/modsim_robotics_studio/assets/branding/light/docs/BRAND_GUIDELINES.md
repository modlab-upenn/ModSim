# ModSim Brand Guidelines — Final Honeycomb System

**Brand asset release 5.0 · 17 November 2026.** This number describes the asset library, not the ModSim Python package.

## 1. Approved direction

The identity combines the original rounded-square module symbol and outlined ModSim wordmark with a **hexagonal / honeycomb environment**. Light is now **Glacier & Steel**: white and icy-blue interface surfaces with dark navy type, a visible steel-blue secondary accent, and a deeper blue stage behind the white SMORES modules. Dark and Midnight retain their approved visual assets and covers.

Honeycomb is a supporting pattern, not a replacement logo. Use matte or softly lit blue hexagonal cells, straight connection lines, restrained teal details, and readable solid UI surfaces. Do not reintroduce the old sweeping waves, square-grid environment, purple palette, generic quadruped, exploded CAD views, reconfiguration arrow, or cover subtitle. The small grid inside the simulation-window illustration is part of that approved illustration and remains.

## 2. Logo and typography

Use `logos/svg/modsim-horizontal.svg` as the primary lockup. The lettering is the existing outlined wordmark from the preferred supplied reference; its original typeface has not been identified. Do not substitute a guessed font. SVG outlines display without installed fonts. No font files are included in the library.

Retain the symbol's three solid rounded modules, segmented reticle, proportions, and relative placement. Keep at least one solid module width of clear space where possible. Suggested minimums are 160 px for a horizontal logo and 24 px for the symbol; 16 px exports are supplied for constrained platform slots and should be checked at actual size. Do not stretch, add perspective, recolor automatically, or rebuild the mark as a hexagon.

Use the supplied theme-specific, monochrome, stacked, and wordmark-only variants. Descriptor-lockup exports are retained as optional compatibility assets; the default lockup and all covers have no subtitle. Inter or the application's existing sans-serif is suitable for supporting interface copy; that does not identify the logo font. Use normal text rendering for labels, not lettering cropped from a brand-board image.

## 3. Theme colors

| Role | Light / Glacier & Steel | Dark | Midnight |
| --- | --- | --- | --- |
| Canvas | #E8F3FA | #132331 | #071427 |
| Surface | #FFFFFF | #1C3042 | #0D2339 |
| Raised surface | #D4E7F2 | #263D51 | #143550 |
| Main text | #0B1E3A | #F1F6FB | #F2F6FB |
| Secondary text | #425E7C | #B8CBDC | #B4CDDF |
| Secondary accent | #425E7C | #93AEC5 | #5592C1 |
| Brand blue | #188FE5 | #53AEF3 | #2AA8FF |
| Brand teal | #00BFAF | #35CDBF | #00BFAF |
| Primary action | #087F79 | #35CDBF | #00BFAF |
| Action text | #FFFFFF | #0B2228 | #071427 |
| Control boundary | #526D82 | #7E9DB6 | #6896B7 |

`theme/colors.json` defines the exact software colors. CSS, Qt stylesheets, SVGs, PNGs, platform icons and state examples are exported from those values. The rendered hero contains lighting variation; do not sample its pixels to define UI tokens.

### Light: meaningful contrast, not white on gray

Use **#425E7C** for secondary text and secondary icons. Its calculated contrast against white is **6.72:1**. Use **#526D82** for functional control boundaries, with **5.43:1** against white. Primary text **#0B1E3A** has **16.65:1** against white. The action fill is dark teal **#087F79**, not the brighter decorative brand teal; white on this action fill is **4.86:1**.

The background is icy blue **#E8F3FA** rather than neutral light gray. Behind the white robot illustration, use the deeper steel-blue stage family **#426E8B / #6D99B2**. Bright blue and cyan are identity/decorative accents, not automatically suitable for small body text. Pale fills and disabled colors are not intended for meaningful active content. The included contrast reports are individual sRGB color-pair calculations, not application-wide accessibility certification.

## 4. Cover art and preservation

The production source is the approved **1671 × 941 dark-blue honeycomb cover**. It is retained in each pack at `reference/approved-smores-cover.png` and `backgrounds/robot-cover-approved-native.png`; the provenance JSON records its hash. This is the approved illustrative scene, not a CAD-certified model or an actual screenshot of running ModSim.

**Midnight** uses that native cover unchanged. **Dark** retains the approved muted blue-charcoal adaptation unchanged. **Light** recolors the honeycomb surroundings into an icy-blue upper field and steel-blue lower stage, reuses the original simulation window and robot foreground, and applies the navy vector logo. No robot geometry has been generated again for this release. Original fully opaque robot pixels are compared at native size; boundary blending and resized exports are not claimed to be pixel-identical.

Do not recolor the robot bodies, add AprilTags, invent module connections, substitute another robot, or put the colored exploded CAD assembly on a cover. Keep the two approved reconfigurations and the floating simulation-window motif. Loading examples use a static demonstration bar below the illustration; replace it with a real progress widget in the application.

| Path | Purpose |
| --- | --- |
| `backgrounds/robot-cover-theme-native.png` | Theme-specific native cover. |
| `backgrounds/robot-cover-1920x1080.png` | Main software/website cover export. |
| `backgrounds/robot-cover-960x540.png` | Smaller splash/preview export. |
| `backgrounds/smores-cover-fullbleed-*` | Matching compatibility aliases. |
| `backgrounds/robot-background-1920x1080.png` | Cover alias; includes logo and window, not an empty background. |
| `backgrounds/theme-background.svg` | Empty, scalable honeycomb background. |
| `artwork/smores-configurations-transparent-native.png` | Original robot foreground on a transparent full-size canvas. |
| `artwork/smores-configuration-compact.png` | Cropped original compact configuration. |
| `artwork/smores-configuration-raised.png` | Cropped original raised configuration. |
| `artwork/simulation-window.png` | Original isolated illustrative window. |

Raster exports larger than the native source are resampled, not newly modeled detail. The native original and all adaptation details are documented in `reference/artwork-provenance.json`.

## 5. Complete icon and pattern system

The toolbar includes 24 actions: camera, check, close, composer, connect, disconnect, dock, fit view, grid, info, modules, open folder, pause, play, reset, results, save, settings, stop, undock, visibility, warning, zoom in and zoom out.

SVG states are `default`, `accent`, `secondary`, `disabled`, and `currentcolor`. PNG states are default, accent, secondary and disabled, in 24, 48 and 96 px. Default raster icons keep the established path `ui/png/24/`, not an extra `default` subdirectory. Toolbar geometry uses a 24-unit viewBox and 1.8-unit strokes. `currentColor` SVGs are intended for inline/CSS-aware rendering; external img elements do not automatically inherit surrounding text color.

App/symbol raster sizes are 16, 20, 24, 32, 40, 48, 64, 96, 128, 192, 256, 512 and 1024 px. ICO, ICNS, Apple touch, favicon and maskable web variants are also supplied. Keep the symbol inside the maskable safe area. Use labels, shape and focus feedback alongside color for important interface states.

Patterns include `honeycomb-outline`, `honeycomb-material`, `honeycomb-tile`, `honeycomb-stage`, and `hex-connections`, each as SVG and 2x PNG. `module-pattern` and `connection-lines` retain their older filenames for compatibility but now contain hexagonal outlines and straight connections, not old curved motifs. Keep dense patterns away from controls and body copy.

## 6. Software integration

Every theme uses the same relative paths. Change the pack root and theme styles together. Replace the entire previous theme directory; do not mix the old light PNGs with the new CSS. Rebuild Qt resources and clear cached web assets after replacement.

```text
ModSim_Light_Pack/
ModSim_Dark_Pack/
ModSim_Midnight_Pack/
  logos/        icons/        ui/        patterns/
  backgrounds/  artwork/      theme/     previews/
  docs/         reference/    examples/
  index.html    resources.qrc asset-manifest.json
```

Open `START_HERE.html` in the combined bundle, or `index.html` in any standalone pack, to browse the full files locally. The catalog is offline and does not need a web service.

### Qt / PySide6

```python
from pathlib import Path
from PySide6.QtGui import QIcon
pack = Path("ModSim_Light_Pack")
app.setWindowIcon(QIcon(str(pack / "icons/svg/modsim-app.svg")))
app.setStyleSheet((pack / "theme/theme.qss").read_text(encoding="utf-8"))
play_button.setIcon(QIcon(str(pack / "ui/svg/default/play.svg")))
```

Resource prefixes are `:/modsim/light/`, `:/modsim/dark/` and `:/modsim/midnight/`. Run `pyside6-rcc resources.qrc -o resources_rc.py` from a pack root, then import the generated module. `examples/qt_theme_demo.py` is an optional styling demonstration; controls are not wired to a simulator. It is syntax-checked but has not been run inside ModSim.

### Web

Load `theme/theme.css` for one theme. For runtime switching, use `theme/modsim-all-themes.css` and set `data-modsim-theme="light"`, `"dark"` or `"midnight"`. Switch fixed-color SVG/PNG paths as well. Configure application-specific fields in the starter web manifest before deployment.

## 7. Release scope and migration

Release 5.0 supersedes the earlier preview-only Light refresh. The actual Light vector and raster logo/icon files, desktop/web formats, toolbar state colors, pattern exports, covers, CSS, QSS and JSON tokens have been regenerated or updated. Dark and Midnight approved visual assets are preserved; their documentation, catalogs, previews and shared multi-theme configuration now describe the same final suite.

The combined ZIP contains three complete theme folders, not a set of mockup images and not only nested ZIPs. Standalone ZIPs contain the corresponding complete theme folder. Existing primary filenames remain usable. Updated PDF/Markdown guidelines, browser pages, inventories, provenance, checksums and validation reports are included.

## 8. Validation and limitations

Checks cover SVG/XML parsing, JSON loading, raster dimensions, transparent alpha channels, icon states and sizes, resource paths, HTML links, manifest hashes, native source preservation, original opaque robot foreground pixels, and ZIP integrity. The release report also compares the Light asset groups with the old pack and confirms the approved Dark and Midnight native covers remain unchanged.

The asset-manifest JSON contains relative paths, byte counts and SHA-256 values; its own file is excluded to avoid a recursive hash. Bundle checksums follow the same principle. These checks do not verify hardware geometry, simulator behavior, application-wide accessibility, or platform-specific visual behavior. Review the optional Qt example and icons in the actual application before shipping.
