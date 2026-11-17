# ModSim — Midnight software branding pack

Version 1.0 · Prepared 2026-11-16

## Start here

Open `index.html` for a browsable local catalog. The files can be used independently; the HTML is not required by the application. `previews/asset-overview.png` shows the actual assets in this pack. `examples/qt_theme_demo.py` is an optional PySide6 preview.

The production canvas is **#0B1521**, with foreground **#C5CCD3**. The same relative filenames are used in all three packs so that switching the pack root switches the theme.

## Contents

| Folder | Contents |
| --- | --- |
| `logos/svg/` | Horizontal logo; horizontal logo with descriptor; stacked logo; wordmark; black, white, and theme-foreground horizontal versions. Text is outlined. |
| `logos/png/` | Transparent logo exports at 1000 and 2000 pixels wide; monochrome horizontal logos at 2000 pixels wide. |
| `icons/svg/` | Transparent symbol; rounded-square app tile; maskable tile; reticle; three monochrome symbols. |
| `icons/png/` | Transparent symbol and app tile at 16, 20, 24, 32, 40, 48, 64, 96, 128, 192, 256, 512 and 1024 pixels; monochrome and reticle exports. |
| `icons/platform/` | Windows `.ico`, macOS `.icns`, favicons, Apple touch icon, maskable web icons, and a starter web manifest. |
| `ui/svg/` | 24 toolbar icons in default, accent, disabled and `currentColor` versions. |
| `ui/png/` | Default-color toolbar icons at 24, 48 and 96 pixels. |
| `patterns/` | Transparent module pattern and connection-line motifs, in SVG and 2× PNG. |
| `backgrounds/` | Native approved robot cover; cropped robot inset; theme-only background; theme-specific robot layout with and without branding at 1920×1080; 960×540 cover. |
| `theme/` | `colors.json`, `theme.css`, `theme.qss`, and a GIMP-compatible `.gpl` palette. |
| `examples/` | Optional PySide6 demo. |
| `previews/` | Actual-asset overview and a static loading-screen example. |
| `reference/` | Earlier approved concept board, original symbol, and preferred wordmark reference. |
| `resources.qrc` | Optional Qt resource collection for SVG assets and the 960×540 splash. |
| `asset-manifest.json` | File inventory, sizes, hashes, and image dimensions. |

## Production choices

The symbol is reconstructed from your original 1024-pixel logo: analytic rounded-square modules with the original seven-piece reticle traced from the image. The wordmark is a vector outline traced from your preferred uploaded wordmark, not a replacement typed in a guessed font. The exact original typeface has not been identified. Very small raster-derived contour differences are possible; the source references are included.

Light uses an off-white canvas. Dark uses neutral charcoal. Midnight uses **#0B1521**, sampled from your supplied wordmark image. Theme colors in `theme/colors.json` are authoritative. The approved concept boards are kept as references and can contain earlier palette or typography variations.

The 24 toolbar glyphs are a consistent implementation set based on the actions shown in the branding concepts, with additional common software actions. They are not pixel crops of the concept board. Default stroke width is 1.8 on a 24×24 viewBox.

## Use in Qt / PySide6

Choose SVG for scalable logos and toolbar icons; Qt can rasterize them at the requested display size. Paths below are relative to the pack root:

```python
from pathlib import Path
from PySide6.QtGui import QIcon

pack = Path("path/to/ModSim_Midnight_Pack")
app.setWindowIcon(QIcon(str(pack / "icons/svg/modsim-app.svg")))
app.setStyleSheet((pack / "theme/theme.qss").read_text(encoding="utf-8"))
play_button.setIcon(QIcon(str(pack / "ui/svg/default/play.svg")))
```

The optional demo can be launched with:

```sh
python examples/qt_theme_demo.py
```

PySide6 is not bundled. The example is syntax-checked, but was not run against a Qt GUI in the export environment. It demonstrates styling only; it does not control the simulator.

For compiled resources, run `pyside6-rcc resources.qrc -o resources_rc.py` in the pack directory, then import the generated module in your app. Resource paths begin with `:/modsim/midnight/`, for example `:/modsim/midnight/icons/svg/modsim-app.svg`.

## Use on the web

Use `theme/theme.css` and switch the `data-modsim-theme` attribute on your application root. The sample `index.html` shows the available assets with no external dependencies. `ui/svg/currentcolor/` is intended for **inline SVG** or an SVG component. Loading one of those files through an HTML `<img>` does not make it inherit the parent element's CSS color; use the fixed-color variants in that case.

The starter `site.webmanifest` contains icon references and theme colors. Configure application-specific fields such as `id`, `start_url`, and `scope` before deployment. Keep the manifest next to the referenced icon files or update its relative paths.

## Splash screens and robot imagery

`robot-cover-approved-native.png` is the approved 1672×941 rendered cover, unchanged and shared across the themes. The robot is illustrative concept art, not a verified render of a particular ModSim robot or a screenshot of a functioning interface.

The theme-specific 1920×1080 covers place the same approved scene in a right-hand inset with production branding on the left. The light and charcoal packs do **not** contain a newly relit robot. `robot-background-1920x1080.png` omits the new left-hand text so that your application can draw its own logo, status, and progress. The source illustration is raster art and is not made more detailed by rescaling. The logo and symbol remain scalable SVGs.

The progress bar in `previews/loading-screen-example.png` is a static design example. Do not use it to report actual load progress; draw a live progress widget on a background asset instead.

## Usage notes

Keep the symbol proportions and wordmark letter shapes intact. Use the theme-matched version against the corresponding canvas. Preserve transparent backgrounds when exporting logos. Reserve at least one small module-width around a placed mark where the layout permits. At very small sizes, use the symbol rather than a logo with text. Retain the dark/light variants instead of automatically inverting colors.

The logo files contain outlined paths and require no installed fonts. No font files are included. Supporting descriptor labels and previews were outlined/rendered separately; they do not establish the original wordmark's typeface. Application text may use your existing UI font.
