# ModSim Documentation Setup Plan

Build a local Docusaurus documentation environment on the current `refactor`
branch, then develop the manual in small steps that we can inspect and discuss.
The public website, GitHub Pages deployment, profiling, and scalability work are
deferred. This tracker covers the local documentation setup and its content.

## How we will track progress

Each step has a stable ID, a technical status, and a separate review status.
Checked tasks record completed work. Completion evidence records the command or
visible behavior used to verify it. The review column records our discussion of
the result; a successful build alone does not mean the walkthrough has happened.

The initial scaffold is technically complete. The user manually edited and
accepted the overview in `docs/index.md`, accepted the visual theme and
Installation, and considers Getting started sufficient for this pass. The
active work is now the detailed architecture chapters and the URDF-to-runtime
How-to sequence (S4, S7, S8). The first Python API slice and its contributor
checks are implemented. The new chapters remain drafts for joint source reading
and manual review. Some S2 command walkthrough tasks remain open.

Studio now combines Robot Pack authoring and a configurable Runtime workspace
in one shell. R1 tracks the documentation alignment for that handoff, including
runtime preflight, model-view selection, save-before-start, and execution cleanup.

Installation and distribution preparation are now part of this work. D1 covers
installation without a checkout and preparing the bundled starter;
D2 covers the release pipeline and maintainer setup. L1 tracks the separate
community RobotPack library as future work. Public publication remains separate
from preparing and verifying local artifacts.

For each page, we will work through this loop:

1. Agree on the page's purpose and reader.
2. Read the relevant code or current contract together.
3. Draft a small section that we can discuss.
4. Let the user edit that section manually and refine the wording together.
5. Preview the page and exercise any examples it contains.
6. Record technical evidence and the user's review separately.

A page can render successfully while its prose is still a draft. We will leave
manual-edit and review tasks unchecked until the user confirms doing them.

| ID | Step | Technical status | User review |
| --- | --- | --- | --- |
| S0 | Confirm the documentation choices and scope | Complete | Agreed |
| S1 | Create and verify the local Docusaurus scaffold | Complete | Pending |
| S2 | Walk through local development and edit the overview together | In progress | Overview and theme accepted; workflow pending |
| S3 | Organize navigation and existing contracts with Diataxis | In progress | Pending |
| D1 | Prepare no-checkout installation and the local starter | Studio-first documentation in progress | Earlier installation accepted; revised workflow pending |
| D2 | Prepare checked release artifacts and publication workflow | Pre-release cleanup; account setup reported | Pending |
| S7 | Write getting started and the codebase walkthrough | Getting started accepted; complete source map and experiment drafted | New pages pending |
| S4 | Add and understand authored Mermaid diagrams | Themed class, flow, sequence, and state diagrams implemented | Pending |
| S5 | Generate the first Python API reference | Readable native reference and contributor checks implemented | Pending |
| S6 | Generate class and package diagrams from code | First runtime-state diagrams generated | Pending |
| S8 | Expand explanations, guides, and authored reference | Architecture chapters and Studio authoring guides drafted | Pending |
| R1 | Document the integrated Studio runtime workflow | Implemented; documentation and rendered-page checks complete | Desktop walkthrough pending |
| S9 | Verify the complete local documentation workflow | Planned | Pending |
| L1 | Design the independent community RobotPack library | Planned | Pending |

## S0 Confirm choices and scope

- [x] Use Docusaurus for the authored manual.
- [x] Organize content around Diataxis: tutorials, how-to guides, explanation,
  and reference.
- [x] Use Mermaid for authored diagrams and Pyreverse for code-derived diagrams.
- [x] Keep PlantUML available for cases needing richer UML notation.
- [x] Plan Sphinx and AutoAPI for the first generated Python API reference.
- [x] Refine the integration to generate Markdown for the same Docusaurus
  manual; the user rejected the first separate Sphinx HTML layout.
- [x] Keep authored documentation beside the code and generated output ignored.
- [x] Work locally in this branch before revisiting the public website or hosting.

Evidence: these choices were agreed in the setup discussion.

## S1 Create the local scaffold

Dependency: S0. Learning focus: the distinction between documentation content,
the Docusaurus application, installed dependencies, and generated output.

- [x] Add the Docusaurus application under `website/` with locked npm dependencies.
- [x] Record a supported Node version and explain how to select it locally.
- [x] Configure Docusaurus to read its initial content from `../docs`.
- [x] Add a small starting page and explicit initial navigation.
- [x] Keep existing contract files at their current paths.
- [x] Exclude historical material and unreviewed content from the initial site.
- [x] Add local development, build, preview, and type-check commands.
- [x] Ignore `node_modules`, Docusaurus caches, generated files, and HTML output.
- [x] Write the local setup instructions in `website/README.md`.
- [x] Install dependencies, check configuration types, and verify the local build.
- [x] Start the development server and check the initial page response.
- [x] Record the actual commands and results below.

Done when: a fresh dependency installation can reproduce the initial local site,
the type check and build pass, and the starting page is reachable locally.

Evidence:

- Docusaurus 3.10.2 installed with Node 22.22.2 and npm 10.9.7; a clean `npm ci`
  installation from `package-lock.json` succeeded.
- `npm --prefix website run typecheck` passed.
- `npm --prefix website run docs:build` passed with strict internal link checks.
- `docs:dev -- --port 3100` returned HTTP 200 for both included documentation
  routes under `/ModSim/docs/`.
- `docs:serve -- --port 3101` returned HTTP 200 for the site entry, both
  documentation pages, and representative CSS and JavaScript assets. The built
  pages contained the expected document titles.
- An additional temporary configuration exercised threaded static generation
  with the patched worker dependency successfully. This configuration is ignored
  verification output, not part of the normal development workflow.
- `npm audit --json` reported zero moderate, high, or critical findings after
  targeted dependency overrides. Five low-severity findings stem from one
  upstream deepmerge advisory; maintenance details are in `website/README.md`.

Browser interaction and live-edit review belong to S2. No API or diagram
generation is part of this first step.

## S2 Walk through local development

Dependency: S1. Learning focus: what each command does and which files to edit.
Reviewed page: `docs/index.md`; manually edited and accepted by the user. The
remaining workflow tasks below stay open until their walkthrough occurs.

- [x] Make a small opening overview draft available for joint editing.
- [ ] Review `website/package.json`, the configuration, sidebar, and starting page.
- [ ] Explain the role of `package-lock.json` and when to use `npm ci`.
- [ ] Start the development server together and edit a sentence to see live reload.
- [x] Manually revise the overview and review its wording together.
- [ ] Read the overview's relevant code together.
- [ ] Explain development preview versus the locally served production build.
- [ ] Show how to stop the server, change its port, and diagnose startup failures.
- [ ] Review the Git diff to identify source files and ignored output.

Theme work within S2:

- [x] Agree to combine ModSim's existing visual identity with the Hasura
  quickstart's documentation layout.
- [x] Document the shared tokens, Docusaurus adapter, fonts, assets, and local
  review workflow in `website/THEME.md`.
- [x] Add shared light and dark tokens, self-hosted fonts, and ModSim logo assets.
- [x] Map those tokens to Docusaurus navigation, content, and code-block styling.
- [x] Check theme types, build output, asset paths, and responsive behavior.
- [x] Accept the visual theme while preserving the user's manual `docs/index.md`
  edits.

Done when: the user can start the site, edit a page, locate the configuration,
and explain the difference between source and generated files.

Evidence: the user manually revised `docs/index.md`; those edits were preserved
during theme work. The TypeScript check and production build passed. Browser
checks covered both included pages at 1440 px and narrow layouts at 390 px in
light and dark modes. Self-hosted fonts loaded, mobile navigation opened, and
the theme choice survived reload. The checked pages had no JavaScript errors,
failed asset responses, or document-level horizontal overflow. Screenshots and
check results are in the ignored `website/.generated/visual/` directory.

The production preview now uses `website/scripts/serve.mjs` to serve font files
before route normalization; the route settings remain in the Docusaurus config.
The user accepted the overview and visual theme and chose to move to the next
page. The live-edit and command walkthrough tasks remain open. Record any
preferred command or folder changes here as those tasks are completed.

## S3 Organize navigation and existing contracts

Dependency: S2's first-page editing loop; its remaining command walkthroughs can
continue alongside this step. Learning focus: Diataxis describes reader needs;
sidebar organization does not require moving every existing document.

- [x] Agree on Tutorials, How-to guides, Explanation, and Reference and the reader
  need each section serves.
- [x] Agree to put local documentation development under a separate Contribute
  area rather than in the user manual.
- [x] Add a Documentation sidebar containing Overview and Tutorials.
- [x] Add a Contribute sidebar and navbar entry with a contributor landing page
  at `docs/contribute/index.md`.
- [x] Move the local development guide to
  `docs/contribute/local-development.md` and update its links.
- [x] Put a separate Installation page before Tutorials and link it back to
  Getting started.
- [x] Add contributor development setup and release-process pages to the
  Contribute sidebar.
- [x] Add How-to guides / CLI and keep terminal exercises separate from the
  Studio-first Getting started tutorial.
- [x] Add Explanation / Codebase map and Runtime state model, Reference / Python
  API, and Contribute / API and diagrams for the generation workflow.
- [x] Implement those sections in navigation as pages are edited together.
- [x] Add content under `docs/tutorials`, `docs/how-to`, `docs/explanation`, and
  `docs/reference` as each page is written.
- [ ] Include the six current contracts without changing their repository paths.
- [ ] Review the planning and M-Blocks guides for placement and current scope.
- [ ] Keep `docs/archive` and the historical technical report out of current
  routed documentation.
- [ ] Resolve Markdown rendering, relative links, assets, and heading anchors.
- [ ] Keep one authoritative copy of each rule and link to it from explanations.

Done when: all included pages have a clear place in navigation, existing
contract links work, and the local build passes.

Evidence: the Documentation sidebar contains Overview, Installation,
Tutorials / Getting started, and How-to guides / CLI. A separate Contribute
sidebar leads to its landing page, development setup, local documentation
development, and release process.
The TypeScript check and production build passed. Browser checks verified
Overview → Installation → Getting started, the tutorial's installation link and
return anchor, all three contributor pages with direct refresh, and mobile
navigation. These checks produced no page errors or failed assets. Explanation
and Reference were added in the subsequent API/UML pass. The current contract
files have not been included as routed reference pages. The new page drafts
await manual review.

## D1 Prepare no-checkout installation

Dependency: S3's initial navigation scaffold. Learning focus: end users install
the application, while source checkouts are for contributors.

- [x] Draft `docs/how-to/installation.md` with Python environments, wheel
  installation, optional extras, CLI verification, and the local starter.
- [x] Replace the tutorial's examples-checkout prerequisite with the bundled
  Generic Cube and link installation as its first step.
- [x] Package the small Generic Cube so wheel installation includes its data.
- [x] Implement `modsim-robotics setup`, a configurable home, and versioned starter
  placement under `robot_packs/modlab/generic_cube/0.1.0`.
- [x] Verify `--home`, `MODSIM_HOME`, and script-readable setup output.
- [x] Install the built wheel in a clean environment outside the checkout.
- [x] Run setup, inspect and validate the starter, and execute mock docking using
  only installed package data and the local home.
- [x] Manually review and refine the initial installation page together.

Namespace alignment within D1:

- [x] Update active documentation commands, Python imports, and source paths to
  `modsim-robotics`, `modsim_robotics`, `modsim_robotics_studio`, and
  `modsim_robotics_backend_mujoco` as appropriate.
- [x] Update example Python imports and explain distribution versus import names
  in the installation and contributor guides.
- [x] Verify the renamed Python packages and CLI entry points in fresh wheel and
  source-distribution installations.
- [x] Recheck the affected examples and documentation build after the code and
  configuration rename is complete.

Library location and per-command selection within D1:

- [x] Document `--home` > `MODSIM_PATH` > legacy `MODSIM_HOME` > `~/.modsim`,
  seeded `[paths].robot_packs`, and preserved legacy configuration.
- [x] Document persisting `MODSIM_PATH` in Bash, Zsh, and Windows user settings as
  explicit user actions; setup prints guidance without changing profiles.
- [x] Replace the tutorial's pack-path variable with commands that use the
  explicit `generic_cube` selector, local `pack list`, and optional path selection.
- [x] Verify relative and absolute library locations and preserved configurations,
  including ignoring legacy `[defaults].robot_pack`.
- [x] Verify ID and `ID@VERSION` selection, ambiguity errors instead of latest
  selection, and errors when a robot selector is omitted from read/runtime commands.
- [x] Verify every pack-reading CLI and standalone Studio selector, `--pack`, and
  `--home`, including explicit paths that bypass library configuration.
- [x] Document blank `studio` and standalone Studio launches for URDF authoring,
  preserving explicit selectors for existing packs and all read/runtime commands.
- [x] Verify the blank Builder launch, import/cancel transitions, session logging,
  and unchanged required-selector behavior together with the implementation.
- [x] Rebuild the distributions and repeat listing, named selection, and mock
  docking outside the checkout.
- [x] Recheck the documentation build and the tutorial's updated command outputs.
- [ ] Review the environment-persistence instructions and revised pages together.

Studio-first documentation within D1:

- [x] Make `modsim-robotics[studio]` the standard installation command, including
  both virtual-environment examples, and state the desktop-display requirement.
- [x] Add a separate `Install MuJoCo` section for
  `modsim-robotics[studio,mujoco]`, linked when simulation is introduced.
- [x] Keep setup, library location, pack selection, and `MODSIM_PATH` persistence
  in Installation while moving terminal inspection and docking to `docs/how-to/cli.md`.
- [x] Verify the new guide links, MuJoCo anchor, and documentation build.
- [ ] Review the Studio-first installation and tutorial flow together.

Done when: an end user can install the package, prepare the starter in the chosen
library, and inspect or author the selected RobotPack in Studio without cloning.
MuJoCo is added separately for the simulation step.

Evidence:

- `uv build --offline` produced `dist/modsim_robotics-0.1.0-py3-none-any.whl`
  and `dist/modsim_robotics-0.1.0.tar.gz` after build dependencies were cached.
- `tools/release/smoke_install.py` passed independently for both artifacts in
  fresh environments outside the checkout. Each installed the artifact,
  prepared Generic Cube, inspected and validated it, committed mock docking,
  and preserved the starter on repeated setup.
- A focused regression run covering CLI, user-data initialization, packaged
  scenarios, runtime runners, and the affected demo configurations passed:
  **257 passed, 6 MuJoCo tests deselected**. A separate headless MuJoCo Runtime
  Inspector integration run passed **6 tests**.
- User-data tests cover path precedence, JSON output, repeated setup, invalid
  existing packs, symlink rejection, and concurrent installation outcomes.
- Global Ruff lint and formatting, the configured core Pyright check, and
  `uv lock --check --offline` passed.
- The Installation page passed strict-link production build and desktop/mobile
  browser checks. The user accepted the manually edited PyPI-based installation
  page. Optional-extra installation on a fresh desktop and Windows execution
  remain for walkthrough; live publication verification belongs to D2.

Namespace verification after alignment:

- Renamed the source packages to `modsim_robotics`, `modsim_robotics_studio`,
  and `modsim_robotics_backend_mujoco`. No legacy import aliases are shipped.
- Updated Hatch source/data/version paths, CLI entry points, coverage and type
  checking, CI imports, release checks, tests, examples, and current docs.
- Refreshed this checkout's editable installation. Its commands are
  `modsim-robotics` and `modsim-robotics-studio`; the old commands are absent.
- Rebuilt wheel and source artifacts passed fresh installation checks outside
  the checkout. Both worked with an unrelated `modsim` module installed in the
  temporary environment. Checks cover canonical entry-point metadata,
  `python -m modsim_robotics`, starter setup, validation, and mock docking.
- Distribution inspection found only the three renamed Python packages. The
  artifact guard rejected synthetic archives containing legacy import names.
- The full offscreen non-MuJoCo suite passed **706 tests**, with 121 marked
  tests deselected. The adapter/conformance/viewer group passed **104 tests**;
  headless process/protocol checks passed **25 tests**. These groups overlap;
  they are separate verification results rather than a combined test count.
- Ruff lint and formatting, all three Pyright configurations, the dependency
  lock check, documentation type check, and strict-link site build passed.
- The repository remains `ModSim`; branding, `.modsim`, and `MODSIM_*` settings
  retain their existing roles. Manual page review and public publication remain
  pending.

Superseded default-pack trial evidence:

The following checks covered an earlier default-robot design. The user corrected
that design: configuration locates the library and pack-consuming commands select
a robot explicitly. Studio also supports a blank authoring window.
These results are historical verification, not completion evidence for the
current library-selection update.

- Setup seeds `[defaults].robot_pack` in new configurations. `MODSIM_PATH`
  selects the home, with explicit `--home` and legacy `MODSIM_HOME` support.
  Existing configuration and valid starter files are preserved; malformed
  configuration fails clearly. Pack paths retain symlink identity so the loader
  continues to enforce the RobotPack contract.
- Pack-reading and runner commands, including both Studio entry points, resolve
  the default when a pack is omitted. Positional and `--pack` overrides bypass
  configuration selection. Tests cover explicit home selection, invalid/missing
  setup, help/version without writes, and safely quoted environment guidance.
- The non-MuJoCo regression suite passed **818 tests**, with 121 marked physics
  cases deselected. Storage checks passed 51 cases; launcher checks passed 10.
  These focused checks are included in the regression count, not extra tests.
- Rebuilt wheel and source archives passed metadata/content checks, strict
  Twine validation, and fresh no-path setup/inspect/validate/mock docking checks
  outside the checkout. The base installation does not install Ruff or GUI
  dependencies, and standalone Studio help remains available without them.
- A separate installed-wheel MuJoCo 3.17.0 run loaded the default from a new
  `MODSIM_PATH` and completed one dock and one undock with no failures, zero
  final active connections, and two assemblies. `--home` and `--pack` overrides
  also passed from unrelated working directories.
- Ruff lint/format, core and Studio type checking, dependency lock consistency,
  documentation types, and strict-link site build passed. User page review,
  native desktop viewing, and Windows shell execution remain for walkthrough.

Setup only prints persistence instructions. No user shell profiles, persistent
environment settings, or public release state were changed.

Library-selection evidence before the blank Builder refinement: contracts and
tutorial commands describe `[paths].robot_packs`, explicit names or paths, version selection, local listing,
and missing or ambiguous selector errors. New setup records only the library
location; legacy default-robot settings are preserved but ignored.

The Studio authoring refinement permits an omitted selector to open the existing
welcome screen, with Open and Create from URDF actions. It does not choose a
robot, run setup, or relax selector requirements for inspection and runtime.
The import guide now begins with the blank Builder; import still writes its
initial draft immediately, and Save persists subsequent semantic edits.

Blank Builder verification: 249 focused CLI, selection, launcher, logging,
project, and GUI tests passed. They cover omitted selectors with and without
an installed starter, explicit home selection, optional-dependency errors,
unchanged pack/runtime selection requirements, canceled import dialogs, and
real URDF import followed by a GUI edit, Save, and independent reload. The GUI
tests use real Qt controls and the importer/loader/writer with a viewport stub;
native GPU rendering remains part of the desktop walkthrough. Ruff, core and
Studio Pyright, API documentation policy, and the strict-link documentation
build passed.

- The non-MuJoCo regression suite passed **862 tests**, with 121 marked physics
  cases deselected. Included focused checks cover 70 storage/discovery cases and
  14 standalone/application Studio cases. Name lookup handles multiple IDs and
  versions, same-version ambiguity, malformed unrelated manifests, path/name
  collisions in the working directory, symlinks, and manifest boundaries.
- Discovery reads safe typed root manifests and stops at pack boundaries,
  without loading split specifications or walking mechanical asset trees.
- Rebuilt wheel and source archives passed metadata/content checks, strict
  Twine validation, and fresh setup/list/inspect/validate/mock docking checks
  outside the checkout using explicit named selectors.
- An installed-wheel MuJoCo 3.17.0 run selected `generic_cube` by name and
  completed one dock and one undock, with zero failures, zero active final
  connections, and two assemblies. Exact version selection also passed;
  omitted selectors were rejected even with one installed pack. Explicit
  directory/manifest paths bypassed missing or invalid home configuration.
- Ruff lint/format, all three type-check configurations, the dependency lock
  check, documentation types, and the strict-link site build passed. The
  checksums in `dist/SHA256SUMS` describe the rebuilt local artifacts.

User review, native desktop viewing, and Windows shell execution remain for
walkthrough. No profiles, persistent environment settings, or public release
state were changed.

Studio-first update evidence: the Installation draft now includes Studio in the
standard package install and a separate MuJoCo installation step. Terminal pack
listing, inspection, validation, and mock docking are covered in the new CLI
guide. The earlier CLI and library-selection checks remain evidence for those
commands. Documentation type checking and the strict-link production build
passed. Generated HTML checks confirmed the CLI route and sidebar links,
`install-mujoco`, and the tutorial's docking-step return anchor. Manual review
remains pending. The core dependency boundary is unchanged by these documentation
edits.

## D2 Prepare the release workflow

Dependency: D1's installable artifact path. Learning focus: build and check one
artifact set, then use those same files for release distribution.

- [x] Draft `docs/contribute/release-process.md` with account setup, matching
  version tags, artifact checks, and PyPI Trusted Publishing.
- [x] Add a `release.yml` workflow that checks `v*` tags against the package
  version and verifies the tagged source.
- [x] Build and smoke-test wheel and source artifacts before promotion.
- [x] Configure GitHub Release and PyPI jobs to consume the exact checked files.
- [x] Verify that public distribution contents include the starter and exclude
  restricted or separately licensed example assets.
- [x] Record the maintainer's confirmation of a PyPI account, GitHub `pypi`
  environment, and pending Trusted Publisher for `release.yml`.
- [x] Prepare factual 0.1.0 release notes in `docs/releases/0.1.0.md`.
- [x] Make the authored notes suitable for the GitHub Release body and document
  the MuJoCo 3.3.3 minimum.
- [x] Verify that release validation rejects missing or empty versioned notes
  before PyPI and that GitHub publication uses the authored `--notes-file` body.
- [x] Replace README repository-relative links with absolute GitHub destinations
  for its PyPI long description, preserving same-page anchors.
- [x] Document retrying failed downstream jobs with the retained checked files
  and distinguish full reruns that rebuild the artifacts.
- [x] Verify final metadata: GitHub documentation source while Docusaurus is
  local, and Ruff in development dependencies only at `>=0.16,<1`.
- [x] Rebuild and recheck the final distributions after pre-release cleanup.
- [ ] Commit the intended release changes and integrate the release commit into
  `main`.
- [ ] Review release instructions and verification results together.
- [ ] Publish the first public release only as a separately requested action.
- [ ] Check the tag-triggered workflow's live OIDC publication, PyPI installation,
  and matching GitHub Release assets and checksums.

Done when: local and CI release preparation can produce checked artifacts and
the remaining maintainer account and publication steps are explicit.

Evidence: local wheel and source-archive installation checks passed. The
artifact inventory check requires every bundled Generic Cube file and rejects
other example pack trees. Distribution inspection confirmed that SMORES and
M-Blocks asset trees are absent; tests remain source-checkout-only. CI builds
once, smoke-tests both artifacts, writes `SHA256SUMS`, and retains them for the
publication jobs. Stable-tag verification passed for `v0.1.0` and rejected
mismatched/prerelease tags; workflow YAML parsed successfully. The workflow has
not run on GitHub yet. The maintainer has since confirmed the PyPI account,
GitHub environment, and pending publisher setup; that report has not been
independently checked by a live OIDC publishing run. User review and public
publication remain pending; no tag or release has been published.

Pre-release cleanup evidence:

- Removed Ruff from runtime dependencies, retained `ruff>=0.16,<1` in `dev`,
  and pointed distribution documentation metadata to the GitHub documentation
  source. The lockfile preserves all 72 existing package versions.
- Raised the MuJoCo minimum to 3.3.3 after finding required adapter APIs absent
  from earlier versions. An isolated Python 3.12/MuJoCo 3.3.3 environment passed
  85 backend, docking, contact, hinge, weld, and conformance tests.
- Updated a missing-mjpython recovery hint for installed users and made hinge
  geometry assertions compatible with pytest 9.2. Focused checks passed under
  repository pytest 9.1.1 and pytest 9.2.
- The complete offscreen test suite passed: **827 tests**, including the longer
  physical regressions. Existing VTK rendering and NumPy deprecation warnings
  remain. Ruff lint/format, all three Pyright configurations, dependency lock
  consistency, documentation types, and the strict-link site build passed.
- Rebuilt both distributions. Twine 7.0.0 `check --strict` passed, as did the
  metadata/license/version/entry-point/content verifier. Fresh isolated wheel
  and source installations passed starter setup, validation, repeated setup,
  and mock docking, with no runtime Ruff and an unrelated `modsim` module
  present. `dist/SHA256SUMS` records the final local artifact hashes.
- Archives exclude historical documentation, technical-report render output,
  tests/caches, legacy namespaces, and larger robot asset packs. Synthetic
  artifact checks exercised 39 accepted/rejected metadata and content cases.
- CI checks metadata before retaining release artifacts. Release validation
  requires nonempty versioned notes before PyPI; GitHub publication uses the
  authored notes. Workflow YAML and all shell steps parsed successfully;
  mocked new/draft/published and missing/empty-note cases passed without
  contacting GitHub or PyPI.

Local preparation is complete. Commit and `main` integration, version tagging,
live OIDC publication, and verification of the public installation remain
pending. No commit, tag, push, merge, or publication was performed by cleanup.

## S7 Write getting started and the codebase walkthrough

Dependency: S3's initial navigation scaffold. Learning focus: follow one working
example through the code before branching into platform-specific features or
adding generators. The active page is `docs/tutorials/getting-started.md`.
The current walkthrough prioritizes Studio inspection and authoring. Terminal
exercises live in `docs/how-to/cli.md`.

- [x] Make a small getting-started introduction and outline available for manual
  editing together.
- [x] Review and refine Getting started; the user considers this section
  sufficient for the current pass and has chosen to move to API/UML work.
- [x] Draft a linked installation guide; Studio is required for Getting started
  and MuJoCo is added for simulation. Verification and review are tracked in D1.
- [x] Replace the tutorial outline with the post-installation section
  "Explore and simulate Generic Cube" and update Installation's backlink anchor.
- [x] Preserve the verified `generic_cube` mock exercise and expected terminal
  output in the separate CLI guide.
- [x] Make Builder inspection and validation the main tutorial path, adding
  MuJoCo at its linked installation step for simulation.
- [x] Add an export/edit/save exercise in a workspace outside the installed
  library, followed by the Runtime Inspector's visual docking walkthrough.
- [x] Add a Python example covering loading, validation, scene creation, stepping,
  events or views, and session shutdown.
- [x] Trace that example through concrete modules and public entry points.
- [x] Draft a codebase map with a reading order, per-module responsibilities, and
  an inventory covering all 110 Python source files.
- [x] Explain URDF mechanics, YAML semantics, canonical `WorldState`, and derived
  views without duplicating the contracts.
- [x] Verify library listing and the named-selector examples from the documented
  working directory after library selection is integrated; retain this evidence
  for the CLI guide.
- [x] Verify the Studio-first page flow and its links after the rewrite.
- [ ] Read the codebase map and source together and record confusing terms or
  missing links.

Done when: a reader can complete the tutorial and locate the code responsible
for each major part of the example.

Evidence for the earlier CLI-oriented walkthrough: its commands used the starter
prepared during Installation, without reinstalling ModSim or repeating
environment setup. Those terminal exercises now belong to `docs/how-to/cli.md`.

The CLI commands were exercised from a built-wheel installation in a separate
Python 3.12 environment and temporary ModSim home outside the checkout.
Inspection, both validation profiles, mock docking, mock dock/undock, and the
four-second MuJoCo lifecycle passed. Expected event names and final
connection/assembly counts match the documented results. MuJoCo was 3.17.0 in
that environment. The Builder tree labels and both validation actions were
checked offscreen with a stubbed viewport and left the temporary pack files
unchanged; Runtime Inspector configurations were checked through headless
runtime execution. A native desktop viewer and Windows PowerShell execution
remain for user walkthrough. The documentation build passed strict links.

Those initial command checks used explicit pack paths, followed by a superseded
default-pack trial recorded in D1. Named-selector command checks and the previous
documentation build also passed; see D1 above. The current Getting started
rewrite makes Studio the standard inspection and authoring interface and links
to the separate MuJoCo installation step before simulation. Type checking,
strict-link production build, CLI navigation, and both installation/tutorial
return anchors passed. An offscreen temporary-pack exercise verified Export As,
module-name editing, Apply module metadata, Save, and both validation profiles;
the installed starter's files stayed unchanged. A headless Inspector lifecycle
verified the documented events and final counts. Code inspection confirmed
quarter-speed playback. At that stage the Builder and Inspector were separate
windows; the [integrated workflow](#r1-integrated-studio-runtime-workflow) now
supersedes that handoff. Native viewport rendering and the full desktop
walkthrough await user review.
Getting started is now sufficient for this pass. The codebase map is drafted;
its source-reading order and the first API/UML pages are the next review work.
The new How-to experiment now provides the Python example; its architecture
reading links and the revised source map await manual review.

## S4 Add authored Mermaid diagrams

Dependency: S3. Learning focus: diagram source stays editable with the prose
that explains it.

- [x] Enable the official Docusaurus Mermaid integration.
- [x] Add an architecture diagram showing mechanics, pack semantics, runtime,
  canonical state, backend observations, and generated views.
- [x] Add a docking sequence diagram checked against the lifecycle contract.
- [x] Explain when to use a flowchart, sequence diagram, state diagram, or class
  diagram.
- [x] Check the architecture diagram in light and dark modes in the production
  preview; repeat for future authored diagrams as they are added.
- [x] Add explanatory prose beside the first architecture diagram.

Done when: editing a Mermaid block changes the local diagram, and the diagrams
agree with the current contracts.

Evidence: the pinned `@docusaurus/theme-mermaid` integration is enabled in
Markdown and the theme configuration. The Explanation chapters contain 23
authored diagrams, including class ownership, call sequences, docking lifecycle,
and subsystem flows. Sources were checked against current contracts and actual
classes, methods, and message ordering. The shared theme adapter reads the site's
light/dark palette and fonts and provides expanded viewing and SVG download.
The contributor generation guide explains diagram selection and maintenance.
Browser results are recorded below; user review remains separate.

## S5 Generate the first Python API reference

Dependency: S3. Learning focus: signatures and docstrings come from code;
architecture explanations and examples remain authored content.

- [x] Add a separate Python documentation dependency group and update its lock.
- [x] Add Sphinx and AutoAPI configuration under `website/api`.
- [x] Generate a first reference for `RuntimeSession` and its related public types.
- [x] Configure static source extraction without requiring Qt or MuJoCo imports.
- [x] Stage extraction output under `website/.generated/api` and materialize
  ignored generated Markdown in `docs/reference/api`.
- [x] Load API pages through the same Docusaurus docs plugin and Documentation
  sidebar under Reference / Python API.
- [x] Check native navigation, object links, breadcrumbs, and theme persistence
  between authored and generated pages.
- [x] Remove the separate Sphinx HTML layout and API-specific search.
- [x] Change a docstring and verify that regeneration updates the API page.
- [x] Add a documented generation command to the shared local workflow.

Done when: the same code checkout produces a useful API page locally, all API
assets load, and regeneration reflects a real source change.

Evidence: Sphinx 9.1.1, AutoAPI 3.9.0, sphinx-markdown-builder 0.6.11, and Pylint
4.1.4 are locked in the separate `docs` dependency group. The initial standalone
HTML proof was replaced at the user's request. AutoAPI now feeds 11 native
Docusaurus Markdown module pages; internal helper modules remain parsed only
for import resolution. A minimal Markdown abbreviation-node visitor preserves
Sphinx 9's keyword-only `*` marker.

Strict generation passed. All 524 explicit Python object IDs are retained. The
initial native layout had 537 relative links; the readable layout has 418 after
moving declarations into Python code blocks and retaining linked parameter and
attribute types. Those links resolve. An independent source check found all 116 public
functions/methods/properties in scope, including seven keyword-only signatures.
Temporary-copy edits to docstrings, parameter text, and return text appeared in
generated Markdown while an import guard blocked ModSim, Qt, MuJoCo, PyVista,
and PyQtGraph imports; actual source stayed unchanged. All 11 compiled API
pages passed local route/fragment checks. RuntimeSession
create/step/request_dock/shutdown anchors passed in the browser.

`docs:api` runs the API generator; `docs:generate` runs both Python generators.
The normal `docs:dev` and `docs:build` commands regenerate first. Authored docs
continue to be read directly, preserving their live-edit workflow. Generated
Markdown is ignored and explicitly excluded from source distributions; a fresh
sdist check confirmed the authored API landing is included while generated API
pages and build output are absent. Static `.generated/site` mounting now serves
diagram assets; the old API HTML tree is removed. All public API rendering uses
Docusaurus. Shared site search remains separate planned work in S9. User review
is pending; expand extraction after reviewing this first result.

Readability and contributor requirements within S5:

- [x] Follow the React Native reference's short method headings, separate
  highlighted signatures, descriptions, parameter details, and clear sections.
- [x] Render compact attributes and linked parameter/type/default/description
  tables, Returns, Raises, and Examples from resolved Sphinx data.
- [x] Use the same Type/Description table presentation for return values and
  property types, preserving linked types and source descriptions.
- [x] Support Google-style Args/Returns/Raises/Examples through Napoleon while
  retaining existing reStructuredText docstrings.
- [x] Use one `website/api/scope.py` registry for generation and policy checks;
  derive Sphinx's temporary toctree from it automatically.
- [x] Document mandatory public API source contracts in the contribution guides
  and add a PR checklist.
- [x] Add `docs:check` before local API generation, preview, and production
  builds; preserve the standalone check command.
- [x] Add documentation CI validation for pull requests, `main` pushes, and the
  reusable release checks, with package creation depending on that job.
- [ ] After the workflow is pushed, make its Documentation status check required
  in the integration branch's GitHub protection rule or ruleset.
- [x] Verify policy failures for missing new docs, stale arguments, changed
  signatures, newly raised errors, and baseline growth.
- [ ] Review the readable API pages and contributor rules together.

Evidence: thirteen standalone policy regression tests pass. The static checker
covers 202 objects in the eleven published modules; 105 objects retain 242
explicitly recorded legacy issues. Unchanged signatures may keep or reduce
that debt. New symbols and changed signatures must meet the checked contract;
the baseline updater only removes resolved items. The worked RuntimeSession
class and its public methods have no policy debt. Source docstrings now include
parameter meaning, units, lifecycle behavior, returns, errors, and a complete
Generic Cube example verified in a temporary home. Only documentation changed
in the runtime module.

Temporary generator probes verified Google/RST fields, Yields, positional-only
and keyword-only separators, variadics, False/zero/empty-string defaults, and
blocked application imports. Adding one scope entry generated a twelfth module
page without a separate index edit. Strict generation, the combined site build,
lint/format, type checking, and lock consistency pass. The documentation CI YAML
and shell steps were checked locally; no live GitHub job or publication has
run. Review of semantic accuracy, delegated errors, examples, and registration
outside the first published slice remains a contributor/reviewer responsibility.

Production-preview browser checks verified short method headings and TOC,
highlighted Python signatures, complete create() parameter rows, Returns,
Raises, Examples, native type navigation, light/dark themes, and mobile pages
without document overflow. Screenshots are ignored under
`website/.generated/visual/readable-api/`. Final page review remains pending.

Returns-style follow-up: the shared generator now renders all 99 callable
Returns sections and 17 property Type sections as Type/Description tables.
Generated Markdown and compiled HTML checks confirmed all 116 tables use the
same columns, preserve type links and descriptions, and retain explicit
`None` results. The strict-link site build and renderer lint/format passed.

Table-width follow-up: API tables now retain native column sizing inside a
full-width scroll wrapper added by the scoped rehype plugin. Attribute rows and
headers fill the surrounding card; narrow layouts scroll the table within the
card. Type checking and the site build passed. Browser checks covered all tables
on the runtime/session, core/snapshot, and schema pages at 1440, 1024, and 390 px,
confirming full-width cells, aligned header/body edges, working mobile scrolling,
and no document overflow or page/asset errors. Authored pages are unaffected.
Screenshots are ignored under `website/.generated/visual/api-table-width/`.

## S6 Generate diagrams from code

Dependencies: S4 and S5. Learning focus: generated class relationships describe
code structure; authored diagrams explain intended behavior and boundaries.

- [x] Choose one small subsystem for the first Pyreverse diagram.
- [x] Record scoped inputs and generation options in the repository.
- [x] Add Graphviz installation instructions for SVG output.
- [x] Generate diagram output into the ignored staging directory.
- [x] Initially include the result in the runtime explanation; replace its
  presentation with curated themed UML during the architecture rewrite.
- [x] Verify that rerunning generation reproduces the diagram from current code.
- [x] Keep PlantUML deferred; the current scoped diagrams do not require it.
- [x] Explain that Pyreverse can be used without changing the existing Ruff setup.

Done when: one readable diagram can be regenerated with the documented command
and its labels match the subsystem being explained.

Evidence: `tools/docs/generate_diagrams.py` extracts class and package DOT from
three core source files (`assemblies.py`, `entities.py`, `state.py`) and renders
SVG with Graphviz. The class view has seven connected classes and six inferred
relationships; the package view has three modules and two imports. Two runs
produced byte-identical DOT/SVG files, and SVG XML validation and Ruff passed.
The current architecture chapters use authored Mermaid diagrams rather than
embedding these diagnostic SVGs. The generation guide identifies the limits of
static inference and retains the diagnostic workflow. Pyreverse is a
documentation tool; Ruff remains the code checker.

System Graphviz was absent, so native packages were extracted only to ignored
`.local-tools/graphviz`; the generator detects this local fallback. No system
packages or persistent environment settings were changed. Fresh setups have
documented Graphviz installation steps. Generated files and the local renderer
are ignored. User review remains pending.

## S8 Expand the manual

Dependency: S7. Learning focus: separate practical tasks, explanations, and
reference so that readers know where to look next.

- [x] Add frame, unit, naming, and lifecycle terminology to the relevant chapters.
- [x] Add separate Studio import/authoring and MuJoCo runtime guides with prerequisites.
- [ ] Document how to add a backend, model view, and scenario.
- [ ] Expand public API reference with useful docstrings and examples.
- [ ] Add CLI and Robot Pack reference introductions linked to current contracts.
- [x] Describe planner and physics limitations beside the features they qualify.
- [x] Add troubleshooting based on the workflows we actually exercise.

Done when: the agreed core workflows each have a discoverable starting point and
links to the detailed contracts or API reference.

Evidence: the published `origin/gh-pages:quickstart.html` was used as the source
sequence for the new How-to section, with obsolete installation, command, and
import names updated against current code. Ten pages cover the workflow landing,
URDF preparation/import, connector authoring, optional joints, validation/save,
runtime execution, Python experiments, troubleshooting, and the CLI route.
Explanation now has nine chapters covering system boundaries, all 110 source
files, pack persistence, canonical state, docking, backends, desktop execution,
planning/scenarios, and generated views. These are authored explanations, not
an expansion of the generated API reference scope.

The exact documented cube was exercised through import, authoring APIs,
validation, save/reopen/export, mock docking, and headless MuJoCo docking.
The Python experiment passed on both backends. Three-module CLI docking and
effort-command limit/clear behavior were also checked. UI labels were checked
against source; native desktop interaction remains part of user review.

Architecture rewrite verification: TypeScript checking, the API documentation
policy, strict API generation, Pyreverse generation, and the strict-link
Docusaurus production build passed. Browser checks covered all 19 rewritten
How-to/Explanation pages plus RuntimeSession reference at 1440 px and 390 px in
light and dark modes: 80 page checks, all 23 diagrams rendered, no page errors,
failed asset responses, or document-level horizontal overflow. Expanded viewing,
Escape and close controls, themed SVG download, fit-width/actual-size sizing,
task links, and mobile sidebar navigation also passed. Screenshots and page
results are ignored under `website/.generated/visual/architecture-rewrite/`.
These checks verify local rendering and executable examples; they do not mark
the new prose or native desktop walkthrough as reviewed by the user.

## S9 Verify the complete local workflow

Dependencies: S4 through S8. Learning focus: one repeatable workflow should
prepare generated content, build the manual, and preview the complete result.

- [ ] Run a fresh locked installation and documented generation sequence.
- [ ] Verify authored pages, API pages, diagrams, links, and assets together.
- [ ] Check broken internal links and relevant generation warnings.
- [ ] Confirm generated files stay ignored and outside Python distributions.
- [ ] Review navigation on narrow screens and in light and dark modes.
- [ ] Assess local search and choose an integration if needed.
- [ ] Document how to update prose, docstrings, diagrams, and dependencies.
- [ ] Record any remaining limitations and the commands that verify completion.

Done when: the local setup works from the documented prerequisites and the user
can repeat the complete authoring and preview workflow.

Evidence: the first combined API/UML setup passed the TypeScript check, strict
Sphinx generation, strict-link Docusaurus build, Python generator lint/format,
and the dependency-lock check. A production-preview browser check verified:

- the codebase map's Mermaid SVG in light and dark modes;
- both generated UML images loading with descriptive alternative text;
- native manual → generated API → manual navigation and direct RuntimeSession
  links, preserving the same client application;
- the same sidebar, breadcrumbs, table of contents, and theme selection on API
  pages, including a type link from RuntimeSession to WorldState;
- create, step, request_dock, and shutdown method anchors;
- mobile manual navigation and no document-level horizontal overflow on the
  codebase map, API landing, RuntimeSession, WorldState, or schema pages at
  390 px; and
- no page JavaScript errors or failed asset responses on the checked routes.

The new four-column size table exposed a small mobile overflow. Tables now use
their existing horizontal-scroll behavior as block elements at narrow widths;
the repeated browser check passed. Screenshots and results are ignored output
in `website/.generated/visual/api-uml/` for the initial proof and
`website/.generated/visual/native-api/` for the current native integration. This verifies the first scoped
integration, not the deferred full-manual/API inventory or every platform.
User review and the remaining S9 tasks stay open.

## L1 Design the community RobotPack library

Dependency: agreement on the library's contribution and distribution contract.
Learning focus: a RobotPack is versioned data with mechanical assets, independent
of the application package.

- [ ] Agree on a separate library repository and its maintainers before creating
  or publishing it.
- [ ] Define pack metadata, versions, compatibility, and asset licensing.
- [ ] Choose how larger URDF and mesh assets are released without requiring end
  users to clone the entire library.
- [ ] Define how downloaded packs are installed in the user's ModSim home.
- [ ] Implement and verify acquisition and update commands before documenting
  them as available.
- [ ] Write the pack-author contribution and review workflow.

Done when: the approved library workflow can distribute a compatible pack and
install it locally, with a documented author contribution path.

Evidence: remote community-library design and implementation remain planned.
Installed-pack discovery with `pack list` belongs to D1; no remote catalog,
download command, or remotely created repository is claimed by the current
installation workflow.

## G1 Apply native application branding

The supplied ModSim production packs now drive both desktop applications.
This is native Studio work; the Docusaurus theme and deployment workflow remain
separate tasks.

- [x] Package the Light, Dark, and Midnight palettes, outlined wordmarks,
  application icons, toolbar glyphs, and robot artwork with provenance/hashes.
- [x] Apply shared branding to the Builder and Runtime Inspector, preserving
  theme preferences and restoring the legacy `graphite` setting as `dark`.
- [x] Paint startup splashes before heavy window/plotting imports.
- [x] Add responsive document loading feedback and runtime first-frame feedback,
  with real stages and indeterminate progress.
- [x] Update the Studio/runtime contracts and source map for the new ownership.
- [x] Build a local wheel and source archive and verify their assets.
- [ ] Review the desktop rendering and interactions with the user, including
  the native 3D viewport and platform window icons.

Verification: **895 non-MuJoCo tests passed**, with 121 marked physics cases
deselected. This includes viewport behavior, theme migration, splash cleanup,
responsive document work, header/text layout, runtime first-frame transitions,
and failure/stop paths. Ruff lint/format, core and Studio Pyright, the lock
check, API documentation policy, documentation TypeScript, and the strict-link
Docusaurus build passed.

The wheel and source archive under ignored `dist/branding-preview/` passed the
release content/metadata verifier. All 120 copied brand files match the supplied
source and manifest in both artifacts. An installed copy of the wheel rendered
each theme's icons, wordmark, and splash, and a real mock runtime presented its
docking result. That GUI check reused the development environment's optional
dependencies; a fresh dependency-resolving install could not complete because
PyPI was inaccessible and the available caches lacked required wheels.

Offscreen captures were inspected for all three themes. Builder welcome
captures used a viewport stub; a real VTK desktop capture could not initialize
OpenGL in this headless session. The native desktop walkthrough remains open.
No package publication or deployment was performed.

Ubuntu dock follow-up: setting a Qt window icon alone left native Wayland
windows without a matching desktop identity. Both launchers now set
`io.github.modlab_upenn.ModSim` before creating any splash or window. Linux
GUI launches register an idempotent per-user desktop entry and a stable Midnight
app icon. The entry opens the blank Builder using the most recently launched
GUI's Python environment; its virtualenv interpreter path is preserved.
Registration errors are logged and do not prevent a window from opening.

Follow-up verification: **905 non-MuJoCo tests passed**, including eight desktop
metadata cases and two Qt identity/failure cases. Core/Studio type checks,
Ruff, API policy, the documentation build, and wheel/source artifact checks
passed. The desktop entry and icon were installed for the current Ubuntu user;
`desktop-file-validate` and GIO resolved them. A brief native Wayland test
window emitted the matching `set_app_id` value. That probe used a neutral Qt
platform theme to bypass an incompatible GTK settings schema in the agent's
environment. Existing application processes need to be closed and relaunched;
native 3D viewing and the full desktop walkthrough remain for user review.

Cross-platform/high-DPI follow-up: Windows launches now set the process's
AppUserModelID before QApplication construction. Linux keeps its registered
desktop identity, while macOS uses Qt's runtime Dock/window icon support.
Branded Windows executables/shortcuts and macOS Finder/Launchpad entries still
require native distribution packaging; this wheel does not create them.
Windows/macOS native shell behavior has not been exercised on this Linux host.

The packaged asset set now includes the three original 1024-pixel app PNGs,
bringing the verified copied inventory to 123 files. SVG wordmarks and app
icons stay scalable; toolbar icons provide 1x/2x/3x/4x variants. Loading artwork
and splash canvases now allocate physical pixels for the destination display,
and wordmarks refresh when their display scale changes. The highest-resolution
supplied robot artwork remains 1920x1080; no native 4K art is claimed.

Runtime initialization now shows four visible milestones driven by existing
status callbacks: prepare execution, load/validate the pack, initialize the
backend, and receive the first simulation state. The first accepted frame
completes startup; stopping or failure remains distinct. Progress stays
indeterminate and no artificial display delay is added.

Verification: **922 non-MuJoCo tests passed**, with 121 marked physics cases
deselected. Focused Windows-target type checks and platform-isolated native
identity tests passed, along with core/Studio Pyright, Ruff, API policy, the
documentation build, and artifact checks. Normal and 2x captures of the
startup sequence and narrow splash were inspected. All 123 copied assets match
their supplied hashes in the source tree, wheel, and source archive. The
registered Ubuntu icon was updated to the original 1024-pixel image.

UI readability follow-up: the shared field padding left only 4–12 pixels of
usable text height in inline editors that needed an 18-pixel font. Compact
item editors, matching list/tree/table palettes, native Fusion combo/spin
primitives, and palette-aware file-navigation arrows now keep file-browser
views and renaming readable. Existing hidden-folder preferences, filename
filters, and accepted/cancelled selections are preserved.

Both applications retain system font sizing and scale semantic labels in
points. Builder welcome buttons use their actual text size hints, sidebars
can shrink, validation content wraps/scrolls, and loading panels remain
reachable at larger fonts. Inspector workspace content scrolls independently
of its persistent status/playback controls; inactive pages no longer force
oversized windows. Runtime table rows track font/style changes without
scanning the event model. Owned action/button labels no longer include
ellipsis suffixes, and the authored walkthrough labels were updated.

Verification: **949 non-MuJoCo tests passed**, with 121 marked physics cases
deselected. Coverage includes real file/folder rename commit and cancellation
in list/detail/icon views, 100%/150%/200% DPR processes, three themes, enlarged
fonts, 800x600/1280x720 Inspector layouts, small Builder geometry, and 10,000
retained event rows. Native Ubuntu Wayland dialog rendering was also inspected
using temporary files and a neutral Qt platform theme to bypass the agent
environment's GTK schema mismatch. Ruff, core/Studio Pyright, API policy, the
documentation build, and rebuilt wheel/source artifact checks passed. Native
Windows/macOS rendering and the complete user desktop walkthrough remain open.

Animated splash follow-up: at the user's explicit request, both GUI launchers
now retain their branded startup window for at least three seconds. The
approved SVG symbol assembles in staggered layers, its wordmark reveals, and
the completed logo gently pulses. A responsive 1.1-second intro runs before
the heavier GUI imports/construction; initialization counts toward the total
three-second minimum and only the remaining time is awaited before handoff.
Qt/VTK construction stays on its owning GUI thread. Clicks do not dismiss the
intro; explicit close cancels startup, and failures close it immediately.

Inspector autorun is now scheduled on its first show event so simulation does
not start or finish while the branding window covers a hidden Inspector.
Actual backend/RobotPack loading milestones and first-frame readiness remain
separate from this decorative introduction. The new animation module brings
the source inventory to 114 files (28 Studio files).

Verification: **958 non-MuJoCo tests passed**, with 121 marked physics cases
deselected. Tests cover the real three-second minimum, responsive event
processing, slow-startup fast path, cancellation, hidden-runtime deferral,
timer shutdown, SVG shape preservation across themes, and 2x animation frames.
A failed-startup cleanup issue involving a timer closure and deferred QObject
destruction was reproduced and fixed by disconnecting the callback before
disposal; the complete combined test order now passes. Ruff, core/Studio
Pyright, API policy, the documentation build, and distribution content checks
passed. A three-second animated preview was captured locally for user review.

Shared window-sizing follow-up: the Builder, Runtime Inspector, file browser,
creation forms, startup splash, and operation feedback now use one internal
logical-pixel policy. New windows account for completed layouts and body-font
metrics, with defaults limited to 88% of usable screen width and 86% of usable
height. Normal geometry and main-window dock arrangement are remembered in
local Qt preferences; maximized/fullscreen state is not restored. Temporary
loading windows do not persist geometry. Creation forms scroll independently
of their always-visible confirmation controls.

Monitor selection uses the visible parent or pre-show pointer screen, then
reconciles fresh defaults against the monitor assigned by the native desktop.
Native Ubuntu Wayland testing exposed delayed `wl_surface.enter` after the
first expose and configure events overwriting immediate resizes. The policy
therefore applies corrections through an owned, deferred timer, preserving
saved dimensions and explicit user changes. Geometry persistence uses outer
frames consistently to avoid repeated decoration-size shrinkage. Weak window
references and signal cleanup prevent retaining plotting widgets into Qt
shutdown. The new module brings the source inventory to 115 files (29 Studio).

Native checks covered the actual Builder controls with a viewport placeholder,
the Runtime Inspector with execution disabled, the shared file picker, and a
tall enlarged-font form on the current mixed-DPI Ubuntu desktop. VTK's native
X11 preview was excluded from that Wayland control probe; its existing separate
viewport tests remain in the regression suite. Windows and macOS desktop
testing remains outstanding. The sizing regressions are included in the Studio
CI job.

Verification: **992 non-MuJoCo tests passed**, with 121 marked physics cases
deselected. The 33 focused sizing cases cover remembered normal geometry,
dialog accept/reject, ignored closes, font changes, negative/portrait monitor
coordinates, disconnected displays, delayed native screen assignment, recreated
window handles, user-size preservation, and scrollable confirmation forms.
Ruff lint/format, core and Studio Pyright, API policy, and the Docusaurus build
passed. The rebuilt preview wheel and source bundle passed distribution checks;
the wheel matches all 115 Python sources and resolves its packaged branding
when extracted outside the checkout.

## G2 Align documentation with brand release 5.0

Status: implemented locally; user visual review pending.

- [x] Apply Light / Glacier & Steel and Midnight / Deep Navy & Cyan to the
  existing Docusaurus light/dark modes, using the production JSON color roles.
- [x] Replace navbar graphics and favicon with the supplied outlined artwork;
  preserve the wordmark's 5:1 proportions and existing navigation behavior.
- [x] Share the updated colors across API pages, code blocks, and Mermaid
  diagrams while keeping reading surfaces plain.
- [x] Package a small, hash-verified web asset subset with provenance.
- [x] Add **Contribute → Branding**, including exact palette roles, logo rules,
  approved cover examples, and the desktop/web update workflow.
- [x] Update `THEME.md` and verify types, API policy, strict links, and the
  production build.

The brand asset release number remains independent of the Python package
version. Existing page prose, content routes, self-hosted fonts, API layout,
diagram controls, and application behavior are preserved. The public website
branch and deployment remain separate work.

Verification: all 17 mapped production color roles match the Light/Midnight
JSON palettes, and all 11 copied web assets match their recorded source hashes.
All 17 browser checks passed for overview, API, UML, and branding pages at
desktop/mobile widths in both modes, including navigation, font/image loading,
Mermaid recoloring, and theme persistence. Wide branding tables scroll within
their containers on mobile while headers and hex values remain intact.
The browser review results and captures are local development
artifacts, not published assets.

## R1 Integrated Studio runtime workflow

Status: implemented locally; documentation alignment and checks complete.
The complete desktop walkthrough remains for user review.

- [x] Document the **Robot Pack** and **Runtime** tabs in one Studio shell, with
  standalone Inspector commands retained in the CLI reference.
- [x] Explain backend, demonstration, module-type, and model-view selection,
  advanced SI-unit settings, and the native MuJoCo companion.
- [x] Separate F6 authoring validation, F7 structural simulation metadata, and
  F8/Runtime launch preflight, including their limits.
- [x] Document explicit **Add topology view**, unsaved recipe edits,
  **Save and start**, fresh disk validation, and saved-pack runtime execution.
- [x] Describe **Pause**/**Resume**, **Stop**, retained results, **Back to setup**,
  and the native viewer's final-scene hold until execution resources stop.
- [x] Preserve authoring edits/dock visibility, session-only launch settings,
  backend-comparison tuning, and persistence/project locks during a run.
- [x] Align installation, tutorial, how-to, architecture, platform, contributor,
  API-landing, and repository overview documentation with these behaviors.
- [x] Complete documentation policy, type, link, and production-build checks
  after the cross-manual review.
- [ ] Walk through the finished desktop workflow with the user.

Implementation verification recorded 568 GUI/runtime regression tests passing,
then 14 focused workspace tests passing after the final backend-settings fix.
Those checks included a real mock simulation, pause/resume, stopping and
relaunching, missing-view recovery, save cancellation, disk revalidation,
shutdown, restored docks, and 800×600 layouts at enlarged fonts. An integrated
MuJoCo launch also produced runtime frames and events. Strict core/Studio
Pyright, Ruff, and the earlier production documentation build passed.
The source map now covers 117 Python files, including `runtime_launch.py` and
`runtime_workspace.py`.

Documentation verification passed API policy for 202 published objects,
TypeScript, strict-link production build, and whitespace checks. All 229 relative
documentation file links resolve across 42 active Markdown files. Browser review
passed 60 checks over 15 relevant manual pages at 1440 px and 390 px in light and
dark modes, including rendered Mermaid diagrams, expanded viewing/SVG download,
theme application, page width, and absence of JavaScript or asset-load failures.
Historical archives and the technical report remain outside the current manual.

## Deferred work

Public homepage and docking-widget migration, GitHub Pages deployment, and
documentation versioning will be planned after the local workflow is understood.
Documentation CI checks are now configured as part of the contributor API
contract; live execution awaits pushing the workflow. Package release preparation
is active in D2; public package publication still needs maintainer configuration and a
separate publication step. Profiling, scalability benchmarks, and broader
testing or quality initiatives remain separate later work.
