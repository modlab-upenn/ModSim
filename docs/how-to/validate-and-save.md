---
title: Validate and save a RobotPack
sidebar_label: Validate and save
---

Finish the cube's runtime presentation recipe, check its declared semantics,
and save a complete pack that a runtime can load. Start with
[Author connectors](author-connectors.md) complete in the Builder.

Validation uses the current in-memory document. The Runtime Inspector loads
saved files in its own execution context, so applying properties and saving
are both part of this handoff even when authoring and inspection share Studio.

## Add the runtime model-view recipe

The URDF importer does not create a model-view recipe. General Robot Packs
and core/headless runtimes do not require one, but this workflow uses the
Inspector to display connectivity. Add its recipe explicitly:

1. Switch to **Runtime** and click **Add topology view**.
2. Confirm that **Model view** selects **Module Topology (module_topology_graph)**.
   For this new pack, the added recipe has ID `module_topology`, runtime mode,
   and an empty configuration. It becomes a default when the pack has no
   existing runtime default. Existing recipes remain unchanged.
3. Return to **Robot Pack** to inspect the new entry under **Model Views**.
   The pack shows unsaved changes; adding the recipe does not save or run it.

This shortcut chooses a unique ID if `module_topology` is already present.
If the pack already has a suitable runtime recipe, select it in Runtime
instead of adding another.

### Edit a recipe directly

For a custom ID, name, builder, or default choice, use the authoring editor:

1. Select **Model Views** in Project and click **Add model view**.
2. In **Add model view**, enter these values:

| Field | Value |
| --- | --- |
| **Model view ID** | `module_topology` |
| **Name** | `Module Topology` |
| **Builder** | `module_topology_graph` |
| **Available while authoring** | Unchecked |
| **Available at runtime** | Checked |
| **Default view** | Checked |
| **Builder configuration** | Empty |

3. Click **OK**. The recipe appears under **Model Views**.

If you already created this ID, select its existing row, correct the fields,
and click **Apply model view**. Leave the configuration empty for this builder.
The manual **Add model view** dialog leaves **Default view** unchecked until
you choose it. A default enables automatic selection in a standalone
Inspector launch. Studio's Runtime workspace can select any suitable
runtime-enabled recipe explicitly, without changing its default flag.

The recipe selects a builder. You do not enter graph nodes or edges. During a
run, the builder derives one node per module instance and edges from committed
connections. The Builder does not start a simulation when you add the recipe.

## Run both validation profiles

Choose **Validate → Authoring Profile** (`F6`) and open **Validation** in the
bottom **Document** panel. Then choose
**Validate → Simulation Metadata (Structural)** (`F7`).

| Profile | What it establishes |
| --- | --- |
| Authoring | The declared schema, references, and local assets can be checked while incomplete simulation metadata is reported as warnings. |
| Simulation metadata (structural) | The stricter structural checks promote required missing semantic data to errors. |

For the completed workshop cube, both profiles should report
**0 errors · 0 warnings**. The table below the summary appears when there are
issues and identifies their severity, code, location, and message.

To correct an issue, find the entity named by its location in Project, edit
its Inspector fields, click the appropriate **Apply** button, then rerun the
profile. For this example, common missing values are an acceptance number,
an allowed-orientation set, the fixed constraint, or a load rating.

A passing profile does not execute the backend, check arbitrary URDF source
names against its mechanical files, or prove that two faces will mate. An
empty connector catalog can pass structural validation. These profiles also do
not require a runtime view. Check that the intended connector instances exist,
then use **Runtime → Validate runtime** to check the selected backend,
demonstration, and Inspector recipe before starting. **Validate → Runtime Launch**
(`F8`) opens Runtime and runs that same preflight check.

## Save the current workspace pack

Choose **File → Save** (`Ctrl+S`). The state indicator should change from
**Unsaved changes** to **Saved**, and the status bar reports **Saved** followed
by the pack directory.

Save writes canonical YAML through a staged, verified directory replacement.
It preserves the pack's mechanical assets while persisting semantic edits.
Formatting, comments, or YAML anchors from manually written documents are not
preserved as an editing format.

Save and Export are unavailable while execution is active. Use **Stop** in
Runtime, or close an active MuJoCo companion, before saving later edits.
Backend, duration, and other Runtime launch settings are session choices and
are not part of the YAML saved by this action.

Use **YAML Preview** to confirm that the current documents contain:

- The `workshop_cube` manifest and module identity.
- A `fixed_face` type with complete mating rules and undocking enabled.
- `front` and `rear` on `base_link` with opposite docking and approach axes.
- The runtime-enabled, default `module_topology` recipe.

You can reopen the saved directory with **File → Open Robot Pack** (`Ctrl+O`)
to verify the persisted project. Reopening replaces the current document, so
save pending changes first. A restarted runtime will read these same files.

## Export another copy when needed

Use **File → Export As** (`Ctrl+Shift+S`) to create a new self-contained
directory. In **Export to a new Robot Pack folder**, choose a new workspace
folder name, such as `workshop_cube_copy`; it must not already exist.
Successful export opens that copy as the current project.

Export preserves the pack's ID and version. A copied directory is not a newly
identified robot or a newly installed library entry. Keep experimental copies
in `workspaces` and select them by path. For the next guide, continue with
the original saved `workshop_cube` path, or substitute the exported path
consistently if that is the copy you want to run.

Previous: [Author connectors](author-connectors.md).
Next: [Run the RobotPack](run-robot-pack.md).
