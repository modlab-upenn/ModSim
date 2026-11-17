---
title: Validate and save a RobotPack
sidebar_label: Validate and save
---

Finish the cube's runtime presentation recipe, check its declared semantics,
and save a complete pack that another process can load. Start with
[Author connectors](author-connectors.md) complete in the Builder.

Validation uses the current in-memory document. The Runtime Inspector loads
saved files in a separate runtime, so applying properties and saving are both
part of this handoff.

## Add the runtime model-view recipe

The URDF importer does not create a model-view recipe. Add one explicitly:

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
Marking the recipe as a default is a choice; it is not checked automatically
when adding a new view.

The recipe selects a builder. You do not enter graph nodes or edges. During a
run, the builder derives one node per module instance and edges from committed
connections. The Builder does not start a simulation when you add the recipe.

## Run both validation profiles

Choose **Validate → Authoring Profile** (`F6`) and open **Validation** in the
bottom **Document** panel. Then choose **Validate → Simulation Readiness** (`F7`).

| Profile | What it establishes |
| --- | --- |
| Authoring | The declared schema, references, and local assets can be checked while incomplete simulation metadata is reported as warnings. |
| Simulation readiness | The stricter structural checks promote required missing semantic data to errors. |

For the completed workshop cube, both profiles should report
**0 errors · 0 warnings**, with **Authoring validation passed** or
**Simulation readiness passed**. The table below the summary appears when
there are issues and identifies their severity, code, location, and message.

To correct an issue, find the entity named by its location in Project, edit
its Inspector fields, click the appropriate **Apply** button, then rerun the
profile. For this example, common missing values are an acceptance number,
an allowed-orientation set, the fixed constraint, or a load rating.

A passing profile does not execute the backend, check arbitrary URDF source
names against its mechanical files, or prove that two faces will mate. An
empty connector catalog can pass structural validation, and an absent runtime
view may be rejected only at launch. Check that the intended two connector
instances and default runtime recipe actually exist.

## Save the current workspace pack

Choose **File → Save** (`Ctrl+S`). The state indicator should change from
**Unsaved changes** to **Saved**, and the status bar reports **Saved** followed
by the pack directory.

Save writes canonical YAML through a staged, verified directory replacement.
It preserves the pack's mechanical assets while persisting semantic edits.
Formatting, comments, or YAML anchors from manually written documents are not
preserved as an editing format.

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
