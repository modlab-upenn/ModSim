---
title: Import a URDF in Studio
sidebar_label: Import a URDF
---

Import a mechanical model into a new self-contained RobotPack. The Builder
copies the URDF and resolved mesh assets, creates the YAML documents and URDF
name mapping, and opens the resulting draft.

Before starting, complete [Prepare a URDF](prepare-urdf.md) and install the Studio
dependencies from [Installation](installation.md). This page uses
`workshop_cube.urdf` and the default destination
`~/.modsim/workspaces/workshop_cube`.

## Open the Builder

Launch Studio from your active Python environment:

```bash
modsim-robotics studio
```

The Builder opens at **Build your Robot Pack**, with **Open Robot Pack** and
**Create from URDF** buttons. No pack is loaded; Save and Export As remain
disabled until you open or create one. A blank launch does not require an
installed starter or RobotPack library setup.

To open an existing pack directly, supply its installed name or explicit path
when launching Studio. Omitting the selector opens this blank authoring window;
it does not choose a default robot.

## Create the draft

1. Click **Create from URDF** on the welcome screen, choose
   **File → Create from URDF** (`Ctrl+N`), or click **Import URDF** in the toolbar.
2. In **Select a URDF**, browse to `workshop_cube.urdf` and select it.
3. For the mesh-free cube, Studio proceeds directly to the destination dialog.
   For another model with unresolved mesh references, it first opens
   **Select the package or mesh asset root**. Select the directory containing
   the required local assets.
4. In **Choose a new Robot Pack folder**, browse to the `workspaces` directory
   inside your ModSim home. Enter `workshop_cube` as the new folder name.
   **Do not create the destination folder in advance.** The importer creates
   it and refuses an existing destination.
5. Confirm the destination. Wait for the new pack to appear in the Project tree
   and for the cube to render in the viewport.

The destination dialog initially suggests the source filename without its
extension. Replace that suggestion with your workspace destination. Use your
actual home directory in the dialog; do not enter a literal `~` path and assume
that the file picker expands it.

Enable **Show hidden files and folders** in the picker if `.modsim` is hidden
in your home directory. Studio remembers this choice across its file dialogs.

If you have not run setup, create the parent `workspaces` directory with your
file manager, or choose another existing working folder. Leave the final
`workshop_cube` directory uncreated so the importer can create it.

Canceling a dialog before completion leaves the import unfinished. If Studio
reports **Could not inspect URDF** or **Could not create Robot Pack**, read the
message before retrying. The [troubleshooting guide](troubleshoot-robot-pack.md)
explains malformed XML, unresolved meshes, and existing destinations.

## Inspect what was imported

The cube's root pack and module initially use `workshop_cube` as their display
name. Select these rows:

| Project selection | Expected result |
| --- | --- |
| Root pack | **Pack ID** `workshop_cube`, version `0.1.0`. |
| **Assets → workshop_cube** | The imported URDF asset. |
| **Module Types → workshop_cube** | **Module ID** `workshop_cube`, **Root link** `base_link`, **Mass (kg)** `1`. |
| **Module Types → workshop_cube → Links → base_link** | The single imported link. |
| Module **Joints** and **Connectors** | Empty for this source model. |
| **Connector Types** and **Model Views** | Empty; the importer does not infer these semantics. |

Use **Fit** above the viewport to center the cube. Drag to orbit and use the
wheel to zoom. Toggle **Collision** to compare the collision box with the
visual box. **Frames** displays the link frame; its origin should be at the
center of the cube.

You can give the pack a friendlier label without changing its identity. Select
the root pack, set **Name** to `Workshop Cube`, and click **Apply metadata**.
Then select its module, set **Name** to `Workshop Cube`, and click
**Apply module metadata**. The read-only IDs remain `workshop_cube`.

## Understand the draft files

The import immediately writes the new draft to disk. For this example it creates:

```text
workshop_cube/
  robot_pack.yaml
  assets/urdf/workshop_cube.urdf
  specs/module_types.yaml
  specs/connector_types.yaml
  specs/capabilities.yaml
  mappings/urdf_mapping.yaml
```

There is no mesh directory because the cube uses a primitive shape. The
capability and connector catalogs initially contain no authored entries. The
mapping preserves the imported URDF link and joint correspondence.

Subsequent property edits are held in memory until **Save** or **Export As**.
Use **YAML Preview** in the bottom **Document** panel to inspect their canonical
representation. It is read-only; edit through the Inspector during this workflow.

An imported draft is a starting point. A validation profile can pass when no
connectors have been declared, so that result alone does not mean the pack can
perform the docking demonstration. Add its interfaces and runtime recipe next.

Previous: [Prepare a URDF](prepare-urdf.md).
Next: [Author connectors](author-connectors.md).
