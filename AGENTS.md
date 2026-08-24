# ModSim repository instructions

Before changing the repository, read the contract relevant to the work:

- `docs/robot_pack_spec.md` for Robot Pack schema and persistence;
- `docs/docking_semantics.md` for connector and lifecycle behavior;
- `docs/backends.md` for the core/backend boundary;
- `docs/model_views.md` for generated views;
- `docs/runtime_inspector.md` for live runtime visualization; and
- `docs/studio.md` for the desktop application.

Keep core code independent of Studio and simulator dependencies. Treat URDF as
an imported mechanical asset, Robot Pack YAML as the semantic description,
`WorldState` as canonical runtime state, and graphs as generated views.

Dated plans, implementation snapshots, and former agent instructions are kept
in `docs/archive/` for historical context only; do not treat them as current
contracts or inventories.
