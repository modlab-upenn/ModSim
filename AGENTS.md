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

## Public API documentation

Follow `docs/contribute/api-documentation.md` when adding or changing public
Python APIs. Keep type annotations and source docstrings current, document
parameters and return values, and describe relevant exceptions, units, lifecycle
requirements, and side effects. Use Google-style docstring sections for new
code; existing reStructuredText field lists remain supported.

Register new published modules in `website/api/scope.py`. Run
`npm --prefix website run docs:check` and the documentation build before
submitting an API change. Fix new documentation gaps rather than extending the
legacy-debt baseline. Generated files under `docs/reference/api/` are build
output; change the source or generator instead.
