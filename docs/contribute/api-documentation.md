---
title: Writing Python API documentation
sidebar_label: API documentation
---

Public API documentation is part of a code contribution. Update the type
annotations and docstrings in the same change as the behavior they describe.
AutoAPI extracts that source and Docusaurus renders the reference; contributors
and reviewers remain responsible for its accuracy.

Physical APIs accept and return documented canonical SI values. New authoring
inputs can accept explicit units through `modsim_robotics.units`, then normalize
once. Declare quantity metadata for public physical fields in the closed
`units/fields.py` registry; resolve joint scalars by joint type and control mode.
Do not infer quantities from arbitrary metadata, introduce UI preference state
into stepping code, or silently reinterpret an `_m`/`_rad` parameter. UI controls
and formatters consume the same definitions, while raw JSON and transport remain
canonical. Quaternion ordering and the declared coordinate frame belong in the
contract separately from angle units.


## Document the public contract

Use Google-style docstrings. Sphinx Napoleon converts their sections for the
API renderer. Existing Sphinx field lists such as `:param name:` and
`:returns:` are also supported by the documentation checks.

For each public API change:

1. Give public modules, classes, functions, and methods a concise summary of
   their purpose. Put the main action in the first sentence.
2. Annotate callable parameters and return values, including `-> None` for
   methods that return nothing. Keep types and defaults in the Python
   signature so there is one place to change them.
3. Describe every caller-supplied parameter in `Args:` using its exact name.
   Include keyword-only parameters, `*args`, and `**kwargs` when present.
   Omit `self` and `cls`. Explain defaults, accepted values, and the meaning of
   `None` where callers need them; remove descriptions for deleted parameters.
4. Add `Returns:` for a result other than `None`, or `Yields:` for a generator.
   Explain what the value means, including ordering, ownership, and whether a
   returned object is mutable when relevant.
5. Add `Raises:` for explicit exceptions and for errors callers need to handle
   from delegated operations. State the condition that causes each error.
   Distinguish failure events from exceptions when documenting runtime APIs.
6. Explain relevant units, coordinate frames, lifecycle timing, resource
   ownership, and side effects. For example, a docking request queues intent;
   the processing pass commits the connection after backend acceptance.
7. Include a small, verified example for a nontrivial public workflow. State its
   prerequisites, use existing APIs, and show cleanup for resources it opens.
   Keep extensive tutorials in authored documentation and link to them.

Write descriptions that add information to the signature. A parameter named
`dt_s` needs its simulation-time meaning and backend timing behavior, not just
the statement that it is a float. Describe supported behavior from the current
contracts; keep proposed features out of API promises.

`RuntimeSession.create()` in `src/modsim_robotics/runtime/session.py` is a
worked example. Its docstring includes arguments, return value, actionable
errors, and a complete Generic Cube docking example with backend cleanup.
The [Python API reference](../reference/python-api.mdx) renders these sections
alongside its Python signature.

## Register public modules for extraction

`website/api/scope.py` is the shared scope definition for generation and policy
checks. `API_MODULES` lists the published modules. `PARSED_MODULES` includes
additional dependencies needed for static reference resolution; parsing a
helper does not publish its own page.

Register each new public module in the published scope as part of its code
change, and update the static dependency closure where needed. Extend the
generator when a new namespace needs support. Review its generated module page
and links before requesting review.

The current reference covers runtime sessions, canonical state, scene composition,
backend contracts, RobotPack schemas, units and conversion, and local unit
preferences. It is a first slice of the repository. The documentation
requirements also apply to public APIs outside that slice; reviewers check
them until those modules are included in automated coverage.

## Understand the checks and review

Run the policy check from the repository root:

```bash
npm --prefix website run docs:check
```

The source-based check reads the published scope without importing ModSim,
Studio, or MuJoCo. It checks module/class summaries and public callable
summaries, parameter and return annotations, parameter descriptions matching
the signature, return descriptions, and documentation of directly raised
exception types.

Existing incomplete documentation is recorded in
`website/api/docstring-baseline.json`. An unchanged signature can retain its
existing missing items while documentation improves; a new callable or changed
signature must meet the full checked contract. Documentation edits must
preserve or reduce the existing debt.

After fixing documented baseline issues, prune their resolved entries with:

```bash
uv run --no-sync python tools/docs/check_api_docs.py --update-baseline
```

This operation removes resolved debt and refuses to add new missing items.
Review its diff alongside the docstring changes. Fix missing documentation
instead of adding exceptions for new code.

Reviewers check the meaning that these structural checks cannot prove: units
and frames, lifecycle ordering, side effects, actionable errors from helper
calls, example correctness, and registration of new public modules. Passing the
checker does not replace this review.

## Regenerate and review the result

Install the prerequisites from
[Documentation generation](documentation-generation.md), then regenerate after
editing source docstrings or signatures:

```bash
npm --prefix website run docs:api
npm --prefix website run docs:dev
```

The preview and production build commands regenerate API pages and diagrams
before starting. During a running preview, saving an authored Markdown page
triggers live reload. After editing Python source, run `docs:api` again and
refresh the generated page. Check the Python signature, parameter descriptions,
return and error sections, examples, and object links.

Before submitting the change, run:

```bash
npm --prefix website run docs:check
npm --prefix website run typecheck
npm --prefix website run docs:build
```

CI checks the API documentation policy and builds the manual. Commit source,
docstrings, authored pages, and generator configuration together. Generated
Markdown in `docs/reference/api/`, staging under `website/.generated/`, and
the site build remain untracked and must not be edited manually. Public
documentation deployment is a separate, deferred setup step.

Maintainers must make the **Documentation** status check required in the
integration branch's GitHub protection rule or ruleset to use it as a merge
gate. The workflow is configured in this repository; that GitHub setting is
separate from the local setup.
