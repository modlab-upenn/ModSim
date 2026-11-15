## Change

Describe the problem and the resulting behavior. Link the relevant issue or
architecture contract when useful.

## Verification

List the commands you ran and their results, including relevant tests. Explain
any checks that could not run.

## API and documentation

- [ ] Public API changes include annotations, a concise summary, matching argument
  descriptions, return descriptions, and relevant errors, units, frames, and
  lifecycle behavior; or this change does not alter public APIs.
- [ ] New public modules are registered in `website/api/scope.py`, or no public
  modules were added.
- [ ] Nontrivial workflow examples use existing APIs, state their prerequisites,
  and were verified; or no such examples were needed.
- [ ] Changes to subsystem ownership, call ordering, persistence, transport, or
  implementation limits update the relevant Explanation prose and Mermaid
  diagrams; or this change does not alter those behaviors.
- [ ] `npm --prefix website run docs:check` and `npm --prefix website run docs:build`
  pass, and the affected reference pages were reviewed locally.
- [ ] Generated API Markdown, diagrams, staging files, and site output are absent
  from the commit.

See `docs/contribute/api-documentation.md` for the source contract and the
distinction between automated checks and review, and
`docs/contribute/documentation-generation.md` for diagram maintenance.
