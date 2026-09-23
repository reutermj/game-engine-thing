# Maintenance runbooks

Procedures for maintaining **this repo** that are non-obvious, easy to
forget, and recur: the kind of thing that otherwise gets re-derived
painfully every time. Written to be followed by a human or an agent.

## Format

One file per procedure, `<NNN>-<short-slug>.md`. Numbering just keeps
ordering stable; it isn't meaningful. Sections that have proven useful:

- **Trigger**: when to run this.
- **Gap**: what doesn't work on its own, and why. Cite the source you read,
  not a README summary (see [CLAUDE.md](../../CLAUDE.md) on investigating
  fetched Bazel repos locally).
- **What was tried**: approaches that didn't pan out, so nobody retries them.
- **Resolution**: the commands that work, and how to verify them.

Adapt as the procedure needs; consistency matters more than exact headings.

## When to write one instead of lore

- A **runbook** is a procedure you re-run: it has a trigger and steps.
- [docs/lore/](../lore/) is for a *discovery*: a surprising behavior or an
  abandoned approach, where the value is understanding, not a checklist.

If you run a procedure and learn something non-obvious on the way, the
discovery belongs in lore even though the procedure stays here.
