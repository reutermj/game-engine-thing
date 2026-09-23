# Lore

Non-trivial discoveries: things that took real effort to figure out and
aren't obvious from reading the code or the architecture docs. Tribal
knowledge that would otherwise live in someone's head, or be rediscovered
painfully by the next person (human or agent) who hits the same thing.

## What belongs here

- A `dlopen`, linker or loader behavior that was surprising or
  under-documented.
- A Bazel rule or toolchain quirk that cost time to track down.
- Why a previously tried approach was abandoned, and what specifically went
  wrong with it.
- Any "if you don't know this, you will waste an afternoon" fact.

## What doesn't belong here

- Current design and the reasons for it: that's
  [docs/architecture/](../architecture/).
- A repeatable maintenance procedure: that's a [runbook](../runbooks/).
- Anything easily re-derived by reading the current code.

## Format

One file per discovery, named for the finding as a sentence
(`dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md`), so the index
reads as a list of facts. Keep entries short: what you hit, why it's
surprising, and what the resolution was. Say how the claim was established
(measured, read in source, or inferred), and put the measurement in the
entry, since a lesson that is later reversed goes on teaching the old rule.
When a code change overturns an entry, fix or delete the entry in the same
commit.
