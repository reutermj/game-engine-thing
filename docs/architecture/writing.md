# Writing an architecture doc

How a doc in `docs/architecture/` should read. It grew out of rewriting
[flows.md](flows.md) (2026-10-10), and changes as we learn what works.

## Who it's for

An architecture doc explains one concept of this engine to someone who
has never heard of it, and motivates the choice we made. A reader should
be able to understand *why* the engine works this way from the doc
alone, without having followed the work that produced it.

- **Say what the reader is assumed to know**, near the top: "This doc
  assumes you know what an ECS is." Assume nothing about this engine.
- **Define the engine's own terms where they first appear**, in a
  sentence, or link to the doc that does. Don't use a term before it's
  been explained, including in a summary at the top.

## Order: from the concept to the details

A reader should be able to stop as soon as they know enough. So a doc
goes from the idea to the reference, roughly:

1. **What it is**, in two or three sentences, and a one-line status
   ("Built", "Proposed").
2. **The problem**: the situation that needs it, and what we want from
   a solution. Use a real case from the engine, and give its root cause,
   not a symptom: flows exist because the layout best for storing bodies
   isn't the one best for solving them, which says more than "lookups
   are slow". State the goals directly; don't argue them against the way
   it was done before or elsewhere, which is "Alternatives considered".
   Write the constraints as problems, not as features of the answer:
   "others want to read the data in between", not "other systems can
   read it". The solution's section is where they become features.
3. **The idea, with a small example**: a toy that shows the concept
   whole, with a little code and a diagram if it helps. The toy comes
   before the real user, because the real one needs the vocabulary.
4. **How to use it**: the API a mod author writes against, explained
   through the example.
5. **When to use it, and when not**: the trade-off, with the evidence
   that settled it, stated plainly in this doc.
6. **The real example**: the engine's own use of it, now readable.
7. **Reference**: the full rules, error messages, edge cases, how it
   works inside, how it interacts with hot reload. Dense is fine here.
8. **Alternatives considered**: see below.
9. **Sources**: who we learned from, and what we learned. See below.
10. **Out of scope and open questions.**
11. **Design changes**, if any: see below.

## Describe the design as it is

- **A doc is not a changelog.** It describes what the design is now. Who
  built which part when, stage numbers and bead IDs don't belong in the
  explanation; git and the beads have them.
- **History only for changes to the design.** A design change is when
  something replaced something else for a reason: "X was replaced by Y,
  because Z." That goes in a short "Design changes" section at the end,
  dated, with the bead. Building something, or reaching a later stage of
  it, isn't a design change. Early in the project there may be none.
- **Alternatives considered go at the end.** It matters that we tried
  other approaches and measured them, and the doc should keep that, so
  nobody reinvents a rejected idea. But it isn't what a reader learning
  the concept needs first. In the body, explain what the design does and
  why it works; in "Alternatives considered", what else was weighed and
  why it lost.

## Say what the design provides, not what nothing else could

- **State the value, not a claim of impossibility.** "Flows let other
  systems read the middle of the computation" is the point. "Nothing
  else can see the middle" invites the reader to find the other way,
  and there usually is one. Our design is the solution we chose, not the
  only one possible.
- **Credit each benefit to the mechanism that provides it.** If a
  benefit really comes from a neighbouring feature (parallel work comes
  from shapes, not flows), it belongs in that feature's doc.
- **Check examples against the engine.** An example that says something
  needs this design, when the engine already does it another way, is
  wrong (one-way platforms were already a pre-solve hook on the world).
  An example of something the design makes possible but nobody has built
  yet is fine; track it in a bead.
- **Use words for exactly what the engine does.** If the engine checks
  an order you give but doesn't choose it, "schedules" will mislead.
  Where a reader is likely to assume more than is true, say so.

## Cite our sources

Most of what we build, we learned from someone: an engine whose source
we read, a paper, a talk. A doc says who, near its end, in a "Sources"
section:

- **What we learned from each, not only its name.** "Bevy's `pipe`
  passes a typed value from one system to the next, but joins the two
  into one node and can't branch: we kept the typed hand-off and made
  each use its own node" teaches something. A bare list of links doesn't.
- **Including what we decided against.** A source whose approach we
  rejected belongs here too, with what it showed us; the rejection
  itself is argued in "Alternatives considered".
- **Say how we read it**: its source at a version, its documentation, a
  paper. A claim from a manual is weaker than one from the code, and the
  reader should know which they're getting.
- **Link, and credit.** Every source has an entry in
  [docs/CREDITS.md](../CREDITS.md) (project, authors, licence, what we
  use it for), and code that implements an idea from one names it in a
  comment (CLAUDE.md, "Credit what we build on"). The doc's section is
  where the *lesson* lives; CREDITS.md is where the attribution does.

In the body, mention a source where its idea comes up ("the coloring is
Box2D v3's"), briefly, and leave the full account for the section.

## One concept per doc

- **If a doc explains two concepts, split it.** Flows and parallel
  shapes were one doc; they're separate ideas with different readers. A
  doc links to its neighbours in a sentence rather than covering them.
- **Spike results retire into the doc they produced.** A spike report
  is a record of a question being answered. Once the design is settled,
  the evidence that decided it moves into the design doc (or to the doc
  of the thing it measured) and the spike report is deleted, its inbound
  links repointed.
- **Measurement reports go where they're evidence.** A port's
  benchmarks and bit-for-bit checks belong in the doc of what was
  measured, or a retrospective, not in the concept's explanation.

## Prose

- **Plain, short sentences in the explanation.** One idea per sentence.
  The reference sections can be denser, since their readers have the
  vocabulary.
- **Lead with the shape of the argument.** If the point is "two layouts,
  each optimized for a different job", say that first, and write each
  side in the same form ("X is optimized for A, which lets us B"), so
  the reader sees the comparison before the details of either side.
- **Only as much detail as the point needs.** In the explanation, give
  each side of a contrast what it is, what it's good for, and why the
  other side can't use it: two or three lines, not a paragraph and not
  a slogan. How each side works belongs in the reference, or in the doc
  that covers it, behind a link.
- **Name the who and the what.** "That suits every other system" says
  nothing a reader can picture; "finding contacts depends on it, and so
  do a game's ray casts" does. If a sentence can't name a concrete case,
  it may not need to be there.
- **Concrete before abstract.** Show the case, then name the rule.
- **Numbers that decide something stay**, in the sentence that uses
  them; tables of measurements go in the reference or the retrospective.
