# Workflows and concerns: three layers, and which one owns what

Settled 2026-08-26. Written so the question is not reopened from scratch.

A workflow run is **built on** the concern machinery — a `yield` carries its
remainder into the next leg as a concern, and an audit proceeds by yields.
That shared foundation makes it easy to conclude the two are the same thing
with different packaging. They are not. This note states the division, the
evidence for it, and the one consequence that follows.

Companion reading: `docs/concerns-architecture.md` for the concern dynamics,
and `docs/Workflow/essay-01-a-method-is-not-a-script.md` for what a method
document contains. This note is about the layers around it.

## The three layers

| layer | question it answers | who decides | mechanism |
|---|---|---|---|
| **continuation** | what happens next, given what just happened | the agent, from inside | activation, rhythm, yield remainder, per-concern WIP |
| **acceptance** | what counts as finished, and is this it | the harness, from outside | method vocabulary, deliverable markers, leg cap, run record |
| **isolation** | what is in scope at all | fixed at launch | fresh world per target, the two geofences, `workflow_mode` |

**Continuation** is the concern layer (`src/chat/concerns.py`). It carries
work across turn boundaries. Granularity is not the axis — a standing daily
patrol and a single yielded remainder are both continuation.

**Acceptance** is the method document plus the runner
(`workflows/*/method/*.md`, `workflows/*/runner.py`). The method states what
an acceptable result looks like; the runner refuses anything else.

**Isolation** is the scenario. It is neither continuation nor acceptance, and
it is the layer most often forgotten in this discussion.

## The distinction that does the work

Both the continuation layer and the acceptance layer have a notion of
completion. Concerns have satisfaction, supersession and a stale sweep. So
the division is **not** that one has an ending and the other does not. It is:

> A concern judges its own satisfaction, from the inside.
> The method and runner verify delivery, from the outside.

`workflows/claims_audit/runner.py` looping until `=== GAP MAP ===` actually
appears in a reply is the outer layer declining to accept the inner layer's
self-report. The comment there states the rule: **done is a deliverable, not
an exit reason.** It was written after a model announced completion on
2026-08-22 without producing one.

## Evidence: why acceptance cannot live in the concern layer

The obligation object (`f99fd8a8`, reverted in full at `9444c579` on
2026-08-21) was an attempt to give a concern a delivery contract. Each part
worked; the design did not hold. From the revert message:

> Obligations were exempted from recurrence merge on purpose, because two
> requests from the same person about the same topic are two debts and merging
> them forgives the older. The cost of that stance is that a duplicate never
> dies... That is not a bug on top of the design, it is the design's two ends
> pulling against each other.

The concern layer merges recurrences and sweeps the stale so that continuation
does not silt up. A delivery contract needs the opposite: persist untouched
until discharged. Exempting a concern from both is what makes it a contract
and also what makes a duplicate immortal.

This is the strongest single piece of evidence for the division, because it
was built, validated and backed out in one day.

## Consequence: a workflow is a standalone object

This follows from the layers rather than being a separate preference. A
workflow needs an isolation layer and an external judge of acceptance.
Neither is available inside a live conversational agent:

- A concern fires **inside** an agent's ongoing world, with its memory,
  companion model and prior engagements. There is no isolation from the world
  it lives in. For a client engagement that is disqualifying on
  confidentiality grounds before capability is discussed — see the
  fresh-world-per-target note in `workflows/claims_audit/scenario.yaml`.
- Inside a live session, nothing refuses "I'm done". The acceptance layer has
  no representative.

A workflow embedded in an online agent is therefore a concern with a long
instruction, which is what a concern already is. **Run a workflow standalone,
always.**

An online agent may still *start* one, and may read its deliverables
afterwards. Invocation is not embedding.

## Where the line is blurry

The runner participates in continuation: its `continue` message carries
engagement state — legs, minutes, documents opened. The useful cut is that
the **semantic** remainder — what is done, what remains, what a follow-up must
not rediscover — belongs to the yielded concern, and the runner carries only
bookkeeping. If the runner ever carries the semantic remainder, the layers
have collapsed and the concern is doing nothing.

## The placement rule

Ask which question the thing answers.

- *What happens next, given what just happened* → continuation. Build it as a
  yield and a concern remainder.
- *What counts as finished* → acceptance. Build it into the method document
  and the runner.
- *What is in scope at all* → isolation. Build it into the scenario.

Worked example. Splitting a security patrol into a collect leg and an analyse
leg is continuation, so it is a yield. Requiring that the patrol produce both
a report and a statement of what it could not see is acceptance, so it is the
runner.

## The failure taxonomy

The layers separate two failures that otherwise both read as "the agent did
not finish properly", and they have different fixes.

- **Continuation failure** — work stops and nothing restarts it. Examples on
  record: the hop-budget carrier resetting its own brake, and the 130-turn
  unattended exchange.
- **Acceptance failure** — completion is claimed without the artifact.
  Example on record: the 2026-08-22 run above.

## Settled applications

**The security audit is a workflow, and it is standalone.** It is an offline
process with a method and a deliverable; it does not belong embedded in
`jill-chat.yaml`.

**Sentinel's daily patrol stays a concern.** Continuous vigilance that notices
what changed since yesterday is continuation, and it is online and
conversational. The two are different jobs, not two implementations of one.

**Data gathering belongs to the workflow's own scaffolding.** Nothing
constrains which tools an agent uses once it holds a method, so there is no
architectural reason a workflow cannot collect its own input. Whether the
runner gathers directly or drives its isolated agent to gather is an
implementation detail.

A frozen fixture is then **another engagement**, whose target is a directory
of previously captured data, rather than a second code path. The limitation to
state in the fixture's README: a frozen engagement measures the analysis, not
the collection.

## What this does not settle

- Whether a method document outperforms the same text delivered as a concern
  instruction, holding length constant. There is no measurement. The one on
  record (2026-08-23, `src/chat/workflow.py`) compares a static prompt against
  reading the method with a tool, which is a different comparison. Note also
  that length is the variable that matters here, and an experiment holding it
  constant would answer the wrong question.
- Whether `workflows/claims_audit/runner.py` is a workflow runner or an audit
  runner. Only a second workflow will show which parts are general.
