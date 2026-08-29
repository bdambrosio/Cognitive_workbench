# Concerns: how the agent decides what to act on

*Re-derived from `src/chat/concerns.py`, `chat_loop.py`, `reflection.py`,
`claims.py`, `prompts.py` and `zenoh_io.py` on 2026-08-29. Every number and
every path below was read out of the code, not recalled. The previous version
(2026-07-19) had drifted: it listed a `derived` category that no longer exists,
named three creation paths where there are six, and predated the deletion
lifecycle entirely. For the motivating essay see
[what-agents-care-about.md](what-agents-care-about.md).*

## Two layers, one asymmetry

Two note collections per character:

- **`user_concerns`** — a model of the user. Each carries `strength` ∈ [0,1].
  **Strength decays**; conversation about the topic bumps it back up. Silence
  implies disinterest.
- **`agent_concerns`** — what the agent may act on. Each carries `activation`
  ∈ [0,1]. **Activation grows** with wall-clock time; a fire knocks it down.
  Silence implies untended work.

Strength decays, activation grows: user models are recall-driven, agent action
is pressure-driven. Both share the status vocabulary `active` / `satisfied` /
`abandoned`.

## User-concern dynamics

At user-turn entry (`_decay_user_concerns_per_turn`,
`_bump_user_concerns_on_input`):

- decay **0.05** per user turn; prune below **0.10**
- input with embedding similarity **≥ 0.5** bumps **+0.15**, at most **3**
  concerns per turn (uncapped bumping once sustained a 34-concern population
  decay could not win against)
- unbumped for **14 days** → `active` → `satisfied`
- top **5** by strength reach the prompt

Creation, update and closure ride the post-turn reflection call.

**Heat coupling.** A user concern bumped *across* strength **0.70** applies an
evidence bump to any active agent concern flagged `user_model_reviewer`, so the
review fires ahead of its rhythm while the heat is real.

## When an agent concern fires

All five must hold:

1. `activation` **≥ 0.70**
2. a non-null `instruction`
3. status `active`
4. the launcher ran with `--autonomy`
5. fire-time triage returns `fire`

**Growth** is proportional to elapsed wall-clock time, scaled so a concern at
`rhythm_hours: 24` climbs from the post-service floor to threshold in 24 hours.
Rhythms snap to **1, 2, 4, 8, 12, 24, 168**; the default is **168**. Changing
tick frequency does not change firing rhythm.

**Service** decrements on a completed fire: **−0.60** on `respond`, **−0.60**
on `yield` (the successor carries the remainder, so the parent must not re-fire
the same work), **−0.25** on `max_iters` — partial service leaves pressure in
place. A `one_shot` that exits `respond` is additionally marked `satisfied`:
the loop running to completion *is* the completion, and without this they
regrow and re-fire a finished instruction forever.

**Evidence bumps.** Non-autonomous input matching an active agent concern
(similarity **≥ 0.50**) raises activation **+0.15** and clears any cached
triage defer. Concerns flagged `polled` are skipped: their material arrives on
a clock, so talking about the topic is not evidence that new material exists.

**Fire-time triage.** At threshold, one LLM call asks whether acting *now* is
warranted. A `defer` verdict is cached and suppresses re-triage for
`rhythm/8` hours, clamped to **[1, 24]**.

**Per-tick dispatch cap: 2.** The rest stay due and surface next tick.

## What the autonomy flag does, exactly

`_handle_tick` returns early when `--autonomy` is off. Above that return:
the **stale sweep**, which is housekeeping and always runs. Below it:
**growth** and **firing**.

`_bump_agent_concerns_on_input` is **not** in `_handle_tick` and is not gated.
So with autonomy off, activation still rises on every user turn and nothing
ever fires — the number moves and has no effect. This is the normal condition
for a workflow run.

## Categories and lifetimes

Two categories, not three. `derived` was retired.

| category | default cadence | lifetime |
|---|---|---|
| `one_shot` | 1 h | 0.5 d |
| `durable` | 24 h | 120 d |

The stale sweep retires an `active` non-seed concern whose last activity
(fire, bump, creation) exceeds its lifetime, with a floor of `2 × rhythm/24`
days so a never-fired concern outlives its own rhythm.

## The six ways a concern is created

| path | fires when | notes |
|---|---|---|
| `_seed_concerns_from_config` | launch | scenario `concerns:` block, `seed=True` |
| `reflection.py` stage 4 | post-turn | the only path that *asks a model* what to create; suppressed under `workflow_mode` |
| `_spawn_concern_from_user_yield` | a **user** turn exits `yield`/`max_iters` | depth 0, no parent |
| `_create_successor_concern` | an **autonomous fire** exits `yield`/`max_iters` | depth +1, retires a one-shot parent as `superseded` |
| `_maybe_spawn_suspect_verification` (`claims.py`) | post-turn claim grading | **autonomy-gated**; `system_spawned` |
| `_spawn_concern_from_hop_exhaustion` (`zenoh_io.py`) | a peer exchange runs out of hops | needs peer tools |

A grep that finds one of these and stops will describe the system wrongly. That
has happened.

## Flags, and what each one prevents

- **`seed`** — architectural baseline. Never closed by reflection, never swept,
  never deleted.
- **`system_spawned`** — machine-scheduled work. Reflection may not close it;
  closing a claim-verification concern would be the agent cancelling an audit
  of its own claims. Was silently dropped by `create_note`'s allowlist for its
  whole life until 2026-08-16, so the guard never fired.
- **`yield_continuation`** — *the* live continuation from a user-turn yield.
  Spawning a new one retires the previous **of the same `entity`**. Not set by
  `_create_successor_concern`, so an autonomous successor is never retired this
  way. Whether one counterpart may hold several live continuations is unsettled.
- **`polled`** — skipped by evidence bumps (see above).
- **`user_model_reviewer`**, **`wip_reviewer`**, **`self_extension`** — the
  seeds heat coupling, the stalled-work loop, and capability-gap recording
  target by flag rather than by name.
- **`successor_of` / `successor_depth`** — the chain, capped at
  **`_CONCERN_SUCCESSOR_MAX_DEPTH = 2`**. At the cap no successor is created
  and a one-shot parent is retired `depth_cap`, so an autonomous chain stops
  after three links. Runner-driven work never reaches it: that path is always
  depth 0.

## Closure and deletion

**Satisfied** (`_satisfy_agent_concern`), with the reason recorded: `completed`,
`superseded`, `depth_cap`, `synth_complete`, `stale_sweep`.
**Abandoned** — a deliberate drop, only from reflection, refused for seeds and
`system_spawned`.

`_delete_dead_agent_concerns` then tombstones each dead concern **verbatim to
`<memory>/concerns_graveyard.jsonl` before deleting it** — an unwritable
graveyard blocks the delete. What counts as dead:

- `abandoned` → deleted
- `satisfied` **one-shot** → deleted; its instruction is a snapshot of a
  finished intention and must not revive
- `satisfied` **durable** → **kept**, as the recurrence-revival pool
- `seed` → never

Two schedules. **At startup, grace 0**: whatever was already dead when the
process began is discarded, so the collection a person opens holds only
concerns that can still do something. **On tick, grace 7 days**: a concern that
dies *during* a session stays for that session.

**Recurrence.** A new candidate whose similarity to an existing concern exceeds
**0.8** refreshes it rather than duplicating — unless the caller passes
`skip_recurrence`, which every continuation path does: two requests about one
topic are two debts, and merging them forgives the older.

## What reaches the prompt

Top **5** agent concerns by activation, each rendered with its activation,
`rank i/N`, and `last evidence <date>` where one exists, closing with
`Showing 5 of 13 active. Highest not shown: 0.42`. A top-K list without its
denominator reads as "these are my concerns" when it means "these five of
thirteen".

The instruction is shown at **120 characters when firing is on** — a fire
delivers it whole as the turn's input, so the line is only an awareness
summary — and **whole when firing is off**, where no fire will ever come and
this line is the only place it reaches the agent.

**No growth/bump split is reported**, because none is recorded: both writers do
`activation = min(1.0, a + x)` in place, and a saturated concern is not
decomposable by arithmetic either. Reporting one would need new per-concern
state.

## WIP, and the delivery log

Each concern carries a work-in-progress summary (**1500** chars, **rewritten**
by an LLM after each fire, not appended) so consecutive fires don't start cold.

`_record_surfaced` is separate and **append-only**: what this concern actually
said, to whom. WIP is a summariser and it ate exactly this — "I sent Bruce the
HarnessX paper" compressed to "Harness focus: HarnessX", which reads as scope
rather than as something already said, and the paper went out three more times.

**Both are written only on autonomous fires.** In a workflow run neither is
ever populated.

## Under `workflow_mode`

`src/chat/workflow.py` suppresses **discourse**, **orientation** and
**reflection**. It does **not** suppress concern creation: a `yield` carries
its remainder into the next leg as a concern, and an audit runs on yields.
Reflection is suppressed because it authors continuations *without* being told
`exit_reason`, so it cannot tell a leg that yielded from one that stopped, and
its concern competed with the yield's own.

## Observability

- `/concerns` in the CLI — populations with strengths and activations
- `<memory>/autonomy.jsonl` — append-only: fires, deferrals, successor spawns,
  capability gaps, deletions
- `<memory>/concerns_graveyard.jsonl` — every deleted concern, verbatim
- the Resource Browser (:3001), and `src/world_viewer.py` for a world with no
  live agent behind it
- a workflow run's `concern_log.jsonl` — per leg, the full set before and
  after, with the delta
