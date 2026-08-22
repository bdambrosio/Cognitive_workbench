# Concerns: How an Autonomous Agent Decides What Matters

*(Rewritten 2026-07-19 against `src/chat/concerns.py`. The previous
version described the OODA-era model — three fixed character concerns,
activation-monitor thresholds, concern→task triage — deleted 2026-05-02.
For the motivating essay see
[what-agents-care-about.md](what-agents-care-about.md).)*

## The Problem

An agent sitting idle has no reason to act, and an agent that only
reacts has no persistent model of what's important. Concerns give the
agent both: a durable model of what the user cares about, and standing
evaluative pressure toward its own work that accumulates until it fires
as autonomous action.

## Two Layers, One Asymmetry

Everything lives in two note collections per character
(`src/chat/concerns.py`):

- **`user_concerns`** — a model of the user, maintained by observation.
  Each carries `strength` ∈ [0,1]. **Strength decays**; conversation
  about the topic bumps it back up. Silence implies disinterest.
- **`agent_concerns`** — what the agent itself wants to act on. Each
  carries `activation` ∈ [0,1]. **Activation grows** with wall-clock
  time; servicing it (an autonomous fire) knocks it back down. Silence
  implies untended work.

The asymmetry — strength decays, activation grows — is the design's
center: user models are recall-driven, agent action is pressure-driven.

Both collections share the status vocabulary `active` / `satisfied` /
`abandoned`. `satisfied` stays recallable and can revive through
recurrence (a new candidate whose similarity to an existing note exceeds
0.8 refreshes it rather than duplicating); `abandoned` blocks revival —
it records a deliberate drop decision.

## User-concern dynamics

Applied at user-turn entry (`_decay_user_concerns_per_turn`,
`_bump_user_concerns_on_input`):

- decay 0.05 strength per user turn; prune below 0.10
- input with embedding similarity ≥ 0.5 bumps +0.15, capped at 3
  concerns per turn (uncapped bumping once sustained a 34-concern
  population that decay could never win against)
- unbumped for 14 days → swept `active` → `satisfied`
- top 5 by strength are surfaced in the prompt

Creation, development, and closure ride the post-turn reflection call
(`src/chat/reflection.py`): reflection can capture new user concerns,
update existing ones, and close ones the conversation resolved.

**Heat coupling** bridges the two layers: a user concern bumped *across*
strength 0.70 applies an evidence bump to any active agent concern
carrying `user_model_reviewer` (the "review what the user has been
tracking" seed) — so the review fires ahead of its rhythm while the heat
is real, and the judgment of whether anything warrants action stays in
the reviewer's instruction at fire time.

## Agent-concern dynamics

An agent concern fires autonomously when **all** hold:

1. `activation` ≥ 0.70,
2. it has a non-null `instruction` (what a fire should actually do),
3. status is `active`,
4. the launcher ran with `--autonomy` (off by default), and
5. fire-time triage doesn't defer (below).

**Growth.** Activation grows per tick proportional to *elapsed
wall-clock time*, scaled so a concern with `rhythm_hours: 24` climbs
from the post-service floor back to threshold in 24 hours. Rhythms snap
to buckets (1, 2, 4, 8, 12, 24, 168h; default 168). Changing tick
frequency does not change firing rhythm.

**Service.** A completed fire decrements activation: −0.60 when the fire
ran to a natural finish (`respond`), −0.25 when it hit the iteration cap
(`max_iters`) — partial service leaves pressure in place.

**Evidence bumps.** Non-autonomous input semantically matching an active
agent concern (similarity ≥ 0.5) raises its activation — accumulating
evidence pulls a fire forward ahead of rhythm. A bump also clears any
cached triage defer: new evidence reopens the question.

**Fire-time triage.** At threshold, a cheap triage step asks whether
acting *now* is warranted. A `defer` verdict is cached and suppresses
re-triage for `rhythm/8` hours (clamped 1–24h) so a weekly concern
deferred once isn't re-asked hourly.

**Categories and lifetimes.** `one_shot` (default lifetime 0.5 d),
`derived` (2 d), `durable` (120 d). A staleness sweep retires expired
concerns `active` → `satisfied`. Concerns are created three ways:
seeded from the scenario, derived by reflection from conversation, or
spawned as successors by yield (below).

**Seeds.** The scenario YAML's `concerns:` block seeds durable agent
concerns (`seed=True`) — see
[configuration.md](configuration.md) for the fields (`text`, `name`,
`category`, `rhythm_hours`, `instruction`, `domain`,
`user_model_reviewer`, `wip_reviewer`, `self_extension`). Seeds are
architectural baseline: they are **never closed** — not by reflection,
not by the sweep. `domain` tags (e.g. `factorio`) are stamped onto fire
records so analyses can stratify (chat-only vs all-life composites).

**Closure.** Reflection can retire non-seed agent concerns the user
agreed to drop — status `abandoned`, blocking recurrence revival. The
`wip_reviewer` seed runs the escalate-or-retire loop over stalled work.

## WIP: continuity across fires

Each agent concern carries a work-in-progress summary (cap 1500 chars,
rewritten — not appended — after each completed fire) with a NEXT: line,
so consecutive fires don't start cold. This is the greedy planning
model: no upfront plan, always "most valuable next improvement."

## Yield: multi-turn work without a planner

A fire (or a user turn) that can't finish in one ReAct loop can
**yield**: the remainder is captured into a successor agent concern that
carries the work forward on its own pressure. Auditability comes from
yield points, not plan artifacts (see
the retired harness roadmap, §2).

## Fire-outcome capture

Each autonomous fire registers a pending record awaiting outcome
judgment ([fire-outcome-capture.md](fire-outcome-capture.md), Phase 1
live). Pending fires are surfaced once in the next user turn's prompt
(fire digest, ≤3 items) so the user gets a reaction opportunity;
judgment rides reflection stage 6 (`helped` / `neutral` / `hindered` /
`ignored`, ≤3 per reflection), with runtime-resolved `unobserved` /
`unobservable` when the window (3 user turns or 7 days) closes first.

## Observability

- `/concerns` in the CLI: current populations with strengths/activations
  (see [commands.md](commands.md) for the management subcommands)
- `autonomy.jsonl`: append-only event log (fires, satisfactions,
  abandonments, triage decisions)
- the Resource Browser (:3001) shows both collections with inline
  management
