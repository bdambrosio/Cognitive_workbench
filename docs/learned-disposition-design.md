# Learned Disposition — tiny-LM value learning over Jill's fire decisions

**Status: design settled in discussion 2026-07-24 evening; Stage A
(offline anchor) built and passed; nothing else implemented.** This
version supersedes the morning draft, which mis-framed Bruce's idea as
supervised data augmentation. The actual idea: **a standalone tiny LM
learns a state→value mapping via RL, where most training experience is
imagined — trajectory variants rolled forward from states in recent
history — anchored by the sparse real judged outcomes.** Provenance:
[substack-gut-feeling-draft.md](substack-gut-feeling-draft.md)
(disposition as the fourth memory tier; "inventive replay").

## Architecture in one paragraph

At fire-time triage, a deterministic template renders the decision
*state* as text. A tiny LM (≤1B, standalone process, never the vLLM
substrate) maps that state to a value v ∈ [0,1]. It is retrained
offline (nightly-from-scratch, rolling window) on a mixture of real
transitions (few, anchoring) and imagined trajectory variants generated
by a large LLM from real logged states (many, admitted only after the
simulator passes a calibration gate against real outcomes). The value
first shadows triage (log-only), later becomes one evidence line in
the triage prompt — an ordinary composite-gated M-cycle knob.

## 1. State model

Two-part rendered text block, **fixed schema, deterministic template**
— a ledger, not a narrative. No learned summarizer inside the state
pipeline in v1 (that is a second model to validate; revisit later —
the AREX `update_context` move).

```
== concern ==
text / instruction / activation / rhythm / last_fired / last_bump
wip (incl. NEXT: line) / prior_defer_reason

== situation ==            ← all currently INVISIBLE to triage
local_time (weekday + clock)
user_last_turn age         ← presence/engagement
recent_exchange (last ~2 turns, clipped)
hot_user_concerns (top 3 by strength)
autonomous_fires_last_24h (+ unacknowledged count)
```

Rationale: triage today (`concerns.py:_triage_fire_candidate`) sees
only the concern-local slice, but judged outcomes ("ignored") are
mostly facts about *the user at that moment*. Text (not a feature
vector) because G1 showed content carries the ranking power (AP 0.634
with embeddings vs 0.338 without) and text transfers to unseen
concerns.

**Telemetry urgency:** the situational slice is unreconstructable
retroactively — autonomy.jsonl doesn't capture it. Every triage that
runs without state logging is training data lost forever. Step 1 below
is therefore decision-free and should ship ahead of everything else.

## 2. Insertion point

One consumption site: entry of `_triage_fire_candidate` (the flow
already funnels every autonomous action through it).

- **G2 shadow:** render state → scorer → v; both appended to the
  triage/fire autonomy events. No behavior change. Scorer down →
  null + fail open (triage's own failure discipline).
- **G3 act (later, one knob):** v becomes an evidence line in the
  triage prompt ("disposition: fires like this have landed badly,
  v=0.81") — the prompted LLM keeps the verdict, reasoning stays
  legible in `triage_reason`. Explicitly NOT a hard gate in the first
  cycle (opaque; scorer becomes a single point of failure). Modulating
  activation dynamics instead was considered and rejected: it acts
  before any judgment sees context.

Labels keep arriving via existing machinery (reflection stage 6
fire-outcome capture). Serving: separate small process on the 5060 Ti
(safe-pin recipe) or CPU; short timeout.

## 3. Trajectory structure: per-concern WIP chains

The RL is honest, not decorative, because trajectories already exist
in the flow: successive fires of one concern each read and *rewrite*
the WIP summary.

- state  = (concern, WIP, situation) at threshold
- action = fire / defer / reset
- next   = rewritten WIP at next threshold
- episode = concern lifetime; terminal = satisfied/retired
- value  = discounted future helped-ness of the concern's remaining arc

TD across consecutive fires of the same concern is the first real
credit-assignment target (a fire that "helped" but left WIP in a
doomed configuration was actually bad). Current judged data is
bandit-shaped (latency ~1 turn), so v1 objective is fitted value
regression; TD arrives when the logs contain enough multi-fire arcs.

## 4. Imagination and its discipline

Offline training job, per cycle:

1. **Real transitions** from shadow logs + judged outcomes — few,
   precious, the anchor and the only evaluation data.
2. **Imagined variants**: big LLM conditioned on a *real* logged
   state perturbs the situational slice (same fire at 2am /
   mid-Factorio / three unacknowledged fires pending) and rolls
   forward to an imagined outcome.
3. **Calibration gate (go/no-go for imagination):** the simulator
   must predict real held-out judged outcomes above an agreement
   threshold before its imagined labels are admitted. The zero-shot
   backend already FAILED this test (AUROC 0.452, G1) — so the
   simulator needs few-shot conditioning on the judged history, or a
   cloud model offline; if neither clears the bar, imagination waits.
   Imagination may amplify the real signal, never substitute for it.
4. **Tiny LM**: fitted value regression on real (high weight) +
   admitted-imagined (low weight); evaluated ONLY on real judged
   records against the frozen Stage-A anchor. Can't beat the anchor →
   deleted, not shipped.

## Stage A results (the anchor) — 2026-07-24, commit 07550b73

`bench/disposition/g1.py`, 67 judged fires (14 bad), base rate 0.209:

| model | split | AUROC | CI95 | AP |
|---|---|---|---|---|
| embed+lr | LOO | 0.730 | [0.514, 0.909] | 0.634 |
| embed+lr | leave-one-concern-out | 0.757 | [0.533, 0.932] | 0.664 |
| numeric-lr | LOO | 0.682 | [0.498, 0.851] | 0.338 |
| prompted backend | zero-shot | 0.452 | [0.304, 0.605] | 0.201 |

Real outcome data contains learnable state→value signal; it
generalizes to unseen concerns; zero-shot prompting does not recover
it. n is small — "proceed," not "proven."
(Gotcha, recorded: legacy `utils/llm_api.LLM('vllm')` posts
template-less raw completions and degenerates on gemma; the production
client is `src/chat/backend.py:_ChatBackend`.)

## Build order

1. **`render_disposition_state()` + shadow state logging at triage.**
   Small, urgent, decision-free — the dataset only grows with
   wall-clock time. Ship first.
2. Wire embed+lr as interim shadow scorer (already trained).
3. Simulator-calibration experiment offline (imagination go/no-go).
4. Tiny LM on real + admitted-imagined; beat the anchor or stop.
5. TD over WIP chains when multi-fire arcs accumulate.
6. G3 coupling as a normal composite-gated M-cycle knob.

## Open decisions (Bruce)

- Contents of the situational slice — anything not logged from day
  one is gone. Proposed list in §1.
- Confirm step 1 ships ahead of the rest.
- Simulator choice for the calibration experiment (few-shot local vs
  cloud offline).

## Risks (unchanged from v1 draft)

Coverage bias (judged = 0.216 of observable — labels oversample fires
Bruce reacted to); concern-mix nonstationarity (rolling window +
leave-one-concern-out checks); auditability loss is constitutive
(G2/G3 keep the prompted reason alongside the score); Goodhart once
G3 modulates firing (the label source becomes model-influenced —
park until then).

## Out of scope

Backend weight changes; replacing prompted triage; learned state
summarization (v1); online/incremental training (retrain-from-scratch
only).
