# Learned Disposition — tiny-LM value learning over Jill's fire decisions

**Status: design settled 2026-07-24 evening; Stage A (offline anchor)
built and passed; build-order step 1 (state capture + render) shipped
2026-07-25 in `src/chat/disposition.py`. Nothing scores anything
yet.** This
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

Split into two tiers 2026-07-25, after a pass over the ease/unease
literature (somatic markers, Carver & Scheier's velocity model,
predictive processing, ostracism, Zeigarnik, Kahneman & Klein) looking
for agent-state analogs. The tiers exist because **capture and
conditioning have different costs**: a field not logged from day one is
gone forever, while the render schema stays revisable. So capture wide,
render narrow.

**Tier 1 — capture** (`disposition_state.jsonl`, one row per real
triage): concern-local props, the full recent exchange, companion and
discourse state, hot user concerns, topic fit, pending-unjudged count,
substrate route. Wider than v1 conditions on, deliberately.

**Tier 2 — condition on** (the rendered ledger, ~18 lines):

```
== concern ==
text / instruction / activation (+threshold) / rhythm
last_fired / last_bump / prior_defer_reason / wip
fires since last real progress          ← rumination (Zeigarnik)
outcomes for this concern               ← per-concern contingency

== situation ==            ← all still INVISIBLE to triage itself
local time + user's usual activity at this hour   ← rhythm-relative
user last spoke (vs. typical gap at this hour)    ← baseline-relative
user last said (clipped)
my last 24h: N fires (typical M) · last 3 landed  ← rate + streak
unacknowledged fire count                         ← ostracism analog
hot_user_concerns (top 3 by strength)
this concern vs. current conversation             ← topic fit
recent read on this user: last N judged, K well   ← model fit / precision
states like this: N precedents, K judged          ← epistemic novelty
```

Rationale: triage today (`concerns.py:_triage_fire_candidate`) sees
only the concern-local slice, but judged outcomes ("ignored") are
mostly facts about *the user at that moment*. Text (not a feature
vector) because G1 showed content carries the ranking power (AP 0.634
with embeddings vs 0.338 without) and text transfers to unseen
concerns.

Cut from Tier 2 and left in Tier 1: dysfluency counters, react_iters,
effort/GPU budget, capability-gap and tool-failure rates. All are about
*her own machinery* and only weakly predict user reception, which is
what v1's label measures. Promotable in one render line each when the
target broadens.

**Capture principle: snapshot what is ephemeral, derive what is
reconstructible.** Counts over autonomy.jsonl (fire rates, outcome
histories, hit rate) and over the state file itself (precedent density)
are exactly recoverable by filtering on ts, so they are computed by
`derive_*` functions rather than stored — which also keeps their
definitions revisable. The derive functions must be fed only history
preceding the row's own timestamp; computing precedent density over the
whole corpus would let every training row see its own future.

**Telemetry urgency:** the situational slice is unreconstructable
retroactively — autonomy.jsonl doesn't capture it. Every triage that
runs without state logging is training data lost forever. Step 1 below
is therefore decision-free and shipped ahead of everything else.

**Consequence for the anchor:** G1's 0.730/0.757 was fit on
concern-local features that exist retroactively in autonomy.jsonl. The
Tier-2 situational lines do not exist for those 51 records and cannot
be backfilled, so **the situational slice cannot be evaluated against
the anchor until new judged records accumulate under logging** — months
at the observed judging rate. Until then the interim scorer stays on
concern-local features and the situational lines are pure telemetry.

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

1. ~~`render_disposition_state()` + shadow state logging at triage.~~
   **SHIPPED 2026-07-25** — `src/chat/disposition.py` (DispositionMixin
   + pure `derive_*` / `render_disposition_state`), captured at the top
   of `_triage_fire_candidate` and written with the verdict at exit;
   `tests/test_disposition_state.py`. Mechanical calls made:
   - **Sidecar file**, not autonomy.jsonl — that stream is the
     grep-able "what has she been doing" record and a ~2KB state block
     per triage would wreck it.
   - **Proximity join, no new plumbing.** `fire_id` is minted at
     dispatch, after triage returns a bare string; rows carry
     concern_id + ts and join offline to the nearest-following fire.
   - **Cached defers write no row** (early return at the triage cache
     hit). No fresh judgment, no label, and near-duplicate states would
     swamp the set.
   - Capture is snapshot-at-entry so `reset`'s activation decrement and
     `defer`'s reason rewrite land *after* the row is taken.
   - The `_dur`/`_ago` split and the "typical gap" line exist because
     turn timestamps are naive-local (`conversation_store`) while
     autonomy is aware-UTC; ages are computed live on one basis rather
     than differenced offline.
   - Honest rename: the model-fit line reads as recent **hit rate over
     judged acts**, not "landed as predicted" — a `defer` verdict has
     no counterfactual, so prediction accuracy is not computable.
   - Deferred (Tier 1, needs instrumentation in other subsystems):
     dysfluency counters (`repair_json_string` hits, retries,
     timeouts), context occupancy, GPU-contention flag.
   - `_disposition_topic_fit` deliberately does not call
     `_compute_thread_activation`: that helper caches its embedding in
     `_current_turn_embedding`, which `_update_thread_centroids`
     consumes — calling it off-turn would drift centroids toward
     concern text.
2. Wire embed+lr as interim shadow scorer (already trained).
3. Simulator-calibration experiment offline (imagination go/no-go).
4. Tiny LM on real + admitted-imagined; beat the anchor or stop.
5. TD over WIP chains when multi-fire arcs accumulate.
6. G3 coupling as a normal composite-gated M-cycle knob.

## Open decisions (Bruce)

- ~~Contents of the situational slice~~ — settled 2026-07-25, §1 tiers.
- ~~Confirm step 1 ships ahead of the rest.~~ — shipped.
- Simulator choice for the calibration experiment (few-shot local vs
  cloud offline).
- `_PRECEDENT_SIM` (0.70) and `_PRECEDENT_WINDOW_DAYS` (90) are
  untuned — no corpus existed to tune them against. Revisit once one
  does.

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
