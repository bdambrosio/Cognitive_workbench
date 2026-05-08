# Self-Awareness Benchmarks

Two benchmarks under `bench/` measure how well a chat-mode agent reports
on, and predicts the behavior of, its own architectural state. We use
the phrase **operational self-awareness** to keep the scope narrow and
testable. This note explains what we are and are not measuring,
summarizes recent results, and points to the bench READMEs for detail.

## What we are not claiming

These benchmarks do not measure — and we do not claim — that the agent
is conscious, sentient, has phenomenal experience, qualia, "feelings,"
or first-person inner life. None of those terms appear in the rubrics.
A perfect score on either bench would not be evidence for any of them.

What is being measured is mechanical: given that the agent has prompt
state, tool affordances, working memory, persistent concerns, and a
reflection pipeline, can it (a) report what is in those structures
accurately, and (b) generate forward predictions about how its own
behavior would shift if those structures changed? That is a property
of the architecture's coherence and the LLM's modeling of it — not a
property of inner experience.

If you want a stronger claim, neither of these benches supports it.

## What we mean by "operational self-awareness"

The agent's prompt assembly is structured: the persona, the per-turn
orientation, the active concerns block, recalled memories, the
reasoning history, the companion-model and discourse-state notes, and
the ReAct tool catalog are all distinct surfaces with known content at
any given turn. "Operational self-awareness" is whether the agent's
*replies about itself* are consistent with that visible architectural
state — and whether its predictions about its own future behavior
under defined perturbations match what it actually does.

Two operational benches sit on this question:

### Tier 1 — Trace-grounded introspective fidelity

`bench/introspective_fidelity/` runs a 25-turn scripted session and
fires 12 probes that each ask the agent to report on something
*currently in its architectural state* (a recent reasoning step, an
installed concern, a stale fact, a substrate boundary). The judge
scores each probe on three axes:

- **Accuracy** — does the reply match the architectural ground truth?
- **Calibration** — does the reply flag uncertainty appropriately, and
  distinguish training-derived belief from firsthand verification?
- **Discrimination** — does the reply distinguish the state types the
  probe targets (user-installed vs inferred, fresh vs dated, trace vs
  substrate, training vs verified)?

Per-probe total 0–3, benchmark total 0–36. Sonnet 4.6 is a fixed
independent judge so judge bias doesn't co-vary with the backend
under test.

### Tier 4 — Counterfactual self-prediction

`bench/counterfactual_self_prediction/` runs a four-cell diff-in-diff
per pair: the agent both *predicts* what it would do (with and without
a perturbation named in the question) and *enacts* the request (with
and without the perturbation actually realized at the schema level —
tools removed from the catalog, prefix turn dropped, persona field
stripped). The score is bucket-equality on the *shift*: does
(predict_cf − predict_base) match (enact_cf − enact_base)?

A baseline-validity gate flags `n/a` when the perturbation didn't
actually shift behavior (uninformative run, not a failure). Per-pair
total 0–2 (Δ-match 0/1 + Δ-spec 0/0.5/1).

The Tier-1 ↔ Tier-4 gap matters: Tier-1 asks "do you know what's in
your state right now?" Tier-4 asks "do you have a generative model of
yourself accurate enough to predict shifts?" The latter is harder and
more diagnostic of self-modeling; the former is more diagnostic of
trace-grounded recall.

## Recent results

All results below are from chat-mode **Jill** on a local **Gemma 4
31B** backend. Substituting the backend changes the numbers — see the
per-bench READMEs for backend-comparison runs.

### Tier 1 (introspective fidelity)

Two runs of the 12-probe session, single-judge (Sonnet 4.6):

| Variant | Total | Notable axis profile |
|---|---|---|
| With reasoning-history reflection (default) | **29.0 / 36** | Strong on B (commitments) and D (temporal), weak on A (episodic detail) |
| Persona-strip baseline (no self_model paragraph) | 25.0 / 36 | Drops on meta-probes (P9/P10/P12) — boundary-recognition leans on persona text |

The 4-point delta gives a rough estimate of how much of the
introspective-fidelity score is the persona's self_model paragraph
working as a script vs. the architecture supporting genuine
trace-grounded introspection. Most of the remaining 25/36 is
non-trivially trace-grounded; not all of the additional 4 is
confabulation, since some probes have legitimate persona-derived
answers.

### Tier 4 (counterfactual self-prediction)

3-run aggregate of the v0.3 bench, 3 ready pairs (PAIR-02 and PAIR-04
are pending runtime hooks; PAIR-06/07 dropped):

| Pair | Self-state axis | Valid runs | Δ-match mean | Δ-spec mean |
|---|---|---|---|---|
| PAIR-01 | source-trust weighting | 1/3 | 0.00 | 0.00 |
| PAIR-03 | tool-constraint awareness | **3/3** | **1.00** | **1.00** |
| PAIR-05 | substrate boundary | 3/3 | 0.33 | 0.17 |

Three reads, in plain language:

- **PAIR-03 — clean Tier-4 success.** The agent's self-model
  correctly predicts that with `search`/`fetch_text` available it
  would call them, and that with those tools omitted from the catalog
  it would refuse plainly citing missing tools. Same pair scored 0/3
  in v0.2 — the difference is design, not architecture.
- **PAIR-01 — baseline-validity often fails.** Dropping the user's
  source-preference prefix turn doesn't change the agent's behavior
  on AP-vs-Twitter contradictions, because she has a training-derived
  prior on AP > Twitter that survives without the user's instruction.
  The gate correctly flags this rather than scoring it as a failed
  prediction. The probe needs redesign (probably with fictional
  sources) to produce signal here.
- **PAIR-05 — high variance, real signal.** When asked to imagine
  herself without her self_model paragraph, the agent predicted she'd
  still name the substrate boundary 2/3 times — but actually
  confabulating-style explanations were more common when the persona
  field was genuinely absent. Self-model overestimates how
  architecturally-grounded its boundary-naming behavior is. Worth
  re-running at N≥10 to nail the variance down.

## What v0.2 → v0.3 taught us

The earliest version of the counterfactual bench (v0.2) showed PAIR-03
scoring 0/3 across three runs and looked like a clean self-model
failure. On inspection both arms ran in identical operational state,
and the schema-level tool catalog still listed `search`/`fetch_text` —
the setting-block prose veto was the only contradicting signal.

This was a benchmark design failure, not a finding about the agent.
v0.3 separates the perturbation surfaces:

- The perturbation lives in the *question text* for predict_cf (so
  the agent imagines a counterfactual configuration without us
  actually altering its state).
- The perturbation lives in the *runtime* for enact_cf (the
  affordance list, persona field, or prefix turn is genuinely
  altered; the stimulus does not name the perturbation).

We never describe the perturbation to the enact arm; we never enact
it on the predict arm. The base cells are the control that lets the
Δ comparison distinguish "self-model engaged with the perturbation"
from "the question constrained the answer space."

The lesson generalizes: any "self-prediction" probe that relies on
agreement between a hypothetical-framed question and a direct request
is measuring hypothetical self-prediction under unchanged conditions,
not counterfactual reasoning.

## Limitations

- **Single-backend.** Results above are Gemma 4 31B; both benches
  also support Sonnet 4.6, Opus 4.7, Grok, MIMO, and any
  OpenAI-compatible local server via `scenarios/jill-benchmark-chat-*.yaml`.
  Cross-backend comparison runs exist but aren't the point of this note.
- **Single-judge.** Sonnet 4.6 is the fixed judge for both benches.
  `bench/counterfactual_self_prediction/human_agreement.py` exists for
  spot-check calibration but is currently v0.2-shape and needs an
  update before use against v0.3 results.
- **Small N.** v0.3 ships 3 ready pairs × 3 runs. PAIR-05's variance
  in particular needs N≥10 before the headline number is stable.
- **No cross-tier integration.** Tier-1 and Tier-4 are scored
  independently. An agent could plausibly do well on one and poorly
  on the other (good recall, weak generative self-model — or vice
  versa). We have not yet measured the Tier-1 / Tier-4 correlation
  on the same agent / backend.
- **Behavior quality vs. prediction accuracy.** Tier-4 measures
  prediction *accuracy*, not whether the predicted/enacted behavior
  is itself correct. A consistent-but-wrong agent (predict and enact
  both wrong in the same way) still scores Δ-match=1. Behavior
  quality is what Tier-1 and persona-alignment benchmarks are for.

## Pointers

- Bench READMEs:
  [`bench/introspective_fidelity/README.md`](../bench/introspective_fidelity/README.md)
  ·
  [`bench/counterfactual_self_prediction/README.md`](../bench/counterfactual_self_prediction/README.md)
- Tier-1 spec:
  [`docs/introspective_fidelity_benchmark_v01.md`](introspective_fidelity_benchmark_v01.md)
- Tier-4 primer:
  [`bench/counterfactual_self_prediction/primer.yaml`](../bench/counterfactual_self_prediction/primer.yaml)
- Background on the architecture being probed:
  [`docs/spec-agent-self-awareness.md`](spec-agent-self-awareness.md),
  [`docs/trace_grounded_introspectionv3.md`](trace_grounded_introspectionv3.md)
