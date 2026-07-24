# Learned Disposition Scorer — design note

**Status: design note, nothing built.** Drafted 2026-07-24. Provenance:
the memory-tier musing in [substack-gut-feeling-draft.md](substack-gut-feeling-draft.md)
("value learning as the last tier of the memory stack"), plus Bruce's
framing: *continual learning as a very-small LLM repeatedly trained over
variants of recent history*. This note turns the musing into a testable
v1 with explicit gates. Do-nothing remains a first-class outcome at
every gate.

## Concept

A sidecar model — much smaller than the backend — that maps a
*situation* to a *valence*: "how did acting in situations like this
tend to land?" It is trained on the agent's own outcome history, not
prompted; episodes are discarded, disposition survives. It is the
fourth memory tier from the essay (working → episodic → semantic →
disposition), and the first CW mechanism where experience would change
weights rather than text.

Two properties distinguish it from scaffold-level self-improvement
(cf. the AREX plateau discussion, 2026-07-24):

- **Open loop.** Labels come from observed user reaction
  (fire-outcome capture Phase 1), not from the model judging itself.
  The environment injects information each cycle.
- **Repeated retraining over a rolling window.** The model is cheap
  enough to retrain nightly from scratch over recent history — no
  incremental-learning machinery, no catastrophic-forgetting problem;
  drift tracking comes free from the rolling window.

## Why now (and not when the essay was written)

The essay's missing ingredient was outcome data. It now exists:
`autonomy.jsonl` holds 410 `fire_outcome` records (67 judged:
helped/neutral/hindered/ignored), each joinable to its `fire` event by
`fire_id` (present on all post-Phase-1 fires). Relevant fields:

- from `fire`: `concern_text`, `instruction`, `started_at`,
  `react_iters`, `react_exit_reason`, `response_brief`
- from `fire_outcome`: `outcome`, `valence`, `user_impact`,
  `latency_turns`, `evidence`
- from `triage`: `verdict` (defer/fire/reset), `reason` — the prompted
  judgment this model would eventually inform

Hardware: the 5060 Ti is idle for exactly this class of job (use the
safe-pin recipe — CUDA index reversal hazard between torch and
nvidia-smi ordering).

## v1: offline toy experiment (no agent integration)

**Question:** from the fire-time context alone (concern text,
instruction, time-of-day, recency features — nothing observed after
the fire), can a small trained model predict the judged outcome better
than (a) base rates, (b) the prompted backend asked the same question?

- **Label:** binary — helped+neutral vs hindered+ignored (n=67:
  53 vs 14). Secondary: regress `valence`.
- **Models, in order:**
  1. Frozen small embedder over the fire context + logistic head —
     the legible control, hours of work.
  2. A very-small LM (≤1B, e.g. a Qwen-class 0.6B) LoRA-tuned to emit
     the verdict — the actual idea under test, and the variant that
     supports "repeatedly trained."
- **Evaluation:** leave-one-out (n is far too small for a held-out
  split); AUROC plus precision at the current operating point.
  Reference number: prompted triage precision 0.791 — not directly
  comparable (triage selects which fires happen; this predicts among
  fired), so treat it as context, not the bar. The bar for G1 is
  beating (a) and (b) above.

### The research bet: variants

67 labels is not a training set. The essay's closing line ("requires
inventive replay") is the proposed answer, and it is the part with
genuine research risk:

- **Paraphrase variants:** LLM-rewritten fire contexts, label
  preserved. Cheap, probably safe, probably low-value (embedders
  already give this invariance).
- **Counterfactual variants:** LLM-perturbed contexts *with reasoned
  label flips* ("same fire at 3am / when the user is mid-task →
  ignored"). This is where sample efficiency would come from — and
  where label noise sneaks in, since the variant labels are
  model-opinion, not observation. Mitigation: train on variants,
  evaluate **only** on real judged records. If variants don't move
  real-data LOO performance, the bet fails cleanly.
- **Auxiliary signal (optional):** the 343 unjudged fires still carry
  weak labels (`react_exit_reason`, response length, subsequent user
  turn or silence). Pretrain on those, fine-tune on judged.

## Gates

- **G1 (offline, this note's scope):** toy beats base rate and the
  prompted-backend predictor on real judged records under LOO.
  Fail → write down the numbers, stop.
- **G2 (shadow):** score live fire decisions, log-only, alongside
  triage. Compare against outcomes as they accrue. No behavior change,
  so no composite row needed; a ledger row anyway when it lands, per
  ship-gate discipline.
- **G3 (act):** disposition score becomes an input to triage or to
  concern-activation dynamics — an M-track knob like any other,
  gated on the composite bench. Not designed here.

## Risks and honest caveats

- **Coverage bias:** judged fires are 0.216 of observable ones —
  labels oversample fires Bruce reacted to. The model learns "of the
  fires that get noticed, which land well," which is adjacent to, not
  identical to, "which fires should happen."
- **Nonstationarity:** the concern mix changes (658 of 1263 fires are
  one PV-monitor concern). Rolling-window retraining helps; per-concern
  leakage in LOO must be controlled (leave-one-concern-out as a
  robustness check).
- **Auditability:** constitutive, not fixable — a disposition that
  could fully explain itself would be episodic memory (essay's caveat).
  G2/G3 keep the prompted triage's stated reason alongside the score.
- **Goodhart:** once G3 modulates firing, the label source (user
  reaction) is influenced by the model. Park until G3.

## Out of scope

Backend weight changes (roadmap exclusion stands); replacing the
prompted triage; any write path into agent state before G3; online /
incremental learning (retrain-from-scratch only).
