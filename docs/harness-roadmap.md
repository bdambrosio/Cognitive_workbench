# Harness Roadmap — Measurement-Gated Improvement Loop

**Status:** Proposed roadmap (ASPIRATIONAL — M0 onward not implemented)
**Date:** 2026-07-08
**Inspiration:** Lilian Weng, *Harness Engineering for Self-Improvement*
(https://lilianweng.github.io/posts/2026-07-04-harness/)
**Companion docs:** `fire-outcome-capture.md` (Phase 1 implemented, commit
8a2bacf7 — M1/M2 build directly on it), `capability-gap-reuse-gating.md`
(superseded in intent by M4 of this roadmap).

## 1. Thesis

Weng's post argues that near-term agent capability gains come from the
harness — the system that orchestrates how the model thinks, acts,
remembers, and is evaluated — and that the tractable path to
self-improvement is a closed loop: **mine weaknesses from real
trajectories → propose one bounded harness change → validate against
held-in probes and a held-out regression suite → keep or revert.**
Everything in that loop presumes a frozen evaluator with known variance.

CW already embodies several of the post's patterns, arrived at
independently:

- **Context as evolving playbook** (the post's ACE pattern): no transcript
  accumulation; `reflection.py` is the sole write path; one-patch-per-turn
  concern edits; system prompt reassembled each turn (`format_prompt.py`).
- **Filesystem as persistent memory:** infospace Notes/Collections,
  per-concern WIP notes, `autonomy.jsonl`, `subagent_traces/`, and the
  `fire_id` join key stamped across react trace + autonomy event + outcome
  record — the substrate of a trajectory corpus already exists.
- **Explicit, inspectable sub-agents:** `remember`, `code_subagent`,
  body-sensing — thin, read-only, traced to files.
- **Evaluation infrastructure:** 10 bench suites, two-phase runner/judge,
  resumable.

What CW lacks is the post's *engine*: nothing consumes bench results.
Benches answer "how good is it today," never "did this change help."
The most important behavior — autonomous fires — has no verifier yet
(fire-outcome capture Phase 1 is the first step toward one). And
self-extension (Phase 2a) failed exactly the way the post predicts
ungrounded proposal generation fails: it proposed duplicates
(`pdf-grep`, FMP wrapper) because proposals originate from in-context
impressions, not from mined failures, and acceptance is gated by
heuristics (the reuse-gate) rather than by tests.

**The roadmap is therefore not new capability. It is wiring what exists
into a measurement-gated loop.** Every milestone's deliverable is a number
in a ledger, not a feature.

### What we deliberately do NOT import from the post

- **Evolutionary populations** (AFlow/MCTS, DGM archives, Promptbreeder):
  need fleets of candidate agents and cheap verifiable tasks. CW is a
  single persistent-identity agent with expensive, partly-subjective
  evaluation. Diversity-collapse risk and compute economics both say no.
- **Joint weight + harness optimization** (SIA): breaks the abstraction
  boundary; the post itself flags it. Out of scope indefinitely.
- **Fully autonomous harness self-modification** before the validation
  harness has proven itself manually (see M5 gating).

### Known caveat: the capability floor (STOP finding)

Zelikman et al.'s STOP result: self-improvement loops *improved* with
GPT-4 and *degraded* with weaker models. CW runs local backends, and the
local-vs-cloud question is unresolved (the 2026-04-16 OR trial produced a
suspicious −69pp result, flagged, never root-caused). Any loop stage that
asks the backend to judge or propose must be checked against this floor —
one more reason the frozen regression suite (M0) comes first, and a reason
to consider pinning *judge* calls to a cloud model even when the agent
under test is local.

## 2. Milestones

Six milestones. Each ends with a frozen-suite run and a ledger entry.
M0–M1 are the high-leverage core (a runner script and patience — almost no
new code); everything from M2 on is optional until the data argues for it.

| # | Deliverable | Primary metric | Exit criterion |
|---|---|---|---|
| M0 | Frozen composite regression suite + ledger | Composite score with empirical variance band | 3 unchanged runs; band computed |
| M1 | Fire-outcome data at scale (Phase 1 live) | Fire-outcome distribution; triage precision | ≥50 outcome-labeled fires |
| M2 | Outcome-modulated concern dynamics (Phase 2) | helped-rate ↑, hindered+ignored-rate ↓ | One tuning cycle with gain outside noise, M0 flat |
| M3 | Weakness-mining pass v1 (human-in-loop) | Δ on targeted failure cluster; regression-free cycle rate | 2 completed cycles, ≥1 accepted change |
| M4 | Self-extension re-grounded (test-first proposals) | Proposal precision; duplicate rate | First tool shipped via the new rule |
| M5 | Subagent-drafted harness edits (DGM-lite) | Same as M3, edits drafted by `code_subagent` | Gated on M3+M4 track record |

### M0 — Freeze the regression harness

**Problem.** No two bench runs are comparable today: scenario, judge
model, judge prompts, and question selection drift between runs, and no
suite has a measured run-to-run variance. Without this, no later "gain"
is distinguishable from noise. (Already bitten: HLE judge robustness bug,
n-too-small caveats in the FP8 comparison plan.)

**Deliverable.** One script (`bench/composite/run.py`) that executes a
pinned subset of existing suites end-to-end and appends one line to
`bench/composite/ledger.jsonl`. No changes to the suites themselves —
the composite runner shells out to each suite's existing `runner.py` /
`judge.py` with pinned arguments.

**Suite selection (v1)** — chosen for coverage of distinct harness
functions, all already built:

| Suite | Covers | Size |
|---|---|---|
| `bench/memory_recall` | memory write/recall path (reflection → RAG → remember subagent) | all 19 probes |
| `bench/hle` | open-book ReAct tool loop (search/fetch/process) | pinned subset, ~12 questions from the existing frozen list |
| `bench/discourse_reflect` | discourse agreements + triage/CRUD | existing manifest |
| `bench/introspective_fidelity` | trace-grounded self-report | existing four tiers |

**Pinning rules:**

1. Pinned scenario YAML (a `bench-composite.yaml` copy, never edited in
   place — a change to it is a new baseline, recorded as such).
2. Pinned judge: model ID + prompt text hashed into the ledger entry.
   Judges sit *outside* everything that self-modifies (post's
   reward-hacking guard). If judge robustness fixes are needed (e.g. the
   known `bench/hle/judge.py` issue), fix them *before* the first
   baseline run, then freeze.
3. Pinned question/probe lists checked into `bench/composite/`.
4. Record the harness git commit; refuse to run with a dirty tree unless
   `--allow-dirty` (the ledger entry is then marked dirty).

**Ledger entry (one JSON line per composite run):**

```json
{
  "ts": "...", "harness_commit": "...", "dirty": false,
  "backend": {"model": "...", "host": "..."},
  "judge": {"model": "...", "prompt_sha": "..."},
  "suites": {"memory_recall": 0.79, "hle": 0.42,
             "discourse_reflect": 0.71, "introspective_fidelity": 0.66},
  "composite": 0.645,
  "tokens": {"agent": 0, "judge": 0},
  "note": "free text — what changed since last entry"
}
```

Composite = unweighted mean of per-suite normalized scores (weighting is
a decision to revisit only if one suite dominates variance). Token counts
are recorded so cost regressions are visible alongside quality — the
post's Pareto framing, kept minimal.

**Variance protocol.** Run the composite 3× against an unchanged harness
commit. Band = max − min per suite and for the composite. **Gate for
everything after:** a change ships only if its composite is not below
baseline by more than the band. A per-suite drop beyond the band blocks
even if the composite holds.

**Verify:** three ledger lines exist with identical `harness_commit` and
a computed band recorded in `bench/composite/BASELINE.md`.

### M1 — Autonomy verifier v1 (run fire-outcome Phase 1 at scale)

Phase 1 capture is implemented (commit 8a2bacf7:
`_register_fire_outcome` / `_age_pending_fire_outcomes` in
`chat_loop.py`, reflection Stage 6, `pending_fire_outcomes.json`). What's
missing is *data*. This milestone is mostly wall-clock: run with
`--autonomy` live for 2–3 weeks.

**Deliverable.** A small readout script (`bench/autonomy_review/` is the
natural home) that aggregates `autonomy.jsonl` outcome records into:

- **Outcome distribution:** helped / neutral / hindered / ignored /
  unobserved / unobservable rates, per ledger (`valence` vs
  `user_impact`) and per concern.
- **Triage precision:** fraction of `fire` verdicts whose outcome lands
  helped-or-neutral on the relationship ledger. (Triage recall — good
  fires wrongly deferred — is unmeasurable without counterfactuals;
  acknowledge, don't fake it.)
- **Coverage:** fraction of fires that got judged at all vs aged out
  unobserved (validates the Stage-6-rides-user-turns design).

**Exit criterion:** ≥50 outcome-labeled fires and a baseline table in the
ledger notes. If coverage is poor (most fires age out unobserved), that
finding *is* the milestone result — it redirects Phase 2 toward the
judgment window, not the dynamics.

### M2 — Outcome-modulated dynamics (fire-outcome Phase 2, data-gated)

Exactly as `fire-outcome-capture.md` already scopes it, now with M1 data
in hand. Candidate knobs (pick **one per cycle**, in line with the
one-patch ethos): triage prompt wording, fire threshold / post-service
floor, defer-cooldown scaling, per-concern rhythm priors.

**Metric:** helped-rate up and hindered+ignored-rate down vs the M1
baseline, on the next ≥30 fires. **Gate:** M0 composite flat (within
band) — autonomy tuning must not degrade conversational competence.

This is the first demonstrated *closed-loop* gain: verifier signal →
harness change → re-measured improvement.

### M3 — Weakness-mining pass v1 (Self-Harness pattern, human-in-loop)

The post's core loop (Zhang et al. Self-Harness): weakness mining →
bounded proposal → held-in/held-out validation. All inputs already exist
on disk: bench `raw.jsonl` + judge outputs, react traces (exit_reason
`max_iters` / `llm_error`, format-retry counts), fire outcomes,
`subagent_traces/`.

**Deliverable.** A periodic *offline* analysis pass — a documented
procedure, not resident machinery; running it as a Claude Code session
over the artifacts is fine and is itself the post's "coding agent as
meta-level" pattern. Per cycle:

1. **Mine:** cluster failures from the corpus into named patterns with
   counts and example trajectory IDs. (Precedent: the repeated ReAct
   parse-hardening fixes — truncated actions, runaway tool-args — were
   exactly such a cluster, found reactively; this pass finds them
   systematically.)
2. **Propose:** ONE bounded harness change targeting the largest
   actionable cluster. Written up with: the cluster, the change, the
   held-in check (which existing probes / which new probe exercises the
   failure), prior attempts if any.
3. **Validate:** held-in probes improve AND M0 composite holds. Keep or
   revert. Ledger entry either way — negative results are recorded (the
   post's "negative results" challenge; a reverted change with a reason
   is corpus for the next cycle).

**Metric:** Δ on the targeted cluster's held-in probes;
regression-free-cycle rate. **Exit:** two completed cycles, at least one
accepted change.

**Artifacts:** `docs/weakness-cycles/NNN-<slug>.md` per cycle (mine →
propose → validate → verdict). Deliberately files-not-database.

### M4 — Self-extension re-grounded

Retire the reuse-gate as a standalone apparatus (its own doc already
flags it as likely over-designed). Replace with two rules that fold
capability-gap proposals into the M3 loop:

1. **Grounding:** a proposal must cite ≥3 mined failure instances
   (capability-gap Stage-5 records and/or M3 cluster members) — not a
   single in-context impression.
2. **Test-first acceptance:** the bench probe the new tool must pass is
   written *before* the tool is built. Duplicate detection falls out
   naturally: if the probe already passes with existing tools, there is
   no gap. (This is what would have caught both Phase-2a misfires —
   `pdf-grep` and the FMP wrapper — without any gating bureaucracy.)

**Metric:** proposal precision (proposals surviving validation ÷ total;
Phase-2a baseline is 0/2) and duplicate-proposal rate (target 0).
**Exit:** first tool shipped under the new rule, its probe added to the
M0 composite or a satellite suite.

### M5 — Subagent-drafted harness edits (DGM-lite) — earned, not scheduled

Let `code_subagent` (which already has read-only self-introspection over
`src/`) *draft* the bounded edits that M3 cycles propose. Human review
and the M0 gate unchanged; the no-Docker restart gate from the
self-extension design is the deployment boundary. This is the post's
endgame (agents modifying their own harness code) at minimum viable
scope.

**Hard gate:** attempt only after M3 has ≥3 accepted cycles and M4 shows
proposal precision ≥0.5 — i.e., only once the validation harness has a
track record of catching bad changes. If the local backend sits below
the STOP capability floor for this task, the finding is "not yet" and
the milestone parks.

## 3. Cross-cutting discipline

- **One ledger line per change.** Any harness change that could affect
  behavior gets a composite run and a ledger entry, even "obvious" fixes.
- **Judges are frozen and outside the loop.** Judge model/prompt changes
  create a new baseline (rerun the 3× variance protocol), never a
  mid-stream comparison.
- **One knob per cycle.** Confounded changes produce unattributable
  deltas; the ledger becomes noise.
- **Negative results are results.** Reverted changes stay in the cycle
  docs with the measured delta and a cause hypothesis.
- **Cost rides along.** Token counts in every ledger entry; a quality
  gain bought with 3× tokens is a tradeoff to see, not a win.

## 4. Do-nothing option (per project rules)

M0 + M1 involve almost no new code — a composite runner script, a
readout script, and patience. Stopping there still converts CW from
"benches exist" to "CW is measured," which is the highest-leverage ~20%
of this roadmap. M2–M5 are individually deferrable and each is gated on
the data produced by the milestones before it.
