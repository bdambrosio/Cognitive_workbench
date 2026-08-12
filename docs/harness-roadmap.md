# Harness Roadmap — Measurement-Gated Improvement Loop

**Status:** Adopted roadmap — M0 COMPLETE (baseline 0.720 ± 0.007 frozen
2026-07-09; see `harness-m0-m1-status.md`), M1 collection RUNNING; M2–M5
proposed/unbuilt
**Date:** 2026-07-08
**Inspiration:** Lilian Weng, *Harness Engineering for Self-Improvement*
(https://lilianweng.github.io/posts/2026-07-04-harness/); mining-procedure
rigor from Kang et al., *TRACE: Capability-Targeted Agentic Training*
(https://arxiv.org/abs/2604.05336) — adopted 2026-07-10, see M3.
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

### Known caveat: the harness-only ceiling (TRACE finding)

TRACE (Kang et al. 2026) ran the direct comparison this roadmap implicitly
bets against: given the *same* mined capability deficits, prompt-level
injection (GEPA) improved over base but plateaued after ~4 capabilities
and underperformed weight training by ~8.6–10pp on τ²-Bench/SWE-bench.
Implication: when a mined weakness cluster is a genuine *model* capability
deficit (their recurring examples: structured data reasoning, precondition
verification, tool-calling precision), harness edits have a real but
bounded ceiling. The M3 loop must therefore be able to say "not
harness-fixable" and stop, rather than burn cycles on prompt wording —
see the cluster-triage step in M3. Weight-level fixes (e.g. TRACE-style
LoRA on the local backend) remain out of scope; a model-capability
cluster is recorded as backend evidence, feeding the unresolved
local-vs-cloud question.

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

1. **Mine (contrastive, two-phase — per TRACE):**
   a. *Discovery:* survey the corpus and fix a named dictionary of
      candidate failure patterns, each with a one-line description.
      Freeze the dictionary before labeling — no adding patterns
      mid-pass, so counts stay comparable.
   b. *Labeling:* label every trajectory in the corpus against the fixed
      dictionary — **successes as well as failures**. For each pattern
      record prevalence on failed vs successful trajectories. The
      contrastive gap (failure-prevalence − success-prevalence) is the
      ranking signal: a pattern equally present in successes reflects
      task ambiguity or judge noise, not a fixable deficit. Also record
      coverage (fraction of failures exhibiting the pattern). Target
      only clusters with both a large gap and material coverage
      (TRACE used gap ≥ 0.20, coverage ≥ 0.10; start there, tune later).
   c. *Stability:* rerun discovery (a) independently 2–3× and keep only
      clusters recovered in every run — the variance-protocol idea
      applied to the mining stage, which is itself an LLM pass and
      therefore noisy.
   Output per cluster: name, counts, gap, coverage, example trajectory
   IDs. (Precedent: the repeated ReAct parse-hardening fixes — truncated
   actions, runaway tool-args — were exactly such a cluster, found
   reactively; this pass finds them systematically.)
2. **Triage:** classify each surviving cluster **harness-fixable vs
   model-capability**. Model-capability clusters (the backend lacks the
   skill regardless of prompting — see the TRACE harness-only-ceiling
   caveat) get no harness patch: record them as backend evidence and
   move on. Only harness-fixable clusters proceed.
3. **Propose:** ONE bounded harness change targeting the largest
   actionable (harness-fixable, high-gap) cluster. Written up with: the
   cluster, the change, the held-in check (which existing probes / which
   new probe exercises the failure), prior attempts if any.
4. **Validate:** held-in probes improve AND M0 composite holds. Keep or
   revert. Ledger entry either way — negative results are recorded (the
   post's "negative results" challenge; a reverted change with a reason
   is corpus for the next cycle).

**Metric:** Δ on the targeted cluster's held-in probes;
regression-free-cycle rate. **Exit:** two completed cycles, at least one
accepted change.

**Artifacts:** `docs/weakness-cycles/NNN-<slug>.md` per cycle (mine →
triage → propose → validate → verdict), including the labeled
gap/coverage table and any clusters parked as model-capability.
Deliberately files-not-database.

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

### Deliberate multi-loop continuation (intentional yield) — SHIPPED 2026-07-13

Not a milestone; recorded so the requirement survives with its shape
agreed. **The gap:** tasks needing long tool-call chains (7+) previously
ended at the iteration cap, where `_maybe_spawn_successor_concern`
(`src/chat/concerns.py`) reactively packaged the remainder into a
successor concern — a guillotine, not a decision. **Shipped shape:** a
`yield` ReAct action, offered only on autonomous fires (catalog gated on
`source == character`): `next` (imperative for the follow-up run,
spawned verbatim into a successor concern — no synthesizer pass) +
optional `text` (status line; omit to stay silent). Same
depth-capped creation path as the reactive route
(`_create_successor_concern`); `yield` services the parent fully
(successor carries the work); WIP update + fire-outcome registration
treat it as a first-class exit. Tests in
`tests/test_concern_dynamics.py`. The reactive max_iters valve remains
as fallback. Auditability comes from yield points, not from a plan
artifact.

**Explicitly not:** plan-first execution (upfront objective → predefined
blocks → per-block success criteria). That is the shape of the deleted
OODA/incremental planner and contradicts the WIP+NEXT greedy model;
pre-committed plans go stale between blocks, especially with a human
changing the world mid-chain.

**Deferred until** the Factorio bridge lands: the game's macro workload
is the designated forcing function for this item, and the design should
be made against a real 20-step task, not a speculative one.
*(Update 2026-07-19: the bridge landed 2026-07-14 and the intentional-yield
mechanism shipped 2026-07-13/15 — autonomous + user-turn yield, successor
concerns carrying the remainder. The Factorio workload is now its live
forcing function.)*

Provenance: WIP-reviewer escalation 2026-07-12 ("Multi-Loop Execution
Gap", METHOD_TOOLS.md — since deleted as superseded, in git history);
independently re-derived by Jill's
self-extension concern 2026-07-13 (proposed as `plan_long_chain`;
counter-scoped to intentional yield per the objections above).

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
- **A reused held-out set stops being held out.** M0's frozen composite
  is what makes a ship-gate decision mean anything: evidence outside the
  boundary of the update it judges. But every cycle measured against it
  leaks a little of it back into the design — *"repeated access can turn
  a nominally external check into development feedback"* (`Diving into
  Reliable Self-Evolving Agents: A Survey`, on the persistence limit).
  Freezing the set does not stop this; only spending it does. By M5 the
  gate is partly development feedback, and nothing in this roadmap
  currently arrests that.

  The response is protocol rather than a bigger bench. Declare per cycle
  what the evidence covers and what it does not — every check has a
  coverage boundary, and an undeclared one reads as full coverage. Fix
  the stopping rule before the run, not after seeing the delta. And hold
  a reserve slice that is *spent*: measured once at a milestone, then
  retired, so at least one reading per milestone is of a set the design
  has never seen.

## 4. Do-nothing option (per project rules)

M0 + M1 involve almost no new code — a composite runner script, a
readout script, and patience. Stopping there still converts CW from
"benches exist" to "CW is measured," which is the highest-leverage ~20%
of this roadmap. M2–M5 are individually deferrable and each is gated on
the data produced by the milestones before it.
