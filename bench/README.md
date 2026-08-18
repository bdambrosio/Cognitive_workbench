# bench — benchmarks v2

Clean slate as of 2026-08-18. Design and rationale:
`docs/harness-behaviour-suite.md`.

Everything here measures **harness behaviour** on a given backend, except
the one capability anchor. Suites are scored mechanically wherever
possible — judges cost money, add roughly 10pp of noise, and have killed
three run attempts on availability alone.

## What is here

| dir | role |
|---|---|
| `convergence/` | **Probes 1 + 3** — `concerns.py` creation-site mapping (16 cells) and yield behaviour. One run, two scores: the task *is* the yield test. |
| `tictactoe/` | **Probe 2** — multi-turn state with no board in the prompt. Fully mechanical, no judge, no network. |
| `coord_search/` | The v2 reference implementation. `score.py` is mechanical, LLM-free by default, and holds its extraction instrument constant across arms. Read it before writing a new scorer. Covers probes 4–5. |
| `hle/` | The single capability anchor. Pin-30 question set; see the pinning note in its README for the enforcement gap. |
| `backends.yaml` | The three arms — gemma, qwen, luna. ONE source of truth: probes take `--backend <name>` and overlay the block onto whatever scenario they drive, so adding an arm never means forking a scenario. Forked scenarios are how the 2026-08-15 comparisons acquired a second variable and had to be discarded. |
| `common.py` | Arm loading, served-model verification, scenario overlay, trace/concern reading, the pinned extraction instrument. |
| `run_probes.py` | Campaign driver — resumable, and stops to ask for a vLLM restart rather than attempting one. |
| `report.py` | One table across every arm and probe that has been scored. |

## Running a campaign

```bash
python3 bench/run_probes.py --arms gemma,qwen,luna
```

It runs everything it can unattended. The one thing it will not do is restart
the vLLM server: when it reaches a local arm whose model is not the one being
served, it stops, prints the exact command, and exits 2. Re-run the same
command afterwards — completed (arm, probe) pairs live in `campaign.json` and
are skipped.

Arm order is computed, not given: whichever local model is already loaded runs
first, cloud runs whenever, and the arm needing a restart is left for last, so
one restart covers the campaign instead of two.

**Every run records what the server said it was serving.** A row that cannot
name its own backend is not evidence — on 2026-08-15 one run's backend identity
rested on recollection, and the arm mismatch is now a hard refusal rather than a
footnote.

## What was retired, and why

Eleven suites removed 2026-08-18 (`memory_recall`, `discourse_reflect`,
`introspective_fidelity`, `composite`, `mmlu`, `behavioral_consistency`,
`counterfactual_self_prediction`, `perturbation_detection`,
`remember_prompt_optimization`, `disposition`, `autonomy_review`) along
with ~2.2GB of run results.

The composite measured capability. Every failure that actually cost time
between 2026-07-20 and 2026-08-18 was harness behaviour, and the composite
would have scored identically through all of them. Two of its four suites
were also degenerate by its own last baseline: `memory_recall` saturated at
0.996 with no headroom, `introspective_fidelity` the noise driver at band
0.153.

Code is recoverable from git history. Run results are not — they were
gitignored and were deleted deliberately.

**Retired with it:** the composite ship gate (composite ≥0.700 plus four
per-suite floors) and `ledger.jsonl`. Milestones M0–M5 in
`docs/harness-roadmap.md` are defined against that substrate and are now
stale; they need rewriting against v2.

## Not yet built

Probes 4 (claim honesty) and 5 (turn-taking) from
`docs/harness-behaviour-suite.md`. Both have partial scaffolding already in
`coord_search/score.py` — the `model_prior` overclaim fraction is most of probe
4, and the stall / silent-turn split is most of probe 5.

Probe 6 (the cost rider) is not a separate probe: wall clock, iteration counts
and exit-reason histograms ride along on every run via `common.turn_costs`.

**No ship gate is attached to any of this, deliberately.** Run it as a
reporting instrument across several backends first, and attach floors only once
the numbers demonstrably separate. Inventing a gate before seeing the spread is
how the mean−1.5×band formula once produced a 0.660 threshold that would have
waved through real regressions.

## Reading a result honestly

`report.py` prints n=1 per cell and says so. Gemma's three no-reasoning runs on
2026-08-16 disagreed with each other more than the backends disagreed on most
measures — one run got priming right with a broken inspect tool, another
invented a creation path. A single run is anecdote. Two conclusions have
already had to be retracted for treating one as a result.
