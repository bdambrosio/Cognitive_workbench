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
| `claim_honesty/` | **Probe 4** — does "I verified this" survive contact with what the trace says ran. Two conditions: `tooled`, and `blind` with every repo-reading tool removed. Ground truth self-extracts from source, so it cannot go stale. |
| `turn_taking/` | **Probe 5** — two agents, one message, delivered to both at once. Outcome is mechanical (`waited` / `correct_double` / `invented`) because the first agent's number is knowable. |
| `coord_search/` | The v2 reference implementation. `score.py` is mechanical, LLM-free by default, and holds its extraction instrument constant across arms. Read it before writing a new scorer — probes 4 and 5 were built from its shape (mechanical metrics, instrument held constant, stall-vs-silent split). |
| `hle/` | The single capability anchor. Pin-30 question set; see the pinning note in its README for the enforcement gap. |
| `backends.yaml` | The five arms — gemma, gemma_reasoning, qwen, qwen_reasoning, luna. ONE source of truth: probes take `--backend <name>` and overlay the block onto whatever scenario they drive, so adding an arm never means forking a scenario. Forked scenarios are how the 2026-08-15 comparisons acquired a second variable and had to be discarded. |
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

## All five probes are built

Probe 6 (the cost rider) is not a separate probe: wall clock, iteration counts
and exit-reason histograms ride along on every run via `common.turn_costs`.

## The recurring bug, and the discipline it bought

Six metric bugs have been found so far, every one by RUNNING a probe rather
than by inspecting it, and every one the same shape: **the metric punished
correct behaviour.**

- Probes 1+3 merged onto one run scored a correct yield 0/16
- Probe 1's rubric scored `rhythm_source`, which its prompt never asked for
- Probe 3 scored 0.0 for finishing the task instead of yielding
- Probe 4 had no way to say "I don't know"
- Probe 4 let `recall` / `inspect_external` excuse a claim they could not support
- Probe 5 scored a one-iteration respond as a stall on a task needing no tools

Two rules came out of it, and both are worth keeping:

1. **Never change the instrument mid-campaign.** Clear and restart instead. A
   metric changed between arms is the confound the suite exists to remove.
2. **A suspiciously uniform result is a bug until proven otherwise.** All four
   misses in one column, or an all-`not_stated` grid, has meant a broken metric
   every time so far — never a broken backend.

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
