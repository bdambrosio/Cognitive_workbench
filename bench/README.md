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
| `coord_search/` | The v2 reference implementation. `score.py` is mechanical, LLM-free by default, and holds its extraction instrument constant across arms. Read it before writing a new scorer. Covers probes 4–5. |
| `hle/` | The single capability anchor. Pin-30 question set; see the pinning note in its README for the enforcement gap. |

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

Probes 1–3 and 6 from `docs/harness-behaviour-suite.md`: convergence
(`concerns.py` mapping), state-across-turns (tic-tac-toe), yield, and the
cost rider. No ship gate is attached to any of it yet — deliberately.
Run it as a reporting instrument across several backends first, and attach
floors only once the numbers demonstrably separate.
