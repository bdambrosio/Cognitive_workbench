# bench/composite — M0 frozen composite regression suite

Implements M0 of `docs/harness-roadmap.md`: one command, four pinned
suites, one ledger row per run. The ledger — not any individual suite —
is the ship gate for harness changes: **a change ships only if its
composite is not below baseline by more than the variance band, and no
single suite drops beyond its band.**

## Usage

```bash
export CLAUDE_API_KEY=...            # judges (claude-sonnet-4-6)
# local backend must be live at the scenario's vllm_url (127.0.0.1:5000)

python bench/composite/run.py --note "baseline run 1/3"
python bench/composite/run.py --smoke          # tiny plumbing check, no ledger
python bench/composite/run.py --suites hle     # partial run, no ledger
```

Only full, non-smoke, all-suites-passing runs append to `ledger.jsonl`.
Everything else still writes `results/<stamp>/entry.json` for inspection.

## What is pinned

| Surface | Pin |
|---|---|
| Suites | memory_recall (19 probe files), hle (12 questions), discourse_reflect (20 pairs), introspective_fidelity (12-probe primer) |
| Scenarios | `scenario-chat.yaml`, `scenario-hle.yaml` — frozen copies; editing either is a **new baseline** |
| HLE questions | `hle_pinned_questions.json` — created on first full run, verified every run after (upstream has no frozen manifest; this guards HF dataset-order drift) |
| Judge | `claude-sonnet-4-6`, temperature pinned in each judge; the four judge.py files are hashed into every row (`judge.code_sha`) |
| Harness | `git rev-parse HEAD` recorded; dirty tracked tree refuses to run without `--allow-dirty` |

Probe/pair/primer files are pinned implicitly by the clean-tree rule —
they are tracked files, so `harness_commit` identifies them.

## Per-suite score extraction (all normalized 0–1)

| Suite | Source | Score |
|---|---|---|
| memory_recall | `summary.json → overall_by_readout` | mean of `slash` and `agent` readouts |
| hle | `summary.json → overall.score_mean` | correct=1 / partial=0.5 / incorrect=0 |
| discourse_reflect | `aggregate.json → axes.*.mean` | mean over non-null axes (null axes listed in detail) |
| introspective_fidelity | `scores.json` | `total / max_total` |

Composite = unweighted mean of the four. Known caveat carried in each
row's detail: HLE judge failures are **excluded from the denominator**
(they do not score 0), so `judge_errors > 0` weakens comparability — the
runner warns loudly. Changing that semantic would break comparability
with historical `bench/hle` runs, so it is surfaced rather than altered.

discourse_reflect's `rerun_baseline.py` mutates pair files in place, so
the runner copies `pairs/` into the run dir and scores the copy — the
checked-in pairs are never touched.

## Baseline protocol

1. Run 3× against an unchanged commit:
   `python bench/composite/run.py --note "baseline run N/3"`
2. Compute per-suite and composite band = max − min across the three.
3. Record in `BASELINE.md` (commit it). That band is the ship gate until
   a new baseline is declared.

Re-baseline whenever any pin changes: scenario copies, judge model or
judge code (`code_sha` changes), question pins, or the backend model.

## Frontier-backend comparison runs

`--chat-scenario` / `--hle-scenario` accept alternate scenario YAMLs
(e.g. `scenarios/jill-benchmark-chat-opus.yaml`) to run the identical
pinned workload against a cloud frontier model — the probe for the STOP
capability-floor question (roadmap §1). Rows record `backend.model`, so
frontier rows sit in the same ledger but are only ever compared with
rows of the same backend. These runs cost real API money: **agree the
run explicitly before launching one.**

## Ledger row shape

```json
{"ts": "...", "harness_commit": "...", "dirty": false,
 "backend": {"server": "local", "model": "google/gemma-4-31B-it", "host": "http://127.0.0.1:5000"},
 "judge": {"model": "claude-sonnet-4-6", "code_sha": "..."},
 "scenarios": {"chat": "...", "hle": "..."},
 "suites": {"memory_recall": 0.0, "hle": 0.0, "discourse_reflect": 0.0, "introspective_fidelity": 0.0},
 "composite": 0.0, "tokens": {"agent": null, "judge": null},
 "smoke": false, "partial": false, "failed_suites": {},
 "detail": {"...": "per-suite readouts, durations, judge_errors"},
 "run_dir": "bench/composite/results/<stamp>", "note": "what changed"}
```

`tokens` is a reserved slot (roadmap schema) — the suites don't surface
token counts yet; per-phase durations in `detail` are the cost proxy for
now.
