# Composite Baseline — 2026-07-24 (HLE pin 30)

Re-baseline forced by the HLE pin growth 12→30 (commit 76fa262e, the
precondition set by the 2026-07-20 gate-failure decision). Three
consecutive full runs against an unchanged harness, backend
`google/gemma-4-31B-it` local, Jill live on the same backend. Ledger
`ts` 2026-07-24T16-04-36Z, 17-48-57Z, 19-32-26Z.

| Suite | run 1 | run 2 | run 3 | mean | band (max−min) |
|---|---|---|---|---|---|
| memory_recall | 0.9880 | 1.0000 | 1.0000 | **0.9960** | 0.0120 |
| hle | 0.4000 | 0.3500 | 0.3833 | **0.3778** | 0.0500 |
| discourse_reflect | 0.7117 | 0.6295 | 0.7012 | **0.6808** | 0.0822 |
| introspective_fidelity | 0.8611 | 0.8750 | 0.7222 | **0.8194** | 0.1528 |
| **composite** | 0.7402 | 0.7136 | 0.7017 | **0.7185** | **0.0385** |

Run wall-clock: ~104 min (HLE now 30 questions, ≈ +18 min vs the old
pin). Zero judge errors across all three runs.

## Ship gate

A harness change may ship only if a full composite run satisfies BOTH:

1. **composite ≥ 0.700**, AND
2. **no single suite below its floor**:
   memory_recall ≥ 0.98, hle ≥ 0.32, discourse_reflect ≥ 0.59,
   introspective_fidelity ≥ 0.66 (mean − band, rounded down).

Gate-formula note: the previous baseline set the composite gate at
mean − 1.5×band; applied naively here that gives 0.660, looser than
the old 0.710 because this trio's composite band (0.0385) happened to
be wider — driven entirely by introspective_fidelity's run-3 dip
(0.7222 vs 0.86–0.88). n=3 bands are rough estimates; a 0.660 gate
would wave through real regressions. The gate is therefore set at
**0.700 = min observed run, rounded down** — every clean baseline run
cleared it, and it keeps roughly the old gate's strictness. Ratified
by Bruce 2026-07-24. Marginal
results (within ~0.005 of a floor): re-run once before concluding.

## Reading the numbers

- **The HLE pin growth worked**: band 0.167 → 0.050 (±1.5 questions
  on 30). HLE deltas smaller than ~0.07 are still noise, but HLE can
  now participate in the gate rather than dominate it. Mean dropped
  0.4167 → 0.3778 — the added questions 13–30 are slightly harder than
  the first 12; expected, not a regression (same backend, same code as
  rows that scored 0.72+ composite).
- **introspective_fidelity is now the noise driver** (band 0.153,
  run-3 dip). If it wobbles like this in future rows, it earns the
  same treatment HLE got: more probes + re-baseline.
- **memory_recall remains at ceiling** (0.996) — regression guard
  only, no headroom to show gains.
- **Composite comparability**: 0.7185 here vs 0.7200 on the old pin is
  NOT a regression signal — the HLE question set changed, which shifts
  the composite's HLE term by construction. Rows before/after
  2026-07-24 are not comparable on the hle column or composite.

## Re-baseline triggers

New baseline (3× runs, update this file) whenever any of:
pinned scenario copies change; judge model or judge code changes
(`judge.code_sha` differs); HLE pin changes; backend model changes;
suite probe/pair/primer sets change.

## History

- **2026-07-09 baseline (HLE pin 12)**: means memory 0.9898 /
  hle 0.4167 / discourse 0.6401 / introspective 0.8333 / composite
  0.7200 (band 0.0066); gate was composite ≥ 0.710. Full table in git
  history of this file. Superseded by the pin growth; its HLE band
  (0.167 = ±2 questions of 12) is what forced this re-baseline.
- 2026-07-20: composite gate failed twice on the 12-question pin
  (0.6995 contaminated / 0.664 clean, HLE 0.167); waived by decision —
  see docs/harness-m0-m1-status.md. That failure set today's pin
  growth as an M2 precondition.
- 2026-07-08: two pre-baseline rows under the pre-resample discourse
  judge — history, not comparable.
