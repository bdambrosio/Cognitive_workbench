# Composite Baseline — 2026-07-09

Three consecutive full runs against an unchanged harness
(judge-sha `58f11cdb`, backend `google/gemma-4-31B-it` local,
harness commits 58f11cdb / aa1aedcc — the only diff between them is
the ledger dirty-check exemption, no scoring machinery). Ledger rows
3–5 (`ts` 2026-07-08T22-14-53Z, 23-41-15Z, 2026-07-09T01-08-30Z).

| Suite | run 1 | run 2 | run 3 | mean | band (max−min) |
|---|---|---|---|---|---|
| memory_recall | 0.9880 | 0.9815 | 1.0000 | **0.9898** | 0.0185 |
| hle | 0.3333 | 0.5000 | 0.4167 | **0.4167** | 0.1667 |
| discourse_reflect | 0.6857 | 0.6072 | 0.6275 | **0.6401** | 0.0785 |
| introspective_fidelity | 0.8611 | 0.8056 | 0.8333 | **0.8333** | 0.0555 |
| **composite** | 0.7170 | 0.7236 | 0.7194 | **0.7200** | **0.0066** |

Run wall-clock: 81–87 min (Jill live on the same backend). Zero judge
errors; the discourse judge resample fired once in run 1 and once in
run 2 and rescued both (pre-hardening those runs would have failed).

## Ship gate

A harness change may ship only if a full composite run satisfies BOTH:

1. **composite ≥ 0.710** (mean − ~1.5× band, rounded down), AND
2. **no single suite below its floor**:
   memory_recall ≥ 0.97, hle ≥ 0.25, discourse_reflect ≥ 0.56,
   introspective_fidelity ≥ 0.77 (mean − band, rounded down).

Marginal result (within ~0.005 of a floor): re-run once before
concluding; n=3 bands are rough estimates, not confidence intervals.
A clear pass on composite with one suite just under its floor is a
"investigate before shipping," not an automatic block.

## Reading the numbers

- **composite band 0.0066 is much tighter than the suite bands** —
  the four-suite mean damps single-suite noise (suite wobbles were
  partially anticorrelated across these runs). This is why the gate
  is composite-first with per-suite floors as a backstop.
- **hle is the noise driver**: band 0.167 = ±2 questions on a
  12-question pin. HLE deltas smaller than ~0.17 are noise. If HLE
  ever needs to carry signal on its own, grow the pin to ~30+
  questions (≈ +15 min/run) and re-baseline.
- **memory_recall is at ceiling** (0.99 mean). It guards against
  regression but has no headroom to show improvement; a future
  weakness cycle could add harder probe tiers (new probes = new
  baseline for that suite).

## Re-baseline triggers

New baseline (3× runs, update this file) whenever any of:
pinned scenario copies change; judge model or judge code changes
(`judge.code_sha` differs); HLE pin changes; backend model changes;
suite probe/pair/primer sets change.

## History

- 2026-07-08: two pre-baseline rows (ledger rows 1–2) under the
  pre-resample discourse judge — retained as history, not comparable.
- 2026-07-08 evening: three run attempts lost to judge availability
  (discourse broken-JSON ×2 pairs; HLE ReadTimeout ×1) before judges
  gained bounded resampling (commits 9d0baa92, 58f11cdb). The
  integrity guards withheld every affected row — the ledger has never
  contained a poisoned score.
