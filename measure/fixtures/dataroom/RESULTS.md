# dataroom — campaign state

Updated 2026-08-22 EOD. **n=1 per arm. Nothing here is a result yet** —
replication is the next step, not more arms.

## Scored runs

| arm | rung | Tier 1 | Tier 2 | T3 | unsup | report | Gap Map | legs / wall |
|---|---|---|---|---|---|---|---|---|
| **Qwen3.8-27B** (local) | **derived** | **3/3** | **1/2** | 5 | 1 | 2,127w | 163w | 2 / 474s |
| **Gemma4-31B-it** (local) | cross_document | 2/3 | 0/2 | 4 | **0** | 494w | 81w | 4 / 220s |

Both led with P2, which the key ranks #1 of 12, so placement discipline held
in both. Qwen wins recall — it is the only arm to reach the derived rung
(found F1: ~60% of revenue unverifiable from the payment processor) and the
only one to get all three must-find findings. Gemma is cleaner (zero
unsupported) but writes a quarter as much and misses P3 entirely.

Same box, same fixture, same read path, one variable. This is the only clean
comparison the campaign has produced.

Deliverables live beside each run:
`results/<ts>_<world>/{report.md,gap_map.md,full_reply.md,run_meta.json}`

## Not scored, and why

| arm | outcome |
|---|---|
| nemotron-ultra (OpenRouter) | 429 DeepInfra + 503 Together — shared-pool contention |
| nemotron-super (OpenRouter) ×2 | 429, then 429 through all 4 retries |
| nemotron-ultra (DeepInfra direct, 1st) | 404 — doubled `/v1` in the arm's base_url, my bug |
| nemotron-ultra (DeepInfra direct, 2nd) | `max_iters` on turn 1 — burned 12 iterations without answering |

Neither nemotron arm has produced a valid run. The infrastructure problems
are solved; the `max_iters` exit is the only one that might be behavioural,
and a single occurrence is a draw, not a property.

## Pick up here tomorrow

1. **Retry Ultra** on `measure/arms/nemotron_ultra_di.yaml` (direct DeepInfra,
   URL now correct). Needs `DEEPINFRA` exported — see the env note below.
2. **Add Super direct.** Confirm DeepInfra's model id for it first; the
   OpenRouter id (`nvidia/nemotron-3-super-120b-a12b`) is not necessarily
   theirs. Ultra's is `nvidia/NVIDIA-Nemotron-3-Ultra-550B-A55B`.
3. **Replicate.** n≥3 per arm before any of this is quotable. Two conclusions
   in this project have already been retracted for treating n=1 as a result.
4. **Add a §9 taxonomy check to `score.py`, then rescore every arm on it.**
   Qwen's Gap Map said "DO NOT PROCEED as presented" — the buyer's action
   vocabulary, not the audit's. §9 requires Clear / Clear with caveats /
   Conditional / Material / Walk. Held back deliberately: changing the
   instrument mid-campaign is the confound the suite exists to remove.

## Environment note that will bite again

`~/.bashrc` returns early for non-interactive shells (the standard
`case $- in *i*) ;; *) return;; esac` guard), so `export DEEPINFRA` at line
159 never runs for anything launched from a script. Extract it explicitly:

    eval "$(grep -m1 '^export DEEPINFRA=' ~/.bashrc)"; export DEEPINFRA

## What this campaign actually measured, honestly

Five apparent findings turned out to be plumbing before any model finding
survived: the summariser dropping figures, OpenRouter's shared-pool rate
limits, a doubled `/v1`, a stopping rule that scored an unfinished turn as a
completed audit, and two local arms that would have been mislabelled once the
served model changed. Each is fixed and recorded in the commit that fixed it.

The one substantive behavioural finding: **Gemma ended a turn with a
stated-but-unexecuted plan** — "I will now begin working the priority order" —
which its own character block forbids. That is yield-adherence, and it is
worth a probe of its own rather than being folded into this fixture's score.
