# Harness Roadmap — M0/M1 Status

**Date:** 2026-07-09 (session of 2026-07-08 evening)
**Roadmap:** `docs/harness-roadmap.md`
**State: M0 COMPLETE. M1 collection RUNNING.**

## M0 — frozen composite regression suite: DONE

- `bench/composite/run.py` runs the four pinned suites end-to-end and
  appends one row per full run to `bench/composite/ledger.jsonl`.
  Full run: **81–87 min** on the local gemma-4-31B backend.
- **Baseline frozen 2026-07-09** (`bench/composite/BASELINE.md`,
  commit f774a1af), 3 clean runs at judge-sha 58f11cdb:

  | | memory_recall | hle | discourse | introspective | **composite** |
  |---|---|---|---|---|---|
  | mean | 0.990 | 0.417 | 0.640 | 0.833 | **0.720** |
  | band | 0.019 | 0.167 | 0.079 | 0.056 | **0.0066** |

- **Ship gate live:** composite ≥ 0.710 AND per-suite floors
  (memory ≥ 0.97, hle ≥ 0.25, discourse ≥ 0.56, introspective ≥ 0.77).
  One `run.py --note "<what changed>"` row per harness change.
- Hardening shipped along the way (all committed): bounded judge
  resampling in **all four** suite judges (9d0baa92, 58f11cdb) after
  judge availability killed three run attempts; ledger-poisoning
  guards in run.py (judge errors / shrunken denominators / silent
  zeros fail the suite, never average in); ledger exempt from the
  clean-tree check (aa1aedcc). The ledger has never contained a
  poisoned row.
- Verified the suites exercise the real harness, not the bare LLM:
  runners instantiate `ChatLoop` and drive `_process_user_turn`; bench
  worlds contain reasoning traces and reflection-written memories.

## M1 — fire-outcome collection: RUNNING

- Jill is live with `--autonomy` (started 2026-07-08 evening).
- Readout: `python bench/autonomy_review/outcomes.py` (built, tested;
  same commit 321a80ad). Target: **≥50 judged outcomes**; watch
  `coverage` first.
- Conversation guide: `docs/m1-collection-inputs.md` — react to fires
  visibly within 1–2 turns; honest reactions only; deliberate silence
  (`ignored`) is a valid datum used sparingly.
- Starting state: 844 historical fires, **0 outcome records** (capture
  shipped in 8a2bacf7, first live tonight). Historical skew to note:
  591/844 fires are the PV-monitor concern; triage split fire=334 /
  defer=495 / reset=1.

## M1 coverage fix — fire digest (2026-07-12)

First readout (2026-07-12): 41 records but only 3 judged; coverage
0.125 (21 unobserved, 17 unobservable) — the instrument was starving.
Two changes, both instrument-side, not M2 tuning:

- **Fire digest** (`_take_unsurfaced_pending_fires` +
  `_render_pending_fires_block`): each pending fire is surfaced ONCE
  in the next user turn's system prompt so Jill can mention it and the
  user can react inside the judgment window. Harness change — needs a
  composite ledger row before "known good".
- **Dialog-initiation seed** (`jill-chat.yaml`): HF daily-papers
  concern, 24h rhythm — a probe of whether passivity is
  portfolio-shaped (fire volume was ~97% three clock-driven monitors).

Outcome records from before these changes mostly measure the
instrument's blindness; treat post-digest data as the real M1 set.

## WIP reviewer + retire path (2026-07-13)

- **WIP reviewer seed** (`jill-chat.yaml`, `wip_reviewer: true`): 24h
  escalate-or-retire pass over the WIP accumulated across
  agent_concerns; its fires see every active concern's WIP in context
  (`_collect_concern_wip` → `_render_wip_review_block`). At most one
  item per fire, proposal-in-words only.
- **Retire execution** (`agent_concerns_closed`): reflection stage 4
  gains a closure list mirroring `user_concerns_closed` — user assent
  in conversation marks the concern `abandoned` (blocks recurrence
  revival; seeds never close; `concern_abandoned` event logged to
  autonomy.jsonl). Before this, nothing in the runtime could ever
  write `abandoned`.
- **NEXT: line in WIP rewrites** (2026-07-13 follow-up): the post-fire
  WIP rewrite prompt (`_update_concern_wip`) now ends the WIP with a
  'NEXT: <step>' line when a genuine unfinished arc exists (omitted
  otherwise — no invented steps), and the reviewer's inventory framing
  tells it to weigh NEXT-bearing entries first. Write side of the
  "greedy planner" reading of WIP; feeds the reviewer non-procedural
  material.
- **Seed instruction sync**: `_seed_concerns_from_config` now syncs a
  seed's `instruction` from scenario YAML on restart (runtime dynamics
  untouched; sync never deletes). Fixes silently-stale seed procedures
  — observed live 2026-07-12: the self-extension concern fired its
  original instruction, missing the inspect-first clause added
  2026-06-19.

- **Substrate line** (`_compute_substrate_line`): session-start system
  prompt section with HEAD + uncommitted-file count + commit subjects
  since the last session's HEAD (marker file in `<memory>/`) + backend
  model. Push-shaped fix for the self-observed blind spot ("I can't
  tell when my substrate changed", fire of 2026-07-12 19:21); her
  `inspect_version` tool proposal was declined — `exec-script` + git
  already covers ad-hoc depth.

Both the reflection-prompt change and the reviewer are harness changes
— composite ledger row owed before "known good".

## Next steps

1. **(Bruce, ongoing)** Daily-ish conversation per the collection
   guide; weekly `outcomes.py` check. ETA to 50 judged: ~2–3 weeks.
2. **(M2, gated on data)** First outcome-modulated tuning cycle — one
   knob — once outcomes ≥50 and coverage is known. Candidates already
   visible: PV-concern rhythm/verbosity, triage prompt, judgment
   window (if coverage is poor). **Leading candidate: generic
   sensor-event → concern-activation ingress (jill-integration.md §4)**
   — time is currently the only high-bandwidth activation source, so
   behavior collapses to scheduled monitors while opportunistic /
   dialog-initiating concerns starve at threshold.
3. **(Ship-gate discipline, immediate)** Any harness change now gets a
   composite row before merging to "known good."
4. **(Optional, first weakness-cycle candidate)** Parallelize composite
   internals (judges are embarrassingly parallel; suites use
   independent worlds) — wall-clock only, no scoring change; would
   roughly halve the 85-min run.
5. **(Deferred decisions, documented in BASELINE.md)** memory_recall is
   at ceiling (no headroom to show gains); HLE deltas < 0.17 are noise
   at the 12-question pin — grow to ~30+ and re-baseline if HLE ever
   needs to carry signal alone.
6. **(Frontier comparison — ASK FIRST)** `run.py --chat-scenario
   scenarios/jill-benchmark-chat-opus.yaml ...` runs the identical
   pinned workload against a cloud model (STOP capability-floor probe).
   Costs real API money; natural trigger is after M2's first cycle if
   local tuning deltas are inconclusive.

## Session commit trail

09f1aeca roadmap doc · 321a80ad M0 runner + M1 readout (+10 review
fixes) · 9d0baa92 discourse judge resample · 9bd6e3e0 HLE pin + ledger
· 58f11cdb all-judge resampling + silent-zero guard · aa1aedcc ledger
dirty-check exemption · f774a1af baseline frozen.
