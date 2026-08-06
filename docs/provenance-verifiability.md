# Provenance & Verifiability — Levels Plan and Status

**Status 2026-08-06: Levels 1–2 live-validated; epistemic grader v1
(taxonomy tags + ordinal grades + self-audit) SHIPPED and
live-validated on all three branches; Stage 5 background verification
SHIPPED and live-validated, including the unprompted-correction
branch.** Commits: `0e3d0fd8` (L1–2), `8cf87812` (justify read path),
`f30d3f05` + `ad9bbda5` (quotes, taxonomy, grades, audit notes),
`8c1396e0` (Stage 5). Live arc 2026-08-03/04: the SpaceX probe
exercised refute (suspect volatile prior → search → led with
retraction, turn 2231), confirm (suspect → search → affirmed with new
evidence, turn 2236), and quiet (stable prior → probable, no search,
turn 2239). Stage 5 arcs: confirm/silent 2026-08-04 (turn 2247 →
Note_5933, deduped against a fresh justify-turn verification, user saw
nothing); unprompted correction 2026-08-06 (turn 2295: one
model_prior × volatile mechanism claim graded suspect → Note_6016
spawned ~30 s post-reply → probe found no support → correction posted
~12 min after the original reply, correction-first, no defense). Note
the 2295 correction was an epistemic downgrade (asserted mechanism →
"my own inference, not documented"), not a fact reversal — a
*reclassify* outcome the refute/confirm/silent design didn't name;
absence-of-support was treated as soft refutation of an overclaim
rather than "inconclusive → silent". Accepted behavior for now.

## Goal

Jill should be able to answer **"justify your response / why should I
believe this?"** with an auditable evidence trail: every factual claim in
a reply decomposes to typed grounding (tool observation, recalled memory,
user assertion, inference, or model prior) with pointers that a
deterministic walker can resolve to the exact recorded evidence.

The verifier has two separable layers — a **structural checker**
(decidable: every ref resolves, quotes are verbatim substrings of the
persisted observation, no dangling pointers; `tools/trace_claim.py` is
its kernel) and an **epistemic grader** (v1 built — see below: closed
taxonomy tags assigned semantically, then a deterministic ordinal
reduction; never numeric). Design vocabulary borrowed from assurance
cases (claim / evidence / warrant / defeater); justification logic
(Artemov) is the formal skin if one is ever needed.

**Design commitment — no invented numerics.** The claim schema carries
NO confidence numbers. Uncertainty is represented by the grounding type
and the evidence pointers themselves. Probabilities enter only as
reliabilities *calibrated from outcome data* (which leaf types / sources
later prove right or wrong), never as ad-hoc weights.

## Levels

| Level | Claim | Status |
|---|---|---|
| 1 Traceable | Any memory/claim-bearing record walks to its source turn and raw observation | **Implemented 2026-08-01** |
| 2 Cited | Replies decompose into claims with typed grounding + resolvable refs | **Implemented 2026-08-01** |
| 3 Evidenced | Citations point at sha256 content-addressed snapshots of the exact bytes seen (extend `_persist_image_url` pattern to fetched pages) | Unbuilt |
| 4 Tamper-evident | Hash-chained jsonl, externally anchored | Deferred — no third-party consumer yet |

## What shipped (file map)

Level 1 (stop the provenance losses):
- `src/utils/chat_tool_stub.py` — CapturingResourceManager retains
  `tool_metadata`; `translate_result` attaches `meta`.
- `src/tools/search-web/tool.py` — react path returns full structured
  source list as `meta` (uncapped, unlike the 8-source prose block).
- `src/chat/tools.py` + `src/chat/react.py` — `_pending_tool_meta` slot;
  per-step provenance lands in `reasoning_trace.jsonl` as `tool_meta`
  keyed by `$stepN`.
- `src/chat/memories.py` — memory notes carry `source_turn_seq`.
- `src/infospace_resource_manager.py` — allowlist warns (once per
  source_skill×key) instead of silently dropping extra_props.
- `src/conversation_store.py` — turn records keep `modality`.
- `tools/trace_claim.py` — audit walker: note → turn → observations,
  `BROKEN HOP` exit 1 on any dangling pointer.

Level 2 (claim-level citation):
- `src/chat/claims.py` (new) — post-turn pass decomposes the reply into
  claims: `{claim, grounding ∈ {retrieved, memory, user_asserted,
  context, inferred, model_prior}, refs}` → `<memory>/claims.jsonl`
  (joined by `turn_seq` + `reply_sha1`). Attribution is by content
  support, not co-occurrence (Gettier guard); invented refs are
  structurally rejected. Same function runs in production and offline.
- Recall is a 5-tuple; memories render `[Note_N · date]` in the prompt
  and trace, so claims can cite them and the model can age-discount.
- `process_text` citation discipline triggers on the structural
  binding→sources map, not the old `'Sources:'` substring test.
- `turn_seq` seeds from trace line count at first write — globally
  monotonic across restarts, restoring line N = turn N (previously it
  restarted at 1 per session, making all sidecar joins ambiguous).
- `utils/file_utils.append_jsonl` — canonical jsonl event writer
  (memories / autonomy / claims all use it).

Offline validation: attribution over four real records (live search-web
turn + three bench-introspective turns) via the Sonnet judge backend.
Correct retrieved/$step refs, inferred-with-premises, and model_prior
detection (Mariana Trench figure NOT credited to a source). End-to-end
`trace_claim` audit green; dangling-ref path exits 1.

`justify` read path (2026-08-02): a no-argument ReAct built-in
("justify your response" / "why should I believe that?"). Deterministic
and LLM-free — renders the most recent reply-to-this-source's claims
plus a resolved evidence index (search sources with URLs from
`tool_meta`, recalled notes with dates from `memories.jsonl`, the
user's words) directly from the persisted records, so the trail cannot
be re-synthesized or embellished. Matching on trace `source` skips
interleaved autonomous fires. If the turn has no claims record yet,
`justify` attributes ON DEMAND (same `attribute_claims`, persisted via
the same writer) rather than waiting: post-turn attribution runs last
behind discourse + reflection and landed ~2.5 min after the reply in
live testing (turn 2187, 2026-08-02) — a poll can't win that race. A
duplicate write from the still-queued post-turn job is benign
(last-match-wins join); on-demand also covers turns that predate claim
attribution. Files: `src/chat/claims.py` (module-level lookups +
`render_justification` + `_run_justify`), dispatch in
`src/chat/react.py`, catalog entry in `src/chat/tools.py`; tests in
`tests/test_justify.py`. The lookups intentionally duplicate ~15 lines
of `tools/trace_claim.py`, which stays standalone stdlib-only by
design.

## Epistemic grader v1 + self-audit (2026-08-04)

Taxonomy: `docs/justification-taxonomy.md` — closed vocabulary of leaf
dimensions (volatility, source-grade, quote, polarity, age, testimony),
12 inference edge types (incl. negation-from-absence + query-adequacy,
evidence-repurposing, circular-support), and structural conditions;
each entry carries a review key and a grade effect. Admission rule: an
entry stays only if it moves a grade or redirects a min-path.

Mechanics (all off the query path — attribution is post-turn, grading
is at justify time):
- Attribution also emits a verbatim `quote` per retrieved claim,
  machine-checked as a (whitespace-normalized) substring of the
  persisted `working_log`; synthesized quotes dropped, claim kept.
- Attribution tags claims from the closed vocabularies (validated like
  groundings; absent tag = legacy behavior). Volatility prompt wording
  is load-bearing: "judge the KIND of fact, not your confidence; when
  unsure, volatile" — the offline replay gate caught the local backend
  tagging "SpaceX is private" as stable, which would have suppressed
  the audit. Gate any vocab change with a replay over real trace
  records through the live backend.
- `grade_claim()` reduces grounding × tags to an ordinal grade
  (`verified > probable > unverified > suspect`; conflict off-scale)
  deterministically; `render_justification` shows per-claim grades, a
  weakest-link line, and pattern-driven audit notes (top 3 by
  severity). The self-audit hinge: the model can't reliably re-answer
  a stale fact, but reliably answers the *category* question ("is this
  the kind of fact that goes stale?") — so the audit note sends it to
  tools instead of its prior, with the retraction pre-licensed ("lead
  with the correction").
- Honesty guard: `justify` audits the most recent reply only; a trail
  must never be reconstructed from recall (paraphrase-as-provenance
  laundering, observed live turn 2219).

Stage 5 — background verification (`8c1396e0`): post-turn suspect
grades spawn a one-shot verification concern (user-yield vehicle,
`via: suspect_verification` in autonomy.jsonl) that probes each
suspect claim and posts a correction only if refuted; silent on
confirmed. Gates: `--autonomy` only, user-facing turns only, post-turn
path only; loop-free by construction (autonomous fires get no claim
pass). No cooldown yet — watch for chattiness.

## Next steps (in order)

1. ~~**Validate Stage 5 live**~~ — DONE 2026-08-06 (see status above).
   Both branches observed: confirm/silent (turn 2247) and unprompted
   correction (turn 2295). Still unobserved but no longer gating: a
   hard refute (verified positive contradiction of a changed fact) —
   the 2295 correction was absence-of-support → downgrade. Cooldown
   watch remains live (two suspect_verification spawns within 40 min
   on 2026-08-06; only one produced a visible message).
2. **Autonomous-turn claims** — post-turn work is skipped on autonomous
   fires, so outward-facing (Telegram/Bluesky) replies currently get no
   claim pass; arguably they need it most. `justify` already reports
   this gap honestly when asked during an autonomous run.
3. **Provenance probe bench** (Level-2 acceptance gate) — probes with
   known sources, judge scores whether claims + refs resolve via
   trace_claim; `bench/memory_recall` style. Build after a few live
   sessions produce real claims to calibrate against.
4. **Level 3** — content-addressed evidence snapshots at ingestion.

Deferred deliberately: Relations-based justification-graph storage +
cross-turn recursion through Note leaves, source-grade ledger, AND/OR
refs bit, oracle argument-audit, numeric calibration (waits on
accumulated grade/outcome pairs, which Stage 5 now produces as a side
effect).

Known limits: old-format records (no `[Note_N]` recall tags) yield
'memory' claims with refs dropped; cross-session `source_turn_seq` joins
from pre-fix memories resolve last-match-wins (may hit wrong session).
