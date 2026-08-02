# Provenance & Verifiability — Levels Plan and Status

**Status 2026-08-02: Levels 1–2 committed (`0e3d0fd8`) and
live-validated; `justify` read path implemented.** Live validation
(2026-08-02, turn 2182 — a real search-web turn): `tool_meta` captured
with the full structured source list, 10 claims written to
`claims.jsonl` all with resolvable refs, `trace_claim turn:2182` green
end-to-end. The `justify` ReAct tool (below) closes the loop: Jill can
now answer "justify your response" from the persisted records.

## Goal

Jill should be able to answer **"justify your response / why should I
believe this?"** with an auditable evidence trail: every factual claim in
a reply decomposes to typed grounding (tool observation, recalled memory,
user assertion, inference, or model prior) with pointers that a
deterministic walker can resolve to the exact recorded evidence.

Target consumer (later levels): a verifier with two separable layers —
a **structural checker** (decidable: every ref resolves, hashes match,
no dangling pointers; `tools/trace_claim.py` is its kernel) and an
**epistemic grader** (LLM/entailment judgments per inference step,
graded not proven). Design vocabulary borrowed from assurance cases
(claim / evidence / warrant / defeater); justification logic (Artemov)
is the formal skin if one is ever needed.

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

## Next steps (in order)

1. **Re-validate `justify` live** — first live test (2026-08-02, turn
   2188) proved the dispatch + honest-EMPTY path but lost the race to
   the post-turn executor (hence on-demand attribution above); retest
   after restart. Same session also showed the system catching a real
   hallucination: turn 2187 (Satisfactory aluminum advice, no search)
   attributed as 9/9 `model_prior`, refs [] — and the advice was in
   fact substantially wrong. Whether Jill *should* have searched there
   is a disposition question, out of scope for provenance.
2. **Autonomous-turn claims** — post-turn work is skipped on autonomous
   fires, so outward-facing (Telegram/Bluesky) replies currently get no
   claim pass; arguably they need it most. `justify` already reports
   this gap honestly when asked during an autonomous run.
3. **Provenance probe bench** (Level-2 acceptance gate) — probes with
   known sources, judge scores whether claims + refs resolve via
   trace_claim; `bench/memory_recall` style. Build after a few live
   sessions produce real claims to calibrate against.
4. **Level 3** — content-addressed evidence snapshots at ingestion.

Known limits: old-format records (no `[Note_N]` recall tags) yield
'memory' claims with refs dropped; cross-session `source_turn_seq` joins
from pre-fix memories resolve last-match-wins (may hit wrong session).
