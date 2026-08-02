# Provenance & Verifiability — Levels Plan and Status

**Status 2026-08-01: Levels 1–2 implemented, uncommitted, live-session
validation pending.** All code compiled, unit-verified, and validated
offline against real trace records; 73/73 existing tests pass. Nothing
has run in a live Jill session yet.

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

## Next steps (in order)

1. **Live validation** — next Jill session with a web search: check the
   newest trace record for `tool_meta`, wait for post-turn claims, run
   `python tools/trace_claim.py --world jill_chat turn:<N>`.
2. **Autonomous-turn claims** — post-turn work is skipped on autonomous
   fires, so outward-facing (Telegram/Bluesky) replies currently get no
   claim pass; arguably they need it most.
3. **Provenance probe bench** (Level-2 acceptance gate) — probes with
   known sources, judge scores whether claims + refs resolve via
   trace_claim; `bench/memory_recall` style. Build after a few live
   sessions produce real claims to calibrate against.
4. **Level 3** — content-addressed evidence snapshots at ingestion.

Known limits: old-format records (no `[Note_N]` recall tags) yield
'memory' claims with refs dropped; cross-session `source_turn_seq` joins
from pre-fix memories resolve last-match-wins (may hit wrong session).
