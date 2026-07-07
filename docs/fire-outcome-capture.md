# Fire-Outcome Capture Specification

**Status:** Draft / proposed (ASPIRATIONAL — not implemented)
**Audience:** Internal (chat-loop + reflection developers)
**Companion analysis:** `modular-memory/docs/jill-bridge.md` (the
cross-project design note this spec serves), though the spec stands on
internal grounds alone.

## 1. Motivation and scope

An autonomous fire currently records a **process** outcome only: the exit
reason (`respond` / `max_iters`) drives the service decrement
(`_service_agent_concern`, 0.60 vs 0.25) and the WIP rewrite. Nothing records
whether the act **helped** — did the user engage, ignore it, or push back?
Post-turn reflection is deliberately skipped on autonomous turns
(`chat_loop.py` ~1320-1345, to keep a Jill-only monologue from polluting the
user model), so the episodes that most need outcome evaluation are exactly
the ones excluded from the write path. Two consequences:

- **No autonomy bench** (README limitation): whether fires are useful and
  well-timed is judged anecdotally from `autonomy.jsonl`.
- **No training signal**: nothing accumulates that could ever calibrate
  fire/defer judgment against observed results.

This spec adds *capture only*: a signed, per-ledger outcome record for each
fire, written to `autonomy.jsonl`, judged from the cheapest reliable
evidence — the user's subsequent turns.

**Non-goals (phase 1):** no change to concern dynamics (activation, service,
rhythm, triage are untouched); no new LLM calls; no scoring of
non-autonomous turns; no weight training. Outcome-modulated dynamics are
phase 2, gated on phase-1 data.

## 2. Design constraints (from the live architecture)

1. **Reflection stays the only write path.** Outcome judgment runs as a new
   stage inside the existing post-turn reflection call on *user* turns —
   judging *past* fires. The autonomous-turn reflection skip is unchanged.
2. **Zero added LLM calls.** Stage 6 rides the existing `_reflect_and_remember`
   call (same pattern as `capability_gap`, stage 5: an extra payload key
   read straight off the JSON).
3. **Conservatism.** Cap outcomes judged per reflection (`_FIRE_OUTCOME_MAX_PER_REFLECTION = 3`),
   mirroring the one-patch ethos.
4. **Silent fires are first-class.** A "Silent on healthy" concern
   (`intentionally_silent`, `chat_loop.py` ~1259) produces no user-visible
   output; its downstream outcome is *structurally unobservable*, which is a
   different fact than "observed and neutral." The schema must distinguish
   `unobservable` (nothing to react to) from `unobserved` (reaction window
   expired) from `neutral` (observed, no effect).
5. **Ledger-relativity.** An outcome is a property of (act, ledger). One
   fire gets at most two ledger entries: the fired concern's domain
   (`valence`) and the relationship ledger (`user_impact`, ≈ attend_to_user).
   The same act can be +domain/−relationship ("useful but intrusive").

## 3. Data model

### 3.1 Pending registry — `<memory>/pending_fire_outcomes.json`

Atomic write-temp-then-rename (`src/utils/file_utils.py`). One record per
fire awaiting judgment:

```json
{"fire_id": "<uuid4>", "concern_id": "...", "concern_text": "...",
 "fired_at": "<iso>", "exit_reason": "respond|max_iters",
 "reply_digest": "<= 280 chars of the fire's reply>",
 "intentionally_silent": false, "user_turns_since": 0}
```

`fire_id` is minted at dispatch and **also stamped into the react-trace
record and the fire's autonomy.jsonl event** — the join key that lets a
trajectory builder later assemble (trace beats, outcome) for one episode.

### 3.2 Outcome record — appended to `autonomy.jsonl`

```json
{"event": "fire_outcome", "fire_id": "...", "concern_id": "...",
 "concern_text": "...", "exit_reason": "respond",
 "outcome": "helped|neutral|hindered|ignored|unobserved|unobservable",
 "valence": 0.7,          // [-1,1] on the fired concern's ledger; null if un*
 "user_impact": -0.2,     // [-1,1] on the relationship ledger; null if un*
 "evidence": "<= 200 chars, quote or paraphrase of the user evidence>",
 "latency_turns": 1, "observed_at": "<iso>"}
```

Outcome vocabulary:
- `helped` / `neutral` / `hindered` — user-visible evidence existed and was judged.
- `ignored` — the reply was plausibly seen across ≥2 user turns and never
  acknowledged (weak negative prior, distinct from hindered).
- `unobserved` — expiry hit before any evidence; `valence: null`. Consumers
  weight these down, not zero-code them as neutral.
- `unobservable` — intentionally-silent fire; resolved at registration time,
  process fields only.

## 4. Capture flow

1. **Registration** (`chat_loop.py`, beside `_service_agent_concern` at
   ~1279): after reply publication on an autonomous turn, append a pending
   record. If `intentionally_silent`, skip the registry and write the
   `fire_outcome` record immediately with `outcome: unobservable`.
2. **Aging** (each non-autonomous user turn, same place user-concern decay
   runs): increment `user_turns_since` on all pending records.
3. **Judgment** (reflection stage 6): when the pending registry is
   non-empty, `_reflect_and_remember` appends a section to `user_parts`:

   ```
   ## Recent autonomous acts awaiting outcome
   - [fire_id] concern: <text> — Jill did/said: <reply_digest> (N user turns ago)
   ```

   and the return-JSON instruction gains the key `fire_outcomes`. Stage-6
   prompt rule (added to `_REFLECT_SYS`):

   > STAGE 6 — fire outcomes. For each listed autonomous act, judge ONLY
   > from evidence visible in this exchange whether the user's words or
   > behavior show the act helped, was neutral, hindered, or is being
   > ignored. Absence of mention is NOT evidence — omit the entry and it
   > stays pending. Do not infer approval from politeness. valence is for
   > the concern's domain; user_impact is for how the act landed with the
   > user; they may disagree.

   Parsed like `capability_gap`: read off the payload, validated, capped at
   `_FIRE_OUTCOME_MAX_PER_REFLECTION`. Judged records are removed from the
   registry and appended to `autonomy.jsonl`.
4. **Expiry**: pending records with `user_turns_since >= 3` or age > 7 days
   resolve to `outcome: unobserved` and leave the registry. (Both constants
   configurable; expiry runs during step 2.)

Note the natural evidence window: `_build_dialog(source, limit=4)` already
includes recent `auto_say` lines when the fire's reply went to the same
entity, so the stage-6 judgment usually sees both the act and the reaction
verbatim in the dialog it was already given — the pending section mostly
supplies identity (`fire_id`) and reach-back beyond the 4-turn window.

## 5. Consumers

- **Autonomy bench** (closes the README gap): helped/ignored/hindered rates
  per concern and per `rhythm_hours`; triage-verdict ↔ outcome calibration
  (did `defer` verdicts correlate with `ignored` outcomes?); silent-fire
  health from process fields.
- **modular-memory step 6** (trajectory corpus): join `fire_id` →
  react-trace beats; **terminal valence := the captured outcome**
  (environment-grounded — unlike fables, where the oracle supplies the
  ending); the oracle annotates beats/pivotal offline. `unobserved` →
  low loss weight (exactly the ~0-valence fable treatment). One fire yields
  up to two trajectories — same beats, different ledger conditioning
  (concern-domain vs. user-impact) — per the ledger-relativity invariant.

## 6. Tests

- Registration on fire completion; silent path writes `unobservable`
  immediately and never enters the registry.
- Aging + expiry-to-`unobserved`; registry atomicity across restart.
- Stage-6 parse with mocked LLM payload (valid, malformed, over-cap).
- `fire_id` join integrity: trace record ↔ fire event ↔ outcome record.
- Reflection with empty registry adds no section and no key (prompt-stability
  for KV caching).

## 7. Phasing

- **Phase 1 (this spec):** capture only. Behavior identical; only new
  writes are the registry file and `fire_outcome` events.
- **Phase 2 (deferred, needs phase-1 data + bench):** outcome-modulated
  dynamics — e.g. `hindered`/`ignored` history damping a concern's
  effective fire threshold, or informing triage prompts. Explicitly out of
  scope until the bench says the captured signal is trustworthy.
