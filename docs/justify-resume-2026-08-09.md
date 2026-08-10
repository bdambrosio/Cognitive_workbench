# Justify / provenance — resume point, 2026-08-09

State at shutdown. Everything below is committed and pushed to `main`
(`1fa7b0d0` is the last of the run).

## Shipped tonight

**Truncation (the bug that started this).** The attribution call used a
literal `max_tokens=1600` and reached `backend.chat` directly, bypassing
`_make_llm_callable`'s floor table. Long replies overran, `finish=length`,
JSON cut mid-object, whole pass dropped with only a WARNING. Intermittent:
3 of 6 replies over 2600 chars lost their trail. Budget now reads the same
floor table (16384 local / 4096 cloud); one retry at 2×; schema-aware
salvage recovers complete claims and marks the record `incomplete`; a hard
failure writes nothing and returns `ERROR` (not `EMPTY`, which read as
retryable and produced the retry-then-apologise loop).

**Ordering.** `_post_turn_work` ran five stages sequentially on a
single-worker executor with claims 4th — 37-98s to land. Claims now runs
first (~11-20s). Every stage individually isolated. Attribution skipped
when a record already exists for the same `reply_sha1` + `source`.

**Audit by turn number.** Each reply carries its `turn_seq` to the CLI,
shown beside the timestamp (`Jill [20:01:35] #2400`). `justify` takes an
optional `turn_seq`. The number comes from the USER reading it off the
reply — not inferred by the model. Source predicate blocks
cross-conversation reach; `reply_sha1` (written since attribution existed,
never previously read) catches a stale record under a reused seq.

Verified live: `justify 2400` reached past two intervening turns (2401
weather, 2402 an autonomous fire) and audited the right one.

## Two open findings from the last trace

### 1. Re-served answers are mislabelled

Turn 2400 answered the EU AI Act question with tools `['respond']` only —
no search. It reproduced turn 2398's answer from the reasoning-history
block in its prompt, **carrying that answer's inline citations forward**
("Per eur-lex.europa.eu..."). Its own thought: *"citing the sources as
established in the previous turn."*

Attribution graded it 18 `model_prior` + 2 `retrieved` (the latter with
empty refs). Both wrong. `recall_hits` held nothing about the EU AI Act,
so it is not `memory`; no tool ran, so it is not `retrieved`. The correct
grounding is **`context`** — "from the assistant's own prompt — earlier
conversation... refs = []". Empty refs are *correct* for `context`, so the
earlier idea of rejecting "retrieved with empty refs" was wrong and is
dropped.

The mislabel was protective by accident: `model_prior/volatile` graded
three claims `suspect`, which spawned the verification that confirmed the
dates. Correct `context` labelling would have graded all 20 `probable` and
fired nothing.

**The deeper gap:** `context` cannot say *which* turn it came from. Ref
forms are `$stepN` / `Note_N` / `user_input` only. The most useful trail
for a re-served answer — "this is a copy of 2398; the real provenance is
there" — is inexpressible. A `turn:N` ref form would make it chainable,
and Ship 3 just made turn numbers user-visible, so the chain is now
followable end to end.

**Unknown, check first:** whether the attributor mislabels re-served
content consistently or 2400 was a one-off. Sweep the corpus for turns
whose tools were `['respond']` only and look at their grounding profiles
before designing anything.

### 2. "Upgraded to verified" is a naming collision

Turn 2403's justify observation was complete and correct — grounding
profile, `Weakest link: claim 13 (suspect — model_prior, volatile)`, and
the audit note telling her to *"verify it with a tool now before affirming
it"*. She followed it exactly: ran `search-web`, confirmed the dates.
That is the audit machinery working as designed, first time observed.

Then she wrote *"These claims are now upgraded to **verified**"* and
`[model_prior → retrieved | verified]`. Neither "upgraded" nor any
claim-level grade change appears anywhere in the tool output (checked
against `observations_full`, not just the capped working log).

But this is not plain confabulation. The word `verified` **was** in her
observation — it is the top rung in the grounding key's ladder
(`verified > probable > unverified > suspect`). The audit note instructs
her to *verify*. Having verified something, calling it "verified" is the
obvious English move, and it names a formal grade she cannot assign.

**The instruction and the grade share a word, and the instruction causes
the action whose natural description collides with the grade.** This is
why the catalog's "don't explain grades" rule keeps failing — the
collision is in the vocabulary, not the reasoning.

**Proposed:** rename the top grade to something unreachable by performing
a check (`sourced` / `documented`). Cleaner than a fourth prohibition.
Touches `GRADES` in `claims.py`, the grounding key, and
`docs/justification-taxonomy.md`. Needs Bruce's call — it is his taxonomy
vocabulary.

## Also still open

- **URLs never survive into her replies.** The evidence index hands her
  resolvable URLs; she renders bare domains. Third catalog attempt has not
  landed. Worth a different approach than more instruction text.
- **Source tiering** (reliable/middle/low) — deferred pending evidence
  that a search turn ever escalates to fetching a source. Still 0 for 26.
  The audit note now fires reliably, so this is finally measurable: watch
  for a composition line reading `1 read here` instead of `0`.
- `docs/justification-taxonomy.md` specifies `source-grade`
  (`primary | unknown | unreliable`) and nothing emits it, so `verified`
  remains unreachable and every recorded-evidence claim grades `probable`.

## Fixtures

- **2384 / 2386** — positive, 13 claims each, both carry `source` and
  `reply_sha1`.
- **2393 / 2396** — the truncation negative: traces present, no claims
  record. Re-running attribution over them through the local backend now
  returns `finish=stop` with 18 and 20 claims.
- **2400** — the re-served-answer case (tools `['respond']`, 18
  model_prior + 2 bad retrieved).
- **2403** — the audit-note success (`justify` → `search-web` → respond).

Tests: `tests/test_justify.py`, 44 tests. Full suite 295 passing;
`test_klein.py` excluded on a missing `diffusers` import (pre-existing).
