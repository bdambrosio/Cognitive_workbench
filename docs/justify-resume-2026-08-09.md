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

## 2026-08-10 update

**Finding 2 is fixed.** Top grade renamed `verified` → `sourced` (see
below). Grades are computed by `grade_claim`, never persisted, so no
migration: `claims.jsonl` stores grounding + tags only. Sites changed —
`GRADES` and the grounding key in `claims.py`, the `justify` tool
description in `tools.py` (the ladder the model reads *before* any
observation arrives — missed in the original triage), the taxonomy doc,
the technical note, `provenance-verifiability.md`, README. The taxonomy's
`quote` dimension also used `verified` as a value; now `matched | absent`,
so the word appears nowhere in the vocabulary. No fourth prohibition was
added, deliberately — the next `justify` → `search-web` turn is a clean
test of whether the rename alone stops the phantom upgrade.

**Finding 1: the sweep is done, and it reframes the problem.** 939 user
turns in `jill_chat/Jill`, 473 tool-free (`respond` only), 28 of those
within attribution's coverage (82 claims records, seqs 2182-2403).
Aggregate grounding on those 28: **107 model_prior**, 10 user_asserted,
7 context, 6 inferred, 3 memory, 2 retrieved.

The 107 cannot be split into "genuine background knowledge" vs "re-served
from an earlier turn" by any sweep — that is a per-claim semantic
judgment. But the structural finding is stronger than a rate: **every one
of the 21 `context` claims in the whole corpus is a self-state or
self-action claim** ("I recommended gedit", "The assistant uses a ReAct
loop", "I'm functioning well"), and not one is content re-served from an
earlier turn. That is not a coincidence — `attribute_claims` builds its
evidence view from ALLOWED REFS, ASSISTANT STATE (`active_concerns`),
user input, recall hits, working log, reply. **It never sees the
conversation history or the reasoning-history block.** So the only
`context` channel it can actually check is the concerns block, and
re-served content has no visible support at all — model_prior is the
honest label *given what the attributor is shown*. Turn 2400 is not a
one-off; it is the only case anyone looked at.

Consequence for the design: this is not a taxonomy gap and not a note
attribute. See "turn_num" below.

## 2026-08-11: finding 1's third channel, and the cheap half shipped

Turn 2447, "do I look like a crow?" — one claim, `model_prior × volatile →
suspect`, the worst grade available. Jill ran `justify`, **disputed her own
trail**, and named the real source with a verbatim quote: the Substrate
block in her system prompt, `running commit 51f86f04; 1 commit(s) since my
last session: world: creature avatars — Bruce a crow, Jill a kitten,
Sentinel an owl`. All three parts of that check out — `git log --oneline
47bb48e8..51f86f04` is exactly that one commit, her session marker recorded
head `51f86f04`, and `world-look` really did only say `Bruce (the human)`,
with no species in it. She was right and the trail was wrong.

Same structural cause as the reasoning-history case, one more channel: the
attributor's evidence view is ALLOWED REFS, ASSISTANT STATE, user input,
recall hits, working log, reply. The Substrate and Embodiment lines live in
the *system* prompt and were invisible, so a claim lifted from them had no
visible support and fell through to model_prior by construction.

It cost a real fire: the suspect grade spawned `Note_6374` fourteen seconds
later, to go verify with web tools whether Bruce looks like a crow — a fact
recorded in this repo's git log and nowhere else. Abandoned by hand through
the live process's `control/concern_manage` queryable.

**Shipped**, because unlike the reasoning-history block this half is nearly
free — two short lines, machine-generated from `git`, not model text:
`_write_react_trace` records `substrate` and `embodiment`, and
`attribute_claims` folds them into ASSISTANT STATE tagged `[harness
provenance]` / `[body]`. No new ref form: `context` already covers the
assistant's own prompt, and the discipline note now says so explicitly for
a commit subject, which is the case most likely to read as something the
model simply knows. Absent on legacy records, which render exactly as
before. Turn 2447 is the regression test.

Still open, and still the expensive half: re-served answers from the
reasoning-history block, where the laundering risk argues for verbatim
quoting against the persisted reply before any `turn:N` ref exists.

## Reviewing the weather turn (2407 / 2408) — two more ships

Turn 2407: `search-web` → `process_text` → `respond`, one weather.gov
source. Turn 2408: `justify 2407`, four `retrieved | probable` claims,
all four quotes verbatim.

Two firsts worth recording. **URLs finally survived into the reply** —
all four claims carried the full resolvable `forecast.weather.gov` link,
after three failed catalog attempts. And `justify 2407` reached past two
intervening autonomous fires (2405 Ramana, 2406 PV). `observations_full`
confirms she read `Grades (sourced > …)`, so the rename is live — but
this trail had no suspect claims and no audit-note verification, so the
collision itself is **still untested**.

**The error, and ship 1.** She wrote "graded as **probable** because they
rely on a model-synthesized summary from a search step." False: mediation
is not a cap, and `grade_claim` returns `probable` flat for every
`retrieved` claim — they would grade the same against a document read
byte-for-byte. A bare grade begs "why?", and the fourth prohibition was
never going to be the answer, so the reducer now states its own ceiling
in the grounding key: no claim can reach `sourced` (source-grade is not
emitted), recorded evidence grades `probable` flat, mediation and missing
quotes are reported but capped nothing. Same move as the rename — remove
the vacuum instead of forbidding it being filled. Revise that text if
source-grade ever ships.

**The structural hole, and ship 2.** Only `search-web` (and `fetch-text`
since `b9f60d17`) wrote `tool_meta`, and `render_justification` derived
mediation from `tool_meta` *presence* — so a step absent from it got no
mediation verdict at all. Corpus: 240 claim-refs to steps with metadata,
183 to steps with none (`justify` 101, `process_text` 26, `recall` 17,
`fetch-text` 15 pre-fix, `world-look` 9, `inspect` 6, `fac-status` 6,
`world-move` 3). Two consequences, one unsafe: a claim quoted against a
`process_text` synthesis carried **no** warning while the same claim
against `search-web` carried one — backwards, since process_text is a
second model pass and the likeliest place for drift. And the evidence
line degraded to "no structured metadata recorded": turn 2346 *read a
paper* with `fetch-text` and its entire evidence section is that one
line, no URL. That, not prompt-catalog failure, is why URLs never
survived — for those tools there was no URL in the trail to carry.

`react.py` now records `{'tool': <name>, 'meta': []}` for every step, so
`observation_mediation` reaches every ref. The render distinguishes a
mediated step that listed sources ("the sources below were listed by that
model") from one that did not ("this tool's own output over what was
already in the turn"), and no longer heads an empty list with a colon.
Turn 2407 was a near miss: `$step2` (process_text) held the reply text
*verbatim* — the most quotable span in the log — and the attributor
happened to cite `$step1`.

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
vocabulary. **FIXED 2026-08-10 as `sourced` — see the update at the top.**

## `turn_num` on notes and collections (Bruce's 2026-08-10 question)

Three separate things wear that one name; they do not travel together.

**Notes: already there, just never rendered.** Memory notes have carried
`source_turn_seq` since memories existed — on the note (`memories.py:98`,
allowlisted at `infospace_resource_manager.py:1023`) and in the
`memories.jsonl` sidecar. `tools/trace_claim.py` already walks it.
`render_justification` did not, so the memory → originating-turn hop
existed in storage and in the offline CLI but not in the trail the agent
reads. **Shipped:** the `Note_N` evidence line now names the turn and
invites `justify N`.

Guarded, because the number is not always live. Seqs restarted at 1 each
session until seeding landed, so an old memory's seq now points at an
unrelated recent turn — live case `Note_4191`, written 2026-07-14 "from
turn 14", whose seq-14 record is a 2026-07-17 turn. The pointer is offered
only when the resolved turn's date matches the memory's write date
(reflection writes the memory seconds after the reply); otherwise the line
says the number belongs to an earlier session's numbering and must not be
justified. A midnight-boundary miss degrades to "unresolvable", which is
the safe direction. Same hazard `trace_claim.py:62` already warns about —
this is the first place it could have bitten a user.

**Collections: no, and not yet.** Nothing can cite a Collection —
`valid_refs_for` admits `$stepN`, `Note_N`, `user_input` and nothing else
— so a turn number on one resolves no trail today. It would also be close
to meaningless: `memories`, `concerns` and `threads` are long-lived and
mutated across thousands of turns, and a single creation turn says nothing
about the content a claim actually cited. Revisit only if a ref form ever
points at a Collection.

**The `context` case needs a ref form, not an attribute.** The content a
re-served answer copies lives in `reasoning_trace.jsonl`, rendered into
the prompt as `### trace #N` — it is not in a note or a collection, so no
attribute on either can resolve it. What is missing is a `turn:N` ref
form, and the enabling data is already recorded per turn:
`prefix_trace_refs` is the exact list of turn_seqs visible to that turn,
so `valid_refs_for` can derive the allowed `turn:N` refs with **no new
persistence**. Two changes: give the attributor a one-line digest of the
prefix turns (it currently sees none) with `turn:N` in ALLOWED REFS, and
resolve `turn:N` in the render to that turn's reply plus a "justify N for
its own trail" pointer — the same one-hop-at-a-time shape just shipped for
notes, not a recursive render.

**One argument for going slow, and it is the reason this is not built
yet.** model_prior is the conservative label: it grades `suspect` when
volatile and fires the verification note. `context` grades `probable` and
fires nothing. Handing the attributor prior-turn text trades mislabels in
the *unsafe* direction — background knowledge relabelled "context from
turn N" is laundered to probable and silenced. Turn 2400 is the proof: the
mislabel is what caught the dates. So `turn:N` should carry the same
verbatim discipline as `$stepN` — a quote machine-checked against turn N's
persisted reply, and no `turn:N` ref without one. That is what makes the
chain honest rather than merely tidier, and it is the piece to design
before any of it ships.

## Also still open

- ~~**URLs never survive into her replies.**~~ Landed at turn 2408 — four
  full resolvable links, one per claim. n=1 and a single-source trail, so
  the pairing rule ("do not pair claims to individual sources unless the
  trail records the pairing") is still untested; that only bites on a
  multi-source step.
- **Source tiering** (reliable/middle/low) — deferred pending evidence
  that a search turn ever escalates to fetching a source. Now 0 for 27:
  turn 2407's composition line read `0 read here, 1 named … not opened:
  weather.gov`, and she reported the audit note's descriptive half while
  dropping its instruction to fetch. Defensible for a weather forecast,
  but she did not say she had judged it not load-bearing.
- `docs/justification-taxonomy.md` specifies `source-grade`
  (`primary | unknown | unreliable`) and nothing emits it, so `sourced`
  remains unreachable and every recorded-evidence claim grades `probable`.
  The grounding key now says this outright rather than leaving the model
  to guess at it.

## Fixtures

- **2384 / 2386** — positive, 13 claims each, both carry `source` and
  `reply_sha1`.
- **2393 / 2396** — the truncation negative: traces present, no claims
  record. Re-running attribution over them through the local backend now
  returns `finish=stop` with 18 and 20 claims.
- **2400** — the re-served-answer case (tools `['respond']`, 18
  model_prior + 2 bad retrieved).
- **2403** — the audit-note success (`justify` → `search-web` → respond).
- **2407 / 2408** — the weather turn: URLs carried into the reply, and the
  invented grade rationale. `$step2` is the process_text near-miss.
- **2346** — a `fetch-text` read whose whole evidence section is "no
  structured metadata recorded" (pre-`b9f60d17`); **2402 / 2406** are the
  same tool recording its URL properly after the fix.

Tests: `tests/test_justify.py`, 47 tests. Full suite 297 passing;
`test_klein.py` excluded on a missing `diffusers` import (pre-existing),
and `test_concern_dynamics.py::test_add_user_concern_stores_context_and_description`
fails on CUDA OOM whenever a live chat session holds the GPU (verified
pre-existing by stashing).
