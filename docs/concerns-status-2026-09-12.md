# The agent's own concerns: status and remaining work, 2026-09-12

*Written at the end of the session that read Bähner et al. (Trends in
Cognitive Sciences 2026, the PROBE/COIN rule-creation family) against
`src/chat/concerns.py`. Facts below were read from the code and from
Jill's logs (`scenarios/jill_chat/Jill/memory/*.jsonl`, the resource store)
on this date. For the mechanism itself see
[concerns-architecture.md](concerns-architecture.md); for the shadow-first
sources see the constants block at the top of `concerns.py`.*

## Where this stands

The machinery that lets Jill's own noticing become a concern was built in
stages from 2026-09-07: a population cap of 12 active non-seed concerns
(live), reflection stage 7 candidates that promote after three recurrences in
a week (live since 09-10), stage 8 expectation checks and the fire-side
`CHECK:` line (shadow), and the `mint` action (shadow). Flags:
`_CANDIDATES_LIVE=True`, `_EXPECTATIONS_LIVE=False`, `_MINT_LIVE=False`.

Jill proposed three PROBE-derived changes on 09-12: a per-concern
responsibility value summed from expectation verdicts, a retired tier so a
cap overflow displaces a concern instead of refusing the newcomer, and
recombination of retired concerns. The data changed the diagnosis:

- **The cap was biting for a different reason.** Candidate promotion created
  concerns with no instruction. The fire gate skips those, so four
  near-duplicates of one theme (a memory ranking as a near-miss every week)
  held a third of the working set at activation 1.00 for a 120-day lifetime,
  and generated their own candidates every turn. The three
  `concern_refused_cap` events (09-11 21:09, 09-12 15:58, 09-12 17:27) were
  real tasks refused against inert slots. Jill's reflection abandoned all
  four, and the cap-check concern itself, on 09-12.
- **The similarity merge missed the duplicates legitimately.** Their pairwise
  cosines (BGE-small) are 0.68 to 0.79, under the 0.8 threshold. Candidate
  recurrence had passed because it counts matches across a cloud of
  paraphrase rows. Recurrence is measured against a cloud, the merge against
  one stored sentence; a threshold change does not fix that.
- **The retired tier already exists.** `satisfied` is recallable, kept 120
  days for durables, and revived to `active` when a similar concern is
  created. Only displacement at the cap is missing.
- **The expectation stream cannot carry a value yet.** 225 rows in 4.5 days
  over 14 concerns; 120 of them belong to two concerns. Reflection checks the
  top five by activation only, fire rows exist only for concerns that fire,
  rows are written on change only. And the sign is undefined: the live
  design bumps a concern whose aversive expectation was violated; Jill's
  draft would subtract.

## Landed this session (commit named in the git log for this file)

1. Stage 7 emits `promote` and `instruction`, in Jill's wording. A
   recurring candidate is created only with both; otherwise its
   `would_promote` row says `skipped: no_instruction`.
2. A theme is promoted once per seven days. Each `would_promote` row carries
   the `concern_id` it created or merged into (None when the cap refused).
   A later recurrence is `skipped: already_promoted` with `merged_into`;
   a satisfied concern is revived (`revived: true`); abandoned stays out.
3. A cap refusal event carries `would_displace_last_fired` and
   `would_displace_triage` (id, text, share). Nothing is retired. Rules:
   never-fired concerns first, oldest created; else oldest `last_fired_at`.
   Triage: share of reset and defer verdicts over the last seven days, at
   least three events. Seeds, one-shots and system-spawned work are never
   eligible.
4. `utils.file_utils.read_jsonl`; `matching_candidate_rows` behind
   `candidate_recurrence`.

Replay of the three real refusals: the two rules disagree every time.
Last-fired picks the never-fired near-miss concern; triage picks the
cap-check concern (48 to 57 triage events a week, 0.91 to 0.96 reset or
defer, bump-inflated). Jill reads them as complementary: never-fired first,
else highest triage share, is the candidate rule if displacement is built.

## Remaining work, in dependency order

Every item below touches Jill's concern machinery and is put to her before it
lands (CLAUDE.md, "Changes that affect Jill are put to Jill").

1. **Restart Jill on the landed code**, then read `concern_candidates.jsonl`
   for `skipped` and `concern_id` rows and `autonomy.jsonl` for
   `concern_refused_cap`. Expected: refusals stop; promotions carry
   instructions. If a refusal does occur, the event says what each rule
   would have evicted.
2. **Flip `_EXPECTATIONS_LIVE` and `_MINT_LIVE`** when their shadow rows say
   enough (see the constants block; the mint plan is in session memory).
   Independent of the cap work.
3. **Displacement out of shadow**, only if refusals recur after item 1.
   Jill's condition: the first production displacement is a logged dry run
   before it retires anything. Rule: item 3 above, or the combined rule.
4. **Responsibility value.** Needs, in order: a stated definition of what a
   high and a low value cause for a concern (the sign question); check
   coverage of the whole working set, which is a prompt and cost change; and
   at least one real refusal where both existing rules evict the wrong
   concern. If the value would pick the same victim as last-fired or triage,
   it adds nothing.
5. **Recombination.** Needs item 4, displacement built, and a generation step
   that writes a new concern from two retired ones. That is a new model call
   on the concern path, ruled out by the working agreement unless agreed.
   PROBE's rule space is discrete; ours is prose. Least derivable; parked.

Separate: Note_10003, a yield continuation on a one-hour rhythm, drew 40
triage resets in two days. Yield remainders are exempt from the cap and from
eviction. Jill has taken it as her own to look at.
