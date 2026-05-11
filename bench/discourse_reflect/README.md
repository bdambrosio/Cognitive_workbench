# bench/discourse_reflect — v0.1

Score the discourse-reflection pass (`DiscourseTracker.analyze_segment`)
on real production updates: given the prior discourse state and the
dialog window the model saw, did the new state correctly update the
prior?

## Pair structure

Each `pairs/pair_NN.json` captures one production discourse-update event:

| Field | What it is |
|---|---|
| `prior_discourse_state` | What the prompt fed into reflection (`previous_discourse_state` slot) |
| `dialog_window` | Up to ~10 user/Jill exchange pairs leading up to the trigger turn (approximates `_build_dialog(limit=20)`) |
| `user_input_at_prior` | The trigger turn's input — most of the new content reflection had to incorporate |
| `production_new_discourse_state` | What reflection actually wrote |
| `gold` | Empty slots for the labeller's reference (see below) |

Pairs are mined from `scenarios/jill_chat/Jill/memory/reasoning_trace.jsonl`.
Autonomous turns are filtered out (reflection skips them, per
`chat_loop.py:5027-5031`). No-op reflections (where prior == new) are
also dropped.

## Sampling

20 pairs stratified across conversation depth (trace indices 1 → 234)
with light bias toward agreement-count-changing updates (~70/30).
Agreement count is approximated by counting `(this segment)` /
`(established earlier)` provenance tags — format-agnostic so it works
across the recent prompt-format change.

## Labelling task (gold)

For each pair, fill in `gold`:

- `should_add_agreements` / `should_add_decisions`: items the dialog
  window genuinely added that should appear in the new state. Free-form
  short paraphrases — wording-equivalence will be matched by judge later.
- `should_modify`: prior items the dialog refined or partially
  superseded; describe both the prior wording and the correct update.
- `should_remove`: prior items the dialog made obsolete OR that fall
  outside the ~30-turn pruning rule.
- `should_carry_over_unchanged`: prior items that should remain in the
  new state verbatim. (Don't enumerate all of them — only ones at risk
  of being silently re-worded or dropped.)
- `labeller_notes`: anything that informed your call but doesn't fit
  the slots above.

## Scoring axes (downstream from gold)

A separate scoring pass compares `production_new_discourse_state`
against `gold` on six axes. v0.1 axes:

- **add-recall**: of `should_add_*`, what fraction appear in production output (semantically)
- **add-precision**: of items in production output that aren't in prior, what fraction are grounded in dialog
- **update-correctness**: did `should_modify` items get the modification right
- **carryover-preservation**: did `should_carry_over_unchanged` items survive intact
- **prune-correctness**: did `should_remove` items get removed
- **wording**: are written items concise, faithful, self-contained

Add-recall + add-precision are first-order. The rest are refinements.

## Re-mining

```
python bench/discourse_reflect/mine_pairs.py \
    --trace scenarios/jill_chat/Jill/memory/reasoning_trace.jsonl \
    --out bench/discourse_reflect/pairs \
    --n 20
```

Re-mining will overwrite gold labels, so freeze pair files (or copy
gold out first) before re-running.
