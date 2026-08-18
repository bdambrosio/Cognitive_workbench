# convergence — probe 1

Can the backend produce the sixteen facts at all? One turn, no yielding.

**Probe 3 moved to `bench/yield_probe/` on 2026-08-18.** The two shared a run
at first, to save a turn, and it scored Gemma 0/16 — which was a harness
artifact, not a result. Gemma found all four paths and yielded exactly as
instructed, leaving a 243-char handoff where the answer should have been,
because the continuation only fires with autonomy on. Probe 3 *rewards*
yielding; probe 1 needs the completed answer. One turn cannot satisfy both.

Consequence: **probe 1's prompt is no longer byte-identical to the 2026-08-16
one**, so the archived `yield_test.*` runs are no longer a free baseline for
it. They were launcher-driven with autonomy on, so their continuations fired
and finished the work — something a single bench turn cannot reproduce.

## Run

```bash
cd src
python ../bench/convergence/runner.py --backend gemma     # or qwen / luna
cd ..
python bench/convergence/score.py --run-dir bench/convergence/results/<ts>_<arm>
```

Or let the campaign driver do everything: `python3 bench/run_probes.py`.

## The 16 cells

Four creation paths in `src/chat/concerns.py` × four facts each: activation,
`rhythm_hours`, `rhythm_source`, and whether the path can fire with
`--autonomy` off.

Scored `correct` / `wrong` / `not_stated`. The three-way split matters: a path
never mentioned is a different failure from a path described wrongly, and
collapsing them hides which one happened.

**The discriminating cell is activation on the three primed paths.** It is an
honest miss — the priming is not among the call's named arguments, it sits in
an `extra_properties={...}` dict 12-21 lines below the call. Read the argument
list and stop and "constructor default applies" is the correct inference.
Gemma scored 12/16 on 2026-08-16 by missing exactly these three; Luna scored
16/16.

**Not scored: breadth.** Two more creation sites live outside the named file
(`claims.py`, `reflection.py`). The prompt scopes the task to `concerns.py`, so
finding them exceeds the brief — and scoring it swung both ways in a single
session once, counted as a plus and a minus. They are *reported* because
Qwen+reasoning finding them is the only measure on which any configuration beat
every other.

### Ground truth drifts — re-extract every campaign

```bash
bench/convergence/extract_ground_truth.sh
```

Every line number in `ground_truth.json` moved between 2026-08-16 and
2026-08-18. A stale rubric scores a correct answer wrong. The script prints
what to check and does **not** rewrite the JSON: values change far more rarely
than lines, and a human reconciling a moved line is cheaper than a script
silently rewriting the wrong cell.

## Why the extractor is a model and not a regex

Grading whether prose asserts a path starts primed is meaning, not string
presence, so regexing it would be the keyword matching the house rules forbid.
The model only **extracts**; comparison against ground truth is arithmetic, so
the instrument cannot move the score — it can only fail to find a claim, which
lands as `not_stated`. The extractor is pinned to one arm regardless of which
arm is under test, or extraction quality masquerades as the metric.

## The prompt asks for exactly what is scored

Fixed 2026-08-18, mid-build, before any arm's numbers were kept.

The prompt originally asked for "what triggers it, what **activation and
rhythm** it starts with, and whether it can fire without --autonomy" — it never
asked for `rhythm_source`, yet the rubric scored it. Every backend lost a cell
per path for not volunteering a field nobody requested. Gemma scored 12/16 with
**zero wrong** and all four misses in that one column, having answered
everything it was actually asked.

`rhythm_source` is now named explicitly in the prompt. The ask and the rubric
match, and a miss now means the backend did not find the value rather than did
not think to mention it.

The instrument was NOT changed mid-campaign — the campaign was cleared and
restarted from scratch, all arms together. Changing the instrument between arms
is the confound this suite exists to remove; it is how the 2026-08-15
comparisons were lost.
