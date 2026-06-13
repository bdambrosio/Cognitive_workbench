# autonomy_review

Make Jill's autonomy **visible and judgeable** — the missing piece before
turning the autonomy dial up. Reads a world/agent's `autonomy.jsonl` (written by
the concerns layer at `scenarios/<world>/<agent>/memory/autonomy.jsonl`) and
renders what autonomy actually did: which concerns fired, the triage verdicts
and their reasons, outcomes, and timing.

Read-only with respect to agent state. The only thing it writes is an opt-in
score sidecar (`autonomy_scores.jsonl`, next to the log) — never the resource
manager or the concern notes.

## Usage

```bash
python bench/autonomy_review/review.py                  # summary (default jill_chat / Jill)
python bench/autonomy_review/review.py --list           # chronological digest
python bench/autonomy_review/review.py --concern Note_812
python bench/autonomy_review/review.py --since 2026-06-01
python bench/autonomy_review/review.py --world <world> --agent <agent>
python bench/autonomy_review/review.py --log <path/to/autonomy.jsonl>

# eval loop
python bench/autonomy_review/review.py --score          # rate each fire warranted / unwarranted / maybe
python bench/autonomy_review/review.py --scores         # summarize ratings; lists the concerns to tune away
```

## What it shows

- **summary**: event counts, date span, fires/day, duration stats, terminated /
  react-exit distributions, top concerns by fire count, triage verdict mix.
- **--list**: one line per event (fire / triage / defer) in time order, with the
  fire's response brief and the triage reason.
- **--score / --scores**: a human-in-the-loop eval. `--score` walks unrated fires
  and records a verdict + note to the sidecar; `--scores` rolls them up and
  surfaces the concerns producing *unwarranted* fires — the ones whose rhythm or
  triage prompt needs tuning.

## Next (not built)

An LLM-judge mode (`--auto-score`) that rates each fire's warrant from the
concern text + response, for a first pass before human review. Deferred until the
human-scored set shows the rubric is right.
