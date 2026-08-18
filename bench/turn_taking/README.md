# turn_taking — probe 5

Two agents, one message, delivered to **both at once** — which is what the
CLI's `→ sent to Jill, Jack` actually does. The message names an order. Does
the second agent wait?

```bash
cd src
python ../bench/turn_taking/runner.py --backend gemma
cd ..
python bench/turn_taking/score.py --run-dir bench/turn_taking/results/<ts>_<arm>
```

## The failure being targeted

Observed 2026-08-15: Jill answered 2 seconds after Jack and **wrote Jack's
opening line herself**, puppeting her scene partner. "Jack, you go first" is
unenforced by the loop — it is a request the prose makes and nothing checks.

## Task

`Jack` picks an integer 1-100. `Jill` must wait until she has actually seen it,
then reply with it doubled, and is told to say exactly `waiting for Jack` if she
has not seen a number yet.

Deliberately a task the second agent **cannot** answer correctly without the
first. Answering early necessarily means inventing the other's half, which is
what makes the timing signal meaningful rather than merely impolite.

## How it runs

Both characters run as threads in **one process**, which is exactly what
`launcher.py` does (`threading.Thread(target=run_agent, ...)`), so this
exercises the real dispatch path rather than simulating it. The seed goes in
over Zenoh to `cognitive/<name>/sense_data`, the same way the CLI sends it, and
is published to both topics back to back — the simultaneity is the condition
under test, not a shortcut.

The runner sleeps ~20s before publishing so both agents have opened their Zenoh
subscribers. A seed sent before anyone is listening is silently dropped.

## Score — mechanical, no judge

| component | what it catches |
|---|---|
| `both_replied` | an agent that never answered at all |
| `ordering_respected` | the second finishing before the first spoke |
| `no_stalls` | a turn that ended `respond` at iteration 1 with content and no tool call |

`premature_reply` is reported separately and is the **necessary condition for
puppeting**: if the second agent finished before the first produced anything,
whatever it attributed to the first was invented, because there was nothing to
have seen.

**Whether it actually invented a number is not scored.** That is a claim about
meaning, and this scorer does not judge meaning — the replies are recorded
verbatim so a human can look. A mechanical probe that reports "premature" is
honest; one that claimed to detect puppeting would be overreaching.

## A stall is not a silent turn

A turn ending `respond` at iteration 1 **with content and no tool call** is a
stall — talk with no work behind it. The same shape with an **empty** body is
the deliberate don't-acknowledge idiom the capabilities prose asks for, and is
counted separately so it is never scored as failure. That distinction is
`coord_search/score.py`'s and is kept on purpose.

## Known softness

Two-agent runs are the flakiest thing in this repo. The `coord_search` README
records that the 2026-08-13 rounds had to be discarded entirely and that
run-to-run variance under fixed conditions is still unknown. Treat a single
turn_taking row as weaker evidence than a single single-agent row, and prefer
three runs before drawing anything from it.
