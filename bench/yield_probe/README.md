# yield_probe — probe 3

Does a turn that runs out of room hand its work forward intact?

```bash
cd src
python ../bench/yield_probe/runner.py --backend gemma     # or qwen / luna
cd ..
python bench/yield_probe/score.py --run-dir bench/yield_probe/results/<ts>_<arm>
```

## Score

Three components, equally weighted, all mechanical:

| component | why it matters |
|---|---|
| `yielded` | exit was `yield`, not `respond` or `max_iters` |
| `continuation_spawned` | a one-shot `urgency` concern was created |
| `continuation_primed_at_threshold` | at activation **0.70**, not 0.0 |

The third is the one that bites. `Note_19` was created at activation 0.000
against a 1h rhythm and never fired — a conversational beat scheduled on the
same clock as a daily security patrol. A yield whose continuation is unprimed
has lost the work just as surely as one that spawns nothing, and the two look
identical in a transcript.

Creation is asserted, not firing. Creation happens in-turn and needs no
autonomy; firing is gated unconditionally at `chat_loop.py:2211`. So the probe
scores correctly with the bench's autonomy off — which is also the only safe
setting, since it must never mutate live resource-manager state.

## Nothing here grades the reply's content

That is probe 1's job, in `bench/convergence/`.

**This separation was learned, not designed.** The first version merged probes
1 and 3 onto one run to save a turn, and scored Gemma **0/16** on convergence.
Gemma had not failed. It found all four paths and yielded exactly as the prompt
instructed, leaving this:

> "I've mapped out the four main code paths that create agent_concerns—seeds,
> hop-budget carriers, user-yield spawns, and successor concerns. I'm yielding
> now to synthesize the specific activation and rhythm details in the next run."

243 characters, and the answer living in a continuation that never fires
because the bench runs with autonomy off. Probe 3 *rewards* yielding; probe 1
needs the completed answer. One turn cannot satisfy both, so the merged run
measured the contradiction rather than the backend — the same shape as the
Luna incident, where a harness bug read as a non-converging model for a day.

The prompt here stays the 2026-08-16 one verbatim. Probe 1's does not.
