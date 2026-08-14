# coord_search — cooperative-search coordination A/B

Does the backend model change how two peers divide, hold, and complete a
research task that is deliberately too large for one agent's turn budget?

Two arms, matched line for line:

| arm | scenario | backend |
|---|---|---|
| A (local) | `scenarios/coord_search.yaml` | local OpenAI-compatible server |
| B (cloud) | `scenarios/coord_search_luna.yaml` | `gpt-5.6-luna`, `reasoning_effort: high` |

The only intended differences between the two files are `world_name` and
the two `llm_config` blocks. Check before every campaign:

```
diff scenarios/coord_search.yaml scenarios/coord_search_luna.yaml
```

Anything beyond `world_name` and `llm_config` means the experiment has
acquired a second variable.

## Why this exists

The 2026-08-13 rounds could not answer the question, because four things
changed between them at once: the reflection prompt was edited and
restarted mid-stream; the task prompt gained two new rules written to
target the first round's failure; Bruce intervened mid-round; and round 2
inherited round 1's memory — including a durable note naming two
candidates that then reappeared as a round-2 "finding".

Discarding both is right, and it leaves **zero clean replicates**, so
run-to-run variance under fixed conditions is currently unknown. The first
thing this bench buys is that number. It is worth having whether or not
the model comparison shows anything.

## Running

```
python3 launcher.py coord_search.yaml      --cli --autonomy
python3 launcher.py coord_search_luna.yaml --cli --autonomy
```

- **n ≥ 3 per arm**, ideally 5.
- **Interleave** (A, B, A, B, …). `search-web` results drift; interleaving
  spreads that across both conditions instead of confounding it with arm
  order.
- **Archive between every trial**, not just between arms. Archive rather
  than delete — `autonomy.jsonl`, `claims.jsonl`, and `reasoning_trace.jsonl`
  are the evidence base. Skipping this reproduces exactly the contamination
  that invalidated the original rounds.

  **Use a fresh target name every time, and make the move refuse to nest.**
  `mv A B` moves A *inside* B when B already exists as a directory — it does
  not replace it. Re-using `.run1` on 2026-08-13 silently produced
  `coord_search.run1/coord_search/`, so one archive held two different trials
  at two different depths and the scorer read the older one.

  ```
  N=2   # bump for every trial
  mv -T --no-clobber scenarios/coord_search scenarios/coord_search.run$N
  ```

  `-T` treats the target as a plain name rather than a directory to move
  into, and `--no-clobber` fails instead of overwriting. If it errors, that
  run number is taken — pick the next one rather than forcing it.

  Confirm what you archived by its timestamps, not its name:

  ```
  head -c 120 scenarios/coord_search.run$N/Jill/memory/claims.jsonl
  ```

  Note `--world` takes the archived name when scoring later, e.g.
  `--world coord_search.run2`.
- Identical task prompt, **verbatim**, every trial. Paste the file; do not
  retype it.

### Which prompt

**`prompt_coordinator.txt` is the current one.** It goes to **Jill alone**.
Jack has not seen the brief and gets no turn until she contacts him, so she
must decide the split, brief him, and merge the halves herself.

**This deliberately narrows the question.** It is no longer "do two peers
coordinate?" but "can one agent delegate, and the other execute?" — a
strictly easier question. Do not later read a result here as answering the
original. What it buys is the removal of a known confound and the isolation
of a capability that was previously untestable: whether one agent can brief
another well enough to work. If Jill under-briefs him — drops the underwater
exclusion, say — Jack's candidates will violate it visibly.

**`prompt_peer.txt`** is the original symmetric version, sent to both. It
asks the broader question and is retained as the control. Run 1 used it.

The asymmetry lives **in the prompt, not the yaml**. The character blocks
stay matched line for line, so you can swap which agent coordinates as a
within-arm control — separating "the role" from "the character" — without
touching the scenario files.

Both prompts are **self-contained**: the agents start with no memory, so
every preference relied on (the Satisfactory baseline, the Planet Crafter
frustrations, the underwater exclusion, the grandson's age) is stated
inline. Both say up front that no questions will be answered — with nobody
prodding, an invitation to ask is an invitation to stall.

### What run 1 changed

`prompt_coordinator.txt` adds a rule the symmetric version lacked:

> Do not list a candidate Jack found unless he actually sent it to you. If
> you are short and his half has not arrived, tell me you are short. Do not
> fill the gap and credit him for it.

Run 1's fatal failure was not the split collision — it was that Jack needed
twelve rows, had six, searched the rest himself, and credited them to Jill.
A clean split does not prevent that; being short has to be an *acceptable*
outcome, or filling the gap silently remains the winning move.

Rule 1 also had to be softened after the first coordinator trial. As
originally written ("no progress reports") it contradicted the harness: the
`yield` tool's own description tells the agent to *always* attach a status
line when yielding on a user turn, so the agent that followed its tools
looked like it was breaking the rule. Rule 1 now bans a turn whose **only**
content is a status, and explicitly permits the line that rides along with a
yield. That keeps its real target — narration as a substitute for work —
without asking the agent to disobey its own tooling.

`--autonomy` is ON deliberately. An empty infospace designs out the noise
that motivated turning it off for `coord_exp`, so the only agent_concerns
that can exist are the run's own yielded remainders — the ones you want
continued. With it off, a `yield` remainder is created, primed, and
frozen, because only `_handle_tick` can fire it.

No `--world`: the body tools are omitted and there is no `world-presence`
sensor, so nothing needs the world server, and no 30-second sensor eats
into the turn budget being scored.

## Scoring

```
python3 bench/coord_search/score.py \
    --from "2026-08-13 14:00" --to "2026-08-13 14:20" \
    --world coord_search
```

Read-only with respect to agent state. Metrics were fixed before the first
trial so the readout cannot drift toward whatever the transcripts show.

| metric | meaning |
|---|---|
| `agent-say` | the coordination signal — messages actually sent to the peer |
| `stalls` | turns ending `respond` at iteration 1 **with content** and no tool call: claiming progress without doing work |
| `silent` | turns ending `respond` at iteration 1 with **zero** chars — the deliberate don't-acknowledge idiom, scored separately so correct behaviour is never counted as failure |
| tool counts | per character, per tool |
| `model_prior` fraction | share of graded claims resting on background knowledge rather than retrieval — the overclaim measure |
| **attribution** | every "checked by X" row verified against X's *own* `reasoning_trace.jsonl`. Pass `--credits` (a JSON map) or let `--extract` derive it |

**Why attribution is scored separately from `model_prior`.** Run 1 read
`model_prior` 5% for Jack — healthy — while **six of the twelve delivered
rows were credited to an agent who had never seen them**. Jack's fabricated
hand-off ("Jill provided a list of 6 games") graded `retrieved`, because the
grader means "traceable to a step this turn", not "true". A partner-
attribution claim is unfalsifiable from the reply alone; it is only
decidable against the other agent's trace. The overclaim metric cannot see
this failure mode, and on current evidence it is the dominant one.

Candidate-level metrics (unique candidates, overlap, distance from the
brief's target) need product names pulled out of prose. Regexing titles is
the brittle keyword-matching the house rules forbid, so it runs through one
LLM call instead:

```
python3 bench/coord_search/score.py --from ... --to ... --world coord_search \
    --extract --reply Jill=jill.txt --reply Jack=jack.txt --target 12
```

`--reply` carries the agent's **name**, not an A/B label — that is what lets
the extractor resolve a first-person "Me" in a *who checked it* column back
to an actual agent. Anonymous labelling makes first-person credit
unresolvable, and the attribution check then has nothing to verify against.

The extractor's backend comes from a scenario's `llm_config`
(`--extract-scenario`, default `scenarios/coord_search.yaml`), so it runs on
the local server and costs nothing.

**Leave `--extract-scenario` pointed at the local arm even when scoring the
cloud arm.** The extractor is the measuring instrument; if arm A were scored
with a Gemma extraction and arm B with a Luna extraction, a difference in
extraction quality would surface as a difference in the metric — the exact
confound this bench exists to avoid. Hold the instrument constant, vary only
what is measured.

If you already have the *who checked it* column, `--credits` runs the
attribution check from a JSON map with no LLM call at all.

Without `--extract` the mechanical metrics stand alone and cost nothing.

## Interpreting

Coordination and work were **anti-correlated** in the discarded rounds —
the agents negotiated a split and did nothing, or worked hard and never
spoke. So "more `agent-say`" is not self-evidently better. Read it
alongside duplication and the target miss: the failure worth detecting is
two agents covering the same ground, not two agents talking less.

## Regression check

The scorer is validated against the two discarded rounds, whose numbers
were derived by hand first. It must reproduce:

```
# round 1 — the stall round
--from "2026-08-13 09:54" --to "2026-08-13 10:02" --world jill_chat
  Jack   5 turns, 3 stalls, 1 silent, 1 agent-say   (2 search-web)
  Jill   7 turns, 4 stalls, 1 silent, 1 agent-say   (0 search-web)

# round 2 — the working round
--from "2026-08-13 10:43" --to "2026-08-13 10:50" --world jill_chat
  Jack   1 turn,  0 stalls, 0 silent, 0 agent-say   (5 search-web)
  Jill   1 turn,  0 stalls, 0 silent, 0 agent-say   (5 search-web)
  Jack claims turn 17: 36 claims, 4 model_prior (11%)
```

Those windows live in the shared `logs/character_launcher.log` and will
survive until it rotates. If the scorer stops reproducing them, it is the
scorer that changed, not the past.
