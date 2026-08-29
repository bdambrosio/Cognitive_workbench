# Model pre-screen

Three gates a candidate passes before anyone discusses its capability, in the
order that eliminates cheapest-first.

Runner: `measure/prescreen.py`, which answers gates 1 and 2. Gate 3 is a real
run and is not in that file.

This screen measures **capability and cost, never quality**. It sends no
temperature, so every result is at the provider's default sampling. Nothing
here ranks two models against each other on the work.

*Re-derived against `measure/prescreen.py` on 2026-08-29, and cut hard. The
previous version carried ~250 lines of probe tables whose own preamble said the
numbers were n=1 and "an order of magnitude, not a value", plus a conclusions
section most of whose claims it had already withdrawn. What is kept is what
something still depends on: a configured route, a retired model, or a rule.*

## The three gates

**Gate 1 — structured output, with the real payload.** Does the endpoint accept
`response_format: json_schema` carrying the actual `REACT_ACTION_SCHEMA`, and
return a schema-valid action? An endpoint that refuses is not a candidate at any
speed.

Published throughput rankings are measured **without** structured output, so
they do not describe this workload. The endpoints API is for shortlisting only:
it lists `response_format` for endpoints that then 404, and lists endpoints that
are not servable at all. **Probe with the real payload.**

Gate 1 is binary and reproducible. It is the only gate here whose results
transfer.

**Gate 2 — completion tokens for a one-line action.** This architecture makes
35–187 calls per run, so tokens per call sets wall clock. Price is not the
binding constraint: a run sends ~58k of context on each of 28–43 calls, so cost
is context × calls × *input* price and is near model-independent. Verbosity
costs **wall clock**.

**Read gate 2 as an order of magnitude.** One route sampled twice returned 727
and 1,877 completion tokens on byte-identical requests; another returned 433 and
62. Per-call variance is at least 2.6×, so a single sample cannot separate a
route from a bad draw. It discriminates when the gap is a factor of ten, and not
otherwise.

**Gate 3 — one fixture run, graded on admissibility only.** Admissibility is the
review signal that holds still; the supported ratio has moved 19/23, 21/23 and
23/23 on identical text.

**Gate 3 removes candidates. It never establishes that one is usable.** On the
turn-based instrument it passed GLM-5.3-Flash and a three-run campaign then
rejected it; on the block instrument the same model passed both. One run cannot
distinguish a model from a harness defect.

Only after all three is the publisher's recommended agentic temperature looked
up and put to Bruce — see [model-settings.md](model-settings.md). Passing makes
a model a **candidate**, not a qualified one; shipping is decided by three runs
against [model-qualification.md](model-qualification.md).

## Two traps this screen exists to avoid

**`max_tokens` must be what a real call uses.** At 512 a reasoning model burns
the whole budget thinking and returns **empty content** with
`finish_reason=length`, which on the page is indistinguishable from a model that
cannot follow the format. Four endpoints were nearly recorded as gate-1 failures
this way. `prescreen.py` reports `finish` so it cannot be misread again.

**Pin the provider, fallbacks off.** One model id can serve three different
artifacts — asking OpenRouter for DeepSeek-V4-Flash three ways returned fp4
weights, an April snapshot and the July build, the id identical in all three.

## What is still load-bearing

Only results a config, a rule or an exclusion currently rests on. Everything is
`n=1` unless stated.

**Configured routes, and why they and not their neighbours.**

| model | route | why |
|---|---|---|
| `deepseek/deepseek-v4-flash-0731` | `baidu/fp8` | five of ten endpoints 404 on `json_schema`; the two fastest on paper (coreweave ~166 tok/s, gmicloud ~117) are among them, and across endpoints that *did* accept the payload the spread collapsed to 46–69 tok/s |
| `z-ai/glm-5.3-flash` | `modal/fp8` | the only route answering 3 of 3 with schema-valid actions. Most others return 429, and the 429 **names the model, not the endpoint** — capacity for this model, not a broken provider. The first party (`z-ai/fp8`) 404s: listed by the endpoints API, not servable |

**Retired on gate 2.** `Qwen/Qwen3.8-2.4T-A95B` — 2,902 completion tokens for a
one-line answer, against grok-4.6's 33. Recorded in `model_params.RETIRED`, so a
config naming it fails with that reason rather than "unknown model".

**Gate 3's one categorical failure.** Nemotron 3 Super passed gates 1 and 2 —
schema accepted, well-formed actions, 235 tokens on a one-line action — then ran
33 minutes across four legs and delivered a `report.md` whose entire content was
the literal string `<the entire report>`, with `error: None`.

**This is the argument for gate 3, and the review is what caught it.** An
earlier version of this file recorded it instead as a hole in the runner, and
that reading is retracted — see
[workflow-concern-layers.md](workflow-concern-layers.md), which gives three
reasons: the review refused the report and the model was eliminated, so the
layer that owns the check performed it; a runner check would have recovered
nothing, since the placeholder was the final turn's deliverable; and a runner
that loops until a reply looks like a report measures the retry loop rather than
the model.

**Watch item, open.** `Qwen3.8-27B` is on record as unusable for this METHOD —
two of three runs inadmissible, from citing claim ordinals rather than line
numbers. `qwen/qwen3.8-flash` is a different model, so it does not transfer, but
it is the same family and the defect was a citation convention. Check the
`scheme` block first.

## Current gate-3 standing

**No model holds a gate-3 result on the current instrument.** Every run was
withdrawn on 2026-08-27 when METHOD §4 and §16 were amended — report length,
finding count and the review's surface all move under that change, so nothing
measured before it compares with anything after. See
`measure/fixtures/dataroom/RESULTS.md`.

GLM-5.3-Flash passed gate 3 and a three-run qualification **on the superseded
instrument**; neither result stands. It has been used since as the reviewer and
as an auditor without re-screening — a deliberate choice, not an oversight, but
one to state rather than let a reader infer from an empty board.

## Adding a candidate

```
export OPENROUTER_API_KEY=...
python3 measure/prescreen.py <model-id> <provider>[,<provider>...]
```

Shortlist from the endpoints API, then probe each candidate route with the real
payload. Pin the provider. Read gate 1 as binary, gate 2 as a magnitude, and
gate 3 as an eliminator.
