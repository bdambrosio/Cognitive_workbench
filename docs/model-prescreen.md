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

**Shortlist a direct host before a broker.** Check Fireworks' model list first
(`GET https://api.fireworks.ai/inference/v1/models`); a model it serves is
probed there, against `https://api.fireworks.ai/inference/v1`, and OpenRouter is
the fallback for models it does not. Bruce, 2026-09-02, after the OpenRouter
GLM-5.3-Flash route was rate-limited on every day it was used: a broker's 429
names the model and is a shared upstream quota; a direct host's is the
account's own ceiling, which a payment method lifts. The record is in
`measure/models/fw_glm53flash.yaml`.

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

**Route re-verified 2026-09-01.** `baidu/fp8` still accepts the real payload and
is now considerably faster than when it was chosen: 109–149 tok/s over four
calls — 2.5–7.5 s, 270–1,120 completion tokens — against the 69 tok/s recorded
on 2026-08-26. Same model id, same pin, so the route was re-measured rather
than re-chosen. `open-inference/fp8` also accepts, at 25 tok/s.
`digitalocean` is listed by the endpoints API and returns 404 on the real
payload, which is the trap above appearing again under a new provider.

**Gate 2's second categorical rejection — `tencent/hy4-preview`, 2026-09-01.**
Its one endpoint, `tencent/fp8`, passes gate 1 in six of six calls: schema-valid
action, correct tool, `finish_reason: stop`. It fails gate 2 by a wide margin.
Five calls with reasoning on at `effort: low` returned 3,024 / 5,233 / 7,752 /
8,951 / 14,558 completion tokens for the one-line action, at 50–56 tok/s
throughout — 60 to 257 seconds a call. Throughput is not the problem; the model
writes 12,000 to 60,000 characters of reasoning to choose one tool. At this
architecture's 35–187 calls a run that is hours to days of wall clock, and it is
five times the verbosity that retired `Qwen/Qwen3.8-2.4T-A95B`.

**The budget control is inert; only the master toggle works.**
`reasoning: {max_tokens: 256}` returned 13,740 completion tokens — the cap is
accepted and ignored. `reasoning: {enabled: false}` returned **132 tokens in
4.3 s**, still schema-valid. So the model is affordable only with reasoning off
entirely, which is a different operating point from the one the audit runs
(`reasoning_effort: low`, thinking on) and needs its own decision, not a silent
substitution. Compare `measure/models/or_qwen38flash_off.yaml`, where the same
toggle is used deliberately as one arm of a controlled pair.

**Stopped at the pre-screen**, Bruce's decision 2026-09-01. The reasoning-off
arm was not taken to gate 3: nothing currently needs a third hosted candidate,
and a model usable only with thinking disabled answers a different question from
the one the campaign is asking. No temperature was requested and none is
configured, so it cannot run past this point in any case — the gate working as
designed. Re-screen if a non-preview Hy4 ships; a preview route can change
artifact under a pinned id.

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
