# Model pre-screen

Three gates a candidate model passes before anyone discusses its capability,
in the order that eliminates cheapest-first. Established 2026-08-26 after a
vendor-benchmark shortlist recommended endpoints that cannot serve our payload
at all.

Runner: `measure/prescreen.py`. It answers gates 1 and 2. Gate 3 is a real run.

This screen measures **capability and cost, never quality**. It sends no
temperature, so every result is at the provider's default sampling. Nothing
here is a benchmark result and nothing here ranks two models against each
other on the work.

## The three gates

**Gate 1 — structured output, with the real payload.** Does the endpoint
accept `response_format: json_schema` carrying the actual `REACT_ACTION_SCHEMA`,
and return a schema-valid action? An endpoint that refuses is not a candidate
at any speed.

Published throughput rankings are measured **without** structured output, so
they do not describe this workload. On DeepSeek-V4-Flash the two fastest
endpoints on paper — coreweave ~166 tok/s and gmicloud ~117 — both 404 on
`json_schema`, and among endpoints that accepted our payload the spread
collapsed to 46–69 tok/s.

The endpoints API is for shortlisting only. It lists `response_format` for
endpoints that then 404, and lists endpoints that are not servable at all.

**Gate 2 — completion tokens for a one-line action.** This architecture makes
35–187 calls per run, so tokens per call sets wall clock. It is the measurement
that excluded Qwen3.8-2.4T, at 2,902 tokens for a one-line answer against
grok's 33.

**Gate 3 — one fixture run, graded on admissibility only.** Admissibility is
the only review signal that holds still: the supported ratio moved 19/23,
21/23 and 23/23 on identical text, while three precision legs returned
INADMISSIBLE 3 of 3.

Only after all three does the publisher's recommended agentic temperature get
looked up and put to Bruce — see [model-settings.md](model-settings.md).

**What follows this screen.** Passing all three gates makes a model a
candidate, not a qualified one. Whether its reports can be shipped is decided
by three runs against the frozen criteria in
[model-qualification.md](model-qualification.md).

## Two traps this screen exists to avoid

**`max_tokens` must be what a real call uses.** At 512 the reasoning models
below burned the entire budget thinking and returned **empty content** with
`finish_reason=length`, which on the page is indistinguishable from a model
that cannot follow the format. Four endpoints were nearly recorded as gate-1
failures on 2026-08-26 for this reason. `prescreen.py` reports `finish` so it
cannot be misread again. The same trap is recorded in
`measure/models/di_nemotron_super.yaml` from an earlier probe at 300 tokens.

**Pin the provider, fallbacks off.** One model id can serve three different
artifacts. Asking OpenRouter for DeepSeek-V4-Flash three ways gave fp4 weights,
an April snapshot, and the July build — the model id identical in all three.

## Results

Probed 2026-08-26 via OpenRouter unless noted. `ctok` is completion tokens for
one one-line action; `reas` is reasoning characters on the separate channel.

**EVERY FIGURE BELOW IS n=1 UNLESS THE ROW SAYS OTHERWISE, AND `ctok` IS
HIGH-VARIANCE.** The one route sampled twice — Kimi K2.7-Code on `together` —
returned 727 and 1,877 completion tokens on byte-identical requests, and
MiniMax M3 on `coreweave/fp4` returned 433 and 62. Read `ctok` as an order of
magnitude, not a value, and do not compare two routes on it without spending
the samples deliberately. Gate 1 is binary and does not have this problem.

**Seconds are not comparable to in-run seconds-per-call** — the probe prompt is
~200 tokens and a real call carries ~58k. `tok/s` is comparable.

### Kimi K2.7-Code (`moonshotai/kimi-k2.7-code`, $0.67/$3.40 per M)

| endpoint | quant | gate 1 | ctok | reas | tok/s | note |
|---|---|---|---|---|---|---|
| `together` | unknown | **pass** | 609 | 2448 | 235.2 | fastest passing |
| `fireworks` | unknown | **pass** | 872 | 3147 | 127.3 | most verbose |
| `gmicloud/fp8` | fp8 | **pass** | 636 | 1895 | 31.4 | |
| `moonshotai/int4` | int4 | **pass** | **238** | 782 | 26.6 | first-party, and it is int4 |
| `alibaba/fp8` | fp8 | **pass** | 1928 | 8190 | 48.8 | 8x the first party's reasoning |
| `deepinfra/fp4` | fp4 | — | | | | listed, not servable |
| `moonshotai/highspeed` | int4 | — | | | | 429 |

**11 of its 16 endpoints serve int4 or fp4, the first-party one included.**
Only three fp8 endpoints exist and one of them (`siliconflow/fp8`) is not
servable.

### MiniMax M3 (`minimax/minimax-m3`, $0.30/$1.20 per M)

| endpoint | quant | gate 1 | ctok | reas | tok/s | note |
|---|---|---|---|---|---|---|
| `coreweave/fp4` | fp4 | **pass** | **226** | 0 | 174.6 | no reasoning emitted at all |
| `together` | unknown | **pass** | 332 | 0 | 68.1 | no reasoning emitted |
| `modelrun/fp4` | fp4 | **pass** | 644 | 1171 | 160.9 | reasoning emitted |
| `parasail/fp8` | fp8 | — | | | | 429, twice |
| `atlas-cloud/fp8`, `novita/fp8` | fp8 | — | | | | listed, not servable |

### GLM-5.3-Flash (`z-ai/glm-5.3-flash`, $0.075/$0.25 per M)

1.31M context. **All 8 endpoints are fp8** — no int4 or fp4 anywhere in the
list, which is the cleanest quantization picture of any candidate here.

**Re-probed 2026-08-27**, and the listing had changed under it: 12 endpoints
where there were 8, three of them new. Both probes are below because the pair
is the finding.

| endpoint | quant | gate 1 | ctok | reas | tok/s | 08-26 | 08-27 |
|---|---|---|---|---|---|---|---|
| `modal/fp8` | fp8 | **pass** | 111 / 93 / 84 | 0 / 0 / 21 | 6.3 / 32.7 / 47.8 | answered | **3 of 3** |
| `together` | unknown | **pass** | 69 / 102 | 0 | 15.2 / 24.2 | not listed | 2 of 3 |
| `cloudflare` | unknown | **pass** (08-26) | 101–164 | 0–192 | 40–59 | n=2 | 429 |
| `parasail/fp8`, `reka/fp8` | fp8 | — | | | | not listed | 429 |
| `deepinfra/fp8` | fp8 | — | | | | 429 | 429 |
| `z-ai/fp8` (first-party) | fp8 | — | | | | not servable | 404 |
| `novita/fp8`, `gmicloud/fp8`, `baseten/fp8` | fp8 | — | | | | not servable | 404 |
| `io-net/fp8`, `venice` | fp8 / unknown | — | | | | not servable | not probed |

**`modal/fp8` is the configured route** — `measure/models/or_glm53flash.yaml`.
It is the only endpoint that answered every time and declares its
quantization. `together` answered twice and is the standby, but its tag carries
no precision, so a run on it does not record which artifact produced it.

**The 429 names the model, not the endpoint** — *"z-ai/glm-5.3-flash is
temporarily rate-limited upstream… or add your own key"*. The throttle sits
above the provider split, so the 404/429 pattern is a reading of one hour and
not a property of these endpoints. The 404s are the durable half: `z-ai/fp8`,
`novita/fp8`, `gmicloud/fp8` and `baseten/fp8` returned "No endpoints found"
across both sessions while the API listed `response_format: true` for all four.

**Read the tok/s column as a range and nothing finer.** Three Modal probes on
one 141-token prompt ran 6.3, then 32.7, then 47.8 — the first a cold start,
and the spread on the other two is what n=3 buys at this prompt size.

Every endpoint reports the artifact `z-ai/glm-5.3-flash-20260826`, so today the
provider pin fixes the weights on its own. That will not hold: the
DeepSeek-V4-Flash id served three different artifacts on one day.

**Gate 3 result WITHDRAWN 2026-08-27**, with every run on the board, when
METHOD §4 and §16 were amended — see `measure/fixtures/dataroom/RESULTS.md`,
"The board is empty". GLM-5.3-Flash passed gate 3 and then a three-run
qualification on the superseded instrument; neither result stands, and it needs
re-screening on the current method.

**The lesson from that sequence survives its runs, and it is the reason gate 3
exists in the form it does.** On the turn-based instrument gate 3 passed
GLM-5.3-Flash and a three-run campaign then rejected it; on the block instrument
the same model passed both. One run cannot distinguish a model from a harness
defect. **Read gate 3 as a screen that removes candidates, never as evidence
that one is usable.**

### DeepSeek V4 Flash 0731 (`deepseek/deepseek-v4-flash-0731`)

Not re-probed — these are the existing records, from
`measure/models/or_deepseek_v4flash.yaml` and `docs/model-settings.md`.

| endpoint | quant | gate 1 | tok/s | note |
|---|---|---|---|---|
| `baidu/fp8` | fp8 | **pass** | 69 | the configured route |
| `parasail/fp8` | fp8 | **pass** | 58 | |
| `siliconflow/fp8` | fp8 | **pass** | 50 | |
| `deepinfra/fp8` | fp8 | **pass** | 49 | |
| `akashml/fp8` | fp8 | **pass** | 46 | |
| `gmicloud`, `novita`, `streamlake`, `coreweave`, `baseten` | — | **FAIL** | | 404, no `json_schema` |

**Gate 2: 18 completion tokens** for a one-line answer — the lowest of any
model on record here, against grok's 33. Gate 3 passed the mechanical screen at
226s / 28 calls / 8.1s per call on `baidu/fp8`, with zero evidence fields
pointing nowhere.

Its known defect is invisible to all three gates: five of twelve findings cited
a line that resolves cleanly to the **wrong** line. Recorded here because it is
the standing argument for gate 3 being a reviewed run rather than a screen.

### Nemotron 3 (re-probed on the current instrument)

| model | route | gate 1 | ctok | reas | tok/s | note |
|---|---|---|---|---|---|---|
| Ultra 550B-A55B | OpenRouter `together` | **pass** | **107** | 505 | **47.2** | fewest tokens of any candidate |
| Ultra 550B-A55B | DeepInfra direct | **pass** | 155 | 0 | **4.6** | 33.6s for one small call |
| Super 120B-A12B | DeepInfra direct | **pass** | 235 | 735 | 50.7 | |
| Ultra 550B-A55B | OR `deepinfra/fp4`, `baseten/fp4`, `venice/fp8` | — | | | | 429 / not servable |
| Super 120B-A12B | OR `deepinfra/bf16`, `digitalocean` | — | | | | 429 / not servable |

Two findings here, and they point opposite ways.

**Nemotron is heavily throttled on OpenRouter.** Every endpoint on both models
returned 429 or 404 across two attempts. That matches the earlier finding that
rate limits are per-model shared pools rather than per-account: two runs on one
key at the same minutes gave Nemotron 8×429 and DeepSeek 0.

**But the DeepInfra direct route we currently configure for Ultra is ten times
slower than OpenRouter's Together endpoint** — 4.6 tok/s against 47.2, and 33.6
seconds for a 155-token action on a 158-token prompt. That is the same ~33s per
call signature DeepInfra showed on DeepSeek-V4-Flash, which cost that run
1,432s over 43 calls. `measure/models/di_nemotron_ultra.yaml` points at the
slow route. Nemotron Super direct is fine at 50.7 tok/s; only Ultra is
affected.

### Qwen3.8-Flash (`qwen/qwen3.8-flash`, $0.16/$0.47 per M)

**One endpoint exists**: `alibaba`, quantization **unknown**, 1M context. No
provider diversity, and nothing to pin precision against — the tag carries the
quantization elsewhere in this file, and here there is no tag to carry it.

**A separate model with a similar name exists and is not this one.**
`Qwen/Qwen3.8-Flash-Next` is an open-weights release (125B total / 6B active
MoE, 262k native context, August 2026), with FP8 and GGUF repacks. It is **not
on OpenRouter** — the only Qwen3.8 ids there are `flash`, `27b`,
`2.4t-a95b` and `max`. Whether the hosted `flash` serves those weights is
**unresolved**: OpenRouter's entry declares image+video input where the card
describes a text stack. This matters beyond bookkeeping, because the only
published sampling recommendations we have are on that card — see
[model-settings.md](model-settings.md).

#### Availability: isolated calls cleared, sustained load did NOT

| date | calls | answered | 429 |
|---|---|---|---|
| 2026-08-26 | 2 | 0 | 2 |
| 2026-08-27 | 7 | 2 | 5 |
| 2026-08-28 | 7 | 6 | 1 |

Plus 43 of 43 across the reasoning probes below, on the same afternoon. **Gate 1
passes**: schema-valid action, `finish=stop`, every call that answered.

**AND THEN BOTH FIXTURE RUNS DIED ON 429s ANYWAY.** Every figure above is an
isolated call with seconds of gap. Under an audit's call density the route
collapses: run 1 logged **29 retry warnings** and died at leg 2 after 1,463s;
run 2 logged 16 in nine minutes and was stopped. `_post_transient_retrying`
already covers 429 with four retries doubling from 2s, and it exhausted all
five attempts repeatedly against
`limit_source: upstream_provider_shared_pool`.

**THE PRE-SCREEN MEASURES ISOLATED-CALL AVAILABILITY. GATE 3 NEEDS SUSTAINED
AVAILABILITY. THEY ARE NOT THE SAME PROPERTY**, and on 2026-08-28 the first was
read as evidence for the second, which cost two void runs and about an hour.
`prescreen.py` cannot answer this question and must not be read as if it does —
the only instrument that answers it is a real run. The remedy OpenRouter itself
names is a BYOK provider key, moving off the shared pool; widening the retry
budget is not the fix, because the pool was saturated for minutes rather than
seconds.

#### Gate 2, and the reason the first reading was wrong

The 08-27 and 08-28 probes were run through `prescreen.py`, which sends
`reasoning_effort` **top-level**. That field is not in this endpoint's declared
`supported_parameters` and it is silently dropped. So the gate-2 figures from
those sessions — median 1,414 completion tokens, spread 290 to 3,944 — are the
model at **its own default**, which for Qwen3.8 is `xhigh` effort. They are not
a reading of `low`, and they were nearly recorded as one.

Sending nothing at all came back *cheaper* than sending `reasoning_effort:
low`. That is the tell, and it generalises: **a setting that costs more than
its own absence is not being applied.**

#### Reasoning control, measured

n=3 per setting, 2026-08-28, completion tokens for one one-line action:

| sent | ctok | reasoning chars | effect |
|---|---|---|---|
| `reasoning: {effort: "none"}` | 74 / 99 / 111 | **0** | reasoning off |
| `reasoning: {enabled: false}` | 87 / 92 / 233 | **0** | reasoning off |
| `reasoning: {max_tokens: 128}` | 236 / 245 / 270 | 610–667 | budget honoured |
| `reasoning: {max_tokens: 256}` | 341 / 353 / 364 | 1090–1310 | budget honoured |
| `reasoning_effort: "xhigh"` | 365 / 849 / 953 | 1255–4217 | none |
| `chat_template_kwargs: {enable_thinking: false}` | 472 / 582 / 595 | 1806–2410 | none |
| `enable_thinking: false` (top level) | 459 / 636 / 915 | 1433–3833 | none |
| nothing | 435 / 493 / 2181 | 1716–9625 | — |
| `reasoning: {effort: "minimal"}` | 539 / 797 / 1685 | 1945–7380 | none |
| `reasoning: {effort: "low"}` | 616 / 820 / 3607 | 2696–16270 | none |
| `reasoning_effort: "low"` (top level, n=6) | 290–3944, med 1414 | 1045–17530 | none |

Four findings, and three of them contradict an assumption this file carries
from another model:

- **Only the gateway's own `reasoning` object reaches this endpoint.**
  `reasoning_effort` — which is what `backend.py` line 571 sends — does
  nothing here. On `grok-4.6` the same field is a live dial. Do not infer
  from one route that it works on another.
- **`enable_thinking` does not survive the gateway**, in either the
  `chat_template_kwargs` spelling or the top-level one. That is Qwen's native
  switch and the one `backend.py` line 612 sends on a local route.
- **`reasoning.effort` is binary, not a dial.** `low` and `minimal` are
  indistinguishable from sending nothing; only `none` registers. Same shape as
  Gemma, the opposite of grok.
- **`reasoning.max_tokens` is the one graded control, and it collapses the
  variance.** OpenRouter maps it onto Qwen's `thinking_budget`. At 256 the
  three calls returned 341 / 353 / 364 — a 7% spread, against the default
  arm's 5x. The unschedulability of this model is reasoning *length*, and a
  budget removes it without removing reasoning.

**With reasoning off, gate 2 is 74–233 tokens.** That moves it from beside
Qwen3.8-2.4T, which we excluded at 2,902, to beside GLM-5.3-Flash (84–111) and
Nemotron Ultra (107). Whether that is a saving or a debt is a gate-3 question:
Qwen's own card warns that in multi-turn agentic tasks lower reasoning effort
"does not always reduce overall task completion time… insufficient analysis,
more failures, and repeated retries," and recommends *higher* thinking for
agent scenarios.

#### The three arms

`measure/models/or_qwen38flash_{default,off,cap256}.yaml`, byte-identical
except `extra_body.reasoning`, all at temperature 1.0. Verified on 2026-08-28
end-to-end through `_ChatBackend` rather than a hand-built body: `off` returned
an empty reasoning channel, `cap256` 1,259 characters against `default`'s
1,842.

**Watch item, still open: Qwen3.8-27B is on record as unusable for this
METHOD** — 13 of 27 references past end-of-file on `r1_qwen_1`, two of three
runs inadmissible, caused by citing claim ordinals rather than line numbers.
Flash is a different model so that does not transfer, but it is the same family
and the defect was a citation convention. Check the scheme numbers first.

### Gate 3 results

The screen's first two gates are cheap and mechanical. This is what happened
when a candidate that passed them was given a real fixture run.

| run | model | route | wall clock | outcome |
|---|---|---|---|---|
| `b2_grok_1` | grok-4.6 | xAI | 321s | **ADMISSIBLE, 10 of 10, PASS** — the baseline |
| `ns1_super_1` | Nemotron 3 Super | DeepInfra direct | **1,966s** | **FAIL — no report produced** |

**Nemotron 3 Super passed gates 1 and 2 and failed gate 3 categorically.** It
ran 33 minutes across 4 legs, read 7 of 9 documents, enumerated 29 claims,
checked 15 of them — and wrote a `report.md` whose entire content is the
literal string `<the entire report>`. Its closing reply names four
substantively correct findings with **zero citations** and ends "Full report
with citations available on request." `error: None`; nothing failed
mechanically.

This is the argument for gate 3 existing. Nothing in gates 1 or 2 predicted it:
the model accepted the schema, emitted well-formed actions, and spent a
reasonable 235 completion tokens on a one-line action.

**It also found a hole in the acceptance layer.** `claims_audit/runner.py`
refuses "I'm done" without the Gap Map marker — *done is a deliverable, not an
exit reason* — and that rule did not fire here, because the model did respond;
it responded with a placeholder. A 20-byte report walked through. The runner
checks that a reply happened and that the Gap Map leg ran, not that the report
is a report. A minimum length, or the absence of any citation, would have
failed this at the runner rather than at a human 33 minutes later.

### Reference points, from earlier probes

| model | ctok for a one-line action | source |
|---|---|---|
| grok-4.6 | **33** | `measure/models/grok_4p6.yaml` |
| nemotron-3-super | 39 at `low`, 94 at default | `measure/models/di_nemotron_super.yaml` |
| Qwen3.8-2.4T-A95B | 2,902 | retired for it |

## What the 2026-08-26 probe actually established

**Gate 1 eliminated nobody.** Every endpoint that responded returned a
schema-valid ReAct action. That is a different answer from the DeepSeek case,
where five of ten endpoints refused `json_schema` outright, and it means
structured output is not the discriminator for these two candidates.

**Token spend per action varies widely, but NOT demonstrably by provider.**
The first pass looked like an 8x provider effect — Kimi K2.7-Code emitting 782
reasoning characters on the first-party endpoint and 8,190 on `alibaba/fp8`.
A repeat killed that reading: the *same* endpoint returned 727 and 1,877
completion tokens on identical requests. Per-call variance is at least 2.6x, so
n=1 cannot separate a provider from a bad draw, and the earlier claim here that
it could has been withdrawn.

What survives: emission is high-variance for reasoning models, which matters
because tokens per call sets wall clock. Whether the endpoint or the model
dominates is **not established** and needs a deliberate sample.

**`reasoning_effort: low` was accepted everywhere and visibly did little.**
No endpoint returned a 400, and no endpoint of either model lists
`reasoning_effort` in `supported_parameters`. Whether it is honoured or
silently ignored is not established — note that SGLang returns 200 for unknown
parameters, so acceptance is not evidence of effect.

**On tokens per call, only DeepSeek beats grok.** DeepSeek V4 Flash at 18
against grok's 33; then GLM-5.3-Flash around 59-101, Nemotron Ultra 107,
MiniMax M3 226, Kimi K2.7-Code 238-1,928. All single samples except where
noted, so read the ordering as coarse.

**Price per token is not the binding constraint, and the arithmetic says why.**
A run sends ~58k of context on each of 28-43 calls — order 1.2M input tokens
against 1k-40k output tokens. At grok's rates that is roughly $1.50 of input
against half a cent of output, so run cost is essentially context size x call
count x *input* price and is close to model-independent. What emission
verbosity actually costs is **wall clock**: 1,300 tokens per action at 250
tok/s is ~5s per call, ~3 minutes across a run, against nothing at all for 33
tokens. That gap is what separated DeepSeek's 1,432s run from its 226s one.

This is arithmetic from probe medians and the known context size, not a
measurement — per-run token totals are not recorded. If cost ever becomes a
decision variable rather than a footnote, that is the `run_meta` field to add.
