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
**Seconds are not comparable to in-run seconds-per-call** — the probe prompt is
~200 tokens and a real call carries ~58k. `tok/s` and `ctok` are comparable.

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

**Provider choice changes token spend by up to 8x on identical payloads.** Kimi
K2.7-Code emitted 782 reasoning characters on the first-party endpoint and
8,190 on `alibaba/fp8` — same model id, same request. MiniMax M3 emitted no
reasoning at all on two endpoints and 1,171 characters on a third. Since tokens
per call is what sets wall clock here, **the endpoint is a bigger variable than
the model** and must be pinned before any comparison means anything.

**`reasoning_effort: low` was accepted everywhere and visibly did little.**
No endpoint returned a 400, and no endpoint of either model lists
`reasoning_effort` in `supported_parameters`. Whether it is honoured or
silently ignored is not established — note that SGLang returns 200 for unknown
parameters, so acceptance is not evidence of effect.

**On tokens per call, none of these beats grok's 33.** The best candidate
figures are Nemotron Ultra at 107 and MiniMax M3 at 226. Price per token is
roughly 1/5 of grok's, which on a run costing a few dollars against a $5K
engagement is not the binding constraint.
