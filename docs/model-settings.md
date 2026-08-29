# Model settings

Values live in `src/chat/model_params.py`, which is what executes. This file
is the provenance: where each number came from, and who decided it.

*Re-derived against that module 2026-08-29. What had drifted: one of the twelve
configured models was missing entirely, and five rows named a model id where
the table holds a shorter key. The code's own comments carry provenance this
file did not — which is the wrong way round for a file whose job is provenance.*

Settled 2026-08-24, after twelve benchmark runs were discarded for having been
made at an inherited default nobody chose.

## top_p — global

**0.95, every model, every call site.** Runs, scoring, the grader, chat,
subagents. Not a per-model setting.

There is one exemption and it is a protocol limit rather than a choice: the
Anthropic route in `backend.py` omits `top_p` entirely because Opus 4.7
rejects it. No Anthropic model is currently configured, so nothing exercises
that path today.

`measure/regrade.py` used to pin `top_p=1.0` against the house default, on the
argument that a pinned instrument must not move when an unrelated default
moves. That argument is sound in general and was overruled here: the grader is
not exempt. **The consequence is undischarged — the grader's hand-validation
against the answer key was done at 1.0, so it wants re-validating before its
numbers are trusted.**

## Temperature — per model

| model | temp | source |
|---|---|---|
| `gpt-5.6-luna` | 0.2 | Bruce, 2026-08-24 |
| `gpt-5.6-terra` (grader) | 0.1 | Bruce, 2026-08-24 — kept from the prior hardcoded value |
| `grok-4.6` | 0.5 | Bruce, 2026-08-24. **xAI publishes no recommendation** — checked the model page and the 4.6 docs page; the "0.7/0.95" seen in search results is third-party example code, not xAI guidance |
| `Qwen3.8-27B` (local) | 0.25 | Bruce, 2026-08-24. Key **narrowed** from the bare `Qwen3.8` on 2026-08-26: that spelling also matched `qwen3.8-flash` and `qwen3.8-max`, handing each a temperature nobody chose |
| `gemma-4-31B` | 0.25 | Bruce, 2026-08-24 |
| `DeepSeek-V4-Flash` | 1.0 | publisher, for agentic scenarios |
| `nemotron-3-ultra` | 1.0 | publisher (NVIDIA examples run 1.0 / 0.95) |
| `nemotron-3-super` | 1.0 | Bruce, 2026-08-24, same publisher as Ultra |
| `minimax-m3` | 1.0 | publisher, 2026-08-26 — the MiniMax-M3 card gives 1.0 / top_p 0.95 as general guidance for best performance, not a benchmark config. Confirmed by Bruce. Their top_p equals our global, so nothing per-model is recorded for it |
| `glm-5.3-flash` | 1.0 | Bruce, 2026-08-26. **Z.ai publishes no general recommendation** — the card gives only per-benchmark configs and they disagree (1.0/0.95 HLE-with-tools, 1.0/1.0 NL2Repo and Terminal-Bench, 0.95/1.0 DeepSWE). Chose the standard 1.0 / top_p 0.95 pair |
| `qwen3.8-flash-next` (local) | 1.0 | Qwen, via the Qwen3.8-Flash-Next open-weights card. **The one row whose publisher recommendation applies to the model it names** — these are those weights, at a third party's quantization. 1.0 is the THINKING-mode value and thinking is the mode we run; Qwen publishes 0.7 for non-thinking, so a config that turns thinking off is at the wrong operating point and needs its own conversation |
| `qwen/qwen3.8-flash` | 1.0 | Bruce, 2026-08-28. Publisher value, but for a model that is **probably, not provably, this one**: Qwen publishes 1.0 (thinking) and 0.7 (non-thinking) on the Qwen3.8-Flash-Next open-weights card, and it is not established that the hosted route serves those weights — OpenRouter's entry declares image+video input where the card describes a text stack. No recommendation for the hosted model was found. **One value across both the reasoning-on and reasoning-off arms**, rather than the published pair, so those runs vary reasoning and not two things at once; the off arm therefore runs above its recommended 0.7 |

**The keys above are the literal keys in `MODEL_TEMPERATURE`**, not the model
ids they match. Most are shorter than the id: `nemotron-3-ultra` matches
`nvidia/NVIDIA-Nemotron-3-Ultra-550B-A55B`, `glm-5.3-flash` matches
`z-ai/glm-5.3-flash`. Five rows here named the full id until 2026-08-29, which
would send anyone debugging a match failure looking for a string that is not in
the table.

Matching is exact first, then unique case-insensitive substring. `Qwen3.8-27B`
is a substring key because the local server reports `Qwen/Qwen3.8-27B` while
configs declare `model: ""`. It is spelled out in full, and `qwen3.8-flash`
likewise, because a shared `Qwen3.8` prefix would match every sibling in the
family. A model id matching two keys with **different** temperatures raises
rather than letting match order decide; matching two keys that agree is fine.

## Retired

Named explicitly so a stale config fails with a reason instead of a bare
"unknown model".

| model | why |
|---|---|
| `Qwen/Qwen3.8-2.4T-A95B` | 2,902 completion tokens for a one-line answer against a 6k prompt, where DeepSeek used 18. At ~21 tok/s and 35–187 calls per audit, unusable regardless of parameter count |
| `claude-opus-4-7` | dropped 2026-08-24; revisit at Sonnet 5 |
| `claude-sonnet-4-6` | dropped 2026-08-24; revisit at Sonnet 5 |
| `mimo-v2.5` | dropped 2026-08-24 |

## Adding a model

0. Run the pre-screen first — [model-prescreen.md](model-prescreen.md). Three
   gates: does the endpoint accept our real `json_schema` payload, how many
   completion tokens does it spend on a one-line action, and does one fixture
   run come back admissible. A model that fails those is not worth a
   temperature conversation.
1. Search for the publisher's recommended temperature for **agentic / tool-use**
   work — not the general chat default, which is usually different.
2. If no recommendation exists, say so. Do not pick one.
3. Get Bruce's explicit confirmation.
4. Add the line to `MODEL_TEMPERATURE`, and a row here with its source.

`resolve_temperature` raises for anything not in the table. That is the
enforcement, and it is deliberately fatal: a model absent from the table
cannot run, which forces the conversation rather than producing a
mislabelled result.

## How it is enforced

- `src/chat/model_params.py` holds the values and raises on unknown models.
- `_ChatBackend.chat()` takes `temperature=None` by default and resolves it
  from the model actually answering. **The `0.7` literal is gone**; there is
  no fallback to inherit.
- `_ChatBackend.resolved_model()` reads `/v1/models` for `model: ""` configs,
  so a local run is labelled with the model that answered rather than the
  empty string the config wrote down.
- An explicit `temperature=` argument is still honoured — `run.py
  --temperature` is a real experimental knob — but it is now a decision
  someone made rather than one nobody made.
- `tests/test_model_params.py` — eight tests: the house `top_p`, every arm in
  `measure/models/`, every scenario's model, that an unknown model raises, that
  `model: ""` is an error rather than a wildcard, that each retired model fails
  *with its reason*, that the local Qwen resolves from the served id, and that
  no key is a prefix of another carrying a different value.

## Why this exists at all

Nothing was misconfigured on 2026-08-24. No scenario and no model set a
temperature, so all twelve runs inherited `chat_loop.py`'s 0.7, and
`run.py --help` reported that default as though it were a decision. The three
DeepInfra models are the sharp case: their yaml states the publisher's 1.0,
explains that the runner must pass it, and warns in as many words about "the
silent second variable this exists to avoid".

**Documenting a setting in a file the runner does not read is not setting it.**
