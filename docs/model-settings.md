# Model settings

Values live in `src/chat/model_params.py`, which is what executes. This file
is the provenance: where each number came from, and who decided it.

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
| `Qwen3.8` (local 27B) | 0.25 | Bruce, 2026-08-24 |
| `gemma-4-31B` | 0.25 | Bruce, 2026-08-24 |
| `deepseek-ai/DeepSeek-V4-Flash-0731` | 1.0 | publisher, for agentic scenarios |
| `nvidia/NVIDIA-Nemotron-3-Ultra-550B-A55B` | 1.0 | publisher (NVIDIA examples run 1.0 / 0.95) |
| `nvidia/nemotron-3-super-120b-a12b` | 1.0 | Bruce, 2026-08-24, same publisher as Ultra |

Keys match the served or configured model id: exact first, then unique
case-insensitive substring. `Qwen3.8` is a substring key because the local
server reports `Qwen/Qwen3.8-27B` while models declare `model: ""`. A model id
matching two keys with **different** temperatures raises rather than letting
match order decide.

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
- `tests/test_model_params.py` walks every model and scenario and asserts each
  names a model that resolves.

## Why this exists at all

Nothing was misconfigured on 2026-08-24. No scenario and no model set a
temperature, so all twelve runs inherited `chat_loop.py`'s 0.7, and
`run.py --help` reported that default as though it were a decision. The three
DeepInfra models are the sharp case: their yaml states the publisher's 1.0,
explains that the runner must pass it, and warns in as many words about "the
silent second variable this exists to avoid".

**Documenting a setting in a file the runner does not read is not setting it.**
