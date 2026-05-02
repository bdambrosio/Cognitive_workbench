# Cognitive Workbench

A research framework for studying LLM-based cognitive architectures, focused on chat-mode agents with persistent state, ReAct tool use, durable cross-session memory, and scheduled autonomy. Prioritizes **inspectable agent behavior** and fast iteration over stability.

[![Status: Research Laboratory](https://img.shields.io/badge/status-research_laboratory-purple.svg)]()
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## What This Is

The chat subproject (`src/chat/`) is the active and primary surface. Each user turn drives a reflective ReAct loop with a small fixed tool set, persistent concerns, durable cross-session memory, and rolling reflection products (companion model, discourse state, per-turn orientation) surfaced into Jill's prompt prefix. A 30-minute `tick` sensor runs autonomous ReAct loops on due concerns when their cadence elapses.

```
                        user input
                            │
                ┌───────────▼──────────────────┐
                │  Pre-loop reflection         │
                │   • orientation evaluator    │  per-turn posture +
                │     (concerns + companion +  │  epistemic constraint +
                │      persona + self-model)   │  concern-surfacing
                └───────────┬──────────────────┘
                            │
                ┌───────────▼──────────────────┐
                │  ReAct loop  (≤ 8 iters)     │
                │   ┌──────────────────┐       │   tools:
                │   │  thought + tool  │       │   • process_text
                │   │        ↓         │       │   • search
                │   │  observation     │       │   • fetch_text
                │   └────┬─────────┬───┘       │   • remember (subagent)
                │        │         │           │   • respond (exits)
                │        │         │           │
                │        └─────────┘           │
                └───────────┬──────────────────┘
                            │ respond
                ┌───────────▼──────────────────┐
                │  Post-turn (background)      │
                │   • reflection (memories +   │
                │     derived concerns)        │
                │   • discourse update         │
                │   • companion update         │
                │   • reasoning-history write  │
                └───────────┬──────────────────┘
                            │
                          reply

   Parallel:  `tick` sensor (30 min)
                            │
                ┌───────────▼──────────────────┐
                │  _check_and_fire_concerns    │  due concerns →
                │   (cadence elapsed)          │  autonomous ReAct
                │                              │  with concern.instruction
                └──────────────────────────────┘  as input
```

## Key Features

- **ReAct tool use** — `process_text`, web `search`, `fetch_text` (full-page extraction), `remember` (active-recall subagent over the agent's memory substrate), `respond`. Each emission is a single JSON object that MUST include a `thought` field (one terse sentence supporting the action choice); the thought persists into the reasoning-history feed for future turns. Per-iteration trace is written before any post-turn LLM work, plus a live CLI status line ("thinking…" → "using search…") that overwrites in place. Prompt construction is store-and-append: the system prompt and user-message prefix are built once at loop entry and reused verbatim across iterations; only the working log grows by literal string append, so the prefix is byte-stable and the backend's KV cache hits on iter 2+. Section headings carry brief mechanism-tags in parentheses (e.g. `## Active concerns (from YAML seeds + post-turn reflection + semantic recall)`) so the model can read source provenance directly from the prefix without confabulating origins.
- **Reasoning history (awareness feed)** — after each ReAct loop (user-driven or autonomous), the per-iteration thoughts, actions, and observations persist as a Note in a rolling `reasoning_history` collection. The most recent few traces surface in the user-message prefix on subsequent turns — making the agent's own prior thinking a structural input to current reasoning. Recent traces render in full; older ones render as a compressed action-sequence digest. Ring-bounded on disk.
- **Long-term memory** — per-character `memories` collection with categorized recall (`fact` / `preference` / `commitment`), auto-RAG injection at turn start, and post-turn reflection that suppresses writes from hypothetical / roleplay / counterfactual frames. Discourse update + reflection run in a background single-worker executor so the response publishes without waiting on slow LLM-bound side effects.
- **Concerns** — actionable directives stored in a per-character `concerns` collection separate from memories. Three categories (`one_shot` / `durable` / `derived`) with independent per-concern firing parameters generated by reflection: `cadence_hours` from a discrete allowlist `{1, 2, 4, 8, 12, 24, 168}` (firing rhythm), `lifetime_days` (decay tau), `instruction` (the action to take). Two timestamps anchor the lifecycle: `last_engaged_at` is updated only by user engagement and drives decay; `last_acted_at` is the cadence anchor, updated when the concern is acted on. Surfaced in the system prompt as an "Active concerns" block with a per-concern operational status sub-line (`runnable every 24h, due now` / `idle Xh` / `standing directive, no firing rhythm`). Recurrence-detection at write time promotes `one_shot → durable` on re-emission and revives satisfied concerns.
- **Sensors framework + Phase C autonomy** — first chat-mode sensor is `tick`, a stateless 30 min heartbeat that publishes to `sense_data`. Tick events route to `_check_and_fire_concerns` (cadence elapsed since `last_acted_at`) and autonomously execute due concerns' instructions through the standard ReAct loop (cap 2 per tick; the rest stay due for the next tick). A CLI preamble announces each fire. Autonomous turns reuse the full prompt construction (voice and trace format identical to user turns) but skip post-turn reflection and discourse update and don't refresh user-engagement timestamps.
- **Companion Model + Discourse tracking** — single-user fair-witness texture (CURRENT CHAPTER, STATE OF MIND, WHAT MATTERS, HOW THEY THINK & WORK, ON THEIR MIND, HOW TO BE USEFUL) and outstanding-discourse state (ACTIVE COMMITMENTS, CURRENT AGREEMENTS, KEY DECISIONS), persisted across sessions.
- **Memory substrate + active-recall subagent** — per-world per-agent `memory/` directory holds the agent's queryable substrate as plain files: appended `chat_trace.txt` (per-turn LLM I/O), appended `conversation.txt` (verbatim dialogue), current-value `companion_state_<entity>.txt` and `discourse_state_<entity>.txt` (overwritten on each update). The `remember` tool delegates to a thin persona-less ReAct subagent (`src/chat/remember.py`) with read-only fs primitives (`list`, `read`, `grep`, `respond`) scoped to the memory dir; from the parent's vantage it's a single ReAct step with the synthesized answer bound to `$stepN`. Per-call subagent traces land in a sibling `subagent_traces/` directory for debugging. Subagent prompt is stable across calls so caches well on the anthropic route.
- **Persona / Self-model split + reflection backdrop** — character config separates `character:` (voice and stance) from `self_model:` (architectural account: what the agent is, its access boundary, and the roster of inbound reflection products it did not produce and cannot inspect). Reflection processes (discourse tracker, companion model, per-turn orientation evaluator) generate against persona + self-model as backdrop via `NARRATOR STANCE` + `AGENT ARCHITECTURE` prepends, so their artifacts don't drift into characterizations the agent would have to disclaim and posture decisions can anchor in boundary-language when probes target inaccessible substrate.
- **Per-turn orientation evaluator** — single LLM pass that reads the active concerns, companion-model state, recent context, persona, and self-model, and emits a posture line (LLM-authored, not from a deterministic mapping), epistemic constraint flags, and per-concern relevance. Includes opportunistic concern-surfacing guidance: when an active concern is `due now` AND the current exchange offers a natural opening, the posture surfaces it as a brief mention.
- **Unified cloud-LLM config** — `api_key` field naming an env var triggers Bearer-auth POST to any OpenAI-compatible endpoint (MIMO, OpenRouter, OpenAI, hosted vLLM, …); legacy server shortcuts still work. A dedicated `server: anthropic` route hits Anthropic's native Messages API (`/v1/messages`, `x-api-key` + `anthropic-version` headers, system as a top-level field).

## Quick Start

### 1. Install

```bash
git clone https://github.com/bdambrosio/Cognitive_workbench.git
cd Cognitive_workbench
python3 -m venv zenoh_venv
source zenoh_venv/bin/activate
pip install -r requirements.txt
```

### 2. Configure an LLM backend

Pick a scenario based on the backend you want:

- **Local vLLM**: edit `scenarios/jill-chat-vllm.yaml`, set the model path, and start a vLLM server. The scenario points at it via `base_url`.
- **Anthropic Claude (Sonnet 4.6)**: `scenarios/jill-chat-sonnet.yaml`. Set `CLAUDE_API_KEY` in your environment.
  ```bash
  export CLAUDE_API_KEY="sk-ant-..."
  ```
- **MIMO cloud** (or any OpenAI-compatible endpoint): `scenarios/jill-chat-mimo.yaml` is the template. Set the env var named in `api_key`.
- **Local llama.cpp / SGLang / generic OpenAI-compatible**: `scenarios/jill-chat.yaml`. Default points at `http://127.0.0.1:5000`; edit `base_url` for your local server.

### 3. Run

```bash
source zenoh_venv/bin/activate
cd src
python3 launcher.py ../scenarios/jill-chat-sonnet.yaml --cli --resource-browser
```

Flags:
- `--cli` — interactive CLI chat
- `--resource-browser` — opens the Resource Browser web UI on port 3001 for inspecting Notes, Collections, and Concerns

### 4. (Optional) Browser automation

The `browse` tool requires the [agent-browser](https://github.com/vercel-labs/agent-browser) CLI:

```bash
cargo install agent-browser
# or download a prebuilt binary from https://github.com/vercel-labs/agent-browser/releases
```

Skip if you don't need browser automation — search and fetch_text work without it.

## Resource Browser

Optional web UI on port 3001. Two-panel layout with a resource list and content viewer. Tabs:
- **Notes** — durable specifics extracted by post-turn reflection (facts / preferences / commitments).
- **Collections** — `memories`, `concerns`, `reasoning_history`, plus state Notes (companion/discourse).
- **Concerns** — categorized by `one_shot` / `durable` / `derived`, with status, weight, cadence, and direct abandon/revive controls.

Used to inspect what the agent is carrying across turns and to manually prune/edit state.

## Available Scenarios

| Scenario | Backend | Notes |
|----------|---------|-------|
| `jill-chat.yaml` | OpenAI-compatible local server | Default; `base_url` defaults to `http://127.0.0.1:5000` |
| `jill-chat-vllm.yaml` | vLLM (local GPU) | Set `vllm_model_path` |
| `jill-chat-mimo.yaml` | MIMO cloud (unified `api_key` form) | Generic OpenAI-compat cloud route |
| `jill-chat-sonnet.yaml` | Anthropic Claude Sonnet 4.6 | Native Messages API |
| `jill-benchmark-chat.yaml` | Local OpenAI-compat | Benchmark harness — frozen scripted session |
| `jill-benchmark-chat-sonnet.yaml` | Anthropic Claude Sonnet 4.6 | Benchmark harness, cloud variant |

## Repository Structure

```
Cognitive_workbench/
├── README.md                          # This file
├── BACKGROUND.md                      # Research philosophy
├── requirements.txt                   # Python dependencies
├── docs/                              # Detailed documentation
├── scenarios/                         # Scenario YAML files + chat-mode runtime data
└── src/
    ├── launcher.py                    # Entry point — dispatches by scenario `mode`
    │                                  # (chat is the only supported mode)
    ├── chat/
    │   ├── chat_loop.py               # ReAct loop, status line, memories + concerns
    │   │                              # collections, frame-aware reflection,
    │   │                              # recurrence promotion, concern firing,
    │   │                              # fetch_text, unified cloud LLM,
    │   │                              # background post-turn executor,
    │   │                              # memory-substrate writers
    │   └── remember.py                # Active-recall subagent — thin persona-less
    │                                  # ReAct loop with fs primitives (list/read/
    │                                  # grep/respond) scoped to the memory dir
    ├── discourse.py                   # DiscourseTracker (analyze_segment +
    │                                  # update_companion_from_discourse_segment)
    │                                  # with persona + self-model backdrop
    ├── character_evaluator.py         # Per-turn orientation evaluator
    ├── infospace_resource_manager.py  # Notes / Collections / Relations + FAISS
    ├── conversation_store.py          # Dialog lifecycle, archival, session backfill
    ├── resource_browser.py            # Resource Browser UI (port 3001)
    ├── sensor_runner.py               # Sensor scheduling and execution
    ├── sensors/
    │   └── tick/                      # 30-min heartbeat → Phase C autonomy
    ├── tools/                         # Core tools (search-web, fetch_text, …)
    └── utils/                         # Shared utilities

Per-world per-agent memory substrate (queried by the `remember` subagent):

scenarios/<world>/<agent>/
├── memory/
│   ├── chat_trace.txt                 # appended LLM I/O byte-stream per turn
│   ├── conversation.txt               # appended verbatim dialogue
│   ├── companion_state_<entity>.txt   # current-value rolling profile
│   └── discourse_state_<entity>.txt   # current-value commitments/agreements
└── subagent_traces/                   # per-call remember-subagent debug traces
```

## Documentation

| Document | Description |
|----------|-------------|
| **[Background](BACKGROUND.md)** | Research motivation and philosophy |
| **[Tool Development](docs/TOOL_DEVELOPMENT_GUIDE.md)** | Creating new tools (`Skill.md` + `tool.py`) |
| **[Contributor Guidelines](src/AGENTS.md)** | Code style, testing, commit conventions |

## Contributing

See [src/AGENTS.md](src/AGENTS.md) for repository guidelines, code style, and commit conventions.

## License

MIT License — see [LICENSE](LICENSE).
