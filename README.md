# Cognitive Workbench

An agent built around a ReAct chat loop, file-backed memory, and a small set of geofenced subagents.

*Research code, single-author. The chat subproject is the active surface; older infospace/OODA layers remain in the tree but are not exercised — see Limitations.*

## What this is

Most agent demos lean heavily on either retrieval-augmented prompting or tool use. This one keeps both as first-class inputs and treats *reflection* — the background pass between turns — as the layer that updates the agent's state.

Each turn the system prompt is reassembled from persistent state files: persona, self-model, a fair-witness companion model of the user, current discourse agreements, recalled memories, and active concerns. The turn itself is a ReAct loop over a small tool catalog (`process_text`, `search`, `fetch_text`, `recall`, `inspect[_external]`, `security`, `respond`). After the response ships, reflection updates those state files for the next turn.

## Example turn

> *Placeholder — real transcript fragment showing one turn with 2-3 ReAct iterations (thought → tool → observation → respond).*

## Quick start

Python 3.10+. Optional system binaries: `ripgrep` (for `inspect`), `nmap` (for `security`).

```
pip install -r requirements.txt
```

Pick a backend by editing `scenarios/jill-chat.yaml`'s `llm_config` block. The simplest path is `server: anthropic` with `ANTHROPIC_API_KEY` exported; local vLLM and llama.cpp variants are shown in sibling scenario files.

```
cd src
python launcher.py ../scenarios/jill-chat.yaml --cli
```

If the backend isn't configured, the launcher prints what's missing.

At the chat prompt, type **`/help`** for the full list of slash commands — `/recall`, `/concerns`, `/note`, `/img`, `/paste`, `/set-external-repo`, `/status`, `/resources`, and others. Slash commands are the primary way to inspect agent state, send images, and manage concerns / external-repo bindings outside the normal turn flow.

## Concept overview

### Turn structure

A turn runs four phases. Within the turn the prompt is read-only; state files are updated *between* turns.

1. **Orient** — one LLM pass reads active concerns, the companion model, recent context, persona, and self-model; emits a posture line and per-concern relevance.
2. **ReAct** — the main loop. Each iteration emits one JSON action; the dispatcher runs the tool and binds the result to `$stepN`. The loop exits on `respond`.
3. **Respond** — the reply ships to the user.
4. **Reflect** — asynchronous post-turn updates to memories, discourse state, companion model, and concerns. The reply does not wait.

### Read-side and write-side state

Two flavors of state coexist.

**Push state** is always rendered into the system prompt: top-K recalled memories from a FAISS-indexed collection, the companion model text, current discourse agreements, active concerns. The agent does not have to ask for these; they are present.

**Pull state** is the `recall` tool — a read-only subagent that navigates the per-world memory directory (list/read/grep) and synthesizes an answer. Useful when the agent needs to check what was actually said earlier rather than what's currently summarized.

Writes happen only via reflection. There is no in-loop write tool. When the user says "remember X", the agent acknowledges and reflection extracts the memory afterward.

### Geofenced subagents

Three subagents, each a thin persona-less ReAct loop scoped to a typed surface. From the parent's vantage each call is one ReAct step; the synthesized answer binds to `$stepN`. Per-call traces land in a sibling `subagent_traces/` directory.

| Subagent | Scope | Primitives |
|---|---|---|
| `recall` (`src/chat/remember.py`) | per-world per-agent `memory/` directory | list, read (with line ranges), grep |
| `inspect` / `inspect_external` (`src/chat/code_subagent.py`) | own `src/` or an externally-bound repo; gitignore-respecting | list, read, grep (ripgrep) |
| `security` (`src/chat/security.py`) | LAN probes, RFC1918 ranges only | nmap host discovery, nmap -sV, ss/ip |

### Concerns and autonomy

Concerns are persisted, cadence-fired actionable directives — categories `one_shot`, `durable`, `derived` with per-concern firing parameters (`cadence_hours` from `{1, 2, 4, 8, 12, 24, 168}`, `lifetime_days`, `instruction`). When `--autonomy` is set on the launcher, the `tick` sensor (default 30 min) checks for due concerns and fires them through the same ReAct loop, capped per tick.

Off by default.

## Architecture at a glance

```
                  user input
                      │
                      ▼
                 orient pass  ◄── persona · self-model · companion state
                      │            discourse state · recalled memories
                      │            active concerns · recent traces
                      ▼
          ┌────────── ReAct loop ──────────┐
          │  thought → tool → observation  │
          │             │                  │
          │             ▼                  │
          │   process_text · search        │
          │   fetch_text                   │
          │   recall   ──▶ memory/ dir     │  (subagent)
          │   inspect  ──▶ src/   tree     │  (subagent)
          │   security ──▶ LAN            │  (subagent)
          │   respond ─────────┐           │
          └─────────────────────┼──────────┘
                                ▼
                              reply
                                │
                                ▼
                    reflection queue (async)
                                │
                                ▼
              memories · discourse_state · companion_state
              agent_concerns · agent_threads
```

## Configuration

A scenario is a YAML file under `scenarios/`. Minimum shape:

```yaml
world_config:
  world_name: jill_chat

characters:
  Jill:
    mode: chat
    llm_config:
      server: anthropic
      model: claude-opus-4-7
      api_key: ANTHROPIC_API_KEY        # env var name
    discourse:
      enabled: true
    orientation:
      enabled: true
    character: |
      [voice and stance]
    self_model: |
      [architectural account: what the agent is and isn't]
    concerns:
      - text: "..."
        category: durable
    sensors:
      - name: tick
        schedule: "30m"
```

**Backends.** The `server` field selects the route. `anthropic` uses the native Messages API. Cloud OpenAI-compatible endpoints (`openrouter`, `openai`, `xai`, or any URL) use a unified Bearer-auth POST and read the API key from the env var named by `api_key`. Local servers (`vllm`, `llama.cpp`, `sglang_api_server`, `lmstudio`) POST to `vllm_url` without auth and accept grammar / chat-template kwargs.

**Per-scenario tool gating.** `chat.omitted_tools: [...]` removes tools from the catalog (used by benches to ablate behaviors). Catalog numbers shift automatically.

**External-repo binding.** Set `external_repo:` on a character to pre-bind the `inspect_external` subagent. Can also be set at runtime via `/set-external-repo` (sticky for the session).

## Memory substrate

Each character in each world has a `memory/` directory.

**Overwritten snapshots — authoritative:**
- `companion_state_<entity>.txt` — fair-witness model of the user
- `discourse_state_<entity>.txt` — outstanding commitments, current agreements, key decisions

**Append-only history — point-in-time records:**
- `conversation.txt` — verbatim dialogue
- `chat_trace.txt` — per-turn LLM I/O
- `reasoning_trace.jsonl` — per-iteration ReAct records
- `memories.jsonl` — provenance log of memory writes
- `autonomy.jsonl` — log of concern firings

**In-process FAISS collections:** `memories`, `reasoning_history`, `agent_concerns`, `user_concerns`, `agent_threads`. The push-side prompt retrieves via the FAISS index; the `recall` subagent grep/reads the plain-text files.

## Project layout

```
src/
  chat/                  live subproject — ReAct loop, subagents, backend
    chat_loop.py         main per-character chat engine
    remember.py          recall subagent
    code_subagent.py     inspect / inspect_external subagent
    security.py          security subagent
  tools/                 process_text, search, fetch_text implementations
  cli.py                 interactive terminal frontend
  telegram_bridge.py     alternate frontend (Telegram)
  launcher.py            scenario loader + character dispatcher
scenarios/               YAML configs
bench/                   eval harnesses
docs/                    design notes (referenced from code, not duplicated)
```

The tree also contains older infospace / OODA / planner code (`src/primitives/`, `src/Metrics/`, `src/musing/`, `src/saved_plans/`) which `mode: chat` does not import. See Limitations.

## Development

**Adding a tool.** Append a `(name, description)` entry in `_build_react_tool_catalog` in `chat_loop.py` and a dispatch branch in `_run_react_loop`. Return observations with `OK: ` / `EMPTY: ` / `ERROR: ` prefixes so the agent can route on outcome.

**Adding a subagent.** Use `src/chat/remember.py` as the template: a static `_build_system_prompt()`, a small set of read-only primitives, a `respond` exit, and a per-call trace file written to a sibling `*_traces/` directory. Keep the prompt stable across calls so the backend's KV cache hits.

**Adding a scenario.** Copy `scenarios/jill-chat.yaml`, change `world_name` and `characters.<name>`, point the backend, write the persona and self_model. The launcher creates per-world per-agent directories on first run.

**Local-LLM gotchas:**
- llama.cpp GBNF rule names cannot contain underscores — use hyphens.
- `{N,M}` quantifiers in GBNF are approximate, not exact.
- Qwen3 jinja chat templates auto-open `<think>` on the assistant turn; pass `--reasoning-format none` or strip in post-processing.
- Reasoning-model detection is substring-based on model name; override via `is_reasoning_model: true|false` in the scenario YAML if it misfires.

**Benchmarks.** `bench/` holds eval harnesses (discourse reflection, memory recall, recall-subagent prompt A/B, cspred, …). Each has its own README. Runs land in `bench/runs/` (gitignored).

## Limitations and non-goals

- Single-author research code. Breaking changes without notice.
- Memory and reflection are LLM-dependent. Frontier models extract and update reliably; small local models lose recall on subtler write-side moves — see `bench/discourse_reflect/` and `docs/design_note_agreements_rag.md` for the current investigation.
- No multi-agent coordination beyond Zenoh pub/sub primitives.
- The infospace / OODA / executive-node legacy code remains in the tree (`src/primitives/`, `src/Metrics/`, `src/musing/`, `src/saved_plans/`, parts of `bench/`) but is not exercised by `mode: chat` and is not maintained.
- Body / robot integration is **not** in this repo. The Body stack (Pi onboard software, SLAM, navigation, motor control, desktop teleop client) lives in a separate repository.

## License

[TBD]
