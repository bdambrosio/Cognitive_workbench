# Cognitive Workbench

A research framework for autonomous agents with incremental planning, persistent memory, and tool use.

[![Status: Research Laboratory](https://img.shields.io/badge/status-research_laboratory-purple.svg)]()
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## Project Direction (April 2026)

The chat-mode subproject (`src/chat/`) is becoming the primary interface. It already provides:

- **ReAct tool use** — web search, full-page fetch, in-context text transformation, with per-iteration tracing
- **Long-term memory** — per-character `memories` collection with categorized recall (`fact` / `preference` / `commitment`), auto-RAG injection at turn start, and post-turn reflection that suppresses writes from hypothetical / role-play / counterfactual frames
- **Companion Model + Discourse tracking** — single-user fair-witness texture and outstanding-discourse state, persisted across sessions
- **Unified cloud-LLM config** — `api_key` field naming an env var triggers Bearer-auth POST to any OpenAI-compatible endpoint (MIMO, OpenRouter, OpenAI, hosted vLLM, …); legacy server shortcuts still work

The full executive-node architecture described below — continuous OODA planner, incremental planner, cognitive graph, concerns, sensors — remains operational, but future development is shifting to chat mode. **Concerns and sensors are the main pieces that still need to be ported into chat before executive-mode components can be retired.** This is a stated intention, not a present-day rewrite; the executive path is still fully usable today and is what runs by default for `jill-infospace*.yaml` scenarios. Chat mode runs from `jill-chat*.yaml` scenarios via the same launcher.

## What This Is

Cognitive Workbench is experimental research software for studying LLM-based cognitive architectures. It prioritizes **inspectable agent behavior** and fast iteration over stability.

Two coupled loops sit at the core. A **continuous OODA planner** maintains strategic context across cycles — choosing at each turn whether to submit a goal, update a concern, ask, say, reflect, or sleep — rather than resetting reasoning every tick. When it launches a goal, an **incremental planner** takes over and interleaves LLM reasoning with tool execution, generating one step at a time and adapting to real results. Every OODA event, decision, action, and outcome is recorded as a typed node in a persistent **cognitive graph** that serves as both long-term memory and a reflective computational trace.

```
User: "goal: Find recent papers on multi-agent coordination"
                    │
         ┌──────────▼──────────┐
         │  OODA Planner       │  Continuous strategic loop — context persists
         │  (ooda_planner.py)  │  across cycles. Actions: submit-goal,
         │                     │  update-concern, say, ask, reflect, sleep, ...
         │  ┌───────────────┐  │  Event-action history with progressive rollup
         │  │ Observe/Orient│  │
         │  │ Decide → Act  │  │  Writes every stage into the cognitive graph
         │  └───────────────┘  │
         └──────────┬──────────┘
                    │ submit-goal
         ┌──────────▼──────────┐
         │ Incremental Planner │  Stage 0: Retrieve context (FAISS + graph)
         │                     │  Stage 1: Analyze + select tools
         │  ┌───────────────┐  │  Stage 2: Generate code → Execute → Evaluate
         │  │ Reason → Act  │──│──────► repeat until done
         │  │ ← Observe     │  │
         │  └───────────────┘  │  Reflect: learn from execution trace
         └──────────┬──────────┘
                    │
         ┌──────────▼──────────┐
         │ Infospace Executor   │  Primitives + Tools
         │                     │  Notes + Collections + Relations
         │  search-web, say,   │  FAISS semantic search
         │  create-note, ...   │  Persistent memory
         └─────────────────────┘

         ┌─────────────────────┐
         │  Cognitive Graph    │  Typed nodes (event, assessment, decision,
         │  (cognitive_graph.py)│ goal_launch/outcome, concern_change, ...)
         │                     │  FAISS-backed semantic search + BFS subgraph
         │                     │  expansion; idle-time consolidation
         └─────────────────────┘
```

## Key Features

- **[Continuous OODA Planner](docs/ooda-as-incremental-planner.md)** — single strategic loop that persists context across cycles; chooses each turn from nine actions (submit-goal, update-concern, say, ask, configure-sensor, reflect, wait, sleep, update-user-model). Event-action history is kept with progressive rollup so the LLM sees recent cycles in full and older strategic intent in summary.
- **[Incremental Planning](docs/architecture.md)** — once a goal is submitted, the inner planner interleaves LLM reasoning with tool execution, adapting its approach based on real results
- **[Cognitive Graph & Reflective Trace](docs/cognitive_graph_spec.md)** — a persistent, FAISS-backed graph of typed nodes (events, assessments, decisions, action results, goal launches/outcomes, concern changes, triage nominations, conversation turns) and directed edges recording the full OODA provenance. Supports semantic search, BFS subgraph expansion, and idle-time consolidation that summarizes old windows into `consolidation` nodes ([explorer guide](docs/cognitive_graph_explorer.md))
- **[Concerns & Derived Concerns](docs/concerns-architecture.md)** — user concerns and agent-derived concerns with a recurring lifecycle: `active` → `satisfied` (with per-concern revisit timer) → back to `active` when the timer expires; `abandoned` is the only terminal state. Homeostatic time-pressure keeps seeded concerns alive; the OODA planner's `update-concern` action adjusts weight/status/notes directly, and an LLM-activation path lets derived-concern reasoning nominate without waiting for an activation threshold
- **[Goal Scheduling](docs/goals-and-scheduling.md)** — submit goals with `goal:` prefix; schedule them for manual, automatic, recurring, or daily-at-time execution
- **[Envisioning & Quality Control](docs/envisioning-and-quality-control.md)** — lightweight LLM framing for coherent dialog; post-execution reflection for failure recovery and learning
- **[Infospace Memory](docs/architecture.md#infospace-memory-model)** — Notes, Collections, and Relations as structured working memory with FAISS semantic search + entity-augmented retrieval
- **[Theory of Mind + Companion Model](docs/architecture.md#theory-of-mind-tom)** — two complementary lenses on peers. ToM (all peers) tracks trust, competence, reliability, transparency, goals, and emotional state. The Companion Model (user only) adds a friendship lens: current chapter, state of mind, what matters, thinking style, what's on their mind, and how to be useful right now. Fast-moving sections update per conversation; slow-moving ones only on new evidence
- **[World Model](docs/architecture.md#world-model-bayesian-recency-weighted)** — Bayesian cross-goal knowledge with recency-weighted evidence decay and staleness detection
- **[Extensible Tools](docs/tools-and-primitives.md)** — 24 built-in tools (web search, email, Bluesky, academic papers, shell scripts) plus world-specific integrations
- **[Sensors](docs/ui-guide.md#sensors)** — autonomous data collectors (browser visit tracking, RSS feeds) that feed real-world context to the agent
- **[Web UI](docs/ui-guide.md)** — real-time activation field visualization, chat, goal management, resource browser, and task/concern manager
- **[World Integrations](docs/configuration.md#available-scenarios)** — optional worlds (Minecraft, file system, desktop automation, ScienceWorld) with specialized tools

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

**Option A — Local GPU (SGLang):**
 - Edit `scenarios/jill-infospace.yaml` and set `sgl_model_path` to your preferred model. SGLang can be finicky, sorry, but use of @function makes reasoning loop so much faster.
 - Or `scenarios/jill-infospace-vllm.yaml` and set `vllm_model_path` to your preferred model.

**Option B — Cloud API (no GPU needed):**
```bash
export OPENROUTER_API_KEY="sk-or-v1-..."   # from openrouter.ai
```

**Alt Model for semantic processing:**
Some tools, like refine, extract-struct, filter-semantic, assess, perform complex semantic processing of text (e.g. extracting field from json). If your basic llm isn't up to the task, you can provide a heavier weight model for these to use:
```yaml
alt_llm_config:
  openrouter_model_path: "qwen/qwen3-235b-a22b-2507"
```

### 3. Run

```bash
source zenoh_venv/bin/activate
cd src

python3 launcher.py ../scenarios/jill-infospace.yaml --cli --resource-browser
# Or with the web UI:
python3 launcher.py ../scenarios/jill-infospace.yaml --ui --resource-browser
# Or for OpenRouter:
python3 launcher.py ../scenarios/jill-infospace-openrouter.yaml --ui --resource-browser
```

### 4. Optional: Browser automation

The `browse` tool requires the [agent-browser](https://github.com/vercel-labs/agent-browser) CLI (Rust binary, not a Python package):

```bash
cargo install agent-browser        # if you have Rust/cargo
# or download a prebuilt binary from https://github.com/vercel-labs/agent-browser/releases
```

Skip this if you don't need browser automation — all other tools work without it.

Open [http://localhost:3000](http://localhost:3000) and submit a goal via the **+ Goal** button:
```
Find and summarize recent papers on transformer architectures
```

See [Getting Started](docs/getting-started.md) for full setup details, environment variables, and troubleshooting.

## Web UI

The system provides three web-facing components plus an optional browser extension. See the [UI Guide](docs/ui-guide.md) for full details.

### Activation Field (port 3000)

The default view is an interactive D3 force-directed graph centered on the agent. Nodes represent the agent, its goals, concerns, notes, and variable bindings — sized and colored by activation level. Click any node to inspect it in the side panel.

The bottom dock bar provides controls for chat, goal entry, execution control (stop, continuous, LLM toggle), and links to the other UI components.

An OODA pulse overlay shows the agent's cognitive cycle in real time — expanding colored rings indicate Observe (blue), Orient (yellow), Decide (orange), and Act (green) phases.

### Classic UI (port 3000/classic)

A text-oriented alternative with a scrollable action log, character sidebar with tabs (Plan, Bindings, Goals, Plans, State, Schedule, Tasks), and direct text input for goals and chat.

### Resource Browser (port 3001)

Browse, view, edit, and delete Notes, Collections, and Concerns — the agent's working memory. Two-panel layout with a resource list and content viewer. The Concerns tab shows user and derived concerns with activation, weight, revisit interval, and status/delete actions (replaces the previous standalone Task Manager).

### Browser Extension (optional)

A Chrome extension that captures page visits and feeds them to the agent via the `browser-visits` sensor. Install by loading the `browser_extension/` directory as an unpacked extension.

## How It Works (In Brief)

1. **You type a message**: the unified chat handler decides whether to respond conversationally, escalate to a goal (tool use needed), or dispatch a system command — all in a single LLM call
2. **The OODA planner runs continuously**: on strategic events (timer ticks, sensor input, concern activation, `inform`) it assembles live context — concerns, goals, sensors, cognitive-graph slices, character/capabilities — and emits one JSON action per cycle. Chat and alerts take a fast path but receive a summary of ongoing OODA activity for awareness
3. **Goal execution**: when the planner picks `submit-goal`, the Executive Node hands off to the Incremental Planner, which retrieves context (FAISS + entity-augmented + cognitive-graph subgraph), then loops:
   - LLM writes a code block calling tools (`search-web`, `stock-price`, `create-note`, etc.)
   - Executor runs it, returns structured results
   - LLM evaluates: done? next step? error recovery?
4. **Reflection** analyzes the full execution trace — updates world model (recency-weighted Bayesian facts), tool insights, and cross-goal learnings. Every OODA stage and goal lifecycle transition is also recorded as typed nodes and edges in the cognitive graph, forming a reflective computational trace the agent can query later
5. **Named entities** are extracted from user input, goals, and persistent notes — feeding the cognitive graph with entity nodes and `mentions` edges that improve retrieval over time
6. **Concerns evolve with a recurring lifecycle**: active → satisfied (for the concern's revisit interval) → active again. Seeded concerns accrue homeostatic time-pressure so they re-surface on their own; the planner can directly `update-concern` rather than waiting for activation triage
7. **Theory of Mind and Companion Model** are updated when conversations are archived (`/done`, `/next`, `/bye`). ToM covers every peer (trust, competence, goals, emotional state); the Companion Model runs only for the user and captures the "how are they right now" picture that shapes engagement style
8. **Scheduled goals** can repeat daily at a set time, or auto-proceed through multi-step workflows
9. **Sensors** (browser visits, RSS feeds) run on timers and feed real-world context back into the agent's concern model

## Available Scenarios

| Scenario | Mode | World | Backend |
|----------|------|-------|---------|
| `jill-chat.yaml` | **Chat (primary)** | Chat-only world | OpenAI-compatible local server |
| `jill-chat-vllm.yaml` | Chat | Chat-only world | vLLM (local GPU) |
| `jill-chat-mimo.yaml` | Chat | Chat-only world | MIMO cloud (unified `api_key` form) |
| `jill-infospace.yaml` | Executive (legacy) | Core infospace | SGLang (local GPU) |
| `jill-infospace-openrouter.yaml` | Executive (legacy) | Core infospace | OpenRouter (cloud) |
| `jill-infospace-anthropic.yaml` | Executive (legacy) | Core infospace | Anthropic Claude |
| `jill-infospace-openai.yaml` | Executive (legacy) | Core infospace | OpenAI |
| `jill-infospace-vllm.yaml` | Executive (legacy) | Core infospace | vLLM (local GPU) |
| `jill-fs.yaml` | Executive | File system | SGLang |
| `jill-fs-openrouter.yaml` | Executive | File system | OpenRouter (cloud) |
| `jill-minecraft.yaml` | Executive | Minecraft 3D world | SGLang |
| `jill-osworld.yaml` | Executive | Desktop automation | SGLang |
| `jill-scienceworld.yaml` | Executive | Science simulation | SGLang |
| `jack-and-jill.yaml` | Executive | Multi-agent | SGLang |

See [Configuration](docs/configuration.md) for details on each.

## Repository Structure

```
Cognitive_workbench/
├── README.md                          # This file
├── BACKGROUND.md                      # Research philosophy
├── requirements.txt                   # Python dependencies
├── docs/                              # Detailed documentation
├── scenarios/                         # Scenario YAML files + runtime data
├── browser_extension/                 # Chrome extension for page visit tracking
└── src/
    ├── launcher.py                    # Entry point — dispatches by scenario `mode`
    ├── chat/                          # Chat-mode subproject (becoming primary)
    │   └── chat_loop.py               #   ReAct loop, memories collection, fetch_text, unified cloud LLM
    ├── executive_node.py              # Main tick coordinator, fast-path chat, goal lifecycle (legacy)
    ├── ooda_planner.py                # Continuous OODA planner (legacy)
    ├── incremental_planner.py         # Inner goal planner (legacy)
    ├── infospace_executor.py           # Primitives + tool execution
    ├── infospace_resource_manager.py   # Notes/Collections/Relations + FAISS (shared by chat and executive)
    ├── entity_index.py                # NER extraction, entity index, graph integration
    ├── cognitive_graph.py             # Typed event graph — reflective computational trace
    ├── conversation_store.py          # Dialog lifecycle, archival, session backfill
    ├── discourse.py                   # Theory of Mind + Companion Model templates
    ├── world_model.py                 # Bayesian recency-weighted knowledge
    ├── fastapi_action_display.py      # Web UI (Activation Field + Classic)
    ├── resource_browser.py            # Resource Browser UI
    ├── goal_scheduler.py              # Autonomous goal scheduling
    ├── concern_triage.py              # Concern nomination paths (activation, orient, LLM)
    ├── user_concern_model.py          # User concerns with recurring lifecycle
    ├── derived_concern_model.py       # Agent-derived concerns + revisit timers
    ├── sensor_runner.py               # Sensor scheduling and execution
    ├── sensors/                       # Sensor implementations
    │   ├── browser-visits/            # Browser page visit sensor
    │   └── rss-watcher/               # RSS feed monitor
    ├── tools/                         # Core tools (search-web, run-script, etc.)
    ├── world-tools/                   # World-specific tools (minecraft, fs, etc.)
    ├── static/ui/                     # Activation Field frontend (HTML/JS/CSS)
    ├── scripts/                       # Shell scripts for run-script tool
    └── utils/                         # Shared utilities
```

## Documentation

| Document | Description |
|----------|-------------|
| **[Getting Started](docs/getting-started.md)** | Installation, credentials, LLM backend setup, first run |
| **[Architecture](docs/architecture.md)** | Core cognitive architecture — incremental planner, OODA loop, infospace memory |
| **[OODA as Incremental Planner](docs/ooda-as-incremental-planner.md)** | Continuous strategic planner: action schemas, context assembly, event-action history ([decisions log](docs/ooda-incremental-planner-decisions.md)) |
| **[Cognitive Graph Spec](docs/cognitive_graph_spec.md)** | Typed nodes/edges, FAISS semantic index, consolidation, reflective trace ([explorer](docs/cognitive_graph_explorer.md)) |
| **[Concerns Architecture](docs/concerns-architecture.md)** | User + derived concerns, revisit lifecycle, homeostatic pressure, triage paths ([user concern model](docs/user_concern_model.md)) |
| **[UI Guide](docs/ui-guide.md)** | Activation Field, Classic UI, Resource Browser (with Concerns tab), sensors |
| **[Goals & Scheduling](docs/goals-and-scheduling.md)** | Goal submission (`goal:` prefix), scheduled goals, daily-at-time, autonomous execution |
| **[Envisioning & QC](docs/envisioning-and-quality-control.md)** | Conversational envisioning, reflection, failure recovery, missing affordance monitoring |
| **[Tools & Primitives](docs/tools-and-primitives.md)** | Infospace primitives, tool catalog, run-script, plan tools |
| **[Configuration](docs/configuration.md)** | Scenario YAML reference, available scenarios, directory structure |
| **[Tool Development](docs/TOOL_DEVELOPMENT_GUIDE.md)** | Creating new tools (`Skill.md` + `tool.py`) |
| **[Background](BACKGROUND.md)** | Research motivation and philosophy |
| **[Contributor Guidelines](src/AGENTS.md)** | Code style, testing, commit conventions |

## Contributing

See [src/AGENTS.md](src/AGENTS.md) for repository guidelines, code style, and commit conventions.

## License

MIT License — see [LICENSE](LICENSE).
