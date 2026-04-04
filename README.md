# Cognitive Workbench

A research framework for autonomous agents with incremental planning, persistent memory, and tool use.

[![Status: Research Laboratory](https://img.shields.io/badge/status-research_laboratory-purple.svg)]()
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## What This Is

Cognitive Workbench is experimental research software for studying LLM-based cognitive architectures. It prioritizes **inspectable agent behavior** and fast iteration over stability.

The core idea: an **incremental planner** that interleaves reasoning with tool execution. Rather than generating a complete plan and then executing it, the planner generates one step at a time, runs it, observes the result, and decides what to do next. This tight feedback loop — combined with persistent memory, reflective quality control, and autonomous goal scheduling — produces agents that can pursue complex goals over extended periods.

```
User: "goal: Find recent papers on multi-agent coordination"
                    │
         ┌──────────▼──────────┐
         │   Executive Node    │  OODA loop: Observe → Orient → Decide → Act
         │   (goal queue,      │
         │    scheduling)      │
         └──────────┬──────────┘
                    │
         ┌──────────▼──────────┐
         │ Incremental Planner │  Stage 0: Retrieve context (FAISS)
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
```

## Key Features

- **[Incremental Planning](docs/architecture.md)** — the planner interleaves LLM reasoning with tool execution, adapting its approach based on real results
- **[Goal Scheduling](docs/goals-and-scheduling.md)** — submit goals with `goal:` prefix; schedule them for manual, automatic, recurring, or daily-at-time execution
- **[Concern Model](docs/user_concern_model.md)** — user concerns and agent-derived concerns with activation-based triage into actionable tasks
- **[Envisioning & Quality Control](docs/envisioning-and-quality-control.md)** — lightweight LLM framing for coherent dialog; post-execution reflection for failure recovery and learning
- **[Infospace Memory](docs/architecture.md#infospace-memory-model)** — Notes, Collections, and Relations as structured working memory with FAISS semantic search + entity-augmented retrieval
- **[NER & Entity Graph](docs/architecture.md#named-entity-recognition-ner-pipeline)** — automatic entity extraction from user input, goals, and notes; cognitive graph integration with entity nodes and mentions edges ([explorer guide](docs/cognitive_graph_explorer.md))
- **[Theory of Mind](docs/architecture.md#theory-of-mind-tom)** — persistent per-peer models of trust, competence, goals, and emotional state, updated from conversation evidence
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

python3 launcher.py ../scenarios/jill-infospace.yaml --ui --resource-browser --task-manager
# Or for OpenRouter:
python3 launcher.py ../scenarios/jill-infospace-openrouter.yaml --ui --resource-browser --task-manager
```

Open [http://localhost:3000](http://localhost:3000) and submit a goal via the **+ Goal** button:
```
Find and summarize recent papers on transformer architectures
```

See [Getting Started](docs/getting-started.md) for full setup details, environment variables, and troubleshooting.

## Web UI

The system provides four web-facing components. See the [UI Guide](docs/ui-guide.md) for full details.

### Activation Field (port 3000)

The default view is an interactive D3 force-directed graph centered on the agent. Nodes represent the agent, its goals, concerns, notes, and variable bindings — sized and colored by activation level. Click any node to inspect it in the side panel.

The bottom dock bar provides controls for chat, goal entry, execution control (stop, continuous, LLM toggle), and links to the other UI components.

An OODA pulse overlay shows the agent's cognitive cycle in real time — expanding colored rings indicate Observe (blue), Orient (yellow), Decide (orange), and Act (green) phases.

### Classic UI (port 3000/classic)

A text-oriented alternative with a scrollable action log, character sidebar with tabs (Plan, Bindings, Goals, Plans, State, Schedule, Tasks), and direct text input for goals and chat.

### Resource Browser (port 3001)

Browse, view, edit, and delete Notes and Collections — the agent's working memory. Two-panel layout with a resource list and content viewer.

### Task & Concern Manager (port 3002)

Monitor the concern-to-task pipeline. The left panel shows user and derived concerns with activation levels and management controls (close, resolve, abandon, delete). The right panel shows task WIPs with approve/edit/abandon controls, scheduled goals, situation notes, and triage status.

### Browser Extension (optional)

A Chrome extension that captures page visits and feeds them to the agent via the `browser-visits` sensor. Install by loading the `browser_extension/` directory as an unpacked extension.

## How It Works (In Brief)

1. **You type a message**: the unified chat handler decides whether to respond conversationally, escalate to a goal (tool use needed), or dispatch a system command — all in a single LLM call
2. **The Executive Node** queues goals and invokes the Incremental Planner
3. **The Planner** retrieves relevant context (FAISS semantic search + entity-augmented retrieval), selects tools, then enters a generate-execute-evaluate loop:
   - LLM writes a code block calling tools (`search-web`, `stock-price`, `create-note`, etc.)
   - Executor runs it, returns structured results
   - LLM evaluates: done? next step? error recovery?
4. **Reflection** analyzes the full execution trace — updates world model (recency-weighted Bayesian facts), tool insights, and cross-goal learnings
5. **Named entities** are extracted from user input, goals, and persistent notes — building a cognitive graph of entities and mentions that improves retrieval over time
6. **Theory of Mind** models are updated when conversations are archived (`/done`, `/next`, `/bye`), tracking trust, competence, goals, and emotional state per peer
7. **Scheduled goals** can repeat daily at a set time, or auto-proceed through multi-step workflows
8. **Sensors** (browser visits, RSS feeds) run on timers and feed real-world context back into the agent's concern model

## Available Scenarios

| Scenario | World | Backend |
|----------|-------|---------|
| `jill-infospace.yaml` | Core infospace | SGLang (local GPU) |
| `jill-infospace-openrouter.yaml` | Core infospace | OpenRouter (cloud) |
| `jill-infospace-anthropic.yaml` | Core infospace | Anthropic Claude |
| `jill-infospace-openai.yaml` | Core infospace | OpenAI |
| `jill-infospace-vllm.yaml` | Core infospace | vLLM (local GPU) |
| `jill-fs.yaml` | File system | SGLang |
| `jill-fs-openrouter.yaml` | File system | OpenRouter (cloud) |
| `jill-minecraft.yaml` | Minecraft 3D world | SGLang |
| `jill-osworld.yaml` | Desktop automation | SGLang |
| `jill-scienceworld.yaml` | Science simulation | SGLang |
| `jack-and-jill.yaml` | Multi-agent | SGLang |

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
    ├── launcher.py                    # Entry point
    ├── executive_node.py              # OODA loop coordinator
    ├── incremental_planner.py         # Core planner (the heart of the system)
    ├── infospace_executor.py           # Primitives + tool execution
    ├── infospace_resource_manager.py   # Notes/Collections/Relations + FAISS
    ├── entity_index.py                # NER extraction, entity index, graph integration
    ├── cognitive_graph.py             # OODA event graph + entity/ToM nodes
    ├── conversation_store.py          # Dialog lifecycle, archival, session backfill
    ├── discourse.py                   # Theory of Mind templates + discourse analysis
    ├── world_model.py                 # Bayesian recency-weighted knowledge
    ├── fastapi_action_display.py      # Web UI (Activation Field + Classic)
    ├── resource_browser.py            # Resource Browser UI
    ├── task_manager.py                # Task & Concern Manager UI
    ├── goal_scheduler.py              # Autonomous goal scheduling
    ├── concern_triage.py              # Concern → task pipeline
    ├── derived_concern_model.py       # Agent-derived concerns
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
| **[UI Guide](docs/ui-guide.md)** | Activation Field, Classic UI, Resource Browser, Task Manager, sensors |
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
