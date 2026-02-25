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
- **[Envisioning & Quality Control](docs/envisioning-and-quality-control.md)** — lightweight LLM framing for coherent dialog; post-execution reflection for failure recovery and learning
- **[Infospace Memory](docs/architecture.md#infospace-memory-model)** — Notes, Collections, and Relations as structured working memory with FAISS semantic search
- **[Extensible Tools](docs/tools-and-primitives.md)** — 20+ built-in tools (web search, email, academic papers, shell scripts) plus world-specific integrations
- **[Missing Affordance Monitoring](docs/envisioning-and-quality-control.md#missing-affordance-monitoring)** — automatic detection and logging of capability gaps for future tool development
- **[World Integrations](docs/configuration.md#available-scenarios)** — optional worlds (Minecraft, file system, desktop automation, ScienceWorld) with specialized tools
- **[Web UI](docs/ui-guide.md)** — real-time dashboard with action log, goal scheduling, and resource browser

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

**Option A — Local GPU (SGLang):** Edit `scenarios/jill-infospace.yaml` and set `sgl_model_path` to your preferred model.

**Option B — Cloud API (no GPU needed):**
```bash
export OPENROUTER_API_KEY="sk-or-v1-..."   # from openrouter.ai
```

### 3. Run

```bash
cd src
python3 launcher.py ../scenarios/jill-infospace.yaml --ui --resource-browser
# Or for OpenRouter:
python3 launcher.py ../scenarios/jill-infospace-openrouter.yaml --ui --resource-browser
```

Open [http://localhost:3000](http://localhost:3000) and type:
```
goal: Find and summarize recent papers on transformer architectures
```

See [Getting Started](docs/getting-started.md) for full setup details, environment variables, and troubleshooting.

## Documentation

| Document | Description |
|----------|-------------|
| **[Getting Started](docs/getting-started.md)** | Installation, credentials, LLM backend setup, first run |
| **[Architecture](docs/architecture.md)** | Core cognitive architecture — incremental planner, OODA loop, infospace memory |
| **[Goals & Scheduling](docs/goals-and-scheduling.md)** | Goal submission (`goal:` prefix), scheduled goals, daily-at-time, autonomous execution |
| **[Envisioning & QC](docs/envisioning-and-quality-control.md)** | Conversational envisioning, reflection, failure recovery, missing affordance monitoring |
| **[Tools & Primitives](docs/tools-and-primitives.md)** | Infospace primitives, tool catalog, run-script, plan tools |
| **[Configuration](docs/configuration.md)** | Scenario YAML reference, available scenarios, directory structure |
| **[UI Guide](docs/ui-guide.md)** | Web dashboard, resource browser, API endpoints |
| **[Tool Development](docs/TOOL_DEVELOPMENT_GUIDE.md)** | Creating new tools (`Skill.md` + `tool.py`) |
| **[Background](BACKGROUND.md)** | Research motivation and philosophy |
| **[Contributor Guidelines](src/AGENTS.md)** | Code style, testing, commit conventions |

## How It Works (In Brief)

1. **You submit a goal**: `goal: Monitor stock prices and alert on changes > 5%`
2. **The Executive Node** queues it and invokes the Incremental Planner
3. **The Planner** retrieves relevant context (FAISS), selects tools, then enters a generate-execute-evaluate loop:
   - LLM writes a code block calling tools (`search-web`, `stock-price`, `create-note`, etc.)
   - Executor runs it, returns structured results
   - LLM evaluates: done? next step? error recovery?
4. **Reflection** analyzes the full execution trace — updates task state, world model, tool insights
5. **If it failed** with a missing capability, the gap is logged for future tool development
6. **Scheduled goals** can repeat daily at a set time, or auto-proceed through multi-step workflows

## Available Scenarios

| Scenario | World | Backend |
|----------|-------|---------|
| `jill-infospace.yaml` | Core infospace | SGLang (local GPU) |
| `jill-infospace-openrouter.yaml` | Core infospace | OpenRouter (cloud) |
| `jill-fs.yaml` | File system | SGLang |
| `jill-fs-anthropic.yaml` | File system | Anthropic Claude |
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
└── src/
    ├── launcher.py                    # Entry point
    ├── executive_node.py              # OODA loop coordinator
    ├── incremental_planner.py         # Core planner (the heart of the system)
    ├── infospace_executor.py           # Primitives + tool execution
    ├── infospace_resource_manager.py   # Notes/Collections/Relations + FAISS
    ├── fastapi_action_display.py      # Web UI
    ├── task_scheduler.py              # Autonomous goal scheduling
    ├── tools/                         # Core tools (search-web, run-script, etc.)
    ├── world-tools/                   # World-specific tools (minecraft, fs, etc.)
    ├── scripts/                       # Shell scripts for run-script tool
    └── utils/                         # Shared utilities
```

## Contributing

See [src/AGENTS.md](src/AGENTS.md) for repository guidelines, code style, and commit conventions.

## License

MIT License — see [LICENSE](LICENSE).
