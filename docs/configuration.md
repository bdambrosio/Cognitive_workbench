# Configuration

## Scenario YAML

Every run is defined by a scenario YAML file in `scenarios/`. The scenario specifies the LLM backend, world integration, and one or more characters.

### Annotated Example

```yaml
# --- World Configuration ---
world_config:
  world_name: "infolab"          # Loads tools from src/world-tools/<world_name>/
                                  # Use "infolab" for core infospace (no world tools)
  # port: 3003                    # Optional: world-specific service port

# --- LLM Configuration ---
llm_config:
  # Choose ONE backend:
  sgl_model_path: "Qwen/Qwen3-30B-A3B-Instruct-2507"    # SGLang (local GPU)
  # openrouter_model_path: "deepseek/deepseek-v3.2"      # OpenRouter (cloud)
  # anthropic_model_path: "claude-sonnet-4-6"             # Anthropic (cloud)

  grobid: "http://localhost:8070/"    # Optional: GROBID PDF parsing server
  pdf_parser: "pymupdf"               # "pymupdf" or "grobid" (default: grobid if available)

# --- Alt LLM (optional) ---
# alt_llm_config:                      # Optional: second LLM for extraction/synthesis tools
#   openrouter_model_path: "anthropic/claude-3.5-sonnet"  # No grobid here

# --- World Description ---
setting: |
  Cognitive Workbench, a research framework for building autonomous agents
  with persistent memory, goal-directed planning, and tool use.
  Characters operate in information spaces.

# --- Characters ---
characters:
  Jill:
    manual: true                 # Receive goals from user input
    manual_response: false       # Don't auto-respond to non-goal messages

    task_scheduler:              # Goal scheduler configuration
      enabled: true
      interval: 15               # Check interval in minutes

    character: |                 # Character description (seen by the planner)
      Jill, a young AI assistant. Jill works as research assistant for User.
      She is widely knowledgeable in philosophy, cognitive science, and AI.

    capabilities: |              # Capabilities description (seen by the planner)
      Jill exists as an agent within Cognitive Workbench. She operates in
      the infospace, receiving and executing goals from the user. She has
      access to Note/Collection primitives, web search, and shell access.
```

### Configuration Sections

#### `world_config`

| Field | Required | Description |
|-------|----------|-------------|
| `world_name` | Yes | Which world tools to load. Use `"infolab"` for none |
| `port` | No | Port for world-specific services (e.g., Minecraft bot) |
| `state` | No | Initial world state as JSON string |

#### `llm_config`

| Field | Required | Description |
|-------|----------|-------------|
| `sgl_model_path` | One of these | HuggingFace model ID for SGLang local inference |
| `openrouter_model_path` | required | OpenRouter model ID (e.g., `deepseek/deepseek-v3.2`) |
| `anthropic_model_path` | | Anthropic model ID (e.g., `claude-sonnet-4-6`) |
| `grobid` | No | GROBID server URL for PDF parsing |
| `pdf_parser` | No | `"pymupdf"` or `"grobid"` (default: grobid if configured) |

#### `alt_llm_config` (optional)

Optional second LLM backend used by content-processing tools: `synthesize`, `extract`, `extract-struct`, `extract-references`, `refine`. LLM-only (no grobid). When absent, those tools use the main `llm_config` backend.

| Field | Required | Description |
|-------|----------|-------------|
| `openrouter_model_path` | One of these | OpenRouter model ID |
| `vllm_model_path` | | vLLM model name |
| `anthropic_model_path` | | Anthropic model ID |
| `openrouter_provider` | No | OpenRouter provider pin (see main llm_config) |

#### `characters`

Each character is a named entry with:

| Field | Required | Description |
|-------|----------|-------------|
| `manual` | No | `true` = receive goals from user input (default: false) |
| `manual_response` | No | `true` = auto-respond to non-goal messages |
| `character` | Yes | Character description (multiline string) |
| `capabilities` | Yes | Capabilities description (multiline string) |
| `task_scheduler` | No | Scheduler config (see below) |

#### `task_scheduler` (per character)

| Field | Default | Description |
|-------|---------|-------------|
| `enabled` | `false` | Enable the autonomous goal scheduler |
| `interval` | `15` | How often (minutes) the scheduler checks for eligible goals |

## Available Scenarios

| File | Backend | World | Description |
|------|---------|-------|-------------|
| `jill-infospace.yaml` | SGLang | infolab | Core agent with web search, semantic scholar |
| `jill-infospace-openrouter.yaml` | OpenRouter | infolab | Same, via OpenRouter API |
| `jill-infospace-anthropic.yaml` | Anthropic | infolab | Same, via Anthropic Claude |
| `jill-infospace-openai.yaml` | OpenAI | infolab | Same, via OpenAI API |
| `jill-infospace-vllm.yaml` | vLLM | infolab | Same, via vLLM backend |
| `jill-fs.yaml` | SGLang | fs | File system tools (list, read, grep, find) |
| `jill-fs-openrouter.yaml` | OpenRouter | fs | File system via OpenRouter |
| `jill-minecraft.yaml` | SGLang | minecraft | 3D world navigation and crafting |
| `jill-osworld.yaml` | SGLang | osworld | Desktop automation |
| `jill-scienceworld.yaml` | SGLang | scienceworld | Science simulation tasks |
| `jack-and-jill.yaml` | SGLang | — | Multi-agent (two characters) |

## Creating a New Scenario

1. Copy an existing scenario that's close to what you want:
   ```bash
   cp scenarios/jill-infospace.yaml scenarios/my-agent.yaml
   ```

2. Edit `llm_config` to set your preferred LLM backend

3. Modify `character` and `capabilities` to define your agent's personality and role

4. Optionally change `world_name` to load different world tools

5. Run it:
   ```bash
   cd src
   python3 launcher.py ../scenarios/my-agent.yaml --ui
   ```

## Data Directories

Runtime data is organized per scenario. These directories are gitignored.

```
scenarios/
  <scenario_name>/
    resources/              # Persistent Notes, Collections, Relations (JSON)
    memory/                 # Character memory artifacts
    vector/                 # FAISS vector indexes (.faiss files)

logs/
  launcher.log              # Launcher startup and process management
  executive_node.log        # OODA loop, goal processing
  incremental_planner.log   # Planning stages, tool selection, reflection
  fastapi_action_display.log # Web UI events
  action_trace_*.md         # Markdown trace of all actions per scenario
```

### Clearing Data

To reset a character's state, delete the scenario data directory:
```bash
rm -rf scenarios/<name>/resources/ scenarios/<name>/vector/
```

To clear just the vector index (forces re-indexing):
```bash
rm -rf scenarios/<name>/vector/
```

## Project Directory Structure

```
Cognitive_workbench/
├── README.md                      # Overview and quick start
├── BACKGROUND.md                  # Research motivation and philosophy
├── requirements.txt               # Python dependencies
├── scenarios/                     # Scenario YAML files + runtime data
│   ├── jill-infospace.yaml
│   ├── jill-minecraft.yaml
│   └── ...
├── docs/                          # Documentation
│   ├── architecture.md
│   ├── getting-started.md
│   └── ...
├── src/
│   ├── launcher.py                # Entry point
│   ├── executive_node.py          # OODA loop coordinator
│   ├── incremental_planner.py     # Core planner
│   ├── infospace_executor.py      # Primitive + tool execution
│   ├── infospace_resource_manager.py  # Persistence + FAISS
│   ├── fastapi_action_display.py  # Web UI (Activation Field + Classic)
│   ├── resource_browser.py        # Resource Browser UI (port 3001)
│   ├── task_manager.py            # Task & Concern Manager UI (port 3002)
│   ├── goal_scheduler.py          # Autonomous goal scheduler
│   ├── concern_triage.py          # Concern → task pipeline
│   ├── derived_concern_model.py   # Agent-derived concerns
│   ├── sensor_runner.py           # Sensor scheduling and execution
│   ├── conversation_store.py      # Dialog tracking
│   ├── tool_model.py              # Tool success tracking
│   ├── world_model.py             # Persistent knowledge base
│   ├── templates.py               # Prompt templates
│   ├── AGENTS.md                  # Contributor guidelines
│   ├── tools/                     # Core tools
│   │   ├── search-web/
│   │   ├── semantic-scholar/
│   │   ├── run-script/
│   │   └── ...
│   ├── world-tools/               # World-specific tools
│   │   ├── minecraft/
│   │   ├── fs/
│   │   ├── osworld/
│   │   └── scienceworld/
│   ├── sensors/                   # Sensor implementations (browser-visits, rss-watcher)
│   ├── static/ui/                 # Activation Field frontend (HTML/JS/CSS)
│   ├── scripts/                   # Shell scripts for run-script tool
│   ├── utils/                     # Shared utilities
│   │   ├── llm_api.py
│   │   ├── OpenAIClient.py
│   │   ├── OpenRouterClient.py
│   │   ├── tool_loader.py
│   │   └── ...
│   ├── goals/                     # Goal YAML definitions for testing
│   ├── saved_plans/               # Saved plan templates
│   └── logs/                      # Runtime logs
└── tests/                         # Test files
```

## Next

- [Getting Started](getting-started.md) — first run walkthrough
- [Architecture](architecture.md) — how the components fit together
- [Tools & Primitives](tools-and-primitives.md) — tool catalog reference
