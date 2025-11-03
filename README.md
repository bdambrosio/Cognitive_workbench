# Cognitive Workbench

**A cognitive architecture for building LLM-powered agents that plan, reason, and operate over information spaces.**

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: Research](https://img.shields.io/badge/license-Research-green.svg)](LICENSE)
[![Status: Active Development](https://img.shields.io/badge/status-active%20development-orange.svg)]()

---
more at [Devin] ([https://app.devin.ai/wiki/bdambrosio/Cognitive_workbench])

## What is Cognitive Workbench?

Cognitive Workbench is a research framework for building autonomous agents with persistent memory, goal-directed planning, and tool use. Agents can operate in both **physical spaces** (simulated 2D worlds) and **information spaces** (semantic operations over notes, collections, and web content).

**Key capabilities:**
- **Autonomous planning**: Agents generate multi-step plans with control flow (conditionals, loops) to achieve goals
- **Information space operations**: Create, search, index, and manipulate structured information (notes, collections)
- **Extensible tool system**: Web search, content summarization, entity extraction, and custom tools
- **Interactive development**: Manual goal setting, plan editing, and step-by-step execution
- **Parallel execution**: Multi-agent scenarios with Zenoh-based communication

Built for researchers exploring cognitive architectures, tool-augmented LLMs, and agent reasoning.

---

## Quick Start

### Prerequisites
- Python 3.10+
- OpenAI API key, or local LLM server (vLLM/Ollama), or OpenRouter account

### Installation

```bash
# Clone repository
git clone https://github.com/bdambrosio/Cognitive_workbench.git
cd Cognitive_workbench

# Create virtual environment
python3 -m venv zenoh_venv
source zenoh_venv/bin/activate  # On Windows: zenoh_venv\Scripts\activate

# Install dependencies
pip install -r src/requirements.txt
```

### Configure LLM

Edit `scenarios/jill.yaml` to set your LLM provider:

```yaml
# For OpenAI
llm_config:
  server_name: "openai"
  model_name: "gpt-4.1"

# For local vLLM
llm_config:
  server_name: "vllm"
  model_name: "llama3.3-70B"
```

Set your API key (if using OpenAI/OpenRouter):
```bash
export OPENAI_API_KEY='your-key-here'
```

### Launch

```bash
cd src
python3 launcher.py ../scenarios/jill.yaml --ui
```

A browser tab will open at `localhost:3000` with the interactive UI. Click **Step** to advance the simulation, or enter goals in the chat interface.

---

## Core Concepts

### Information Space (Infospace)

Agents can operate over **semantic information** rather than just physical actions:

- **Notes**: Persistent text/data objects with metadata
- **Collections**: Ordered sets of Notes/Collections
- **Primitives**: create-note, create-collection, load, persist, index, search, map, flatten, coerce, add, expand, say, display, think, if, while, wait, focus
- **Operations**: index (RAG embeddings), search (semantic), coerce (flatten nested lists), map (apply operation to each item)

Example agent goal: *"Search the web for transformer papers, create a collection, index it, and find papers about attention mechanisms"*

### Planning System

Agents generate **structured JSON plans** with:
- **Primitives**: Core operations (create-note, search, say, display)
- **Tools**: Extensible skills (query-web, summarize, as-json, revise)
- **Control flow**: Conditionals (`if`), loops (`while`), error handling
- **Variables**: Plan-local bindings (`$note`, `$results`)

Plans can be:
- Auto-generated from natural language goals
- Manually edited via `edit:` command
- Stepped through interactively in the UI

### Tool System

Tools are defined as `Skill.md` files with YAML frontmatter:

```yaml
---
name: query-web
type: python
description: Search the web using Google CSE
parameters: query (required)
---
```

Types:
- **prompt_augmentation**: LLM-based tools (summarize, relate, refine, assess, extract-entities)
- **python**: Code execution tools (query-web, render, as-json, as-markdown, revise, text-find, matches, filter-by-predicate)

Built-in tools include: query-web, summarize, relate, refine, assess, extract-entities, render, as-json, as-markdown, revise, text-find, matches, filter-by-predicate, is-empty, is-question, is-positive, word-count, download-pdf, extract-paper-text, extract-struct, save.

**Tool argument conventions**: Most tools use `target` for input data; `query-web` uses `args.query` for the search query.

### Architecture

**Shared Nodes:**
- **FastAPI UI**: Web interface for monitoring and interaction
- **Map Node**: World state, spatial resources, turn management
- **LLM Service** (optional): Centralized LLM API access

**Per-Character Nodes:**
- **Executive Node**: OODA loop, planning, decision-making
- **Memory Node**: Conversation history, entity models, RAG storage
- **Infospace Executor**: Primitive execution, tool invocation

All nodes communicate via **Zenoh** pub/sub and queryables.

---

## Example Usage

### Information Research Assistant

```python
# Launch Jill (configured for infospace)
python3 launcher.py ../scenarios/jill.yaml --ui
```

In the UI chat, enter:
```
goal: search the web for Berkeley weather Oct 26 2025, summarize results, and report to user
```

Jill will:
1. Generate a plan (query-web → summarize → say)
2. Execute each step
3. Display results in the UI

### Interactive Planning

```
goal: create a note about AI safety
edit: make it more detailed and add references
edit: organize into bullet points
```

The `edit:` command uses the full planning template to intelligently modify plans.

### Physical World Simulation

```python
# Launch multi-character scenario
python3 launcher.py ../scenarios/lost.yaml --ui
```

Characters navigate a 2D world, manage physiological states (hunger, thirst), select activities, and interact with each other.

---

## Development Status

**Active features:**
- ✅ Infospace primitives (12+ primitives: create-note, create-collection, load, persist, index, search, map, flatten, coerce, add, expand, say, display, think, if, while, wait, focus)
- ✅ Tool system (20+ built-in tools)
- ✅ Web UI with plan editing, resizable display, collapsible logs
- ✅ Note/Collection viewer
- ✅ Multi-agent parallelism
- ✅ Physical world simulation

**In progress:**
- 🚧 Advanced reflection and learning
- 🚧 Cross-agent information sharing
- 🚧 SLAM for spatial awareness
- 🚧 Better error recovery

**Known limitations:**
- Plan validation can be strict (use `edit:` to iterate)
- World save/load is fragile
- Debugging requires disabling timeouts
- Documentation incomplete (see `Docs/` for details)

---

## Documentation

- [Implementation Status](Docs/IMPLEMENTATION_STATUS.md)
- [Infospace Architecture](Docs/SPACEMAP_ARCHITECTURE.md)
- [Map Node API](Docs/MAP_NODE_API_ANALYSIS.md)
- Research context and claims: See [BACKGROUND.md](BACKGROUND.md)

---

## Contributing

This is a research project in active development. Contributions welcome:

1. **Issues**: Bug reports and feature requests
2. **Pull requests**: Code improvements (test thoroughly!)
3. **Ideas**: Architectural feedback and use cases

**Development notes:**
- Most complexity is in `executive_node.py` (OODA loop), `infospace_executor.py` (primitives), and `map_node.py` (world state)
- Zenoh timeouts can interfere with debugging - set longer timeouts or disable turn management in debug mode
- The UI uses HTMX + vanilla JS (may migrate to React)

---

## Citation

If you use Cognitive Workbench in research, please cite:

```
@software{cognitive_workbench,
  author = {Bruce D'Ambrosio},
  title = {Cognitive Workbench: A Framework for LLM-Powered Cognitive Agents},
  year = {2024-2025},
  url = {https://github.com/bdambrosio/Cognitive_workbench}
}
```

---

## License

Provided as-is for educational and research purposes. See [LICENSE](LICENSE) for details.

---

## Research Context

**Core hypothesis**: LLMs have implicit knowledge of "near-natural" world ontologies from training. By providing a cognitively-inspired architecture (OODA, planning, memory, tools), we can build agents that leverage this knowledge for coherent long-term behavior.

For detailed background and claims, see [BACKGROUND.md](BACKGROUND.md).

---

## Contact

Questions, ideas, or collaboration? Open an issue or reach out via GitHub.

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/bdambrosio/Cognitive_workbench)

