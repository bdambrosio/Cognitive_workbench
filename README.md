# Cognitive Workbench

**Code as Laboratory for LLM Cognitive Architecture Research**

[![Status: Research Laboratory](https://img.shields.io/badge/status-research_laboratory-purple.svg)]()
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

---

## 🧪 Code as Laboratory

**Cognitive Workbench is not a finished product.** It is a research framework—a laboratory—for exploring the frontiers of LLM-based cognitive architecture.

The code here is constantly evolving. It prioritizes **experimental capability** over stability. We view the codebase itself as a "workbench" where we test hypotheses about agency, memory, planning, and theory of mind.

If you are looking for a production-ready agent framework, this is likely not it. If you are looking to experiment with novel cognitive architectures, welcome to the lab.

---

## 🔬 Key Research Frontiers

This workbench currently focuses on three main areas of cognitive agent research:

### 1. Interleaved Reasoning and Tool Use (SGLang)
We are moving beyond simple ReAct loops. The **Incremental Planner** (powered by SGLang) implements **interleaved reasoning**, where the agent:
- **Plans** a few steps ahead using structured, constrained generation.
- **Executes** tools *during* the planning phase to gather information.
- **Reflects** on the results immediately.
- **Refines** the plan dynamically based on execution feedback.
- All within a single SGLang *function*, which means a single continuous evolution of latent state, no recycling of 'context'

This creates a tighter loop between thought and action, allowing agents to handle complex, multi-stage information retrieval tasks that brittle "generate-then-execute" planners fail at.
*See: `src/incremental_planner.py`*

### 2. Entity Modeling & Theory of Mind (ToM)
Agents in this system don't just "chat"; they build mental models of their interlocutors. The **Entity Model** system:
- Tracks every interaction with other agents (or humans).
- Maintains distinct **discourse states** for each relationship.
- explicitly models the **Theory of Mind (ToM)** of the other party—estimating what they know, what they want, and their intent.
- Consolidates conversation history into semantic summaries to maintain long-term coherence without context window overflow.
*See: `src/entity_model.py`, `src/discourse.py`*

### 3. Information Space ("Infospace")
We treat information as a spatial environment. Agents don't just "process data"; they inhabit an **Infospace** where:
- **Notes** and **Collections** are primitive persistent objects in the agent's world (think items and sets).
- **Cognitive Operations** (search, index, summarize, relate) are implemented as "spatial" actions, analogous to moving or picking up objects in a physical world.
- Tools are first-class citizens, discoverable and executable within this space (adapted from Claude Skills: prompt_augmentation, python, and plan skills are currently supported).
*See: `src/infospace.py`, `src/infospace_resource_manager.py`*

---

## 🏗️ Architecture

The system is built as a distributed set of cognitive nodes communicating via **Zenoh**, a high-performance pub/sub protocol. This allows for low-latency communication and easy parallelization.

### Core Nodes
| Node | Responsibility |
|------|----------------|
| **Executive Node** | The brain. Runs the OODA loop (Observe-Orient-Decide-Act), manages the Planner, and drives behavior. |
| **Memory Node** | The hippocampus. Manages RAG (txtai), Entity Models, and short/long-term memory consolidation. |
| **Map Node** | The world server. Maintains the "ground truth" state of the world (Physical or Infospace) and handles resource locking. |
| **Infospace Executor** | The hands. Executes primitives (create, load, persist) and Tools (Python/LLM skills). |

### Technology Stack
- **Communication**: [Eclipse Zenoh](https://zenoh.io/) (Pure Python implementation)
- **Planning**: [SGLang](https://github.com/sgl-project/sglang) (Structured Generation Language)
- **LLM Support**: OpenAI (GPT-4), vLLM (Llama 3, Qwen), OpenRouter.
- **Vector Store**: `sentence-transformers` + `faiss` / `txtai` for local embedding and retrieval.

---

## 🚀 Setting Up the Lab

### Prerequisites
- Python 3.10+
- An adventurous spirit (and willingness to read code).

### Installation
```bash
# Clone the laboratory
git clone https://github.com/bdambrosio/Cognitive_workbench.git
cd Cognitive_workbench

# Initialize environment
python3 -m venv zenoh_venv
source zenoh_venv/bin/activate

# Install dependencies
pip install -r src/requirements.txt
```

### Configuration
Edit `scenarios/jill.yaml` to point to your LLM backend (OpenAI, vLLM, etc.) and the model you want SGLang to load (currently separate).

### Running an Experiment
The primary research agent right now is **Jill**, an Infospace assistant using the Incremental Planner.

```bash
cd src
python3 launcher.py ../scenarios/jill.yaml --ui
```

Access the experimental UI at `http://localhost:3000`.

---

## 📂 Repository Structure

- `src/` - The core Python codebase.
  - `executive_node.py` - Main agent loop.
  - `incremental_planner.py` - SGLang planner logic.
  - `entity_model.py` - Theory of Mind implementation.
  - `infospace_*.py` - Information space physics and tools.
- `scenarios/` - YAML definitions for agents and worlds.
- `Docs/` - Detailed design documents (often out of date, as code moves fast).
- `tests/` - Evaluation framework.

---

## ⚠️ Disclaimer

This software is experimental. Breaking changes happen frequently. Documentation trails implementation. If something looks interesting but doesn't work, check the code—it is the ultimate source of truth.

**Citation:** If you use this workbench for your research, please cite:
```bibtex
@software{cognitive_workbench,
  author = {Bruce D'Ambrosio},
  title = {Cognitive Workbench: A Framework for LLM-Powered Cognitive Agents},
  year = {2024-2025},
  url = {https://github.com/bdambrosio/Cognitive_workbench}
}
```
