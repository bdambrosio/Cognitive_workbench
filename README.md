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

### 1. Incremental Planning with SGLang
The **Incremental Planner** implements **interleaved reasoning** where the agent:
- **Plans** incrementally, a few steps at a time
- **Executes** tools during planning to gather information
- **Adapts** the plan based on execution results
- **Requests** additional tools dynamically as needs emerge
- All within SGLang's structured generation framework for efficient inference

This creates a tight loop between thought and action, allowing agents to handle complex, multi-stage research tasks that require iterative information gathering and synthesis.

*See: `src/incremental_planner.py`*

### 2. Entity Modeling & Theory of Mind (ToM)
Agents build mental models of their interlocutors:
- Tracks every interaction indexed by other character (default: "User")
- Maintains distinct **discourse states** for each relationship
- Models **Theory of Mind (ToM)** — what they know, want, and intend
- Consolidates conversation history to maintain long-term coherence

*See: `src/memory.py`, `src/entity_model.py`, `src/discourse.py`*

### 3. Information Space ("Infospace")
Information is treated as a spatial environment where:
- **Notes** and **Collections** are primitive persistent objects
- **Cognitive Operations** (search, index, summarize, relate, filter, map) are "spatial" actions
- **Tools** are dynamically loaded Python/LLM skills executed within this space
- All operations use vector embeddings for semantic search and organization

*See: `src/infospace_executor.py`, `src/infospace_resource_manager.py`*

---

## 🏗️ Architecture (Current - Nov 2025)

The system is 100% **Infospace-only** (physical world support removed). It uses **Zenoh** for inter-process communication and **SGLang** for LLM inference.

### Core Components

| Component | Responsibility |
|-----------|----------------|
| **Executive Node** | OODA loop, planning, goal management, memory coordination |
| **Incremental Planner** | SGLang-based iterative planning with tool execution feedback |
| **Infospace Executor** | Executes primitives (create, load, search, etc.) and tools (Python/LLM skills) |
| **Resource Manager** | Manages Notes/Collections, vector indexing, semantic search |
| **Memory Module** | Entity models, discourse tracking, conversation history (no RAG) |

### Removed Components (Nov 2025)
- ~~Memory Node~~ → Integrated into Executive Node as `memory.py` module
- ~~Map Node~~ → Functionality moved to `InfospaceResourceManager`
- ~~Perception Node~~ → Refactored to `action_post_processing.py` utility
- ~~Semantic Validator~~ → Validation now handled by Incremental Planner
- ~~Physical World~~ → System is 100% infospace-only
- ~~CharacterRAGStore~~ → Removed (unused)

### Technology Stack
- **Communication**: [Eclipse Zenoh](https://zenoh.io/) (Python implementation)
- **LLM Backend**: [SGLang](https://github.com/sgl-project/sglang) (primary), OpenAI/vLLM (fallback)
- **Planning**: SGLang-based incremental planner with constrained generation
- **Vector Store**: `txtai` with `sentence-transformers` embeddings
- **Tools**: 32 dynamically loaded skills in `src/tools/`

---

## 🎯 Example: Complex Research Task

**Goal:** "I need to understand the debate around emergent capabilities in large models. Find arguments on both sides, assess their evidence quality, and tell me what questions remain unresolved."

**System Response:** 16-step incremental plan executing over ~60 seconds:

```
Step 0: query-web → Searched web for "emergent capabilities large language models debate"
  → Found 6 sources (arXiv papers, policy articles, technical blogs)

Step 1: summarize → Summarized web results (5221 tokens → 1060 tokens)
  → Identified key themes: sudden vs. gradual scaling, metric artifacts, policy implications

Step 2: semantic-scholar → Searched academic literature
  → Found 9 papers including "Emergent Abilities of Large Language Models" (Wei et al.)

Step 3: summarize → Summarized academic papers
  → Extracted core arguments about emergence mechanisms

Step 4: extract-entities → Extracted key researchers, concepts, institutions
  → Identified: Jason Wei, Rylan Schaeffer, BIG-Bench, scaling laws

Step 7: map → Applied extract-struct to all 9 papers
  → Created structured metadata (titles, years, methodologies)

Step 8: filter-collection → Filtered for papers addressing core debate
  → All 9 papers relevant (broad search needed)

Step 13: query-web → Searched for pro-emergence papers
  → Found 6 papers including foundational Wei et al. 2022

Step 14: query-web → Searched for anti-emergence papers
  → Found 4 papers including "Are Emergent Abilities a Mirage?" (Schaeffer et al.)

Step 15: create-note → Synthesized final analysis
  → Structured report with arguments for/against, evidence quality, open questions
```

**Final Output (Note_367):**
```markdown
## Analysis of the Emergent Capabilities Debate in Large Language Models

### Arguments for Emergence:
1. Capabilities appear suddenly at scale rather than building linearly
2. Some abilities only manifest in very large models (like reasoning)
3. These capabilities aren't simply extrapolations from smaller models
4. Complex emergent behaviors are often surprising to developers
5. Qualitative differences in performance across model sizes

### Arguments Against Emergence:
1. All capabilities can be attributed to scaling laws and training
2. No true 'new' abilities - just better extrapolation of existing patterns
3. Emergent behavior might be explained by improved training methods
4. Claims of emergence may reflect imagination rather than genuine new capabilities
5. The phenomenon can be fully understood through current theoretical frameworks

### Evidence Quality:
- **Strong evidence** for scaling relationships and performance improvements
- **Mixed evidence** for genuine emergence vs. extrapolation
- **Methodological differences** between empirical and theoretical approaches
- **Peer review varies** significantly in quality and depth of analysis
- **Replication concerns** in some empirical studies

### Remaining Questions:
1. How do we distinguish genuine emergence from improved scaling?
2. What constitutes "emergence" in ML context?
3. How should we evaluate evidence quality for emergence claims?
4. Are our theoretical frameworks adequate for studying underlying mechanisms?
5. What are the implications for general intelligence research?

### Conclusion:
The debate remains unresolved due to methodological disagreements and unclear 
definitions. The core issue is whether observed capabilities represent a 
qualitative leap beyond scaling or just the amplification of existing patterns.
```

**Key Capabilities Demonstrated:**
- Multi-source information gathering (web + academic)
- Iterative refinement based on results
- Tool composition (summarize → extract-entities → filter)
- Evidence quality assessment
- Synthesis of conflicting viewpoints

---

## 🚀 Setting Up the Lab

### Prerequisites
- Python 3.10+
- GPU recommended for SGLang (CPU fallback available)

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
Edit `scenarios/jill.yaml`:
- Set `llm_config` for fallback LLM (OpenAI/vLLM)
- Set `sgl_model_path` for SGLang runtime (e.g., `/path/to/Qwen2.5-32B-Instruct`)
- Configure `is_infospace: true` (required)

### Running an Experiment
```bash
cd src
python3 launcher.py ../scenarios/jill.yaml --ui
```

Access the UI at `http://localhost:3000`.

---

## 📂 Repository Structure

```
src/
├── executive_node.py           # Main OODA loop, SGLang runtime initialization
├── incremental_planner.py      # SGLang-based iterative planning
├── infospace_executor.py       # Primitive & tool execution (4191 lines)
├── infospace_resource_manager.py  # Note/Collection management, vector search
├── memory.py                   # Entity model & discourse wrapper
├── entity_model.py            # Theory of Mind tracking
├── discourse.py               # Conversation analysis
├── unified_planner.py         # Template-based planning (legacy)
├── utils/
│   ├── action_post_processing.py  # Expectation comparison
│   └── llm_api.py             # LLM client abstraction
└── tools/                     # 32 dynamically loaded skills
    ├── query-web/             # Web search & extraction
    ├── semantic-scholar/      # Academic paper search
    ├── summarize/             # Text summarization
    ├── filter-collection/     # Semantic filtering
    ├── relate/                # Relationship analysis
    └── ...

scenarios/                     # Agent configuration YAML files
tests/                        # Evaluation framework
Docs/                         # Design documents (may be outdated)
```

---

## 🛠️ Available Tools (32)

**Information Gathering:**
- `query-web` - Web search with LLM-based extraction
- `semantic-scholar` - Academic paper search
- `search-notes` - Semantic search over existing Notes
- `search-collections` - Search Collections

**Transformation:**
- `summarize` - Hierarchical text summarization
- `filter-collection` - Semantic filtering with predicates
- `map` - Apply operation to each item in Collection
- `expand` - Split Note into Collection (lines/JSON/etc.)
- `flatten` - Merge Collection items into single Note
- `relate` - Compare/relate multiple items

**Analysis:**
- `extract-entities` - Named entity extraction
- `extract-struct` - Structured data extraction
- `assess` - Quality/relevance assessment
- `as-json` - Parse/validate JSON content

**Generation:**
- `generate-note` - LLM-generated content
- `refine` - Iterative content refinement
- `create-note` - Create Note with literal content
- `create-collection` - Create Collection from items

**Primitives (built-in):**
- `load`, `save`, `delete`, `display`, `think`, `ask`, `tell`, `index`

---

## ⚠️ Current Limitations

1. **No physical world simulation** (removed Nov 2025)
2. **Single-agent focus** (multi-agent WIP)
3. **Memory persistence** is simple JSON (no graph DB)
4. **UI is minimal** (research tool, not production)
5. **Documentation lags code** by design (code = truth)

---

## 🔬 Research Notes

### Recent Architectural Changes (Nov 2025)

**Simplification:** Removed 4 major components (Memory Node, Map Node, Perception Node, Semantic Validator) by:
- Consolidating memory into `executive_node` as simple module
- Moving indexing/search to `InfospaceResourceManager` (direct method calls)
- Converting validation to inline checks in incremental planner
- Removing physical world entirely (100% infospace)

**Benefits:**
- 40% reduction in inter-process communication
- Simpler debugging (fewer nodes)
- Direct method calls vs. Zenoh queries
- SGLang as primary LLM backend (faster inference)

**Trade-offs:**
- Less modularity (acceptable for research)
- Harder to scale to multi-agent scenarios
- Executive node is larger (~2400 lines)

---

## 📊 Performance Characteristics

**Typical Complex Task (16 steps):**
- Total time: ~60 seconds
- Web searches: 2-4 seconds each
- Semantic Scholar: 1 second
- Summarization: 1-8 seconds (depends on input size)
- Vector indexing: 300-500 embeddings/sec (txtai + sentence-transformers)
- SGLang inference: Context-dependent (batch size 1, GPU)

**Resource Usage:**
- Memory: ~2-4GB (without SGLang)
- SGLang: +8-16GB GPU memory (model-dependent)
- Disk: ~50MB per agent (Note/Collection persistence)

---

## 📝 Citation

If you use this workbench for your research, please cite:

```bibtex
@software{cognitive_workbench,
  author = {Bruce D'Ambrosio},
  title = {Cognitive Workbench: A Framework for LLM-Powered Cognitive Agents},
  year = {2024-2025},
  url = {https://github.com/bdambrosio/Cognitive_workbench}
}
```

---

## 🤝 Contributing

This is a research laboratory. Contributions are welcome, but expect frequent breaking changes. The best way to contribute:

1. **Experiment** with the code
2. **Document** your findings (even if informal)
3. **Share** interesting results or failure modes
4. **Propose** architectural changes with rationale

Code quality standards are intentionally relaxed to prioritize research velocity.

---

## 📄 License

MIT License - See LICENSE file for details.

**Disclaimer:** This is experimental research software. It may break, change direction, or be completely rewritten. Use at your own risk.
