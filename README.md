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
- **Executes** tools **during planning** to progress on goal
- **Adapts** the plan based on execution results  
- **Requests** additional tools dynamically as needs emerge
- All within SGLang's `@function` context for efficient synchronous execution

**Key Innovation:** Tool execution happens **inside** the planning loop, not after. The planner sees actual results (e.g., "Word count: 306") and adapts immediately.

**Result Format:** Tools now return both status and actual values:
```
[SUCCESS] Word count: 306 | word-count completed | Bound: $count to Note_529
```

This tight integration between thought and action enables complex research tasks requiring iterative information gathering and synthesis.

*See: `src/incremental_planner.py`*

<video src="docs/incremental_planning.mp4" controls width="480"></video>


### New Interactive Primitives

**`ask` Primitive:**
- Agent can request information from the user mid-plan
- Synchronous execution: planning suspends until user responds
- UI shows pending question indicator in input area
- Example: `{"type": "ask", "target": "user", "value": "Which paper should I focus on?", "out": "$user_choice"}`

**`think` Primitive (Revised):**
- Internal reasoning appended to SGLang conversation state
- Not displayed externally, but logged for debugging
- Creates actual Notes in infospace for later reference
- Example: `{"type": "think", "value": "Should I verify these claims before proceeding?", "out": "$thought"}`

*See: `src/infospace_executor.py`*

### 2. Information Space ("Infospace")
Information is treated as a spatial environment where:
- **Notes** and **Collections** are primitive persistent objects (Items / Sets metaphor, but set uniqueness is on Note ID, not content, for now)
- **Cognitive Operations** (search, index, summarize, relate, filter, map) are "spatial" actions
- **Tools** are dynamically loaded prompt / Python  skills executed within this space
- All operations use vector embeddings for semantic search and organization

*See: `src/infospace_executor.py`, `src/infospace_resource_manager.py`*

### 3. Entity Modeling & Theory of Mind (ToM) (still there from older multi-agent version, not fully functional right now)
Agents build mental models of their interlocutors:
- Tracks every interaction indexed by other character (default: "User")
- Maintains distinct **discourse states** for each relationship
- Models **Theory of Mind (ToM)** — what they know, want, and intend
- Consolidates conversation history to maintain long-term coherence

*See: `src/memory.py`, `src/entity_model.py`, `src/discourse.py`*

---

## 🏗️ Architecture (Current - Nov 2025)

The system is 100% **Infospace-only** (physical world support removed). It uses **Zenoh** for inter-process communication and **SGLang** for LLM inference.

### Core Components

| Component | Responsibility |
|-----------|----------------|
| **Executive Node** | OODA loop, planning coordination, goal management, memory integration |
| **Incremental Planner** | SGLang-based synchronous planning with in-loop tool execution |
| **Infospace Executor** | Executes primitives (create, load, search, ask, think) and tools |
| **Resource Manager** | Manages Notes/Collections, vector indexing, semantic search |
| **Memory Module** | Entity models, discourse tracking, conversation history |
| **FastAPI UI** | Web interface for monitoring and interaction |

### Recent Removals (Nov 2025)
- ~~Memory Node~~ → Integrated into Executive Node as `memory.py` module
- ~~Map Node~~ → Functionality moved to `InfospaceResourceManager` (direct method calls)
- ~~Perception Node~~ → Refactored to `action_post_processing.py` utility
- ~~Semantic Validator~~ → Validation now handled by Incremental Planner
- ~~Physical World~~ → System is 100% infospace-only
- ~~CharacterRAGStore~~ → Removed (unused)
- ~~UnifiedPlanner~~ → Removed (redundant abstraction layer)
- ~~Activity System~~ → Removed (activity_manager, activity UI tabs, activity.py)
- ~~Turn-based Execution~~ → Continuous OODA loop with direct planning
- ~~Simulation Time~~ → Now uses real `datetime.datetime`

### Technology Stack
- **Communication**: [Eclipse Zenoh](https://zenoh.io/) (Python) - for inter-process pub/sub
- **LLM Backend**: [SGLang](https://github.com/sgl-project/sglang) (required for planning)
- **Planning**: SGLang `@function` with constrained generation, synchronous execution
- **Vector Store**: `txtai` with `sentence-transformers` embeddings
- **Tools**: 32+ dynamically loaded skills in `src/tools/`
- **UI**: FastAPI + WebSockets for real-time monitoring

---

## 🎯 Example: Complex Research Task

**Goal:** "I need to understand the debate around emergent capabilities in large models. Find arguments on both sides, assess their evidence quality, and tell me what questions remain unresolved."

**System Response:** 16-step incremental plan with adaptive reasoning:

```
📥 Jill queued goal → Incremental planner activated

FIRST_TASK: Conduct initial web and academic literature search

Step 0: query-web → [SUCCESS] Bound $web_results to Collection_95 (6 items)
  THOUGHTS: Should summarize to understand key arguments, then proceed to academic papers
  DONE: NO
  NEXT_TASK: Summarize web search results
  REQUEST_TOOLS: ["summarize", "create-collection"]

Step 1: summarize → [SUCCESS] Bound $web_summary to Note_334
  THOUGHTS: Web results summarized. Now need academic papers for detailed arguments
  DONE: NO
  NEXT_TASK: Conduct academic literature search
  REQUEST_TOOLS: ["semantic-scholar"]

Step 2: semantic-scholar → [SUCCESS] Bound $papers to Collection_96 (9 papers)
  THOUGHTS: 9 academic papers found. Should summarize to understand core arguments
  DONE: NO
  NEXT_TASK: Summarize academic papers for key arguments and evidence
  REQUEST_TOOLS: ["summarize"]

Step 3: summarize → [SUCCESS] Bound $paper_summary to Note_344
  THOUGHTS: Papers summarized. Need to examine arguments closely, look for contrasts
  DONE: NO
  NEXT_TASK: Identify key papers representing both sides, focus on evidence quality
  REQUEST_TOOLS: ["create-collection", "relate", "extract-entities"]

Step 4: extract-entities → [SUCCESS] Bound $entities to Note_345
  THOUGHTS: Entities extracted. Create focused collection for core debate papers
  DONE: NO
  NEXT_TASK: Create focused collection, examine contrasting arguments
  REQUEST_TOOLS: ["create-collection", "filter-collection"]

Step 5: create-collection → [SUCCESS] Bound $focused_papers to Collection_97
  THOUGHTS: Collection wrapped in another collection. Need to filter for key papers
  DONE: NO
  NEXT_TASK: Refine to papers explicitly addressing emergence debate
  REQUEST_TOOLS: ["map", "filter-collection", "flatten"]

Step 6: flatten → [SUCCESS] Bound $flattened_papers to Note_346
  THOUGHTS: Need to see actual titles/years. Use map to extract metadata
  DONE: NO
  NEXT_TASK: Extract metadata (titles, years) for targeted filtering
  REQUEST_TOOLS: ["map"]

Step 7: map → [SUCCESS] Bound $paper_metadata to Collection_98 (9 items)
  THOUGHTS: Metadata extracted. Filter for papers with strong emergence claims
  DONE: NO
  NEXT_TASK: Filter for key papers addressing core debate
  REQUEST_TOOLS: ["filter-collection"]

Step 8: filter-collection → [SUCCESS] Bound $debate_papers to Collection_99 (9 items)
  THOUGHTS: All 9 papers discuss emergence. Need different approach for contrasts
  DONE: NO
  NEXT_TASK: Focus on papers with arguments for vs. against emergence
  REQUEST_TOOLS: ["display"]

Step 9: display → [SUCCESS] display completed
  THOUGHTS: Display didn't show full papers. Analyze structure, extract arguments
  DONE: NO
  NEXT_TASK: Analyze key paper to understand debate structure
  REQUEST_TOOLS: ["load", "summarize", "relate", "create-note"]

Step 10: load → [ERROR] load failed: No response for Collection_96
  THOUGHTS: Can't load collection directly. Return to web results for details
  DONE: NO
  NEXT_TASK: Use web results for papers addressing for/against arguments
  REQUEST_TOOLS: ["display", "create-collection"]

Step 11: display → [SUCCESS] display completed
  THOUGHTS: Need systematic approach. Synthesize current understanding now
  DONE: NO
  NEXT_TASK: Synthesize understanding focusing on arguments and evidence quality
  REQUEST_TOOLS: ["create-note", "summarize"]

Step 12: create-note → [SUCCESS] Bound $synthesis to Note_356
  THOUGHTS: Synthesized key arguments. Should examine strong pro/con paper
  DONE: NO
  NEXT_TASK: Select paper representing emergence or non-emergence perspective
  REQUEST_TOOLS: ["load", "query-web"]

Step 13: query-web → [SUCCESS] Bound $emergence_papers to Collection_100 (6 items)
  THOUGHTS: Found pro-emergence papers. Now search anti-emergence for comparison
  DONE: NO
  NEXT_TASK: Search papers arguing against emergence
  REQUEST_TOOLS: ["query-web"]

Step 14: query-web → [SUCCESS] Bound $anti_emergence_papers to Collection_101 (4 items)
  THOUGHTS: Have both sides. Summarize understanding of debate and evidence quality
  DONE: NO
  NEXT_TASK: Create comprehensive summary with arguments, evidence, open questions
  REQUEST_TOOLS: ["summarize", "create-note"]

Step 15: create-note → [SUCCESS] Bound $final_analysis to Note_367
  THOUGHTS: Completed analysis of debate from both sides with evidence assessment
  DONE: YES ✓
  NEXT_TASK: Present final analysis to user
  REQUEST_TOOLS: ["display"]
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

**Minimal Configuration:**
```yaml
llm_config:
  sgl_model_path: "/path/to/model"  # Required - path to your SGLang model

characters:
  Jill:
    character: |
      Character description...
    drives:
      - Goal 1
      - Goal 2
```

**SGLang Model Path:** This is the only required LLM configuration. Point it to a local model compatible with SGLang (e.g., Qwen, Llama, etc.).

**Removed Configurations:**
- `server_name`, `model_name` - No longer used (SGLang only)
- Activity ontology files (`*-activity-ontology.json`)
- Activities list files (`*-activities.json`)
- Simulation time configuration

### Running an Experiment
```bash
cd src
python3 launcher.py ../scenarios/jill.yaml --ui
```
Startup can take a while, it loads llm, be patient...

Access the UI at `http://localhost:3000`.

---

## 📂 Repository Structure

```
src/
├── executive_node.py           # Main OODA loop, SGLang runtime, planning coordination
├── incremental_planner.py      # SGLang-based synchronous planning with tool execution
├── infospace_executor.py       # Primitive & tool execution engine
├── infospace_resource_manager.py  # Note/Collection persistence, vector search
├── memory.py                   # Entity model & discourse integration
├── entity_model.py            # Theory of Mind tracking per entity
├── discourse.py               # Conversation state analysis
├── fastapi_action_display.py  # Web UI server (FastAPI + WebSockets)
├── launcher.py                # Process orchestration
├── templates.py               # LLM prompt templates
├── utils/
│   ├── action_post_processing.py  # Result validation utilities
│   └── llm_api.py             # LLM client wrapper (legacy)
└── tools/                     # 32+ dynamically loaded skills
    ├── query-web/             # Web search & extraction
    ├── semantic-scholar/      # Academic paper search
    ├── summarize/             # Text summarization
    ├── filter-collection/     # Semantic filtering
    ├── relate/                # Relationship analysis
    ├── word-count/            # Text metrics
    └── ...

scenarios/                     # Agent configuration YAML files
data/                         # Persistent storage (Notes, Collections, memory)
logs/                         # Execution logs and planner traces
Docs/                         # Design documents (may lag code)

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
- `load` - Load Note or Collection by ID or name
- `save` - Persist Note or Collection to disk
- `delete` - Remove Note or Collection
- `display` - Show content in UI popup
- `think` - Internal reasoning (logged, not executed externally)
- `ask` - Interactive user input (synchronous wait for response)
- `tell` - Send message to another character
- `index` - Build vector index for Collection search

---

## ⚠️ Current Limitations

1. **SGLang Required** - No longer optional; needed for planning
2. **No physical world simulation** (removed Nov 2025)
3. **Single-agent focus** (multi-agent capabilities reduced)
4. **Memory persistence** is simple JSON (no graph DB)
5. **UI is minimal** (research tool, not production-ready)
6. **Documentation lags code** by design (code = truth)

---

## 🚨 Breaking Changes (Nov 2025)

If migrating from an earlier version:

**Removed:**
- Activity system (activity_manager, activity ontology, activity.json files)
- UnifiedPlanner (use IncrementalPlanner directly)
- Simulation time (use real datetime)
- Map node queryables (use resource_manager direct calls)
- Physical world support
- `server_name` and `model_name` in llm_config (use `sgl_model_path` only)

**Changed:**
- Tool result format (now includes actual values)
- `think` primitive now creates Notes (not just logs)
- `ask` primitive works synchronously (polls for response)
- Tools receive `resource_manager` as kwarg
- Startup is faster (planners initialized once)

**Migration Guide:**
1. Update `llm_config` in YAML - only `sgl_model_path` needed
2. Remove activity-related files (`*-activity-ontology.json`, `*-activities.json`)
3. SGLang is now required (install if not present)
4. Tool implementations may need `resource_manager` parameter
5. UI will only show "Plan" tab (Activity tabs removed)

---

## 🔬 Research Notes

### Recent Architectural Changes (Nov 2025)

**Major Simplifications:**

1. **Removed UnifiedPlanner abstraction** (redundant layer)
   - `IncrementalPlanner` and `InfospacePlanner` now initialized directly in `executive_node`
   - Eliminated startup delay from repeated planner instantiation
   - Cleaner architecture with fewer indirection layers

2. **Eliminated Activity System** (activity_manager, activity.py, activity UI tabs)
   - System is now purely goal/plan-based
   - Removed activity ontology, activity selection, activity tracking
   - Simplified UI: only "Plan" tab remains in character panel
   - Removed ~500 lines of dead code

3. **Resource Access Refactoring**
   - Replaced Zenoh queries with direct method calls to `resource_manager`
   - Tools now receive `resource_manager` as kwarg parameter
   - Fixed broken `load`, `generate-note`, `relate`, `fetch-text` tools
   - Eliminated map_node queryables (no longer needed)

4. **Removed Simulation Time**
   - Now uses real `datetime.datetime` throughout
   - Simplified time handling (no time advancement coordination)
   - Removed time subscribers and publishers

5. **Improved Tool Result Reporting**
   - Results now show actual values, not just status
   - Format: `[SUCCESS] <actual_result> | <action> | Bound: <var> to <resource>`
   - 128-char truncation for long results
   - Planner sees both execution status and actual data

**Benefits:**
- 40% reduction in inter-process communication
- Simpler debugging (fewer nodes, direct calls)
- Faster startup (single planner initialization)
- Better planner feedback (sees actual tool results)
- Cleaner codebase (~700 lines of dead code removed)

**Trade-offs:**
- Less modularity (acceptable for research)
- Executive node is larger (~2400 lines)
- SGLang is now required (not optional)

---

## 📊 Performance Characteristics

**Typical Complex Task (16 steps):**
- Total time: ~45-60 seconds (improved with direct method calls)
- Web searches: 2-4 seconds each
- Semantic Scholar: 1 second
- Summarization: 1-8 seconds (depends on input size)
- Vector indexing: 300-500 embeddings/sec (txtai + sentence-transformers)
- SGLang inference: Context-dependent (batch size 1, GPU)
- Startup: ~15-20 seconds (improved with single planner init)

**Resource Usage:**
- Memory: ~2-4GB (without SGLang)
- SGLang: +8-96GB GPU memory (model-dependent)
  - Recommended: Qwen2.5-32B or Qwen3-Coder-30B (16-bit, ~60GB VRAM)
  - Configure in `jill.yaml`: `sgl_model_path: "/path/to/model"`
- Disk: ~50MB per agent (Note/Collection persistence)
- Logs: Planner traces in `logs/planner_trace_{character}.txt` (full conversation state)

**Recent Improvements:**
- 40% reduction in Zenoh query overhead (direct method calls)
- Faster startup (single planner initialization vs repeated instantiation)
- Better planner feedback (actual tool results visible)
- Reduced shutdown time (5s vs 10s)

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
