# Core Cognitive Architecture

The Cognitive Workbench is built around an **incremental planner** that interleaves reasoning with tool execution. Unlike plan-then-execute architectures, the planner generates one step at a time, executes it, observes the result, and decides what to do next. This tight feedback loop is the central design principle.

## The Cognitive Cycle (OODA Loop)

Each character runs a continuous **Observe-Orient-Decide-Act** loop, coordinated by the Executive Node (`executive_node.py`):

```
                    ┌────────────────────────────────────────┐
                    │          Executive Node (OODA)          │
                    │                                        │
  User input ──────►  OBSERVE   sense data, messages, goals │
  Scheduled goal ──►  ORIENT    character model, context     │
                    │  DECIDE   select goal, invoke planner  │
                    │  ACT      execute plan via executor    │
                    │                │                       │
                    └────────────────┼───────────────────────┘
                                     │
                    ┌────────────────▼───────────────────────┐
                    │       Incremental Planner              │
                    │                                        │
                    │  Stage 0: Retrieve relevant resources  │
                    │  Stage 1: Analyze goal + select tools  │
                    │  Stage 1.5: Load tool skill docs       │
                    │  ┌──────────────────────────────┐      │
                    │  │  Stage 2: Generate code block │◄─┐  │
                    │  │  Execute via Executor         │  │  │
                    │  │  Stage 3: Evaluate result     ├──┘  │
                    │  │  (loop until done or budget)  │     │
                    │  └──────────────────────────────┘      │
                    │  Reflect: update task_state,           │
                    │           world_model, tool_insights   │
                    └────────────────┬───────────────────────┘
                                     │
                    ┌────────────────▼───────────────────────┐
                    │       Infospace Executor               │
                    │                                        │
                    │  Primitives: create-note, search,      │
                    │    map, filter, say, ask, think...     │
                    │  Tools: search-web, run-script,        │
                    │    semantic-scholar, check-email...    │
                    │                                        │
                    │  ┌────────────────────────────────┐    │
                    │  │  Infospace Resource Manager     │    │
                    │  │  Notes + Collections + Relations│    │
                    │  │  FAISS semantic index           │    │
                    │  └────────────────────────────────┘    │
                    └────────────────────────────────────────┘
```

## The Incremental Planner

The planner (`incremental_planner.py`) is the core reasoning engine. It uses an LLM (via SGLang, OpenRouter, or Anthropic) to generate and execute plans one step at a time.

### Planning Stages

**Stage 0 — Resource Retrieval**

Before planning begins, the planner uses FAISS semantic search to retrieve Notes and Collections relevant to the goal. These are injected as context so the LLM can reference existing knowledge.

**Stage 1 — Analysis + Tool Selection**

The LLM receives:
- The goal text
- Character description and capabilities
- Current situation / world state
- The full tool catalog (names + short descriptions)
- Retrieved resources from Stage 0

It produces:
- `<reasoning>`: Analysis of what needs to be done
- `<tools>`: Which tools it will need
- `<first_task>`: The first concrete step

**Stage 1.5 — Skill Doc Injection**

For each tool selected in Stage 1, the planner lazy-loads the full `SKILL.md` documentation and injects it into the conversation. This gives the LLM detailed parameter schemas, examples, and usage patterns without bloating the initial prompt. Tools are tracked so docs aren't re-injected on subsequent iterations.

**Stage 2 — Code Block Generation (Loop)**

The LLM writes a Python code block that calls tool primitives through the executor. For example:

```python
results = exec("search-web", {"query": "recent papers on transformer architectures"})
if results["status"] == "success":
    exec("create-note", {"name": "search_results", "value": results["data"]})
```

The executor runs the code block, captures results, and returns them to the planner.

**Stage 3 — Evaluation + Next Task**

The LLM reflects on the execution result:
- Was the task accomplished?
- Is the overall goal complete?
- What should the next step be?
- Did an error occur that needs recovery?

If there's more work to do, the loop returns to Stage 2 with the next task. This continues until the goal is met or the step budget is exhausted.

### Step Budget

Each plan has a configurable `max_steps` (default 16). This prevents runaway execution. If the budget is exhausted, the planner produces a partial result and marks the goal accordingly.

### Plan Bindings

Variables created during plan execution (e.g., `$search_results`, `$paper_collection`) are stored in a **binding stack**. Bindings persist across plans within a session, allowing later goals to reference artifacts created by earlier ones.

## The Executive Node

The Executive Node (`executive_node.py`) is the central coordinator for each character. It manages:

- **Goal queue**: Goals arrive from user input (`goal:` prefix), scheduled goals, or inter-agent messages
- **OODA loop**: Continuous observe-orient-decide-act cycle
- **Zenoh pub/sub**: Publishes actions to `cognitive/{character}/action`, subscribes to sense data and control channels
- **Goal Scheduler**: Daemon thread that auto-proceeds scheduled goals (see [Goals & Scheduling](goals-and-scheduling.md))
- **Conversation Store**: Tracks dialog history per entity for envisioning (see [Envisioning & QC](envisioning-and-quality-control.md))
- **Interrupt handling**: Goals can be interrupted mid-execution; the planner checks an `interrupt_requested` flag at key checkpoints

### Goal Processing Flow

```
Text input arrives
    │
    ├─ Starts with "goal:" ?
    │   YES → parse_and_set_goal()
    │           ├─ Create Goal object (name, description, termination conditions)
    │           ├─ Publish goal via Zenoh
    │           ├─ Invoke planner: _plan(template, goal)
    │           ├─ Execute plan via infospace_executor
    │           └─ Publish result, persist artifacts
    │
    └─ Regular text → envisioning + dialog handling
```

## Infospace Memory Model

The planner "thinks" through an **Infospace** — a structured memory of Notes, Collections, and Relations managed by the Resource Manager (`infospace_resource_manager.py`).

### Notes

A **Note** is the atomic unit of knowledge. It holds string content (plain text or JSON serialized as text) plus metadata.

- **Content is always a string** — the planner sees text, not structured objects
- **Metadata** is stored in a separate linked Note via a `meta` Relation (not embedded in content)
- Notes can be **transient** (session-only) or **persistent** (saved to disk)

### Collections

A **Collection** is an ordered list of Notes, often produced by search, filtering, or join operations. Collections support set operations (union, intersection, difference) and transformations (map, filter, project, sort).

### Relations

A **Relation** is a typed directed edge between any two resources:
- `meta` — links a content Note to its metadata Note
- `related` — general semantic association
- `supports` — evidential relationship
- Custom types as needed

Relations persist when either endpoint is persistent, and are automatically removed when either endpoint is deleted.

### Semantic Search (FAISS) + Entity-Augmented Retrieval

The Resource Manager maintains FAISS vector indexes over Notes and Collections. An **Entity Index** (`entity_index.py`) supplements embedding search with structured entity lookups. Together, these enable:
- **Stage 0 resource retrieval**: FAISS embedding similarity finds semantically relevant notes; entity augmentation adds notes that share named entities with the goal even when surface-form similarity is low
- **`discover-notes`** / **`discover-collections`** primitives: semantic search during execution
- Automatic re-indexing when resources change

### Named Entity Recognition (NER) Pipeline

The system extracts named entities (people, organizations, locations, topics) from three ingestion points:

- **User input**: entities extracted during OODA observe, linked to conversation_turn graph nodes
- **Goal text**: entities extracted at goal launch, cached for retrieval augmentation
- **Persistent notes**: batch-extracted during idle ticks, tracked via `entities_extracted` flag

Extracted entities are stored in an in-memory index (entity name → set of resource IDs) and emitted as persistent `entity` nodes in the cognitive graph with `mentions` edges linking them to content nodes. An alias map resolves variant names to canonical forms (e.g., "Bruce" → "user").

### Cognitive Graph

A graph of OODA-cycle events, goals, concerns, tasks, entities, and their relationships (`cognitive_graph.py`). Node types include `event`, `conversation_turn`, `assessment`, `decision`, `goal_launch`, `goal_outcome`, `concern_change`, `entity`, `tom_update`, and more. The graph supports:

- **Semantic search**: FAISS-indexed node content
- **Subgraph expansion**: BFS traversal from seed nodes with edge-type filtering
- **Concern-weighted boosting**: prioritize nodes related to active concerns
- **Consolidation**: old nodes compressed into hourly summaries; entity and ToM nodes are exempt

### World Model (Bayesian, Recency-Weighted)

The world model (`world_model.py`) accumulates cross-goal facts with Bayesian confidence estimation. Key features:

- **Recency-weighted evidence**: observations decay with a 30-day half-life — recent evidence weighs more than old
- **Staleness detection**: facts older than 90 days with fewer than 3 observations are flagged as "stale"
- **Generalization guard**: LLM checks whether a fact is reusable across contexts before promotion
- **Tool contracts**: Dirichlet posterior over tool reliability votes (reliable/unreliable/constrained)

### Theory of Mind (ToM)

The system maintains a persistent **Theory of Mind** model for each peer entity (e.g., `_tom_user`). The ToM captures:

- **Trust assessment**: competence, intentionality, reliability, transparency
- **Goals & alignment**: what the entity wants and how aligned it is with the agent
- **Emotional state**: current affect
- **Concerns/uncertainties**: open questions about the entity

ToM updates are triggered by conversation archival (`/done`, `/next`, `/bye`, or shutdown). The update prompt receives the raw conversation transcript plus active user concerns as evidence. Updated ToM is persisted as a named Note and injected into the system prompt for chat and planner contexts. ToM update events are emitted as consolidation-exempt `tom_update` nodes in the cognitive graph.

### Conversation Management

Conversations are tracked by `conversation_store.py` with per-entity dialog management. Key features:

- **Dialog lifecycle**: turns accumulate in a "conversation" Collection until closed by `/done`, `/next`, `/bye`, or shutdown
- **Archival**: on close, turns are synthesized into a summary, added to `conversation_history`, and original turn notes are deleted
- **Prior session backfill**: when the current session has few turns, `get_entity_context()` backfills with prior session summaries (proportional to empty slots, capped at 5)
- **Concern-weighted context**: `get_themed_context()` retrieves conversation history organized by user concern weights

## Reflection and Learning

After each plan execution, the planner runs a **reflection** phase (`_reflect()`) that analyzes the entire execution trace and produces a `ReflectionFrame`.

### ReflectionFrame

The reflection analyst LLM examines the execution trace and produces:

**task_state** — ephemeral state for retrying the same goal:
- `immediate_blockers`: what prevented progress
- `active_hypotheses`: strategies to try next (max 6)
- `proven_safe_paths`: approaches known to work
- `exhausted_search`: what's been tried (to avoid repetition)

**world_model updates** — conservative, reusable cross-goal knowledge:
- Only facts that remain true under different goals and times are promoted
- Excludes: current agent state, one-time observations, trivial details
- Evidence is recency-weighted with a 30-day half-life

**tool_insights** — discovered tool behaviors:
- Tool name, insight text, reliability status (reliable / unreliable / constrained)

**failure_mode** — classification of what went wrong (if anything):
- `none`, `missing_affordance`, `incorrect_context`, `resource_exhaustion`, `tool_limitation_or_misbehavior`, `exhausted_or_misdirected_search`

**quality_status** — overall assessment:
- `passed` / `failed` / `needs_revision` / `interrupted`

### Learnings

A short list of cross-goal learnings maintained in the situation note. Updated after each goal completion via LLM, with a hard cap of 20 entries. Near-duplicates are merged, and a consolidation pass prunes stale entries at shutdown.

See [Envisioning & Quality Control](envisioning-and-quality-control.md) for details on how reflection feeds failure recovery and missing affordance detection.

## Zenoh Communication

The system uses [Eclipse Zenoh](https://zenoh.io/) for inter-process messaging. Key topics:

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `cognitive/{character}/goal` | Publish | Announce current goal |
| `cognitive/{character}/action` | Publish | Report executed actions |
| `cognitive/{character}/sense_data` | Subscribe | Receive user input and environment data |
| `cognitive/{character}/control/*` | Subscribe | Execution control (step/run/stop/interrupt) |
| `cognitive/User/action` | Publish | Log user actions for trace |
| `cognitive/launcher/ready` | Subscribe | Launcher readiness signal |

Zenoh enables multi-character scenarios where agents communicate via pub/sub, and the web UI observes all activity in real time.

## Key Source Files

| File | Role |
|------|------|
| `src/launcher.py` | Entry point; sets up SGLang runtime, launches characters |
| `src/executive_node.py` | OODA loop, goal handling, scheduling, ToM, conversation lifecycle |
| `src/incremental_planner.py` | Multi-stage iterative planner, reflection |
| `src/infospace_executor.py` | Tool execution engine, primitives, entity-augmented search |
| `src/infospace_resource_manager.py` | Notes/Collections/Relations persistence, FAISS indexing |
| `src/entity_index.py` | NER extraction, entity index, cognitive graph entity nodes |
| `src/cognitive_graph.py` | OODA event graph, entity/ToM nodes, semantic search + BFS |
| `src/conversation_store.py` | Dialog persistence, archival, prior-session backfill |
| `src/discourse.py` | Theory of Mind templates and discourse analysis |
| `src/world_model.py` | Bayesian recency-weighted cross-goal knowledge |
| `src/goal_scheduler.py` | Autonomous goal scheduling daemon |
| `src/tool_model.py` | Tool success tracking, embedding-based recommendations |
| `src/fastapi_action_display.py` | Web UI (FastAPI + WebSockets) |

## Design Principles

1. **Incremental over monolithic**: Plans are generated and executed one step at a time with LLM feedback after each step, not generated all at once
2. **Memory is central**: Notes/Collections/Relations are working memory, not just storage for the action system
3. **Conservative reflection**: Only genuinely reusable facts are promoted to the persistent world model
4. **Tool-first extensibility**: New capabilities are added as tools with `Skill.md` + `tool.py`, not by modifying core code
5. **Inspectability**: All plans, actions, reflections, and state transitions are logged and visible in the UI
6. **Interrupt-driven**: Goals can arrive or be cancelled mid-execution; the system handles this gracefully

## Next

- [Getting Started](getting-started.md) — installation and first run
- [Goals & Scheduling](goals-and-scheduling.md) — goal handling and autonomous execution
- [Envisioning & Quality Control](envisioning-and-quality-control.md) — conversational QC and failure recovery
- [Tools & Primitives](tools-and-primitives.md) — the tool system
