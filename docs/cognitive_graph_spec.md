# Cognitive Memory Graph — Implementation Specification

## 1. Purpose

Replace Jill's `_ooda_living_state` snapshot and rolling `_ooda_event_feed` buffer with a persistent semantic graph that records cognitive objects and their provenance relationships as the OODA loop and planner execute. The graph serves as the substrate for context assembly: when Jill needs to build an LLM prompt, she queries the graph for relevant subgraphs rather than maintaining ad-hoc state variables.

## 2. Architecture Overview

The system has three components:

1. **CognitiveGraph** — the graph store (nodes, typed edges, attributes)
2. **SemanticIndex** — FAISS index over node content for similarity retrieval
3. **GraphRenderer** — serializes subgraphs into text suitable for LLM context windows

The graph accumulates across the lifetime of a session. It is never wiped. Old nodes are consolidated during idle ticks (see §8).

## 3. Node Specification

Every node has:

| Field | Type | Description |
|-------|------|-------------|
| `node_id` | str | Auto-generated unique ID (e.g., `n_0001`) |
| `type` | str | One of the defined node types (see §3.1) |
| `content` | str | Natural-language summary for FAISS indexing (see §3.2) |
| `attrs` | dict | Type-specific structured attributes (see §3.3) |
| `ts` | float | Unix timestamp of creation |
| `session_id` | str | Identifier for the current runtime session |

### 3.1 Node Types

| Type | Created by | Content is | Key attrs |
|------|-----------|-----------|-----------|
| `event` | `_ooda_observe` | The event text/sensor content | `event_type`, `classification`, `source` |
| `assessment` | `_ooda_orient` | `overall_rationale` from assessment | `concern_bumps` (dict: concern_id → level), `action_choice`, `salience` (dict: novelty, persistence, etc.) |
| `decision` | `_ooda_decide` | Human-readable: "Decided: chat_response to User" | `action_type`, `payload_summary` |
| `action_result` | `_ooda_act` | Brief outcome description | `action_type`, `success` (bool) |
| `concern_change` | `_update_character_concern_activations` | "Concern 'attend_to_user' bumped to 0.72 (was 0.55)" | `concern_id`, `concern_label`, `old_activation`, `new_activation`, `trigger` (bump level) |
| `triage_nomination` | `_triage_orient_nominations` or `_triage_idle_tick` | "Concern X nominated for triage: activation 0.8, trend rising" | `concern_id`, `concern_label`, `activation`, `trend`, `nomination_source` ("orient" or "idle") |
| `triage_decision` | `_handle_triage_decisions` | The triage reasoning text | `concern_id`, `action` ("create_task", "attach_to_task", "defer", "dismiss"), `task_intention` (if create) |
| `goal_launch` | `_handle_goal_proceed` / `_ooda_act` dispatch | The goal text (immutable — not re-embedded on status change) | `goal_id`, `source` ("user", "scheduler", "triage", "sensor_trigger"), `concern_id` (if concern-originated), `status` ("active", "completed", "failed") — attrs only, updated via `update_attrs` on completion |
| `goal_outcome` | `_set_scheduled_goal_result` | Result summary (first 500 chars of last_result) | `goal_id`, `success` (bool), `primary_product`, `quality_status`, `verification_answer` |
| `task_created` | `_create_proposed_task` / `_begin_task_establishment` | Task intention text | `task_wip_id`, `linked_concern_id`, `phase` |
| `task_milestone` | `_set_scheduled_goal_result` (when task_wip_id present) | Milestone goal_text + result_summary | `task_wip_id`, `phase`, `milestone_status` |
| `task_cycle` | `_complete_task_cycle` | Cycle summary | `task_note_name`, `goals_count`, `goals_achieved`, `cycle_number` |
| `conversation_turn` | `_ooda_act` (chat_response), `_handle_chat_response`, sense_data_callback | The message text | `source`, `direction` ("in" or "out"), `entity` |
| `concern_created` | `_derived_concern_model.update_from_event` | Concern label + description | `concern_id`, `concern_label`, `origin`, `weight` |
| `concern_status_change` | `_apply_concern_recommendation`, `_cmd_concern_manage`, etc. | "Concern X satisfied: reason" | `concern_id`, `old_status`, `new_status`, `rationale` |
| `situation_update` | `_update_situation_note` | The learnings section text | `goal_text`, `outcome_status` |
| `consolidation` | Consolidation process (§8) | Summary of consolidated subgraph | `time_range_start`, `time_range_end`, `node_count_consolidated`, `node_types_summary` |

### 3.2 Content Field Rules

The `content` field is what gets embedded in FAISS. It must be:

- **Self-contained**: understandable without reading attrs. A human or LLM should grasp what happened from content alone.
- **Natural language**: not JSON, not key=value pairs. A sentence or short paragraph.
- **Concise**: typically 1-3 sentences. Never more than 500 characters.
- **Semantically meaningful**: captures *what* happened, not just that something happened.

Examples:
- Event: `"User asked about the Gaggia Classic E24 brass boiler specifications"`
- Assessment: `"User has a practical purchasing concern about espresso machines; strong relevance to attend_to_user, weak to homeostasis"`
- Goal outcome: `"Research goal completed successfully; produced comparison note of three Gaggia Classic generations with boiler material details"`
- Concern change: `"Concern 'espresso-research' activation increased from 0.3 to 0.65 after user's second question about boiler specs"`

### 3.3 Attrs Field

Attrs is a flat dict of type-specific structured data. Values should be JSON-serializable primitives (str, int, float, bool) or simple dicts/lists of primitives. Attrs enable structured queries like "find all nodes where concern_id = X" without parsing content text.

Attrs are NOT embedded in FAISS. They are for graph traversal and filtering only.

## 4. Edge Specification

Every edge has:

| Field | Type | Description |
|-------|------|-------------|
| `edge_id` | str | Auto-generated unique ID |
| `source` | str | Source node_id |
| `target` | str | Target node_id |
| `type` | str | One of the defined edge types (see §4.1) |
| `ts` | float | Unix timestamp of creation |

### 4.1 Edge Types

These correspond to code paths in the executive node. The naming convention is `verb` describing the relationship from source to target.

| Edge Type | Source Node Type | Target Node Type | Created at |
|-----------|-----------------|-----------------|------------|
| `observed` | `event` | `assessment` | After `_ooda_orient` returns |
| `decided_from` | `assessment` | `decision` | After `_ooda_decide` returns |
| `executed` | `decision` | `action_result` | After `_ooda_act` completes |
| `bumped` | `assessment` | `concern_change` | Inside `_update_character_concern_activations` |
| `nominated` | `concern_change` | `triage_nomination` | Inside `_triage_orient_nominations` or `_triage_idle_tick` |
| `triaged` | `triage_nomination` | `triage_decision` | Inside `_handle_triage_decisions` |
| `spawned_task` | `triage_decision` | `task_created` | When triage creates a proposed task |
| `spawned_goal` | `decision` | `goal_launch` | When act dispatches a goal |
| `spawned_goal` | `triage_decision` | `goal_launch` | When a triage-created task's milestone launches |
| `produced` | `goal_launch` | `goal_outcome` | Inside `_set_scheduled_goal_result` |
| `updated_concern` | `goal_outcome` | `concern_change` or `concern_status_change` | After `update_from_goal_completion` |
| `milestone_of` | `task_milestone` | `goal_launch` | Inside `_set_scheduled_goal_result` (task path) |
| `cycle_of` | `task_cycle` | `task_created` | Inside `_complete_task_cycle` |
| `triggered_by` | `conversation_turn` | `event` | When an incoming message creates an event |
| `replied_to` | `conversation_turn` (out) | `conversation_turn` (in) | When Jill responds to a user turn |
| `concern_for` | `concern_created` | `event` or `assessment` | When a new concern is created from an event (in `_trigger_concern_from_event`) |
| `consolidated_from` | `consolidation` | (multiple via attrs) | See §8 |

### 4.2 Edge Attrs

Edges generally do not carry attrs. The edge type plus source/target is sufficient. Exception: `consolidated_from` edges carry `original_node_ids` as an attr (list of node_ids that were summarized).

## 5. API

```python
class CognitiveGraph:

    def add_node(self, type: str, content: str, attrs: dict = None, ts: float = None) -> str:
        """Add a node. Returns node_id. Automatically indexes content in FAISS."""

    def add_edge(self, source: str, target: str, type: str, ts: float = None) -> str:
        """Add a directed edge. Returns edge_id."""

    def get_node(self, node_id: str) -> dict:
        """Return node dict: {node_id, type, content, attrs, ts, session_id}"""

    def update_attrs(self, node_id: str, attrs: dict) -> None:
        """Merge attrs into existing node attrs. Existing keys are overwritten,
           new keys are added. Use for mutable state like goal_launch status."""

    def get_edges_from(self, node_id: str, edge_type: str = None) -> list[dict]:
        """Return outgoing edges from node, optionally filtered by type."""

    def get_edges_to(self, node_id: str, edge_type: str = None) -> list[dict]:
        """Return incoming edges to node, optionally filtered by type."""

    def query_nodes(self, type: str = None, attrs_filter: dict = None,
                    since: float = None, until: float = None,
                    limit: int = 100) -> list[dict]:
        """Return nodes matching criteria. attrs_filter does exact match on attrs keys.
           Example: query_nodes(type="concern_change", attrs_filter={"concern_id": "attend_to_user"})"""

    def semantic_search(self, query_text: str, k: int = 10,
                        type_filter: str = None) -> list[tuple[str, float]]:
        """Return (node_id, distance) pairs for k nearest nodes by FAISS similarity.
           Optional type_filter restricts results to a single node type."""

    def expand_subgraph(self, seed_node_ids: list[str], max_hops: int = 2,
                        edge_types: list[str] = None) -> tuple[list[dict], list[dict]]:
        """BFS expansion from seed nodes along edges (optionally filtered by type).
           Returns (nodes, edges) comprising the subgraph."""

    def latest_per_key(self, type: str, key_attr: str) -> list[dict]:
        """Return the most recent node of given type for each distinct value of key_attr.
           Example: latest_per_key("concern_change", "concern_id") returns the latest
           activation snapshot for each concern."""

    def prune_before(self, ts: float, exclude_types: list[str] = None) -> int:
        """Delete nodes (and their edges) older than ts. Nodes of exclude_types are kept.
           Returns count of deleted nodes. Used by consolidation (§8)."""

    def node_count(self) -> int:
        """Total number of nodes in the graph."""

    def save(self, path: str):
        """Persist graph + FAISS index to disk."""

    def load(self, path: str):
        """Load graph + FAISS index from disk."""
```

## 6. Storage Backend

Use in-memory dicts for the graph structure:
- `nodes: dict[str, dict]` — node_id → node data
- `edges: dict[str, dict]` — edge_id → edge data
- `edges_from: dict[str, list[str]]` — node_id → list of outgoing edge_ids
- `edges_to: dict[str, list[str]]` — node_id → list of incoming edge_ids

FAISS index: use `faiss.IndexIDMap(faiss.IndexFlatIP(dim))` so that each vector is stored with its integer ID (derived from a monotonic counter). This allows targeted removal via `remove_ids()` during consolidation without a full rebuild. Map between integer FAISS IDs and string node_ids via two dicts: `_faiss_id_to_node: dict[int, str]` and `_node_to_faiss_id: dict[str, int]`.

Embedding model: reuse the existing `BAAI/bge-small-en-v1.5` model already loaded by `infospace_resource_manager.py` (`_init_embedder` / `_generate_embedding`). 384-dim, CPU-only. Do **not** introduce a second embedding model — share the embedder instance to avoid double memory and ensure vector compatibility with existing infospace indexes.

Persistence: save as a single JSON file (graph structure) + a FAISS index file. Load on startup. Save periodically during idle ticks (same cadence as `maybe_persist`).

## 7. Integration Points in Executive Node

The following methods in `ZenohExecutiveNode` emit nodes and edges. Each integration is a small addition (2-5 lines) at the indicated location.

### 7.1 OODA Cycle (`_main_loop_tick` path)

**`_ooda_observe`** — after returning a non-None EventPacket:
```python
event_node = graph.add_node("event", event.content,
    attrs={"event_type": event.event_type, "classification": event.classification, "source": event.source})
```
Stash `event_node` on the EventPacket or pass through.

**`_ooda_orient`** — after evaluation completes:
```python
assessment_node = graph.add_node("assessment", assessment.get("overall_rationale", ""),
    attrs={"concern_bumps": bumps_dict, "action_choice": action_choice, "salience": salience_dict})
graph.add_edge(event_node, assessment_node, "observed")
```

**`_update_character_concern_activations`** — for each concern that changes by more than a threshold (e.g., > 0.05):
```python
change_node = graph.add_node("concern_change",
    f"Concern '{cid}' activation {old:.2f} → {new:.2f} ({level} bump)",
    attrs={"concern_id": cid, "concern_label": label, "old_activation": old, "new_activation": new, "trigger": level})
graph.add_edge(assessment_node, change_node, "bumped")
```

**`_ooda_decide`** — after returning Action:
```python
decision_node = graph.add_node("decision",
    f"Decided: {action.type}" + (f" — {action.payload.get('text', '')[:100]}" if action.payload else ""),
    attrs={"action_type": action.type, "payload_summary": str(action.payload)[:200]})
graph.add_edge(assessment_node, decision_node, "decided_from")
```

**`_ooda_act`** — after execution (varies by action type). For goal dispatch:
```python
goal_node = graph.add_node("goal_launch", goal_text,
    attrs={"goal_id": goal_id, "source": source, "status": "active"})
graph.add_edge(decision_node, goal_node, "spawned_goal")
self._graph_node_by_key[f"goal:{goal_id}"] = goal_node
```
For chat_response:
```python
turn_out = graph.add_node("conversation_turn", response_text,
    attrs={"source": self.character_name, "direction": "out", "entity": target_entity})
graph.add_edge(decision_node, turn_out, "executed")
```

### 7.2 Triage Path

**`_triage_orient_nominations`** — when nominating:
```python
nom_node = graph.add_node("triage_nomination",
    f"Concern '{label}' nominated: activation={activation}, bump={bump_level}",
    attrs={"concern_id": cid, "concern_label": label, "activation": activation, "nomination_source": "orient"})
# Link to the concern_change that triggered it (if we have the node_id)
graph.add_edge(concern_change_node, nom_node, "nominated")
```

**`_handle_triage_decisions`** — for each decision:
```python
td_node = graph.add_node("triage_decision", d.reason,
    attrs={"concern_id": d.concern_id, "action": d.action, "task_intention": d.task_intention or ""})
graph.add_edge(nom_node, td_node, "triaged")
```

### 7.2.1 Derived Concern Creation

New derived concerns are created in `_trigger_concern_from_event` (called from the orient/decide path), **not** from triage. Triage creates tasks, not concerns directly.

```python
concern_node = graph.add_node("concern_created", f"{label}: {description}",
    attrs={"concern_id": cid, "concern_label": label, "origin": origin, "weight": weight})
self._graph_node_by_key[f"concern_change:{cid}"] = concern_node
# Link to the event or assessment that triggered concern creation
graph.add_edge(concern_node, event_or_assessment_node, "concern_for")
```

### 7.3 Goal Lifecycle

**`_set_scheduled_goal_result`** — after goal completes:
```python
outcome_node = graph.add_node("goal_outcome", last_result_raw[:500],
    attrs={"goal_id": goal_id, "success": success, "primary_product": primary_product,
           "quality_status": result.get("quality_status", ""), "verification_answer": result.get("verification_answer", "")})
goal_launch_node = self._graph_node_by_key.get(f"goal:{goal_id}")
if goal_launch_node:
    graph.add_edge(goal_launch_node, outcome_node, "produced")
    graph.update_attrs(goal_launch_node, {"status": "completed" if success else "failed"})
```
After concern model updates:
```python
# If the concern model changed, emit a concern_status_change or concern_change node
# and link: graph.add_edge(outcome_node, concern_change_node, "updated_concern")
```

### 7.4 Task Lifecycle

**`_create_proposed_task`**:
```python
task_node = graph.add_node("task_created", intention,
    attrs={"task_wip_id": wip_id, "linked_concern_id": concern_id, "phase": "proposed"})
# Link to triage decision if available
graph.add_edge(triage_decision_node, task_node, "spawned_task")
```

**Milestone results** (inside `_set_scheduled_goal_result` task path):
```python
ms_node = graph.add_node("task_milestone", f"{goal_text}: {result_summary}",
    attrs={"task_wip_id": task_wip_id, "phase": phase, "milestone_status": status_str})
graph.add_edge(ms_node, goal_launch_node, "milestone_of")
```

### 7.5 Conversation Turns

Incoming user/agent messages (in `_ooda_act` or `sense_data_callback`):
```python
turn_in = graph.add_node("conversation_turn", text,
    attrs={"source": source, "direction": "in", "entity": source})
# conversation_turn triggers the event (message arrives → event created)
graph.add_edge(turn_in, event_node, "triggered_by")
```

### 7.6 Node ID Propagation

The main challenge is threading node IDs through the OODA pipeline so edges can link them. Three approaches (choose one):

**Option A: Thread through existing objects.** Add a `_graph_node_id` attribute to `EventPacket`, `OrientedEvent`, and `Action`. Lightweight but couples graph to data classes.

**Option B: Stash on self.** `self._last_event_node`, `self._last_assessment_node`, `self._last_decision_node`. Simple but not thread-safe (adequate since OODA loop is single-threaded; goal thread uses its own path).

**Option C: Return node IDs from a wrapper.** A `GraphEmitter` class wraps the graph and tracks "current chain" state internally. Call `emitter.observe(event)` → returns node_id and remembers it. `emitter.orient(assessment)` → creates assessment node, auto-links to last observed event.

**Recommendation: Hybrid.** Option B (`self._last_event_node`, `self._last_assessment_node`, `self._last_decision_node`) for the tight OODA chain (single-threaded, same tick). For deferred relationships that span minutes or hours (goal_launch→goal_outcome, task milestones, idle-tick triage), maintain a **lookup dict** `self._graph_node_by_key: dict[str, str]` keyed on domain identifiers:

- `f"goal:{goal_id}"` → goal_launch node_id
- `f"task:{task_wip_id}"` → task_created node_id
- `f"concern_change:{concern_id}"` → most recent concern_change node_id
- `f"triage_nom:{concern_id}"` → most recent triage_nomination node_id

This allows any method that knows a goal_id or concern_id to look up the corresponding graph node without threading node_ids through call signatures. Entries are overwritten as new nodes are created (only the latest mapping matters for edge creation).

## 8. Consolidation

Consolidation compresses old graph regions into summary nodes. It runs during `_ooda_idle_tick` when no goal is running.

### 8.1 Policy

- **Warm → Cold threshold**: nodes older than N hours (configurable, default 4 hours).
- **Exempt from consolidation**: `consolidation` nodes (they're already summaries), `concern_created` nodes (long-lived reference), `task_created` nodes (long-lived reference), `situation_update` nodes.
- **Consolidation granularity**: group old nodes by 1-hour time windows. Each window becomes one `consolidation` node.

### 8.2 Process

1. Query nodes older than threshold, excluding exempt types.
2. Group by 1-hour windows.
3. For each window with > 5 nodes:
   a. Collect all nodes and their edges.
   b. Generate a summary using an LLM call (or a template-based summary if LLM budget is a concern):
      - Count of each node type
      - Which concerns were active and their activation trends
      - Goals launched and their outcomes
      - Key events (highest-salience assessments)
   c. Create a `consolidation` node with the summary as content and `time_range_start`, `time_range_end`, `node_count_consolidated` as attrs.
   d. Prune the original nodes and their edges. Remove corresponding vectors from FAISS via `IndexIDMap.remove_ids()`, and clean up the `_faiss_id_to_node` / `_node_to_faiss_id` mapping dicts for the pruned node IDs.

### 8.3 LLM-Free Consolidation (Default)

To avoid spending LLM tokens on consolidation, use a template:

```
"[{time_range}] {n_events} events, {n_goals} goals ({n_succeeded} succeeded), "
"concerns active: {concern_list}. Key: {highest_salience_content}"
```

To ensure the consolidation node embeds well in FAISS (template text alone is too generic), append the `content` field of the top-3 highest-salience original nodes to the summary, separated by ` | `. This preserves the most semantically meaningful terms from the consolidated window. Truncate the combined content to `max_content_length`.

## 9. Rendering Subgraphs for Context Assembly

`GraphRenderer.render(nodes, edges) -> str` produces text suitable for injection into an LLM prompt.

### 9.1 Format

Render as a linearized narrative with provenance markers:

```
[14:32] Event (sensor:arxiv-monitor): New papers matching 'agent memory architecture'
  → Assessment: Moderate novelty, relevant to knowledge-improvement concern (activation: 0.45 → 0.62)
  → Decided: no_action (below triage threshold)

[14:47] Event (User): "What about using the spare GPU as agent memory?"
  → Assessment: Strong attend_to_user, novel user request
  → Decided: chat_response
  → Goal launched: "Respond to User" [goal_12]
  → Goal completed: success, produced research comparison note
  → Concern 'espresso-research' activation 0.3 → 0.15 (superseded by new topic)
```

### 9.2 Context Assembly Flow

When assembling context for an LLM call (e.g., in `_plan`, `_handle_chat_response`, `_advance_task_wip`):

1. **Seed selection**: semantic_search(query=goal_text, k=20) to find relevant nodes.
2. **Concern weighting**: boost nodes connected to currently-active concerns (via `concern_change` edges).
3. **Subgraph expansion**: expand_subgraph(seed_nodes, max_hops=2) to pull in provenance.
4. **Render**: GraphRenderer.render(subgraph_nodes, subgraph_edges) → text block.
5. **Inject**: include rendered text in the LLM prompt where `_build_agent_state_block` or `_build_process_block` currently go.

### 9.3 Replacing Living State

`render_living_state()` becomes:
```python
def render_living_state(graph):
    concern_states = graph.latest_per_key("concern_change", "concern_id")
    recent_events = graph.query_nodes(type="event", limit=10)  # most recent
    active_goals = graph.query_nodes(type="goal_launch",
        attrs_filter={"status": "active"})  # needs goal status tracking
    return GraphRenderer.render_snapshot(concern_states, recent_events, active_goals)
```

## 10. Thread Safety

- The OODA loop runs on the main thread. All `add_node`/`add_edge` calls from OODA methods are single-threaded.
- Goal execution runs on `goal-worker` thread. Goal lifecycle methods (`_set_scheduled_goal_result`, etc.) run on the main thread (called after `_goal_done_event` is set).
- **`sense_data_callback` runs on Zenoh subscriber threads** but only enqueues to `text_input_queue` / sensor queues. All graph node emission (`conversation_turn`, `event`, etc.) happens on the main thread in `_ooda_observe` / `_ooda_act`, not in the callback itself. This avoids a concurrent-write path.
- Consolidation runs during `_ooda_idle_tick`, which only fires when no goal is running.
- Therefore: **no concurrent writes in practice.** A `threading.Lock` on `add_node`/`add_edge`/`update_attrs`/`prune_before` is still required as a safety net (and for `semantic_search` reads that may come from any thread). The lock should rarely contend.
- FAISS index rebuilds (during consolidation) use a **build-then-swap** pattern: build the new index into a separate object, then atomically replace `self._faiss_index` under the lock. This ensures `semantic_search` never sees a partially-built index.

## 11. Persistence

- Save location: same directory as `resources.json` (the existing resource manager persistence path).
- Files: `cognitive_graph.json` (nodes + edges) and `cognitive_graph.faiss` (FAISS index).
- Save triggers: same as `maybe_persist` in living state — periodic during idle ticks, and at shutdown.
- Load: at startup, before the first `_main_loop_tick`. If files don't exist, start with an empty graph.
- Session boundary: on load, set `self._current_session_id` to a new UUID. New nodes get the new session_id. Old nodes retain their original session_id. This allows "show me what happened in previous sessions" queries.

## 12. Configuration

```python
COGNITIVE_GRAPH_CONFIG = {
    "embedding_model": "BAAI/bge-small-en-v1.5",  # reuse existing infospace model
    "embedding_dim": 384,                      # must match model
    "consolidation_threshold_hours": 4,        # warm → cold after this many hours
    "consolidation_window_hours": 1,           # group consolidated nodes by this window
    "consolidation_min_nodes": 5,              # don't consolidate windows with fewer nodes
    "concern_change_threshold": 0.05,          # minimum activation delta to record a concern_change
    "max_content_length": 500,                 # truncate content field to this length
    "persist_interval_seconds": 300,           # auto-save interval during idle ticks
    "save_dir": None,                          # auto-detect from resource_manager.base_dir
}
```

## 13. Migration Path

This is an additive change. The graph coexists with existing systems during development:

1. **Phase 1**: Implement `CognitiveGraph` class with the API in §5. Unit tests for add/query/expand/prune.
2. **Phase 2**: Add integration points (§7) to executive node. Emit nodes and edges alongside existing behavior. No existing code removed.
3. **Phase 3**: Implement `GraphRenderer` and wire into context assembly (`_build_agent_state_block`, `_build_process_block`). Run both old and new paths, compare.
4. **Phase 4**: Replace `_ooda_living_state` with graph-derived rendering. Remove `_ooda_event_feed` (superseded by graph queries). Remove `maybe_persist` from living state (graph handles its own persistence).

## 14. What This Spec Does NOT Cover

- **Spreading activation from concerns**: the activation-weighting step in context assembly (§9.2 step 2) is described conceptually but the specific algorithm (how concern activation translates to node relevance scores) is left to implementation. A simple approach: multiply FAISS similarity score by (1 + activation_of_connected_concern). More sophisticated approaches can be added later.
- **Planner-level graph nodes**: the planner loop (tool_planner_infospace_vllm) generates its own trace (stages, code blocks, reflections). Whether these become graph nodes is a separate decision. The current spec covers only OODA-loop and executive-level cognitive objects.
- **The reflective language**: this spec implements the "recording" layer. The question of a declarative notation for expressing the OODA/planner loops (the 3-Lisp direction) is a separate research project that builds on top of this infrastructure.
- **GPU memory appliance**: the FAISS index runs on CPU. If the graph grows large enough to benefit from GPU-accelerated search, that's a future optimization using the spare 5060 Ti, but it is not needed for the scale of a single agent's cognitive trace.
