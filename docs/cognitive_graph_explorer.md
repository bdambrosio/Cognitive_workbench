# Cognitive Graph Explorer

The cognitive graph explorer is an interactive visualization of the agent's cognitive event graph, accessible via the Resource Browser (default: http://localhost:3001, Graph tab).

## What the Graph Contains

The cognitive graph records the agent's cognitive activity as it processes events through the OODA loop and executes goals. Nodes represent cognitive objects; directed edges represent relationships between them.

### Node Types

| Type | Color | Size | Description |
|------|-------|------|-------------|
| **entity** | Teal | Scales with mention count | Named entities (people, places, organizations, topics) extracted via NER |
| **goal_launch** | Blue | Fixed (10px) | A goal that was dispatched for execution |
| **goal_outcome** | Blue | Fixed (10px) | The result of a completed goal |
| **conversation_turn** | Gray | Fixed (4px) | A user or agent message |
| **concern_created** | Orange | Fixed (8px) | A new concern surfaced by the agent |
| **concern_change** | Orange | Fixed (8px) | A change in concern activation level |
| **assessment** | Pale yellow | Fixed (5px) | Orient-phase evaluation of an event's significance |
| **decision** | Pale yellow | Fixed (5px) | Decide-phase action choice |
| **tom_update** | Purple | Fixed (7px) | Theory-of-mind update about another agent |
| **task_created** | Green | Fixed (8px) | A new task created from triage |
| **consolidation** | Dark gray | Fixed (4px) | Idle-tick graph consolidation event |
| **event** | Medium gray | Fixed (4px) | Raw event from observe phase |

### Edge Types

Edges are directed (shown with arrowheads) and labeled with their relationship type:

| Edge Type | Meaning |
|-----------|---------|
| **mentions** | A cognitive event references a named entity |
| **observed** | An event was observed and produced an assessment |
| **decided_from** | A decision was made based on an assessment |
| **executed** | A decision led to an action result |
| **bumped** | An assessment changed a concern's activation |
| **spawned_goal** | A decision triggered a goal |
| **produced** | A goal produced an outcome |
| **triggered_by** | A triage nomination was triggered by a concern |
| **nominated** | A concern was nominated for triage |
| **triaged** | A triage decision was made about a concern |
| **spawned_task** | Triage created a new task |
| **updated_concern** | An action updated a concern |

## Using the Explorer

### Entity Sidebar

The left panel lists all named entities with their mention counts (e.g., "Jill (12)" means 12 resources reference Jill). Click an entity to view its subgraph -- the entity node plus all directly connected nodes and their edges.

### Search Bar

Type a query to find nodes by semantic similarity (cosine similarity on BGE embeddings). Results are filtered to a minimum 0.3 similarity threshold and expanded 1 hop to show context. The search matches against node content, not just names -- so searching "weather" will find goal nodes about weather forecasts, conversation turns discussing weather, etc.

### Click to Expand

Click any node in the graph to expand 1 hop from it, adding its neighbors to the current view. This lets you progressively explore connected subgraphs.

### Type Filters

Checkboxes in the control bar let you hide/show node types to reduce visual clutter.

### Visual Encoding

- **Disc size**: Entity nodes scale with mention count (more mentions = larger). All other types have fixed sizes per type.
- **Edges (lines)**: A line between two nodes means there is an explicit typed relationship in the graph. The arrowhead shows direction (source -> target). The label shows the relationship type.
- **Distance**: Determined by D3 force simulation layout, not by semantic similarity or relationship strength. Connected nodes are pulled together; all nodes repel each other. Distance is a layout artifact, not a data signal.
- **Color**: Encodes node type (see table above).

## Technical Details

- **Visualization**: D3.js v7 force-directed graph
- **Embedding model**: BAAI/bge-small-en-v1.5 (384-dim, GPU-accelerated when available)
- **Graph storage**: In-memory with FAISS semantic index; persisted to disk on save/shutdown
- **NER extraction**: LLM-based entity extraction from user input, goals, and goal outcomes
- **API**: Subgraph expansion and semantic search via Zenoh query/reply through the executive node

For the full implementation specification, see [cognitive_graph_spec.md](cognitive_graph_spec.md).
