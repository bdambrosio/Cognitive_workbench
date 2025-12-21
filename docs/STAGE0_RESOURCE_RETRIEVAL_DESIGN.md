# Stage 0 Resource Retrieval Design Analysis

## Overview

This document analyzes the proposal to add "working memory" to the incremental planner by enabling it to discover and reference pre-existing Notes and Collections through embedding-based retrieval.

**Goal**: Allow `tool_planner_infospace` to discover relevant existing resources before planning, avoiding redundant work and enabling continuity across planning sessions.

---

## Current State Analysis

### What Works Now
- ✅ Notes/Collections can be created and persisted
- ✅ Resources can be loaded explicitly by name/ID via `load` primitive
- ✅ FAISS-based embedding infrastructure exists (`FAISSStore` in `infospace_resource_manager.py`)
- ✅ Collections can be indexed for semantic search (`index` primitive)
- ✅ Stage 3 reflections generate `THOUGHTS` and `NEXT_TASK` text

### What's Missing
- ❌ Planner has no awareness of existing resources unless explicitly loaded
- ❌ No cross-resource discovery mechanism
- ❌ No embedding index of resource metadata (only Collection content is indexed)
- ❌ Stage 3 reflections are not captured for resource description

---

## Design Proposal Review

### Strengths

1. **Leverages Existing Infrastructure**: FAISSStore and embedding model already exist
2. **Minimal Core Loop Changes**: Stage 0 is additive, doesn't disrupt Stage 1-3
3. **Clear Separation**: Index maintenance vs. retrieval vs. prompt injection
4. **Type-Aware**: Separate indexes for Notes vs. Collections enables filtering

### Concerns & Questions

#### 1. **Index Scope & Freshness**

**Question**: Should we index ALL Notes/Collections, or only persistent ones?

**Recommendation**: 
- **Index only persistent resources** initially (simpler, less noise)
- Add optional "bookmark" metadata flag for non-persistent but important resources
- Rationale: Transient scratch notes would pollute search results

**Question**: How to handle resources created DURING a planning session?

**Recommendation**:
- Stage 3 reflection hooks can trigger immediate indexing (before next plan)
- Batch updates at plan completion for efficiency
- Consider "dirty flag" to mark resources needing re-index after content changes

#### 2. **Embedding Content Strategy**

**Proposal**: Concatenate identifier + action metadata + Stage 3 commentary

**Questions**:
- What if Stage 3 doesn't generate useful commentary?
- Should we include actual Note content snippets (privacy/context window concerns)?
- How to handle Collections with 100+ items (metadata explosion)?

**Recommendation**:
```
For Notes:
  text = f"{name_or_id}\n{action_summary}\n{stage3_commentary}\n{content_preview[:200]}"

For Collections:
  text = f"{name_or_id}\n{action_summary}\n{item_count} items\n{first_3_item_summaries}"
```

**Content Preview Strategy**:
- Notes: First 200 chars of content (or summary if available)
- Collections: Count + first 3 item summaries (via `summarize` tool if indexed)
- Avoid full content to keep embeddings focused on "what is this resource about"

#### 3. **Stage 0 Query Generation**

**Proposal**: Tiny gen to convert goal → 1-2 search queries

**Questions**:
- Should this be a separate SGLang function or inline in Stage 0?
- How to handle goals that don't map well to resource queries (e.g., "create a new note")?

**Recommendation**:
```python
@function
def stage0_query_generator(s, goal: str):
    """Generate 1-2 search queries from goal"""
    s += system("Convert goal into search queries for finding relevant Notes/Collections.")
    s += user(f"Goal: {goal}\nGenerate 1-2 concise search queries.")
    s += assistant(gen("queries", max_tokens=128))
    return s["queries"]  # JSON array or newline-separated
```

**Fallback**: If query generation fails or returns empty, skip Stage 0 (no resources injected).

#### 4. **Index Maintenance Hooks**

**Proposal**: Extend InfospaceExecutor hooks (create-note, create-collection, etc.)

**Questions**:
- Should indexing be synchronous (slows execution) or async (complexity)?
- How to handle resources created outside incremental planner (standard planner, manual)?

**Recommendation**:
- **Synchronous for persistent resources** (user expects them indexed)
- **Async batch for non-persistent** (if bookmark flag set)
- Hook into `_execute_create_note`, `_execute_create_collection`, `_execute_persist`
- Also hook into `_execute_flatten`, `_execute_map` (modify Collections)

**Implementation Location**:
- New `ResourceIndexer` class in `infospace_resource_manager.py` (reuse FAISSStore)
- Called from `InfospaceExecutor` hooks
- Store indexes at `data/vector/notes.index` and `data/vector/collections.index`

#### 5. **Prompt Injection Format**

**Proposal**: Bullet summaries with name/ID and description

**Questions**:
- How many resources to inject? (proposal says 3 Notes + 2 Collections)
- Should we include variable bindings from previous plans?
- How to format for clarity without token bloat?

**Recommendation**:
```markdown
# Available Notes / Collections (may be relevant)

## Notes:
- Note_42 ("market-share-memo"): Summary of Q3 earnings call. Created via summarize on $earnings_call.
- Note_87 ("transformer-paper"): Research notes on attention mechanisms. Created via search-web + refine.

## Collections:
- Collection_15 ("research-papers"): 23 items from semantic-scholar on LLM interpretability.
- Collection_8 ("emergent-capabilities-corpus"): 20 items with topic/stance metadata.

To use these resources, reference by name (e.g., "market-share-memo") or ID (e.g., "Note_42") in load actions.
```

**Token Budget**: Keep to <400 tokens total (5 resources max).

#### 6. **Executor Bridge - Avoiding Duplicates**

**Proposal**: `sgl_to_infospace_action` should recognize existing resources

**Questions**:
- How to distinguish "load existing" vs "create new" when planner says "create-note"?
- Should Stage 0 pre-bind resources to `$preload_X` variables?

**Recommendation**:
- **Don't auto-bind** (adds complexity, may confuse planner)
- **Enhance Stage 1 prompt** to suggest using `load` for listed resources
- Let planner decide: if it says "create-note" with same name, that's intentional
- Add validation warning if planner creates duplicate of listed resource

**Alternative**: Add `load-if-exists` primitive that checks name first, but this adds API surface.

#### 7. **Privacy & Retention**

**Concern**: Embeddings contain content snippets (may be sensitive)

**Recommendation**:
- Store indexes locally only (`data/vector/`)
- Add optional "exclude_from_index" flag for sensitive Notes
- Document retention policy: indexes cleared on world reset, persist with world save

---

## Implementation Plan

### Phase 1: Index Infrastructure

1. **Create `ResourceIndexer` class** (`infospace_resource_manager.py`)
   - Two FAISSStore instances: `notes_index`, `collections_index`
   - Methods: `index_note(resource_id, text)`, `index_collection(resource_id, text)`
   - Methods: `search_notes(query, k=3)`, `search_collections(query, k=2)`
   - Persistence: `save_indexes()`, `load_indexes()`

2. **Build embedding text from resources**
   - Extract name/ID, action metadata, content preview
   - For Collections: include item count + summaries
   - Store metadata mapping: `resource_id -> {name, type, created_by, ...}`

3. **Hook into executor** (`infospace_executor.py`)
   - `_execute_create_note`: Call indexer if persistent
   - `_execute_create_collection`: Call indexer if persistent
   - `_execute_persist`: Re-index resource
   - `_execute_flatten`/`_execute_map`: Update Collection index if modified

### Phase 2: Stage 0 Retrieval

1. **Add query generator function** (`incremental_planner.py`)
   ```python
   @function
   def stage0_query_generator(s, goal: str):
       # Generate 1-2 queries from goal
   ```

2. **Add Stage 0 to `tool_planner_infospace`**
   - Before Stage 1: generate queries → search indexes → format results → inject prompt
   - If no results or query fails, skip injection (graceful degradation)

3. **Format injection text**
   - Build bullet list of top-k results
   - Include name, ID, brief description
   - Add usage hint: "Use load primitive to reference these"

### Phase 3: Stage 3 Commentary Capture

1. **Extract Stage 3 reflections**
   - Capture `THOUGHTS` and `NEXT_TASK` from Stage 3 outputs
   - Store in executor state: `resource_commentary[resource_id] = thoughts`

2. **Update index with commentary**
   - On resource creation/modification: include latest commentary
   - Re-index when Stage 3 updates resource description

### Phase 4: Testing & Refinement

1. **Test scenarios**:
   - Create persistent Note → new plan → verify it appears in Stage 0
   - Create Collection → index it → new plan → verify discovery
   - Multiple resources → verify top-k selection works
   - Empty indexes → verify graceful skip

2. **Metrics**:
   - Index size (number of resources)
   - Search latency
   - Token cost of injection
   - Planner success rate with/without Stage 0

---

## Open Questions for Discussion

1. **Indexing Strategy**: 
   - Should we index non-persistent resources with a "bookmark" flag, or only persistent?
   - How to handle resources created outside incremental planner?

2. **Content Inclusion**:
   - How much Note content to include in embedding? (200 chars? Summary only?)
   - For Collections, should we summarize member Notes or just count them?

3. **Query Generation**:
   - Separate function or inline? 
   - Should we generate separate queries for Notes vs. Collections?

4. **Variable Binding**:
   - Should Stage 0 pre-bind resources to `$preload_X` variables, or just list them?
   - How to handle name collisions (planner wants to create "research" but "research" exists)?

5. **Performance**:
   - Should indexing be async to avoid blocking execution?
   - How to handle large indexes (1000+ resources)?

6. **Privacy**:
   - Should users be able to mark resources as "exclude from index"?
   - How to handle sensitive content in embeddings?

---

## Recommended Next Steps

1. **Clarify scope**: Confirm indexing only persistent resources vs. all resources
2. **Prototype Stage 0**: Implement minimal query → search → injection flow
3. **Test with real scenarios**: Create test corpus, verify discovery works
4. **Iterate on format**: Refine prompt injection based on planner behavior
5. **Add commentary capture**: Integrate Stage 3 reflections into index updates

---

## Alternative Approaches Considered

### Option A: Query all resources on startup
- **Pros**: Simple, no indexing needed
- **Cons**: Doesn't scale, no semantic matching

### Option B: Use existing Collection search infrastructure
- **Pros**: Reuse `index`/`search` primitives
- **Cons**: Requires Collections to be indexed (not all are), doesn't cover Notes

### Option C: LLM-based resource summarization
- **Pros**: Rich descriptions
- **Cons**: Expensive, slow, adds latency to Stage 0

**Selected**: Embedding-based approach balances speed, cost, and semantic matching.

