# Retrieval Envisionment Spec

## Design Intent

Current infospace retrieval sends the raw user input string (or a crude substring) as the FAISS query. This produces garbage for most real queries because user utterances are questions, commands, or conversational fragments — none of which resemble the stored content they need to match against.

This spec introduces **retrieval envisionments**: HyDE-style hypothetical answer statements generated in a dedicated post-orient LLM call, used as both FAISS queries and post-retrieval relevance filters. The envisionment is what the answer *would look like* if it existed in memory, not a reformulation of the question. The retrieval plan runs as a separate, conditional call after orient — it receives orient's output plus the concern graph, and only fires when orient's action choice suggests memory context is needed.

Example: User says "how old is Joe?" → envisionment: "Joe is approximately 65 years old." The envisionment embeds close to any stored Note containing Joe's age, birthday, or biographical timeline — far closer than the question "how old is Joe?" would.

Envisionments may be compound. "Joe joined the Berkeley AI lab around 2019 as a postdoc" is a single envisionment capturing an entity, an organization, a relationship, and a temporal anchor. Decomposing this into independent single-entity queries would lose the relational structure that makes retrieval precise.

---

## Integration Point: Separate Post-Orient Call

Envisionment generation is a **separate LLM call that runs after orient completes**, not an addition to the orient prompt. Orient already carries significant cognitive load (classification, salience, concern bumps, action choice, rationale). Adding retrieval planning to that call risks degrading orient quality and adds latency to turns that don't need retrieval at all.

### When to Fire

The retrieval plan call is **conditional**. Orient's output already tells us whether memory retrieval is likely needed. Skip the retrieval plan call when:

- Orient classifies the input as a greeting, acknowledgment, or simple command
- Orient's action choice is purely reactive (direct `say`, status report from live data)
- No active concerns are relevant to the input (concern_bumps is empty or trivial)

Fire the retrieval plan call when:

- Orient's action choice involves goal execution, planning, or deliberation
- The input contains questions, references to past events, or ambiguous referents
- Orient's rationale mentions needing context, history, or stored information

The trigger logic can start simple — fire whenever orient's action choice is anything other than a direct response — and be tightened later based on trace analysis.

### Input Context

The retrieval plan call receives richer context than orient had, because it includes orient's own output:

```
Input to retrieval plan:
  - user_input: the original user message
  - orient_output: full orient result (classification, action_choice, 
      concern_bumps, overall_rationale)
  - active_concerns: list of { name, activation_level, description }
      from the concern graph
  - concern_relationships: relevant edges from the concern graph
      (parent/child, related_to) — provides structural context 
      about how concerns relate to each other
  - recent_conversation: last 2–3 turns for referent resolution
```

The concern graph is the key addition over what orient sees. Orient knows concern *names and activation levels*. The retrieval plan also sees concern *descriptions and relationships* — enough to understand that "the imaging thing" might connect to both "NINA integration" and "astrophotography session planning" through the concern graph structure.

### Output Schema

```
{
    "retrieval_needed": bool,
    "envisionments": [
        {
            "envisionment": str,   # HyDE-style hypothetical answer statement
            "scope": str,          # "recent" | "session" | "any"
            "target": str | null   # collection name if scoped, else null
        },
        ...
    ]
}
```

`retrieval_needed: false` is a valid output — the LLM may decide after examining orient's output and the concern graph that no retrieval is warranted even though the trigger fired. This is a second gate that catches false positives from the simple trigger logic.

### Generation Rules

1. **Generate 0–4 envisionments.** Most turns that reach this call produce 1–2. Zero is valid (sets `retrieval_needed: false`).

2. **Each envisionment is a declarative statement, not a question.** It describes what the matching memory entry would say if it existed. "The NINA API plugin was updated to version 2.3" not "what version is the NINA API plugin?"

3. **Compound envisionments are preferred when the information need is relational.** "Jill's last tool failure involved the semantic scholar API returning a timeout during the arXiv survey task" is better than separate envisionments for "Jill tool failure" and "semantic scholar timeout." The compound form captures the causal/relational structure.

4. **Scope reflects temporal intent.** "recent" = last few hours/current session, use when the query references something just discussed or recently changed. "session" = current working session. "any" = search all memory, use for biographical facts, stable preferences, reference knowledge.

5. **Use orient output and concern graph to resolve ambiguity.** If the user says "how's that going?" — orient's concern_bumps identify which concern was activated, the concern graph provides its description and relationships, and the envisionment should target that specific concern's domain. "The NINA integration testing is proceeding with current status and recent test results..." not "something is going well."

6. **Do not hallucinate specific values.** Use placeholders or hedging. "Joe is approximately X years old" or "Joe was born around [year]" — the embedding will still land in the right neighborhood. Inventing "Joe is 67" risks pulling in unrelated content about the number 67.

7. **If the user input contains multiple distinct information needs, generate separate envisionments for each.** "How old is Joe, and what's the status of the astrophotography session?" → two envisionments, likely different scopes.

### Retrieval Plan Prompt

```
You are generating a retrieval plan for an agent's memory system. 
Given the user's message, the orient assessment, and the active concern 
graph, produce hypothetical answer statements that describe what relevant 
memory entries would look like if they existed. These will be used as 
search queries against an embedding index — they are not shown to the user.

Rules:
- Each envisionment must be a declarative statement, not a question.
- Prefer compound statements that capture relationships between entities 
  over decomposed single-entity queries.
- Use the orient rationale and concern graph to resolve ambiguous 
  references ("that thing" → the specific concern or topic).
- Set scope to "recent" for things just discussed or recently changed, 
  "session" for current working session context, "any" for stable facts 
  or historical information.
- Do not invent specific values (numbers, dates, names not present in 
  the input or concern graph). Use hedging: "approximately," "around," 
  "related to."
- If no memory retrieval would help answer the user's message or support 
  the orient action choice, set retrieval_needed to false.

User message: {user_input}

Orient assessment:
  Classification: {orient.classification}
  Action choice: {orient.action_choice}
  Concern bumps: {orient.concern_bumps}
  Rationale: {orient.overall_rationale}

Active concerns:
{formatted_concern_list_with_descriptions}

Concern relationships:
{formatted_concern_edges}

Recent conversation:
{last_2_3_turns}

Respond with JSON only.
```

---

## Retrieval Pipeline

### Phase 1: Embed and Search (parallel per envisionment)

For each envisionment:

1. Embed the envisionment string using the same model as the index (currently BGE-small-en-v1.5).
2. Apply scope as a pre-filter:
   - "recent": filter to nodes/notes with `ts` within last N hours (configurable, default 4)
   - "session": filter to current `session_id`
   - "any": no temporal filter
   - If `target` is set: restrict to that collection
3. FAISS ANN search, top-K candidates (K=20 default, configurable).
4. Return candidate set tagged with which envisionment produced it.

All envisionments search in parallel. At infospace scale this is sub-millisecond per query on CPU, so parallelism is about code clarity not performance.

### Phase 2: Post-Retrieval Relevance Filtering

Each candidate is scored against the envisionment that retrieved it using cosine similarity between the envisionment embedding and the candidate content embedding. Candidates below a configurable threshold (default 0.6) are discarded.

This is cheap and fast — the envisionment embeddings are already computed from Phase 1, and candidate embeddings are stored in the index. The envisionment has already done the heavy lifting by reformulating the query from question-space into answer-space, so even simple embedding match is a substantial improvement over the current approach of sending raw user input.

**Known limitation:** Embedding match does not catch negation polarity, temporal staleness, or relational mismatch. An LLM micro-judge (batching all candidates per envisionment into a single scoring call) would catch these, but adds 300–500ms per envisionment. This is a natural future extension point — the pipeline is structured so that an LLM judge step can be inserted here without changing anything upstream or downstream.

### Phase 3: Merge and Deduplicate

Collect surviving candidates from all envisionments. If the same note/node appears in multiple candidate sets, keep the highest relevance score. Order by score descending. Truncate to context budget (configurable, default top-10 across all envisionments).

The merged set is injected into the agent's working context for the current turn.

---

## Interaction with CognitiveGraph

The envisionment mechanism is storage-layer agnostic. It produces queries and filters; the retrieval backend executes them. For infospace search operations (search-notes, search-collections, search-within-collection), the envisionment replaces the raw user input as the query string. For CognitiveGraph retrieval, the same envisionments can drive `semantic_search()` calls, with the additional option of graph expansion from retrieved nodes.

The envisionment `scope` maps to:
- Infospace: temporal filtering on Note creation/modification timestamps
- CognitiveGraph: temporal filtering on `ts` field, session filtering on `session_id`

The envisionment `target` maps to:
- Infospace: `search-within-collection` scoped to that collection
- CognitiveGraph: not directly applicable (graph has no collection concept), but could scope by concern subgraph

---

## Latency Budget

The retrieval plan adds one LLM call to the critical path for turns that need memory. Approximate budget on Qwen3.5-27B via SGLang:

- Input tokens: ~300–500 (user input + orient output + concern graph + recent turns)
- Output tokens: ~60–120 (JSON with 1–3 envisionments)
- Expected latency: **200–400ms**

This runs sequentially after orient (needs orient's output) but before any goal execution or response generation. The FAISS searches and post-filtering that follow are sub-millisecond and do not meaningfully add to the budget.

For turns where the trigger logic skips the retrieval plan, the latency cost is zero — just the conditional check against `skip_on_actions`.

Total added latency for a retrieval-needing turn: ~200–400ms for the retrieval plan call. This is comparable to one additional orient-scale inference and should be acceptable given that the alternative is injecting garbage context that degrades the much more expensive downstream response generation.

---

## Configuration

```python
ENVISIONMENT_CONFIG = {
    "max_envisionments": 4,
    "faiss_top_k": 20,               # candidates per envisionment
    "embedding_threshold": 0.6,       # minimum cosine sim for post-filter
    "recent_scope_hours": 4,
    "final_context_budget": 10,       # max total candidates across all envisionments
    "skip_on_actions": [              # orient action_choices that skip retrieval plan
        "greet", "acknowledge", "direct_response"
    ],
}
```

---

## What This Does NOT Solve

- **Negation polarity.** The envisionment "NVFP4 is supported on Blackwell" still embeds near "NVFP4 is NOT supported on Blackwell." Embedding-based post-filtering cannot catch this. The CognitiveGraph `polarity` attribute remains the structural fix. A future LLM micro-judge post-filter (see Phase 2 extension point) would also address this.

- **Multi-hop retrieval.** When the second query depends on the first result ("what did Joe say about the thing he mentioned last Tuesday"), the retrieval plan cannot formulate the second envisionment without the first answer. This requires planner-loop-level reasoning and is out of scope. The current spec handles the common case (independent information needs) well.

- **Memory that the agent doesn't know exists.** If no active concern or recent context hints at the relevant memory, the envisionment won't point toward it. This is the "you don't know what you don't know" problem. Broad semantic search (the existing approach) is actually better for serendipitous discovery. Consider retaining a low-weight broad search alongside envisionment-targeted retrieval.

- **Envisionment hallucination.** If the retrieval plan generates a plausible-sounding but wrong envisionment, retrieval may pull in irrelevant content or miss relevant content. The mitigation is the generation rules above (no specific values, grounded in extracted entities and active concerns), but this remains a risk. Monitor envisionment quality in execution traces.

---

## Implementation Sequence

1. **Implement the retrieval plan call as a standalone function.** Write the prompt, wire in orient output and concern graph as inputs, validate output schema. Test against a range of input types (questions, commands, chat, goal-relevant queries, ambiguous references). No retrieval changes yet — just generate envisionments and log them alongside the existing raw-input retrieval results for comparison.

2. **Add trigger logic after orient.** Implement the conditional gate: check orient's action_choice against `skip_on_actions`, fire the retrieval plan call when needed, pass through an empty envisionment list when skipped. Measure how often the call fires and how often it returns `retrieval_needed: false` (second gate effectiveness).

3. **Wire envisionments into infospace search.** Replace raw user input with envisionment text in search-notes / search-within-collection calls. Apply embedding-match post-filtering. Compare retrieval quality against baseline in execution traces.

4. **Wire into CognitiveGraph retrieval.** Same envisionments drive `semantic_search()` on the graph. Graph expansion from retrieved nodes provides the relational depth that flat search cannot.

5. **Tune.** Adjust `faiss_top_k`, `embedding_threshold`, `recent_scope_hours`, and `final_context_budget` based on observed retrieval quality. The right values are empirical, not theoretical. Also tune the trigger logic — the initial simple gate will likely fire too often; tighten based on trace data showing which action_choices actually benefit from retrieval.
