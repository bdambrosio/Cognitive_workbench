# Envisioning & Quality Control

The Cognitive Workbench uses two mechanisms to maintain quality: **envisioning** (lightweight conversational framing) and **reflection** (post-execution analysis). Together they enable coherent dialog, quality assessment, and systematic failure recovery.

## Envisioning: Conversational Framing

Envisioning is a lightweight LLM call that frames conversational moments before the planner processes them. It answers: "What is the other party doing, and what should I do next?"

### How it works

When a message arrives (from the user or another agent), the Executive Node calls `_envision_conversation_turn()` before passing the message to the planner:

1. **Input**: Recent dialog history (last 6 turns), character description, conversation purpose, incoming message
2. **LLM call**: Small, fast generation (128 tokens, temperature 0.3)
3. **Output**:
   - `turn_intent` — What conversational move is the other party making? (1 sentence)
   - `my_move` — What dialogue act should I perform next? (1 sentence, action-oriented)

### Why it matters

Without envisioning, the planner sees raw messages without conversational context. With envisioning, the planner receives framing like:

> **turn_intent**: "User is requesting a literature review on a specific topic with particular constraints."
> **my_move**: "Acknowledge the constraints and begin structured search."

This produces more coherent multi-turn dialog. The planner knows not just *what* was said but *what kind of conversational move* it is and what an appropriate response looks like.

### When envisioning runs

- **User messages**: Every non-goal message from the user triggers envisioning
- **Agent-to-agent dialog**: When one agent sends a message to another via the `ask` primitive, the receiving agent envisions the conversation turn before responding
- **Goal submissions**: Goals bypass envisioning (they go directly to the planner)

### UI indicator

The **End** button in the UI turns **red** when a dialog is active (User or agent-to-agent) and **grey** when closed. Users can interrupt internal agent-to-agent conversations by clicking End.

## Reflection: Post-Execution Analysis

After every plan execution, the planner runs a **reflection** phase that analyzes the full execution trace and produces a structured `ReflectionFrame`.

### The Reflection Process

1. The **reflection analyst** LLM receives:
   - Previous `task_state` (ephemeral working memory from last attempt)
   - Previous `world_model` (persistent cross-goal knowledge)
   - Full execution trace (every step, result, and error)
   - Original goal text
   - Step budget used vs. available

2. It produces a `ReflectionFrame` — a structured JSON analysis.

### ReflectionFrame Structure

#### task_state (ephemeral — for retrying the same goal)

```
summary:            Brief description of current progress
immediate_blockers: What's preventing progress right now
active_hypotheses:  Strategies to try next (max 6)
proven_safe_paths:  Approaches known to work
exhausted_search:   What's been tried (locations, objects, actions)
```

The task_state carries forward if the same goal is retried. This prevents the planner from repeating failed approaches — it can see what was already tried and focus on untested hypotheses.

#### world_model updates (persistent — reusable across goals)

The reflection analyst conservatively promotes facts to the persistent world model. Only facts that remain true under different goals and at different times are promoted. Excluded:
- Current agent state or location
- Single observations or one-time events
- Trivial task details

This ensures the world model grows slowly with genuinely reusable knowledge.

#### tool_insights

Discovered tool behaviors and constraints:

```
tool:    tool name
insight: description of capability or constraint
status:  reliable | unreliable | constrained
```

Example: `{"tool": "search-web", "insight": "Returns max 10 results per query", "status": "constrained"}`

#### failure_mode

Classification of what went wrong (if anything):

| Mode | Meaning |
|------|---------|
| `none` | Goal completed successfully |
| `missing_affordance` | The agent needed a capability it doesn't have |
| `incorrect_context` | Wrong assumptions about the situation |
| `resource_exhaustion` | Ran out of step budget or API quota |
| `tool_limitation_or_misbehavior` | A tool didn't work as expected |
| `exhausted_or_misdirected_search` | Searched everywhere plausible without finding the answer |
| `incorrect_task_interpretation` | Misunderstood what the goal was asking |

#### quality_status and verification

- **quality_status**: `passed` / `failed` / `needs_revision` / `interrupted`
- **verification_answer**: `YES` (goal fully met) / `PARTIAL` (partially achieved) / `NO` (failed)

These are used by the scheduler to decide whether to mark a goal as completed or retry it.

## Missing Affordance Monitoring

When the reflection analyst classifies a failure as `missing_affordance`, the system logs the event for future tool authoring.

### What gets logged

Each missing affordance event is appended to `create_tool_opportunities.jsonl` (in the history directory) with:

```json
{
  "seq": 42,
  "ts": "2026-02-25T14:30:00",
  "agent": "Jill",
  "world": "infolab",
  "goal": "Monitor stock prices and alert on changes > 5%",
  "failure_evidence": ["No tool available for setting up periodic checks"],
  "open_questions": ["Should alerts be email or UI notifications?"],
  "immediate_blockers": ["No scheduling primitive for sub-goal triggers"],
  "available_tools": ["search-web", "stock-price", "check-email", ...],
  "compressed_trace": "... execution trace ..."
}
```

### Tool spec inference

The `create-tool` tool (`src/tools/create-tool/analyze.py`) can analyze accumulated `create_tool_opportunities.jsonl` entries and use an LLM to infer what new tool should be built:

- Input: failure evidence, blockers, available tools, execution trace
- Output: proposed tool name, description, required parameters, implementation hints

This creates a feedback loop: the agent encounters capability gaps → the system records them → developers (or future automation) can prioritize which tools to build.

### Affordance Filtering (World-Specific)

For world integrations like ScienceWorld, the system also uses an **AffordanceFilter** (`affordances.py`) that validates actions before attempting them:

- **Layer 1**: Hard type exclusions (can't manipulate abstract concepts)
- **Layer 2**: Verb-role rules (can only "open" container-like things)
- **Layer 3**: Inventory rules (must hold an object before using it)
- **Layer 3**: Location/visibility rules (must be near something to interact with it)

Failed actions are tracked with `fail_counts`; after repeated failures (default 2), the action is suppressed to avoid wasting step budget.

## How Envisioning and Reflection Work Together

```
Message arrives (User or agent)
    │
    ▼
_envision_conversation_turn()
    ├── turn_intent: "User is asking for X"
    └── my_move: "I should do Y"
    │
    ▼
Planner receives framed context
    │
    ▼
Plan execution (Stages 0-3 loop)
    │
    ▼
_reflect() on execution trace
    ├── task_state: for retry
    ├── world_model: for future goals
    ├── tool_insights: for tool learning
    ├── failure_mode: for diagnosis
    │   └── "missing_affordance" → log to create_tool_opportunities.jsonl
    └── quality_status: for scheduler
    │
    ▼
Result published
    ├── [quality: passed]   → goal completed
    ├── [quality: failed]   → retry with task_state context
    └── [quality: partial]  → needs_revision, may retry
```

The key insight is that envisioning operates **before** planning (framing the conversational context) while reflection operates **after** planning (learning from the execution). Together they create a cycle of contextual understanding and continuous improvement.

## Next

- [Architecture](architecture.md) — the incremental planner that sits between envisioning and reflection
- [Goals & Scheduling](goals-and-scheduling.md) — how quality_status affects goal lifecycle
- [Tools & Primitives](tools-and-primitives.md) — the tools that plans execute
