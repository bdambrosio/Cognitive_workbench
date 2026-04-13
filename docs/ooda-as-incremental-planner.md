# OODA Loop as Continuous Incremental Planner

**Status:** Informal spec / design discussion document  
**Date:** 2026-04-12  
**Context:** Emerged from comparison with Hermes Agent architecture and analysis of what CWB's separated reasoning/execution layers actually buy us.

## Core Insight

The OODA loop and the goal executor (tool_planner_infospace) are the same architecture operating at different levels of abstraction. The goal executor is an incremental planner whose tools are infospace primitives (`generate-note`, `search-web`, `exec-script`, etc.). The OODA loop is an incremental planner whose "tools" are strategic-level actions: goal submission, concern state management, user model updates, sensor configuration, and conversational moves.

This isn't a metaphor. It's a literal architectural claim: the OODA loop should be implemented as a continuous incremental planner — same step-by-step generation pattern, same observe-result-decide-next-step rhythm — but with a different action vocabulary operating at a different granularity.

Note: Do NOT at this time actually share implementation. this is a side note for deepening understanding only.

## What This Replaces

The current OODA loop is a Python-orchestrated pipeline: `_ooda_observe()` → `_ooda_orient()` → `_ooda_decide()` → `_ooda_act()`, called per-tick from `_main_loop_tick()`. Orient is the only phase that involves an LLM call. Decide is pure routing logic. The loop processes one event per tick and resets.

there is separate code that independently monitors and maintains concerns. This should be integrated, perhaps into the "orient" phase, or perhaps at multiple points in the loop according to activity: note impact and adjust activations (Observe?), evolve concerns (Orient?), trigger task(currently disabled) (an element in Decide?)/
The revised design replaces this with a *continuous* computation that maintains context across multiple event-action cycles, reasons about concern evolution over time, and generates strategic actions the same way the goal executor generates tool calls. By continuous we mean a fixed prefix with continued appending as we progress through a loop AND repeat loops, ie, each OODA loop is analogous to a single step of the incremental planner. 

## Architecture

### Level Separation via Action Vocabulary

The OODA planner and the goal executor are distinguished by what actions are available at each level, not by architectural differences in how they work.

**OODA-level actions** (the "tools" available to the strategic planner):

- `submit-goal` — Package and submit a bounded goal for execution. Includes goal text, expected outcome, concern linkage, and priority. This is the primary way the OODA level causes things to happen in the world.
- `update-concern` — Modify concern state: adjust weight, change status, add notes, link/unlink tasks. Reflects the OODA loop's evolving understanding of what matters.
- `update-user-model` — Write observations about the user's current state, interests, engagement patterns, or inferred needs.
- `configure-sensor` — Adjust sensor parameters, enable/disable sensors, change disposition routing. The OODA loop tunes its own perception.
- `say` — Direct conversational output to the user. Used for proactive engagement, status updates, or remarks that don't require a full goal execution.
- `ask` — Pose a question to the user. Used when the OODA loop needs information to make a strategic decision.
- `wait` — Explicitly decide to do nothing. Records *why* inaction is the right choice (important for concern-aware agents — deliberate inaction is different from having nothing to do).
- `reflect` — Write a strategic-level observation to the OODA context. Not an external action, but a way to reason on paper about patterns, priorities, or uncertainties before committing to an action. Analogous to `think` in the goal executor.
- `sleep` — Initiate a context consolidation cycle (see below).

**Goal-executor-level actions** remain as they are today: infospace primitives, tool calls, `ask`/`say` within conversation, etc. The OODA level cannot invoke these directly. It can only submit goals.

This boundary is critical. The temptation to let the OODA loop "just quickly do this one thing directly" would be strong and corrosive. The level discipline is what preserves the attention budget at each layer.

### Continuous Context, Not Per-Tick Reset

The current OODA loop processes one event per tick and discards its reasoning context. The revised design maintains a running context that accumulates across multiple event-action cycles. This implies: 1. we probably need an expanded report of goal results at the OODA level; 2. there is probably a one-time OODA context 'prefix', the per-iteration context added should be compact, and perhaps compressed prior to the next loop.

The OODA planner's context includes:

- **Concern landscape:** Current concerns with activation levels, trends, and linked tasks. This is the primary orientation substrate.
- **Recent event-action pairs:** The last N fully-expanded cycles (event observed, assessment, action taken, result).
- **Rolled-up history:** Older cycles compressed to one-line summaries ("advanced paper-monitoring concern; found 3 papers; user engaged with one").
- **User model snapshot:** Current state of the user model, foregrounded when relevant.
- **Sensor state:** Active sensors, recent readings, disposition routing.
- **Goal field:** Currently running goal (if any), queued goals, recent goal outcomes.
- **Task state:** Active tasks with their WIP summaries, current milestones.

The key difference from the goal executor: the OODA loop's dependency structure is *present-tense*. The question "what should I do now" depends primarily on current concern state, the most recent event, and recent action history — not on a dense chain of reasoning where step 7 depends on steps 1–6. This means context can be managed with a moving window rather than requiring full chain preservation.

### Sleep Cycles (Context Consolidation)

When the OODA context approaches its budget limit, the planner can invoke `sleep` — a managed consolidation, not a hard reset. we currently have a max context in the local llm of 65384 tokens. need to always ensure we have enought o complete next loop before starting it. Note that this is only for OODA context, goal planner has its own separate context.

**Sleep does:**
1. Persist all durable state updates (concern changes, user model observations, task WIP updates) that haven't already been written.
2. Compress the current event-action history into rollup summaries.
3. Write a "consolidation note" — a brief strategic-level summary of where things stand: what concerns are active, what the agent was paying attention to, any open questions or pending decisions.
4. Terminate the current continuous context.

**Sleep does not:**
- Lose concern state (already persisted in the concern model).
- Lose user model state (already persisted).
- Lose task state (already in WIP notes).

**Resumption:** A new OODA context begins, loaded with current concern state, the consolidation note, recent rollup history, and fresh sensor/goal state. The agent doesn't "wake up confused" — it wakes up with a rich persistent substrate and a brief orientation document that its prior self wrote.

The consolidation note is the key mechanism. It's the OODA loop writing a message to its future self about what matters right now. This is where tonal continuity and strategic coherence survive the context boundary.

### Rollup Strategy

Within a continuous OODA context, older event-action cycles are progressively compressed:

- **Full expansion** (last 2–3 cycles): Complete event, assessment, action, and result.
- **Summary** (cycles 4–N): One-line descriptions preserving what happened and which concern it served.
- **Concern-level aggregation** (oldest): Folded into concern state updates. "Over the past hour, the paper-monitoring concern was serviced twice, finding 5 relevant papers. User engaged with 2."

The rollup is driven by context budget pressure, not by a fixed schedule. If the OODA loop is processing a burst of related events, it may keep more cycles expanded. During quiet periods, aggressive rollup preserves budget for richer reasoning when something does happen.

## Relationship to Goal Execution

When the OODA planner invokes `submit-goal`, the goal enters the existing execution pipeline: the goal executor (tool_planner_infospace or tool_planner_infospace_vllm) plans and executes it step by step. On completion, the result is delivered back to the OODA planner's context as a new "event" — specifically, a goal-completion event with the outcome summary, primary product, and quality status.

The OODA planner then reasons about this result in the context of the concern that motivated the goal. Did this advance the concern? Is the next milestone now appropriate? Has the user's situation changed in a way that makes the next planned action irrelevant? Should the concern's activation be adjusted?

This is where the incremental-planner framing pays off. The OODA loop doesn't just receive a goal result and route it — it *reasons about what the result means* for the concern landscape, with full access to the strategic context of why that goal was submitted in the first place.

## Nested Planners: The Recursive Observation

If the OODA loop is an incremental planner whose tool calls include `submit-goal`, and the goal executor is an incremental planner whose tool calls include infospace primitives, then CWB's core abstraction is *nested incremental planners operating at different granularities*.

This is compositional. In principle, a concern itself could be managed by a planner. You could nest three levels deep if a problem warranted it: strategic concern management → tactical goal planning → operational tool execution.

Whether deeper nesting is useful is an empirical question. But the architectural symmetry means it's available without new machinery.

## What This Design Addresses

### The Hermes Comparison

Hermes Agent uses a single synchronous loop for everything. That loop has no separation between strategic reasoning and tactical execution, which gives it natural fluidity for short tasks (build a thing → run it → reason about output → offer follow-up). But it has no architectural home for:

- Reasoning about *whether* to act, not just *how* to act.
- Managing multiple evolving concerns with different time horizons.
- Evaluating goal outcomes against the concerns that motivated them.
- Spontaneously connecting a result to something the user cares about without being prompted.
- Sustaining engagement that deepens over time rather than resetting per-task.

The OODA-as-incremental-planner design addresses all of these by giving strategic reasoning its own continuous computational context, while preserving bounded goal execution for tactical work.

### The Attention Budget Problem

A single loop doing everything — concern tracking, tool execution, conversation management, result interpretation — competes for one context window. The two-level design gives each level its full attention budget: the OODA level uses its context for strategic reasoning and concern evolution, the goal executor uses its context for tactical planning and tool selection.

### Information Loss at the Level Boundary

The current OODA loop's per-tick reset discards reasoning context between events. The revised design's continuous context preserves the strategic thread: why a goal was submitted, what the concern behind it needs, how the user's situation has been evolving. Goal results land in a context that already holds the full motivational history.

### The Companion Use Case

An agent that holds the threads of someone's life needs continuity of attention. The continuous OODA context — with its concern landscape, user model, and accumulated strategic reasoning — is the substrate for that continuity. Sleep cycles preserve it across context boundaries via consolidation notes. The agent doesn't wake up and catch up; it resumes with orientation.

## Open Questions

1. **Context budget for the OODA level.** At the goal-executor level, ~10 step cycles is manageable for a single focused goal. The OODA level's present-tense dependency structure suggests the ceiling is higher, but how much higher? Needs empirical testing with rollup strategies.

2. **Sleep trigger policy.** When should the OODA loop sleep? Context budget pressure is the obvious trigger, but there may be natural breakpoints (user goes quiet, a significant concern transition completes) that produce better consolidation notes.

3. **Consolidation note quality.** The continuity of the agent's strategic reasoning depends on how well the consolidation note captures the current state of attention. What makes a good consolidation note? This may itself warrant a small study.

4. **sglang @function vs. prefix reload.** The two goal-executor implementations (sglang continuous @function vs. vllm prefix reload) produce identical results given identical token sequences. The OODA level could use either approach. The sglang @function preserves exact token sequences including the model's own hedges and reasoning artifacts; prefix reload reconstructs from saved state. For the OODA level specifically, where tonal continuity may matter for the companion use case, the sglang @function may have an edge — but this is a hypothesis, not a demonstrated requirement. Either implementation is compatible with this design.

5. **Concern-to-goal generation.** The current concern triage system (`concern_triage.py`) bridges concerns to task proposals via periodic LLM calls. In the revised design, this becomes native to the OODA planner — it can reason about whether a concern warrants a goal submission as part of its ongoing strategic computation, rather than as a separate periodic process. The triage module may be absorbed into the OODA planner's action vocabulary.

6. **Event prioritization under load.** If multiple events arrive while the OODA planner is reasoning, how are they queued and prioritized? The current per-tick model processes one event at a time. The continuous planner needs a policy for event batching or preemption.

## Implementation Notes

- The OODA planner should share the incremental planning infrastructure with the goal executor where possible. Same step generation pattern, same result handling, different action schemas.
- The action vocabulary (submit-goal, update-concern, etc.) should be defined as tool schemas, exactly as infospace primitives are defined for the goal executor. This keeps the architecture genuinely symmetric.
- The current `_ooda_observe`, `_ooda_orient`, `_ooda_decide`, `_ooda_act` methods become input processing for the continuous planner (observe packages events) and output handling (act dispatches the planner's chosen action). Orient and decide are absorbed into the planner's reasoning.
- Living state (`ooda_living_state.py`) continues to serve as the persistent substrate, but is now *also* a source of context injection for the OODA planner, not just a record of what happened.
- The concern triage module's nomination logic (activation monitor, orient-stage bypass) can feed events into the OODA planner rather than running as a separate periodic process.
