# OODA-as-Incremental-Planner: Design Decisions Log

**Status:** Design discussion, pre-implementation  
**Date:** 2026-04-12  
**Companion to:** `docs/ooda-as-incremental-planner.md` (original spec)

This document captures design decisions and clarifications from the
2026-04-12 review session. Read the original spec first; this document
records what was confirmed, refined, or resolved.

---

## Confirmed Design Principles

### 1. Two levels, not three

The nested-planners observation (OODA → goal executor → tools) is
architecturally sound, but we will implement exactly two levels. The
recursive nesting to three levels is noted as theoretically possible
but explicitly deferred. No use case currently justifies it, and the
debugging/context-management cost would be high.

### 2. Conceptual guidance, not code sharing

The OODA planner and the goal executor are the same *pattern* — 
build context, generate next action, execute, append result, repeat —
but they will NOT share implementation code from `incremental_planner.py`.
The goal executor has extensive machinery specific to tool-level
execution (codeblock parsing, `$var` binding, persist/bind, step-limit
guards, quality checks) that does not apply at the OODA level.

The incremental planner is a *reference implementation* of the
step-generate-execute-append pattern. The OODA planner should be its
own implementation using the same rhythm but different internals.
Shared utilities only where they naturally overlap (context assembly,
rollup compression, LLM call interface).

### 3. Level boundary discipline is non-negotiable

The OODA planner cannot invoke infospace primitives directly. It can
only submit goals. This constraint preserves the attention budget
separation between strategic and tactical reasoning. The temptation
to let the OODA loop "just quickly do this one thing" must be resisted.

### 4. Adapt OODA phases, don't force-fit

The current Observe/Orient/Decide/Act phase structure should be adapted
to serve the continuous planner model, not forced into the existing
`_ooda_observe`, `_ooda_orient`, `_ooda_decide`, `_ooda_act` method
set. The OODA planner's needs are different enough that the phase
implementations will be substantially new.

---

## Context Budget Assessment

### Finding: Context budget is NOT the binding constraint

The goal executor has a huge static prefix due to:
- Large number of tools with complex descriptions
- Complex planner codeblock syntax
- Large "current state" block

Despite this, 10-15 step goals rarely exceed 40K tokens.

The OODA planner's prefix will be much shorter:
- ~8 actions with simple schemas (vs dozens of tool schemas)
- No codeblock syntax complexity
- Per-step generation will be smaller (declarative actions like
  "submit this goal" vs procedural tool calls with parameters)

**Estimate:** 20-30 OODA cycles should fit comfortably in 65K tokens.
Possibly many more. Context limit can be increased if needed (model
supports larger contexts).

### Sleep may be driven by focus, not budget

Even if 30 cycles fit, sleeping earlier may produce better strategic
reasoning. A 30-cycle context spanning 5 different concerns means the
planner is reasoning about paper monitoring while holding detailed
reasoning about user model updates from 20 cycles ago. Sleeping after
a natural concern transition (significant concern resolved, user goes
quiet, topic shift) may produce more coherent consolidation notes and
sharper subsequent reasoning.

**Decision:** Implement budget-pressure sleep as the safety net, but
design for focus-driven sleep as the primary trigger. The right sleep
policy is an empirical question — test both.

---

## Event Handling During Goal Execution

### Decision: Async observe/orient, gated decide/act

When a goal is running:
- Sensor readings and user inputs continue to be processed through
  Observe and Orient phases, updating concern activations, user model,
  and current state in real time
- Decide and Act are skipped — no new goals are submitted while one
  is executing
- When the running goal completes, the OODA planner picks up with a
  *current* concern landscape and user model, not a stale batch

This avoids two problems:
- Pure queuing: events pile up during a 5-minute goal, OODA wakes to
  a stale batch where ordering mattered
- Full concurrent processing: complex scheduling and contention for
  the action channel

**Side benefit:** If observe/orient updates concern activations during
goal execution (e.g., user changes topic), the OODA planner can
handle the case where a goal completes but its motivating concern is
no longer active. This is exactly the kind of strategic reasoning the
continuous context enables — and is impossible with per-tick reset.

---

## Conversation Ownership

### Decision: OODA planner owns all user-facing communication

The goal executor does NOT have a direct channel to the user. All
communication is mediated by the OODA planner. This solves:
- Interleaving problems (no question of "who owns the channel")
- Strategic context for communication decisions (only OODA knows
  whether a status update is worth surfacing)
- Architectural clarity (goal executor is an internal worker)

### Implementation: Signal-based with default passthrough

**`say` from goal executor:**
- Goal executor posts a structured `say` signal to an event queue
  (same queue as sensor readings, goal completions)
- OODA planner sees it in next observe phase
- Default policy: forward to user as-is (prefix with "progress:" or
  similar framing)
- Future: OODA can suppress noisy updates, batch related messages,
  or add strategic context

**`ask` from goal executor:**
- Goal executor posts a structured `ask` signal to the event queue
- Goal executor thread *blocks* on a future/event, waiting for response
- OODA planner sees the ask, default policy: forward to user verbatim
- User responds → response routed back through OODA → goal executor
  future is resolved → goal resumes
- Future: OODA can answer from context ("user said 2024 earlier"),
  redirect ("not worth asking, use your judgment"), or timeout
  ("proceed with best judgment after N minutes")

**Timeout handling:** If user doesn't respond to a forwarded ask,
the OODA planner should eventually send back a "proceed with best
judgment" or terminate the goal. Policy details deferred but the
mechanism must support it from day one.

**Key architectural point:** The plumbing is in place for smart
behavior later. The goal executor's code doesn't change when OODA
policies get smarter — only the OODA-level event handling changes.

### say/ask in the goal executor are NOT the same as OODA-level say/ask

The OODA planner's `say` and `ask` actions are first-class strategic
actions — the planner decides to communicate as part of its ongoing
reasoning about concerns and user state.

The goal executor's `say` and `ask` are signals/interrupts thrown
upward to the OODA level. They are structurally similar to exceptions:
"I want to report something" or "I can't proceed without input."
The goal executor proposes communication; the OODA planner approves,
modifies, or suppresses it.

---

## Goal Result Reporting to OODA

### Decision: Rich results, iterate on format

When a goal completes, the result inserted into the OODA context
needs to be relevant to OODA-level reasoning: did this advance the
concern? What was produced? Was the quality adequate?

Err on the side of too much information initially — 65K tokens is
substantial budget. The goal executor or its completion handler should
generate a result summary that includes:
- Goal text (what was requested)
- Outcome status and quality assessment
- Primary product reference and brief description
- What concern this was servicing
- Any notable issues (step limit approached, tools failed, etc.)

The exact format needs iteration. Options for where this summary is
generated:
- `incremental_planner.py` generates it when deciding DONE
- `generate_plan` wrapper produces it
- OODA ACT handler formats it on receipt

**Open:** Which component owns this summary generation. Likely the
goal completion handler, since it has access to both the execution
trace and the concern linkage.

---

## Observe/Orient Asymmetry

### Clarification: Single observe/orient is sufficient at OODA level

The goal executor effectively observes/orients twice per step:
- Pre-step: "Given tools and progress, what next?" (planning orientation)
- Post-step: "What happened? Does this change my plan?" (assessment)

The OODA level does not need this split because:
- OODA "steps" are entire goal completions (minutes, not seconds)
- Goal results arrive pre-digested (not raw tool output)
- The backward assessment ("goal result means X for concern Y") and
  forward planning ("therefore submit goal Z, or wait") can happen
  in a single reasoning generation

If multiple events batch up (sensor readings + goal completion + user
message), the natural incremental planner rhythm handles it: each
action generation implicitly re-orients to current context. The
"second orient" is just the next loop iteration, not a separate phase.

---

## Concurrent LLM Access

### Assumption: OK for now, verify empirically

The OODA planner and goal executor run in separate threads. The sglang
runtime should handle concurrent requests (it batches). Known working
case: user chat during goal execution already involves an OODA-level
LLM call concurrent with goal executor LLM calls.

If using `@function` continuation for both planners, verify that
separate function contexts don't interfere with each other's KV cache
state. If using prefix-reload for OODA, this is cleanly separated.

**Decision:** Assume OK, smoke test before relying on it. Worst case
it blocks (serializes), which is acceptable.

---

## Relationship to Existing Components

### Concern triage (`concern_triage.py`)

Currently runs as a separate periodic process bridging concerns to
task proposals. In the revised design, this becomes native to the
OODA planner — concern-to-goal reasoning happens as part of ongoing
strategic computation. The triage module's nomination logic (activation
monitor, orient-stage bypass) feeds events into the OODA planner
rather than running independently.

**Decision:** Absorb into OODA planner. The triage module's logic
becomes input processing for the OODA event stream.

### Living state (`ooda_living_state.py`)

Continues as the persistent substrate. Additionally becomes a source
of context injection for the OODA planner — not just a record of
what happened, but active state that shapes the planner's reasoning.

### Current OODA methods

`_ooda_observe`, `_ooda_orient`, `_ooda_decide`, `_ooda_act` become
input processing (observe packages events) and output handling (act
dispatches chosen action). Orient and decide are absorbed into the
planner's reasoning — they become emergent from the LLM's generation
rather than separate coded phases.

---

## Implementation Sequence (Preliminary)

Not yet committed, but the natural order appears to be:

1. **Define OODA action schemas** — `submit-goal`, `update-concern`,
   `update-user-model`, `configure-sensor`, `say`, `ask`, `wait`,
   `reflect`, `sleep` as structured tool-like schemas
2. **Build OODA context assembly** — fixed prefix (concern landscape,
   user model, sensor state, goal field, task state) + rolling
   event-action history
3. **Implement the core OODA planning loop** — new code, not adapted
   from incremental_planner. Same rhythm, different internals.
4. **Wire say/ask signal mechanism** — event queue from goal executor
   to OODA, future/event for ask blocking, default passthrough policy
5. **Implement sleep/consolidation** — consolidation note generation,
   context teardown, resumption with fresh prefix + consolidation note
6. **Implement rollup strategy** — progressive compression of older
   cycles within a continuous context
7. **Absorb concern triage** — move nomination logic into OODA event
   processing
8. **Retire current OODA pipeline** — replace `_ooda_observe` →
   `_ooda_orient` → `_ooda_decide` → `_ooda_act` with new planner

---

## Open Items Remaining

1. **Goal result summary format** — what exactly goes into the OODA
   context when a goal completes. Need to iterate.
2. **Sleep trigger policy** — budget-pressure vs focus-driven.
   Empirical question.
3. **Consolidation note quality** — what makes a good one. May
   warrant a small study.
4. **Ask timeout policy** — how long before OODA auto-responds to
   a forwarded ask. Practical question, not architectural.
5. **sglang @function vs prefix-reload for OODA** — tonal continuity
   hypothesis. Test when implementation is far enough along.
