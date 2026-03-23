# Task Execution Specification

## Overview

This document specifies how the executive node manages multiple autonomous tasks
competing for execution time, from concern-driven initiation through ongoing
periodic execution. It supersedes the current implementation where operational
goals are handed to the goal scheduler — that model is wrong for autonomous
task management.

The goal scheduler remains appropriate for simple user-initiated one-off goals
(manual proceed, daily triggers). Task execution is a different concern with
different requirements: round-robin fairness, budget awareness, and concern-linked
lifecycle management.

---

## Core Concepts

### Task

A task is a sustained operational commitment — something the agent has decided
(or been asked) to do repeatedly. A task has:

- **Intention**: what the task accomplishes (from triage or user)
- **Linked concern**: the derived concern that motivated it
- **Lifecycle**: proposed -> establishing -> active -> completed/archived/abandoned
- **Execution state**: idle / running / cooldown
- **Task state**: persistent scratch pad accumulated across goals within a cycle

A task is NOT a single goal. A task is an **outer loop** that dynamically
decides what goal to run next based on accumulated state — analogous to how
the incremental planner decides what step to run next within a single goal.

### Goal (Step)

A goal is a single executable unit — one full planner run. Goals are the atoms
of scheduling. The tick loop dispatches one goal per tick across all active
tasks. Between goals, other tasks and user events get their turn.

### Execution Cycle

One complete operational run of an active task. A cycle consists of 1-N goals
decided dynamically by the task's operational advance LLM. The cycle ends when
the LLM decides the task's work for this cycle is complete. The task then
enters cooldown until its next cycle.

---

## The Two-Level Planner Architecture

The system has two nested planning loops:

```
OUTER LOOP: Task operational cycle (this spec)
  - State: task WIP (intention, findings, execution history, cycle state)
  - Decision: LLM decides next goal for this cycle
  - Execution: dispatches one goal per tick (yields between goals)
  - Termination: LLM says "cycle complete" → cooldown

  INNER LOOP: Incremental planner (existing)
    - State: goal context, bindings, step history
    - Decision: LLM decides next step for this goal
    - Execution: runs steps sequentially within a single goal
    - Termination: LLM says "done" → goal complete
```

This mirrors the existing establishment loop, where `_advance_task_wip` is
the outer loop deciding milestone goals, and the planner is the inner loop
executing each milestone. The operational cycle uses the **same pattern**
with a **different prompt** (operational instead of establishment).

### Comparison

| Aspect | Establishment (current) | Operational (this spec) |
|--------|------------------------|------------------------|
| Outer loop | `_advance_task_wip` | `_advance_task_execution` (new) |
| Decision | "What milestone next?" | "What goal next for this cycle?" |
| Phase gating | spec → capability → infra → activation | None — dynamic |
| Concurrency | One at a time | Multiple tasks, round-robin |
| Prompt | `_ADVANCE_TASK_PROMPT` | `_OPERATIONAL_TASK_PROMPT` (new) |
| Yields | Does not yield between milestones | Yields after each goal |
| Budget | Not budgeted (user approved) | Budgeted (autonomous) |
| Termination | Phases complete → COMPLETE | LLM says "CYCLE_DONE" |

---

## Task States

```
                    +-----------+
                    | proposed  |  (triage created, awaiting approval)
                    +-----+-----+
                          | approve (Task Manager UI)
                    +-----v-----+
                    |establishing|  (milestone loop, single-threaded)
                    +-----+-----+
                          | _complete_task_wip
                    +-----v-----+
              +---->|   idle    |  (active, waiting for next cycle)
              |     +-----+-----+
              |           | tick loop selects (round-robin, cooldown elapsed)
              |     +-----v-----+
              |     |  running  |  (executing goals for this cycle)
              |     +-----+-----+
              |           | LLM says CYCLE_DONE
              |     +-----v-----+
              |     | cooldown  |  (minimum interval before next cycle)
              |     +-----+-----+
              |           | cooldown expires
              +-----------+
```

Terminal states: `completed`, `archived`, `abandoned`

Note: there is no "pending" state. Selection and dispatch happen in the
same tick — a task goes directly from idle to running.

---

## The Operational Advance Loop

When the tick loop selects a task for execution, it calls
`_advance_task_execution(task_wip)` — the operational analog of
`_advance_task_wip`.

### Task WIP State During Execution

The task WIP note accumulates state across goals within a cycle:

```python
{
    # Identity (set during establishment)
    "task_wip_id": "twip_1",
    "intention": "Run check-health diagnostic and report failures",
    "status": "active",
    "lifecycle": "operational",
    "linked_concern_id": "dconcern_2",

    # Operational state (persists across cycles)
    "execution_count": 3,
    "last_executed": "2026-03-23T10:00:00Z",
    "cooldown_seconds": 3600,
    "execution_history": [...],          # ring buffer of past cycles

    # Cycle state (reset each cycle, accumulates across goals within cycle)
    "cycle_state": "running",            # idle | running | done
    "cycle_goals_completed": [           # goals completed in THIS cycle
        {
            "goal_text": "Run check-health and capture output",
            "result_summary": "Overall status: ok, no concerns",
            "status": "completed",
            "timestamp": "2026-03-23T11:00:05Z"
        }
    ],
    "cycle_findings": [                  # accumulated within THIS cycle
        "Health check returned ok, all subsystems nominal"
    ],

    # Establishment artifacts (from setup, read-only during execution)
    "establishment_findings": [...],
    "establishment_milestones": [...],
}
```

### The Advance Prompt

```
_OPERATIONAL_TASK_PROMPT:

You are executing one cycle of a recurring autonomous task.

TASK INTENTION: {intention}

ESTABLISHMENT CONTEXT: {establishment_findings}

CYCLE STATE:
Goals completed this cycle: {cycle_goals_completed}
Findings this cycle: {cycle_findings}

EXECUTION HISTORY (recent cycles): {recent_execution_history}

Determine what to do next. Your options are:
1. SUBMIT_GOAL: Submit a goal for the next step of this cycle.
2. CYCLE_DONE: This cycle is complete. Summarize what was accomplished.

IMPORTANT:
- Each goal should be a single, focused operation.
- Reference notes by NAME, never by ID.
- Do not repeat work already done in this cycle.
- If the task's intention is fully satisfied by completed goals, say CYCLE_DONE.

Respond in this format:
ACTION: <SUBMIT_GOAL | CYCLE_DONE>
GOAL_TEXT: <goal text if SUBMIT_GOAL, or cycle summary if CYCLE_DONE>
```

### Goal-to-Task State Flow

When a goal completes within an operational cycle:

```
1. Goal thread finishes → _set_task_goal_result(task_wip, result)
2. Append to cycle_goals_completed
3. Append findings from result to cycle_findings
4. Sanitize Note IDs in findings (existing _sanitize_note_ids)
5. Persist updated task WIP
6. Task state remains "running" — next tick may select it again
```

When the advance LLM says CYCLE_DONE:

```
1. Record cycle in execution_history (ring buffer, max 20)
2. Increment execution_count
3. Set last_executed = now
4. Clear cycle_goals_completed and cycle_findings
5. Set cycle_state = "idle"
6. Cooldown timer starts (elapsed time check, not a timer thread)
```

---

## Tick Loop: Task-Aware Execution

### Revised `_main_loop_tick` Structure

```python
def _main_loop_tick(self):
    # 1. Goal completion: check if worker thread finished
    if self._goal_done_event.is_set() and not self._is_goal_running():
        self._goal_done_event.clear()
        self._handle_goal_completion()  # updates task state, records result

    # 2. Task establishment: if one task is establishing, advance it
    if self.active_task_wip and not self.active_task_wip_waiting and not self._is_goal_running():
        self._advance_task_wip()
        return  # establishment gets priority — one task at a time

    # 3. Ask reply routing (while goal blocked on ask)
    if self.awaiting_ask_response and self.text_input_queue:
        self._route_ask_reply()

    # 4. Skip event processing while goal is running
    if self._is_goal_running():
        return

    # 5. Task execution dispatch (round-robin)
    if not self.active_task_wip:  # don't dispatch while establishing
        task = self._select_next_task()
        if task:
            self._advance_task_execution(task)
            return  # dispatched a goal — done for this tick

    # 6. OODA pipeline (only when truly idle)
    event = self._ooda_observe()
    if event is None:
        self._ooda_idle_tick()  # triage runs here
        return
    # ... orient, decide, act
```

### Task Selection

```python
def _select_next_task(self) -> Optional[Dict]:
    """Select the next eligible task for execution (round-robin by staleness)."""
    if self._is_goal_running():
        return None

    # Check autonomy budget
    if self.goal_scheduler.budget_remaining() <= 0:
        return None

    active_tasks = [
        t for t in self._get_all_task_data()
        if t.get("status") == "active"
        and t.get("lifecycle") == "operational"
        and t.get("cycle_state", "idle") in ("idle", "running")
    ]

    now = time.time()
    eligible = []
    for t in active_tasks:
        if t.get("cycle_state") == "running":
            # Already mid-cycle — always eligible (continue where we left off)
            eligible.append(t)
        elif t.get("cycle_state", "idle") == "idle":
            # Check cooldown
            last = t.get("last_executed")
            cooldown = t.get("cooldown_seconds", 3600)
            if last is None or (now - parse_iso(last)) > cooldown:
                eligible.append(t)

    if not eligible:
        return None

    # Sort: running tasks first (finish what you started), then by staleness
    eligible.sort(key=lambda t: (
        0 if t.get("cycle_state") == "running" else 1,
        t.get("last_executed") or ""
    ))

    return eligible[0]
```

Key behaviors:
- Tasks mid-cycle (running) are always eligible — finish what you started
- Idle tasks must pass cooldown check
- Running tasks get priority over idle tasks (complete current work)
- Among idle tasks, stalest-first (round-robin by last_executed)

---

## Integration with Concern Triage

The triage system creates proposed tasks. The lifecycle:

```
Concern activation → Triage nomination → Triage LLM decision
    → create_task → proposed (awaiting approval)
    → approve → establishing (milestone loop)
    → complete → active (idle, in execution pool)
    → tick selects → running (cycle goals execute)
    → cycle done → cooldown → idle → (repeat)
```

After each execution cycle:
- Seed concern tasks: concern stays active, task re-executes after cooldown
- Event-triggered concern tasks: may resolve concern after successful cycle
- Distillation fires on task abandonment/archival, not on every cycle

---

## Budget Integration

The autonomy budget gates task dispatch, not task existence:

- **Establishing**: Not budgeted. The user approved it (or it's a triage
  proposal the user approved). Establishment can be interrupted via Task
  Manager if it's consuming too much.
- **Operational execution**: Budgeted. Each goal's wall-clock execution time
  is tracked against the rolling budget window.
- **User-initiated goals** (via chat/UI): Not budgeted. Always execute.

Budget check happens in `_select_next_task` — if budget is exhausted, no
autonomous task is selected. The OODA pipeline still runs (user events,
triage), but no autonomous goals execute until budget replenishes.

---

## Interaction with User Events

The OODA pipeline processes user chat, sensor events, and other inputs. It
runs only when no goal is executing AND no task was dispatched this tick:

```
Tick timeline with 3 active tasks (A, B, C):

Tick 1: Select A (running, mid-cycle) → dispatch A goal 2 → [executing]
Tick 2: Goal running → skip
...
Tick N: A goal 2 completes → record result
Tick N+1: Select B (idle, cooldown elapsed) → dispatch B goal 1 → [executing]
...
Tick M: B goal 1 completes → advance says CYCLE_DONE → B enters cooldown
Tick M+1: No task selected (A idle/cooldown, B cooldown, C cooldown)
          → OODA pipeline runs → user chat processed → triage checks
Tick M+2: Select C (idle, cooldown elapsed) → dispatch C goal 1
...
```

The agent remains responsive because:
1. Tasks yield between goals (each goal = one tick of dispatch)
2. OODA runs when no task needs dispatch
3. User-initiated goals can preempt by entering the text_input_queue
   and being classified by the OODA pipeline as a goal command

---

## Task Manager UI Integration

The Task Manager displays:

| Field | Source |
|-------|--------|
| Execution state | `cycle_state` (idle/running) + cooldown check |
| Last run | `last_executed` timestamp |
| Next eligible | `last_executed + cooldown_seconds` |
| Executions | `execution_count` |
| Current cycle | `cycle_goals_completed` list |
| Cycle progress | Number of goals completed this cycle |
| Budget remaining | From autonomy budget tracker |

Actions available:
- **Run now**: Reset cooldown, set `cycle_state = "idle"` — eligible next tick
- **Pause**: Set `cycle_state = "paused"` — excluded from selection
- **Resume**: Set `cycle_state = "idle"` — re-enters selection pool
- **Abandon**: Terminal state, triggers distillation
- **Delete**: Remove task and all artifacts
- **Edit cooldown**: Adjust `cooldown_seconds`

---

## Cooldown Defaults

Fixed values, configurable per-task at creation time:

| Task origin | Default cooldown |
|------------|-----------------|
| Seed concern (health monitoring) | 3600s (1 hour) |
| Seed concern (workspace maintenance) | 7200s (2 hours) |
| Derived concern (event-triggered) | 1800s (30 min) |
| User-initiated task | 0s (run on demand) |

---

## Establishment → Operational Transition

When `_complete_task_wip` runs (task establishment complete):

1. Synthesize operational context from establishment findings
2. Set `status = "active"`, `lifecycle = "operational"`
3. Initialize `cycle_state = "idle"`, `cycle_goals_completed = []`,
   `cycle_findings = []`
4. Set `cooldown_seconds` based on task origin/type
5. **Do NOT create a scheduled goal** — the tick loop manages execution
6. Clear `self.active_task_wip` — establishment is done
7. Task is now in the active pool for round-robin dispatch

---

## Resolved Design Decisions

1. **Establishment budget**: Not budgeted. User approved the establishment.
   Establishment can be interrupted via Task Manager if excessive.

2. **Preemption**: No. A goal is the basic unit of scheduling. Once dispatched,
   it runs to completion. The planner supports interrupts but not resumption.
   Between goals, other tasks and user events get their turn.

3. **Step synthesis**: Dynamic, not pre-planned. The operational advance LLM
   decides the next goal based on accumulated cycle state — same pattern as
   the incremental planner deciding the next step. No fixed `operational_steps`
   list.

4. **Cooldown tuning**: Fixed per-task. KISS. May revisit with adaptive
   cooldowns later if needed.
