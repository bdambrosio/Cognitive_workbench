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
- **Execution state**: idle / pending / running / cooldown
- **WIP chain**: a sequence of goals (steps) that constitute one execution cycle

A task is NOT a single goal. A task may require multiple goals per execution
cycle (e.g., "check health, update digest, report anomalies" = 3 steps).

### Goal (Step)

A goal is a single executable unit — one planner run. Goals are the atoms of
execution. During task establishment, goals are milestones. During operational
execution, goals are steps in the task's recurring work.

### Execution Cycle

One complete run of an active task. May involve 1-N goals executed sequentially.
When the last goal in the cycle completes, the task returns to idle and its
cooldown timer starts.

---

## Task States

```
                    +-----------+
                    | proposed  |  (triage created, awaiting approval)
                    +-----+-----+
                          | approve (Task Manager UI)
                    +-----v-----+
                    |establishing|  (milestone loop running)
                    +-----+-----+
                          | _complete_task_wip
                    +-----v-----+
              +---->|   idle    |  (active, waiting for next execution)
              |     +-----+-----+
              |           | _tick selects this task (round-robin)
              |     +-----v-----+
              |     |  pending  |  (selected, goal about to be dispatched)
              |     +-----+-----+
              |           | goal dispatched to thread
              |     +-----v-----+
              |     |  running  |  (goal executing on worker thread)
              |     +-----+-----+
              |           | goal completes
              |           |
              |     +-----v-----+
              |     | cooldown  |  (minimum interval before next run)
              |     +-----+-----+
              |           | cooldown expires
              +-----------+
```

Terminal states: `completed`, `archived`, `abandoned`

---

## The Tick Loop: Task-Aware Execution

The `_main_loop_tick` currently has this structure:

```
1. Goal completion check
2. Task WIP advancement (if establishing)
3. Ask reply routing
4. Skip if goal running
5. OODA pipeline
```

The revised structure for multi-task execution:

```
1. Goal completion check
   - If goal thread done: record result, update executing task state
   - Clear running state

2. Task execution dispatch (if no goal running)
   - Select next eligible task (round-robin among idle tasks past cooldown)
   - Check autonomy budget (skip if exhausted)
   - Dispatch next goal/step for selected task
   - Set task state to running

3. Task establishment advancement (if establishing task exists and no goal running)
   - Only one task can be establishing at a time
   - Submit next milestone goal

4. Ask reply routing (if goal blocked on ask)

5. If goal running: return (skip OODA)

6. OODA pipeline (only when truly idle — no tasks running, no establishment)
   - Observe -> Orient -> Decide -> Act
   - Triage runs here (idle tick path)
   - User chat/events processed here
```

### Key changes from current:

- **Step 2 is new**: active tasks get dispatched from the tick loop, not from
  the goal scheduler. The tick loop owns task execution.
- **Round-robin selection**: each tick picks the next eligible task, cycling
  through all active tasks fairly. No task starves another.
- **Budget check at dispatch time**: if autonomy budget is exhausted, no
  autonomous task runs this tick. User-initiated goals bypass the budget.
- **OODA only when idle**: the OODA pipeline (including user chat) runs only
  when no task is executing. This preserves responsiveness — tasks yield
  between goals, and the agent can respond to the user in those gaps.

---

## Task Selection: Round-Robin with Cooldowns

### Data Structures

Each active task tracks:

```python
{
    "execution_state": "idle",       # idle | pending | running | cooldown
    "last_executed": "ISO timestamp",
    "cooldown_seconds": 3600,        # minimum interval between executions
    "execution_count": 0,
    "priority": "normal",            # normal | high (user-initiated get high)
}
```

### Selection Algorithm

```
eligible_tasks = [
    t for t in active_tasks
    if t.execution_state == "idle"
    and (now - t.last_executed) > t.cooldown_seconds
    and autonomy_budget_remaining > 0
]

# Round-robin: sort by last_executed ascending (stalest first)
eligible_tasks.sort(key=lambda t: t.last_executed or "")

if eligible_tasks:
    next_task = eligible_tasks[0]
    dispatch(next_task)
```

This ensures:
- Tasks that haven't run in the longest time go first
- Cooldowns prevent rapid re-execution
- Budget gate prevents runaway autonomous work
- A task that just finished goes to the back of the queue

### Cooldown Defaults (configurable per-task)

| Task origin | Default cooldown |
|------------|-----------------|
| Seed concern (health monitoring) | 3600s (1 hour) |
| Seed concern (workspace maintenance) | 7200s (2 hours) |
| Derived concern (event-triggered) | 1800s (30 min) |
| User-initiated task | 0s (run immediately when selected) |

---

## Execution Cycle: Multi-Step Tasks

A task's operational execution may require multiple sequential goals. The
task WIP stores an `operational_steps` field — either a single goal text
(simple task) or a list of steps (complex task).

### Simple Task (single goal)

```python
operational_steps = [
    {"goal_text": "Run check-health and update check-health-digest", "status": "pending"}
]
```

Each execution cycle runs the one goal, records the result, returns to idle.

### Complex Task (multi-step)

```python
operational_steps = [
    {"goal_text": "Fetch new papers from HuggingFace daily feed", "status": "pending"},
    {"goal_text": "Summarize and append to research-digest note", "status": "pending"},
    {"goal_text": "Check if any papers match active user concerns", "status": "pending"},
]
```

Each tick dispatches the **next pending step**. When step 1 completes, the
task remains `running` but yields — next tick may dispatch step 2 (or may
dispatch a different task's step if round-robin selects differently).

This is the key insight: **the tick loop dispatches one goal per tick, across
all tasks**. A 3-step task takes 3 ticks to complete one cycle. Between each
tick, other tasks (or user events) can interleave. No task monopolizes the
executive.

### Step Completion

When a step goal completes:

1. Mark step as completed in the task's `operational_steps`
2. If more pending steps remain: task stays `running`, next step dispatches
   on a future tick (after round-robin considers other tasks)
3. If all steps completed: execution cycle done
   - Record in `execution_history`
   - Increment `execution_count`
   - Update `last_executed`
   - Transition to `cooldown`
4. If step fails: record failure, optionally retry or skip depending on
   task configuration

---

## Task Establishment vs Task Execution

These are two distinct modes that share the tick loop but have different
mechanics:

| Aspect | Establishment | Execution |
|--------|--------------|-----------|
| State | `in_progress` | `active` (idle/pending/running/cooldown) |
| Goal source | `_advance_task_wip` LLM decision | `operational_steps` list |
| Concurrency | One at a time | Multiple tasks, round-robin |
| Phase gating | specification -> capability_evaluation -> etc. | No phases |
| Budget | Not budgeted (user approved it) | Budgeted (autonomous) |
| Yield | Does not yield between milestones | Yields between steps |

### Establishment completion

When `_complete_task_wip` runs, instead of creating a scheduled goal, it:

1. Sets `status = "active"`, `execution_state = "idle"`
2. Populates `operational_steps` from the synthesized goal
3. Sets `cooldown_seconds` based on task type / triage recommendation
4. The task is now in the active pool for round-robin dispatch
5. No scheduled goal is created — the tick loop manages execution directly

---

## Integration with Concern Triage

The triage system creates proposed tasks. When approved:

1. Task enters establishment (milestone loop)
2. Establishment determines what the task does and what infrastructure it needs
3. `_complete_task_wip` synthesizes `operational_steps` and `cooldown_seconds`
4. Task enters the active pool

The concern remains linked. After each execution cycle:

- If the task succeeded and the concern type is non-recurring: mark concern resolved
- If the task is recurring (seed concern): concern stays active, task re-executes
  after cooldown

---

## Budget Integration

The autonomy budget gates task dispatch, not task existence:

```
if task.origin == "autonomous" and budget_remaining <= 0:
    skip  # don't dispatch this tick, try again later

if task.origin == "user":
    always dispatch  # user-initiated tasks bypass budget
```

Budget is consumed when a goal actually executes (tracked by wall-clock time
of goal thread execution). Proposing, approving, and establishing tasks do
not consume budget — only operational execution does.

---

## Interaction with User Events

The OODA pipeline processes user chat, sensor events, and other inputs. It
runs only when no goal is executing:

```
Tick: [task goal runs] → [goal completes] → [next tick: OODA runs] → [next tick: another task goal]
```

This means user messages are processed in the gaps between task goal
executions. With 3 active tasks each taking ~30 seconds per goal:

- Task A step 1 runs (30s)
- Gap: OODA tick processes user message, triage runs (instant)
- Task B step 1 runs (30s)
- Gap: OODA tick
- Task C step 1 runs (30s)
- Gap: OODA tick
- Task A step 2 runs (if multi-step)
- ...

The agent remains responsive to the user even with multiple active tasks,
because tasks yield between steps and the OODA pipeline runs in the gaps.

If a user sends a high-priority message (e.g., a goal command), the OODA
pipeline processes it on the next gap tick and can interrupt or preempt
autonomous task execution.

---

## Task Manager UI Integration

The Task Manager displays:

| Field | Source |
|-------|--------|
| Status | `execution_state` (idle/pending/running/cooldown) |
| Last run | `last_executed` timestamp |
| Next eligible | `last_executed + cooldown_seconds` |
| Executions | `execution_count` |
| Current step | `operational_steps[current_index].goal_text` |
| Step progress | `completed_steps / total_steps` |
| Budget remaining | From goal scheduler budget tracker |

Actions available:
- **Run now**: Override cooldown, dispatch immediately on next tick
- **Pause**: Set `execution_state = "paused"`, excluded from selection
- **Resume**: Set `execution_state = "idle"`, re-enters selection pool
- **Abandon**: Terminal state, triggers distillation
- **Delete**: Remove task and all artifacts
- **Edit cooldown**: Adjust `cooldown_seconds`

---

## Migration Path

The current implementation uses the goal scheduler for operational task goals.
The migration to tick-loop-managed execution:

1. **Phase 1** (current): Tasks create scheduled goals via `_complete_task_wip`.
   Goal scheduler auto-proceeds them. This works but has fairness and
   responsiveness issues.

2. **Phase 2** (this spec): `_complete_task_wip` populates `operational_steps`
   instead of creating a scheduled goal. The tick loop manages dispatch
   directly. Goal scheduler remains for simple user goals only.

3. **Phase 3** (future): Multi-step tasks, step-level error handling,
   conditional branching in operational steps.

---

## Open Questions

1. **Establishment budget**: Should task establishment consume autonomy budget?
   Currently it doesn't (the user approved it). But a long establishment with
   many milestones could monopolize the LLM.

2. **Preemption**: Can a user goal preempt a running autonomous task goal?
   Currently no — goals run to completion on their thread. Preemption would
   require interrupt support in the planner.

3. **Step synthesis**: Who decides the `operational_steps` list? Currently
   `_complete_task_wip` synthesizes a single goal. Multi-step synthesis
   would need a richer prompt or a separate planning phase.

4. **Cooldown tuning**: Should cooldowns be static or adaptive? A task that
   keeps finding nothing to do could lengthen its cooldown. A task that
   detects anomalies could shorten it.
