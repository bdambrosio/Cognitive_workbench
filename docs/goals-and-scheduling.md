# Goals & Scheduling

## Submitting Goals

Goals are the primary way to drive the agent. To submit a goal, type in the web UI text input with the `goal:` prefix:

```
goal: Find recent papers on multi-agent coordination and summarize key findings
```

The `goal:` prefix is detected by `_is_goal_cmd()` in the Executive Node. Text without this prefix is treated as a conversational message (processed via envisioning and dialog handling).

### What happens when you submit a goal

1. **Parse**: The text after `goal:` is extracted and parsed into a `Goal` object with a name, description, and termination conditions
2. **Publish**: The goal is announced via Zenoh to `cognitive/{character}/goal`
3. **Plan**: The Incremental Planner is invoked — it generates and executes a plan step-by-step (see [Architecture](architecture.md))
4. **Result**: The plan result (success/failure, artifacts created, quality status) is published and persisted as a scheduled goal entry

## Scheduled Goals

Every submitted goal becomes a **scheduled goal** — a persistent entry stored as a Note with the prefix `_scheduled_goal_`. All scheduled goals are grouped in the `_scheduled_goals` collection.

Each scheduled goal has:

| Field | Description |
|-------|-------------|
| `goal_id` | Unique identifier |
| `name` | Editable display name |
| `goal_text` | The original goal text |
| `status` | `ready`, `executing`, `completed`, `blocked`, `abandoned` |
| `schedule_mode` | How/when the goal should execute (see below) |
| `run_at` | ISO time for daily scheduling (e.g., `"14:30:00"`) |
| `cached_plan_actions` | Previously executed steps (for re-use) |

### Schedule Modes

| Mode | Behavior |
|------|----------|
| **manual** | Goal only executes when you click "Proceed" in the UI |
| **auto** | Goal auto-proceeds to the next step after each step completes |
| **recurring** | Goal repeats its execution cycle automatically |
| **daily** | Goal runs once per day at a specified time |

You can change the schedule mode from the **Schedule** tab in the character sidebar.

## Daily-at-Time Scheduling

When a goal's schedule mode is set to **daily**, a time picker appears in the UI. Set the desired run time (24-hour format). The goal will execute once per day at that time.

### How it works

The **GoalScheduler** (`goal_scheduler.py`) is a daemon thread that runs in the background:

1. It checks for eligible goals every 15 seconds (configurable via `interval` in the scenario YAML)
2. A goal is eligible when:
   - Its status is `ready`
   - For **daily** mode: the current time >= `run_at` and it hasn't already run today
   - For **auto** / **recurring** mode: the mode is active
3. Only one goal executes at a time (a concurrency guard `_executing_task_id` prevents overlap)
4. When a goal completes, callbacks (`notify_step_completed`, `notify_task_terminal`) clear the execution guard, allowing the next eligible goal to start

### Configuring the scheduler

In your scenario YAML:

```yaml
characters:
  Jill:
    task_scheduler:
      enabled: true
      interval: 15   # check interval in minutes
```

The scheduler can also be enabled/disabled from the UI (Schedule tab controls).

## Goal Commands

Beyond submitting new goals, you can issue commands for existing scheduled goals:

| Command | Effect |
|---------|--------|
| `proceed <goal_id>` | Manually execute one step of a scheduled goal |
| `reuse <goal_id>` | Re-execute using the cached plan (skips re-planning) |
| `terminate <goal_id>` | Stop execution and mark as abandoned |
| `clear-cache <goal_id>` | Clear cached plan actions for the goal |

These can be typed in the text input or triggered via UI buttons in the Schedule tab.

## Plan Caching

When a goal executes successfully, its plan actions are cached in `cached_plan_actions`. On re-execution (via `reuse`), the system replays the cached steps instead of invoking the planner from scratch. This is useful for:

- **Recurring goals** where the same steps apply each time
- **Quick re-runs** when you want the same behavior without LLM cost
- **Debugging** to reproduce a specific execution sequence

Cached plans can be cleared with `clear-cache <goal_id>` if you want a fresh plan.

## Autonomous Execution

The UI provides execution controls:

- **Step**: Execute one planner iteration, then pause
- **Autonomous**: Run continuously — the agent keeps processing goals from its queue until interrupted
- **Stop**: Pause all execution and interrupt the current plan
- **Continuous Mode** (toggle): When a goal completes, automatically proceed to the next queued goal

In autonomous mode, the agent will work through its scheduled goals queue, executing eligible goals based on their schedule mode and timing.

## Goal Lifecycle Diagram

```
User types "goal: ..."
    │
    ▼
parse_and_set_goal()
    │
    ├── Create Goal object
    ├── Store as _scheduled_goal_ Note
    ├── Add to _scheduled_goals Collection
    │
    ▼
status: ready
    │
    ├── [manual] Wait for user "Proceed"
    ├── [auto]   TaskScheduler auto-proceeds
    ├── [daily]  TaskScheduler checks time
    │
    ▼
status: executing
    │
    ├── Invoke IncrementalPlanner
    ├── Execute plan steps
    ├── Cache successful actions
    │
    ▼
status: completed (or blocked/abandoned on failure)
    │
    ├── Persist result + metadata
    ├── Record scheduler event
    ├── [recurring] Reset to "ready" for next cycle
    └── Clear execution guard → next goal can start
```

## UI: Schedule Tab

The **Schedule** tab in the character sidebar shows:

- List of all scheduled goals with status badges
- **Mode dropdown** for each goal (manual / auto / recurring / daily)
- **Time picker** (appears only for daily mode)
- **Proceed / Terminate / Reuse / Clear Cache** buttons
- **Cached action count** for each goal
- **Scheduler status**: disabled, waiting, or running with current task ID
- **Scheduler events log**: recent scheduler activity

## Next

- [Architecture](architecture.md) — how the planner executes goals
- [Envisioning & Quality Control](envisioning-and-quality-control.md) — quality assurance on goal execution
- [UI Guide](ui-guide.md) — full UI reference
