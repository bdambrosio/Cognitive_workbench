# Spec: Deeper Agent Self-Awareness

## Problem

The agent (Jill) lacks a functional self-model. When asked how she would organize sustained autonomous activity, she produces generic AI-assistant patterns rather than reasoning about her actual operational mechanisms. Two things are missing:

1. **Runtime state & dynamics** — variables and mechanisms that represent what the agent *is doing*, what it *can do*, and the temporal dynamics of its operation (some exist, some don't)
2. **Planner-visible self-model** — extraction and rendering of that state into the OODA snapshot / reflective context so the planner can reason about it during goal execution

The goal is to give the agent enough self-knowledge that when it reasons about how to organize activity, it reasons in terms of its actual capabilities and operational dynamics — including the concern → task → goal hierarchy that structures sustained activity.

### The three-level hierarchy

The architecture has (or should have) three levels of intentional structure:

- **Concerns** (what matters): representational, evaluated via LLM patches, weight reflects importance. User concerns and derived concerns.
- **Tasks** (what I'm committed to doing about it): persistent operational entities that manage sequences of goals, accumulate state across executions, and make judgment calls about when and what to execute next. Each task is linked to a concern.
- **Goals** (what I'm doing right now): atomic units of execution. From the OODA loop's perspective, a goal is a mostly non-interruptible primitive. The planner generates and executes plans step-by-step within a goal.

Currently, the task layer exists only during establishment (the WIP milestone loop) and collapses into a single scheduled goal once established. This spec makes tasks visible as first-class entities in the agent's self-model, and lays groundwork for tasks to persist as ongoing operational entities beyond establishment.

---

## Architecture Context

### Existing pieces (read these files first)

- `src/executive_node.py` — OODA loop, task WIP milestone loop, goal scheduling, concern activations
- `src/ooda_living_state.py` — persisted orientation state (concern activations, goal field, transitions, epistemic markers)
- `src/ooda_snapshot_renderer.py` — renders living state + concerns into markdown for planner injection
- `src/derived_concern_model.py` — agent-originated concerns with LLM patch system
- `src/incremental_planner.py` — Stage 1 planning, reflective state injection point (search for `render_reflective_snapshot`)
- `src/goal_scheduler.py` — timer-based goal auto-proceed
- `src/tools/generate-reflective-note/Skill.md` — existing tool for state-aware generation
- `src/tools/manage-goals/Skill.md` — existing tool for goal CRUD

### Injection path

The planner receives self-model information through this chain:
1. `executive_node._run_goal_on_thread()` stashes `_ooda_living_state`, `_derived_concerns_snapshot`, `_user_concerns_snapshot` on the executor
2. `incremental_planner.py` calls `render_reflective_snapshot()` from `ooda_snapshot_renderer.py` and appends it to `_planner_reflective_state`
3. The reflective state is injected into the planner prompt as context

Additionally, `_build_agent_state_block()` in `executive_node.py` provides an authoritative state block to the system prompt for character-level reasoning.

---

## Level 1: Runtime State & Dynamics

These are actual variables and lightweight mechanisms that need to exist so the self-model has something real to report. Organized by what exists vs. what's new.

### 1A. Already exists — needs exposure, not implementation

These are already tracked in `executive_node.py` or related modules but are not currently surfaced to the planner in a coherent way.

| State element | Where it lives | Current status |
|---|---|---|
| Concern activations + trends | `_character_concern_activations` dict, living state | Surfaced in snapshot but without self-model framing |
| Derived concerns (active/surfaced) | `DerivedConcernModel.concerns` | Surfaced in snapshot |
| Scheduled goals + schedule modes | `_all_scheduled_goals()` | Partially surfaced (goal field in living state) |
| Goal scheduler status | `GoalScheduler.get_status()` | NOT surfaced to planner |
| Task WIP state | `active_task_wip`, `_read_task_wip()` | NOT surfaced to planner |
| Available tools catalog | `self.available_tools` in executor | NOT surfaced (planner sees selected tools in Stage 1.5, not full catalog) |
| Sensor configuration | `self.sensors` in executive_node | NOT surfaced |
| Active sensors + last fire times | sensor instances in executive_node | NOT surfaced |
| User concern model | `self.user_concern_model` | Surfaced in living state |
| Execution mode (step/run) | `self.execution_paused`, `self.execution_mode` | NOT surfaced |
| Action history (recent) | `self.action_history` | NOT surfaced to planner |

### 1B. New state — needs implementation

These are new runtime variables that don't currently exist but are needed for the agent to have a meaningful self-model of its temporal dynamics and operational capacity.

#### 1B-i. Task as a persistent entity (extending Task WIP beyond establishment)

**Purpose**: Tasks are the bridge between concerns (what matters) and goals (what to do now). Currently, task WIP notes exist only during establishment and get archived when the operational scheduled goal is created. This change makes tasks persist as ongoing entities that the agent can reason about.

**Location**: Task WIP notes in `executive_node.py` — extend the data model and lifecycle.

**Extended task data model** (the WIP note content, persisted as JSON in a named Note):

```python
{
    "task_id": "task_2",
    "intention": "Keep Bruce updated on AI developments",
    "linked_concern_id": "dconcern_3",           # the concern this task services
    
    # Lifecycle
    "lifecycle": "operational",                    # establishing | operational | paused | completed
    "phase": "operational",                        # during establishing: specification/capability_eval/etc.
                                                   # during operational: just "operational"
    
    # Establishment history (preserved for context)
    "establishment_milestones": [...],             # the existing milestones_completed list
    "establishment_findings": [...],               # the existing accumulated_findings list
    
    # Operational state (new — accumulates across goal executions)
    "execution_history": [                         # ring buffer, last N executions
        {
            "goal_id": "goal_5",
            "goal_text": "Scan arXiv for papers on reasoning and agents",
            "timestamp": "2025-03-21T09:15:00",
            "outcome": "success",
            "summary": "Found 3 papers: ...",      # compressed result
            "duration_minutes": 4
        }
    ],
    "working_notes": "_task_2_working_memory",     # named Note for accumulated findings
    "operational_goal_id": "goal_5",               # the current operational scheduled goal
    "schedule_mode": "daily",                      # inherited from scheduled goal
    "last_executed": "2025-03-21T09:15:00",
    "execution_count": 7,
    
    # Timestamps
    "created": "2025-03-15T10:00:00",
    "updated": "2025-03-21T09:20:00",
}
```

**Lifecycle transition**: When `_complete_task_wip()` runs, instead of archiving the WIP note:
1. Set `lifecycle` to `"operational"`
2. Move establishment milestones to `establishment_milestones`
3. Initialize `execution_history` as empty list
4. Record `operational_goal_id` (the scheduled goal that was created)
5. Keep the WIP note alive as the task's persistent state

**Post-execution update**: When the operational scheduled goal completes (detected in goal completion handler), update the task:
1. Append to `execution_history` (ring buffer, keep last ~20)
2. Update `last_executed`, `execution_count`
3. Optionally append key findings to the working notes

**Important scope note**: This phase does NOT implement operational-mode advance (where the task decides what *different* goal to run next time). That's a future mechanism. For now, the task persists as a readable entity with execution history, which is what the self-model needs to render. The scheduled goal continues to fire mechanically as it does today.

#### 1B-ii. Task-to-concern linkage

**Purpose**: Connect tasks to the concerns that motivated them, creating the full concern → task → goal chain visible in the self-model.

**Location**: Task data model (above) has `linked_concern_id`. Scheduled goals get `task_id` instead of the current `task_wip_id` (or in addition to it during transition).

```python
# Scheduled goal gets task linkage:
{
    "goal_id": "goal_5",
    # ... existing fields ...
    "task_id": "task_2",  # the task this goal belongs to (replaces/supplements task_wip_id)
}
```

**Derived concern update**: When a task-linked goal completes, the derived concern model can update the concern's state based on the task's execution outcome. This replaces the standalone concern service tracking fields — the task *is* the service record.

#### 1B-iii. Operational capability summary

**Purpose**: A compact, relatively static description of what the agent can actually do — tool categories, sensing capabilities, execution modes. This is the "what I am" part of the self-model vs. the "what I'm doing" part.

**Location**: New named Note `_agent_capabilities`, generated once at startup and refreshed when tools/sensors change.

**Content structure** (stored as markdown in a named Note):

```markdown
## Execution Model
- OODA loop: continuous observe-orient-decide-act cycle
- Three-level intentional hierarchy: concerns (what matters) → tasks (commitments) → goals (atomic action)
- Goal execution: plans are generated and executed step-by-step with real-time judgment
- Tasks: persistent entities that manage goal sequences, accumulate state across executions
- Task lifecycle: establishing (multi-milestone setup) → operational (recurring execution)
- Scheduled goals: can run manually, daily (at specific time), or auto (when eligible)
- One goal at a time; OODA loop pauses event processing during goal execution

## Available Tool Categories
- Information: web-search, load, list-collection, search (semantic)
- Creation: create-note, generate-note, generate-reflective-note
- Communication: say, ask (interactive dialog with user)
- Goal management: manage-goals (create/list/update/delete scheduled goals)
- Shell: execute-shell (bash commands)
- [world-specific tools listed here]

## Active Sensors
- [sensor_name]: [type], [disposition], last fired [timestamp]

## Scheduling
- Goal scheduler: [enabled/disabled], interval [N]s
- [N] scheduled goals: [breakdown by schedule_mode]

## Constraints
- Tasks accumulate state but do not yet autonomously decide what different goal to run next
- Goals come from user input, scheduler triggers, or sensor events
- No persistent memory across restarts beyond named Notes, tasks, and scheduled goals
- Web access via web-search tool only (no persistent connections)
```

**Generation**: Build this in `executive_node.__init__()` after tools, sensors, and scheduler are initialized. Regenerate on tool/sensor changes. Store as a named Note via `_write_named_note()`.

---

## Level 2: Planner-Visible Self-Model

This is what the planner actually sees during goal execution. All Level 1 state flows through here.

### 2A. New section in `ooda_snapshot_renderer.render_reflective_snapshot()`

Add a new `## Operational Self-Model` section to the reflective snapshot. This section is the agent's self-knowledge, rendered fresh each time a goal starts.

**New function** in `ooda_snapshot_renderer.py`:

```python
def render_self_model_section(
    capabilities_note_content: str,
    scheduler_status: dict,
    tasks: Sequence[Dict],              # all task notes (establishing + operational)
    derived_concerns: Sequence[Dict],
    scheduled_goals: Sequence[Dict],
    recent_action_summary: str,
    sensor_status: Sequence[Dict],
) -> str:
    """Render the operational self-model section for planner consumption."""
```

This function produces a markdown section with these subsections:

#### 2A-i. "How I Work" (from capabilities note)

Compact version of the `_agent_capabilities` note. Not the full thing — a 3-5 line summary of execution model and key constraints. This is relatively stable and can be truncated aggressively.

```
### How I Work
I operate via a continuous OODA loop. I pursue sustained activities through tasks —
persistent commitments linked to concerns. Each task accumulates state across executions
and manages a sequence of goals. Goals are my atomic unit of action (planned and executed
step-by-step). I can have multiple active tasks but execute only one goal at a time.
I have [N] tools across [categories]. I persist state in named Notes.
```

#### 2A-ii. "What I'm Doing" (dynamic operational state)

```
### Current Operational State
- Scheduler: enabled, 60s interval, idle
- Active tasks: 2 operational, 1 establishing
- Running goal: "Summarize latest AI papers" (step 4/12, task: monitor_ai_developments)
- Execution mode: run (not paused)
```

Source: `GoalScheduler.get_status()`, task notes, current goal from living state, `execution_mode`.

#### 2A-iii. "My Commitments" (active tasks as the concern-goal bridge)

This is the central new section. Tasks are the persistent entities that make sustained activity visible to the planner.

```
### My Commitments (Active Tasks)
- **monitor_ai_developments** [operational, linked concern: dconcern_3]
  Intention: Keep Bruce updated on AI developments
  Schedule: daily @ 09:00 | Last executed: 18h ago → success
  Execution history: 7 runs, last found 3 papers on reasoning
  Working memory: _task_2_working_memory (12 accumulated entries)

- **establish_obsidian_sensing** [establishing, phase: capability_evaluation]
  Intention: Set up Obsidian clipper integration for life context
  Last milestone: tested clipper access (success)
  Next: infrastructure_setup phase

- dconcern_5 "maintain_bruce_context" [active, weight=0.5] — NO TASK
  (Concern exists but no task has been created to service it)
```

**Source**: Task notes read from `resource_manager.named_notes` (all notes matching `_task_*` pattern), joined with derived concerns via `linked_concern_id`. Concerns without tasks are shown separately to let the planner reason about gaps.

**Key insight for the planner**: The juxtaposition of "tasks with execution history" and "concerns without tasks" gives the agent a concrete way to reason about what it's committed to vs. what it's merely aware of. When asked "how would you organize AI monitoring," the planner can see whether a task already exists for that concern, what its execution history looks like, and what's missing.

#### 2A-iv. "My Sensing" (sensor awareness)

```
### Active Sensors
- obsidian_clipper: plan-type, disposition=inform, last fired 2h ago
- desktop_activity: code-type, disposition=trigger:review_work, last fired 30min ago
```

Source: sensor instances from executive_node.

### 2B. Integration point

In `incremental_planner.py`, where `render_reflective_snapshot()` is currently called (search for `_ooda_living_state`), gather the additional inputs and pass them through:

```python
# After existing snapshot rendering:
try:
    from ooda_snapshot_renderer import render_self_model_section
    self_model = render_self_model_section(
        capabilities_note_content=_get_capabilities_note(executor),
        scheduler_status=_get_scheduler_status(executor),
        tasks=_get_all_tasks(executor),
        derived_concerns=_dc,
        scheduled_goals=_get_scheduled_goals(executor),
        recent_action_summary=_get_recent_actions(executor),
        sensor_status=_get_sensor_status(executor),
    )
    if self_model:
        base_state += f"\n\n{self_model}"
except Exception:
    pass
```

The helper functions (`_get_capabilities_note`, `_get_all_tasks`, etc.) are thin wrappers that safely access executor/executive_node attributes. `_get_all_tasks()` reads all named Notes matching `_task_*` and parses their JSON content. They should fail silently and return empty defaults.

### 2C. Token budget

The reflective snapshot is already injected into the planner context. The self-model section should be budget-conscious:

- "How I Work": ~100 tokens (semi-static, compress hard)
- "Current Operational State": ~60 tokens
- "My Commitments (Active Tasks)": ~60 tokens per task + ~20 per unserviced concern, typically 2-5 tasks = ~200-350 tokens
- "My Sensing": ~20 tokens per sensor, typically 2-4 = ~60 tokens

**Total budget**: ~550 tokens max. This is acceptable given the planner's context window.

If token pressure becomes an issue, "How I Work" can be omitted after the first goal execution in a session (the agent has already seen it).

---

## Implementation Order

### Phase 1: Surface existing state + task persistence (minimal new mechanisms)

1. Add `render_self_model_section()` to `ooda_snapshot_renderer.py`
2. Wire it into `incremental_planner.py` at the reflective state injection point
3. Surface: scheduler status, execution mode, sensor list
4. For "How I Work", start with a hardcoded template string in the renderer that's parameterized by tool count, sensor count, scheduler state, and task count
5. **Extend task WIP lifecycle**: In `_complete_task_wip()`, instead of archiving the WIP note, transition it to `lifecycle: "operational"`. Move establishment milestones to `establishment_milestones`, initialize empty `execution_history`, record `operational_goal_id`. Keep the note alive.
6. **Read all task notes** for the renderer: Add `_get_all_tasks()` helper that finds named Notes matching `_task_wip_*` (or rename to `_task_*`) and parses their JSON. This gives the renderer access to both establishing and operational tasks.
7. Render the "My Commitments" section showing task lifecycle state, linked concerns, and (for now) basic scheduling info.

**Validation**: 
- Establish a task. Verify the WIP note transitions to `lifecycle: "operational"` instead of being archived.
- Run a goal like "Describe how you would organize a daily AI monitoring task" and check the planner trace to see if Stage 1 reasoning references the task in the self-model.

### Phase 2: Task execution tracking

8. Add `task_id` field to scheduled goal schema (replacing or supplementing `task_wip_id`)
9. In the goal completion handler, when a task-linked goal completes, update the task note: append to `execution_history`, update `last_executed`, `execution_count`
10. Add `linked_concern_id` to task data model; wire it through from task establishment (if the task was initiated from a concern) or allow the planner to set it via a tool
11. Update `DerivedConcernModel.update_from_goal_completion()` to use task linkage for concern state updates
12. Update `render_self_model_section()` to show execution history summaries and concern-gap detection (concerns without tasks)

**Validation**: Establish a task, let its scheduled goal run, verify `execution_history` accumulates in the task note. Verify the planner sees the execution history in the self-model.

### Phase 3: Capabilities note

13. Implement `_generate_capabilities_note()` in `executive_node.py`
14. Call it after init, store as named Note `_agent_capabilities`
15. Wire into `render_self_model_section()` to replace the hardcoded template
16. Add regeneration trigger when tools or sensors change

**Validation**: Check the named Note content after startup. Verify it updates if a sensor is added.

---

## What This Does NOT Do

This spec intentionally avoids:

- **Operational-mode task advance**: The mechanism where a task, post-establishment, decides what *different* goal to run next time based on accumulated state. The `_advance_task_wip` prompt pattern is the natural foundation, but switching from establishment-mode to operational-mode advance is a separate spec. This spec makes tasks persist and accumulate execution history, which is the prerequisite.
- **Concern-driven task creation**: The mechanism where a concern's activation crosses a threshold and spawns a new task. This spec makes the gap between "concern with task" and "concern without task" visible in the self-model, which informs the design of that mechanism.
- **Temporal activation pressure**: Making concern activations increase over time when unserviced. This is a dynamics change that should be informed by how the agent actually uses the task-level self-model.
- **Cross-task coordination**: Multiple tasks competing for the single goal execution slot, priority negotiation between tasks. Deferred until real scenarios demand it.
- **New tools**: No new tools are added. The agent reasons about its existing capabilities and task commitments more accurately. (A `manage-tasks` tool is a natural future addition but not needed for self-awareness.)

The philosophy: give the agent self-knowledge first, see what it does with it, then build the mechanisms it reaches for. Specifically, once the planner can see "I have a task for this concern, it last ran 18 hours ago, its execution history shows X" — the agent may start reasoning about what it *would* do differently next time, which tells us what the operational advance mechanism needs to support.

---

## Files to Modify

| File | Changes |
|---|---|
| `src/ooda_snapshot_renderer.py` | Add `render_self_model_section()` function with task rendering |
| `src/incremental_planner.py` | Wire self-model into reflective state injection, add `_get_all_tasks()` helper |
| `src/executive_node.py` | Extend `_complete_task_wip()` for lifecycle transition, add `task_id` to scheduled goal schema, update goal completion handler to write back to task notes, `_generate_capabilities_note()`, expose scheduler/sensor state for renderer |
| `src/derived_concern_model.py` | Update `update_from_goal_completion()` to use task linkage for concern state |
| `src/ooda_living_state.py` | No changes expected |
| `src/goal_scheduler.py` | No changes expected (already has `get_status()`) |

## Files to Read (context for implementer)

- `src/tools/manage-goals/Skill.md` — scheduled goal schema
- `src/tools/generate-reflective-note/Skill.md` — existing reflective generation pattern
- `src/sensors/` — sensor base classes and configuration
- `scenarios/*.yaml` — character config including sensor definitions
- Look specifically at `_complete_task_wip()`, `_advance_task_wip()`, `_begin_task_establishment()`, and the goal completion handler sections of `executive_node.py` for the existing task lifecycle code
