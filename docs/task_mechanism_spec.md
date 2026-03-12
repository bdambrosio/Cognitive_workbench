# Task Mechanism: Design Specification

## Overview

A **task** is an ongoing concern for an agent — a persistent intention that activates periodically or in response to events, executes through a sequence of milestone goals, and accumulates state across activations. This mechanism unifies homeostatic monitoring, user-requested recurring work, bounded experiments, and task establishment itself.

The mechanism has two components:
1. **Milestone loop** — executes a task as a series of goals with adaptive sequencing
2. **Scheduled activation** — integrates completed tasks with the existing scheduled goal system

Task establishment is triggered explicitly by the user via a `task:` prefix (analogous to the existing `goal:` prefix). No automatic classification of user messages is needed.

---

## Architecture Context

### Existing components this builds on (do not replace)

- **OODA loop** (`_main_loop_tick` in `executive_node.py`): The outer Python loop that ticks every 200ms. It processes input, runs orient, submits goals, and waits for completion. It is Python — it cannot conduct conversations, but it can call `llm_generate` for short reasoning and submit goals to the planner.
- **Planner** (`incremental_planner.py`): Executes a single goal using a small number of steps. Has access to all tools including `say`, `ask`, `manage-goals`, `check-health`, and infospace operations. Can converse with the user via `say`/`ask`. Cannot run sub-goals or recursive planning.
- **Goal Scheduler** (`goal_scheduler.py`): Timer-based daemon that checks for eligible scheduled goals and injects synthetic proceed commands. Already supports `manual`, `auto`, `recurring`, and `daily` modes.
- **Scheduled Goals** (`manage-goals` tool): CRUD for persistent goal records stored as Notes in a `_scheduled_goals` collection. Each goal has `goal_id`, `goal_text`, `name`, `schedule_mode`, `run_at`, `status`, `last_result`, `primary_product`.
- **Infospace**: Notes, Collections, and Relations. Notes are the primary artifact. Named notes can be looked up by name. Collections group notes. The `persist` action makes notes survive restart. Goals remove transient (non-persisted) notes created during goal execution by the planner.
- **`llm_generate`**: Lightweight LLM call available from the OODA loop (Python level). Short reasoning, classification, goal formulation. No tool access. Fast.
- **`ask` tool**: Available in the planner. Blocks the planner thread and waits for the user's response, then continues execution with the response bound to the output variable. Enables multi-turn dialogue within a single goal.

### Two levels of execution

The OODA loop (Python) is the outer executive. It submits goals to the planner and waits for results. The planner executes goals using tool calls and LLM reasoning. The OODA loop cannot conduct conversations — only the planner can (via `say`/`ask`). The OODA loop can do short `llm_generate` calls for classification and goal formulation. These are two distinct levels; do not conflate them.

### How `ask` works (blocking model)

The `ask` tool in the planner **blocks the planner thread** until the user responds:

1. Planner calls `ask` with a question and an `out` variable
2. `_execute_ask` publishes the question to the UI and sets `awaiting_ask_response = True`
3. `_execute_ask` blocks on `executive_node._ask_response_queue.get(timeout=300)`
4. The main loop detects `awaiting_ask_response`, routes the next User message into `_ask_response_queue`
5. `_execute_ask` unblocks, creates a Note with the user's response, binds it to `out`, returns success
6. The planner continues from where it was — all bindings, step count, and reasoning context are preserved

If the user doesn't respond within the timeout (5 minutes), `ask` returns a failure so the planner can handle it gracefully (e.g., save progress to a Note and terminate). If the user clicks Interrupt, the queue receives a sentinel that causes `ask` to return failure.

This is critical for task establishment: the specification phase needs multi-turn dialogue within a single planner session to clarify requirements, and the planner must retain context across ask/response cycles.

---

## Component 1: Task Prefix and Establishment

### Trigger

The user explicitly requests task establishment with a `task:` prefix:

```
task: whenever we get an email from billpay@pge.com, forward it to Jane
```

This is analogous to `goal:` for one-time goals. No automatic classification is needed — the user decides when something is a task.

### Integration point

In `_process_text_input` (in `executive_node.py`), add a check alongside the existing `_is_goal_cmd` check:

```python
if source == 'User':
    if clean_input.strip().lower().startswith('task:'):
        task_text = clean_input[5:].strip()
        if self.active_task_wip:
            self._say_to_user("A task is already being established. Please wait for it to complete.")
            return
        if self._is_goal_running():
            self._say_to_user("A goal is currently running. Please wait for it to complete.")
            return
        self._begin_task_establishment(task_text)
        return
    elif _is_goal_cmd(clean_input):
        # existing goal handling...
```

### What `_begin_task_establishment` does

1. Create the task WIP Note with the user's message as `intention`
2. Persist the Note
3. Set `self.active_task_wip` to the Note name
4. The milestone loop picks it up on the next tick

---

## Component 2: Milestone Loop

### Core concept

A task-in-progress is executed as a series of **milestone goals**. After each milestone completes, the OODA loop uses `llm_generate` to decide the next milestone — advance, fall back, or declare completion. The task's accumulated state lives in an infospace Note that is progressively updated.

### Task-in-progress Note

A regular infospace Note with naming convention `_task_wip_<id>` where `<id>` is an incrementing counter (use the existing `_scheduled_goal_counter` pattern or a separate counter).

**Initial content** (created at task start):

```json
{
  "task_wip_id": "twip_1",
  "intention": "<the user's original request or task description>",
  "status": "in_progress",
  "phase": "specification",
  "milestones_completed": [],
  "current_milestone": null,
  "accumulated_findings": [],
  "created": "<ISO timestamp>",
  "updated": "<ISO timestamp>"
}
```

This Note is persisted immediately on creation.

**Updated after each milestone** by reading the Note, modifying it in Python, and writing it back. The `milestones_completed` array grows; `accumulated_findings` captures key results; `phase` advances through the task lifecycle.

### Python state on executive node

```python
self.active_task_wip = None  # Note name (e.g., "_task_wip_1") or None
self.active_task_wip_waiting = False  # True while a milestone goal is running
```

These are volatile — they exist for the OODA loop to know whether it's mid-task. On restart, the OODA loop can check for a `_active_task_wip` pointer Note to recover.

### Milestone loop in `_main_loop_tick`

Add a check after the goal-done-event processing block (after line ~1366), before the idle OODA loop:

```
# If we have an active task-in-progress and no goal is currently running:
if self.active_task_wip and not self.active_task_wip_waiting and not self._is_goal_running():
    self._advance_task_wip()
```

### `_advance_task_wip` method

This is the heart of the mechanism:

1. **Read the task WIP Note** from infospace (by name `self.active_task_wip`)
2. **Read the most recent milestone result** (if any — from the last completed goal's result/artifact)
3. **Call `llm_generate`** with the following prompt structure:

```
You are managing a multi-step task. Your job is to determine the next action.

TASK INTENTION:
<intention from Note>

CURRENT PHASE: <phase>

MILESTONES COMPLETED:
<list from Note>

ACCUMULATED FINDINGS:
<findings from Note>

MOST RECENT MILESTONE RESULT:
<result of the last goal that completed, or "None — this is the first milestone">

AVAILABLE PHASES: specification, capability_evaluation, infrastructure_setup, activation, complete

Determine what to do next. Your options are:
1. SUBMIT_GOAL: Submit a goal for the planner to execute. Provide the full goal text.
2. FALL_BACK: A previous milestone result was unexpected. Describe what to revisit.
3. COMPLETE: The task is fully established. Provide a summary.

Respond in this exact format:
ACTION: <SUBMIT_GOAL | FALL_BACK | COMPLETE>
PHASE: <which phase this belongs to>
GOAL_TEXT: <full goal text if SUBMIT_GOAL, or explanation if FALL_BACK/COMPLETE>
```

4. **Parse the response** and act:
   - **SUBMIT_GOAL**: Formulate and submit the goal to the planner. Set `self.active_task_wip_waiting = True`. Update the Note's `current_milestone`.
   - **FALL_BACK**: Update the Note to reflect the fallback (remove relevant completed milestones, adjust phase), then loop will re-enter `_advance_task_wip` on next tick.
   - **COMPLETE**: Call `_complete_task_wip()` (see below).

### Goal submission from the milestone loop

When the milestone loop decides to submit a goal, it uses the same mechanism as existing goal submission:

1. Create a scheduled goal with the goal text from the LLM
2. Add `task_wip_id` field to the scheduled goal record so the completion handler knows this is a milestone
3. Launch on worker thread via `_run_goal_on_thread`
4. Set `self.active_task_wip_waiting = True`

### Goal completion callback

When a goal completes (in `_set_scheduled_goal_result`), if the scheduled goal record contains a `task_wip_id` field:

1. Capture the goal result (from the planner's output)
2. Read the task WIP Note
3. Append the completed milestone to `milestones_completed`:
   ```json
   {
     "goal_text": "<what was submitted>",
     "result_summary": "<abbreviated result>",
     "timestamp": "<ISO>"
   }
   ```
4. If the goal result contains findings relevant to capability or feasibility, append to `accumulated_findings`
5. Write the updated Note back
6. Set `self.active_task_wip_waiting = False`
7. On the next tick, `_advance_task_wip` will fire again

### `_complete_task_wip` method

Called when the milestone loop determines the task is fully established:

1. Read the final task WIP Note
2. Extract the key fields needed for a scheduled goal:
   - `goal_text`: the recurring goal instruction (synthesized from the accumulated spec)
   - `name`: short display name
   - `schedule_mode`: determined during specification (e.g., "daily")
   - `run_at`: if applicable
3. Create the scheduled goal via `_upsert_scheduled_goal` (same as `manage-goals` create)
4. Copy key fields from the WIP Note into the scheduled goal record:
   - Add a `task_context_note` field pointing to the WIP Note name
   - Add any infrastructure references (e.g., collection names created during setup)
5. Update the WIP Note: set `status: "completed"`, add `scheduled_goal_id` reference
6. Clear `self.active_task_wip`
7. Use `say` to inform the user the task is established

**The WIP Note is retained** as a permanent reference. The scheduled goal record gets the operational fields it needs to execute, plus a back-pointer to the full WIP Note for context.

---

## Component 3: Scheduled Activation with Task Context

### Enhancement to scheduled goal execution

When a scheduled goal fires (via the GoalScheduler), if the goal record contains a `task_context_note` field:

1. Load the referenced WIP Note (now in "completed" status, serving as the task's context package)
2. Prepend the context to the goal text when submitting to the planner:

```
TASK CONTEXT (from task establishment):
<WIP Note content — findings, infrastructure references, constraints>

GOAL:
<the recurring goal text>
```

This gives the planner the rich context it needs — the fact that it should use collection `_pge_forwarding_log` to check for duplicates, that Jane's email is jane@example.com, etc. — without the planner having to rediscover any of this on each activation.

### Activation-level state (per-run)

Some tasks accumulate state across activations (the paper monitor remembering which papers it already reviewed, the temperature experiment accumulating results). For these, the task context Note can include a `run_state` field that gets updated after each activation:

```json
{
  "...existing WIP fields...",
  "run_state": {
    "papers_reviewed": ["arxiv:2026.12345", "arxiv:2026.12346"],
    "last_check": "2026-03-10T09:00:00Z"
  }
}
```

After each activation goal completes:
1. If the goal produced artifacts relevant to ongoing tracking, update the `run_state` in the WIP Note
2. This can be done by a brief `llm_generate` call that reads the goal result and decides what, if anything, to add to `run_state`
3. Or the goal itself can update the Note directly using infospace tools (if given the Note name)

**Decision**: For the first implementation, let the planner update the task context Note during execution (it has infospace tools). The OODA loop does not need to mediate this. The planner receives the Note name as part of its context and can `refine` or `update-note` as needed.

---

## Implementation Sequence

### Phase 1: Blocking `ask` (foundation)

1. Add `_ask_response_queue` (threading.Queue) to `ZenohExecutiveNode.__init__`
2. Modify `_execute_ask` in `infospace_executor.py`: instead of setting `interrupt_requested`, block on `_ask_response_queue.get(timeout=300)`
3. On response: create Note with user's text, bind to `out` variable, return success
4. On timeout: return failure with reason "User did not respond within timeout"
5. On interrupt: push sentinel to queue, `_execute_ask` returns failure
6. Modify `_main_loop_tick` ask-reply handling: push user reply text to `_ask_response_queue` instead of calling `_process_text_input_item`
7. Test: create a goal that uses `ask` to have a multi-turn conversation, verify planner retains context

### Phase 2: Task prefix + WIP Note CRUD + milestone loop

8. Add `task:` prefix detection in `_process_text_input` (alongside `goal:`)
9. Add `active_task_wip` and `active_task_wip_waiting` state to `__init__`
10. Add `_task_wip_counter` with initialization pattern (same as `_scheduled_goal_counter`)
11. Implement `_create_task_wip_note(intention: str) -> str` — creates and persists Note, returns Note name
12. Implement `_read_task_wip() -> dict` — reads current WIP Note content
13. Implement `_update_task_wip(updates: dict)` — reads, merges, writes back
14. Implement `_begin_task_establishment(user_text: str)` — creates WIP Note, sets active_task_wip
15. Implement `_advance_task_wip()` — the milestone decision method (llm_generate + parse + act)
16. Implement `_complete_task_wip()` — creates scheduled goal, updates references
17. Add milestone loop check to `_main_loop_tick` (after done-event processing)
18. Add `task_wip_id` to scheduled goal record when submitting milestone goals
19. Add WIP Note update in `_set_scheduled_goal_result` when `task_wip_id` is present
20. Add guard: reject `task:` / `goal:` commands while WIP is active
21. Test with PGE bill forwarding scenario

### Phase 3: Scheduled activation with context

22. Add `task_context_note` field support to scheduled goal records (in `_new_scheduled_goal`)
23. Modify scheduled goal execution to load and prepend context when `task_context_note` is present
24. Add `run_state` update logic after activation completes
25. Test: verify recurring goal execution receives task context

### Phase 4: Recovery

26. On startup, check for `_active_task_wip` pointer Note in infospace
27. If found, restore `self.active_task_wip` and resume milestone loop

---

## Testing Strategy

### Phase 1 test: Blocking ask

Create a goal that asks the user two questions in sequence:
```
goal: Ask the user their name, then ask their favorite color, then say a greeting using both.
```
Verify:
- Planner asks first question, blocks
- User responds, planner continues (same session, same bindings)
- Planner asks second question, blocks again
- User responds, planner generates greeting using both answers
- Single planner session throughout

### Phase 2 test: PGE bill forwarding

User says:
```
task: whenever we get an email from billpay@pge.com, forward it to Jane
```

Expected milestone sequence:
1. **Specification**: Planner asks user for clarification (Jane's email, frequency, forwarding style) using `ask` within a single goal. Updates WIP Note with all collected parameters.
2. **Capability evaluation**: Planner checks whether email tools can detect duplicate forwarding. Finds they cannot. Records finding in WIP Note.
3. **Infrastructure setup**: Planner creates persistent collection `_pge_forwarding_log`. Records collection name in WIP Note.
4. **Activation**: Milestone loop creates scheduled goal with context referencing the WIP Note and collection. Goal text is something like: "Check email from billpay@pge.com. If found, check _pge_forwarding_log for this email's message-id. If not already logged, forward to jane@example.com and log the message-id."
5. **Complete**: User is informed, recurring goal is active.

### Secondary test cases (for later)

- Homeostatic monitoring: "task: keep an eye on your tool success rate and let me know if it drops"
- Paper monitoring: "task: track HuggingFace trending papers relevant to our project"
- Bounded experiment: "task: test planner performance at temperatures 0.1, 0.3, 0.5, 0.7, 0.9"

---

## Key Design Constraints

1. **No new reasoning engine**: The LLM does the adaptive sequencing via `llm_generate`. The planner does grounded execution. The OODA loop connects them.
2. **No scripted task flows**: The milestone loop does not hardcode phases or transitions. The LLM decides what's next each time. The phases in the WIP Note are advisory, not enforced.
3. **Reuse existing tools**: `manage-goals`, `ask`, `say`, `check-health`, infospace operations — no new tools needed for the core mechanism.
4. **One task-in-progress at a time**: The OODA loop manages one active WIP. If the user tries to establish a second task while one is in progress, the system should inform them and either queue or replace.
5. **WIP Notes are permanent artifacts**: They persist and serve as the task's context package after establishment. They are not temporary.
6. **The milestone loop prompt must not be over-engineered**: Keep it simple. The LLM gets the intention, what's done, what just happened, and decides the next move. Let the LLM's judgment do the work.
7. **Initiation and each step of the milestone loop should be logged.**
8. **Explicit invocation only**: Tasks are established via `task:` prefix. No automatic classification of user messages — the user decides when something is a task.

---

## File Modifications Summary

| File | Changes |
|------|---------|
| `src/infospace_executor.py` | Modify `_execute_ask` to block on response queue instead of interrupting |
| `src/executive_node.py` | New state fields (`active_task_wip`, `active_task_wip_waiting`, `_ask_response_queue`), `_begin_task_establishment`, `_create_task_wip_note`, `_read_task_wip`, `_update_task_wip`, `_advance_task_wip`, `_complete_task_wip`, `task:` prefix routing in `_process_text_input`, milestone loop check in `_main_loop_tick`, WIP Note update in goal completion path, ask-reply routing to queue, recovery on startup, task context loading for scheduled goals |
| `src/goal_scheduler.py` | No changes expected — scheduled goals fire the same way |
| `src/incremental_planner.py` | No changes expected — executes milestone goals as regular goals |
| `src/tools/manage-goals/tool.py` | Add `task_context_note` and `task_wip_id` field support in create/update |

Most changes are concentrated in `executive_node.py` and `infospace_executor.py`. This is intentional — the mechanism is an extension of the OODA loop's decision-making, not a new subsystem.
