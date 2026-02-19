#!/usr/bin/env python3
"""
Task Manager — persistent cross-session task abstraction.

Tasks are structured JSON Notes that survive across sessions. Each task has an
abstract plan (coarse steps), a state machine, and references to artifacts
produced along the way. The executive node orchestrates task lifecycle by
submitting goals to the planner; this module handles data and state only.
"""

import json
import logging
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Task status values
TASK_CREATED = "created"
TASK_PLANNING = "planning"
TASK_STEP_READY = "step_ready"
TASK_EXECUTING = "executing"
TASK_REVIEWING = "reviewing"
TASK_COMPLETED = "completed"
TASK_BLOCKED = "blocked"
TASK_ABANDONED = "abandoned"

TERMINAL_STATUSES = {TASK_COMPLETED, TASK_ABANDONED}

# Step status values
STEP_PENDING = "pending"
STEP_READY = "ready"
STEP_EXECUTING = "executing"
STEP_COMPLETED = "completed"
STEP_SKIPPED = "skipped"

# Note naming
TASK_NOTE_PREFIX = "_task_"
TASKS_COLLECTION = "_tasks"


def _new_task(task_id: str, description: str) -> Dict[str, Any]:
    """Create a blank task dict."""
    now = datetime.now().isoformat()
    return {
        "task_id": task_id,
        "name": description[:120],
        "description": description,
        "status": TASK_CREATED,
        "created": now,
        "updated": now,
        "abstract_plan": [],
        "current_step": 0,
        "blockers": [],
        "artifacts": [],
        "task_created_resources": [],
        "notes": "",
    }


class TaskManager:
    """CRUD + state-machine wrapper around task Notes in the resource manager."""

    def __init__(self, resource_manager: Any, executor: Any, character_name: str):
        self.resource_manager = resource_manager
        self.executor = executor
        self.character_name = character_name
        self._task_counter = 0
        self._active_task_goal: Optional[Dict[str, Any]] = None

    # ------------------------------------------------------------------
    # Bootstrap
    # ------------------------------------------------------------------

    def initialize(self):
        """Ensure _tasks collection exists; load counter from existing tasks."""
        result = self.executor.execute_action({"type": "load", "target": TASKS_COLLECTION, "out": "$_tasks_col"})
        if result.get("status") != "success" or not result.get("resource_id"):
            self.executor.execute_action({"type": "create-collection", "name": TASKS_COLLECTION, "out": "$_tasks_col"})
            self.executor.execute_action({"type": "persist", "target": "$_tasks_col"})
            logger.info("Created _tasks collection")
        else:
            logger.info("_tasks collection exists")

        # Derive counter from existing task notes
        max_id = 0
        for name in list(self.resource_manager.named_notes.keys()):
            if name.startswith(TASK_NOTE_PREFIX):
                try:
                    num = int(name[len(TASK_NOTE_PREFIX):])
                    max_id = max(max_id, num)
                except ValueError:
                    pass
        self._task_counter = max_id
        logger.info(f"TaskManager initialized (counter={self._task_counter})")

    # ------------------------------------------------------------------
    # CRUD
    # ------------------------------------------------------------------

    def _get_tasks_collection_note_ids(self) -> List[str]:
        """Get note IDs from _tasks collection. Falls back to named_notes if collection empty."""
        col_id = self.resource_manager.named_collections.get(TASKS_COLLECTION)
        if not col_id and hasattr(self.resource_manager, '_resolve_resource_id'):
            col_id = self.resource_manager._resolve_resource_id(TASKS_COLLECTION)
        if col_id:
            res = self.resource_manager.get_resource(col_id)
            if res:
                content = res.get('properties', {}).get('content', [])
                if isinstance(content, list) and content:
                    return content
        return [self.resource_manager.named_notes[n] for n in self.resource_manager.named_notes
                if n.startswith(TASK_NOTE_PREFIX)]

    def create_task(self, description: str) -> Tuple[str, Dict]:
        """Create a new task. Returns (task_id, task_dict)."""
        self._task_counter += 1
        task_id = f"task_{self._task_counter}"
        base_name = description[:120]
        existing_names = {t.get("name") for t in self.get_all_tasks()}
        name = base_name
        suffix = 1
        while name in existing_names:
            name = f"{base_name} ({suffix})"
            suffix += 1
        task = _new_task(task_id, description)
        task["name"] = name
        self._save_task(task)
        logger.info(f"Created {task_id}: {description[:80]}")
        return task_id, task

    def delete_task(self, task_id: str) -> bool:
        """Delete a task note by task id."""
        note_name = TASK_NOTE_PREFIX + task_id.replace("task_", "")
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id and hasattr(self.resource_manager, "_resolve_resource_id"):
            note_id = self.resource_manager._resolve_resource_id(note_name)
        if not note_id:
            logger.warning(f"Task note not found for delete: {task_id}")
            return False
        deleted, error = self.resource_manager.delete_resource(note_id)
        if not deleted:
            logger.warning(f"Failed to delete task {task_id}: {error}")
            return False
        return True

    def get_task(self, task_id: str) -> Optional[Dict]:
        """Load a task by id."""
        note_name = TASK_NOTE_PREFIX + task_id.replace("task_", "")
        result = self.executor.execute_action({"type": "load", "target": note_name, "out": "$_task_tmp"})
        if result.get("status") == "success" and result.get("resource_id"):
            content = self.executor._get_content(result["resource_id"])
            if content:
                try:
                    return json.loads(content) if isinstance(content, str) else content
                except json.JSONDecodeError:
                    logger.warning(f"Task note {note_name} has non-JSON content")
        return None

    def get_active_tasks(self) -> List[Dict]:
        """Return all non-terminal tasks, sorted by creation time."""
        tasks = []
        note_ids = self._get_tasks_collection_note_ids()
        for note_id in note_ids:
            content = self.executor._get_content(note_id)
            if not content:
                continue
            try:
                task = json.loads(content) if isinstance(content, str) else content
            except (json.JSONDecodeError, TypeError):
                continue
            if not isinstance(task, dict):
                continue
            if task.get("status") not in TERMINAL_STATUSES:
                tasks.append(task)
        tasks.sort(key=lambda t: t.get("created", ""))
        return tasks

    def get_all_tasks(self) -> List[Dict]:
        """Return all tasks (including terminal), sorted by creation time."""
        tasks = []
        note_ids = self._get_tasks_collection_note_ids()
        for note_id in note_ids:
            content = self.executor._get_content(note_id)
            if not content:
                continue
            try:
                task = json.loads(content) if isinstance(content, str) else content
            except (json.JSONDecodeError, TypeError):
                continue
            if not isinstance(task, dict):
                continue
            tasks.append(task)
        tasks.sort(key=lambda t: t.get("created", ""))
        return tasks

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def update_task(self, task_id: str, **fields) -> Optional[Dict]:
        """Update arbitrary fields on a task and persist."""
        task = self.get_task(task_id)
        if not task:
            logger.warning(f"Task {task_id} not found for update")
            return None
        task.update(fields)
        task["updated"] = datetime.now().isoformat()
        self._save_task(task)
        return task

    def set_status(self, task_id: str, status: str) -> Optional[Dict]:
        return self.update_task(task_id, status=status)

    def set_abstract_plan(self, task_id: str, plan: List[Dict]) -> Optional[Dict]:
        """Set the abstract plan and mark first step ready."""
        for i, step in enumerate(plan):
            step.setdefault("step", i + 1)
            step.setdefault("status", STEP_PENDING)
            step.setdefault("artifacts", [])
            step.setdefault("outcome", "")
        if plan:
            plan[0]["status"] = STEP_READY
        return self.update_task(task_id, abstract_plan=plan, current_step=1, status=TASK_STEP_READY)

    def complete_current_step(self, task_id: str, outcome: str = "", artifacts: List[str] = None) -> Optional[Dict]:
        """Mark current step completed and advance to next."""
        task = self.get_task(task_id)
        if not task:
            return None
        plan = task.get("abstract_plan", [])
        idx = task.get("current_step", 1) - 1
        if 0 <= idx < len(plan):
            plan[idx]["status"] = STEP_COMPLETED
            plan[idx]["outcome"] = outcome
            if artifacts:
                plan[idx]["artifacts"] = artifacts

        next_idx = idx + 1
        if next_idx < len(plan):
            plan[next_idx]["status"] = STEP_READY
            return self.update_task(task_id, abstract_plan=plan, current_step=next_idx + 1, status=TASK_STEP_READY)
        else:
            return self.update_task(task_id, abstract_plan=plan, status=TASK_COMPLETED)

    def update_remaining_plan(self, task_id: str, remaining_steps: List[Dict]) -> Optional[Dict]:
        """Replace remaining (non-completed) steps with a revised plan."""
        task = self.get_task(task_id)
        if not task:
            return None
        plan = task.get("abstract_plan", [])
        completed = [s for s in plan if s.get("status") == STEP_COMPLETED]
        for i, step in enumerate(remaining_steps):
            step["step"] = len(completed) + i + 1
            step.setdefault("status", STEP_PENDING)
            step.setdefault("artifacts", [])
            step.setdefault("outcome", "")
        if remaining_steps:
            remaining_steps[0]["status"] = STEP_READY
        new_plan = completed + remaining_steps
        new_current = len(completed) + 1
        return self.update_task(task_id, abstract_plan=new_plan, current_step=new_current)

    def reset_for_reuse(self, task_id: str) -> Optional[Dict]:
        """Reset a completed task back to step 1 while preserving the plan text."""
        task = self.get_task(task_id)
        if not task:
            return None
        plan = task.get("abstract_plan", [])
        for i, step in enumerate(plan):
            step.setdefault("step", i + 1)
            step["status"] = STEP_PENDING
            step["outcome"] = ""
            step["artifacts"] = []
        if plan:
            plan[0]["status"] = STEP_READY
        return self.update_task(
            task_id,
            status=TASK_STEP_READY,
            current_step=1,
            blockers=[],
            artifacts=[],
            task_created_resources=[],
            notes="",
            abstract_plan=plan,
        )

    # ------------------------------------------------------------------
    # Active task-goal tracking (which goal is running for which task)
    # ------------------------------------------------------------------

    @property
    def active_task_goal(self) -> Optional[Dict[str, Any]]:
        return self._active_task_goal

    def set_active_task_goal(self, task_id: str, goal_type: str, step_number: int = 0, **extra_fields):
        """Record that the currently-running goal is task-related."""
        self._active_task_goal = {"task_id": task_id, "goal_type": goal_type, "step_number": step_number}
        if extra_fields:
            self._active_task_goal.update(extra_fields)

    def clear_active_task_goal(self):
        self._active_task_goal = None

    # ------------------------------------------------------------------
    # Goal text generation
    # ------------------------------------------------------------------

    def planning_goal_text(self, task: Dict, character_desc: str) -> str:
        """Generate goal text for the plan-generation phase."""
        return (
            f"You are producing a plan. Answer without performing the target task, using only think and create-note tools:\n"
            f"Given your full set of tools and capabilities, could you plausibly accomplish this:\n"
            f"\"{task['description']}\"\n\n"
            f"Assess whether this task is feasible with your available tools and capabilities.\n"
            f"If feasible, create a Note named '_task_plan_draft' containing a JSON array containing "
            f"a minimal set of correct and robust high-level steps (3-7 steps) for achieving the task.\n"
            f" Each step should be a JSON object with keys:\n "
            f"\"description\" (what to accomplish) and \"expected_artifacts\" (what to produce).\n"
            f"If not feasible, explain why in your FINAL_ANSWER and create no Note.\n"
            f"Output ONLY the JSON array in the Note, no wrapper object."
        )

    def step_execution_goal_text(self, task: Dict, character_desc: str) -> str:
        """Generate goal text for executing the current step."""
        plan = task.get("abstract_plan", [])
        idx = task.get("current_step", 1) - 1
        if idx < 0 or idx >= len(plan):
            return f"Task {task['task_id']} has no executable step."
        step = plan[idx]
        completed_summary = ""
        for s in plan[:idx]:
            if s.get("status") == STEP_COMPLETED:
                arts = ", ".join(s.get("artifacts", [])) or "none"
                completed_summary += f"  Step {s['step']}: {s['description']} → {s.get('outcome', 'done')} [artifacts: {arts}]\n"

        remaining_summary = ""
        for s in plan[idx + 1:]:
            remaining_summary += f"  Step {s['step']}: {s['description']}\n"

        return (
            f"You are executing a multi-step task.\n"
            f"Task: \"{task['description']}\"\n\n"
            f"Your character:\n{character_desc}\n\n"
            f"{'Prior steps completed:\\n' + completed_summary if completed_summary else ''}"
            f"CURRENT STEP ({step['step']} of {len(plan)}): {step['description']}\n"
            f"{'Remaining steps after this:\\n' + remaining_summary if remaining_summary else ''}\n"
            f"Execute this step. Persist any important output Notes."
        )

    def review_goal_text(self, task: Dict, goal_result: Dict, character_desc: str) -> str:
        """Generate goal text for the post-step review phase."""
        plan = task.get("abstract_plan", [])
        idx = task.get("current_step", 1) - 1
        step_desc = plan[idx]["description"] if 0 <= idx < len(plan) else "unknown"
        success = goal_result.get("success", False)
        final_thoughts = (goal_result.get("response") or "")[:500]

        return (
            f"You just completed a step in a multi-step task. Now review the results.\n"
            f"Task: \"{task['description']}\"\n"
            f"Step just executed ({idx + 1} of {len(plan)}): {step_desc}\n"
            f"Outcome: {'success' if success else 'failed'}\n"
            f"Summary: {final_thoughts}\n\n"
            f"Your character:\n{character_desc}\n\n"
            f"Do the following:\n"
            f"1. Load and inspect the Notes/Collections created during this step.\n"
            f"2. Identify which artifacts are valuable outputs vs intermediate scratch.\n"
            f"3. Delete intermediate artifacts that are not needed going forward.\n"
            f"4. Persist valuable output artifacts.\n"
            f"5. Assess whether the remaining plan steps are still appropriate given the actual results.\n"
            f"6. Create a Note named '_task_review' with a JSON object containing:\n"
            f"   - \"step_outcome\": brief description of what was accomplished\n"
            f"   - \"artifacts\": list of resource names/IDs worth keeping\n"
            f"   - \"revised_remaining_steps\": updated array of remaining steps "
            f"(same format as planning: objects with \"description\" and \"expected_artifacts\"), "
            f"or null if no changes needed\n"
            f"   - \"blocked\": true/false — is the task blocked?\n"
            f"   - \"block_reason\": explanation if blocked, else empty string\n"
        )

    # ------------------------------------------------------------------
    # Situation summary (derived view for planner context)
    # ------------------------------------------------------------------

    def situation_summary(self) -> str:
        """Build a situation summary from active task state."""
        tasks = self.get_active_tasks()
        if not tasks:
            return ""
        lines = ["# ACTIVE TASKS"]
        for t in tasks:
            plan = t.get("abstract_plan", [])
            total = len(plan)
            completed = sum(1 for s in plan if s.get("status") == STEP_COMPLETED)
            status = t.get("status", "unknown")
            lines.append(f"- [{status}] {t['name']} (step {t.get('current_step', '?')}/{total}, {completed} done)")
            if t.get("blockers"):
                lines.append(f"  Blocked: {'; '.join(t['blockers'])}")
            current_idx = t.get("current_step", 1) - 1
            if 0 <= current_idx < total:
                lines.append(f"  Next: {plan[current_idx].get('description', '')}")
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _save_task(self, task: Dict):
        """Write task dict to its named Note and ensure it's in the _tasks collection."""
        task_id = task["task_id"]
        note_name = TASK_NOTE_PREFIX + task_id.replace("task_", "")
        content = json.dumps(task, ensure_ascii=False, indent=2)

        # Write Note (create or update)
        result = self.executor.execute_action({"type": "load", "target": note_name, "out": "$_task_save_tmp"})
        if result.get("status") == "success" and result.get("resource_id"):
            self.resource_manager.update_note_content(result["resource_id"], content)
        else:
            create_result = self.executor.execute_action({"type": "create-note", "value": content, "name": note_name, "out": "$_task_save_tmp"})
            note_id = create_result.get("resource_id")
            self.executor.execute_action({"type": "persist", "target": "$_task_save_tmp"})
            # Add to _tasks collection via resource manager (load returns a slice, not the original collection)
            tasks_col_id = self.resource_manager.named_collections.get(TASKS_COLLECTION) or self.resource_manager._resolve_resource_id(TASKS_COLLECTION)
            if tasks_col_id and note_id:
                added, _, err = self.resource_manager.add_to_collection(tasks_col_id, note_id, self.character_name, operation="add")
                if not added:
                    logger.warning(f"Failed to add task note to _tasks: {err}")
