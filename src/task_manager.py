#!/usr/bin/env python3
"""
Task Manager — persistent cross-session task abstraction.

Tasks are structured JSON Notes that survive across sessions. Each task has an
abstract plan (coarse steps), a state machine, and references to artifacts
produced along the way. The executive node orchestrates task lifecycle by
submitting goals to the planner; this module handles data and state only.
"""

import copy
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
        if hasattr(self.executor, "delete_resource_and_unbind"):
            deleted, error = self.executor.delete_resource_and_unbind(note_id)
        else:
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
                    task = json.loads(content) if isinstance(content, str) else content
                    if isinstance(task, dict):
                        task, changed = self._normalize_task_schema(task)
                        if changed:
                            self._save_task(task)
                        return task
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
            task, changed = self._normalize_task_schema(task)
            if changed:
                self._save_task(task)
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
            task, changed = self._normalize_task_schema(task)
            if changed:
                self._save_task(task)
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
        task = self.get_task(task_id)
        if not task:
            return None
        task_artifacts = self._normalize_artifact_list(task.get("artifacts", []))
        plan, _ = self._normalize_plan_steps(
            plan,
            base_inputs=task_artifacts,
            start_step=1,
            preserve_runtime_fields=False,
        )
        if plan:
            plan[0]["status"] = STEP_READY
        updates = {"abstract_plan": plan, "current_step": 1, "status": TASK_STEP_READY}
        if "initial_abstract_plan" not in task or not task.get("initial_abstract_plan"):
            updates["initial_abstract_plan"] = copy.deepcopy(plan)
        return self.update_task(task_id, **updates)

    def complete_current_step(self, task_id: str, outcome: str = "", output_artifacts: List[str] = None) -> Optional[Dict]:
        """Mark current step completed and advance to next."""
        task = self.get_task(task_id)
        if not task:
            return None
        plan = task.get("abstract_plan", [])
        idx = task.get("current_step", 1) - 1
        output_artifacts = self._normalize_artifact_list(output_artifacts)
        if 0 <= idx < len(plan):
            plan[idx]["status"] = STEP_COMPLETED
            plan[idx]["outcome"] = outcome
            if output_artifacts:
                plan[idx]["output_artifacts"] = output_artifacts

        next_idx = idx + 1
        merged_artifacts = self._normalize_artifact_list(task.get("artifacts", []))
        merged_artifacts = list(dict.fromkeys(merged_artifacts + output_artifacts))
        if next_idx < len(plan):
            plan[next_idx]["status"] = STEP_READY
            return self.update_task(
                task_id,
                abstract_plan=plan,
                artifacts=merged_artifacts,
                current_step=next_idx + 1,
                status=TASK_STEP_READY,
            )
        else:
            return self.update_task(task_id, abstract_plan=plan, artifacts=merged_artifacts, status=TASK_COMPLETED)

    def update_remaining_plan(self, task_id: str, remaining_steps: List[Dict]) -> Optional[Dict]:
        """Replace remaining (non-completed) steps with a revised plan."""
        task = self.get_task(task_id)
        if not task:
            return None
        plan = task.get("abstract_plan", [])
        completed = [s for s in plan if s.get("status") == STEP_COMPLETED]
        base_inputs = self._normalize_artifact_list(task.get("artifacts", []))
        for step in completed:
            base_inputs.extend(self._normalize_artifact_list(step.get("output_artifacts", [])))
        remaining_steps, _ = self._normalize_plan_steps(
            remaining_steps,
            base_inputs=list(dict.fromkeys(base_inputs)),
            start_step=len(completed) + 1,
            preserve_runtime_fields=False,
        )
        if remaining_steps:
            remaining_steps[0]["status"] = STEP_READY
        new_plan = completed + remaining_steps
        new_current = len(completed) + 1
        return self.update_task(task_id, abstract_plan=new_plan, current_step=new_current)

    def reset_for_reuse(self, task_id: str) -> Optional[Dict]:
        """Reset task to step 1, restoring initial plan if cached."""
        task = self.get_task(task_id)
        if not task:
            return None
        plan = copy.deepcopy(task.get("initial_abstract_plan") or task.get("abstract_plan", []))
        for i, step in enumerate(plan):
            step.setdefault("step", i + 1)
            step["status"] = STEP_PENDING
            step["outcome"] = ""
            step["output_artifacts"] = []
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
            f"Given your full set of tools and capabilities, could you plausibly accomplish this:\n\n"
            f"\"{task['description']}\"\n\n"

            f"If feasible, create a Note named '_task_plan_draft' containing a JSON array of steps.\n"
            f"IMPORTANT: Each step is a FULL planning session that can execute many tool calls "
            f"(search, load, synthesize, create-note, say, etc.) in sequence. "
            f"Do NOT split sequential tool operations into separate steps. "
            f"Only create a new step when there is a genuine pause point: "
            f"human input needed, an external dependency, or a fundamentally different capability. "
            f"Prefer 1-2 steps. A task that searches, processes, and reports is ONE step.\n"
            f"Each step is a JSON object with keys: \"description\", \"input_artifacts\", \"output_artifacts\".\n"
            f"Rules: input_artifacts must reference existing resources or prior-step output_artifacts. "
            f"output_artifacts must name only artifacts this step exports for later steps. "
            f"The final step's output_artifacts should contain the deliverable.\n"
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
                outputs = ", ".join(s.get("output_artifacts", [])) or "none"
                completed_summary += f"  Step {s['step']}: {s['description']} → {s.get('outcome', 'done')} [outputs: {outputs}]\n"

        remaining_summary = ""
        for s in plan[idx + 1:]:
            remaining_summary += f"  Step {s['step']}: {s['description']}\n"
        step_inputs = ", ".join(step.get("input_artifacts", [])) or "none"
        step_outputs = ", ".join(step.get("output_artifacts", [])) or "none declared"

        return (
            f"You are executing a multi-step task.\n"
            f"Task: \"{task['description']}\"\n\n"
            f"Your character:\n{character_desc}\n\n"
            f"{'Prior steps completed:\\n' + completed_summary if completed_summary else ''}"
            f"CURRENT STEP ({step['step']} of {len(plan)}): {step['description']}\n"
            f"Declared inputs: {step_inputs}\n"
            f"Declared outputs: {step_outputs}\n"
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
            f"   - \"output_artifacts\": list of resource names/IDs worth exporting to later steps\n"
            f"   - \"revised_remaining_steps\": updated array of remaining steps "
            f"(same format as planning: objects with \"description\", \"input_artifacts\", and \"output_artifacts\"), "
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

    @staticmethod
    def _normalize_artifact_list(values: Any) -> List[str]:
        """Normalize artifact identifiers into a stable string list."""
        if values is None:
            return []
        if isinstance(values, str):
            values = [values]
        if not isinstance(values, list):
            return []
        normalized = []
        for value in values:
            if value is None:
                continue
            if not isinstance(value, str):
                value = str(value)
            value = value.strip()
            if value:
                normalized.append(value)
        return list(dict.fromkeys(normalized))

    def _normalize_plan_steps(
        self,
        plan: Any,
        base_inputs: Optional[List[str]] = None,
        start_step: int = 1,
        preserve_runtime_fields: bool = True,
    ) -> Tuple[List[Dict[str, Any]], bool]:
        """Normalize/migrate plan steps to explicit input/output artifact schema."""
        if not isinstance(plan, list):
            return [], True
        changed = False
        normalized_plan: List[Dict[str, Any]] = []
        available_outputs = self._normalize_artifact_list(base_inputs or [])
        for i, raw_step in enumerate(plan):
            if isinstance(raw_step, dict):
                step = dict(raw_step)
            else:
                step = {"description": str(raw_step) if raw_step is not None else ""}
                changed = True

            expected_step = start_step + i
            if step.get("step") != expected_step:
                step["step"] = expected_step
                changed = True

            description = step.get("description", "")
            if not isinstance(description, str):
                step["description"] = str(description)
                changed = True

            input_artifacts = self._normalize_artifact_list(step.get("input_artifacts"))
            if step.get("input_artifacts") != input_artifacts:
                changed = True
            step["input_artifacts"] = input_artifacts

            output_artifacts = self._normalize_artifact_list(step.get("output_artifacts"))
            if step.get("output_artifacts") != output_artifacts:
                changed = True
            step["output_artifacts"] = output_artifacts

            if "artifacts" in step:
                step.pop("artifacts", None)
                changed = True
            if "expected_artifacts" in step:
                step.pop("expected_artifacts", None)
                changed = True

            if preserve_runtime_fields:
                if "status" not in step:
                    step["status"] = STEP_PENDING
                    changed = True
                if "outcome" not in step:
                    step["outcome"] = ""
                    changed = True
            else:
                if step.get("status") != STEP_PENDING:
                    step["status"] = STEP_PENDING
                    changed = True
                if step.get("outcome") != "":
                    step["outcome"] = ""
                    changed = True

            available_outputs = list(dict.fromkeys(available_outputs + output_artifacts))
            normalized_plan.append(step)
        return normalized_plan, changed

    def _normalize_task_schema(self, task: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
        """Normalize/migrate task schema in-place for artifact IO compatibility."""
        changed = False
        task_artifacts = self._normalize_artifact_list(task.get("artifacts", []))
        if task.get("artifacts") != task_artifacts:
            task["artifacts"] = task_artifacts
            changed = True

        plan = task.get("abstract_plan", [])
        normalized_plan, plan_changed = self._normalize_plan_steps(
            plan,
            base_inputs=task_artifacts,
            start_step=1,
            preserve_runtime_fields=True,
        )
        if plan_changed:
            task["abstract_plan"] = normalized_plan
            changed = True
        return task, changed
