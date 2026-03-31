"""
CRUD operations on tasks (task WIPs).

Parallels manage-goals: create, list, update, delete.
Delegates to executive_node methods for task lifecycle management.
"""

import json
import logging
from datetime import datetime
from typing import Any, Dict, List, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

VALID_STATUSES = ("proposed", "in_progress", "interrupted", "abandoned", "completed")


def _fail(executor: InfospaceExecutor, reason: str,
          value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed", value=value or reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str,
             data: Any = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "success", value=value, resource_id=None, extra=extra, data=data or value)


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------

def _action_create(executive_node, executor, **kwargs) -> Dict:
    intention = (kwargs.get("intention") or "").strip()
    if not intention:
        return _fail(executor, "intention is required for create")

    # Guard: don't create while another task is actively establishing
    if executive_node.active_task_wip:
        return _fail(executor,
                     f"Cannot create task — {executive_node.active_task_wip} is currently establishing. "
                     f"Wait for it to complete or abandon it first.")

    concern_id = kwargs.get("concern_id", "")

    executive_node._task_wip_counter += 1
    wip_id = f"twip_{executive_node._task_wip_counter}"
    note_name = f"_task_wip_{executive_node._task_wip_counter}"
    now = datetime.now().isoformat()
    wip_content = {
        "task_wip_id": wip_id,
        "intention": intention,
        "status": "in_progress",
        "phase": "specification",
        "milestones_completed": [],
        "current_milestone": None,
        "accumulated_findings": [],
        "created": now,
        "updated": now,
    }
    if concern_id:
        wip_content["linked_concern_id"] = concern_id

    executor.execute_action({
        "type": "create-note",
        "value": json.dumps(wip_content),
        "name": note_name,
        "out": f"${note_name}",
    })
    executor.execute_action({
        "type": "persist",
        "target": f"${note_name}",
        "name": note_name,
    })
    executive_node.active_task_wip = note_name
    executive_node.active_task_wip_waiting = False
    executive_node._task_wip_pre_resource_ids = set(
        executive_node.resource_manager.resource_registry.keys()
    ) if executive_node.resource_manager else set()

    summary = f"Created task '{intention[:80]}' ({note_name})"
    logger.info(f"manage-tasks: {summary}")
    return _success(executor, value=summary, data=wip_content)


def _action_list(executive_node, executor, **kwargs) -> Dict:
    tasks: List[Dict] = executive_node._get_all_task_data()

    if not tasks:
        return _success(executor, value="No tasks.", data=[])

    lines = []
    for t in tasks:
        note_name = t.get("_note_name", "?")
        intention = (t.get("intention") or "?")[:80]
        status = t.get("status", "?")
        phase = t.get("phase", "")
        concern = t.get("linked_concern_id", "")
        line = f"- {note_name}: {intention} [{status}"
        if phase:
            line += f", {phase}"
        line += "]"
        if concern:
            line += f" (concern: {concern})"
        if note_name == executive_node.active_task_wip:
            line += " ← active"
        lines.append(line)

    summary = f"{len(tasks)} task(s):\n" + "\n".join(lines)
    return _success(executor, value=summary, data=tasks)


def _action_update(executive_node, executor, **kwargs) -> Dict:
    task_id = (kwargs.get("task_id") or "").strip()
    if not task_id:
        return _fail(executor, "task_id is required for update (e.g. _task_wip_1 or just 1)")

    # Normalize: accept bare number or full note name
    if not task_id.startswith("_task_wip_"):
        task_id = f"_task_wip_{task_id}"

    if not executive_node.resource_manager:
        return _fail(executor, "resource manager not available")

    note_id = executive_node.resource_manager.named_notes.get(task_id)
    if not note_id:
        return _fail(executor, f"Task not found: {task_id}")

    try:
        res = executive_node.resource_manager.get_resource(note_id)
        content = json.loads(res.get("properties", {}).get("content", "{}"))
    except Exception as e:
        return _fail(executor, f"Could not read task {task_id}: {e}")

    updates = {}
    for field in ("intention", "status"):
        val = kwargs.get(field)
        if val is not None:
            val = str(val).strip()
            if field == "status" and val not in VALID_STATUSES:
                return _fail(executor,
                             f"Invalid status '{val}'. Must be one of: {', '.join(VALID_STATUSES)}")
            updates[field] = val

    if not updates:
        return _fail(executor, "No fields to update. Provide intention or status.")

    # Handle status transitions with side effects
    new_status = updates.get("status")
    old_status = content.get("status", "")

    if new_status == "abandoned" and old_status != "abandoned":
        # Delegate to abandon handler for proper cleanup
        reason = kwargs.get("reason", "abandoned via manage-tasks tool")
        executive_node._abandon_task(task_id, reason)
        summary = f"Task {task_id} abandoned"
        logger.info(f"manage-tasks: {summary}")
        return _success(executor, value=summary, data={"task_id": task_id, "status": "abandoned"})

    if new_status == "in_progress" and old_status in ("proposed", "interrupted"):
        # Delegate to approve handler for proper activation
        edited_intention = updates.get("intention", "")
        executive_node._approve_proposed_task(task_id, edited_intention)
        summary = f"Task {task_id} approved and activating"
        logger.info(f"manage-tasks: {summary}")
        return _success(executor, value=summary, data={"task_id": task_id, "status": "in_progress"})

    # Generic field update
    content.update(updates)
    content["updated"] = datetime.now().isoformat()
    success, err = executive_node.resource_manager.update_note_content(
        note_id, json.dumps(content))
    if not success:
        return _fail(executor, f"Failed to update {task_id}: {err}")

    summary = f"Updated task {task_id}: {', '.join(f'{k}={v}' for k, v in updates.items())}"
    logger.info(f"manage-tasks: {summary}")
    return _success(executor, value=summary, data=content)


def _action_delete(executive_node, executor, **kwargs) -> Dict:
    task_id = (kwargs.get("task_id") or "").strip()
    if not task_id:
        return _fail(executor, "task_id is required for delete")

    if not task_id.startswith("_task_wip_"):
        task_id = f"_task_wip_{task_id}"

    if not executive_node.resource_manager:
        return _fail(executor, "resource manager not available")

    note_id = executive_node.resource_manager.named_notes.get(task_id)
    if not note_id:
        return _fail(executor, f"Task not found: {task_id}")

    # Read intention for display before deleting
    display = task_id
    try:
        res = executive_node.resource_manager.get_resource(note_id)
        content = json.loads(res.get("properties", {}).get("content", "{}"))
        display = content.get("intention", task_id)[:80]
    except Exception:
        pass

    executive_node._cmd_task_delete_impl(task_id)

    summary = f"Deleted task '{display}' ({task_id})"
    logger.info(f"manage-tasks: {summary}")
    return _success(executor, value=summary, data=True)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

_ACTIONS = {
    "create": _action_create,
    "list": _action_list,
    "update": _action_update,
    "delete": _action_delete,
}


def tool(input_value=None, runtime=None, **kwargs):
    """Manage tasks: create, list, update, delete."""
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    executive_node = kwargs.get("executive_node")
    if not executive_node:
        return _fail(executor, "executive_node not available")

    action = (kwargs.get("action") or input_value or "").strip().lower()
    if action not in _ACTIONS:
        return _fail(executor,
                     f"Unknown action '{action}'. Must be one of: {', '.join(_ACTIONS)}")

    try:
        action_kwargs = {k: v for k, v in kwargs.items() if k not in ("executive_node", "executor")}
        return _ACTIONS[action](executive_node, executor, **action_kwargs)
    except Exception as e:
        logger.error(f"manage-tasks {action} failed: {e}", exc_info=True)
        return _fail(executor, f"{action} failed: {e}")
