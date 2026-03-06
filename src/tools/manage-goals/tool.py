"""
CRUD operations on scheduled goals.

Delegates to executive_node methods for goal creation, listing,
updating, and deletion.
"""

import json
import logging
from typing import Any, Dict, List, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

VALID_SCHEDULE_MODES = ("manual", "auto", "recurring", "daily")


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
    goal_text = (kwargs.get("goal_text") or "").strip()
    if not goal_text:
        return _fail(executor, "goal_text is required for create")

    goal = executive_node._upsert_scheduled_goal(goal_text)
    goal_id = goal["goal_id"]

    # Apply optional overrides
    updates: Dict[str, Any] = {}
    name = kwargs.get("name")
    if name:
        updates["name"] = str(name)[:120]
        updates["name_customized"] = True
    mode = kwargs.get("schedule_mode")
    if mode and mode in VALID_SCHEDULE_MODES:
        updates["schedule_mode"] = mode
    run_at = kwargs.get("run_at")
    if run_at:
        updates["run_at"] = str(run_at)

    if updates:
        executive_node._update_scheduled_goal(goal_id, **updates)
        goal.update(updates)

    display_name = goal.get("name") or goal_text[:80]
    mode_str = goal.get("schedule_mode", "manual")
    summary = f"Created goal '{display_name}' ({goal_id}, {mode_str})"
    if goal.get("run_at"):
        summary += f" at {goal['run_at']}"

    logger.info(f"manage-goals: {summary}")
    return _success(executor, value=summary, data=goal)


def _action_list(executive_node, executor, **kwargs) -> Dict:
    goals: List[Dict] = executive_node._all_scheduled_goals()

    if not goals:
        return _success(executor, value="No scheduled goals.", data=[])

    lines = []
    for g in goals:
        name = g.get("name") or g.get("goal_text", "?")[:80]
        status = g.get("status", "?")
        mode = g.get("schedule_mode", "manual")
        run_at = g.get("run_at", "")
        last_result = (g.get("last_result") or "")[:100]
        primary_product = g.get("primary_product", "")
        line = f"- {g['goal_id']}: {name} [{status}, {mode}"
        if run_at:
            line += f" at {run_at}"
        line += "]"
        if primary_product:
            line += f" → {primary_product}"
        if last_result:
            line += f" — {last_result}"
        lines.append(line)

    summary = f"{len(goals)} scheduled goal(s):\n" + "\n".join(lines)
    return _success(executor, value=summary, data=goals)


def _action_update(executive_node, executor, **kwargs) -> Dict:
    goal_id = (kwargs.get("goal_id") or "").strip()
    if not goal_id:
        return _fail(executor, "goal_id is required for update")

    existing = executive_node._get_scheduled_goal(goal_id)
    if not existing:
        return _fail(executor, f"Goal not found: {goal_id}")

    updates: Dict[str, Any] = {}
    for field in ("name", "goal_text", "status", "schedule_mode", "run_at"):
        val = kwargs.get(field)
        if val is not None:
            val = str(val).strip()
            if field == "schedule_mode" and val not in VALID_SCHEDULE_MODES:
                return _fail(executor,
                             f"Invalid schedule_mode '{val}'. Must be one of: {', '.join(VALID_SCHEDULE_MODES)}")
            if field == "name":
                val = val[:120]
                updates["name_customized"] = True
            updates[field] = val

    if not updates:
        return _fail(executor, "No fields to update. Provide name, goal_text, status, schedule_mode, or run_at.")

    updated = executive_node._update_scheduled_goal(goal_id, **updates)
    display_name = updated.get("name") or goal_id
    summary = f"Updated goal '{display_name}' ({goal_id}): {', '.join(f'{k}={v}' for k, v in updates.items() if k != 'name_customized')}"

    logger.info(f"manage-goals: {summary}")
    return _success(executor, value=summary, data=updated)


def _action_delete(executive_node, executor, **kwargs) -> Dict:
    goal_id = (kwargs.get("goal_id") or "").strip()
    if not goal_id:
        return _fail(executor, "goal_id is required for delete")

    existing = executive_node._get_scheduled_goal(goal_id)
    if not existing:
        return _fail(executor, f"Goal not found: {goal_id}")

    display_name = existing.get("name") or goal_id
    deleted = executive_node._delete_scheduled_goal(goal_id)
    if not deleted:
        return _fail(executor, f"Failed to delete goal {goal_id}")

    summary = f"Deleted goal '{display_name}' ({goal_id})"
    logger.info(f"manage-goals: {summary}")
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
    """Manage scheduled goals: create, list, update, delete."""
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
        logger.error(f"manage-goals {action} failed: {e}", exc_info=True)
        return _fail(executor, f"{action} failed: {e}")
