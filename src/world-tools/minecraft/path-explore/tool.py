"""
path-explore

Method tool: perform one frontier-directed exploration episode.

High-level behavior:
- Reason about locally reachable frontier cells (path-frontier)
- Select one frontier target (hardcoded: farthest)
- Attempt to reach it via nav primitives
- Update spatial map
- Return a concise outcome summary

Non-goals:
- No guarantees of coverage, optimality, or success
- No persistence of failure annotations (future work)
- No planner-tunable heuristics
"""

import logging
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

MIN_FRONTIER = 1  # retry with allow_unknown if below this


def tool(input_value=None, **kwargs):
    """
    Execute a single frontier exploration attempt.
    Args: input_value: ignored, **kwargs: executor (required)
    Returns: Uniform tool return with: value = {"outcome": "no_frontier" | "reached" | "blocked" | "no_progress" | "status_failed", "target": {"x": int, "z": int}  # present unless outcome is "no_frontier"}
    """
    executor = kwargs.get("executor")
    if not executor:
        return executor._create_uniform_return("failed", value="Executor not available", reason="executor_not_available")

    # ------------------------------------------------------------
    # Phase 1–3 — Frontier reasoning + execute frontier "path" as hint
    # ------------------------------------------------------------
    def _get_pos() -> Optional[Dict[str, Any]]:
        # Prefer nav cache (fast), fallback to mc-status.
        nav_list = executor.get_world_state("nav")
        if isinstance(nav_list, list) and nav_list and isinstance(nav_list[0], dict):
            pose = nav_list[0].get("pose", {})
            if isinstance(pose, dict) and pose:
                return pose
        status = executor.execute_action_with_log({"type": "mc-status"}, "path-explore")
        if status.get("status") != "success":
            return None
        return status.get("data", {}).get("position", {})

    def _select_target(reachable_list: list, banned: set, x0: float, z0: float) -> Optional[Dict[str, Any]]:
        candidates = [p for p in reachable_list if (p.get("x"), p.get("z")) not in banned]
        if not candidates:
            return None
        def dist_sq(p):
            return (p["x"] - x0) ** 2 + (p["z"] - z0) ** 2
        return max(candidates, key=dist_sq)

    def _exec_hint_step(step_name: str) -> Dict[str, Any]:
        if step_name == "nav-move":
            return executor.execute_action_with_log({"type": "nav-move"}, "path-explore")
        if step_name == "nav-climb":
            return executor.execute_action_with_log({"type": "nav-climb"}, "path-explore")
        if step_name == "nav-descend":
            return executor.execute_action_with_log({"type": "nav-descend"}, "path-explore")
        if step_name == "nav-turn-left":
            return executor.execute_action_with_log({"type": "nav-turn", "direction": "left"}, "path-explore")
        if step_name == "nav-turn-right":
            return executor.execute_action_with_log({"type": "nav-turn", "direction": "right"}, "path-explore")
        # Unknown hint step: treat as failure.
        return {"status": "failed", "reason": "unknown_step"}

    pos0 = _get_pos()
    if not pos0:
        return executor._create_uniform_return("failed", value="Failed to get status", reason="status_failed")
    x0, z0 = pos0.get("x"), pos0.get("z")
    if x0 is None or z0 is None:
        return executor._create_uniform_return("failed", value="Missing position", reason="status_failed")

    execution_result = "no_progress"
    target = None
    banned_targets = set()

    # Bounded replanning loop: each iteration gets a new frontier hint path.
    for attempt_idx in range(3):
        frontier = executor.execute_action_with_log({"type": "path-frontier"}, "path-explore")
        if frontier.get("status") != "success":
            return executor._create_uniform_return("failed", value="Failed to get frontier", reason="path_frontier_failed")

        reachable = frontier.get("data", {}).get("reachable", [])
        if len(reachable) < MIN_FRONTIER:
            frontier = executor.execute_action_with_log({"type": "path-frontier", "allow_unknown": True}, "path-explore")
            if frontier.get("status") != "success":
                return executor._create_uniform_return("failed", value="Failed to get frontier with allow_unknown", reason="path_frontier_failed_allow_unknown")
            reachable = frontier.get("data", {}).get("reachable", [])

        if not reachable:
            return executor._create_uniform_return("success", value={"outcome": "no_frontier"})

        target = _select_target(reachable, banned_targets, x0, z0)
        if not target:
            # No non-banned candidates remain.
            break

        hint_path = target.get("path") or []
        if not isinstance(hint_path, list) or not hint_path:
            # Frontier returned a target without a usable path.
            banned_targets.add((target.get("x"), target.get("z")))
            continue

        step_failed = False
        for step_name in hint_path:
            res = _exec_hint_step(str(step_name))
            if res.get("status") != "success":
                step_failed = True
                break
            pos = _get_pos()
            if not pos:
                execution_result = "status_failed"
                step_failed = True
                break
            if round(pos.get("x", 0)) == target["x"] and round(pos.get("z", 0)) == target["z"]:
                execution_result = "reached"
                break

        if execution_result == "reached":
            break

        if step_failed:
            execution_result = "blocked"
            banned_targets.add((target.get("x"), target.get("z")))
            # Small map refresh before replanning.
            executor.execute_action_with_log({"type": "mc-map-update"}, "path-explore")
            continue

        # Path executed but we didn't reach target (should be rare with rounding/physics).
        execution_result = "no_progress"
        banned_targets.add((target.get("x"), target.get("z")))

    # ------------------------------------------------------------
    # Phase 4 — Map reconciliation (minimal)
    # ------------------------------------------------------------
    # Observe is now forward-cone-limited, so do a small multi-facing sweep.
    # KISS refinement: skip "back" since that's presumably where we came from.
    # Sweep: current (forward), left, right, then restore original yaw.
    executor.execute_action_with_log({"type": "mc-map-update"}, "path-explore")
    turned_left = executor.execute_action_with_log({"type": "nav-turn", "direction": "left"}, "path-explore")
    if turned_left.get("status") == "success":
        executor.execute_action_with_log({"type": "mc-map-update"}, "path-explore")
        turned_right_1 = executor.execute_action_with_log({"type": "nav-turn", "direction": "right"}, "path-explore")
        if turned_right_1.get("status") == "success":
            turned_right_2 = executor.execute_action_with_log({"type": "nav-turn", "direction": "right"}, "path-explore")
            if turned_right_2.get("status") == "success":
                executor.execute_action_with_log({"type": "mc-map-update"}, "path-explore")
                executor.execute_action_with_log({"type": "nav-turn", "direction": "left"}, "path-explore")

    # ------------------------------------------------------------
    # Final outcome
    # ------------------------------------------------------------
    if target:
        return executor._create_uniform_return("success", value={"outcome": execution_result, "target": {"x": target["x"], "z": target["z"]}})
    return executor._create_uniform_return("success", value={"outcome": execution_result})


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("path-explore module loaded")
