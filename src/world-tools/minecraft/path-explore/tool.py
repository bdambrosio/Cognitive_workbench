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
    # Phase 1 — Frontier reasoning
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # Phase 2 — Frontier selection (hardcoded: farthest)
    # ------------------------------------------------------------
    status = executor.execute_action_with_log({"type": "mc-status"}, "path-explore")
    if status.get("status") != "success":
        return executor._create_uniform_return("failed", value="Failed to get status", reason="status_failed")

    pos = status.get("data", {}).get("position", {})
    x0, z0 = pos.get("x"), pos.get("z")

    def dist_sq(p):
        return (p["x"] - x0) ** 2 + (p["z"] - z0) ** 2

    target = max(reachable, key=dist_sq)

    # ------------------------------------------------------------
    # Phase 3 — Attempt execution toward target
    # NOTE: We intentionally do not assume traces exist yet.
    # We simply advance stepwise, replanning locally.
    # ------------------------------------------------------------
    execution_result = "no_progress"

    for _ in range(8):  # conservative hard bound
        step = executor.execute_action_with_log({"type": "nav-advance", "target": 1}, "path-explore")
        if step.get("status") != "success":
            execution_result = "blocked"
            break
        status = executor.execute_action_with_log({"type": "mc-status"}, "path-explore")
        if status.get("status") != "success":
            execution_result = "status_failed"
            break
        pos = status.get("data", {}).get("position", {})
        if round(pos.get("x", 0)) == target["x"] and round(pos.get("z", 0)) == target["z"]:
            execution_result = "reached"
            break

    # ------------------------------------------------------------
    # Phase 4 — Map reconciliation (minimal)
    # ------------------------------------------------------------
    executor.execute_action_with_log({"type": "mc-map-update"}, "path-explore")

    # ------------------------------------------------------------
    # Final outcome
    # ------------------------------------------------------------
    return executor._create_uniform_return("success", value={"outcome": execution_result, "target": {"x": target["x"], "z": target["z"]}})


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("path-explore module loaded")
