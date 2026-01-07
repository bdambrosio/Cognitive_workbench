"""
Navigation move primitive.

Attempt exactly ONE forward adjacent move with verification, snap-to-grid, and nav history update.
"""

import logging
import os
import sys
from typing import Any, Dict

# Ensure minecraft world-tools root is importable (src/world-tools/minecraft)
_THIS_DIR = os.path.dirname(__file__)
_MINECRAFT_TOOLS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _MINECRAFT_TOOLS_ROOT not in sys.path:
    sys.path.insert(0, _MINECRAFT_TOOLS_ROOT)

from infospace_executor import InfospaceExecutor
from nav_core import (
    DEFAULT_MINECRAFT_URL,
    call_move_endpoint,
    calculate_snap_position_and_yaw,
    snap_to_position,
    update_nav_state,
)

logger = logging.getLogger(__name__)


def tool(input_value=None, **kwargs):
    """
    Attempt exactly ONE forward step (atomic nav primitive).

    Args:
        step_duration: float seconds for the move (default: 0.2)
        max_abs_delta_y: float tolerance for Y noise (default: 0.2)
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    step_duration = float(kwargs.get("step_duration", 0.2))
    max_abs_delta_y = float(kwargs.get("max_abs_delta_y", 0.2))
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL

    # --- Initial status ---
    status_before = executor.execute_action_with_log({"type": "mc-status"}, "nav-move")
    if status_before.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain initial status",
            reason="status_failed",
        )

    start_pos = status_before["data"]["position"]
    start_y = start_pos.get("y")
    current_yaw = status_before["data"].get("yaw")

    # --- Attempt move ---
    move_data = call_move_endpoint(minecraft_url, forward=True, duration=step_duration, check_collision=True)
    if move_data is None or not move_data.get("ok"):
        return executor._create_uniform_return(
            "failed",
            value="Move request failed",
            reason="move_failed",
            extra={"from": start_pos, "to": None}
        )

    move_status = move_data.get("status", "success")

    if move_status == "collision":
        # Observe anyway to expose clearance facts
        obs = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "nav-move")
        clear = obs.get("data", {}).get("clear", {}) if obs.get("status") == "success" else {}

        return executor._create_uniform_return(
            "failed",
            value="Movement blocked by collision",
            reason="collision",
            extra={
                "from": start_pos,
                "to": start_pos,
                "clear_fwd_body": clear.get("fwd", {}).get("body"),
                "clear_fwd_head": clear.get("fwd", {}).get("head"),
            }
        )

    if move_status == "fell":
        status_after = executor.execute_action_with_log({"type": "mc-status"}, "nav-move")
        end_pos = status_after.get("data", {}).get("position")

        if end_pos:
            snap_pos, _, _ = calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
            snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
            end_pos = snap_pos
            pose = {
                "x": end_pos["x"],
                "y": end_pos["y"],
                "z": end_pos["z"],
                "yaw": status_after.get("data", {}).get("yaw", current_yaw),
            }
            update_nav_state(executor, pose=pose, support_here="unknown", fell=True, was_fall=True)

        return executor._create_uniform_return(
            "failed",
            value="Fall detected during movement",
            reason="fell",
            extra={"fell": True, "from": start_pos, "to": end_pos}
        )

    # --- Observe landing ---
    obs = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "nav-move")
    if obs.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to observe blocks at destination",
            reason="observation_failed",
            extra={"from": start_pos, "to": start_pos}
        )

    support_here = obs.get("data", {}).get("support", {}).get("here", {})
    support_type = support_here.get("type", "unknown")
    clear = obs.get("data", {}).get("clear", {})

    # --- Final status ---
    status_after = executor.execute_action_with_log({"type": "mc-status"}, "nav-move")
    if status_after.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain final status",
            reason="observation_failed",
            extra={"from": start_pos, "to": start_pos}
        )

    end_pos = status_after["data"]["position"]
    final_yaw = status_after["data"].get("yaw", current_yaw)
    end_y = end_pos.get("y")

    delta_y = None
    if isinstance(start_y, (int, float)) and isinstance(end_y, (int, float)):
        delta_y = end_y - start_y

    # --- Vertical sanity check ---
    if delta_y is not None and abs(delta_y) > max_abs_delta_y:
        snap_pos, _, _ = calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
        snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
        end_pos = snap_pos
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
        update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

        reason_text = f"Unexpected vertical change: {delta_y:.2f}"
        return executor._create_uniform_return(
            "failed",
            value=reason_text,
            reason="unexpected_vertical_change",
            extra={
                "from": start_pos,
                "to": end_pos,
                "delta_y": delta_y,
                "clear_fwd_body": clear.get("fwd", {}).get("body"),
                "clear_fwd_head": clear.get("fwd", {}).get("head"),
            },
        )

    # --- Landing support check ---
    if support_type not in {"solid", "unsafe"}:
        snap_pos, _, _ = calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
        snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
        end_pos = snap_pos
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
        update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

        reason_text = f"Support ambiguous: {support_type}"
        return executor._create_uniform_return(
            "failed",
            value=reason_text,
            reason="support_ambiguous",
            extra={
                "from": start_pos,
                "to": end_pos,
                "support_here": support_type,
            },
        )

    # --- Success: Snap to destination block center and set yaw ---
    snap_pos, _, _ = calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
    snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
    end_pos = snap_pos

    pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
    update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

    return executor._create_uniform_return(
        "success",
        value="Move completed successfully",
        extra={
            "from": start_pos,
            "to": end_pos,
            "delta_y": delta_y,
            "support_here": support_type,
            "clear_fwd_body": clear.get("fwd", {}).get("body"),
            "clear_fwd_head": clear.get("fwd", {}).get("head"),
        },
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-move module loaded")
