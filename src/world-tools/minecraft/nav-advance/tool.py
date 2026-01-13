"""
Navigation advance composite.

Advance forward by a specified number of blocks using nav conventions (verification, snap-to-grid, nav history).
"""

import logging
import os
import sys
from typing import Any, Dict, Optional

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


def _coerce_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def tool(input_value=None, **kwargs):
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    # Accept 'blocks' (preferred) or 'target' (backward compatibility)
    blocks = _coerce_int(kwargs.get("blocks"))
    if blocks is None:
        blocks = _coerce_int(kwargs.get("target"))  # Backward compatibility
    if blocks is None:
        blocks = _coerce_int(input_value)
    if blocks is None or blocks <= 0:
        return executor._create_uniform_return(
            "failed",
            value="nav-advance requires blocks (positive integer)",
            reason="INVALID_TARGET",
            extra={"stop_reason": "INVALID_TARGET"}
        )

    step_duration = float(kwargs.get("step_duration", 0.2))
    max_abs_delta_y = float(kwargs.get("max_abs_delta_y", 0.2))
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL

    status_before = executor.execute_action_with_log({"type": "mc-status"}, "nav-advance")
    if status_before.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain initial status",
            reason="STATUS_FAILED",
            extra={"stop_reason": "STATUS_FAILED"}
        )

    start_pos = status_before["data"]["position"]
    current_yaw = status_before["data"].get("yaw")

    steps_completed = 0
    last_pos = start_pos
    last_y = start_pos.get("y")

    for _ in range(blocks):
        move_data = call_move_endpoint(minecraft_url, forward=True, duration=step_duration, check_collision=True)
        if move_data is None or not move_data.get("ok"):
            return executor._create_uniform_return(
                "failed",
                value=f"Advance stopped: MOVE_FAILED after {steps_completed}/{blocks}",
                reason="MOVE_FAILED",
            )

        move_status = move_data.get("status", "success")

        if move_status == "collision":
            return executor._create_uniform_return(
                "failed",
                value=f"Advance stopped: COLLISION after {steps_completed}/{blocks}",
                reason="COLLISION",
            )

        if move_status == "fell":
            status_after = executor.execute_action_with_log({"type": "mc-status"}, "nav-advance")
            end_pos = status_after.get("data", {}).get("position")
            if end_pos:
                snap_pos, _, _ = calculate_snap_position_and_yaw(last_pos, end_pos, current_yaw)
                snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
                end_pos = snap_pos
                pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
                update_nav_state(executor, pose=pose, support_here="unknown", fell=True, was_fall=True)

            return executor._create_uniform_return(
                "failed",
                value=f"Advance stopped: FELL after {steps_completed}/{blocks}",
                reason="FELL",
            )

        # Observe for support
        obs = executor.execute_action_with_log({"type": "mc-observe"}, "nav-advance")
        if obs.get("status") != "success":
            return executor._create_uniform_return(
                "failed",
                value=f"Advance stopped: OBSERVATION_FAILED after {steps_completed}/{blocks}",
                reason="OBSERVATION_FAILED",
            )

        support_here = obs.get("data", {}).get("support", {}).get("here", {})
        support_type = support_here.get("type", "unknown")

        status_after = executor.execute_action_with_log({"type": "mc-status"}, "nav-advance")
        if status_after.get("status") != "success":
            return executor._create_uniform_return(
                "failed",
                value=f"Advance stopped: STATUS_FAILED after {steps_completed}/{blocks}",
                reason="STATUS_FAILED",
            )

        end_pos = status_after["data"]["position"]
        end_y = end_pos.get("y")

        delta_y = None
        if isinstance(last_y, (int, float)) and isinstance(end_y, (int, float)):
            delta_y = end_y - last_y

        if delta_y is not None and abs(delta_y) > max_abs_delta_y:
            snap_pos, _, _ = calculate_snap_position_and_yaw(last_pos, end_pos, current_yaw)
            snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
            end_pos = snap_pos
            pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
            update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

            return executor._create_uniform_return(
                "failed",
                value=f"Advance stopped: UNEXPECTED_VERTICAL_CHANGE after {steps_completed}/{blocks}",
                reason="UNEXPECTED_VERTICAL_CHANGE",
            )

        if support_type not in {"solid", "unsafe"}:
            snap_pos, _, _ = calculate_snap_position_and_yaw(last_pos, end_pos, current_yaw)
            snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
            end_pos = snap_pos
            pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
            update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

            return executor._create_uniform_return(
                "failed",
                value=f"Advance stopped: SUPPORT_AMBIGUOUS after {steps_completed}/{blocks}",
                reason="SUPPORT_AMBIGUOUS",
            )

        # Success for this block: snap and record nav state
        snap_pos, _, _ = calculate_snap_position_and_yaw(last_pos, end_pos, current_yaw)
        snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
        end_pos = snap_pos
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
        update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

        steps_completed += 1
        last_pos = end_pos
        last_y = end_pos.get("y")
        # current_yaw remains unchanged (preserved from initial status)

    return executor._create_uniform_return(
        "success",
        value=f"Advance completed: TARGET_REACHED ({steps_completed}/{blocks})",
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-advance module loaded")

