"""
Navigation move primitive.

Attempt exactly ONE forward adjacent move with verification, snap-to-grid, and nav history update.
"""

import logging
import math
import os
import sys
from typing import Any, Dict, Optional

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
    ensure_grid_aligned,
    snap_to_position,
    update_nav_state,
)

logger = logging.getLogger(__name__)

# Session-local rolling occupancy grid (agent-owned)
try:
    from local_grid import set_center_from_pose, get_cell_name, is_air_name
except Exception:
    set_center_from_pose = None
    get_cell_name = None
    is_air_name = None


def _to_relative_delta(from_pos: Dict[str, float], to_pos: Optional[Dict[str, float]]) -> Dict[str, float]:
    """Convert absolute positions to relative delta (dx, dy, dz)."""
    if to_pos is None:
        return {"dx": 0.0, "dy": 0.0, "dz": 0.0}
    return {
        "dx": to_pos.get("x", 0.0) - from_pos.get("x", 0.0),
        "dy": to_pos.get("y", 0.0) - from_pos.get("y", 0.0),
        "dz": to_pos.get("z", 0.0) - from_pos.get("z", 0.0),
    }


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

    # --- Ensure grid-aligned before movement (block center, cardinal yaw, pitch 0) ---
    normalized_pose, normalized_yaw = ensure_grid_aligned(executor, minecraft_url, status_data=status_before["data"])
    # Update start_pos and current_yaw to normalized values for consistency
    start_pos = normalized_pose
    current_yaw = normalized_yaw

    # --- Pre-check forward clearance using local_grid ---
    if get_cell_name is not None and is_air_name is not None:
        try:
            bx = int(math.floor(start_pos.get("x", 0.0)))
            by = int(math.floor(start_pos.get("y", 0.0)))
            bz = int(math.floor(start_pos.get("z", 0.0)))
            dx = -math.sin(math.radians(current_yaw))
            dz = -math.cos(math.radians(current_yaw))
            fwd_x = bx + int(round(dx))
            fwd_z = bz + int(round(dz))
            
            # Check forward body and head clearance
            fwd_body_name = get_cell_name(executor, fwd_x, by + 1, fwd_z)
            fwd_head_name = get_cell_name(executor, fwd_x, by + 2, fwd_z)
            
            # If known blockers exist, pre-dig them (up to 2 blocks)
            blockers = []
            if fwd_body_name and not is_air_name(fwd_body_name):
                blockers.append((fwd_x, by + 1, fwd_z))
            if fwd_head_name and not is_air_name(fwd_head_name):
                blockers.append((fwd_x, by + 2, fwd_z))
            
            for b_x, b_y, b_z in blockers[:2]:  # Limit to 2 pre-digs
                try:
                    executor.execute_action_with_log(
                        {"type": "mc-dig", "x": float(b_x), "y": float(b_y), "z": float(b_z)},
                        "nav-move:grid_pre_dig"
                    )
                except Exception:
                    pass
        except Exception:
            pass  # If pre-check fails, proceed with move attempt

    # --- Attempt move ---
    move_data = call_move_endpoint(minecraft_url, forward=True, duration=step_duration, check_collision=True)
    if move_data is None or not move_data.get("ok"):
        return executor._create_uniform_return(
            "failed",
            value="Move request failed",
            reason="move_failed",
            extra={"delta": _to_relative_delta(start_pos, None)}
        )

    move_status = move_data.get("status", "success")

    if move_status == "collision":
        # Observe anyway to expose clearance facts
        obs = executor.execute_action_with_log({"type": "mc-observe"}, "nav-move")
        clear = obs.get("data", {}).get("clear", {}) if obs.get("status") == "success" else {}

        return executor._create_uniform_return(
            "failed",
            value="Movement blocked by collision",
            reason="collision",
            extra={
                "delta": _to_relative_delta(start_pos, start_pos),
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
            try:
                if set_center_from_pose is not None:
                    set_center_from_pose(executor, {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"]})
            except Exception:
                pass
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
            extra={"fell": True, "delta": _to_relative_delta(start_pos, end_pos)}
        )

    # --- Observe landing ---
    obs = executor.execute_action_with_log({"type": "mc-observe"}, "nav-move")
    if obs.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to observe blocks at destination",
            reason="observation_failed",
            extra={"delta": _to_relative_delta(start_pos, start_pos)}
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
            extra={"delta": _to_relative_delta(start_pos, start_pos)}
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
        try:
            if set_center_from_pose is not None:
                set_center_from_pose(executor, {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"]})
        except Exception:
            pass
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
        update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

        reason_text = f"Unexpected vertical change: {delta_y:.2f}"
        return executor._create_uniform_return(
            "failed",
            value=reason_text,
            reason="unexpected_vertical_change",
            extra={
                "delta": _to_relative_delta(start_pos, end_pos),
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
        try:
            if set_center_from_pose is not None:
                set_center_from_pose(executor, {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"]})
        except Exception:
            pass
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
        update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

        reason_text = f"Support ambiguous: {support_type}"
        return executor._create_uniform_return(
            "failed",
            value=reason_text,
            reason="support_ambiguous",
            extra={
                "delta": _to_relative_delta(start_pos, end_pos),
                "support_here": support_type,
            },
        )

    # --- Success: Snap to destination block center and set yaw ---
    snap_pos, _, _ = calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
    snap_to_position(minecraft_url, x=snap_pos["x"], y=snap_pos["y"], z=snap_pos["z"])
    end_pos = snap_pos
    try:
        if set_center_from_pose is not None:
            set_center_from_pose(executor, {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"]})
    except Exception:
        pass

    pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
    update_nav_state(executor, pose=pose, support_here=support_type, fell=False, was_fall=False)

    return executor._create_uniform_return(
        "success",
        value="Move completed successfully",
        extra={
            "delta": _to_relative_delta(start_pos, end_pos),
            "delta_y": delta_y,
            "support_here": support_type,
            "clear_fwd_body": clear.get("fwd", {}).get("body"),
            "clear_fwd_head": clear.get("fwd", {}).get("head"),
        },
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-move module loaded")
