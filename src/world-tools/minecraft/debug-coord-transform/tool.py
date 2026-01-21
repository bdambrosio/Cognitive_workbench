"""
Debug tool: Transform coordinates between agent-relative and world-relative.

Hidden from planner catalog - for testing/debugging only.
"""

import sys
import os
from typing import Any, Dict

# Ensure minecraft world-tools root is importable
_THIS_DIR = os.path.dirname(__file__)
_MINECRAFT_TOOLS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _MINECRAFT_TOOLS_ROOT not in sys.path:
    sys.path.insert(0, _MINECRAFT_TOOLS_ROOT)

from nav_core import agent_rel_to_world, world_to_agent_rel


def tool(input_value=None, **kwargs):
    """
    Transform coordinates between agent-relative and world-relative.
    
    Args:
        dx: float - X coordinate
        dy: float - Y coordinate
        dz: float - Z coordinate
        yaw: float - Agent yaw in degrees (optional, fetched from status if not provided)
        direction: string - "agent-to-world" or "world-to-agent" (default: "agent-to-world")
    
    Returns:
        Uniform return format with transformed coordinates.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    dx = kwargs.get("dx")
    dy = kwargs.get("dy")
    dz = kwargs.get("dz")
    direction = kwargs.get("direction", "agent-to-world")
    
    if dx is None or dy is None or dz is None:
        return executor._create_uniform_return(
            "failed",
            value="dx, dy, dz required",
            reason="missing_coordinates"
        )
    
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    
    # Get yaw (from parameter or current status)
    yaw = kwargs.get("yaw")
    if yaw is None:
        try:
            status_result = executor.execute_action_with_log({"type": "mc-status"}, "debug-coord-transform")
            if status_result.get("status") == "success":
                yaw = status_result.get("data", {}).get("yaw", 0.0)
            else:
                yaw = 0.0
        except Exception:
            yaw = 0.0
    else:
        yaw = float(yaw)
    
    # Transform coordinates
    if direction == "agent-to-world":
        world_dx, world_dy, world_dz = agent_rel_to_world(dx, dy, dz, yaw)
        result_coords = {"dx": world_dx, "dy": world_dy, "dz": world_dz}
        summary = f"Agent-relative [{dx},{dy},{dz}] @ yaw={yaw}° → World-relative [{world_dx:.2f},{world_dy:.2f},{world_dz:.2f}]"
    elif direction == "world-to-agent":
        agent_dx, agent_dy, agent_dz = world_to_agent_rel(dx, dy, dz, yaw)
        result_coords = {"dx": agent_dx, "dy": agent_dy, "dz": agent_dz}
        summary = f"World-relative [{dx},{dy},{dz}] @ yaw={yaw}° → Agent-relative [{agent_dx:.2f},{agent_dy:.2f},{agent_dz:.2f}]"
    else:
        return executor._create_uniform_return(
            "failed",
            value=f"Invalid direction: {direction} (must be 'agent-to-world' or 'world-to-agent')",
            reason="invalid_direction"
        )
    
    return executor._create_uniform_return(
        "success",
        value=summary,
        extra={
            "input": {"dx": dx, "dy": dy, "dz": dz, "yaw": yaw, "direction": direction},
            "output": result_coords
        }
    )
