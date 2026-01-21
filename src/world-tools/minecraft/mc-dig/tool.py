"""
Minecraft dig tool.
Remove a block - world manipulation.
"""

import logging
import os
import requests
import sys
import time
from pathlib import Path
from typing import Any, Dict

logger = logging.getLogger(__name__)

# Session-local rolling occupancy grid (agent-owned)
try:
    from local_grid import set_cell
except Exception:
    set_cell = None  # optional; do not break mc-dig

# Import nav_core helper for coordinate conversion
_THIS_DIR = os.path.dirname(__file__)
_NAV_CORE_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _NAV_CORE_DIR not in sys.path:
    sys.path.insert(0, _NAV_CORE_DIR)
from nav_core import dx_dy_dz_to_absolute, ensure_grid_aligned

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Dig/remove a block - world manipulation.
    
    Args:
        input_value: ignored
        dx: float - agent-relative X offset from agent (positive = right, negative = left)
        dy: float - agent-relative Y offset from agent (positive = up, negative = down)
        dz: float - agent-relative Z offset from agent (positive = forward, negative = back)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
        Note: Bridge dig is synchronous with timeout: it returns on completion or timeout.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    # Get dx, dy, dz (world-relative coordinates, origin at agent)
    dx = kwargs.get("dx")
    dy = kwargs.get("dy")
    dz = kwargs.get("dz")
    
    if dx is None or dy is None or dz is None:
        return executor._create_uniform_return(
            'failed',
            value="position required (dx, dy, dz - agent-relative offsets from agent)",
            reason="missing_position"
        )
    
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    
    # Get agent position to convert dx,dy,dz to absolute
    try:
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-dig")
        if status_result.get("status") != "success":
            return executor._create_uniform_return(
                'failed',
                value="Failed to get agent position for coordinate conversion",
                reason="status_failed"
            )
        status_data = status_result.get("data", {})
        agent_pos = status_data.get("position", {})
        agent_yaw = status_data.get("yaw", 0.0)
        if not isinstance(agent_pos, dict):
            return executor._create_uniform_return(
                'failed',
                value="Invalid agent position data",
                reason="invalid_position"
            )
    except Exception as e:
        logger.error(f"mc-dig: Failed to get agent position: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to get agent position: {e}",
            reason="status_failed"
        )
    
    # Convert agent-relative dx,dy,dz to absolute block coordinates
    abs_x, abs_y, abs_z = dx_dy_dz_to_absolute(dx, dy, dz, agent_pos, yaw=agent_yaw)
    
    # Build position parameters - pass absolute to bridge
    dig_params = {
        "pos": {
            "x": float(abs_x),
            "y": float(abs_y),
            "z": float(abs_z)
        }
    }
    
    try:
        url = f"{minecraft_url}/act/dig"
        logger.info(f"🔍 mc-dig: POST {url} body={dig_params}")
        # Increased timeout to 60s - digging blocks can take time, especially hard blocks
        # Bridge returns immediately after initiating dig, but some blocks take 10-30+ seconds to break
        response = requests.post(url, json=dig_params, timeout=60.0)
        logger.info(f"📥 mc-dig: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-dig: Response body={data}")
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Dig failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                reason="dig_failed"
            )
        
        # Dig completes synchronously (or times out) in the bridge.
        dug = data.get("dug", {})
        block_name = dug.get("name", "unknown")
        dug_pos = dug.get("position", {})
        
        # Get agent position for relative coordinate conversion (text output only)
        agent_pos = None
        try:
            status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-dig")
            if status_result.get("status") == "success":
                agent_pos = status_result.get("data", {}).get("position", {})
        except Exception:
            pass
        
        if block_name == "air":
            result_text = f"Dig request accepted: target block is already air"
        elif dug_pos and agent_pos:
            # Convert to relative coordinates for text output
            dx = int(round(dug_pos.get('x', 0) - agent_pos.get('x', 0)))
            dy = int(round(dug_pos.get('y', 0) - agent_pos.get('y', 0)))
            dz = int(round(dug_pos.get('z', 0) - agent_pos.get('z', 0)))
            pos_str = f"[{dx},{dy},{dz}]"
            result_text = f"Dig request accepted: {block_name} at {pos_str}"
        elif dug_pos:
            # Fallback to absolute if agent position unavailable
            pos_str = f"({dug_pos.get('x', 0)}, {dug_pos.get('y', 0)}, {dug_pos.get('z', 0)})"
            result_text = f"Dig request accepted: {block_name} at {pos_str}"
        else:
            result_text = f"Dig request accepted: {block_name}"
        
        # Build structured data dict
        # Extract metadata fields for extra
        extra_metadata = {
            "status": "accepted",  # Request accepted, but digging may still fail asynchronously
            "dug": dug
        }
        # Merge API response data into extra
        extra_metadata.update(data)

        # Side-effect: optimistic local grid update (dug block becomes air)
        # dug_pos is already absolute from bridge response
        try:
            if set_cell is not None and isinstance(dug_pos, dict):
                x = dug_pos.get("x")
                y = dug_pos.get("y")
                z = dug_pos.get("z")
                if x is not None and y is not None and z is not None:
                    set_cell(executor, int(x), int(y), int(z), "air")
        except Exception:
            pass
        
        # Side-effect: update persistent spatial map (non-fatal)
        try:
            map_update_result = executor.execute_action_with_log({"type": "mc-map-update"}, "mc-dig")
            if map_update_result.get("status") != "success":
                logger.debug(f"mc-dig: spatial map update failed (non-fatal): {map_update_result.get('reason', 'unknown')}")
        except Exception as e:
            logger.debug(f"mc-dig: spatial map update skipped (non-fatal): {e}")
        
        # Delay to allow dig animation to start, then restore snapto conditions
        time.sleep(0.3)
        
        # Recheck position and restore grid alignment (block center, cardinal yaw, pitch 0)
        try:
            status_after = executor.execute_action_with_log({"type": "mc-status"}, "mc-dig")
            if status_after.get("status") == "success":
                ensure_grid_aligned(executor, minecraft_url, status_data=status_after.get("data"))
        except Exception as e:
            logger.debug(f"mc-dig: grid alignment restoration skipped (non-fatal): {e}")
        
        # Observe blocks to refresh grid after block removal (like nav-move does)
        try:
            obs = executor.execute_action_with_log({"type": "mc-observe"}, "mc-dig")
            if obs.get("status") != "success":
                logger.debug(f"mc-dig: Observation failed (non-fatal): {obs.get('reason', 'unknown')}")
        except Exception as e:
            logger.debug(f"mc-dig: Observation skipped (non-fatal): {e}")
        
        return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft dig request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(dx=0, dy=-1, dz=1)
    print(result)

