"""
Minecraft place tool.
Build / modify world - place blocks.
"""

import logging
import os
import requests
import sys
from pathlib import Path
from typing import Any, Dict
import time

logger = logging.getLogger(__name__)

# Session-local rolling occupancy grid (agent-owned)
try:
    from local_grid import set_cell
except Exception:
    set_cell = None  # optional; do not break mc-place

# Import nav_core helper for coordinate conversion
_THIS_DIR = os.path.dirname(__file__)
_NAV_CORE_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _NAV_CORE_DIR not in sys.path:
    sys.path.insert(0, _NAV_CORE_DIR)
from nav_core import dx_dy_dz_to_absolute

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def _format_af1_text(af1: Dict[str, Any]) -> str:
    """Compact AF1 string (best-effort)."""
    if not isinstance(af1, dict):
        return ""
    yaw = af1.get("yaw", af1.get("yaw_cardinal"))
    v = af1.get("vertical", {}) if isinstance(af1.get("vertical"), dict) else {}
    nav = af1.get("nav", {}) if isinstance(af1.get("nav"), dict) else {}
    climb = nav.get("climb", nav.get("climb_forward_by_yaw", {}))
    placement = af1.get("placement", {}) if isinstance(af1.get("placement"), dict) else {}
    pending_targets = placement.get("pending_targets", []) if isinstance(placement.get("pending_targets"), list) else []
    anchors = af1.get("anchors", []) if isinstance(af1.get("anchors"), list) else []
    lines = [
        f"AF1 yaw={yaw}",
        f"Vertical: support_depth={v.get('support_depth','unknown')} gap_like={bool(v.get('gap_like'))} up_blk={v.get('up_blk', v.get('up_block'))} down_blk={v.get('down_blk', v.get('down_block'))}",
    ]
    if isinstance(climb, dict) and climb:
        # Support both v2 and v0.1 formats
        if any(isinstance(climb.get(k), str) for k in ("0", "90", "180", "270")):
            climb_s = " ".join([f"{k}:{climb.get(k,'?')}" for k in ("0", "90", "180", "270")])
        else:
            climb_s = " ".join([f"{k}:{(climb.get(k, {}) or {}).get('status','?')}" for k in ("0", "90", "180", "270")])
        lines.append(f"nav-climb by yaw: {climb_s}")
    if pending_targets:
        lines.append(f"placement: pending_targets={len(pending_targets)} (must mc-observe next)")
    if anchors:
        a0 = anchors[0]
        lines.append(f"anchors: n={len(anchors)} anchor0={[a0.get('dx'), a0.get('dy'), a0.get('dz')]} faces={a0.get('faces')}")
    return "\n".join(lines).strip()


def tool(input_value=None, **kwargs):
    """
    Place a block - build/modify world.
    
    Args:
        input_value: Item/block name to place (preferred)
        item: Item/block name to place (alternative to input_value)
        dx: float - world-relative X offset to reference block (positive = east, negative = west)
        dy: float - world-relative Y offset to reference block (positive = up, negative = down)
        dz: float - world-relative Z offset to reference block (positive = south, negative = north)
        face: string - face of reference block to place against (absolute: "top", "bottom", "north", "south", "east", "west")
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
        Note: Placement is asynchronous - the request may be accepted but placement may fail
        due to Minecraft physics (e.g., block already exists, insufficient space, item not in inventory).
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    item = kwargs.get("item") or input_value or ""
    if not isinstance(item, str) or not item.strip():
        return executor._create_uniform_return(
            'failed',
            value="item/block name required (string)",
            reason="missing_item"
        )
    
    # Get dx, dy, dz (world-relative coordinates, origin at agent)
    dx = kwargs.get("dx")
    dy = kwargs.get("dy")
    dz = kwargs.get("dz")
    
    if dx is None or dy is None or dz is None:
        return executor._create_uniform_return(
            'failed',
            value="reference block position required (dx, dy, dz - world-relative offsets from agent)",
            reason="missing_position"
        )
    
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    dx_req, dy_req, dz_req = dx, dy, dz
    
    # Get agent position to convert dx,dy,dz to absolute
    try:
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-place")
        if status_result.get("status") != "success":
            return executor._create_uniform_return(
                'failed',
                value="Failed to get agent position for coordinate conversion",
                reason="status_failed"
            )
        agent_pos = status_result.get("data", {}).get("position", {})
        if not isinstance(agent_pos, dict):
            return executor._create_uniform_return(
                'failed',
                value="Invalid agent position data",
                reason="invalid_position"
            )
    except Exception as e:
        logger.error(f"mc-place: Failed to get agent position: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to get agent position: {e}",
            reason="status_failed"
        )
    
    # Convert dx,dy,dz to absolute block coordinates
    abs_x, abs_y, abs_z = dx_dy_dz_to_absolute(dx, dy, dz, agent_pos)
    
    # Build position parameters - pass absolute to bridge
    place_params = {
        "item": item,
        "ref": {
            "pos": {
                "x": float(abs_x),
                "y": float(abs_y),
                "z": float(abs_z)
            }
        }
    }
    
    # Face is required (convert "top"/"bottom" to "up"/"down" if needed)
    face = kwargs.get("face")
    face_for_af1 = face
    if not face:
        return executor._create_uniform_return(
            'failed',
            value="face required (top, bottom, north, south, east, west)",
            reason="missing_face"
        )
    if face == "top":
        face = "up"
    elif face == "bottom":
        face = "down"
    place_params["face"] = face
    
    try:
        url = f"{minecraft_url}/act/place"
        logger.info(f"🔍 mc-place: POST {url} body={place_params}")
        response = requests.post(url, json=place_params, timeout=30.0)
        logger.info(f"📥 mc-place: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-place: Response body={data}")
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Place failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                reason="placement_failed"
            )
        
        # Placement request accepted - note that actual placement is asynchronous
        # and may fail due to Minecraft physics (block already exists, insufficient space, etc.)
        placed_info = data.get("placed", {})
        placed_pos = placed_info.get("position", {})
        
        # Get agent position for relative coordinate conversion (text output only)
        agent_pos = None
        try:
            status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-place")
            if status_result.get("status") == "success":
                agent_pos = status_result.get("data", {}).get("position", {})
        except Exception:
            pass
        
        if placed_pos and agent_pos:
            # Convert to relative coordinates for text output
            dx = int(round(placed_pos.get('x', 0) - agent_pos.get('x', 0)))
            dy = int(round(placed_pos.get('y', 0) - agent_pos.get('y', 0)))
            dz = int(round(placed_pos.get('z', 0) - agent_pos.get('z', 0)))
            pos_str = f"[{dx},{dy},{dz}]"
            result_text = f"Place request accepted: {item} at {pos_str} (face={face})"
        elif placed_pos:
            # Fallback to absolute if agent position unavailable
            pos_str = f"({placed_pos.get('x', 0)}, {placed_pos.get('y', 0)}, {placed_pos.get('z', 0)})"
            result_text = f"Place request accepted: {item} at {pos_str} (face={face})"
        else:
            result_text = f"Place request accepted: {item} (face={face})"
        
        # Extract metadata fields for extra
        extra_metadata = {
            "status": "accepted",  # Request accepted, but placement may still fail asynchronously
            "placed": placed_info
        }
        # Merge API response data into extra
        extra_metadata.update(data)

        # Side-effect: optimistic local grid update (placed block becomes occupied)
        # placed_pos is already absolute from bridge response
        try:
            if set_cell is not None and isinstance(placed_pos, dict):
                x = placed_pos.get("x")
                y = placed_pos.get("y")
                z = placed_pos.get("z")
                if x is not None and y is not None and z is not None:
                    set_cell(executor, int(x), int(y), int(z), str(item))
        except Exception:
            pass
        
        # Side-effect: update persistent spatial map (non-fatal)
        # Note: placement is async, so observation may capture intermediate state
        # Next observation will correct any inconsistencies
        try:
            map_update_result = executor.execute_action_with_log({"type": "mc-map-update"}, "mc-place")
            if map_update_result.get("status") != "success":
                logger.debug(f"mc-place: spatial map update failed (non-fatal): {map_update_result.get('reason', 'unknown')}")
        except Exception as e:
            logger.debug(f"mc-place: spatial map update skipped (non-fatal): {e}")

        # AF1 v2: record pending placement target (clears on next successful mc-observe)
        try:
            af1 = executor.get_world_state("af1")
            if not isinstance(af1, dict):
                af1 = {"af1_version": "2"}
            placement = af1.get("placement")
            if not isinstance(placement, dict):
                placement = {}
            pending_targets = placement.get("pending_targets")
            if not isinstance(pending_targets, list):
                pending_targets = []
            af1["af1_version"] = "2"
            pending_targets.insert(0, {"dx": float(dx_req), "dy": float(dy_req), "dz": float(dz_req), "face": str(face_for_af1)})
            placement["pending_targets"] = pending_targets[:3]
            placement["pending"] = bool(placement["pending_targets"])
            af1["placement"] = placement
            executor.set_world_state("af1", af1)
            executor.set_world_state("af1_text", _format_af1_text(af1))
        except Exception:
            pass
        
        return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft place request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("dirt", dx=0, dy=0, dz=1, face="north")
    print(result)

