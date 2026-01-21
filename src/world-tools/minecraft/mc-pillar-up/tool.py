"""
Minecraft pillar-up tool.
Atomic jump + place operation for ascending by building under self.
"""

import logging
import math
import os
import requests
import sys
import time
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

# Import nav_core helpers
_THIS_DIR = os.path.dirname(__file__)
_NAV_CORE_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _NAV_CORE_DIR not in sys.path:
    sys.path.insert(0, _NAV_CORE_DIR)
from nav_core import ensure_grid_aligned, snap_to_position

# Session-local rolling occupancy grid (agent-owned)
try:
    from local_grid import set_cell, get_cell_name, is_air_name
except Exception:
    set_cell = None
    get_cell_name = None
    is_air_name = None

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")

# Timing constants for jump physics
# Note: HTTP round-trip takes ~200-250ms, during which agent reaches peak
# So we only need a small additional wait (or none) after jump returns
JUMP_TO_PEAK_MS = 50  # Small buffer after HTTP returns (agent near peak during call)
PEAK_PLACE_WINDOW_MS = 100  # Window at peak where placement is safe
POST_PLACE_SETTLE_MS = 400  # Time to wait after place for landing


def _get_status(minecraft_url: str, timeout: float = 5.0) -> Optional[Dict]:
    """Get current bot status from bridge."""
    try:
        response = requests.get(f"{minecraft_url}/status", timeout=timeout)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        logger.warning(f"mc-pillar-up: status request failed: {e}")
        return None


def _call_move_endpoint(minecraft_url: str, forward: bool = False, jump: bool = True, duration_ms: int = 100) -> Optional[Dict]:
    """
    Call bridge /act/move endpoint for jump.

    Args:
        minecraft_url: Bridge URL
        forward: Whether to move forward (False for pure vertical jump)
        jump: Whether to jump
        duration_ms: Duration of movement in milliseconds

    Returns:
        Response dict or None on error
    """
    move_params = {
        "forward": forward,
        "jump": jump,
        "duration_ms": duration_ms,
        "check_collision": False,  # Don't abort on collision during pillar
        "wait_for_stable": False   # Return immediately, don't wait for landing
    }

    try:
        url = f"{minecraft_url}/act/move"
        logger.info(f"mc-pillar-up: JUMP POST {url} body={move_params}")
        response = requests.post(url, json=move_params, timeout=10.0)
        response.raise_for_status()
        result = response.json()
        logger.info(f"mc-pillar-up: JUMP response status={response.status_code}, result={result}")
        return result
    except Exception as e:
        logger.warning(f"mc-pillar-up: move request failed: {e}")
        return None


def _call_place_endpoint(minecraft_url: str, item: str, ref_pos: Dict, face: str) -> Optional[Dict]:
    """
    Call bridge /act/place endpoint directly (bypasses mc-place tool's overlap check).

    Args:
        minecraft_url: Bridge URL
        item: Block name to place
        ref_pos: Reference/anchor position dict with x, y, z
        face: Face to place against ("up", "down", "north", "south", "east", "west")

    Returns:
        Response dict or None on error
    """
    place_params = {
        "item": item,
        "ref": {
            "pos": {
                "x": float(ref_pos["x"]),
                "y": float(ref_pos["y"]),
                "z": float(ref_pos["z"])
            }
        },
        "face": face
    }

    try:
        url = f"{minecraft_url}/act/place"
        logger.info(f"mc-pillar-up: PLACE POST {url} body={place_params}")
        response = requests.post(url, json=place_params, timeout=10.0)
        response.raise_for_status()
        result = response.json()
        logger.info(f"mc-pillar-up: PLACE response status={response.status_code}, result={result}")
        return result
    except Exception as e:
        logger.warning(f"mc-pillar-up: place request failed: {e}")
        return None


def tool(input_value=None, **kwargs):
    """
    Atomic pillar-up: jump and place block under self to ascend by 1 block.

    This tool handles the timing-critical sequence of:
    1. Jump straight up
    2. At peak height, place block at original feet level
    3. Land on the new block

    Args:
        input_value: Item/block name to place (preferred)
        item: Item/block name to place (alternative to input_value)

    Returns:
        Dict with result using uniform return format.
        On success: agent is standing 1 block higher than before.
        On failure: agent returns to original position (or falls).
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL

    # Get item/block name
    item = kwargs.get("item") or input_value or ""
    if not isinstance(item, str) or not item.strip():
        return executor._create_uniform_return(
            'failed',
            value="item/block name required (string)",
            reason="missing_item"
        )
    item = item.strip()

    # Step 1: Get initial position and ensure grid-aligned
    status_before = executor.execute_action_with_log({"type": "mc-status"}, "mc-pillar-up")
    if status_before.get("status") != "success":
        return executor._create_uniform_return(
            'failed',
            value="Failed to get initial status",
            reason="status_failed"
        )

    status_data = status_before.get("data", {})
    start_pos = status_data.get("position", {})
    start_yaw = status_data.get("yaw", 0.0)

    if not isinstance(start_pos, dict) or "x" not in start_pos:
        return executor._create_uniform_return(
            'failed',
            value="Invalid position data from status",
            reason="invalid_position"
        )

    # Ensure grid-aligned before pillar attempt
    try:
        aligned_pos, aligned_yaw = ensure_grid_aligned(executor, minecraft_url, status_data=status_data)
        start_pos = aligned_pos
        start_yaw = aligned_yaw
    except Exception as e:
        logger.warning(f"mc-pillar-up: grid alignment failed (continuing): {e}")

    # Calculate block positions
    start_block_x = int(math.floor(start_pos.get("x", 0)))
    start_block_y = int(math.floor(start_pos.get("y", 0)))  # feet level
    start_block_z = int(math.floor(start_pos.get("z", 0)))

    # Anchor block is at y-1 (block below feet), we place on "top" face
    # This will put the new block at start_block_y (original feet level)
    # IMPORTANT: Use integer block coordinates, NOT +0.5 center offset!
    # The bridge uses int() truncation which breaks for negative coords with +0.5
    anchor_pos = {
        "x": float(start_block_x),  # integer block coord (NOT center)
        "y": float(start_block_y - 1),  # block below feet
        "z": float(start_block_z)   # integer block coord (NOT center)
    }
    target_y = start_block_y  # where the new block will be placed

    logger.info(f"mc-pillar-up: Starting at ({start_block_x}, {start_block_y}, {start_block_z}), "
                f"will place {item} at y={target_y}")

    # Step 1b: Check headroom - need y+2 and y+3 to be clear for jump
    # After pillar-up, agent will stand at y+1 with head at y+2
    # Jump peak raises head to ~y+3
    if get_cell_name is not None and is_air_name is not None:
        head_y2 = get_cell_name(executor, start_block_x, start_block_y + 2, start_block_z)
        head_y3 = get_cell_name(executor, start_block_x, start_block_y + 3, start_block_z)

        # Check y+2 (required for post-pillar standing)
        if head_y2 is not None and not is_air_name(head_y2):
            return executor._create_uniform_return(
                'failed',
                value=f"Insufficient headroom: block at y+2 ({head_y2})",
                reason="no_headroom",
                extra={"blocking_y": start_block_y + 2, "block": head_y2}
            )

        # Check y+3 (required for jump peak)
        if head_y3 is not None and not is_air_name(head_y3):
            return executor._create_uniform_return(
                'failed',
                value=f"Insufficient headroom: block at y+3 ({head_y3})",
                reason="no_headroom",
                extra={"blocking_y": start_block_y + 3, "block": head_y3}
            )

        # If cells are unknown (None), we proceed but warn
        if head_y2 is None or head_y3 is None:
            logger.warning(f"mc-pillar-up: headroom cells unknown, proceeding cautiously")

    # Step 2: Initiate jump (pure vertical, no forward movement)
    # Use short duration - we just need to trigger the jump, physics handles the rest
    logger.info(f"mc-pillar-up: [T=0ms] Initiating jump...")
    t_start = time.time()
    jump_result = _call_move_endpoint(minecraft_url, forward=False, jump=True, duration_ms=200)
    t_after_jump = time.time()
    logger.info(f"mc-pillar-up: [T={int((t_after_jump-t_start)*1000)}ms] Jump call returned: {jump_result}")

    if jump_result is None:
        return executor._create_uniform_return(
            'failed',
            value="Jump initiation failed",
            reason="jump_failed"
        )

    # Step 3: Wait for peak height
    # Minecraft jump reaches peak at ~0.35 seconds
    logger.info(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Waiting {JUMP_TO_PEAK_MS}ms for jump peak...")
    time.sleep(JUMP_TO_PEAK_MS / 1000.0)

    # Check intermediate position at peak
    mid_status = _get_status(minecraft_url)
    if mid_status:
        mid_pos = mid_status.get("position", {})
        mid_y = mid_pos.get("y", 0)
        logger.info(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Mid-jump position: y={mid_y:.2f} (started at {start_block_y})")
    else:
        logger.warning(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Could not get mid-jump status")

    # Step 4: Place block at original feet level while airborne
    # We call the bridge directly to bypass mc-place's overlap check
    # (the check would pass anyway since we're airborne, but direct call is faster)
    logger.info(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Placing block at anchor {anchor_pos}...")
    place_result = _call_place_endpoint(minecraft_url, item, anchor_pos, "up")
    logger.info(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Place call returned: {place_result}")

    if place_result is None or not place_result.get("ok"):
        error = place_result.get("error", "unknown") if place_result else "request failed"
        logger.warning(f"mc-pillar-up: placement failed: {error}")

        # Wait for landing before returning failure
        time.sleep(POST_PLACE_SETTLE_MS / 1000.0)

        return executor._create_uniform_return(
            'failed',
            value=f"Block placement failed while airborne: {error}",
            reason="placement_failed",
            extra={
                "anchor": anchor_pos,
                "target_y": target_y,
                "item": item
            }
        )

    # Step 5: Wait for landing
    logger.info(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Waiting {POST_PLACE_SETTLE_MS}ms for landing...")
    time.sleep(POST_PLACE_SETTLE_MS / 1000.0)

    # Step 6: Verify success - check final position
    logger.info(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Checking final position...")
    status_after = executor.execute_action_with_log({"type": "mc-status"}, "mc-pillar-up")
    if status_after.get("status") != "success":
        return executor._create_uniform_return(
            'failed',
            value="Failed to get final status (placement may have succeeded)",
            reason="status_failed_after",
            extra={"placement_accepted": True}
        )

    end_pos = status_after.get("data", {}).get("position", {})
    end_y = end_pos.get("y", 0)
    end_block_y = int(math.floor(end_y))
    delta_y = end_block_y - start_block_y
    logger.info(f"mc-pillar-up: [T={int((time.time()-t_start)*1000)}ms] Final position: y={end_y:.2f} (block {end_block_y}), delta_y={delta_y}")

    # Update local grid with placed block
    try:
        if set_cell is not None:
            set_cell(executor, start_block_x, target_y, start_block_z, item)
    except Exception:
        pass

    # Ensure grid-aligned after pillar
    try:
        ensure_grid_aligned(executor, minecraft_url)
    except Exception:
        pass

    # Observe to update world state
    try:
        executor.execute_action_with_log({"type": "mc-observe"}, "mc-pillar-up")
    except Exception:
        pass

    if delta_y >= 1:
        # Success - agent is now standing on the new block
        result_text = f"Pillar up successful: placed {item}, ascended {delta_y} block(s)"
        logger.info(f"mc-pillar-up: {result_text}")

        return executor._create_uniform_return(
            'success',
            value=result_text,
            extra={
                "delta_y": delta_y,
                "from_y": start_block_y,
                "to_y": end_block_y,
                "placed_at": {"x": start_block_x, "y": target_y, "z": start_block_z},
                "item": item
            }
        )
    else:
        # Placement may have failed or agent didn't land on block
        result_text = f"Pillar up incomplete: delta_y={delta_y} (expected >= 1). " \
                      f"Block may not have been placed or agent didn't land on it."
        logger.warning(f"mc-pillar-up: {result_text}")

        return executor._create_uniform_return(
            'failed',
            value=result_text,
            reason="ascent_failed",
            extra={
                "delta_y": delta_y,
                "from_y": start_block_y,
                "to_y": end_block_y,
                "expected_placed_at": {"x": start_block_x, "y": target_y, "z": start_block_z},
                "item": item
            }
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("mc-pillar-up module loaded")
