"""
Minecraft place-until-supported tool.
Create reliable footing by placing blocks until observed support becomes provably solid.
"""

import logging
import time
from typing import Any, Dict, Optional
from collections import defaultdict

logger = logging.getLogger(__name__)

# Solid blocks that can provide support (expanded list)
SOLID_BLOCKS = {
    # User's base list
    'stone', 'cobblestone', 'dirt', 'planks', 'gravel', 'netherrack',
    # Common solid blocks
    'grass_block', 'sand', 'sandstone', 'red_sandstone', 'stone_bricks', 'brick',
    'concrete', 'terracotta', 'andesite', 'diorite', 'granite', 'deepslate',
    'blackstone', 'basalt', 'end_stone', 'obsidian', 'cobbled_deepslate',
    # Wood variants (planks covers most, but include logs)
    'oak_planks', 'spruce_planks', 'birch_planks', 'jungle_planks', 'acacia_planks', 'dark_oak_planks',
    'crimson_planks', 'warped_planks', 'mangrove_planks', 'cherry_planks',
    'oak_log', 'spruce_log', 'birch_log', 'jungle_log', 'acacia_log', 'dark_oak_log',
    'crimson_stem', 'warped_stem', 'mangrove_log', 'cherry_log',
    # Stone variants
    'smooth_stone', 'polished_andesite', 'polished_diorite', 'polished_granite',
    'polished_deepslate', 'polished_blackstone', 'polished_basalt',
    # Concrete and terracotta variants (base colors)
    'white_concrete', 'orange_concrete', 'magenta_concrete', 'light_blue_concrete',
    'yellow_concrete', 'lime_concrete', 'pink_concrete', 'gray_concrete',
    'light_gray_concrete', 'cyan_concrete', 'purple_concrete', 'blue_concrete',
    'brown_concrete', 'green_concrete', 'red_concrete', 'black_concrete',
    # Nether blocks
    'nether_bricks', 'red_nether_bricks', 'soul_sand', 'soul_soil',
    # End blocks
    'end_stone_bricks', 'purpur_block', 'purpur_pillar',
    # Other solid blocks
    'clay', 'packed_mud', 'mud_bricks', 'prismarine', 'dark_prismarine', 'prismarine_bricks',
    'quartz_block', 'smooth_quartz', 'redstone_block', 'lapis_block', 'emerald_block',
    'diamond_block', 'gold_block', 'iron_block', 'coal_block',
}


def _find_best_block_from_inventory(executor) -> Optional[str]:
    """
    Find the most abundant solid block from inventory.
    
    Returns:
        Block name (with minecraft: prefix stripped) or None if no solid blocks found
    """
    try:
        # Query inventory
        inv_result = executor.execute_action({"type": "mc-inventory"})
        if inv_result.get("status") != "success":
            logger.warning("mc-place-until-supported: Failed to query inventory for auto-selection")
            return None
        
        inv_data = inv_result.get("data", {})
        slots = inv_data.get("slots", [])
        equipped = inv_data.get("equipped", {})
        
        # Count blocks by name (normalize minecraft: prefix)
        block_counts = defaultdict(int)
        
        # Count from slots
        for slot in slots:
            item = slot.get("item", "")
            if not item or item == "air":
                continue
            
            # Strip minecraft: prefix if present
            item_name = item.split(":")[-1] if ":" in item else item
            
            # Check if it's a solid block
            if item_name in SOLID_BLOCKS:
                count = slot.get("count", 0)
                block_counts[item_name] += count
        
        # Count from equipped items
        for hand_type in ["hand", "offhand"]:
            item = equipped.get(hand_type)
            if not item or item == "air":
                continue
            
            item_name = item.split(":")[-1] if ":" in item else item
            if item_name in SOLID_BLOCKS:
                # Equipped items typically count as 1, but we'll add it
                block_counts[item_name] += 1
        
        if not block_counts:
            logger.warning("mc-place-until-supported: No solid blocks found in inventory for auto-selection")
            return None
        
        # Find block with highest count
        best_block = max(block_counts.items(), key=lambda x: x[1])
        logger.info(f"mc-place-until-supported: Auto-selected '{best_block[0]}' ({best_block[1]} available)")
        return best_block[0]
        
    except Exception as e:
        logger.error(f"mc-place-until-supported: Error during inventory auto-selection: {e}")
        return None


def tool(input_value=None, **kwargs):
    """
    Place blocks until support becomes provably solid.
    
    Args:
        input_value: Item/block name to place (optional, auto-selected from inventory if not provided)
        item: Item/block name to place (alternative to input_value, optional)
        placement_policy: one of {"underfoot", "forward-underfoot", "lateral"} (default: "underfoot")
        max_attempts: integer maximum placement attempts (default: 3)
        verify_delay: float seconds to wait before re-observation (default: 0.0)
        
    Returns:
        Dict with result using uniform return format.
        Outcome values: "SUPPORTED", "NO_PLACEABLE_TARGET", "PLACEMENT_IMPOSSIBLE", 
        "INVENTORY_EXHAUSTED", "SUPPORT_AMBIGUOUS"
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    # Parse parameters - item is now optional
    item = kwargs.get("item") or input_value or ""
    
    # Auto-select from inventory if not provided
    if not isinstance(item, str) or not item.strip():
        item = _find_best_block_from_inventory(executor)
        if not item:
            return executor._create_uniform_return(
                'failed',
                value="item/block name required and no solid blocks found in inventory",
                reason="no_item_available"
            )
    
    placement_policy = kwargs.get("placement_policy", "underfoot").lower()
    if placement_policy not in ["underfoot", "forward-underfoot", "lateral"]:
        return executor._create_uniform_return(
            'failed',
            value=f"Invalid placement_policy: {placement_policy}. Must be one of: underfoot, forward-underfoot, lateral",
            reason="invalid_policy"
        )
    
    max_attempts = int(kwargs.get("max_attempts", 3))
    verify_delay = float(kwargs.get("verify_delay", 0.0))
    
    # Initialize
    attempt_count = 0
    log_messages = []
    logger.info(f"mc-place-until-supported: Starting placement (item={item}, policy={placement_policy}, max_attempts={max_attempts})")
    
    # Determine placement parameters based on policy
    # Uses agent-relative coordinates: dx (right/left), dy (up/down), dz (forward/back)
    place_params = {"item": item}

    if placement_policy == "underfoot":
        # Place block directly underfoot: anchor at (0, -1, 0), face="top"
        # Anchor is block below agent, place on top of it = at agent's feet
        place_params["dx"] = 0.0
        place_params["dy"] = -1.0
        place_params["dz"] = 0.0
        place_params["face"] = "top"
    elif placement_policy == "forward-underfoot":
        # Place block forward and underfoot: anchor at (0, -1, 1), face="top"
        # Anchor is block forward-and-below, place on top = forward at agent's feet level
        place_params["dx"] = 0.0
        place_params["dy"] = -1.0
        place_params["dz"] = 1.0
        place_params["face"] = "top"
    elif placement_policy == "lateral":
        # Place block laterally forward: anchor at (0, 0, 1), face depends on yaw
        # For simplicity, use "north" which places south of anchor (toward agent if facing north)
        place_params["dx"] = 0.0
        place_params["dy"] = 0.0
        place_params["dz"] = 1.0
        place_params["face"] = "north"
    
    # Main loop
    while attempt_count < max_attempts:
        attempt_count += 1
        logger.info(f"mc-place-until-supported: Attempt {attempt_count}/{max_attempts}")
        
        # Step 1: Attempt placement
        place_result = executor.execute_action_with_log({"type": "mc-place", **place_params}, "mc-place-until-supported")
        
        if place_result.get("status") != "success":
            # Placement request failed
            reason = place_result.get("reason", "unknown error")
            
            # Check if it's an inventory issue
            place_data = place_result.get("data", {})
            if not isinstance(place_data, dict):
                place_data = {}
            error_code = place_data.get("error_code", "")
            if "inventory" in error_code.lower() or "not in inventory" in reason.lower():
                outcome = "INVENTORY_EXHAUSTED"
                result_text = f"Place-until-supported terminated: INVENTORY_EXHAUSTED (no blocks available after {attempt_count - 1} attempt(s))"
                logger.warning(f"mc-place-until-supported: {result_text}")
                log_messages.append(f"Attempt {attempt_count}: Inventory exhausted")
                break
            
            # Check if it's a placement target issue
            if "reference" in reason.lower() or "target" in reason.lower() or "position" in reason.lower():
                outcome = "NO_PLACEABLE_TARGET"
                result_text = f"Place-until-supported terminated: NO_PLACEABLE_TARGET (no valid placement location after {attempt_count - 1} attempt(s))"
                logger.warning(f"mc-place-until-supported: {result_text}")
                log_messages.append(f"Attempt {attempt_count}: No placeable target")
                break
            
            # Other failure - treat as placement impossible
            outcome = "PLACEMENT_IMPOSSIBLE"
            result_text = f"Place-until-supported terminated: PLACEMENT_IMPOSSIBLE (placement failed: {reason} after {attempt_count - 1} attempt(s))"
            logger.warning(f"mc-place-until-supported: {result_text}")
            log_messages.append(f"Attempt {attempt_count}: Placement failed - {reason}")
            break
        
        # Step 2: Wait for verification delay if specified
        if verify_delay > 0:
            time.sleep(verify_delay)
            log_messages.append(f"Attempt {attempt_count}: Waited {verify_delay}s for placement to settle")
        
        # Step 3: Observe blocks for support verification
        obs_result = executor.execute_action_with_log({"type": "mc-observe"}, "mc-place-until-supported")
        
        if obs_result.get("status") != "success":
            # Observation failed - treat as ambiguous
            outcome = "SUPPORT_AMBIGUOUS"
            result_text = f"Place-until-supported terminated: SUPPORT_AMBIGUOUS (observation failed after {attempt_count} attempt(s))"
            logger.warning(f"mc-place-until-supported: {result_text}")
            log_messages.append(f"Attempt {attempt_count}: Observation failed")
            break
        
        obs_data = obs_result.get("data", {})
        support_info = obs_data.get("support", {})
        
        # Step 4: Evaluate support at current footing
        here_support = support_info.get("here", {})
        here_type = here_support.get("type", "unknown")
        
        if here_type == "solid":
            outcome = "SUPPORTED"
            result_text = f"Place-until-supported completed: SUPPORTED (solid footing achieved after {attempt_count} attempt(s))"
            logger.info(f"mc-place-until-supported: {result_text}")
            log_messages.append(f"Attempt {attempt_count}: Support confirmed solid")
            break
        
        # Support not yet solid, continue loop
        log_messages.append(f"Attempt {attempt_count}: Placement accepted, but support still {here_type}")
    
    # Check if loop exited without achieving solid support
    if 'outcome' not in locals() or outcome is None:
        outcome = "PLACEMENT_IMPOSSIBLE"
        result_text = f"Place-until-supported terminated: PLACEMENT_IMPOSSIBLE (max_attempts={max_attempts} reached without achieving solid support)"
        logger.warning(f"mc-place-until-supported: {result_text}")
    
    # Build result text with log summary
    if log_messages:
        result_text += "\n" + "\n".join(log_messages)
    
    # Determine success based on outcome
    is_success = (outcome == "SUPPORTED")
    
    # Extract metadata fields for extra (excluding redundant success field)
    extra_metadata = {
        "outcome": outcome,
        "attempts_made": attempt_count,
        "max_attempts": max_attempts,
        "placement_policy": placement_policy,
        "item": item,
        "verify_delay": verify_delay,
        "log": log_messages
    }
    
    status = 'success' if is_success else 'failed'
    return executor._create_uniform_return(status, value=result_text, extra=extra_metadata)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("dirt", placement_policy="underfoot", max_attempts=3)
    print(result)

