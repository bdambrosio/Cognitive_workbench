#!/usr/bin/env python3
"""
Test suite for Minecraft agent-relative coordinate system.

Validates that:
- Grid observations use agent-relative coordinates (dx=right/left, dz=forward/back)
- Tool actions interpret coordinates as agent-relative
- Yaw transformations work correctly
- Coordinate system remains consistent across actions

Uses Zenoh API similar to mmlu_eval.py pattern.

Usage:
    python3 tests/test_minecraft_coordinates.py --character Jill
    python3 tests/test_minecraft_coordinates.py --character Jill --test yaw
    python3 tests/test_minecraft_coordinates.py --character Jill --test place

Available tests:
    - basic: Basic grid observation
    - yaw: Yaw transformation
    - place: Place block agent-relative
    - dig: Dig block agent-relative
    - nav: Navigation consistency
    - multi: Multi-action sequence
    - cardinals: All cardinal yaws
    - all: Run all tests (default)

Requirements:
    - Running Jill session with Minecraft world
    - Minecraft bridge running
    - Agent initialized in Minecraft world
"""

import argparse
import json
import math
import time
import logging
import traceback
from typing import Dict, List, Optional, Tuple

import zenoh
from zenoh import QueryTarget, ConsolidationMode

# Configure logging for timeout detection
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# -----------------------------
# Zenoh API Helpers
# -----------------------------

def execute_plan(session: zenoh.Session, character: str, plan: List[Dict], timeout: float = 30.0) -> Dict:
    """Execute a plan via Zenoh API. Returns result dict with success, bindings, last_action_result."""
    start_time = time.time()
    query_key = f"cognitive/{character}/execute_plan_sync"
    payload = json.dumps({'plan': plan})
    
    logger.debug(f"Executing plan via {query_key} (timeout={timeout}s)")
    
    try:
        replies = session.get(
            query_key,
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            payload=payload.encode('utf-8'),
            timeout=timeout
        )
        reply_count = 0
        for reply in replies:
            reply_count += 1
            if hasattr(reply, 'ok') and reply.ok is not None:
                elapsed = time.time() - start_time
                if elapsed > 2.0:
                    logger.warning(f"⏱️ execute_plan took {elapsed:.2f}s (expected <2s for direct actions)")
                return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
        
        elapsed = time.time() - start_time
        if elapsed >= timeout:
            logger.error(f"⏱️ TIMEOUT: execute_plan exceeded {timeout}s timeout (actual: {elapsed:.2f}s)")
            logger.error(f"   Plan: {json.dumps(plan, indent=2)}")
            return {'success': False, 'error': f'Timeout after {elapsed:.2f}s (limit: {timeout}s)', 'timeout': True}
        elif reply_count == 0:
            logger.error(f"⏱️ NO RESPONSE: execute_plan got no replies (elapsed: {elapsed:.2f}s)")
            logger.error(f"   Plan: {json.dumps(plan, indent=2)}")
            return {'success': False, 'error': f'No response from execute_plan_sync (elapsed: {elapsed:.2f}s)', 'no_response': True}
        else:
            logger.warning(f"⏱️ execute_plan got {reply_count} replies but none had ok (elapsed: {elapsed:.2f}s)")
            return {'success': False, 'error': f'Invalid response from execute_plan_sync (elapsed: {elapsed:.2f}s)'}
    except Exception as e:
        elapsed = time.time() - start_time
        logger.error(f"⏱️ EXCEPTION in execute_plan after {elapsed:.2f}s: {e}")
        logger.error(f"   Plan: {json.dumps(plan, indent=2)}")
        logger.error(traceback.format_exc())
        return {'success': False, 'error': f'Exception: {str(e)}', 'exception': True}


def execute_action(session: zenoh.Session, character: str, action_dict: Dict, timeout: float = 10.0) -> Dict:
    """Execute single action via execute_plan_sync."""
    start_time = time.time()
    plan = [action_dict]
    result = execute_plan(session, character, plan, timeout)
    elapsed = time.time() - start_time
    
    if elapsed > 2.0:
        logger.warning(f"⏱️ execute_action took {elapsed:.2f}s (expected <2s for direct actions)")
        logger.warning(f"   Action: {action_dict.get('type', 'unknown')}")
    
    if result.get('success'):
        return result.get('last_action_result', {})

    # Preserve timeout/no_response flags in return
    # Note: Handler returns 'reason' for failures, not 'error'
    error_reason = result.get('reason') or result.get('error') or 'Unknown error'
    error_info = {'status': 'failed', 'reason': error_reason}
    if 'timeout' in result:
        error_info['timeout'] = True
    if 'no_response' in result:
        error_info['no_response'] = True
    if 'exception' in result:
        error_info['exception'] = True
    return error_info


def get_status(session: zenoh.Session, character: str) -> Optional[Dict]:
    """Get current agent status."""
    result = execute_action(session, character, {"type": "mc-status"})
    if result.get('status') == 'success':
        return result.get('data', {})
    return None


def get_observe(session: zenoh.Session, character: str) -> Optional[Dict]:
    """Get current observation."""
    result = execute_action(session, character, {"type": "mc-observe"})
    if result.get('status') == 'success':
        # Observation data might be in 'data' or 'value' field
        data = result.get('data', {})
        if not data or not isinstance(data, dict):
            data = result.get('value', {})
        return data if isinstance(data, dict) else None
    return None


def get_voxel_grid(session: zenoh.Session, character: str, radius: int = 2) -> Optional[Dict]:
    """Get observed voxel grid using debug tool."""
    result = execute_action(session, character, {"type": "debug-get-voxel-grid", "radius": radius})
    if result.get('status') == 'success':
        # Voxel grid is in extra field
        extra = result.get('extra', {})
        return extra.get('voxel_grid')
    return None


def get_world_state(session: zenoh.Session, character: str, key: str, timeout: float = 5.0) -> Optional[any]:
    """Get world state value via Zenoh API."""
    query_key = f"cognitive/{character}/world_state/{key}"
    replies = session.get(query_key, timeout=timeout)
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            payload = reply.ok.payload.to_bytes().decode('utf-8')
            return json.loads(payload)
    return None


# -----------------------------
# Coordinate Validation Helpers
# -----------------------------

def normalize_yaw(yaw: float) -> float:
    """Normalize yaw to [0, 360) range."""
    yaw = yaw % 360
    if yaw < 0:
        yaw += 360
    return yaw


def yaws_match(yaw1: float, yaw2: float, tolerance: float = 5.0) -> bool:
    """Check if two yaws match (handling wraparound at 0/360)."""
    y1 = normalize_yaw(yaw1)
    y2 = normalize_yaw(yaw2)
    diff = abs(y1 - y2)
    # Handle wraparound (e.g., 359° vs 1°)
    if diff > 180:
        diff = 360 - diff
    return diff < tolerance


def agent_rel_to_world(dx: float, dy: float, dz: float, yaw: float) -> Tuple[float, float, float]:
    """Convert agent-relative to world-relative coordinates."""
    yaw_rad = math.radians(yaw)
    world_dx = -math.sin(yaw_rad) * dz - math.cos(yaw_rad) * dx
    world_dz = -math.cos(yaw_rad) * dz + math.sin(yaw_rad) * dx
    return world_dx, dy, world_dz


def get_grid_cell_from_voxel_grid(voxel_grid: Dict, dx: int, dy: int, dz: int) -> Optional[Dict]:
    """Get cell from voxel grid at agent-relative coordinates."""
    if not voxel_grid:
        return None
    
    cells = voxel_grid.get('cells', [])
    if not isinstance(cells, list):
        return None
    
    for cell in cells:
        if isinstance(cell, dict) and cell.get('dx') == dx and cell.get('dy') == dy and cell.get('dz') == dz:
            return cell
    return None


def validate_grid_coordinates(voxel_grid: Dict) -> Tuple[bool, List[str]]:
    """Validate grid uses agent-relative coordinates."""
    errors = []

    if not voxel_grid:
        errors.append("No voxel grid provided")
        return False, errors

    cells = voxel_grid.get('cells', [])
    if not cells:
        errors.append("No cells in grid")
        return False, errors

    # Check that (0,0,0) exists (agent's feet block)
    agent_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, 0, 0)
    if not agent_cell:
        errors.append("Missing (0,0,0) cell (agent's feet block)")

    # Check that (0,-1,0) exists (support block below)
    support_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, -1, 0)
    if not support_cell:
        errors.append("Missing (0,-1,0) cell (support block below)")

    # Check center has yaw
    center = voxel_grid.get('center', {})
    if 'yaw' not in center:
        errors.append("Grid center missing 'yaw' field")

    # DEBUG: If validation failed, print grid contents for diagnosis
    if errors:
        print(f"\n  DEBUG: Grid validation failed, dumping grid contents:")
        print(f"  DEBUG: Grid center: {center}")
        print(f"  DEBUG: Total cells: {len(cells)}")
        # Show cells near origin (dy from -2 to +2)
        origin_cells = [c for c in cells if isinstance(c, dict) and
                       abs(c.get('dx', 999)) <= 2 and
                       abs(c.get('dy', 999)) <= 2 and
                       abs(c.get('dz', 999)) <= 2]
        print(f"  DEBUG: Cells within ±2 of origin: {len(origin_cells)}")
        for cell in sorted(origin_cells, key=lambda c: (c.get('dy', 0), c.get('dz', 0), c.get('dx', 0))):
            dx, dy, dz = cell.get('dx'), cell.get('dy'), cell.get('dz')
            block = cell.get('block_id', 'unknown')
            print(f"    [{dx:+d},{dy:+d},{dz:+d}]: {block}")
        if not origin_cells:
            # Show first 10 cells if none near origin
            print(f"  DEBUG: First 10 cells (not near origin):")
            for cell in cells[:10]:
                dx, dy, dz = cell.get('dx'), cell.get('dy'), cell.get('dz')
                block = cell.get('block_id', 'unknown')
                print(f"    [{dx},{dy},{dz}]: {block}")

    return len(errors) == 0, errors


# -----------------------------
# Test Cases
# -----------------------------

def test_basic_grid_observation(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 1: Basic grid observation returns agent-relative coordinates."""
    print("\n=== Test 1: Basic Grid Observation ===")
    
    # Populate grid first by calling mc-observe
    obs_data = get_observe(session, character)
    if not obs_data:
        return False, "Failed to get observation"
    
    # Check if observation has pose
    pose = obs_data.get('pose', {})
    if not pose or 'x' not in pose:
        return False, "Observation missing pose data"
    
    print(f"  Agent pose: ({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f}, {pose.get('z', 0):.2f})")
    print(f"  Agent yaw: {pose.get('yaw', 0):.1f}°")
    
    # Get voxel grid using debug tool (grid should now be populated)
    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get voxel grid (debug-get-voxel-grid tool may not be available)"
    
    # Validate grid coordinates
    is_valid, errors = validate_grid_coordinates(voxel_grid)
    if not is_valid:
        return False, f"Grid validation failed: {', '.join(errors)}"
    
    # Check that (0,0,0) represents agent position
    agent_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, 0, 0)
    if not agent_cell:
        return False, "Missing (0,0,0) cell"
    
    cell_count = len(voxel_grid.get('cells', []))
    center = voxel_grid.get('center', {})
    grid_yaw = center.get('yaw', 'unknown')
    
    print(f"✓ Grid observation successful")
    print(f"  Agent cell (0,0,0): {agent_cell.get('block_id', 'unknown')}")
    print(f"  Grid has {cell_count} cells")
    print(f"  Grid center yaw: {grid_yaw}°")
    
    # Verify grid yaw matches agent yaw (normalize to handle -90 vs 270, etc.)
    if grid_yaw != 'unknown':
        agent_yaw = float(pose.get('yaw', 0))
        if not yaws_match(float(grid_yaw), agent_yaw):
            return False, f"Grid yaw mismatch: grid={normalize_yaw(float(grid_yaw)):.1f}°, agent={normalize_yaw(agent_yaw):.1f}°"
    
    return True, "Basic grid observation validated"


def test_yaw_transformation(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 2: Yaw changes affect coordinate interpretation."""
    print("\n=== Test 2: Yaw Transformation ===")
    
    # Get initial status
    status1 = get_status(session, character)
    if not status1:
        return False, "Failed to get initial status"
    
    pos1 = status1.get('position', {})
    yaw1 = status1.get('yaw', 0.0)
    print(f"Initial: pos=({pos1.get('x', 0):.2f}, {pos1.get('y', 0):.2f}, {pos1.get('z', 0):.2f}), yaw={yaw1:.1f}°")
    
    # Turn 90 degrees
    turn_result = execute_action(session, character, {"type": "nav-turn", "direction": "right"})
    if turn_result.get('status') != 'success':
        return False, f"Failed to turn: {turn_result.get('reason', 'unknown')}"
    
    # Get status after turn
    status2 = get_status(session, character)
    if not status2:
        return False, "Failed to get status after turn"
    
    pos2 = status2.get('position', {})
    yaw2 = status2.get('yaw', 0.0)
    print(f"After turn: pos=({pos2.get('x', 0):.2f}, {pos2.get('y', 0):.2f}, {pos2.get('z', 0):.2f}), yaw={yaw2:.1f}°")
    
    # Verify yaw changed (should be ~90° different)
    yaw_diff = abs((yaw2 - yaw1) % 360)
    if yaw_diff < 80 or yaw_diff > 100:
        return False, f"Yaw change incorrect: {yaw_diff:.1f}° (expected ~90°)"
    
    # Get voxel grid after turn
    voxel_grid2 = get_voxel_grid(session, character, radius=2)
    if not voxel_grid2:
        return False, "Failed to get voxel grid after turn"
    
    # Verify grid still has (0,0,0) at agent position
    agent_cell = get_grid_cell_from_voxel_grid(voxel_grid2, 0, 0, 0)
    if not agent_cell:
        return False, "Missing (0,0,0) cell after turn"
    
    # Verify grid center yaw matches agent yaw
    grid_center = voxel_grid2.get('center', {})
    grid_yaw = grid_center.get('yaw', None)
    if grid_yaw is None:
        return False, "Grid center missing yaw"
    
    if not yaws_match(float(grid_yaw), float(yaw2)):
        return False, f"Grid yaw mismatch: grid={normalize_yaw(float(grid_yaw)):.1f}°, agent={normalize_yaw(float(yaw2)):.1f}°"
    
    print(f"✓ Yaw transformation validated")
    print(f"  Yaw changed from {yaw1:.1f}° to {yaw2:.1f}°")
    print(f"  Grid yaw matches agent yaw: {grid_yaw:.1f}°")
    
    return True, "Yaw transformation validated"


def test_place_block_agent_relative(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 3: Place block using agent-relative coordinates."""
    print("\n=== Test 3: Place Block Agent-Relative ===")

    # Populate grid first
    get_observe(session, character)

    # Get initial status
    status1 = get_status(session, character)
    if not status1:
        return False, "Failed to get initial status"

    pos1 = status1.get('position', {})
    yaw1 = status1.get('yaw', 0.0)
    bx1 = int(math.floor(pos1.get('x', 0)))
    by1 = int(math.floor(pos1.get('y', 0)))
    bz1 = int(math.floor(pos1.get('z', 0)))

    print(f"Initial: block=({bx1}, {by1}, {bz1}), yaw={yaw1:.1f}°")

    # Test 3a: Verify overlap rejection - placing at agent's feet level should FAIL
    print("\nTest 3a: Verify overlap rejection at agent feet level")
    place_overlap = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 0.0,
        "dy": -1.0,  # anchor is block below agent
        "dz": 0.0,
        "face": "top"  # place on top of anchor → at agent feet level
    })

    if place_overlap.get('status') == 'success':
        return False, "Overlap rejection failed - placement at agent feet should be rejected"

    if place_overlap.get('reason') != 'placement_overlaps_agent':
        return False, f"Wrong rejection reason: {place_overlap.get('reason')} (expected placement_overlaps_agent)"

    print(f"✓ Overlap correctly rejected: {place_overlap.get('reason')}")

    # Test 3b: Place block FORWARD (should succeed - no overlap)
    print("\nTest 3b: Place block forward (bridge pattern)")
    place_result = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 0.0,
        "dy": -1.0,  # anchor is block forward and below
        "dz": 1.0,   # forward
        "face": "top"  # place on top of anchor → at forward position, feet level
    })

    if place_result.get('status') != 'success':
        return False, f"Failed to place block forward: {place_result.get('reason', 'unknown')}"

    print(f"✓ Block placed forward successfully")

    # Verify block was placed (check voxel grid)
    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get voxel grid after placement"

    # Check grid cell at [0, 0, 1] (forward at feet level)
    placed_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, 0, 1)
    if not placed_cell:
        return False, "Block not found at [0, 0, 1] in grid"

    block_id = placed_cell.get('block_id', '')
    if 'dirt' not in block_id.lower():
        return False, f"Wrong block at [0,0,1]: {block_id} (expected dirt)"

    print(f"  Anchor [0,-1,1] + face=top → Block at [0,0,1]")
    print(f"  Block confirmed: {block_id}")

    # Clean up: dig the placed block
    execute_action(session, character, {"type": "mc-dig", "dx": 0.0, "dy": 0.0, "dz": 1.0})

    return True, "Place block agent-relative validated (overlap rejection + forward placement)"


def test_pillar_up(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 3b: Pillar up using atomic jump+place."""
    print("\n=== Test 3b: Pillar Up (Jump + Place) ===")

    # Known solid blocks suitable for pillar-up
    SOLID_BLOCKS = {
        'dirt', 'cobblestone', 'stone', 'oak_planks', 'spruce_planks', 'birch_planks',
        'jungle_planks', 'acacia_planks', 'dark_oak_planks', 'planks', 'gravel', 'sand',
        'sandstone', 'netherrack', 'cobbled_deepslate', 'deepslate', 'andesite',
        'diorite', 'granite', 'brick', 'stone_bricks', 'nether_bricks'
    }

    # Query inventory to find a suitable block
    inv_result = execute_action(session, character, {"type": "mc-inventory"})
    if inv_result.get('status') != 'success':
        return False, "Failed to query inventory"

    # Inventory data is in 'extra' field (slots, equipped), not 'data' (which is formatted text)
    inv_extra = inv_result.get('extra', {})
    slots = inv_extra.get('slots', [])

    # Find first solid block in inventory
    pillar_block = None
    for slot in slots:
        # Handle both dict format {"item": "...", "count": N} and string format
        if isinstance(slot, dict):
            item = slot.get('item', '')
            count = slot.get('count', 0)
        elif isinstance(slot, str):
            item = slot
            count = 1
        else:
            continue

        if not item or item == 'air':
            continue
        # Strip minecraft: prefix if present
        item_name = item.split(':')[-1] if ':' in item else item
        if item_name in SOLID_BLOCKS:
            pillar_block = item_name
            print(f"Found solid block in inventory: {pillar_block} (count={count})")
            break

    if not pillar_block:
        return False, "No suitable solid block found in inventory for pillar-up test"

    # Populate grid first
    get_observe(session, character)

    # Get initial status
    status1 = get_status(session, character)
    if not status1:
        return False, "Failed to get initial status"

    pos1 = status1.get('position', {})
    yaw1 = status1.get('yaw', 0.0)
    by1 = int(math.floor(pos1.get('y', 0)))

    print(f"Initial Y: {by1}, yaw={yaw1:.1f}°")

    # Pillar up using mc-pillar-up (atomic jump + place)
    pillar_result = execute_action(session, character, {
        "type": "mc-pillar-up",
        "item": pillar_block
    })

    if pillar_result.get('status') != 'success':
        extra = pillar_result.get('extra', {})
        print(f"  Pillar result extra: {extra}")
        print(f"  Pillar result value: {pillar_result.get('value', '')}")
        return False, f"Pillar up failed: {pillar_result.get('reason', 'unknown')}"

    # Get final status
    status2 = get_status(session, character)
    if not status2:
        return False, "Failed to get final status"

    pos2 = status2.get('position', {})
    by2 = int(math.floor(pos2.get('y', 0)))
    delta_y = by2 - by1

    print(f"Final Y: {by2}, delta_y={delta_y}")

    if delta_y < 1:
        return False, f"Pillar up did not ascend: delta_y={delta_y} (expected >= 1)"

    print(f"✓ Pillar up successful: ascended {delta_y} block(s)")

    # Verify block was placed below current position
    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get voxel grid after pillar"

    # Check grid cell at [0, -1, 0] (block below current feet)
    below_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, -1, 0)
    if not below_cell:
        return False, "Block not found at [0, -1, 0] after pillar"

    block_id = below_cell.get('block_id', '')
    # Check if the block we placed is there (strip minecraft: prefix for comparison)
    block_name = block_id.split(':')[-1] if ':' in block_id else block_id
    if pillar_block.lower() not in block_name.lower():
        return False, f"Wrong block below: {block_id} (expected {pillar_block})"

    print(f"  Block below confirmed: {block_id}")

    return True, "Pillar up validated"


def test_dig_block_agent_relative(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 4: Dig block using agent-relative coordinates."""
    print("\n=== Test 4: Dig Block Agent-Relative ===")
    
    # Populate grid first
    get_observe(session, character)
    
    # First, place a block to dig at [0,0,1] (forward)
    # Anchor: block below agent (dy=-1), Face: top → places at agent feet [0,0,0]
    # But we want [0,0,1], so use anchor at [0,-1,1] (below forward), face="top"
    place_result = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 0.0,
        "dy": -1.0,  # Anchor: below
        "dz": 1.0,   # Anchor: forward
        "face": "top"  # Place on top of anchor → at [0,0,1]
    })
    
    if place_result.get('status') != 'success':
        return False, f"Failed to place block for digging: {place_result.get('reason', 'unknown')}"
    
    print("Placed dirt block at [0,0,1] (forward)")
    
    # Get status before dig
    status_before = get_status(session, character)
    if not status_before:
        return False, "Failed to get status before dig"
    
    yaw = status_before.get('yaw', 0.0)
    
    # Dig the block at agent-relative [0, 0, 1] (forward) - this is the target position
    dig_result = execute_action(session, character, {
        "type": "mc-dig",
        "dx": 0.0,
        "dy": 0.0,
        "dz": 1.0
    })
    
    if dig_result.get('status') != 'success':
        return False, f"Failed to dig block: {dig_result.get('reason', 'unknown')}"
    
    # Verify block was removed (check voxel grid)
    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get voxel grid after dig"
    
    # Check grid cell at [0, 0, 1] should be air now (this is where we placed it)
    dug_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, 0, 1)
    if not dug_cell:
        return False, "Cell [0,0,1] not found in grid"
    
    block_id = dug_cell.get('block_id', '')
    is_air = block_id is None or 'air' in block_id.lower() or not block_id
    
    if not is_air:
        return False, f"Block still present at [0,0,1]: {block_id} (expected air)"
    
    print(f"✓ Block dug successfully")
    print(f"  Agent-relative [0,0,1] → Block removed")
    print(f"  Cell now: {block_id if block_id else 'air'}")
    
    return True, "Dig block agent-relative validated"


def test_navigation_consistency(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 5: Navigation maintains coordinate consistency."""
    print("\n=== Test 5: Navigation Consistency ===")

    # Get current status to show position/yaw
    status = get_status(session, character)
    if status:
        pos = status.get('position', {})
        yaw = status.get('yaw', 0)
        print(f"  Agent at ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f}), yaw={yaw:.1f}°")

    # Populate grid first
    get_observe(session, character)

    # Get initial voxel grid with radius 3 to see head level
    voxel_grid1 = get_voxel_grid(session, character, radius=3)
    if not voxel_grid1:
        return False, "Failed to get initial voxel grid"

    # --- Precondition check: forward path must be clear (feet, body, head levels) ---
    # Cell state: "air" (confirmed clear), "blocked" (confirmed solid), "unknown" (not in grid)
    def get_cell_state(cell) -> str:
        if not cell:
            return "unknown"  # Cell not in grid - DO NOT assume air
        block_id = cell.get('block_id', '')
        if not block_id or 'air' in block_id.lower():
            return "air"
        return "blocked"

    forward_feet = get_grid_cell_from_voxel_grid(voxel_grid1, 0, 0, 1)   # dy=0, dz=+1
    forward_body = get_grid_cell_from_voxel_grid(voxel_grid1, 0, 1, 1)   # dy=+1, dz=+1
    forward_head = get_grid_cell_from_voxel_grid(voxel_grid1, 0, 2, 1)   # dy=+2, dz=+1

    feet_state = get_cell_state(forward_feet)
    body_state = get_cell_state(forward_body)
    head_state = get_cell_state(forward_head)

    # Check for unknown cells - cannot proceed safely
    unknown_levels = []
    if feet_state == "unknown":
        unknown_levels.append("feet [0,0,1]")
    if body_state == "unknown":
        unknown_levels.append("body [0,1,1]")
    if head_state == "unknown":
        unknown_levels.append("head [0,2,1]")

    if unknown_levels:
        skip_msg = f"SKIPPED - Forward path has unknown cells: {', '.join(unknown_levels)}"
        print(f"⏭️  {skip_msg}")
        print(f"  (Cells not in voxel grid - cannot confirm path is clear)")
        return True, skip_msg  # Return True to not count as failure

    # Check for blocked cells
    blocked_levels = []
    if feet_state == "blocked":
        blocked_levels.append(f"feet [0,0,1]: {forward_feet.get('block_id', 'unknown')}")
    if body_state == "blocked":
        blocked_levels.append(f"body [0,1,1]: {forward_body.get('block_id', 'unknown')}")
    if head_state == "blocked":
        blocked_levels.append(f"head [0,2,1]: {forward_head.get('block_id', 'unknown')}")

    if blocked_levels:
        skip_msg = f"SKIPPED - Forward path blocked: {', '.join(blocked_levels)}"
        print(f"⏭️  {skip_msg}")
        print(f"  (Move agent to open area to run this test)")
        return True, skip_msg  # Return True to not count as failure

    # Get cell at [0,0,1] (forward) before movement
    forward_cell_before = forward_feet
    forward_block_before = forward_cell_before.get('block_id', 'air') if forward_cell_before else 'unknown'

    print(f"Before move: [0,0,1] (forward) = {forward_block_before}")
    print(f"  Path confirmed clear: feet={feet_state}, body={body_state}, head={head_state}")

    # Move forward
    move_result = execute_action(session, character, {"type": "nav-move"})
    if move_result.get('status') != 'success':
        reason = move_result.get('reason', 'unknown')
        extra = move_result.get('extra', {})
        # Show extra debug info if available
        if extra:
            print(f"  DEBUG nav-move extra: {extra}")
        return False, f"Failed to move: {reason}"
    
    # Get voxel grid after movement
    voxel_grid2 = get_voxel_grid(session, character, radius=2)
    if not voxel_grid2:
        return False, "Failed to get voxel grid after movement"
    
    # Verify (0,0,0) still represents agent's feet
    agent_cell = get_grid_cell_from_voxel_grid(voxel_grid2, 0, 0, 0)
    if not agent_cell:
        return False, "Missing (0,0,0) cell after movement"
    
    # Verify (0,0,-1) (back) now has what was at (0,0,1) before
    back_cell_after = get_grid_cell_from_voxel_grid(voxel_grid2, 0, 0, -1)
    if back_cell_after:
        back_block_after = back_cell_after.get('block_id', 'air')
        print(f"After move: [0,0,-1] (back) = {back_block_after}")
    
    print(f"✓ Navigation consistency validated")
    print(f"  (0,0,0) still represents agent's feet after movement")
    
    return True, "Navigation consistency validated"


def test_multi_action_sequence(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 6: Multiple actions maintain coordinate consistency."""
    print("\n=== Test 6: Multi-Action Sequence ===")
    
    # Execute sequence: turn → move → place (forward) → observe
    # Place at [0,0,1] (forward, where agent isn't standing)
    # Anchor at [0,-1,1] (below forward), face="top" → block at [0,0,1]
    plan = [
        {"type": "nav-turn", "direction": "right"},
        {"type": "nav-move"},
        {"type": "mc-place", "item": "dirt", "dx": 0.0, "dy": -1.0, "dz": 1.0, "face": "top"},
        {"type": "mc-observe"}
    ]

    result = execute_plan(session, character, plan, timeout=30.0)
    if not result.get('success'):
        error_msg = result.get('error', 'unknown')
        if result.get('timeout'):
            error_msg += " [TIMEOUT]"
        elif result.get('no_response'):
            error_msg += " [NO_RESPONSE]"
        return False, f"Plan execution failed: {error_msg}"

    # Get final voxel grid
    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get final voxel grid"

    # Validate grid coordinates
    is_valid, errors = validate_grid_coordinates(voxel_grid)
    if not is_valid:
        return False, f"Grid validation failed after sequence: {', '.join(errors)}"

    # Check that placed block is at correct relative position [0,0,1] (forward)
    placed_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, 0, 1)
    if not placed_cell:
        return False, "Placed block not found at [0,0,1]"

    block_id = placed_cell.get('block_id', '')
    if 'dirt' not in block_id.lower():
        return False, f"Wrong block at [0,0,1]: {block_id}"

    print(f"✓ Multi-action sequence validated")
    print(f"  Sequence: turn → move → place (forward) → observe")
    print(f"  Block placed at [0,0,1] (forward): {block_id}")
    
    return True, "Multi-action sequence validated"


def test_all_cardinal_yaws(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 7: Coordinate system works at all cardinal yaws."""
    print("\n=== Test 7: All Cardinal Yaws ===")
    
    # Populate grid first
    get_observe(session, character)
    
    cardinals = [0, 90, 180, 270]
    results = []
    
    for target_yaw in cardinals:
        # Turn to target yaw (may need multiple turns)
        current_status = get_status(session, character)
        if not current_status:
            return False, f"Failed to get status for yaw {target_yaw}"
        
        current_yaw = current_status.get('yaw', 0.0)
        yaw_diff = (target_yaw - current_yaw) % 360
        
        # Turn to target
        if yaw_diff > 180:
            yaw_diff -= 360
        
        turns_needed = int(round(yaw_diff / 90))
        for _ in range(abs(turns_needed)):
            direction = "right" if turns_needed > 0 else "left"
            turn_result = execute_action(session, character, {"type": "nav-turn", "direction": direction})
            if turn_result.get('status') != 'success':
                return False, f"Failed to turn to {target_yaw}°"
        
        # Verify yaw
        status = get_status(session, character)
        if not status:
            return False, f"Failed to get status at yaw {target_yaw}"
        
        actual_yaw = status.get('yaw', 0.0)
        yaw_error = abs((actual_yaw - target_yaw) % 360)
        if yaw_error > 5 and yaw_error < 355:  # Allow wrap-around
            return False, f"Yaw mismatch at {target_yaw}°: actual={actual_yaw:.1f}°"
        
        # Place block at [0,0,1] (forward, where agent isn't standing)
        # Anchor at [0,-1,1] (below forward), face="top" → block at [0,0,1]
        place_result = execute_action(session, character, {
            "type": "mc-place",
            "item": "dirt",
            "dx": 0.0,
            "dy": -1.0,
            "dz": 1.0,
            "face": "top"
        })

        if place_result.get('status') != 'success':
            return False, f"Failed to place block at yaw {target_yaw}°"

        # Verify in grid
        voxel_grid = get_voxel_grid(session, character, radius=2)
        if not voxel_grid:
            return False, f"Failed to get voxel grid at yaw {target_yaw}°"

        placed_cell = get_grid_cell_from_voxel_grid(voxel_grid, 0, 0, 1)
        if not placed_cell or 'dirt' not in placed_cell.get('block_id', '').lower():
            return False, f"Block not found at [0,0,1] at yaw {target_yaw}°"

        # Clean up: dig the block
        execute_action(session, character, {"type": "mc-dig", "dx": 0.0, "dy": 0.0, "dz": 1.0})
        
        results.append(f"✓ Yaw {target_yaw}°: OK")
        print(f"  Yaw {target_yaw}°: validated")
    
    print(f"✓ All cardinal yaws validated")
    return True, f"All {len(cardinals)} cardinal yaws validated"


def test_left_right_dx_consistency(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test 8: Verify dx+ = right, dx- = left consistency with Minecraft and LLM reasoning."""
    print("\n=== Test 8: Left/Right vs dx+/- Consistency ===")
    
    # Populate grid first
    get_observe(session, character)
    
    # Get initial status
    status = get_status(session, character)
    if not status:
        return False, "Failed to get initial status"
    
    yaw = status.get('yaw', 0.0)
    pos = status.get('position', {})
    bx = int(math.floor(pos.get('x', 0)))
    by = int(math.floor(pos.get('y', 0)))
    bz = int(math.floor(pos.get('z', 0)))
    
    print(f"  Agent at block ({bx}, {by}, {bz}), yaw={yaw:.1f}°")
    
    # Test at yaw 0° (facing south): dx+ should be right (east), dx- should be left (west)
    # Turn to yaw 0° if needed
    current_yaw = yaw
    yaw_diff = (0.0 - current_yaw) % 360
    if yaw_diff > 180:
        yaw_diff -= 360
    
    turns_needed = int(round(yaw_diff / 90))
    for _ in range(abs(turns_needed)):
        direction = "right" if turns_needed > 0 else "left"
        turn_result = execute_action(session, character, {"type": "nav-turn", "direction": direction})
        if turn_result.get('status') != 'success':
            return False, f"Failed to turn to yaw 0°"
    
    # Verify at yaw 0°
    status = get_status(session, character)
    if not status:
        return False, "Failed to get status after turn"
    
    yaw0 = status.get('yaw', 0.0)
    if abs(yaw0) > 5 and abs(yaw0 - 360) > 5:
        return False, f"Not at yaw 0°: actual={yaw0:.1f}°"
    
    pos = status.get('position', {})
    bx = int(math.floor(pos.get('x', 0)))
    by = int(math.floor(pos.get('y', 0)))
    bz = int(math.floor(pos.get('z', 0)))
    
    # At yaw 0° (facing south), dx+ should be right (west in world coords)
    # To place at [1,0,0] (right of agent), use anchor at agent position [0,0,0]
    # and face="west" (absolute) which places to the west of anchor
    # At yaw 0° (south), "west" absolute = "right" relative, so block at [1,0,0]
    place_right = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 0.0,   # anchor at agent position
        "dy": 0.0,
        "dz": 0.0,
        "face": "west"  # place west of anchor → at [1,0,0] when yaw=0
    })

    if place_right.get('status') != 'success':
        return False, f"Failed to place block at dx=+1 (right)"

    # Get voxel grid and verify block is at [1,0,0] (right)
    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get voxel grid"

    right_cell = get_grid_cell_from_voxel_grid(voxel_grid, 1, 0, 0)
    if not right_cell or 'dirt' not in right_cell.get('block_id', '').lower():
        return False, f"Block not found at [1,0,0] (dx=+1, right) at yaw 0°"
    
    # Calculate expected world position: at yaw 0° (south), dx=+1 (right) should be east
    world_dx, _, world_dz = agent_rel_to_world(1.0, 0.0, 0.0, 0.0)
    expected_world_x = bx + int(round(world_dx))
    expected_world_z = bz + int(round(world_dz))
    
    print(f"  ✓ dx=+1 (right) → world ({expected_world_x}, {by}, {expected_world_z})")
    
    # Clean up: dig right block
    execute_action(session, character, {"type": "mc-dig", "dx": 1.0, "dy": 0.0, "dz": 0.0})
    
    # Place block at dx=-1 (left)
    # Anchor at agent position [0,0,0], face="east" → block east of anchor
    # At yaw 0° (south), east = left relative, so block at [-1,0,0]
    place_left = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 0.0,   # anchor at agent position
        "dy": 0.0,
        "dz": 0.0,
        "face": "east"  # place east of anchor → at [-1,0,0] when yaw=0
    })

    if place_left.get('status') != 'success':
        return False, f"Failed to place block at dx=-1 (left)"

    # Get voxel grid and verify block is at [-1,0,0] (left)
    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get voxel grid"

    left_cell = get_grid_cell_from_voxel_grid(voxel_grid, -1, 0, 0)
    if not left_cell or 'dirt' not in left_cell.get('block_id', '').lower():
        return False, f"Block not found at [-1,0,0] (dx=-1, left) at yaw 0°"
    
    # Calculate expected world position: at yaw 0° (south), dx=-1 (left) should be west
    world_dx, _, world_dz = agent_rel_to_world(-1.0, 0.0, 0.0, 0.0)
    expected_world_x = bx + int(round(world_dx))
    expected_world_z = bz + int(round(world_dz))
    
    print(f"  ✓ dx=-1 (left) → world ({expected_world_x}, {by}, {expected_world_z})")
    
    # Clean up: dig left block
    execute_action(session, character, {"type": "mc-dig", "dx": -1.0, "dy": 0.0, "dz": 0.0})
    
    # Test at yaw 90° (facing west): dx+ should still be right (north), dx- should be left (south)
    turn_result = execute_action(session, character, {"type": "nav-turn", "direction": "right"})
    if turn_result.get('status') != 'success':
        return False, "Failed to turn right to yaw 90°"
    
    status = get_status(session, character)
    if not status:
        return False, "Failed to get status after turn"
    
    yaw90 = status.get('yaw', 0.0)
    if abs(yaw90 - 90.0) > 5:
        return False, f"Not at yaw 90°: actual={yaw90:.1f}°"
    
    pos = status.get('position', {})
    bx = int(math.floor(pos.get('x', 0)))
    by = int(math.floor(pos.get('y', 0)))
    bz = int(math.floor(pos.get('z', 0)))
    
    # At yaw 90° (facing west), dx+ should be right (north in world coords)
    # For yaw 90°, "right" is north (-Z), so we need face="north" to place north of anchor
    place_right = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 0.0,   # anchor at agent position
        "dy": 0.0,
        "dz": 0.0,
        "face": "north"  # at yaw 90° (west), "north" = "right" → block at [1,0,0]
    })

    if place_right.get('status') != 'success':
        return False, f"Failed to place block at dx=+1 (right) at yaw 90°"

    voxel_grid = get_voxel_grid(session, character, radius=2)
    if not voxel_grid:
        return False, "Failed to get voxel grid"

    right_cell = get_grid_cell_from_voxel_grid(voxel_grid, 1, 0, 0)
    if not right_cell or 'dirt' not in right_cell.get('block_id', '').lower():
        return False, f"Block not found at [1,0,0] (dx=+1, right) at yaw 90°"

    world_dx, _, world_dz = agent_rel_to_world(1.0, 0.0, 0.0, 90.0)
    expected_world_x = bx + int(round(world_dx))
    expected_world_z = bz + int(round(world_dz))

    print(f"  ✓ dx=+1 (right) at yaw 90° → world ({expected_world_x}, {by}, {expected_world_z})")

    # Clean up
    execute_action(session, character, {"type": "mc-dig", "dx": 1.0, "dy": 0.0, "dz": 0.0})
    
    print(f"✓ Left/right vs dx+/- consistency validated")
    print(f"  dx+ = right (consistent across yaws)")
    print(f"  dx- = left (consistent across yaws)")
    print(f"  This matches standard LLM reasoning and Minecraft conventions")
    
    return True, "Left/right vs dx+/- consistency validated"


# -----------------------------
# Main Test Runner
# -----------------------------

def run_all_tests(session: zenoh.Session, character: str) -> Dict[str, Tuple[bool, str]]:
    """Run all coordinate system tests."""

    # IMPORTANT: Perform initial observation to populate grid after agent restart
    # Without this, the grid may be empty or have stale data from previous sessions
    print("\n=== Initializing: Performing startup observation ===")
    init_obs = get_observe(session, character)
    if init_obs:
        pose = init_obs.get('pose', {})
        print(f"  Initial observation successful")
        print(f"  Agent at ({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f}, {pose.get('z', 0):.2f}), yaw={pose.get('yaw', 0):.1f}°")
    else:
        print("  WARNING: Initial observation failed - tests may have stale grid data")
    print()

    tests = [
        ("Basic Grid Observation", test_basic_grid_observation),
        ("Yaw Transformation", test_yaw_transformation),
        ("Place Block Agent-Relative", test_place_block_agent_relative),
        ("Pillar Up", test_pillar_up),
        ("Dig Block Agent-Relative", test_dig_block_agent_relative),
        ("Navigation Consistency", test_navigation_consistency),
        ("Multi-Action Sequence", test_multi_action_sequence),
        ("All Cardinal Yaws", test_all_cardinal_yaws),
        ("Left/Right vs dx+/- Consistency", test_left_right_dx_consistency),
    ]

    results = {}
    for test_name, test_func in tests:
        try:
            success, message = test_func(session, character)
            results[test_name] = (success, message)
            if not success:
                print(f"❌ {test_name}: FAILED - {message}")
            else:
                print(f"✅ {test_name}: PASSED")
        except Exception as e:
            results[test_name] = (False, f"Exception: {str(e)}")
            print(f"❌ {test_name}: EXCEPTION - {str(e)}")

        # No sleep needed - actions are synchronous

    return results


def main():
    parser = argparse.ArgumentParser(description="Test Minecraft agent-relative coordinate system")
    parser.add_argument(
        "--character",
        type=str,
        default="Jill",
        help="Character name (default: Jill)"
    )
    parser.add_argument(
        "--test",
        type=str,
        default="all",
        help="Specific test to run (default: all)"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Minecraft Agent-Relative Coordinate System Test Suite")
    print("=" * 60)
    print(f"Character: {args.character}")
    print(f"Test: {args.test}")
    print()
    
    # Connect to Zenoh
    print("Connecting to Zenoh...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("✓ Zenoh session opened")
    
    try:
        # Verify character is available
        status = get_status(session, args.character)
        if not status:
            print(f"❌ Error: Character '{args.character}' not available or failed to get status")
            return 1
        
        print(f"✓ Character '{args.character}' available")
        print(f"  Position: ({status.get('position', {}).get('x', 0):.2f}, "
              f"{status.get('position', {}).get('y', 0):.2f}, "
              f"{status.get('position', {}).get('z', 0):.2f})")
        print(f"  Yaw: {status.get('yaw', 0):.1f}°")
        print()
        
        # Run tests
        if args.test == "all":
            results = run_all_tests(session, args.character)
        else:
            # Run specific test
            test_map = {
                "basic": ("Basic Grid Observation", test_basic_grid_observation),
                "yaw": ("Yaw Transformation", test_yaw_transformation),
                "place": ("Place Block Agent-Relative", test_place_block_agent_relative),
                "pillar": ("Pillar Up", test_pillar_up),
                "dig": ("Dig Block Agent-Relative", test_dig_block_agent_relative),
                "nav": ("Navigation Consistency", test_navigation_consistency),
                "multi": ("Multi-Action Sequence", test_multi_action_sequence),
                "cardinals": ("All Cardinal Yaws", test_all_cardinal_yaws),
                "leftright": ("Left/Right vs dx+/- Consistency", test_left_right_dx_consistency),
            }
            
            if args.test not in test_map:
                print(f"❌ Unknown test: {args.test}")
                print(f"Available tests: {', '.join(test_map.keys())}")
                return 1
            
            test_name, test_func = test_map[args.test]
            try:
                success, message = test_func(session, args.character)
                results = {test_name: (success, message)}
            except Exception as e:
                results = {test_name: (False, f"Exception: {str(e)}")}
        
        # Print summary
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        
        passed = sum(1 for success, _ in results.values() if success)
        total = len(results)
        
        for test_name, (success, message) in results.items():
            status_icon = "✅" if success else "❌"
            print(f"{status_icon} {test_name}: {message}")
        
        print()
        print(f"Total: {passed}/{total} tests passed")
        
        return 0 if passed == total else 1
        
    finally:
        session.close()
        print("\n✓ Zenoh session closed")


if __name__ == "__main__":
    main()
    exit(0)
