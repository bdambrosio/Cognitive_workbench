#!/usr/bin/env python3
"""
Diagnostic script to identify the root cause of dx coordinate mismatch.

Tests:
1. Check all adjacent cells after placing at dx=+1
2. Place at dz=+1 as control, verify dz works
3. Place at dx=-1, check both [-1,0,0] and [1,0,0]
4. Log absolute world coordinates to verify transformation math
"""

import argparse
import json
import math
import time
from typing import Dict, List, Optional, Tuple

import zenoh
from zenoh import QueryTarget, ConsolidationMode


def execute_plan(session: zenoh.Session, character: str, plan: List[Dict], timeout: float = 30.0) -> Dict:
    """Execute a plan via Zenoh API."""
    query_key = f"cognitive/{character}/execute_plan_sync"
    payload = json.dumps({'plan': plan})

    try:
        replies = session.get(
            query_key,
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            payload=payload.encode('utf-8'),
            timeout=timeout
        )
        for reply in replies:
            if hasattr(reply, 'ok') and reply.ok is not None:
                return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
        return {'success': False, 'error': 'No response'}
    except Exception as e:
        return {'success': False, 'error': str(e)}


def execute_action(session: zenoh.Session, character: str, action_dict: Dict, timeout: float = 10.0) -> Dict:
    """Execute single action."""
    result = execute_plan(session, character, [action_dict], timeout)
    if result.get('success'):
        return result.get('last_action_result', {})
    return {'status': 'failed', 'reason': result.get('error', 'Unknown')}


def get_status(session: zenoh.Session, character: str) -> Optional[Dict]:
    """Get agent status."""
    result = execute_action(session, character, {"type": "mc-status"})
    if result.get('status') == 'success':
        return result.get('data', {})
    return None


def get_observe(session: zenoh.Session, character: str) -> Optional[Dict]:
    """Get observation to populate grid."""
    result = execute_action(session, character, {"type": "mc-observe"})
    if result.get('status') == 'success':
        return result.get('data', {})
    return None


def get_voxel_grid(session: zenoh.Session, character: str, radius: int = 3) -> Optional[Dict]:
    """Get voxel grid with larger radius for diagnostics."""
    result = execute_action(session, character, {"type": "debug-get-voxel-grid", "radius": radius})
    if result.get('status') == 'success':
        return result.get('extra', {}).get('voxel_grid')
    return None


def find_block_in_grid(voxel_grid: Dict, block_type: str) -> List[Dict]:
    """Find all cells containing a specific block type."""
    matches = []
    if not voxel_grid:
        return matches

    cells = voxel_grid.get('cells', [])
    for cell in cells:
        if isinstance(cell, dict):
            block_id = cell.get('block_id', '')
            if block_id and block_type.lower() in block_id.lower():
                matches.append(cell)
    return matches


def get_cell_at(voxel_grid: Dict, dx: int, dy: int, dz: int) -> Optional[Dict]:
    """Get cell at specific agent-relative coordinates."""
    if not voxel_grid:
        return None

    cells = voxel_grid.get('cells', [])
    for cell in cells:
        if isinstance(cell, dict):
            if cell.get('dx') == dx and cell.get('dy') == dy and cell.get('dz') == dz:
                return cell
    return None


def agent_rel_to_world(dx: float, dy: float, dz: float, yaw: float) -> Tuple[float, float, float]:
    """Convert agent-relative to world coordinates."""
    yaw_rad = math.radians(yaw)
    world_dx = -math.sin(yaw_rad) * dz - math.cos(yaw_rad) * dx
    world_dz = -math.cos(yaw_rad) * dz + math.sin(yaw_rad) * dx
    return world_dx, dy, world_dz


def dig_block(session: zenoh.Session, character: str, dx: float, dy: float, dz: float):
    """Dig block at position (cleanup helper)."""
    execute_action(session, character, {"type": "mc-dig", "dx": dx, "dy": dy, "dz": dz})


def print_separator(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print('='*60)


def run_diagnostic(session: zenoh.Session, character: str):
    """Run all diagnostic tests."""

    print_separator("DIAGNOSTIC: dx Coordinate Mismatch Analysis")

    # Get initial status
    status = get_status(session, character)
    if not status:
        print("ERROR: Failed to get agent status")
        return

    pos = status.get('position', {})
    yaw = status.get('yaw', 0.0)
    agent_x = pos.get('x', 0)
    agent_y = pos.get('y', 0)
    agent_z = pos.get('z', 0)

    print(f"\nAgent Position: ({agent_x:.2f}, {agent_y:.2f}, {agent_z:.2f})")
    print(f"Agent Yaw: {yaw:.1f} degrees")
    print(f"Agent Block: ({int(math.floor(agent_x))}, {int(math.floor(agent_y))}, {int(math.floor(agent_z))})")

    # Ensure we're at a known yaw for predictable results
    # Turn to yaw 0 if not already there
    if abs(yaw) > 5 and abs(yaw - 360) > 5:
        print(f"\nTurning to yaw 0 for consistent testing...")
        turns = int(round(-yaw / 90)) % 4
        for _ in range(turns):
            execute_action(session, character, {"type": "nav-turn", "direction": "left"})
        status = get_status(session, character)
        yaw = status.get('yaw', 0.0) if status else 0.0
        print(f"New yaw: {yaw:.1f} degrees")

    # Refresh position after any turns
    status = get_status(session, character)
    pos = status.get('position', {})
    agent_x = pos.get('x', 0)
    agent_y = pos.get('y', 0)
    agent_z = pos.get('z', 0)
    yaw = status.get('yaw', 0.0)

    # =========================================================================
    # TEST 1: Place at dx=+1, check ALL adjacent cells
    # =========================================================================
    print_separator("TEST 1: Place at dx=+1, scan all adjacent cells")

    # Populate grid first
    get_observe(session, character)

    # Place block at dx=+1 (right)
    print(f"\nPlacing dirt at dx=+1, dy=0, dz=0 (right)...")
    place_result = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 1.0,
        "dy": 0.0,
        "dz": 0.0,
        "face": "west"
    })
    print(f"Place result: {place_result.get('status', 'unknown')}")
    if place_result.get('status') != 'success':
        print(f"  Reason: {place_result.get('reason', 'unknown')}")

    # Get voxel grid with larger radius
    voxel_grid = get_voxel_grid(session, character, radius=3)
    if not voxel_grid:
        print("ERROR: Failed to get voxel grid")
    else:
        # Find all dirt blocks
        dirt_cells = find_block_in_grid(voxel_grid, 'dirt')
        print(f"\nDirt blocks found in grid: {len(dirt_cells)}")
        for cell in dirt_cells:
            print(f"  Dirt at: dx={cell.get('dx')}, dy={cell.get('dy')}, dz={cell.get('dz')}")

        # Check specific positions
        print("\nChecking specific positions:")
        positions_to_check = [
            (1, 0, 0, "expected: dx=+1 (right)"),
            (-1, 0, 0, "sign inversion would put it here"),
            (2, 0, 0, "off-by-one would put it here"),
            (0, 0, 1, "axes swapped would put it here"),
            (0, 0, -1, "axes swapped + sign inversion"),
        ]

        for dx, dy, dz, desc in positions_to_check:
            cell = get_cell_at(voxel_grid, dx, dy, dz)
            block = cell.get('block_id', 'not in grid') if cell else 'not in grid'
            marker = " <-- DIRT FOUND" if cell and 'dirt' in block.lower() else ""
            print(f"  [{dx:+d},{dy:+d},{dz:+d}] {desc}: {block}{marker}")

    # Clean up - try to dig at multiple positions
    for dx in [-2, -1, 0, 1, 2]:
        dig_block(session, character, float(dx), 0.0, 0.0)

    # =========================================================================
    # TEST 2: Place at dz=+1 (control test for forward axis)
    # =========================================================================
    print_separator("TEST 2: Place at dz=+1 (control - forward axis)")

    get_observe(session, character)

    print(f"\nPlacing dirt at dx=0, dy=0, dz=+1 (forward)...")
    place_result = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 0.0,
        "dy": 0.0,
        "dz": 1.0,
        "face": "north"
    })
    print(f"Place result: {place_result.get('status', 'unknown')}")

    voxel_grid = get_voxel_grid(session, character, radius=3)
    if voxel_grid:
        dirt_cells = find_block_in_grid(voxel_grid, 'dirt')
        print(f"\nDirt blocks found: {len(dirt_cells)}")
        for cell in dirt_cells:
            print(f"  Dirt at: dx={cell.get('dx')}, dy={cell.get('dy')}, dz={cell.get('dz')}")

        # Check if dz=+1 has the block
        cell_forward = get_cell_at(voxel_grid, 0, 0, 1)
        if cell_forward and 'dirt' in cell_forward.get('block_id', '').lower():
            print("\n  RESULT: dz=+1 (forward) works correctly")
        else:
            print("\n  RESULT: dz=+1 (forward) also broken - broader issue")

    # Clean up
    for dz in [-2, -1, 0, 1, 2]:
        dig_block(session, character, 0.0, 0.0, float(dz))

    # =========================================================================
    # TEST 3: Place at dx=-1, check both [-1,0,0] and [+1,0,0]
    # =========================================================================
    print_separator("TEST 3: Place at dx=-1, check sign behavior")

    get_observe(session, character)

    print(f"\nPlacing dirt at dx=-1, dy=0, dz=0 (left)...")
    place_result = execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": -1.0,
        "dy": 0.0,
        "dz": 0.0,
        "face": "east"
    })
    print(f"Place result: {place_result.get('status', 'unknown')}")

    voxel_grid = get_voxel_grid(session, character, radius=3)
    if voxel_grid:
        dirt_cells = find_block_in_grid(voxel_grid, 'dirt')
        print(f"\nDirt blocks found: {len(dirt_cells)}")
        for cell in dirt_cells:
            print(f"  Dirt at: dx={cell.get('dx')}, dy={cell.get('dy')}, dz={cell.get('dz')}")

        cell_neg1 = get_cell_at(voxel_grid, -1, 0, 0)
        cell_pos1 = get_cell_at(voxel_grid, 1, 0, 0)

        has_at_neg1 = cell_neg1 and 'dirt' in cell_neg1.get('block_id', '').lower()
        has_at_pos1 = cell_pos1 and 'dirt' in cell_pos1.get('block_id', '').lower()

        print("\nAnalysis:")
        if has_at_neg1 and not has_at_pos1:
            print("  dx=-1 placed at [-1,0,0] - LEFT works correctly")
        elif has_at_pos1 and not has_at_neg1:
            print("  dx=-1 placed at [+1,0,0] - SIGN INVERSION CONFIRMED")
        elif has_at_neg1 and has_at_pos1:
            print("  Found dirt at both positions - multiple blocks or pre-existing")
        else:
            print("  Dirt not found at expected positions - check other locations")

    # Clean up
    for dx in [-2, -1, 0, 1, 2]:
        dig_block(session, character, float(dx), 0.0, 0.0)

    # =========================================================================
    # TEST 4: World coordinate verification
    # =========================================================================
    print_separator("TEST 4: World coordinate verification")

    # Get fresh status
    status = get_status(session, character)
    pos = status.get('position', {})
    yaw = status.get('yaw', 0.0)
    agent_x = pos.get('x', 0)
    agent_z = pos.get('z', 0)
    agent_bx = int(math.floor(agent_x))
    agent_bz = int(math.floor(agent_z))
    agent_by = int(math.floor(pos.get('y', 0)))

    print(f"\nAgent block position: ({agent_bx}, {agent_by}, {agent_bz})")
    print(f"Agent yaw: {yaw:.1f} degrees")

    # Calculate expected world position for dx=+1 at current yaw
    world_dx, _, world_dz = agent_rel_to_world(1.0, 0.0, 0.0, yaw)
    expected_world_x = agent_bx + int(round(world_dx))
    expected_world_z = agent_bz + int(round(world_dz))

    print(f"\nExpected world position for dx=+1 (right):")
    print(f"  Transform: agent_rel_to_world(1, 0, 0, {yaw}) = ({world_dx:.2f}, 0, {world_dz:.2f})")
    print(f"  Expected block: ({expected_world_x}, {agent_by}, {expected_world_z})")

    # Also calculate for dx=-1
    world_dx_neg, _, world_dz_neg = agent_rel_to_world(-1.0, 0.0, 0.0, yaw)
    expected_neg_x = agent_bx + int(round(world_dx_neg))
    expected_neg_z = agent_bz + int(round(world_dz_neg))
    print(f"\nExpected world position for dx=-1 (left):")
    print(f"  Transform: agent_rel_to_world(-1, 0, 0, {yaw}) = ({world_dx_neg:.2f}, 0, {world_dz_neg:.2f})")
    print(f"  Expected block: ({expected_neg_x}, {agent_by}, {expected_neg_z})")

    get_observe(session, character)

    # Place at dx=+1
    print(f"\nPlacing dirt at dx=+1...")
    execute_action(session, character, {
        "type": "mc-place",
        "item": "dirt",
        "dx": 1.0,
        "dy": 0.0,
        "dz": 0.0,
        "face": "west"
    })

    # Get voxel grid and find the dirt block's world position
    voxel_grid = get_voxel_grid(session, character, radius=3)
    if voxel_grid:
        dirt_cells = find_block_in_grid(voxel_grid, 'dirt')
        center = voxel_grid.get('center', {})
        center_x = center.get('x', agent_x)
        center_z = center.get('z', agent_z)

        print(f"\nGrid center: ({center_x:.2f}, {center.get('y', 0):.2f}, {center_z:.2f})")

        for cell in dirt_cells:
            cell_dx = cell.get('dx', 0)
            cell_dy = cell.get('dy', 0)
            cell_dz = cell.get('dz', 0)

            # Calculate world position from grid cell
            # Grid uses agent-relative coords, so we need to transform back
            cell_world_dx, _, cell_world_dz = agent_rel_to_world(cell_dx, cell_dy, cell_dz, yaw)
            cell_world_x = int(math.floor(center_x)) + int(round(cell_world_dx))
            cell_world_z = int(math.floor(center_z)) + int(round(cell_world_dz))
            cell_world_y = int(math.floor(center.get('y', 0))) + cell_dy

            print(f"\nDirt found at grid [{cell_dx:+d},{cell_dy:+d},{cell_dz:+d}]")
            print(f"  Calculated world position: ({cell_world_x}, {cell_world_y}, {cell_world_z})")

            if cell_world_x == expected_world_x and cell_world_z == expected_world_z:
                print("  MATCH: Block is where mc-place should have put it")
                print("  CONCLUSION: Grid observation has coordinate error")
            elif cell_world_x == expected_neg_x and cell_world_z == expected_neg_z:
                print("  MATCH with dx=-1 position!")
                print("  CONCLUSION: mc-place has sign inversion on dx")
            else:
                print(f"  NO MATCH: Expected ({expected_world_x}, {agent_by}, {expected_world_z})")
                print(f"            or ({expected_neg_x}, {agent_by}, {expected_neg_z})")

    # Final cleanup
    for dx in [-2, -1, 0, 1, 2]:
        for dz in [-2, -1, 0, 1, 2]:
            dig_block(session, character, float(dx), 0.0, float(dz))

    # =========================================================================
    # SUMMARY
    # =========================================================================
    print_separator("DIAGNOSTIC SUMMARY")
    print("""
Based on test results above, the root cause is:

1. SIGN INVERSION: If placing at dx=+1 results in block at [-1,0,0]
   - Fix: Negate dx in mc-place OR in grid observation

2. OFF-BY-ONE: If placing at dx=+1 results in block at [2,0,0] or [0,0,0]
   - Fix: Check rounding/floor operations in coordinate transform

3. AXES SWAPPED: If placing at dx=+1 results in block at [0,0,1]
   - Fix: dx and dz are transposed somewhere in the pipeline

4. GRID OBSERVATION ERROR: If world coords match expected but grid coords don't
   - Fix: Issue is in debug-get-voxel-grid coordinate reporting
""")


def main():
    parser = argparse.ArgumentParser(description="Diagnose dx coordinate mismatch")
    parser.add_argument("--character", type=str, default="Jill", help="Character name")
    args = parser.parse_args()

    print("Connecting to Zenoh...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("Connected.\n")

    try:
        run_diagnostic(session, args.character)
    finally:
        session.close()
        print("\nZenoh session closed.")


if __name__ == "__main__":
    main()
