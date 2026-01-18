import json
import time
import math
import threading
import itertools
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from typing import Dict, Any, Tuple, List, Optional, Set
from http.server import BaseHTTPRequestHandler, HTTPServer

import minescript as m  # pyright: ignore[reportMissingImports]

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------

HOST = "127.0.0.1"
PORT = 3003
MAX_RADIUS = 10

# ------------------------------------------------------------------------------
# Math & Coordinate Helpers
# ------------------------------------------------------------------------------

def get_player_pos() -> Tuple[float, float, float]:
    """Returns (x, y, z)."""
    return m.player_position()

def get_player_rot() -> Tuple[float, float]:
    """Returns (yaw, pitch)."""
    # Minescript returns [yaw, pitch] list
    rot = m.player_orientation()
    return (rot[0], rot[1]) if rot else (0.0, 0.0)

def rel_to_abs(rel: Dict[str, float]) -> Tuple[int, int, int]:
    """
    Converts relative coordinates (egocentric or offset) to absolute world integer coordinates.
    Supports: {'dx', 'dy', 'dz'} OR {'forward', 'right', 'up'}
    """
    px, py, pz = get_player_pos()

    # Cartesian offset
    if any(k in rel for k in ("dx", "dy", "dz")):
        return (
            int(px + rel.get("dx", 0)),
            int(py + rel.get("dy", 0)),
            int(pz + rel.get("dz", 0)),
        )

    # Egocentric offset
    # Yaw in Minecraft: 0=South, -90=East, 90=West, 180/-180=North
    # Math: 0 radian is usually East. We must convert carefully.
    yaw_deg, _ = get_player_rot()
    yaw_rad = math.radians(yaw_deg)
    
    fwd = float(rel.get("forward", 0))
    right = float(rel.get("right", 0))
    up = float(rel.get("up", 0))

    # Standard Minecraft Yaw to Cartesian conversion
    # Forward vector: (-sin(yaw), -cos(yaw))
    # Right vector: forward rotated 90° counter-clockwise = (cos(yaw), -sin(yaw))
    # dx = -sin(yaw) * fwd + cos(yaw) * right
    # dz = -cos(yaw) * fwd - sin(yaw) * right
    dx = -math.sin(yaw_rad) * fwd + math.cos(yaw_rad) * right
    dz = -math.cos(yaw_rad) * fwd - math.sin(yaw_rad) * right

    return (
        int(round(px + dx)),
        int(round(py + up)),
        int(round(pz + dz)),
    )

# ------------------------------------------------------------------------------
# State & Action Tracking
# ------------------------------------------------------------------------------

class State:
    last_action: Dict[str, Any] = {
        "id": None, 
        "type": None, 
        "timestamp": 0, 
        "note": "bridge_started"
    }
    active_dig_future = None  # Track active dig operation for cancellation
    active_dig_attack_func = None  # Track attack function to stop on cancellation
    # One-time diagnostics for Minescript execution context behavior
    script_loop_checked: bool = False
    script_loop_available: Optional[bool] = None
    script_loop_enter_ok: Optional[bool] = None
    script_loop_error: Optional[str] = None

def log_action(action_type: str, note: str = ""):
    State.last_action = {
        "id": f"{action_type}-{int(time.time()*1000)}",
        "type": action_type,
        "timestamp": int(time.time()*1000),
        "note": note,
    }

def cancel_active_dig():
    """Cancel any active dig operation to allow new actions."""
    if State.active_dig_future and not State.active_dig_future.done():
        State.active_dig_future.cancel()
        # Stop attacking if function is available
        if State.active_dig_attack_func:
            try:
                State.active_dig_attack_func(False)
            except:
                pass
        State.active_dig_future = None
        State.active_dig_attack_func = None
        log_action("dig_cancelled", "interrupted by new action")

# ------------------------------------------------------------------------------
# Core Logic
# ------------------------------------------------------------------------------

def get_status() -> Dict[str, Any]:
    """Retrieves player state using proper API calls."""
    x, y, z = get_player_pos()
    yaw, pitch = get_player_rot()
    
    # Accessing standard Minescript/Fabric API mappings
    # If standard functions aren't available, we fall back to safe defaults
    try:
        health = m.player_health()
    except AttributeError:
        health = getattr(m.player(), "getHealth", lambda: 20.0)()

    # Food level often requires accessing the HungerManager in 1.20+
    food = 20
    try:
        # Try direct Minescript API
        if hasattr(m, "player_food_level"):
            food = m.player_food_level()
        else:
            # Fallback to Java object wrapper (Yarn/Fabric mappings)
            p = m.player()
            if hasattr(p, "getHungerManager"):
                food = p.getHungerManager().getFoodLevel()
            elif hasattr(p, "getFoodData"):
                food = p.getFoodData().getFoodLevel()
    except Exception:
        pass

    return {
        "ok": True,
        "position": {"x": x, "y": y, "z": z},
        "yaw": yaw,
        "pitch": pitch,
        "health": health,
        "food": food,
        "onGround": None,  # Not available in minescript API
        "dimension": None,  # Not available in minescript API
        "action": State.last_action,
    }

def compute_visibility_distances(radius: int) -> Dict[str, Optional[float]]:
    """Compute visibility distances in 6 directions using raycast (optimized for speed)."""
    px, py, pz = get_player_pos()
    yaw_deg, _ = get_player_rot()
    yaw_rad = math.radians(yaw_deg)
    
    # Eye position (1.6 blocks above feet)
    eye_y = py + 1.6
    
    # Direction vectors based on yaw
    # Forward: direction player is facing
    forward_dx = -math.sin(yaw_rad)
    forward_dz = -math.cos(yaw_rad)
    
    # Back: opposite of forward
    back_dx = -forward_dx
    back_dz = -forward_dz
    
    # Left: 90° left of forward (rotate forward vector 90° counterclockwise)
    left_dx = -forward_dz
    left_dz = forward_dx
    
    # Right: 90° right of forward (rotate forward vector 90° clockwise)
    right_dx = forward_dz
    right_dz = -forward_dx
    
    # Transparent blocks (don't block visibility)
    transparent_blocks = {'air', 'glass', 'water', 'ice', 'leaves', 'slab', 'stairs', 
                          'fence', 'fence_gate', 'wall', 'ladder', 'vine', 'cobweb'}
    
    def raycast_direction(dx: float, dy: float, dz: float, max_dist: float) -> Optional[float]:
        """Raycast in a direction, return distance to nearest opaque block (optimized)."""
        # Use larger step size for speed: check every 0.5 blocks instead of 0.1
        # This reduces checks by 5x while still being accurate enough
        step = 0.5
        steps = int(max_dist / step)
        
        for i in range(1, steps + 1):
            dist = i * step
            check_x = px + dx * dist
            check_y = eye_y + dy * dist
            check_z = pz + dz * dist
            
            # Get block at this position
            try:
                try:
                    block = m.getblock(int(check_x), int(check_y), int(check_z))
                except (AttributeError, TypeError) as e:
                    print(f"[Bridge] compute_visibility_distances: ERROR getting block at {check_x},{check_y},{check_z}: {e}", flush=True)
                    block = m.get_block(int(check_x), int(check_y), int(check_z))
            except Exception:
                continue
            
            # Handle both string and object returns
            block_name = None
            if isinstance(block, str):
                block_name = block.lower()
            elif block:
                if hasattr(block, 'is_air') and block.is_air():
                    continue
                block_name = getattr(block, 'name', str(block)).lower()
            
            if block_name and block_name not in transparent_blocks and 'air' not in block_name:
                # Found opaque block
                return round(dist, 1)  # Less precision for speed
        
        # No opaque block found within radius
        return None
    
    # Raycast in each direction
    result = {
        'forward': raycast_direction(forward_dx, 0, forward_dz, radius),
        'back': raycast_direction(back_dx, 0, back_dz, radius),
        'left': raycast_direction(left_dx, 0, left_dz, radius),
        'right': raycast_direction(right_dx, 0, right_dz, radius),
        'up': raycast_direction(0, 1, 0, radius),
        'down': raycast_direction(0, -1, 0, radius),
    }
    return result

def is_position_visible(eye_x: float, eye_y: float, eye_z: float, target_x: int, target_y: int, target_z: int) -> bool:
    """Check if target block position is visible from eye position (not occluded)."""
    # Calculate direction vector
    dx = target_x + 0.5 - eye_x  # Block center
    dy = target_y + 0.5 - eye_y
    dz = target_z + 0.5 - eye_z
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 0.1:  # Too close, consider visible
        return True
    
    # Normalize direction
    dx /= dist
    dy /= dist
    dz /= dist
    
    # Raycast with 0.5 step size (same as visibility_distances)
    step = 0.5
    steps = int(dist / step)
    transparent_blocks = {'air', 'glass', 'water', 'ice', 'leaves', 'slab', 'stairs', 
                          'fence', 'fence_gate', 'wall', 'ladder', 'vine', 'cobweb'}
    
    for i in range(1, steps):
        check_dist = i * step
        check_x = int(eye_x + dx * check_dist)
        check_y = int(eye_y + dy * check_dist)
        check_z = int(eye_z + dz * check_dist)
        
        # Stop if we've reached target position
        if check_x == target_x and check_y == target_y and check_z == target_z:
            return True
        
        # Check if this position has an opaque block
        try:
            try:
                block = m.getblock(check_x, check_y, check_z)
            except (AttributeError, TypeError):
                block = m.get_block(check_x, check_y, check_z)
        except Exception:
            continue
        
        block_name = None
        if isinstance(block, str):
            block_name = block.lower()
        elif block:
            if hasattr(block, 'is_air') and block.is_air():
                continue
            block_name = getattr(block, 'name', str(block)).lower()
        
        if block_name and block_name not in transparent_blocks and 'air' not in block_name:
            # Found opaque block before reaching target - occluded
            return False
    
    return True  # No occlusion found

def scan_blocks(radius: int) -> List[Dict[str, Any]]:
    """
    Navigation-first scan (Option B): enumerate candidate (x,z) cells in a forward cone,
    probe for *support* (walkable surface) via a small downward search, and synthesize a
    lightweight block list from those samples.
    
    This is intentionally NOT "visual rendering visibility". It is tuned for navigation:
    thin layers/cover that do not change support_y should not prevent us from learning
    standable surfaces.
    """
    radius = max(1, min(MAX_RADIUS, radius))
    px, py, pz = get_player_pos()
    eye_y = py + 1.6  # Eye position (used only for legacy angle calculations)
    # Use floor() to match tool-side coordinate system (consistent with negative coordinates)
    cx, cy, cz = int(math.floor(px)), int(math.floor(py)), int(math.floor(pz))

    yaw_deg, _ = get_player_rot()
    yaw_rad = math.radians(yaw_deg)
    fwd_dx = -math.sin(yaw_rad)
    # Use -cos(yaw) to match rel_to_abs() convention (forward Z direction)
    fwd_dz = -math.cos(yaw_rad)

    # Cache for m.getblock calls for this scan
    block_cache: Dict[Tuple[int, int, int], Optional[str]] = {}

    # Non-supporting blocks: occupy space but do not provide vertical support
    NON_SUPPORTING_PREFIXES = (
        'minecraft:snow',  # snow layers (any layers=N)
        'minecraft:carpet',
        'minecraft:pressure_plate',
        'minecraft:farmland',
        'minecraft:rail',
        'minecraft:trapdoor',
        'minecraft:short_grass',
        'minecraft:tall_grass',
    )

    def get_block_name(bx: int, by: int, bz: int) -> Optional[str]:
        """Return block name at coords, or None if air/unknown. Cached to reduce getblock calls."""
        key = (bx, by, bz)
        if key in block_cache:
            return block_cache[key]

        try:
            try:
                block = m.getblock(bx, by, bz)
            except (AttributeError, TypeError):
                block = m.get_block(bx, by, bz)
        except Exception:
            block_cache[key] = None
            return None

        block_name = None
        if isinstance(block, str):
            block_name = block
        elif block:
            if hasattr(block, 'is_air') and block.is_air():
                block_cache[key] = None
                return None
            block_name = getattr(block, 'name', str(block))

        if not block_name:
            block_cache[key] = None
            return None

        lower = block_name.lower()
        if lower in ('air', 'minecraft:air', '') or 'air' in lower:
            block_cache[key] = None
            return None

        block_cache[key] = block_name
        return block_name

    def in_view_cone_2d(bx: int, bz: int) -> bool:
        """Horizontal cone only (yaw ±60°)."""
        dx = (bx + 0.5) - px
        dz = (bz + 0.5) - pz
        dist_h = math.sqrt(dx * dx + dz * dz)
        if dist_h <= 0.001:
            return True
        vx_h = dx / dist_h
        vz_h = dz / dist_h
        return (vx_h * fwd_dx + vz_h * fwd_dz) >= 0.5  # cos(60°)

    def is_non_supporting(block_name: str) -> bool:
        bl = block_name.lower()
        return any(bl.startswith(p) for p in NON_SUPPORTING_PREFIXES)

    def is_walkable_support(block_name: str) -> bool:
        bl = block_name.lower()
        # KISS: treat obvious fluids/hazards as non-walkable
        if 'water' in bl or 'lava' in bl:
            return False
        return True

    # --- nav_surface scan (cached/capped) ---
    nav_surface: List[Dict[str, Any]] = []
    max_nav_cells = 120  # cap work; radius=7 diamond is 113 total cells pre-cone
    max_up = 2
    max_down = min(radius + 1, 6)  # Adaptive depth: radius+1, capped at 6

    visited_xz: Set[Tuple[int, int]] = set()
    for shell_dist in range(0, radius + 1):
        if len(nav_surface) >= max_nav_cells:
            break
        shell_positions = []
        for dx in range(-shell_dist, shell_dist + 1):
            for dz in range(-shell_dist, shell_dist + 1):
                if abs(dx) + abs(dz) == shell_dist:
                    shell_positions.append((dx, dz))
        for dx, dz in shell_positions:
            if len(nav_surface) >= max_nav_cells:
                break
            bx, bz = cx + dx, cz + dz
            if (bx, bz) in visited_xz:
                continue
            visited_xz.add((bx, bz))

            if not in_view_cone_2d(bx, bz):
                continue

            support_y = None
            support_block = None

            # Probe down for supporting block near the agent's current Y.
            for by in range(cy + max_up, cy - max_down - 1, -1):
                bname = get_block_name(bx, by, bz)
                if not bname:
                    continue
                if is_non_supporting(bname):
                    continue
                support_y = by
                support_block = bname
                break

            if support_y is None or support_block is None:
                continue

            surface_y = support_y + 1
            cover_block = get_block_name(bx, surface_y, bz)  # may be snow/grass/None

            nav_surface.append({
                "x": bx,
                "z": bz,
                "dx": dx,
                "dz": dz,
                "support_y": support_y,
                "support_block": support_block,
                "walkable": bool(is_walkable_support(support_block)),
                "cover_block": cover_block,
            })

    # --- synthesize blocks list from nav_surface ---
    blocks: List[Dict[str, Any]] = []
    max_blocks = 200

    def append_block(name: Optional[str], bx: int, by: int, bz: int, dx: int, dy: int, dz: int):
        if not name:
            return
        if len(blocks) >= max_blocks:
            return
        blocks.append({
            "name": name,
            "position": {"x": bx, "y": by, "z": bz},
            "dx": dx,
            "dy": dy,
            "dz": dz,
        })

    for ns in nav_surface:
        if len(blocks) >= max_blocks:
            break
        bx = int(ns["x"])
        bz = int(ns["z"])
        dx = int(ns.get("dx", bx - cx))
        dz = int(ns.get("dz", bz - cz))
        support_y = int(ns["support_y"])
        surface_y = support_y + 1

        # cover block at surface level (e.g., snow layer / tall grass)
        append_block(ns.get("cover_block"), bx, surface_y, bz, dx, surface_y - cy, dz)
        # supporting block (useful for nav/map update)
        append_block(ns.get("support_block"), bx, support_y, bz, dx, support_y - cy, dz)

    # Tri-state surface tagging: True/False/"unknown", derived from cached block reads.
    for b in blocks:
        pos = b.get("position") or {}
        bx = int(pos.get("x", 0))
        by = int(pos.get("y", 0))
        bz = int(pos.get("z", 0))
        above_key = (bx, by + 1, bz)
        if above_key in block_cache:
            b["surface"] = block_cache[above_key] is None
        else:
            b["surface"] = "unknown"

    # Attach nav_surface for /observe payload via a side-channel attribute on the function
    # (keeps signature stable for callers).
    scan_blocks._last_nav_surface = nav_surface  # type: ignore[attr-defined]

    return blocks

def handle_ground_truth(center_x: int, center_y: int, center_z: int, radius: int) -> Dict[str, Any]:
    """
    Generate ground truth grid for training data capture.
    
    Returns full voxel grid (25×25×25 for radius=12) with all properties:
    - Block names (including air)
    - Derived properties: support, passable, head_clear, hazard, breakable
    - Structure cues: surface_likeness, wall_likeness, etc.
    """
    try:
        # Use script_loop if available for performance
        if getattr(State, "script_loop_available", False):
            try:
                with m.script_loop:
                    return _compute_ground_truth_grid(center_x, center_y, center_z, radius)
            except Exception as e:
                print(f"[Bridge] /ground-truth: WARNING failed to use m.script_loop: {e}", flush=True)
                return _compute_ground_truth_grid(center_x, center_y, center_z, radius)
        else:
            return _compute_ground_truth_grid(center_x, center_y, center_z, radius)
    except Exception as e:
        print(f"[Bridge] /ground-truth: ERROR: {e}", flush=True)
        return {"ok": False, "error": str(e)}


def _compute_ground_truth_grid(center_x: int, center_y: int, center_z: int, radius: int) -> Dict[str, Any]:
    """
    Compute ground truth grid by querying all voxels in radius.
    
    Returns:
        {
            "ok": True,
            "center": {"x": int, "y": int, "z": int},
            "radius": int,
            "voxels": [{"x": int, "y": int, "z": int, "name": str, "properties": {...}}, ...]
        }
    """
    block_cache: Dict[Tuple[int, int, int], Optional[str]] = {}
    
    def get_block_name(bx: int, by: int, bz: int) -> Optional[str]:
        """Get block name, caching results."""
        key = (bx, by, bz)
        if key in block_cache:
            return block_cache[key]
        
        try:
            try:
                block = m.getblock(bx, by, bz)
            except (AttributeError, TypeError):
                block = m.get_block(bx, by, bz)
        except Exception:
            block_cache[key] = None
            return None
        
        block_name = None
        if isinstance(block, str):
            block_name = block
        elif block:
            if hasattr(block, 'is_air') and block.is_air():
                block_cache[key] = "air"
                return "air"
            block_name = getattr(block, 'name', str(block))
        
        if not block_name:
            block_cache[key] = "air"
            return "air"
        
        block_cache[key] = block_name
        return block_name
    
    def is_air(name: Optional[str]) -> bool:
        if not name:
            return True
        n = name.lower().replace("minecraft:", "")
        return n in {"air", "cave_air", "void_air"}
    
    def is_hazard(name: str) -> bool:
        n = name.lower().replace("minecraft:", "")
        hazards = {'lava', 'fire', 'cactus', 'magma_block', 'soul_fire', 'wither_rose', 'sweet_berry_bush', 'campfire', 'soul_campfire'}
        return any(h in n for h in hazards)
    
    def is_breakable(name: str) -> bool:
        n = name.lower().replace("minecraft:", "")
        if "air" in n or "bedrock" in n:
            return False
        return True
    
    def is_solid(name: str) -> bool:
        if is_air(name):
            return False
        n = name.lower().replace("minecraft:", "")
        non_solid = {'water', 'lava', 'flowing_water', 'flowing_lava', 'air', 'cave_air', 'void_air'}
        return not any(ns in n for ns in non_solid)
    
    voxels = []
    start_time = time.time()
    
    # Query all voxels in radius (Chebyshev distance)
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            for dz in range(-radius, radius + 1):
                if max(abs(dx), abs(dy), abs(dz)) > radius:
                    continue
                
                bx = center_x + dx
                by = center_y + dy
                bz = center_z + dz
                
                name = get_block_name(bx, by, bz)
                if name is None:
                    name = "air"
                
                # Compute derived properties
                air_here = is_air(name)
                solid_here = is_solid(name)
                
                # Support: solid block provides support, air above solid provides support
                support = False
                if solid_here:
                    support = True
                elif air_here:
                    below_name = get_block_name(bx, by - 1, bz)
                    if below_name and is_solid(below_name):
                        support = True
                
                # Passable: air is passable
                passable = air_here
                
                # Head clear: air above is clear
                head_clear = False
                above_name = get_block_name(bx, by + 1, bz)
                head_clear = (above_name is None) or is_air(above_name)
                
                # Structure cues (simplified heuristics)
                surface_likeness = False
                wall_likeness = False
                enclosure_likeness = False
                
                if solid_here:
                    # Surface: solid with air above
                    if head_clear:
                        surface_likeness = True
                    # Wall: solid with air on side
                    side_air = False
                    for sx, sy, sz in [(bx+1, by, bz), (bx-1, by, bz), (bx, by, bz+1), (bx, by, bz-1)]:
                        side_name = get_block_name(sx, sy, sz)
                        if side_name is None or is_air(side_name):
                            side_air = True
                            break
                    if side_air:
                        wall_likeness = True
                    # Enclosure: solid surrounded by solids
                    surrounded = True
                    for sx, sy, sz in [(bx+1, by, bz), (bx-1, by, bz), (bx, by+1, bz), (bx, by-1, bz), (bx, by, bz+1), (bx, by, bz-1)]:
                        neighbor_name = get_block_name(sx, sy, sz)
                        if neighbor_name is None or is_air(neighbor_name):
                            surrounded = False
                            break
                    if surrounded:
                        enclosure_likeness = True
                
                voxel = {
                    "x": bx,
                    "y": by,
                    "z": bz,
                    "name": name,
                    "properties": {
                        "support": support,
                        "passable": passable,
                        "head_clear": head_clear,
                        "hazard": is_hazard(name),
                        "breakable": is_breakable(name),
                        "structure_cues": {
                            "surface_likeness": surface_likeness,
                            "wall_likeness": wall_likeness,
                            "enclosure_likeness": enclosure_likeness,
                        },
                    },
                }
                voxels.append(voxel)
    
    elapsed_ms = (time.time() - start_time) * 1000
    print(f"[Bridge] /ground-truth: Computed {len(voxels)} voxels in {elapsed_ms:.1f}ms", flush=True)
    
    return {
        "ok": True,
        "center": {"x": center_x, "y": center_y, "z": center_z},
        "radius": radius,
        "voxels": voxels,
        "elapsed_ms": elapsed_ms,
    }

def scan_entities(radius: int, entity_filter: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Scan entities around the player.
    entity_filter: "items" to only return item entities, None for all entities.
    """
    radius = max(1, min(MAX_RADIUS, radius))
    px, py, pz = get_player_pos()
    entities = []
    
    # Try Minescript entity APIs
    try:
        # Try common entity scanning APIs
        entities_func = None
        for api_name in ['get_entities', 'entities', 'player_get_entities', 'scan_entities']:
            try:
                entities_func = getattr(m, api_name, None)
                if entities_func and callable(entities_func):
                    break
            except:
                continue
        
        if entities_func:
            # Call with radius
            try:
                entity_list = entities_func(radius)
            except TypeError:
                # Try without radius
                try:
                    entity_list = entities_func()
                except:
                    entity_list = []
            
            # Helper for visibility check
            # We must respect the 120x120 cone and line-of-sight
            yaw_deg, _ = get_player_rot()
            yaw_rad = math.radians(yaw_deg)
            # View vector (horizontal only for cone check logic, or full 3D?)
            # Requirement: "120 x 120 degrees, centered fwd at pitch 0"
            # So we check relative yaw and relative pitch from (yaw, 0)
            
            # Forward vector at pitch 0
            # dx = -sin(yaw), dz = -cos(yaw) (matches rel_to_abs() convention)
            fwd_dx = -math.sin(yaw_rad)
            fwd_dz = -math.cos(yaw_rad)
            
            if entity_list:
                for entity in entity_list:
                    if not entity:
                        continue
                    
                    # Extract entity info
                    entity_type = None
                    entity_pos = None
                    distance = None
                    
                    # Try different entity object formats
                    if isinstance(entity, dict):
                        entity_type = entity.get('type') or entity.get('name')
                        pos = entity.get('position') or entity.get('pos')
                        if isinstance(pos, dict):
                            entity_pos = (pos.get('x'), pos.get('y'), pos.get('z'))
                        elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                            entity_pos = tuple(pos[:3])
                        distance = entity.get('distance')
                    elif hasattr(entity, 'type'):
                        entity_type = entity.type
                        if hasattr(entity, 'position'):
                            pos = entity.position
                            if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                                entity_pos = tuple(pos[:3])
                        if hasattr(entity, 'distance'):
                            distance = entity.distance
                    
                    # Filter by type if requested
                    if entity_filter == "items":
                        if not entity_type or "item" not in str(entity_type).lower():
                            continue
                    
                    # Calculate distance if not provided
                    if entity_pos:
                        ex, ey, ez = entity_pos
                        dx, dy, dz = ex - px, ey - py, ez - pz
                        curr_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                    else:
                        continue
                        
                    # 1. Distance Check
                    if curr_dist > radius:
                        continue
                        
                    # 2. Cone Check
                    # Vector to entity
                    if curr_dist < 0.1:
                        # Too close, always visible
                        pass
                    else:
                        vx, vy, vz = dx/curr_dist, dy/curr_dist, dz/curr_dist
                        
                        # Dot product with forward vector (horizontal) for yaw
                        # Project v onto xz plane
                        v_horiz_dist = math.sqrt(vx*vx + vz*vz)
                        if v_horiz_dist > 0.001:
                            vx_h, vz_h = vx/v_horiz_dist, vz/v_horiz_dist
                            dot_h = vx_h * fwd_dx + vz_h * fwd_dz
                            # cos(60) = 0.5. If dot < 0.5, outside 120 deg cone
                            if dot_h < 0.5:
                                continue
                                
                        # Pitch check: angle from horizon, allow up 60° and down 90°
                        dist_h = math.sqrt(dx*dx + dz*dz)
                        pitch_deg = math.degrees(math.atan2(-dy, dist_h))
                        if pitch_deg < -60.0 or pitch_deg > 90.0:
                            continue

                    # 3. Occlusion Check
                    eye_y_pos = py + 1.6
                    if not is_position_visible(px, eye_y_pos, pz, int(ex), int(ey), int(ez)):
                         continue

                    # Entity is valid
                    entity_data = {
                        "name": str(entity_type) if entity_type else "unknown",
                        "type": str(entity_type) if entity_type else "unknown",
                        "distance": round(curr_dist, 2),
                        "dx": round(dx, 2),
                        "dy": round(dy, 2),
                        "dz": round(dz, 2),
                    }
                    
                    if entity_pos:
                        entity_data["position"] = {"x": entity_pos[0], "y": entity_pos[1], "z": entity_pos[2]}
                    
                    # Extract age and velocity metadata (KISS: most likely + one fallback)
                    age_ticks = None
                    velocity = None
                    age_extracted = False
                    velocity_extracted = False
                    
                    # Try most likely: dict access
                    if isinstance(entity, dict):
                        age_ticks = entity.get('age') or entity.get('Age') or entity.get('age_ticks')
                        if age_ticks is not None:
                            age_extracted = True
                        
                        motion = entity.get('motion') or entity.get('Motion') or entity.get('velocity')
                        if motion:
                            if isinstance(motion, (list, tuple)) and len(motion) >= 3:
                                velocity = {'x': float(motion[0]), 'y': float(motion[1]), 'z': float(motion[2])}
                                velocity_extracted = True
                            elif isinstance(motion, dict):
                                velocity = {'x': float(motion.get('x', 0)), 'y': float(motion.get('y', 0)), 'z': float(motion.get('z', 0))}
                                velocity_extracted = True
                    # Fallback: object attribute access
                    elif hasattr(entity, 'age'):
                        try:
                            age_ticks = entity.age
                            if age_ticks is not None:
                                age_extracted = True
                        except Exception:
                            pass
                    elif hasattr(entity, 'getAge'):
                        try:
                            age_ticks = entity.getAge()
                            if age_ticks is not None:
                                age_extracted = True
                        except Exception:
                            pass
                    
                    if not velocity_extracted and hasattr(entity, 'motion'):
                        try:
                            motion = entity.motion
                            if isinstance(motion, (list, tuple)) and len(motion) >= 3:
                                velocity = {'x': float(motion[0]), 'y': float(motion[1]), 'z': float(motion[2])}
                                velocity_extracted = True
                        except Exception:
                            pass
                    elif not velocity_extracted and hasattr(entity, 'getMotion'):
                        try:
                            motion = entity.getMotion()
                            if motion:
                                if isinstance(motion, (list, tuple)) and len(motion) >= 3:
                                    velocity = {'x': float(motion[0]), 'y': float(motion[1]), 'z': float(motion[2])}
                                    velocity_extracted = True
                                elif hasattr(motion, 'x'):
                                    velocity = {'x': float(motion.x), 'y': float(motion.y), 'z': float(motion.z)}
                                    velocity_extracted = True
                        except Exception:
                            pass
                    
                    # Log extraction results (only for first entity to minimize noise)
                    if len(entities) == 0:
                        if age_extracted:
                            print(f"[Bridge] scan_entities: ✓ age extraction succeeded (age_ticks={age_ticks})", flush=True)
                        else:
                            print(f"[Bridge] scan_entities: ✗ age extraction failed (entity_type={entity_type})", flush=True)
                        
                        if velocity_extracted:
                            print(f"[Bridge] scan_entities: ✓ velocity extraction succeeded (v={velocity})", flush=True)
                        else:
                            print(f"[Bridge] scan_entities: ✗ velocity extraction failed (entity_type={entity_type})", flush=True)
                    
                    # Add metadata to entity_data if extracted
                    if age_ticks is not None:
                        entity_data["age_ticks"] = int(age_ticks)
                        # For items: compute time until despawn (6000 ticks = 5 minutes)
                        if entity_type and "item" in str(entity_type).lower():
                            time_until_despawn_ticks = max(0, 6000 - age_ticks)
                            entity_data["time_until_despawn_ticks"] = time_until_despawn_ticks
                            entity_data["time_until_despawn_seconds"] = round(time_until_despawn_ticks / 20.0, 1)
                    
                    if velocity:
                        entity_data["velocity"] = velocity
                        # Compute is_stationary (velocity magnitude < 0.01)
                        v_mag = math.sqrt(velocity['x']**2 + velocity['y']**2 + velocity['z']**2)
                        entity_data["is_stationary"] = v_mag < 0.01
                    
                    # For item entities, extract item name and count
                    if entity_filter == "items" or (entity_type and "item" in str(entity_type).lower()):
                        item_name = None
                        item_count = 1
                        
                        # Try to get item name from entity
                        if isinstance(entity, dict):
                            item_name = entity.get('item_name') or entity.get('item')
                            item_count = entity.get('count', entity.get('item_count', 1))
                        elif hasattr(entity, 'item'):
                            item = entity.item
                            if isinstance(item, str):
                                item_name = item
                            elif hasattr(item, 'name'):
                                item_name = item.name
                            if hasattr(entity, 'count'):
                                item_count = entity.count
                        
                        if item_name:
                            entity_data["item_name"] = item_name
                            entity_data["item_count"] = int(item_count) if item_count else 1
                            # Align with block convention: prefer item name as display name
                            entity_data["name"] = item_name
                    
                    entities.append(entity_data)
    except Exception as e:
        print(f"[Bridge] scan_entities: failed to scan entities: {e}")
    
    return entities

# ------------------------------------------------------------------------------
# Action Handlers
# ------------------------------------------------------------------------------

def handle_look(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Set player orientation using minescript API.
    
    According to minescript.net/docs:
    - API: player_set_orientation(yaw: float, pitch: float) -> bool
    - Parameters are in DEGREES (not radians)
    - yaw: degrees rotation around y-axis
    - pitch: degrees rotation from x-z plane
    
    Tool now sends degrees directly (standardized API).
    """
    try:
        # Tool sends degrees directly (standardized API)
        yaw_deg = float(body.get("yaw", 0))
        pitch_deg = float(body.get("pitch", 0))
        # Clamp pitch to [-90°, 90°]
        pitch_deg = max(-90.0, min(90.0, pitch_deg))
        
        # Use authoritative API: player_set_orientation(yaw: float, pitch: float) -> bool
        try:
            result = m.player_set_orientation(yaw_deg, pitch_deg)
            if result is False:
                return {"ok": False, "error": "player_set_orientation returned False"}
            # Flush orientation command to ensure it's applied immediately
            try:
                m.flush()
            except:
                pass
        except AttributeError:
            return {"ok": False, "error": "player_set_orientation API not available"}
        
        log_action("look", f"yaw={yaw_deg:.1f}°, pitch={pitch_deg:.1f}°")
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": f"Failed to set rotation: {e}"}

def handle_snapto(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Snap player to exact position and orientation.
    
    Body: {"x": float, "y": float, "z": float, "yaw": float (optional), "pitch": float (optional)}
    All parameters are in degrees (for yaw/pitch) and world coordinates (for x/y/z).
    If yaw or pitch are not provided, current player orientation is preserved.
    """
    try:
        x = float(body.get("x", 0))
        y = float(body.get("y", 0))
        z = float(body.get("z", 0))
        
        # Preserve current orientation if not provided
        if "yaw" in body:
            yaw = float(body["yaw"])
        else:
            current_yaw, _ = get_player_rot()
            yaw = current_yaw
        
        if "pitch" in body:
            pitch = float(body["pitch"])
        else:
            _, current_pitch = get_player_rot()
            pitch = current_pitch
        
        # Clamp pitch to [-90°, 90°]
        pitch = max(-90.0, min(90.0, pitch))
        
        # Try Minescript 4.0 APIs first
        try:
            # Try player_set_position or player_teleport
            if hasattr(m, 'player_set_position'):
                result = m.player_set_position(x, y, z)
                if result is False:
                    return {"ok": False, "error": "player_set_position returned False"}
            elif hasattr(m, 'player_teleport'):
                result = m.player_teleport(x, y, z)
                if result is False:
                    return {"ok": False, "error": "player_teleport returned False"}
            else:
                # Fallback: use execute command
                m.execute(f"tp @s {x} {y} {z} {yaw} {pitch}")
            
            # Set orientation
            result = m.player_set_orientation(yaw, pitch)
            if result is False:
                return {"ok": False, "error": "player_set_orientation returned False"}
            
            # Flush to ensure commands are applied
            try:
                m.flush()
            except:
                pass
        except AttributeError as e:
            # Fallback: use execute command if APIs not available
            try:
                m.execute(f"tp @s {x} {y} {z} {yaw} {pitch}")
                m.flush()
            except Exception as e2:
                return {"ok": False, "error": f"Minescript APIs not available and execute failed: {e2}"}
        except Exception as e:
            return {"ok": False, "error": f"Failed to snap: {e}"}
        
        log_action("snapto", f"pos=({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.1f}° pitch={pitch:.1f}°")
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": f"Failed to snap: {e}"}

def handle_move(body: Dict[str, Any]) -> Dict[str, Any]:
    """Handle movement command using minescript v4.0 player_press_* APIs."""
    try:
        
        duration_s = max(0.05, min(10.0, body.get("duration_ms", 400) / 1000.0))
        
        # Capture initial position before movement
        initial_pos = get_player_pos()
        initial_x, initial_y, initial_z = initial_pos
        
        # Map request keys to minescript v4.0 player_press_* functions
        key_map = {
            "forward": m.player_press_forward,
            "back": m.player_press_backward,
            "left": m.player_press_left,
            "right": m.player_press_right,
            "jump": m.player_press_jump,
            "sprint": m.player_press_sprint,
            "sneak": m.player_press_sneak,
        }
        
        # Find which keys are pressed
        pressed_keys = [k for k in key_map.keys() if body.get(k)]
        
        if not pressed_keys:
            return {"ok": False, "error": "No movement keys specified"}
        
        # Determine movement intent
        wants_horizontal = any(k in pressed_keys for k in ["forward", "back", "left", "right"])
        wants_jump = "jump" in pressed_keys
        
        # Press all requested keys
        for k in pressed_keys:
            try:
                key_map[k](True)
            except AttributeError:
                return {"ok": False, "error": f"player_press_{k} API not available"}
        
        try:
            m.flush()
        except:
            pass
            
        log_action("move", f"keys={pressed_keys}, ms={int(duration_s*1000)}")
        
        time.sleep(duration_s)
        
        # Release all pressed keys
        for k in pressed_keys:
            try:
                key_map[k](False)
            except:
                pass  # Continue even if key release fails
        
        try:
            m.flush()
        except:
            pass
        
        # Wait for position to stabilize (handles falling, jumping, climbing)
        # Check if Y-coordinate is changing significantly (falling/rising)
        stable_pos = get_player_pos()
        for attempt in range(20):  # Max 2 seconds (20 * 0.1s)
            time.sleep(0.1)
            new_pos = get_player_pos()
            
            # Check if Y stabilized (not falling/rising)
            if abs(stable_pos[1] - new_pos[1]) < 0.05:  # < 5cm change
                stable_pos = new_pos
                break
            
            stable_pos = new_pos  # Update for next check
        
        final_pos = stable_pos
        final_x, final_y, final_z = final_pos
        
        # Validate movement success
        status = "success"
        
        # Check horizontal movement (collision detection)
        if wants_horizontal:
            horizontal_delta = math.sqrt((final_x - initial_x)**2 + (final_z - initial_z)**2)
            if horizontal_delta < 0.3:  # Less than 0.3 blocks moved
                status = "collision"
        
        # Check vertical movement (fall detection)
        if wants_jump:
            if final_y < initial_y - 0.5:  # Fell more than 0.5 blocks
                status = "fell"
        elif final_y < initial_y - 0.5:  # Unexpected fall without jump
            status = "fell"
        
        return {"ok": True, "status": status, "final_position": final_pos}
    except Exception as e:
        return {"ok": False, "error": f"Failed to execute movement: {e}"}

def handle_inventory() -> Dict[str, Any]:
    """
    Get inventory contents and equipped items.
    Returns: {"ok": True, "slots": [...], "equipped": {"hand": "...", "offhand": "..."}}
    """
    try:
        slots = []
        equipped = {"hand": None, "offhand": None}
        
        # Get equipped items
        try:
            hand_items = m.player_hand_items()
            if hand_items:
                if hand_items.main_hand and hand_items.main_hand.item:
                    equipped["hand"] = hand_items.main_hand.item
                if hand_items.off_hand and hand_items.off_hand.item:
                    equipped["offhand"] = hand_items.off_hand.item
        except Exception as e:
            print(f"[Bridge] handle_inventory: failed to get hand items: {e}")
        
        # Try to get inventory slots (may not be available in all Minescript versions)
        try:
            # Try common inventory APIs
            inv_func = getattr(m, 'player_inventory', None) or getattr(m, 'inventory', None)
            if inv_func:
                inv = inv_func()
                if inv:
                    # Handle different inventory formats
                    if isinstance(inv, list):
                        for i, item in enumerate(inv):
                            if item and item != "air":
                                item_name = item if isinstance(item, str) else getattr(item, 'item', getattr(item, 'name', str(item)))
                                count = getattr(item, 'count', 1) if not isinstance(item, str) else 1
                                slots.append({"slot": i, "item": item_name, "count": count})
                    elif hasattr(inv, 'slots'):
                        for i, slot in enumerate(inv.slots):
                            if slot and slot.item:
                                slots.append({"slot": i, "item": slot.item, "count": getattr(slot, 'count', 1)})
        except Exception as e:
            print(f"[Bridge] handle_inventory: failed to get inventory slots: {e}")
        
        return {
            "ok": True,
            "slots": slots,
            "equipped": equipped
        }
    except Exception as e:
        return {"ok": False, "error": f"Failed to get inventory: {e}"}

def handle_say(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Send chat message - try Minescript chat API first, fallback to execute().
    """
    try:
        message = body.get("message", "")
        if not isinstance(message, str) or not message.strip():
            return {"ok": False, "error": "message text required"}
        
        # Try direct chat API first (most reliable)
        chat_func = None
        for api_name in ['chat', 'send_chat', 'player_chat']:
            try:
                chat_func = getattr(m, api_name, None)
                if chat_func and callable(chat_func):
                    chat_func(message)
                    log_action("say", message)
                    return {"ok": True}
            except Exception:
                continue
        
        # Fallback to execute() command
        try:
            execute_func = getattr(m, 'execute', None)
            if not execute_func:
                return {"ok": False, "error": "chat API not available"}
            
            # Try without quotes first (simpler format)
            cmd = f"say {message}"
            execute_func(cmd)
            
            log_action("say", message)
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": f"Failed to send message: {e}"}
    except Exception as e:
        return {"ok": False, "error": f"Unexpected error in handle_say: {e}"}

def handle_equip(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Equip an item from inventory into hand or offhand.
    Uses Minescript inventory APIs or execute() command.
    """
    try:
        item = body.get("item", "")
        if not isinstance(item, str) or not item.strip():
            return {"ok": False, "error": "item name required"}
        
        slot = body.get("slot", "hand")
        if slot not in ["hand", "offhand"]:
            return {"ok": False, "error": "slot must be 'hand' or 'offhand'"}
        
        # Try Minescript equip APIs first
        equip_func = None
        for api_name in ['player_select_item', 'select_item', 'equip_item']:
            try:
                equip_func = getattr(m, api_name, None)
                if equip_func and callable(equip_func):
                    # Try calling with item name and slot
                    try:
                        equip_func(item, slot)
                        log_action("equip", f"{item} in {slot}")
                        return {"ok": True, "equipped": item, "slot": slot}
                    except TypeError:
                        # Try with just item name
                        try:
                            equip_func(item)
                            log_action("equip", f"{item} in {slot}")
                            return {"ok": True, "equipped": item, "slot": slot}
                        except Exception:
                            pass
            except Exception:
                continue
        
        # Fallback: Try using execute() with /item command
        try:
            execute_func = getattr(m, 'execute', None)
            if not execute_func:
                return {"ok": False, "error": "equip API not available"}
            
            # Try /item command (Minecraft 1.20+)
            # Format: /item replace entity @s <slot> with <item>
            # Slot names: weapon.mainhand, weapon.offhand
            slot_map = {"hand": "weapon.mainhand", "offhand": "weapon.offhand"}
            mc_slot = slot_map.get(slot, "weapon.mainhand")
            block_name = item if ":" in item else f"minecraft:{item}"
            cmd = f"item replace entity @s {mc_slot} with {block_name}"
            execute_func(cmd)
            
            log_action("equip", f"{item} in {slot}")
            return {"ok": True, "equipped": item, "slot": slot}
        except Exception as e:
            return {"ok": False, "error": f"Failed to equip item: {e}"}
    except Exception as e:
        return {"ok": False, "error": f"Unexpected error in handle_equip: {e}"}

def handle_stop(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Stop all movement - release all movement keys.
    """
    try:
        # Release all movement keys
        key_map = {
            "forward": m.player_press_forward,
            "back": m.player_press_backward,
            "left": m.player_press_left,
            "right": m.player_press_right,
            "jump": m.player_press_jump,
            "sprint": m.player_press_sprint,
            "sneak": m.player_press_sneak,
        }
        
        for key_name, key_func in key_map.items():
            try:
                key_func(False)
            except Exception:
                pass
        
        try:
            m.flush()
        except:
            pass
        
        log_action("stop", "all movement cancelled")
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": f"Failed to stop: {e}"}

def handle_respawn(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Respawn player after death using execute() command.
    """
    try:
        execute_func = getattr(m, 'execute', None)
        if not execute_func:
            return {"ok": False, "error": "minescript.execute() API not available"}
        
        cmd = "respawn"
        execute_func(cmd)
        
        log_action("respawn", "player respawned")
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": f"Failed to respawn: {e}"}

def handle_drop(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Drop items from inventory - try Minescript drop APIs first, fallback to key simulation.
    """
    try:
        item = body.get("item", "")
        if not isinstance(item, str) or not item.strip():
            return {"ok": False, "error": "item name required"}
        
        count = body.get("count")
        if count is not None:
            try:
                count = int(count)
                if count < 1:
                    return {"ok": False, "error": "count must be >= 1"}
            except (ValueError, TypeError):
                count = None
        
        scatter = body.get("scatter", False)
        
        block_name = item if ":" in item else f"minecraft:{item}"
        
        # Try Minescript drop APIs first
        drop_func = None
        for api_name in ['player_drop', 'drop_item', 'drop']:
            try:
                drop_func = getattr(m, api_name, None)
                if drop_func and callable(drop_func):
                    # Try calling with item name and count
                    try:
                        if count:
                            drop_func(item, count)
                        else:
                            drop_func(item)
                        log_action("drop", f"{block_name} x{count or 'all'}")
                        return {
                            "ok": True,
                            "dropped": [{"item": block_name, "count": count or 1}]
                        }
                    except TypeError:
                        # Try with just item name
                        try:
                            drop_func(item)
                            log_action("drop", f"{block_name} x{count or 'all'}")
                            return {
                                "ok": True,
                                "dropped": [{"item": block_name, "count": count or 1}]
                            }
                        except Exception:
                            pass
            except Exception:
                continue
        
        # Fallback: Simulate drop key press (Q key by default in Minecraft)
        # This drops whatever is in the main hand
        try:
            drop_key = getattr(m, 'player_press_drop', None) or getattr(m, 'key_down', None)
            if drop_key:
                # Press and release drop key
                drop_key(True)
                try:
                    m.flush()
                except:
                    pass
                time.sleep(0.1)  # Brief hold
                drop_key(False)
                try:
                    m.flush()
                except:
                    pass
                
                log_action("drop", f"{block_name} x{count or 1}")
                return {
                    "ok": True,
                    "dropped": [{"item": block_name, "count": count or 1}]
                }
        except Exception:
            pass
        
        return {"ok": False, "error": "drop API not available - no Minescript drop functions found"}
    except Exception as e:
        return {"ok": False, "error": f"Unexpected error in handle_drop: {e}"}

def handle_place(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Place a block using execute() command.
    Similar to dig but places instead of removes.
    """
    try:
        item = body.get("item", "")
        if not isinstance(item, str) or not item.strip():
            return {"ok": False, "error": "item/block name required"}
        
        ref = body.get("ref", {})
        if not ref:
            return {"ok": False, "error": "ref (reference block position) required"}
        
        face = body.get("face", "")
        if not face:
            return {"ok": False, "error": "face required (up, down, north, south, east, west)"}
        
        # Resolve reference block coordinates
        if "pos" in ref:
            ref_x, ref_y, ref_z = ref["pos"]["x"], ref["pos"]["y"], ref["pos"]["z"]
        elif "rel" in ref:
            ref_x, ref_y, ref_z = rel_to_abs(ref["rel"])
        else:
            return {"ok": False, "error": "ref must have 'pos' or 'rel'"}
        
        ref_x, ref_y, ref_z = int(ref_x), int(ref_y), int(ref_z)
        
        # Calculate placement position based on face
        # Face determines which side of the reference block to place on
        face_map = {
            "up": (0, 1, 0),
            "down": (0, -1, 0),
            "north": (0, 0, -1),  # -Z
            "south": (0, 0, 1),   # +Z
            "east": (1, 0, 0),    # +X
            "west": (-1, 0, 0),   # -X
        }
        
        offset = face_map.get(face.lower())
        if not offset:
            return {"ok": False, "error": f"invalid face '{face}' (must be: up, down, north, south, east, west)"}
        
        place_x, place_y, place_z = ref_x + offset[0], ref_y + offset[1], ref_z + offset[2]
        
        # Use execute() to place block
        try:
            execute_func = getattr(m, 'execute', None)
            if not execute_func:
                return {"ok": False, "error": "minescript.execute() API not available"}
            
            # Normalize block name (add minecraft: prefix if needed)
            block_name = item if ":" in item else f"minecraft:{item}"
            cmd = f"setblock {place_x} {place_y} {place_z} {block_name}"
            execute_func(cmd)
            
            log_action("place", f"{block_name} at {place_x},{place_y},{place_z} (face={face})")
            return {
                "ok": True,
                "placed": {
                    "name": block_name,
                    "position": {"x": place_x, "y": place_y, "z": place_z},
                    "face": face
                }
            }
        except Exception as e:
            return {"ok": False, "error": f"Failed to execute place command: {e}"}
    except Exception as e:
        return {"ok": False, "error": f"Unexpected error in handle_place: {e}"}

def handle_dig(body: Dict[str, Any]) -> Dict[str, Any]:
    """
    Dig a block using the simple execute() command approach.
    Uses: execute("setblock {x} {y} {z} air destroy"), then synchronously waits for completion.
    """
    try:
        # Resolve target coordinates
        if "pos" in body:
            tx, ty, tz = body["pos"]["x"], body["pos"]["y"], body["pos"]["z"]
        elif "rel" in body:
            tx, ty, tz = rel_to_abs(body["rel"])
        else:
            return {"ok": False, "error": "Missing 'pos' or 'rel'"}
        
        tx, ty, tz = int(tx), int(ty), int(tz)
        
        # Get block name before digging (for feedback)
        block_name = "unknown"
        try:
            block = m.getblock(tx, ty, tz)
            if isinstance(block, str):
                block_name = block
            elif block:
                block_name = getattr(block, 'name', str(block))
        except:
            try:
                block = m.get_block(tx, ty, tz)
                if isinstance(block, str):
                    block_name = block
                elif block:
                    block_name = getattr(block, 'name', str(block))
            except:
                pass
        
        # Check if already air
        if block_name.lower() in ('air', 'minecraft:air', ''):
            log_action("already_air", f"block is air at {tx},{ty},{tz}")
            return {"ok": True, "dug": {"name": "air", "position": {"x": tx, "y": ty, "z": tz}}}
        
        # Simple dig: execute setblock command.
        # The 'destroy' parameter makes it drop items like normal mining.
        #
        # This endpoint is synchronous: it waits until the block reads as air (or timeout).
        try:
            execute_func = getattr(m, 'execute', None)
            if not execute_func:
                return {"ok": False, "error": "minescript.execute() API not available"}
            
            cmd = f"setblock {tx} {ty} {tz} air destroy"
            execute_func(cmd)
            # Flush immediately so client state updates promptly.
            try:
                m.flush()
            except Exception:
                pass
            
            timeout_s = float(body.get("timeout_s", 5.0))
            timeout_s = max(0.1, min(60.0, timeout_s))
            poll_s = float(body.get("poll_s", 0.1))
            poll_s = max(0.05, min(0.5, poll_s))

            t0 = time.time()
            completed = False
            last_read = None
            while (time.time() - t0) < timeout_s:
                try:
                    blk = m.getblock(tx, ty, tz)
                except Exception:
                    try:
                        blk = m.get_block(tx, ty, tz)
                    except Exception:
                        blk = None

                if isinstance(blk, str):
                    last_read = blk
                    lower = blk.lower()
                    if lower in ("air", "minecraft:air", "") or "air" in lower:
                        completed = True
                        break
                elif blk:
                    # object-ish; treat is_air if available
                    if hasattr(blk, "is_air") and blk.is_air():
                        completed = True
                        last_read = "air"
                        break
                    last_read = getattr(blk, "name", str(blk))
                else:
                    last_read = None

                time.sleep(poll_s)

            elapsed_ms = int((time.time() - t0) * 1000)

            # Best-effort drop detection: scan nearby item entities and return ones close to the dug block.
            drops = []
            try:
                nearby = scan_entities(radius=4, entity_filter="items")
                for ent in nearby or []:
                    pos = ent.get("position") or {}
                    ex, ey, ez = pos.get("x"), pos.get("y"), pos.get("z")
                    if ex is None or ey is None or ez is None:
                        continue
                    dx = float(ex) - float(tx)
                    dy = float(ey) - float(ty)
                    dz = float(ez) - float(tz)
                    # Within ~2 blocks of the broken block center
                    if (dx * dx + dy * dy + dz * dz) <= (2.0 * 2.0):
                        drops.append(ent)
            except Exception:
                drops = []

            if completed:
                log_action("dig_completed", f"{block_name} at {tx},{ty},{tz} ({elapsed_ms}ms)")
                return {
                    "ok": True,
                    "dug": {"name": block_name, "position": {"x": tx, "y": ty, "z": tz}},
                    "completed": True,
                    "elapsed_ms": elapsed_ms,
                    "drops": drops,
                }

            log_action("dig_timeout", f"{block_name} at {tx},{ty},{tz} ({elapsed_ms}ms)")
            return {
                "ok": False,
                "error": "dig_timeout",
                "dug": {"name": block_name, "position": {"x": tx, "y": ty, "z": tz}},
                "completed": False,
                "elapsed_ms": elapsed_ms,
                "last_block_read": last_read,
                "drops": drops,
            }
        except Exception as e:
            return {"ok": False, "error": f"Failed to execute dig command: {e}"}
    except Exception as e:
        return {"ok": False, "error": f"Unexpected error in handle_dig: {e}"}

# ------------------------------------------------------------------------------
# HTTP Server
# ------------------------------------------------------------------------------

class BridgeHandler(BaseHTTPRequestHandler):
    # Route dispatch table
    POST_ROUTES = {
        "/act/look": handle_look,
        "/act/move": handle_move,
        "/act/dig":  handle_dig,
        "/act/place": handle_place,
        "/act/equip": handle_equip,
        "/act/drop": handle_drop,
        "/act/stop": handle_stop,
        "/act/respawn": handle_respawn,
        "/act/snapto": handle_snapto,
        "/say": handle_say,
    }

    def _send_json(self, data: Dict[str, Any], status=200):
        try:
            body = json.dumps(data).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            # Client disconnected before we could send response - this is OK
            # Don't log as error, just ignore
            pass
        except Exception as e:
            # Other errors should be logged
            print(f"[Bridge] Error sending response: {e}")

    def do_GET(self):
        if self.path == "/status":
            self._send_json(get_status())
        elif self.path == "/inventory":
            self._send_json(handle_inventory())
        elif self.path.startswith("/ground-truth"):
            # Parse query string
            from urllib.parse import urlparse, parse_qs
            parsed = urlparse(self.path)
            query_params = parse_qs(parsed.query, keep_blank_values=True)
            center_x = int(query_params.get("center_x", ["0"])[0])
            center_y = int(query_params.get("center_y", ["0"])[0])
            center_z = int(query_params.get("center_z", ["0"])[0])
            radius = max(1, min(15, int(query_params.get("radius", ["12"])[0])))  # Cap at 15 for safety
            self._send_json(handle_ground_truth(center_x, center_y, center_z, radius))
        elif self.path.startswith("/observe"):
            # Parse query string from path
            from urllib.parse import urlparse, parse_qs
            observe_start = time.time()
            parsed = urlparse(self.path)
            # keep_blank_values=True preserves empty strings (entity_filter="" should scan all entities)
            query_params = parse_qs(parsed.query, keep_blank_values=True)
            radius = max(1, min(MAX_RADIUS, int(query_params.get("radius", query_params.get("entities_radius", ["9"]))[0])))
            # Empty string "" means "all entities", None means "skip entities"
            entity_filter = query_params.get("entity_filter", [None])[0]
            
            # Expected format: {"ok": True, "status": {...}, "perception": {"nearby_blocks": [...], "nearby_entities": [...], "visibility_distances": {...}}}
            # Skip compute_visibility_distances (too slow, typically quick compared to other queries)
            print(f"[Bridge] /observe: Starting observation (radius={radius}, entity_filter={entity_filter})", flush=True)

            # One-time diagnostic: can we enter minescript's high-speed script loop context from this thread?
            if not State.script_loop_checked:
                State.script_loop_checked = True
                try:
                    State.script_loop_available = bool(getattr(m, "script_loop", None))
                    print(f"[Bridge] diag: m.script_loop available={State.script_loop_available}", flush=True)
                    print(f"[Bridge] diag: thread={threading.current_thread().name} ident={threading.get_ident()}", flush=True)
                    if State.script_loop_available:
                        px0, py0, pz0 = get_player_pos()
                        tx, ty, tz = int(px0), int(py0), int(pz0)
                        try:
                            with m.script_loop:  # type: ignore[attr-defined]
                                _ = m.getblock(tx, ty, tz)
                            State.script_loop_enter_ok = True
                            print(f"[Bridge] diag: entered m.script_loop OK; getblock({tx},{ty},{tz}) succeeded", flush=True)
                        except Exception as e:
                            State.script_loop_enter_ok = False
                            State.script_loop_error = str(e)
                            print(f"[Bridge] diag: FAILED entering m.script_loop or getblock inside it: {e}", flush=True)
                except Exception as e:
                    State.script_loop_available = False
                    State.script_loop_enter_ok = False
                    State.script_loop_error = str(e)
                    print(f"[Bridge] diag: Exception while probing m.script_loop: {e}", flush=True)
            blocks_start = time.time()
            # Run scan_blocks inside script_loop when available to reduce context-switch overhead per getblock
            if getattr(State, "script_loop_available", False):
                try:
                    with m.script_loop:  # type: ignore[attr-defined]
                        nearby_blocks = scan_blocks(radius)
                except Exception as e:
                    print(f"[Bridge] /observe: WARNING failed to use m.script_loop for scan_blocks: {e}", flush=True)
                    nearby_blocks = scan_blocks(radius)
            else:
                nearby_blocks = scan_blocks(radius)
            blocks_elapsed = (time.time() - blocks_start) * 1000
            print(f"[Bridge] /observe: scan_blocks took {blocks_elapsed:.1f}ms, found {len(nearby_blocks)} blocks", flush=True)

            # Navigation surface samples (Option B): support_y / support_block per (x,z) cell
            nav_surface = getattr(scan_blocks, "_last_nav_surface", [])  # type: ignore[attr-defined]

            # Adjacent blocks around the player (for directional reasoning).
            # This is intentionally independent of the cone-filtered nearby_blocks list.
            adjacent_start = time.time()
            adjacent_cache: Dict[Tuple[int, int, int], Optional[str]] = {}

            def _get_block_name_at(bx: int, by: int, bz: int) -> Optional[str]:
                key = (bx, by, bz)
                if key in adjacent_cache:
                    return adjacent_cache[key]
                try:
                    try:
                        blk = m.getblock(bx, by, bz)
                    except (AttributeError, TypeError):
                        blk = m.get_block(bx, by, bz)
                except Exception:
                    adjacent_cache[key] = None
                    return None

                if isinstance(blk, str):
                    name = blk
                elif blk:
                    if hasattr(blk, 'is_air') and blk.is_air():
                        adjacent_cache[key] = None
                        return None
                    name = getattr(blk, 'name', str(blk))
                else:
                    adjacent_cache[key] = None
                    return None

                if not name:
                    adjacent_cache[key] = None
                    return None
                lower = name.lower()
                if lower in ('air', 'minecraft:air', '') or 'air' in lower:
                    adjacent_cache[key] = None
                    return None
                adjacent_cache[key] = name
                return name

            fwd_x, fwd_y, fwd_z = rel_to_abs({"forward": 1, "right": 0, "up": 0})
            back_x, back_y, back_z = rel_to_abs({"forward": -1, "right": 0, "up": 0})
            left_x, left_y, left_z = rel_to_abs({"forward": 0, "right": -1, "up": 0})
            right_x, right_y, right_z = rel_to_abs({"forward": 0, "right": 1, "up": 0})
            up_x, up_y, up_z = rel_to_abs({"forward": 0, "right": 0, "up": 1})
            down_x, down_y, down_z = rel_to_abs({"forward": 0, "right": 0, "up": -1})

            # Wrap adjacent reads in script_loop if available (lower overhead / fewer context switches)
            if getattr(State, "script_loop_available", False):
                try:
                    with m.script_loop:  # type: ignore[attr-defined]
                        adjacent_blocks = {
                            "fwd": _get_block_name_at(fwd_x, fwd_y, fwd_z),
                            "back": _get_block_name_at(back_x, back_y, back_z),
                            "left": _get_block_name_at(left_x, left_y, left_z),
                            "right": _get_block_name_at(right_x, right_y, right_z),
                            "up": _get_block_name_at(up_x, up_y, up_z),
                            "down": _get_block_name_at(down_x, down_y, down_z),
                        }
                except Exception as e:
                    print(f"[Bridge] /observe: WARNING failed to use m.script_loop for adjacent_blocks: {e}", flush=True)
                    adjacent_blocks = {
                        "fwd": _get_block_name_at(fwd_x, fwd_y, fwd_z),
                        "back": _get_block_name_at(back_x, back_y, back_z),
                        "left": _get_block_name_at(left_x, left_y, left_z),
                        "right": _get_block_name_at(right_x, right_y, right_z),
                        "up": _get_block_name_at(up_x, up_y, up_z),
                        "down": _get_block_name_at(down_x, down_y, down_z),
                    }
            else:
                adjacent_blocks = {
                    "fwd": _get_block_name_at(fwd_x, fwd_y, fwd_z),
                    "back": _get_block_name_at(back_x, back_y, back_z),
                    "left": _get_block_name_at(left_x, left_y, left_z),
                    "right": _get_block_name_at(right_x, right_y, right_z),
                    "up": _get_block_name_at(up_x, up_y, up_z),
                    "down": _get_block_name_at(down_x, down_y, down_z),
                }

            adjacent_elapsed = (time.time() - adjacent_start) * 1000
            print(f"[Bridge] /observe: adjacent_blocks took {adjacent_elapsed:.1f}ms", flush=True)
            
            # Scan entities if entity_filter is specified (optimization: skip if not needed)
            # entity_filter can be "items" (filter items only), "" (all entities), or None (skip)
            nearby_entities = []
            entities_elapsed = 0
            if entity_filter is not None:  # Scan if explicitly requested (including empty string for "all")
                entities_start = time.time()
                # Pass None to scan_entities if empty string (means "all entities")
                filter_value = None if entity_filter == "" else entity_filter
                nearby_entities = scan_entities(radius, filter_value)
                entities_elapsed = (time.time() - entities_start) * 1000
                print(f"[Bridge] /observe: scan_entities took {entities_elapsed:.1f}ms, found {len(nearby_entities)} entities", flush=True)
            else:
                print(f"[Bridge] /observe: scan_entities skipped (entity_filter=None)", flush=True)
            
            status_start = time.time()
            status = get_status()
            status_elapsed = (time.time() - status_start) * 1000
            
            payload_start = time.time()
            payload = {
                "ok": True,
                "status": status,
                "perception": {
                    "nearby_blocks": nearby_blocks,
                    "nearby_entities": nearby_entities,
                    "visibility_distances": {},  # Skipped for performance (was too slow)
                    "adjacent_blocks": adjacent_blocks,
                    "nav_surface": nav_surface,
                    "blocks_complete": True,
                    "blocks_elapsed_ms": int(blocks_elapsed),
                    "entities_complete": True,
                    "entities_elapsed_ms": int(entities_elapsed),
                }
            }
            payload_elapsed = (time.time() - payload_start) * 1000
            total_elapsed = (time.time() - observe_start) * 1000
            print(f"[Bridge] /observe: total time {total_elapsed:.1f}ms (blocks={blocks_elapsed:.1f}ms, adjacent={adjacent_elapsed:.1f}ms, entities={entities_elapsed:.1f}ms, status={status_elapsed:.1f}ms, payload={payload_elapsed:.1f}ms)", flush=True)
            self._send_json(payload)
        else:
            self._send_json({"error": "Not Found"}, 404)

    def do_POST(self):
        if self.path not in self.POST_ROUTES:
            self._send_json({"error": "Endpoint not found"}, 404)
            return

        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length).decode("utf-8")) if length > 0 else {}

        try:
            handler = self.POST_ROUTES[self.path]
            result = handler(body)
            self._send_json(result)
        except Exception as e:
            # Log exception for debugging
            import traceback
            error_msg = str(e)
            traceback_str = traceback.format_exc()
            print(f"[Bridge] Error in {self.path}: {error_msg}")
            print(f"[Bridge] Traceback: {traceback_str}")
            self._send_json({"error": error_msg}, 500)

# Global server reference for shutdown
_server_instance = None
_server_lock = threading.Lock()

def run_server():
    global _server_instance
    server = HTTPServer((HOST, PORT), BridgeHandler)
    
    with _server_lock:
        _server_instance = server
    
    print(f"[Bridge] Running on http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except OSError as e:
        if e.errno == 98:  # Address already in use
            print(f"[Bridge] Error: Port {PORT} is already in use.")
            print(f"[Bridge] To find and kill it: lsof -i :{PORT} or kill $(lsof -t -i:{PORT})")
        else:
            print(f"[Bridge] Error: {e}")
    except KeyboardInterrupt:
        print(f"[Bridge] Interrupted")
    finally:
        print(f"[Bridge] Shutting down server...")
        try:
            server.shutdown()  # Stop serve_forever()
        except Exception:
            pass
        try:
            server.server_close()  # Close the socket
        except Exception:
            pass
        with _server_lock:
            _server_instance = None
        print(f"[Bridge] Server stopped, port {PORT} released")

def shutdown_server():
    """Gracefully shutdown the server."""
    global _server_instance
    with _server_lock:
        if _server_instance:
            print(f"[Bridge] Initiating shutdown...")
            try:
                _server_instance.shutdown()
            except Exception as e:
                print(f"[Bridge] Error during shutdown: {e}")

if __name__ == "__main__":
    import signal
    
    def signal_handler(signum, frame):
        print(f"\n[Bridge] Received signal {signum}, shutting down...")
        shutdown_server()
        # Give server thread time to clean up
        time.sleep(0.5)
        exit(0)
    
    # Register signal handlers for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    t = threading.Thread(target=run_server, daemon=False)  # Non-daemon so we can join
    t.start()
    
    # Keep main thread alive for Minescript context
    try:
        t.join()  # Wait for server thread to finish
    except KeyboardInterrupt:
        shutdown_server()
        t.join(timeout=2)