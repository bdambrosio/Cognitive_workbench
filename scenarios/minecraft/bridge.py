import json
import time
import math
import threading
import itertools
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from typing import Dict, Any, Tuple, List, Optional
from http.server import BaseHTTPRequestHandler, HTTPServer

import minescript as m  # pyright: ignore[reportMissingImports]

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------

HOST = "127.0.0.1"
PORT = 3003
MAX_RADIUS = 6

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
    # dx = -sin(yaw) * fwd - cos(yaw) * right
    # dz =  cos(yaw) * fwd - sin(yaw) * right
    dx = -math.sin(yaw_rad) * fwd + math.cos(yaw_rad) * right
    dz =  math.cos(yaw_rad) * fwd + math.sin(yaw_rad) * right

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
                except (AttributeError, TypeError):
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
    return {
        'forward': raycast_direction(forward_dx, 0, forward_dz, radius),
        'back': raycast_direction(back_dx, 0, back_dz, radius),
        'left': raycast_direction(left_dx, 0, left_dz, radius),
        'right': raycast_direction(right_dx, 0, right_dz, radius),
        'up': raycast_direction(0, 1, 0, radius),
        'down': raycast_direction(0, -1, 0, radius),
    }

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
    """Scans blocks around the player efficiently with coarse angular grid occlusion culling."""
    radius = max(1, min(MAX_RADIUS, radius))
    px, py, pz = get_player_pos()
    eye_y = py + 1.6  # Eye position
    cx, cy, cz = map(int, (px, py, pz))
    blocks = []
    
    # Coarse angular grid: 16 azimuth bins × 8 elevation bins = 128 bins
    # Each bin tracks max visible distance (None = occluded)
    AZIMUTH_BINS = 16
    ELEVATION_BINS = 8
    angular_grid = {}  # (azimuth_bin, elevation_bin) -> max_visible_distance
    
    def pos_to_angular_bin(bx: int, by: int, bz: int) -> tuple:
        """Convert block position to angular grid bin coordinates."""
        # Vector from eye to block center
        dx = bx + 0.5 - px
        dy = by + 0.5 - eye_y
        dz = bz + 0.5 - pz
        
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 0.1:
            return (0, 0)  # Too close, use center bin
        
        # Azimuth: 0-360 degrees (0 = +Z, 90 = -X, 180 = -Z, 270 = +X)
        azimuth_rad = math.atan2(-dx, dz)  # Minecraft convention
        azimuth_deg = math.degrees(azimuth_rad)
        if azimuth_deg < 0:
            azimuth_deg += 360
        azimuth_bin = int((azimuth_deg / 360.0) * AZIMUTH_BINS) % AZIMUTH_BINS
        
        # Elevation: -90 to +90 degrees (-90 = down, 0 = horizontal, +90 = up)
        dist_horizontal = math.sqrt(dx*dx + dz*dz)
        elevation_rad = math.atan2(-dy, dist_horizontal)
        elevation_deg = math.degrees(elevation_rad)
        # Map -90..+90 to 0..ELEVATION_BINS-1
        elevation_bin = int(((elevation_deg + 90) / 180.0) * ELEVATION_BINS)
        elevation_bin = max(0, min(ELEVATION_BINS - 1, elevation_bin))
        
        return (azimuth_bin, elevation_bin)
    
    # Transparent blocks (don't occlude)
    transparent_blocks = {'air', 'glass', 'water', 'ice', 'leaves', 'slab', 'stairs', 
                          'fence', 'fence_gate', 'wall', 'ladder', 'vine', 'cobweb'}
    
    max_blocks = 200
    block_count = 0
    
    # Iterate by distance shells (Manhattan distance for simplicity)
    for shell_dist in range(1, radius + 1):
        if block_count >= max_blocks:
            break
        
        # Generate all positions at this Manhattan distance
        shell_positions = []
        for dx in range(-shell_dist, shell_dist + 1):
            for dy in range(-shell_dist, shell_dist + 1):
                for dz in range(-shell_dist, shell_dist + 1):
                    manhattan_dist = abs(dx) + abs(dy) + abs(dz)
                    if manhattan_dist == shell_dist:
                        shell_positions.append((dx, dy, dz))
        
        # Sort by priority: horizontal layer first (dy=0), then above/below
        shell_positions.sort(key=lambda p: (abs(p[1]), p[1]))
        
        for dx, dy, dz in shell_positions:
            if block_count >= max_blocks:
                break
            
            bx, by, bz = cx + dx, cy + dy, cz + dz
            
            # Get angular bin for this block
            azimuth_bin, elevation_bin = pos_to_angular_bin(bx, by, bz)
            bin_key = (azimuth_bin, elevation_bin)
            
            # Calculate Euclidean distance
            euclidean_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Check if this angular bin is occluded at this distance
            max_visible = angular_grid.get(bin_key)
            if max_visible is not None and euclidean_dist > max_visible:
                continue  # Occluded - skip
            
            # Check the block
            block_name = None
            try:
                try:
                    block = m.getblock(bx, by, bz)
                except (AttributeError, TypeError):
                    block = m.get_block(bx, by, bz)
            except Exception:
                continue
            
            if isinstance(block, str):
                block_name = block
            elif block:
                if hasattr(block, 'is_air') and block.is_air():
                    continue
                block_name = getattr(block, 'name', str(block))
            else:
                continue
            
            # Filter out air blocks
            if block_name and block_name.lower() not in ('air', 'minecraft:air', ''):
                blocks.append({
                    "name": block_name,
                    "position": {"x": bx, "y": by, "z": bz},
                    "dx": dx,
                    "dy": dy,
                    "dz": dz,
                })
                block_count += 1
                
                # If block is opaque, mark this angular bin as occluded beyond this distance
                block_name_lower = block_name.lower()
                if block_name_lower not in transparent_blocks and 'air' not in block_name_lower:
                    # Mark bin as occluded beyond this distance
                    angular_grid[bin_key] = euclidean_dist
            
    return blocks

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
                    if entity_pos and distance is None:
                        dx, dy, dz = entity_pos[0] - px, entity_pos[1] - py, entity_pos[2] - pz
                        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    
                    # Only include entities within radius
                    if distance and distance <= radius:
                        entity_data = {
                            "type": str(entity_type) if entity_type else "unknown",
                            "distance": round(distance, 2),
                        }
                        
                        if entity_pos:
                            entity_data["position"] = {"x": entity_pos[0], "y": entity_pos[1], "z": entity_pos[2]}
                        
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
    Uses: execute("setblock {x} {y} {z} air destroy")
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
        
        # Simple dig: execute setblock command
        # The 'destroy' parameter makes it drop items like normal mining
        try:
            execute_func = getattr(m, 'execute', None)
            if not execute_func:
                return {"ok": False, "error": "minescript.execute() API not available"}
            
            cmd = f"setblock {tx} {ty} {tz} air destroy"
            execute_func(cmd)
            
            log_action("dig_started", f"{block_name} at {tx},{ty},{tz}")
            return {
                "ok": True,
                "dug": {
                    "name": block_name,
                    "position": {"x": tx, "y": ty, "z": tz}
                }
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
        elif self.path.startswith("/observe"):
            # Parse query string from path
            from urllib.parse import urlparse, parse_qs
            observe_start = time.time()
            parsed = urlparse(self.path)
            query_params = parse_qs(parsed.query)
            radius = max(1, min(MAX_RADIUS, int(query_params.get("radius", query_params.get("entities_radius", ["5"]))[0])))
            entity_filter = query_params.get("entity_filter", [None])[0]  # "items" or None
            
            # Expected format: {"ok": True, "status": {...}, "perception": {"nearby_blocks": [...], "nearby_entities": [...], "visibility_distances": {...}}}
            # Compute visibility distances BEFORE scanning blocks (they're needed early)
            vis_start = time.time()
            visibility_distances = compute_visibility_distances(radius)
            vis_elapsed = (time.time() - vis_start) * 1000
            print(f"[Bridge] /observe: visibility_distances took {vis_elapsed:.1f}ms")
            
            blocks_start = time.time()
            nearby_blocks = scan_blocks(radius)
            blocks_elapsed = (time.time() - blocks_start) * 1000
            print(f"[Bridge] /observe: scan_blocks took {blocks_elapsed:.1f}ms, found {len(nearby_blocks)} blocks")
            
            # Only scan entities if entity_filter is specified (optimization)
            nearby_entities = []
            entities_elapsed = 0
            if entity_filter:
                entities_start = time.time()
                nearby_entities = scan_entities(radius, entity_filter)
                entities_elapsed = (time.time() - entities_start) * 1000
                print(f"[Bridge] /observe: scan_entities took {entities_elapsed:.1f}ms, found {len(nearby_entities)} entities")
            
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
                    "visibility_distances": visibility_distances,
                    "blocks_complete": True,
                    "blocks_elapsed_ms": int(blocks_elapsed),
                    "entities_complete": True,
                    "entities_elapsed_ms": int(entities_elapsed),
                }
            }
            total_elapsed = (time.time() - observe_start) * 1000
            print(f"[Bridge] /observe: total time {total_elapsed:.1f}ms (vis={vis_elapsed:.1f}ms, blocks={blocks_elapsed:.1f}ms, entities={entities_elapsed:.1f}ms, status={status_elapsed:.1f}ms)")
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