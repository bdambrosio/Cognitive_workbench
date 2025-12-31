"""
Minecraft observe-blocks tool.
Exhaustive enumeration of ALL visible non-air blocks within radius R.
Visibility = within radius R AND line-of-sight from bot's eye position.
Does NOT report items (use mc-observe-items for that).
"""

import logging
import os
import math
import requests
from typing import Any, Dict, List, Optional, Set, Tuple

logger = logging.getLogger(__name__)

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")

# Solid blocks that provide safe footing
SOLID_BLOCKS = {
    'stone', 'grass', 'dirt', 'cobblestone', 'planks', 'bedrock', 'sand', 'gravel',
    'wood', 'log', 'leaves', 'glass', 'wool', 'brick', 'clay', 'concrete', 'terracotta',
    'ice', 'packed_ice', 'snow', 'snow_block', 'mycelium', 'podzol', 'grass_path',
    'farmland', 'mossy_cobblestone', 'stone_brick', 'netherrack', 'soul_sand', 'end_stone'
}

# Hazardous blocks
HAZARD_BLOCKS = {'lava', 'fire', 'cactus', 'magma_block', 'soul_fire'}

# Fluid blocks
FLUID_BLOCKS = {'water', 'lava', 'flowing_water', 'flowing_lava'}


def rel_to_abs(position: Dict[str, float], yaw: float, forward: int, right: int, up: int) -> Tuple[int, int, int]:
    """
    Convert relative (egocentric) position to absolute block coordinates.
    
    Args:
        position: Bot position dict with x, y, z
        yaw: Bot yaw in radians
        forward: Forward offset (positive = forward)
        right: Right offset (positive = right)
        up: Up offset (positive = up)
    
    Returns:
        Tuple of (x, y, z) absolute block coordinates
    """
    px = position.get('x', 0)
    py = position.get('y', 0)
    pz = position.get('z', 0)
    
    # Convert to block coordinates (floor)
    bx = int(math.floor(px))
    by = int(math.floor(py))
    bz = int(math.floor(pz))
    
    # Calculate egocentric offsets
    # forward vector = (-sin(yaw), 0, -cos(yaw))
    # right vector   = (-cos(yaw), 0, sin(yaw))
    dx = -math.sin(yaw) * forward - math.cos(yaw) * right
    dz = -math.cos(yaw) * forward + math.sin(yaw) * right
    dy = up
    
    # Round to nearest block
    return (bx + int(round(dx)), by + int(round(dy)), bz + int(round(dz)))


def find_block_at(nearby_blocks: List[Dict], target_x: int, target_y: int, target_z: int) -> Optional[str]:
    """
    Find block type at specific absolute coordinates.
    
    Returns:
        Block name if found, None otherwise
    """
    for block in nearby_blocks:
        if not isinstance(block, dict):
            continue
        pos = block.get('position')
        if not pos:
            continue
        
        # Handle both dict and list position formats
        if isinstance(pos, dict):
            bx = int(math.floor(pos.get('x', 0)))
            by = int(math.floor(pos.get('y', 0)))
            bz = int(math.floor(pos.get('z', 0)))
        elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
            bx = int(math.floor(pos[0]))
            by = int(math.floor(pos[1]))
            bz = int(math.floor(pos[2]))
        else:
            continue
        
        if bx == target_x and by == target_y and bz == target_z:
            return block.get('name', 'unknown')
    
    return None


def is_solid(block_name: Optional[str]) -> bool:
    """Check if block is solid (provides safe footing)."""
    if not block_name or block_name == 'air':
        return False
    return block_name in SOLID_BLOCKS or not block_name.startswith(('air', 'water', 'lava'))


def compute_geometry(nearby_blocks: List[Dict], position: Dict[str, float], yaw: float) -> Dict[str, bool]:
    """
    Detect geometry patterns: pit, stair, slope.
    
    Returns:
        Dict with pit, stair, slope booleans
    """
    geom = {'pit': False, 'stair': False, 'slope': False}
    
    # Check for pit: significant Y drop nearby
    py = int(math.floor(position.get('y', 0)))
    for block in nearby_blocks[:50]:  # Sample nearby blocks
        pos = block.get('position')
        if not pos:
            continue
        if isinstance(pos, dict):
            by = int(math.floor(pos.get('y', 0)))
        elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
            by = int(math.floor(pos[1]))
        else:
            continue
        
        # If block is significantly below, might be a pit
        if by < py - 2:
            geom['pit'] = True
            break
    
    # Check for stair pattern: step-like elevation changes forward
    # Simple heuristic: check if forward positions show step pattern
    for fwd in [1, 2]:
        pos_here = rel_to_abs(position, yaw, fwd - 1, 0, 0)
        pos_fwd = rel_to_abs(position, yaw, fwd, 0, 0)
        block_here = find_block_at(nearby_blocks, *pos_here)
        block_fwd = find_block_at(nearby_blocks, *pos_fwd)
        
        if block_here and block_fwd:
            # Check if there's a step up
            pos_fwd_up = rel_to_abs(position, yaw, fwd, 0, 1)
            block_fwd_up = find_block_at(nearby_blocks, *pos_fwd_up)
            if block_fwd_up and is_solid(block_fwd_up):
                geom['stair'] = True
                break
    
    # Check for slope: gradual elevation change
    # Simple heuristic: check multiple forward positions for gradual Y change
    elevations = []
    for fwd in range(1, 4):
        pos = rel_to_abs(position, yaw, fwd, 0, 0)
        block = find_block_at(nearby_blocks, *pos)
        if block and is_solid(block):
            elevations.append(pos[1])
    
    if len(elevations) >= 2:
        elevation_diff = max(elevations) - min(elevations)
        if 1 <= elevation_diff <= 2:
            geom['slope'] = True
    
    return geom


def compute_affordances(
    visibility_distances: Dict[str, Optional[float]],
    support: Dict[str, Any],
    clear: Dict[str, Any],
    dirs: Dict[str, Any]
) -> Dict[str, bool]:
    """
    Compute affordances: step, jump, descend, sky.
    
    Returns:
        Dict with step, jump, descend, sky booleans
    """
    aff = {'step': False, 'jump': False, 'descend': False, 'sky': False}
    
    # step: can step forward (forward block exists and clearance available)
    if dirs.get('fwd', {}).get('dist') is not None:
        fwd_dist = dirs['fwd']['dist']
        if isinstance(fwd_dist, (int, float)) and fwd_dist >= 1.0:
            # Check if forward position has solid block and clearance
            if support.get('fwd', {}).get('type') == 'solid':
                if clear.get('fwd', {}).get('body') and clear.get('fwd', {}).get('head'):
                    aff['step'] = True
    
    # jump: can jump up (up clearance available)
    if clear.get('up', {}).get('body') and clear.get('up', {}).get('head'):
        aff['jump'] = True
    
    # descend: can descend safely (down distance safe)
    down_dist = dirs.get('down', {}).get('dist')
    if isinstance(down_dist, (int, float)) and down_dist <= 1.5:
        aff['descend'] = True
    
    # sky: sky visible above
    up_dist = dirs.get('up', {}).get('dist')
    if up_dist is None or (isinstance(up_dist, (int, float)) and up_dist > 10):
        aff['sky'] = True
    
    return aff


def compute_confidence(blocks_complete: bool, blocks_elapsed_ms: float, nearby_blocks_count: int) -> str:
    """
    Compute confidence level based on data completeness.
    
    Returns:
        'high', 'med', or 'low'
    """
    if not blocks_complete:
        return 'low'
    if blocks_elapsed_ms > 8.0:  # Near timeout
        return 'med'
    if nearby_blocks_count > 100:  # Many blocks might indicate complex environment
        return 'med'
    return 'high'


def tool(input_value=None, **kwargs):
    """
    Get exhaustive enumeration of ALL visible non-air blocks within radius R.
    Visibility = within radius R AND line-of-sight from bot's eye position.
    Does NOT report items (use mc-observe-items for that).
    
    Returns structured observation summary following enhanced schema.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        # New minescript API uses simple radius parameter
        radius = kwargs.get("blocks_radius") or kwargs.get("radius") or 5
        radius = max(1, min(6, int(radius)))  # Clamp to valid range
        
        params = {"radius": radius}
        url = f"{minecraft_url}/observe"
        logger.info(f"🔍 mc-observe-blocks: GET {url} params={params}")
        response = requests.get(url, params=params, timeout=10.0)
        logger.info(f"📥 mc-observe-blocks: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-observe-blocks: Response body keys={list(data.keys())}, blocks_count={len(data.get('perception', {}).get('nearby_blocks', []))}")
        
        status = data.get('status', {})
        perception = data.get('perception', {})
        
        # Extract position and orientation
        position = status.get('position', {})
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            position = {'x': position[0], 'y': position[1], 'z': position[2]}
        
        yaw = status.get('yaw', 0.0)
        pitch = status.get('pitch', 0.0)
        
        visibility_distances = perception.get('visibility_distances', {})
        nearby_blocks = perception.get('nearby_blocks', [])
        blocks_complete = perception.get('blocks_complete', True)
        blocks_elapsed_ms = perception.get('blocks_elapsed_ms', 0)
        radius = kwargs.get("blocks_radius") or kwargs.get("radius") or 4
        
        # Build structured summary
        summary_parts = []
        summary_parts.append("SUMMARY:")
        summary_parts.append("")
        
        # pose
        px = position.get('x', 0)
        py = position.get('y', 0)
        pz = position.get('z', 0)
        summary_parts.append(f"pose: ({px:.2f},{py:.2f},{pz:.2f}, yaw:{yaw:.2f}, pitch:{pitch:.2f})")
        summary_parts.append("")
        
        # dirs: directional visibility and blocks
        summary_parts.append("dirs:")
        dirs_info = {}
        for dir_name, dir_key in [('fwd', 'forward'), ('back', 'back'), ('left', 'left'), 
                                   ('right', 'right'), ('up', 'up'), ('down', 'down')]:
            dist = visibility_distances.get(dir_key)
            if dist is not None:
                dist_str = f"{dist:.1f}"
            else:
                dist_str = f">{radius}"
            
            # Find block at direction-specific positions
            # For forward/back/left/right: check at (dir:1, up:0) - the immediate adjacent block
            # For up/down: check at (forward:0, dir:1/-1)
            blk = None
            if dir_name == 'fwd':
                # Check forward block at (forward:1, up:0) - critical for mc-staircase
                pos = rel_to_abs(position, yaw, 1, 0, 0)
                blk = find_block_at(nearby_blocks, *pos)
            elif dir_name == 'back':
                pos = rel_to_abs(position, yaw, -1, 0, 0)
                blk = find_block_at(nearby_blocks, *pos)
            elif dir_name == 'left':
                pos = rel_to_abs(position, yaw, 0, -1, 0)
                blk = find_block_at(nearby_blocks, *pos)
            elif dir_name == 'right':
                pos = rel_to_abs(position, yaw, 0, 1, 0)
                blk = find_block_at(nearby_blocks, *pos)
            elif dir_name == 'up':
                pos = rel_to_abs(position, yaw, 0, 0, 1)
                blk = find_block_at(nearby_blocks, *pos)
            elif dir_name == 'down':
                pos = rel_to_abs(position, yaw, 0, 0, -1)
                blk = find_block_at(nearby_blocks, *pos)
            
            blk_str = blk if blk else "none"
            summary_parts.append(f"  {dir_name}:  dist: {dist_str},  blk: {blk_str}")
            dirs_info[dir_name] = {'dist': dist if dist is not None else None, 'blk': blk}
        
        summary_parts.append("")
        
        # support: footing information
        summary_parts.append("support:")
        support_info = {}
        
        # Current position footing (forward:0, up:-1)
        pos_here = rel_to_abs(position, yaw, 0, 0, -1)
        block_here = find_block_at(nearby_blocks, *pos_here)
        if block_here and is_solid(block_here):
            summary_parts.append(f"  here: solid({block_here})")
            support_info['here'] = {'type': 'solid', 'block': block_here}
        else:
            summary_parts.append(f"  here: unsafe")
            support_info['here'] = {'type': 'unsafe', 'block': block_here}
        
        # Forward position footing (forward:1, up:-1)
        pos_fwd = rel_to_abs(position, yaw, 1, 0, -1)
        block_fwd_footing = find_block_at(nearby_blocks, *pos_fwd)
        # Also get block at (forward:1, up:0) for explicit block type
        pos_fwd_block = rel_to_abs(position, yaw, 1, 0, 0)
        block_fwd_block = find_block_at(nearby_blocks, *pos_fwd_block)
        if block_fwd_footing and is_solid(block_fwd_footing):
            summary_parts.append(f"  fwd:  solid({block_fwd_footing})")
            support_info['fwd'] = {'type': 'solid', 'block': block_fwd_footing, 'forward_block': block_fwd_block}
        else:
            summary_parts.append(f"  fwd:  unsafe")
            support_info['fwd'] = {'type': 'unsafe', 'block': block_fwd_footing, 'forward_block': block_fwd_block}
        
        summary_parts.append("")
        
        # clear: clearance information
        summary_parts.append("clear:")
        clear_info = {}
        
        # Forward clearance
        pos_fwd_body = rel_to_abs(position, yaw, 1, 0, 1)
        pos_fwd_head = rel_to_abs(position, yaw, 1, 0, 2)
        block_fwd_body = find_block_at(nearby_blocks, *pos_fwd_body)
        block_fwd_head = find_block_at(nearby_blocks, *pos_fwd_head)
        fwd_body_clear = block_fwd_body is None or block_fwd_body == 'air'
        fwd_head_clear = block_fwd_head is None or block_fwd_head == 'air'
        
        clear_str = []
        if fwd_body_clear:
            clear_str.append("body")
        if fwd_head_clear:
            clear_str.append("head")
        summary_parts.append(f"  fwd: {'+'.join(clear_str) if clear_str else 'blocked'}")
        clear_info['fwd'] = {'body': fwd_body_clear, 'head': fwd_head_clear}
        
        # Up clearance
        pos_up_body = rel_to_abs(position, yaw, 0, 0, 1)
        pos_up_head = rel_to_abs(position, yaw, 0, 0, 2)
        block_up_body = find_block_at(nearby_blocks, *pos_up_body)
        block_up_head = find_block_at(nearby_blocks, *pos_up_head)
        up_body_clear = block_up_body is None or block_up_body == 'air'
        up_head_clear = block_up_head is None or block_up_head == 'air'
        
        clear_str = []
        if up_body_clear:
            clear_str.append("body")
        if up_head_clear:
            clear_str.append("head")
        summary_parts.append(f"  up:  {'+'.join(clear_str) if clear_str else 'blocked'}")
        clear_info['up'] = {'body': up_body_clear, 'head': up_head_clear}
        
        summary_parts.append("")
        
        # blocks: summary of seen block types
        summary_parts.append("blocks:")
        seen_blocks: Set[str] = set()
        fluid_blocks: Set[str] = set()
        hazard_blocks: Set[str] = set()
        
        for block in nearby_blocks:
            if not isinstance(block, dict):
                continue
            block_name = block.get('name', 'unknown')
            if block_name and block_name != 'air':
                seen_blocks.add(block_name)
                if block_name in FLUID_BLOCKS:
                    fluid_blocks.add(block_name)
                if block_name in HAZARD_BLOCKS:
                    hazard_blocks.add(block_name)
        
        summary_parts.append(f"  seen: {sorted(list(seen_blocks))}")
        summary_parts.append(f"  fluid: {sorted(list(fluid_blocks))}")
        summary_parts.append(f"  hazard: {sorted(list(hazard_blocks))}")
        summary_parts.append("")
        
        # geom: geometry detection
        geom = compute_geometry(nearby_blocks, position, yaw)
        summary_parts.append("geom:")
        summary_parts.append(f"  pit: {str(geom['pit']).lower()}")
        summary_parts.append(f"  stair: {str(geom['stair']).lower()}")
        summary_parts.append(f"  slope: {str(geom['slope']).lower()}")
        summary_parts.append("")
        
        # aff: affordances
        aff = compute_affordances(visibility_distances, support_info, clear_info, dirs_info)
        summary_parts.append("aff:")
        summary_parts.append(f"  step: {str(aff['step']).lower()}")
        summary_parts.append(f"  jump: {str(aff['jump']).lower()}")
        summary_parts.append(f"  descend: {str(aff['descend']).lower()}")
        summary_parts.append(f"  sky: {str(aff['sky']).lower()}")
        summary_parts.append("")
        
        # conf: confidence
        conf = compute_confidence(blocks_complete, blocks_elapsed_ms, len(nearby_blocks))
        summary_parts.append(f"conf: {conf}")
        
        # note: human-readable summary
        note_parts = []
        if aff['sky']:
            note_parts.append("Open sky visible")
        if geom['pit']:
            note_parts.append("pit detected")
        if geom['stair']:
            note_parts.append("stair-like geometry")
        if geom['slope']:
            note_parts.append("sloping terrain")
        if support_info['here']['type'] == 'solid':
            note_parts.append("solid footing")
        if len(seen_blocks) > 0:
            note_parts.append(f"{len(seen_blocks)} block types visible")
        if not note_parts:
            note_parts.append("Standard terrain")
        
        note = ", ".join(note_parts)
        summary_parts.append(f"note: \"{note}\"")
        
        summary_text = "\n".join(summary_parts)
        
        # Build structured data dict for mc-map-update (planner still sees text SUMMARY)
        structured_data = {
            "pose": {
                "x": px,
                "y": py,
                "z": pz,
                "yaw": yaw,
                "pitch": pitch
            },
            "dirs": dirs_info,
            "support": support_info,
            "clear": clear_info,
            "blocks": {
                "seen": sorted(list(seen_blocks)),
                "fluid": sorted(list(fluid_blocks)),
                "hazard": sorted(list(hazard_blocks))
            },
            "geom": geom,
            "aff": aff,
            "conf": conf,
            "note": note
        }
        
        return {
            "text": summary_text,  # Planner sees this
            "format": "text",
            "metadata": {
                **data,  # Include raw API response
                **structured_data  # Include structured fields for mc-map-update
            },
            "char_count": len(summary_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft observe request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)
