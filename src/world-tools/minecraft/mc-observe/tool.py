"""
Minecraft unified observe tool.
Fetches blocks, entities, and items in a single call to the bridge /observe endpoint.
Includes logging to report what data is returned and what is missing.
"""

import logging
import math
import os
import requests
from collections import defaultdict
from typing import Any, Dict, List, Optional, Set, Tuple

logger = logging.getLogger(__name__)

# Session-local rolling occupancy grid (agent-owned)
try:
    from local_grid import ingest_nearby_blocks, set_cell
except Exception:
    ingest_nearby_blocks = None  # optional; do not break mc-observe
    set_cell = None

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")

# Hazardous blocks
HAZARD_BLOCKS = {'lava', 'fire', 'cactus', 'magma_block', 'soul_fire', 'wither_rose', 'sweet_berry_bush'}

# Fluid blocks
FLUID_BLOCKS = {'water', 'lava', 'flowing_water', 'flowing_lava'}

# Non-solid blocks that do NOT provide safe footing (primary list)
NON_SOLID_BLOCKS = {
    'air', 'cave_air', 'void_air', 'water', 'lava', 'flowing_water', 'flowing_lava',
    'grass', 'tall_grass', 'fern', 'large_fern', 'dead_bush', 'seagrass', 'tall_seagrass',
    'kelp', 'kelp_plant', 'vine', 'lily_pad', 'sugar_cane', 'bamboo', 'bamboo_sapling',
    'dandelion', 'poppy', 'blue_orchid', 'allium', 'azure_bluet', 'red_tulip', 'orange_tulip',
    'white_tulip', 'pink_tulip', 'oxeye_daisy', 'cornflower', 'lily_of_the_valley', 'wither_rose',
    'sunflower', 'lilac', 'rose_bush', 'peony', 'torchflower', 'pitcher_plant',
    'wheat', 'carrots', 'potatoes', 'beetroots', 'melon_stem', 'pumpkin_stem', 'nether_wart', 'sweet_berry_bush', 'cocoa',
    'torch', 'wall_torch', 'redstone_torch', 'redstone_wall_torch', 'soul_torch', 'soul_wall_torch',
    'redstone_wire', 'lever', 'button', 'stone_button', 'oak_button', 'pressure_plate',
    'stone_pressure_plate', 'oak_pressure_plate', 'tripwire', 'tripwire_hook',
    'rail', 'powered_rail', 'detector_rail', 'activator_rail',
    'sign', 'wall_sign', 'oak_sign', 'oak_wall_sign', 'hanging_sign',
    'banner', 'wall_banner',
    'ladder', 'snow', 'carpet', 'fire', 'soul_fire', 'cobweb',
    'mushroom', 'red_mushroom', 'brown_mushroom',
    'sapling', 'oak_sapling', 'spruce_sapling', 'birch_sapling',
}


def categorize_entity(entity_type: str) -> str:
    """
    Categorize entity by type.
    Returns: 'item', 'mob', 'player', or 'other'
    """
    if not entity_type:
        return 'other'

    entity_type_lower = str(entity_type).lower()
    if 'item' in entity_type_lower:
        return 'item'
    if 'player' in entity_type_lower:
        return 'player'

    # Common mob types (heuristic)
    mob_indicators = [
        'zombie', 'skeleton', 'creeper', 'spider', 'cow', 'pig', 'chicken', 'sheep', 'wolf', 'villager',
        'enderman', 'slime', 'witch', 'guardian', 'shulker', 'phantom', 'ghast', 'blaze', 'wither', 'dragon',
        'horse', 'donkey', 'mule', 'llama', 'panda', 'fox', 'bee', 'dolphin', 'turtle', 'cat', 'ocelot', 'parrot',
        'rabbit', 'polar_bear', 'iron_golem', 'snow_golem', 'vex', 'evoker', 'vindicator', 'pillager', 'ravager',
        'hoglin', 'piglin', 'strider', 'zoglin'
    ]
    if any(indicator in entity_type_lower for indicator in mob_indicators):
        return 'mob'
    return 'other'


def rel_to_abs(position: Dict[str, float], yaw: float, forward: int, right: int, up: int) -> Tuple[int, int, int]:
    """
    Convert relative (egocentric) position to absolute block coordinates.
    Mirrors the older mc-observe-blocks helper.
    """
    px = position.get('x', 0.0)
    py = position.get('y', 0.0)
    pz = position.get('z', 0.0)

    bx = int(math.floor(px))
    by = int(math.floor(py))
    bz = int(math.floor(pz))

    dx = -math.sin(math.radians(yaw)) * forward - math.cos(math.radians(yaw)) * right
    dz = -math.cos(math.radians(yaw)) * forward + math.sin(math.radians(yaw)) * right
    dy = up

    return (bx + int(round(dx)), by + int(round(dy)), bz + int(round(dz)))


def find_block_at(nearby_blocks: List[Dict[str, Any]], target_x: int, target_y: int, target_z: int) -> Optional[str]:
    """Find block type at specific absolute coordinates from nearby_blocks list."""
    for block in nearby_blocks:
        if not isinstance(block, dict):
            continue
        pos = block.get('position')
        if not pos:
            continue

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
            name = block.get('name', 'unknown')
            return name if isinstance(name, str) else (str(name) if name is not None else None)
    return None


def is_solid(block_name: Optional[str]) -> bool:
    """Check if block is solid (provides safe footing)."""
    if not block_name:
        return False
    name = block_name.split(':')[-1] if ':' in block_name else block_name
    if name in NON_SOLID_BLOCKS:
        return False
    if any(name.endswith(suffix) for suffix in ('_torch', '_sign', '_button', '_pressure_plate', '_sapling', '_carpet')):
        return False
    return True


def compute_geometry(nearby_blocks: List[Dict[str, Any]], position: Dict[str, float], yaw: float) -> Dict[str, bool]:
    """Detect geometry patterns: pit, stair, slope (ported from old mc-observe-blocks)."""
    geom = {'pit': False, 'stair': False, 'slope': False}

    py = int(math.floor(position.get('y', 0)))
    for block in nearby_blocks[:50]:
        pos = block.get('position')
        if not pos:
            continue
        if isinstance(pos, dict):
            by = int(math.floor(pos.get('y', 0)))
        elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
            by = int(math.floor(pos[1]))
        else:
            continue
        if by < py - 2:
            geom['pit'] = True
            break

    for fwd in [1, 2]:
        pos_fwd = rel_to_abs(position, yaw, fwd, 0, 0)
        block_fwd = find_block_at(nearby_blocks, *pos_fwd)
        if block_fwd:
            pos_fwd_up = rel_to_abs(position, yaw, fwd, 0, 1)
            block_fwd_up = find_block_at(nearby_blocks, *pos_fwd_up)
            if block_fwd_up and is_solid(block_fwd_up):
                geom['stair'] = True
                break

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
    Adapted from old mc-observe-blocks to work when visibility_distances is empty.
    """
    aff = {'step': False, 'jump': False, 'descend': False, 'sky': False}

    # step: if forward is clear and has solid support
    if support.get('fwd', {}).get('type') == 'solid':
        if clear.get('fwd', {}).get('body') and clear.get('fwd', {}).get('head'):
            aff['step'] = True

    # jump: can jump up (up clearance available)
    if clear.get('up', {}).get('body') and clear.get('up', {}).get('head'):
        aff['jump'] = True

    # descend: if forward support exists but at lower depth (heuristic)
    try:
        here_depth = support.get('here', {}).get('depth')
        fwd_depth = support.get('fwd', {}).get('depth')
        if isinstance(here_depth, (int, float)) and isinstance(fwd_depth, (int, float)) and fwd_depth > here_depth + 0.5:
            aff['descend'] = True
    except Exception:
        pass

    # sky: prefer actual up block; fall back to visibility ray result if provided
    up_blk = dirs.get('up', {}).get('blk')
    if up_blk is None and clear.get('up', {}).get('body') and clear.get('up', {}).get('head'):
        aff['sky'] = True
    else:
        up_dist = visibility_distances.get('up') if isinstance(visibility_distances, dict) else None
        if up_dist is not None and isinstance(up_dist, (int, float)) and up_dist > 10:
            aff['sky'] = True

    return aff


def compute_confidence(blocks_complete: bool, entities_complete: bool, blocks_elapsed_ms: float, entities_elapsed_ms: float) -> str:
    if not blocks_complete or not entities_complete:
        return 'low'
    # Keep simple: treat slow observations as medium
    if blocks_elapsed_ms > 8000 or entities_elapsed_ms > 8000:
        return 'med'
    return 'high'


def tool(input_value=None, **kwargs):
    """
    Unified observation tool that fetches blocks, entities, and items in one call.
    
    Returns structured observation summary with all available data.
    Logs what data is successfully returned and what is missing.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        # Use small radius initially to minimize logging noise
        radius = kwargs.get("radius") or 9
        radius = max(1, min(11, int(radius)))
        
        # Request all data: blocks + entities (items are subset of entities)
        params = {
            "radius": radius,
            "entity_filter": ""  # Empty string = all entities (including items)
        }
        url = f"{minecraft_url}/observe"
        
        logger.info(f"🔍 mc-observe: GET {url} params={params}")
        response = requests.get(url, params=params, timeout=10.0)
        logger.info(f"📥 mc-observe: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        # Log what data is present/missing
        status = data.get('status', {})
        perception = data.get('perception', {})
        
        logger.info(f"📊 mc-observe: Data structure check:")
        logger.info(f"  ✓ status: {bool(status)}")
        logger.info(f"  ✓ perception: {bool(perception)}")
        
        # Check blocks data
        nearby_blocks = perception.get('nearby_blocks', [])
        blocks_complete = perception.get('blocks_complete', False)
        blocks_elapsed_ms = perception.get('blocks_elapsed_ms', 0)
        nav_surface = perception.get('nav_surface', [])
        adjacent_blocks = perception.get('adjacent_blocks', {})
        visibility_distances = perception.get('visibility_distances', {})
        
        logger.info(f"  ✓ nearby_blocks: {len(nearby_blocks)} blocks")
        logger.info(f"  ✓ blocks_complete: {blocks_complete}")
        logger.info(f"  ✓ nav_surface: {len(nav_surface)} cells")
        logger.info(f"  ✓ adjacent_blocks: {bool(adjacent_blocks)}")
        logger.info(f"  ✓ visibility_distances: {bool(visibility_distances)}")
        
        # Check entities data
        nearby_entities = perception.get('nearby_entities', [])
        entities_complete = perception.get('entities_complete', False)
        entities_elapsed_ms = perception.get('entities_elapsed_ms', 0)
        
        logger.info(f"  ✓ nearby_entities: {len(nearby_entities)} entities")
        logger.info(f"  ✓ entities_complete: {entities_complete}")
        
        # Check entity metadata (age, velocity)
        entities_with_age = 0
        entities_with_velocity = 0
        entities_with_persistence = 0
        
        for entity in nearby_entities:
            if isinstance(entity, dict):
                if 'age_ticks' in entity:
                    entities_with_age += 1
                if 'velocity' in entity:
                    entities_with_velocity += 1
                if 'time_until_despawn_ticks' in entity or 'is_stationary' in entity:
                    entities_with_persistence += 1
        
        logger.info(f"  📈 Entity metadata:")
        logger.info(f"    - entities with age_ticks: {entities_with_age}/{len(nearby_entities)}")
        logger.info(f"    - entities with velocity: {entities_with_velocity}/{len(nearby_entities)}")
        logger.info(f"    - entities with persistence data: {entities_with_persistence}/{len(nearby_entities)}")
        
        # Extract position and orientation
        position = status.get('position', {})
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            position = {'x': position[0], 'y': position[1], 'z': position[2]}
        
        yaw = status.get('yaw', 0.0)
        pitch = status.get('pitch', 0.0)

        # ------------------------------------------------------------------
        # Derived block/navigation fields (carried over from old mc-observe-blocks)
        # ------------------------------------------------------------------
        px = position.get('x', 0.0)
        py = position.get('y', 0.0)
        pz = position.get('z', 0.0)

        # dirs: prefer adjacent_blocks for blk; dist may be None if bridge skips visibility_distances
        dirs_info: Dict[str, Dict[str, Any]] = {}
        for dir_name, dir_key in [('fwd', 'forward'), ('back', 'back'), ('left', 'left'), ('right', 'right'), ('up', 'up'), ('down', 'down')]:
            blk = adjacent_blocks.get(dir_name) if isinstance(adjacent_blocks, dict) else None
            # if not present, try to find in nearby_blocks list as fallback
            if blk is None:
                pos_abs = None
                if dir_name == 'fwd':
                    pos_abs = rel_to_abs(position, yaw, 1, 0, 0)
                elif dir_name == 'back':
                    pos_abs = rel_to_abs(position, yaw, -1, 0, 0)
                elif dir_name == 'left':
                    pos_abs = rel_to_abs(position, yaw, 0, -1, 0)
                elif dir_name == 'right':
                    pos_abs = rel_to_abs(position, yaw, 0, 1, 0)
                elif dir_name == 'up':
                    pos_abs = rel_to_abs(position, yaw, 0, 0, 1)
                elif dir_name == 'down':
                    pos_abs = rel_to_abs(position, yaw, 0, 0, -1)
                if pos_abs:
                    blk = find_block_at(nearby_blocks, *pos_abs)
            dirs_info[dir_name] = {'dist': visibility_distances.get(dir_key) if isinstance(visibility_distances, dict) else None, 'blk': blk}

        # support + clear logic (ported from old mc-observe-blocks)
        nav_surface_by_xz: Dict[Tuple[int, int], Dict[str, Any]] = {}
        if isinstance(nav_surface, list):
            for ns in nav_surface:
                if not isinstance(ns, dict):
                    continue
                x = ns.get('x')
                z = ns.get('z')
                if x is None or z is None:
                    continue
                nav_surface_by_xz[(int(x), int(z))] = ns

        NON_SUPPORTING_BLOCKS = (
            'minecraft:snow',
            'minecraft:carpet',
            'minecraft:pressure_plate',
            'minecraft:farmland',
            'minecraft:rail',
            'minecraft:trapdoor',
        )
        MAX_SUPPORT_PROBE = 2

        def classify_support_at(forward_offset: int) -> Dict[str, Any]:
            if forward_offset == 0:
                down_blk = adjacent_blocks.get('down') if isinstance(adjacent_blocks, dict) else None
                if down_blk and down_blk != 'air':
                    block_name = down_blk if isinstance(down_blk, str) else str(down_blk)
                    if not any(block_name.startswith(ns) for ns in NON_SUPPORTING_BLOCKS):
                        if is_solid(block_name):
                            down_dist = dirs_info.get('down', {}).get('dist')
                            depth = down_dist if down_dist is not None and down_dist > 0 else 1.0
                            return {'type': 'solid', 'block': block_name, 'depth': depth}

            if forward_offset == 1:
                pos_fwd = rel_to_abs(position, yaw, 1, 0, 0)
                fwd_x, _, fwd_z = pos_fwd
                ns = nav_surface_by_xz.get((int(fwd_x), int(fwd_z)))
                if ns and ns.get('support_y') is not None and ns.get('support_block'):
                    try:
                        depth = float(position.get('y', 0)) - float(ns['support_y'])
                    except Exception:
                        depth = None
                    return {'type': 'solid', 'block': ns['support_block'], 'depth': depth}

            for dy in range(1, MAX_SUPPORT_PROBE + 1):
                pos = rel_to_abs(position, yaw, forward_offset, 0, -dy)
                block = find_block_at(nearby_blocks, *pos)
                if not block:
                    continue
                block_name = block if isinstance(block, str) else str(block)
                if any(block_name.startswith(ns) for ns in NON_SUPPORTING_BLOCKS):
                    continue
                if is_solid(block_name):
                    return {'type': 'solid', 'block': block_name, 'depth': dy}
                return {'type': 'unsafe', 'block': block_name, 'depth': dy}

            return {'type': 'unsafe', 'block': None, 'depth': None}

        here_support = classify_support_at(forward_offset=0)
        fwd_support = classify_support_at(forward_offset=1)

        # Record forward block at foot level for context
        block_fwd_block = adjacent_blocks.get('fwd') if isinstance(adjacent_blocks, dict) else None
        if block_fwd_block is None:
            pos_fwd_block = rel_to_abs(position, yaw, 1, 0, 0)
            block_fwd_block = find_block_at(nearby_blocks, *pos_fwd_block)
        fwd_support['forward_block'] = block_fwd_block

        support_info = {'here': here_support, 'fwd': fwd_support}

        # clear: body/head in front and above (body=+1y, head=+2y)
        pos_fwd_body = rel_to_abs(position, yaw, 1, 0, 1)
        pos_fwd_head = rel_to_abs(position, yaw, 1, 0, 2)
        block_fwd_body = find_block_at(nearby_blocks, *pos_fwd_body)
        block_fwd_head = find_block_at(nearby_blocks, *pos_fwd_head)
        fwd_body_clear = block_fwd_body is None or block_fwd_body == 'air'
        fwd_head_clear = block_fwd_head is None or block_fwd_head == 'air'

        pos_up_body = rel_to_abs(position, yaw, 0, 0, 1)
        pos_up_head = rel_to_abs(position, yaw, 0, 0, 2)
        block_up_body = find_block_at(nearby_blocks, *pos_up_body)
        block_up_head = find_block_at(nearby_blocks, *pos_up_head)
        up_body_clear = block_up_body is None or block_up_body == 'air'
        up_head_clear = block_up_head is None or block_up_head == 'air'

        clear_info = {'fwd': {'body': fwd_body_clear, 'head': fwd_head_clear}, 'up': {'body': up_body_clear, 'head': up_head_clear}}

        # Store air blocks inferred from clear diagnostics (side-effect to local_grid)
        try:
            if set_cell is not None and executor:
                if fwd_body_clear:
                    set_cell(executor, *pos_fwd_body, "air")
                if fwd_head_clear:
                    set_cell(executor, *pos_fwd_head, "air")
                if up_body_clear:
                    set_cell(executor, *pos_up_body, "air")
                if up_head_clear:
                    set_cell(executor, *pos_up_head, "air")
        except Exception as e:
            logger.debug(f"mc-observe: clear air storage skipped: {e}")

        # blocks categorization
        seen_blocks: Set[str] = set()
        fluid_blocks: Set[str] = set()
        hazard_blocks: Set[str] = set()
        for block in nearby_blocks:
            if not isinstance(block, dict):
                continue
            bn = block.get('name')
            if not isinstance(bn, str):
                continue
            if bn and bn != 'air':
                seen_blocks.add(bn)
                lower = bn.lower()
                if any(f in lower for f in FLUID_BLOCKS):
                    fluid_blocks.add(bn)
                if any(h in lower for h in HAZARD_BLOCKS):
                    hazard_blocks.add(bn)

        geom = compute_geometry(nearby_blocks, position, yaw)
        aff = compute_affordances(visibility_distances if isinstance(visibility_distances, dict) else {}, support_info, clear_info, dirs_info)

        # ------------------------------------------------------------------
        # Entity summaries (carry over from old mc-observe-entities)
        # ------------------------------------------------------------------
        entities_by_category = defaultdict(list)
        entities_by_type = defaultdict(int)
        entities_with_distance = []
        nearest_by_type: Dict[str, float] = {}

        for ent in nearby_entities:
            if not isinstance(ent, dict):
                continue
            ent_type = ent.get('type') or ent.get('name') or 'unknown'
            dist = float(ent.get('distance', 0.0) or 0.0)
            cat = categorize_entity(str(ent_type))
            entities_by_category[cat].append(ent)
            entities_by_type[str(ent_type)] += 1

            ent_pos = ent.get('position')
            pos_tuple = None
            if isinstance(ent_pos, dict):
                pos_tuple = (ent_pos.get('x', 0), ent_pos.get('y', 0), ent_pos.get('z', 0))
            elif isinstance(ent_pos, (list, tuple)) and len(ent_pos) >= 3:
                pos_tuple = (ent_pos[0], ent_pos[1], ent_pos[2])

            entry = {
                'type': str(ent_type),
                'category': cat,
                'distance': dist,
                'position': pos_tuple
            }
            # keep item-specific fields if present
            if cat == 'item':
                entry['item_name'] = ent.get('item_name') or ent.get('name') or str(ent_type)
                entry['item_count'] = ent.get('item_count', 1)
            entities_with_distance.append(entry)

        entities_with_distance.sort(key=lambda x: x.get('distance', 0.0))
        for ent in entities_with_distance:
            t = ent.get('type')
            if t and t not in nearest_by_type:
                nearest_by_type[t] = float(ent.get('distance', 0.0))
        
        # Build summary
        summary_parts = []
        summary_parts.append("UNIFIED OBSERVATION SUMMARY:")
        summary_parts.append("")
        summary_parts.append(f"Position: ({position.get('x', 0):.2f}, {position.get('y', 0):.2f}, {position.get('z', 0):.2f})")
        summary_parts.append(f"Orientation: yaw={yaw:.1f}°, pitch={pitch:.1f}°")
        summary_parts.append("")
        summary_parts.append(f"Blocks: {len(nearby_blocks)} visible ({len(seen_blocks)} types)")
        cat_counts = {k: len(v) for k, v in entities_by_category.items()}
        summary_parts.append(f"Entities: {len(nearby_entities)} total (by_category={dict(sorted(cat_counts.items()))})")
        summary_parts.append("")
        
        # Log sample entity data for debugging
        if nearby_entities:
            sample_entity = nearby_entities[0]
            logger.info(f"📋 Sample entity data structure:")
            logger.info(f"  Keys: {list(sample_entity.keys())}")
            if 'age_ticks' in sample_entity:
                logger.info(f"  ✓ age_ticks: {sample_entity['age_ticks']}")
            else:
                logger.info(f"  ✗ age_ticks: MISSING")
            if 'velocity' in sample_entity:
                logger.info(f"  ✓ velocity: {sample_entity['velocity']}")
            else:
                logger.info(f"  ✗ velocity: MISSING")
            if 'time_until_despawn_ticks' in sample_entity:
                logger.info(f"  ✓ time_until_despawn_ticks: {sample_entity['time_until_despawn_ticks']}")
            else:
                logger.info(f"  ✗ time_until_despawn_ticks: MISSING")
            if 'is_stationary' in sample_entity:
                logger.info(f"  ✓ is_stationary: {sample_entity['is_stationary']}")
            else:
                logger.info(f"  ✗ is_stationary: MISSING")
        
        conf = compute_confidence(bool(blocks_complete), bool(entities_complete), float(blocks_elapsed_ms or 0), float(entities_elapsed_ms or 0))

        # note: human-readable (short) summary
        note_parts = []
        if aff.get('sky'):
            note_parts.append("Open sky visible")
        if here_support.get('type') == 'solid':
            note_parts.append("solid footing")
        if len(seen_blocks) > 0:
            note_parts.append(f"{len(seen_blocks)} block types visible")
        if not note_parts:
            note_parts.append("Standard terrain")
        note = ", ".join(note_parts)

        # Build structured data response (union schema)
        structured_data = {
            "pose": {
                "x": px,
                "y": py,
                "z": pz,
                "yaw": yaw,
                "pitch": pitch
            },
            "nav_surface": nav_surface,
            "dirs": dirs_info,
            "support": support_info,
            "clear": clear_info,
            "blocks": {
                "seen": sorted(list(seen_blocks)),
                "fluid": sorted(list(fluid_blocks)),
                "hazard": sorted(list(hazard_blocks)),
                "nearby": nearby_blocks
            },
            "entities": {
                "total": len(nearby_entities),
                "by_category": {cat: len(ents) for cat, ents in entities_by_category.items()} if nearby_entities else {},
                "types": dict(entities_by_type) if nearby_entities else {},
                "nearest": nearest_by_type if nearby_entities else {},
                "by_distance": entities_with_distance if nearby_entities else [],
                "nearby": nearby_entities
            },
            "geom": geom,
            "aff": aff,
            "conf": conf,
            "note": note
        }

        # Side-effect only: keep session-local grid current (no schema changes)
        try:
            if ingest_nearby_blocks is not None:
                ingest_nearby_blocks(executor, pose=structured_data.get("pose", {}), nearby_blocks=nearby_blocks)
        except Exception as e:
            logger.debug(f"mc-observe: local_grid ingest skipped: {e}")
        
        summary_text = "\n".join(summary_parts)
        if len(summary_text) > 1024:
            summary_text = summary_text[:1021] + "..."
        
        # IMPORTANT: return uniform_return so InfospaceExecutor persists the structured dict
        # into the $out Note content (data field), instead of persisting the human summary.
        return executor._create_uniform_return("success", value=structured_data, extra={"summary": summary_text})
        
    except requests.exceptions.RequestException as e:
        logger.error(f"❌ mc-observe: HTTP request failed: {e}")
        return executor._create_uniform_return("failed", reason=f"HTTP request failed: {e}", value=None)
    except Exception as e:
        logger.error(f"❌ mc-observe: Unexpected error: {e}", exc_info=True)
        return executor._create_uniform_return("failed", reason=f"Unexpected error: {e}", value=None)
