"""
Session-local rolling 3D occupancy grid for Minecraft.

- Stored in executor world_state under key "local_grid".
- Transient: exists only for current run/session (in-memory).
- Rolling window: evict cells beyond radius (default 10) from agent block position.
- Updated by:
  - mc-observe (ingest nearby blocks)
  - mc-dig / mc-place (apply optimistic deltas on success)
  - movement tools (update center + prune)

Cells are keyed by absolute block coordinates (x,y,z ints). Values store a known block name.
Absence means unknown (not free).

Training Data Capture:
- Enhanced cells include material class, hazard flags, breakability, geometry flags
- Agent context captured on position changes
- Ground truth grid requires bridge endpoint (future)
"""

from __future__ import annotations

import json
import logging
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

logger = logging.getLogger(__name__)

try:
    import zstandard as zstd
    ZSTD_AVAILABLE = True
except ImportError:
    ZSTD_AVAILABLE = False


WORLD_STATE_KEY = "local_grid"
DEFAULT_RADIUS = 10

# Block classification constants (from existing code)
HAZARD_BLOCKS = {'lava', 'fire', 'cactus', 'magma_block', 'soul_fire', 'wither_rose', 'sweet_berry_bush', 'campfire', 'soul_campfire'}
FLUID_BLOCKS = {'water', 'lava', 'flowing_water', 'flowing_lava'}
PARTIAL_BLOCKS = {'snow', 'carpet', 'pressure_plate', 'trapdoor', 'slab', 'stairs'}
NON_SUPPORTING_PREFIXES = ('minecraft:snow', 'minecraft:carpet', 'minecraft:pressure_plate', 'minecraft:farmland', 'minecraft:rail', 'minecraft:trapdoor', 'minecraft:short_grass', 'minecraft:tall_grass')

MATERIAL_CLASSES = {
    'stone': ['stone', 'cobblestone', 'granite', 'diorite', 'andesite', 'deepslate', 'blackstone', 'basalt'],
    'dirt': ['dirt', 'grass_block', 'podzol', 'mycelium', 'coarse_dirt', 'rooted_dirt', 'mud'],
    'sand': ['sand', 'red_sand', 'gravel', 'soul_sand', 'soul_soil'],
    'wood': ['oak_log', 'spruce_log', 'birch_log', 'jungle_log', 'acacia_log', 'dark_oak_log', 'planks'],
    'foliage': ['leaves', 'grass', 'fern', 'flower', 'mushroom', 'vine', 'moss'],
    'water': ['water', 'flowing_water'],
    'lava': ['lava', 'flowing_lava'],
    'ore': ['ore', 'coal', 'iron', 'gold', 'diamond', 'emerald', 'lapis', 'redstone', 'copper'],
}


def _key(x: int, y: int, z: int) -> str:
    return f"{int(x)},{int(y)},{int(z)}"


def _parse_key(k: str) -> Optional[Tuple[int, int, int]]:
    try:
        xs, ys, zs = k.split(",", 2)
        return int(xs), int(ys), int(zs)
    except Exception:
        return None


def _block_coords_from_pose(pose: Dict[str, Any]) -> Tuple[int, int, int]:
    px = float(pose.get("x", 0.0) or 0.0)
    py = float(pose.get("y", 0.0) or 0.0)
    pz = float(pose.get("z", 0.0) or 0.0)
    return int(math.floor(px)), int(math.floor(py)), int(math.floor(pz))


def _within_radius(x: int, y: int, z: int, cx: int, cy: int, cz: int, radius: int) -> bool:
    # Chebyshev ball in 3D (rolling cube): fast and matches block-grid intuition.
    return max(abs(x - cx), abs(y - cy), abs(z - cz)) <= int(radius)


def is_air_name(name: Optional[str]) -> bool:
    if not name:
        return False
    n = name.split(":")[-1] if ":" in name else str(name)
    return n in {"air", "cave_air", "void_air"}


def get_cell_name(executor: Any, x: int, y: int, z: int) -> Optional[str]:
    """
    Return known block name at (x,y,z) if present in local grid, else None (unknown).
    """
    grid = get_or_init(executor)
    cells = grid.get("cells")
    if not isinstance(cells, dict):
        return None
    entry = cells.get(_key(int(x), int(y), int(z)))
    if not isinstance(entry, dict):
        return None
    name = entry.get("name")
    return str(name) if isinstance(name, str) else (str(name) if name is not None else None)


def get_or_init(executor: Any, *, radius: int = DEFAULT_RADIUS) -> Dict[str, Any]:
    grid = executor.get_world_state(WORLD_STATE_KEY)
    if isinstance(grid, dict) and "cells" in grid and "center" in grid:
        # Ensure radius is present
        if "radius" not in grid:
            grid["radius"] = int(radius)
        return grid

    grid = {
        "radius": int(radius),
        "center": {"x": 0, "y": 0, "z": 0},
        "cells": {},  # key -> {"name": str, "ts": float}
    }
    executor.set_world_state(WORLD_STATE_KEY, grid)
    return grid


def check_cell_safety(executor: Any, dx: int, dy: int, dz: int) -> Optional[Dict[str, Any]]:
    """
    Deterministic safety check for a cell at agent-relative position (dx, dy, dz) from agent.
    Returns dict with safety flags or None if cell is unknown.
    Used by path-frontier to replace PERCEPTUAL_FRAME checks.
    
    Args:
        dx: Agent-relative X (right+/left-)
        dy: Agent-relative Y (up+/down-)
        dz: Agent-relative Z (forward+/back-)
    
    Returns:
        {
            'has_hazard': bool,      # Hazard blocks detected in cell or adjacent
            'is_void': bool,         # Air below for multiple levels (void-like)
            'is_cliff': bool         # Significant drop detected
        }
        or None if cell data unavailable
    """
    grid = get_or_init(executor)
    if not isinstance(grid, dict) or "center" not in grid:
        return None
    
    center = grid.get("center", {})
    if not isinstance(center, dict):
        return None
    
    cx = int(center.get("x", 0))
    cy = int(center.get("y", 0))
    cz = int(center.get("z", 0))
    yaw = center.get("yaw", 0.0)
    if yaw is None:
        yaw = 0.0
    
    # Transform agent-relative to world-relative, then to absolute
    try:
        from nav_core import agent_rel_to_world
        world_dx, world_dy, world_dz = agent_rel_to_world(float(dx), float(dy), float(dz), float(yaw))
        abs_x = cx + int(round(world_dx))
        abs_y = cy + int(round(world_dy))
        abs_z = cz + int(round(world_dz))
    except Exception:
        # Fallback: treat as world-relative (should not happen)
        abs_x = cx + int(dx)
        abs_y = cy + int(dy)
        abs_z = cz + int(dz)
    
    # Check for hazards in cell and adjacent cells (conservative)
    has_hazard = False
    for check_dx in (-1, 0, 1):
        for check_dy in (-1, 0, 1):
            for check_dz in (-1, 0, 1):
                name = get_cell_name(executor, abs_x + check_dx, abs_y + check_dy, abs_z + check_dz)
                if name and _is_hazard(name):
                    has_hazard = True
                    break
            if has_hazard:
                break
        if has_hazard:
            break
    
    # Check for void (air below for multiple levels)
    is_void = False
    void_check_depth = 3
    void_air_count = 0
    for check_y in range(abs_y - 1, abs_y - void_check_depth - 1, -1):
        name = get_cell_name(executor, abs_x, check_y, abs_z)
        if name is None:
            break  # Unknown - can't determine void
        if is_air_name(name):
            void_air_count += 1
        else:
            break  # Hit solid block
    is_void = (void_air_count >= void_check_depth)
    
    # Check for cliff (significant drop - check support_y differences)
    is_cliff = False
    # Simple heuristic: if cell below is air and cell 2 below is also air/unknown, likely cliff
    below_name = get_cell_name(executor, abs_x, abs_y - 1, abs_z)
    below2_name = get_cell_name(executor, abs_x, abs_y - 2, abs_z)
    if below_name and is_air_name(below_name):
        if below2_name is None or is_air_name(below2_name):
            is_cliff = True
    
    return {
        'has_hazard': has_hazard,
        'is_void': is_void,
        'is_cliff': is_cliff
    }


def get_observed_voxel_grid_as_text(executor: Any, radius: int = 1) -> Optional[str]:
    """
    Get observed voxel grid as LLM-friendly formatted string.
    Returns formatted text or None if grid unavailable.
    
    Coordinate semantics (agent-relative):
    - (0,0,0) = agent's feet block
    - (0,-1,0) = support block below
    - dx: right(+)/left(-) relative to agent yaw
    - dz: forward(+)/back(-) relative to agent yaw
    - dy: up(+)/down(-) always vertical
    """
    voxel_grid = get_observed_voxel_grid(executor, radius=radius)
    if not voxel_grid:
        return None
    
    cells = voxel_grid.get("cells", [])
    if not isinstance(cells, list):
        return None
    
    if not cells:
        return f"Observed voxel grid (radius={radius}): empty (no cells observed)"
    
    # Sort cells by dy (vertical), then dz, then dx for readability
    sorted_cells = sorted(cells, key=lambda c: (c.get("dy", 0), c.get("dz", 0), c.get("dx", 0)))

    # Extract agent's absolute position from grid center
    center = voxel_grid.get("center", {})

    lines = [f"Observed voxel grid (radius={radius}): {len(cells)} cells"]
    # Include agent's absolute position so planner knows where agent is
    if center:
        lines.append(f"Agent position: x={center.get('x', '?')}, y={center.get('y', '?')}, z={center.get('z', '?')}, yaw={center.get('yaw', '?')}")
    for cell in sorted_cells:
        dx = cell.get("dx", 0)
        dy = cell.get("dy", 0)
        dz = cell.get("dz", 0)
        solid = cell.get("solid", False)
        block_id = cell.get("block_id")
        support = cell.get("support", False)
        
        block_str = block_id if block_id else "air"
        solid_str = "solid" if solid else "air/fluid"
        support_str = "support" if support else "no_support"
        
        lines.append(f"  [{dx},{dy},{dz}]: {block_str} ({solid_str}, {support_str})")
    
    return "\n".join(lines)


def get_observed_voxel_grid(executor: Any, radius: int = 1) -> Optional[Dict[str, Any]]:
    """
    Extract observed voxel grid: radius-bounded explicit voxel grid centered on agent.
    Planner-facing format with agent-relative coordinates (dx, dy, dz).
    Extracts from local_grid (raw voxel data from mc-observe, not CNN inference results).
    
    Coordinate semantics (agent-relative):
    - center = floor(pose) = agent's feet block coordinate
    - (0,0,0) = agent's feet block (where agent stands, typically air)
    - (0,-1,0) = support block (what agent stands on, solid)
    - (0,1,0) = head space (above agent)
    - dx: right(+)/left(-) relative to agent yaw
    - dz: forward(+)/back(-) relative to agent yaw
    - dy: up(+)/down(-) always vertical
    
    Returns dict with:
    - "center": {"x", "y", "z", "yaw"} (absolute, for reference)
    - "radius": int
    - "cells": [{"dx": int, "dy": int, "dz": int, "solid": bool, "block_id": str, "support": bool}, ...]
    
    Fields:
    - solid: true if block is not air/fluid
    - block_id: block name (e.g., "minecraft:stone") or null if air
    - support: true if cell provides support (solid block, or air with solid below)
    """
    grid = get_or_init(executor)
    if not isinstance(grid, dict) or "center" not in grid or "cells" not in grid:
        return None
    
    center = grid.get("center", {})
    if not isinstance(center, dict):
        return None
    
    cx = int(center.get("x", 0))
    cy = int(center.get("y", 0))
    cz = int(center.get("z", 0))
    yaw = center.get("yaw", 0.0)
    if yaw is None:
        logger.warning("get_observed_voxel_grid: Grid center yaw is None, defaulting to 0.0. "
                      "This may cause incorrect coordinate transformations.")
        yaw = 0.0
    else:
        # Validate yaw is cardinal (0, 90, 180, 270) - warn if not
        yaw_float = float(yaw)
        if yaw_float not in (0.0, 90.0, 180.0, 270.0):
            logger.warning(f"get_observed_voxel_grid: Grid center yaw {yaw_float} is not cardinal. "
                          "Expected 0, 90, 180, or 270. This may cause coordinate errors.")

    # Import transformation function
    try:
        from nav_core import world_to_agent_rel
    except Exception:
        # Fallback: return world-relative if transformation unavailable
        world_to_agent_rel = None
    
    cells = grid.get("cells", {})
    if not isinstance(cells, dict):
        return None
    
    voxel_cells = []
    
    # Iterate over all cells within radius (Chebyshev ball)
    for key, cell_data in cells.items():
        if not isinstance(cell_data, dict):
            continue
        
        xyz = _parse_key(key)
        if not xyz:
            continue
        x, y, z = xyz
        
        # Check if within radius
        if not _within_radius(x, y, z, cx, cy, cz, radius):
            continue
        
        # Compute world-relative coordinates
        world_dx = x - cx
        dy = y - cy  # dy is always vertical, no transformation needed
        world_dz = z - cz
        
        # Transform to agent-relative coordinates
        if world_to_agent_rel is not None:
            dx, _, dz = world_to_agent_rel(world_dx, dy, world_dz, yaw)
            # Round to integers for discrete grid
            dx = int(round(dx))
            dz = int(round(dz))
        else:
            # Fallback: use world-relative (should not happen in normal operation)
            dx = int(world_dx)
            dz = int(world_dz)
        
        # Extract block name
        block_name = cell_data.get("name", "")
        if not isinstance(block_name, str):
            block_name = ""
        
        # Determine solid (not air, not fluid)
        is_air = is_air_name(block_name)
        is_fluid = block_name.lower().replace("minecraft:", "") in FLUID_BLOCKS
        solid = not is_air and not is_fluid
        
        # Compute support flag
        # Support = true if:
        #   - Block is solid (provides support)
        #   - Block is air AND cell below is solid
        support = False
        if solid:
            support = True
        elif is_air:
            # Check cell below for support
            below_name = get_cell_name(executor, x, y - 1, z)
            if below_name and not is_air_name(below_name):
                below_is_fluid = below_name.lower().replace("minecraft:", "") in FLUID_BLOCKS
                if not below_is_fluid:
                    support = True
        
        voxel_cells.append({
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "solid": solid,
            "block_id": block_name if block_name else None,
            "support": support,
        })
    
    return {
        "center": {"x": cx, "y": cy, "z": cz, "yaw": float(yaw)},
        "radius": radius,
        "cells": voxel_cells,
    }




def prune(executor: Any) -> None:
    grid = get_or_init(executor)
    radius = int(grid.get("radius", DEFAULT_RADIUS))
    center = grid.get("center", {}) if isinstance(grid.get("center"), dict) else {}
    cx = int(center.get("x", 0))
    cy = int(center.get("y", 0))
    cz = int(center.get("z", 0))
    cells = grid.get("cells")
    if not isinstance(cells, dict):
        grid["cells"] = {}
        executor.set_world_state(WORLD_STATE_KEY, grid)
        return

    to_delete = []
    for k in list(cells.keys()):
        xyz = _parse_key(k)
        if not xyz:
            to_delete.append(k)
            continue
        x, y, z = xyz
        if not _within_radius(x, y, z, cx, cy, cz, radius):
            to_delete.append(k)
    for k in to_delete:
        cells.pop(k, None)

    executor.set_world_state(WORLD_STATE_KEY, grid)


def enable_training_capture(executor: Any, *, sample_interval: int = 5, include_ground_truth: bool = True, minecraft_url: Optional[str] = None) -> None:
    """
    Enable automatic training sample capture on position changes.
    
    Args:
        executor: InfospaceExecutor instance
        sample_interval: Capture sample every N position changes (default: 5)
        include_ground_truth: If True, fetch ground truth from bridge (default: True, slower but required for useful data)
        minecraft_url: Bridge URL for ground truth (defaults to MINECRAFT_URL env var)
    """
    grid = get_or_init(executor)
    grid["_capture_training"] = True
    grid["_sample_interval"] = int(sample_interval)
    grid["_capture_ground_truth"] = bool(include_ground_truth)
    if minecraft_url:
        grid["_minecraft_url"] = str(minecraft_url)
    executor.set_world_state(WORLD_STATE_KEY, grid)


def disable_training_capture(executor: Any) -> None:
    """Disable automatic training sample capture."""
    grid = get_or_init(executor)
    grid["_capture_training"] = False
    executor.set_world_state(WORLD_STATE_KEY, grid)


def set_center_from_pose(executor: Any, pose: Dict[str, Any], *, radius: Optional[int] = None, capture_training: Optional[bool] = None, sample_interval: int = 5) -> None:
    """
    Update grid center from agent pose and optionally capture training samples.
    
    Args:
        executor: InfospaceExecutor instance
        pose: Agent pose dict with x, y, z, yaw, pitch
        radius: Optional radius override
        capture_training: Optional override (if None, uses grid metadata setting)
        sample_interval: Capture sample every N position changes (if capture_training=True)
    """
    grid = get_or_init(executor, radius=int(radius or DEFAULT_RADIUS))
    if radius is not None:
        grid["radius"] = int(radius)
    cx, cy, cz = _block_coords_from_pose(pose)
    
    # Store yaw (cardinal-snapped) for coordinate transformation
    yaw = pose.get("yaw", 0.0)
    if yaw is not None:
        try:
            from nav_core import _round_to_cardinal
            yaw = _round_to_cardinal(float(yaw))
        except Exception:
            yaw = 0.0
    else:
        yaw = 0.0
    
    # Check if position actually changed
    old_center = grid.get("center", {})
    old_cx = old_center.get("x", None)
    old_cy = old_center.get("y", None)
    old_cz = old_center.get("z", None)
    position_changed = (old_cx != cx) or (old_cy != cy) or (old_cz != cz)
    
    grid["center"] = {"x": cx, "y": cy, "z": cz, "yaw": float(yaw)}
    executor.set_world_state(WORLD_STATE_KEY, grid)
    prune(executor)
    
    # Check if training capture is enabled (from grid metadata or parameter override)
    should_capture = capture_training if capture_training is not None else grid.get("_capture_training", False)
    interval = grid.get("_sample_interval", sample_interval)
    
    # Optional training sample capture on position change
    if should_capture and position_changed:
        try:
            # Track sample count in grid metadata
            sample_count = grid.get("_training_sample_count", 0) + 1
            grid["_training_sample_count"] = sample_count
            # Persist the updated sample count
            executor.set_world_state(WORLD_STATE_KEY, grid)
            
            # Capture every N position changes
            if sample_count % interval == 0:
                # Check if ground truth capture is enabled
                capture_gt = grid.get("_capture_ground_truth", False)
                minecraft_url = grid.get("_minecraft_url", None)
                
                sample = capture_training_sample(
                    executor,
                    world_tick=None,
                    seed=None,
                    include_ground_truth=capture_gt,
                    minecraft_url=minecraft_url,
                )
                if sample:
                    # Save to disk
                    saved_path = save_training_sample(executor, sample, compress=True)
                    if saved_path:
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.info(f"Training sample saved: {saved_path}")
                    
                    # Also keep in memory (last 10 samples for quick access)
                    samples = executor.get_world_state("training_samples") or []
                    if not isinstance(samples, list):
                        samples = []
                    samples.append(sample)
                    if len(samples) > 10:  # Keep only last 10 in memory
                        samples = samples[-10:]
                    executor.set_world_state("training_samples", samples)
        except Exception as e:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning(f"Training capture failed (non-fatal): {e}")  # Don't break position updates
    


def _classify_material(block_name: str) -> str:
    """Classify block into material class."""
    if not block_name:
        return "unknown"
    block_lower = block_name.lower().replace("minecraft:", "")
    for class_name, keywords in MATERIAL_CLASSES.items():
        if any(kw in block_lower for kw in keywords):
            return class_name
    return "unknown"


def _is_hazard(block_name: str) -> bool:
    """Check if block is hazardous."""
    if not block_name:
        return False
    block_lower = block_name.lower().replace("minecraft:", "")
    return any(h in block_lower for h in HAZARD_BLOCKS)


def _is_breakable(block_name: str) -> bool:
    """Check if block is breakable (not bedrock, not air)."""
    if not block_name:
        return False
    block_lower = block_name.lower().replace("minecraft:", "")
    if "air" in block_lower:
        return False
    if "bedrock" in block_lower:
        return False
    return True


def _is_partial(block_name: str) -> bool:
    """Check if block is partial height (snow, carpet, etc.)."""
    if not block_name:
        return False
    block_lower = block_name.lower().replace("minecraft:", "")
    return any(p in block_lower for p in PARTIAL_BLOCKS) or any(block_lower.startswith(p) for p in NON_SUPPORTING_PREFIXES)


def _enrich_cell_data(name: str, center: Dict[str, int], x: int, y: int, z: int, ts: float, world_tick: Optional[int] = None) -> Dict[str, Any]:
    """Enrich cell data with training-relevant properties."""
    cx = center.get("x", 0)
    cy = center.get("y", 0)
    cz = center.get("z", 0)
    
    cell_data = {
        "name": str(name),
        "ts": ts,
        "occupancy_class": "air" if is_air_name(name) else "solid",
        "material_class": _classify_material(name),
        "hazard": _is_hazard(name),
        "breakable": _is_breakable(name),
        "relative_position": {"dx": x - cx, "dy": y - cy, "dz": z - cz},
    }
    
    if world_tick is not None:
        cell_data["observation_age"] = world_tick  # Will be computed as delta when capturing
    
    return cell_data


def set_cell(executor: Any, x: int, y: int, z: int, name: str, *, world_tick: Optional[int] = None) -> None:
    grid = get_or_init(executor)
    radius = int(grid.get("radius", DEFAULT_RADIUS))
    center = grid.get("center", {}) if isinstance(grid.get("center"), dict) else {}
    cx = int(center.get("x", 0))
    cy = int(center.get("y", 0))
    cz = int(center.get("z", 0))

    if not _within_radius(int(x), int(y), int(z), cx, cy, cz, radius):
        # Outside rolling window: ignore.
        return

    cells = grid.get("cells")
    if not isinstance(cells, dict):
        cells = {}
        grid["cells"] = cells

    ts = time.time()
    cell_data = _enrich_cell_data(name, center, int(x), int(y), int(z), ts, world_tick)
    cells[_key(int(x), int(y), int(z))] = cell_data
    executor.set_world_state(WORLD_STATE_KEY, grid)
    


def ingest_nearby_blocks(
    executor: Any,
    *,
    pose: Dict[str, Any],
    nearby_blocks: Iterable[Dict[str, Any]],
    world_tick: Optional[int] = None,
) -> None:
    """
    Ingest an mc-observe nearby_blocks list into the local grid.
    Uses absolute block coordinates; does not change mc-observe return schema.
    """
    # Update center first (and prune)
    set_center_from_pose(executor, pose, radius=DEFAULT_RADIUS)
    grid = get_or_init(executor)
    radius = int(grid.get("radius", DEFAULT_RADIUS))
    center = grid.get("center", {}) if isinstance(grid.get("center"), dict) else {}
    cx = int(center.get("x", 0))
    cy = int(center.get("y", 0))
    cz = int(center.get("z", 0))

    for blk in nearby_blocks:
        if not isinstance(blk, dict):
            continue
        name = blk.get("name")
        if not isinstance(name, str):
            continue
        pos = blk.get("position")
        if isinstance(pos, dict):
            ax = int(math.floor(pos.get("x", 0.0)))
            ay = int(math.floor(pos.get("y", 0.0)))
            az = int(math.floor(pos.get("z", 0.0)))
        elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
            ax = int(math.floor(pos[0]))
            ay = int(math.floor(pos[1]))
            az = int(math.floor(pos[2]))
        else:
            continue

        if not _within_radius(ax, ay, az, cx, cy, cz, radius):
            continue
        set_cell(executor, ax, ay, az, name, world_tick=world_tick)


def _compute_geometry_flags(executor: Any, x: int, y: int, z: int, name: str) -> Dict[str, bool]:
    """
    Compute geometry flags for a cell (support, passable, head_clear).
    Requires neighbor queries, so may be incomplete if neighbors unknown.
    """
    flags = {
        "support": False,
        "passable": False,
        "head_clear": False,
        "thin_layer": _is_partial(name),
    }
    
    if is_air_name(name):
        flags["passable"] = True
        # Check if there's support below
        below_name = get_cell_name(executor, x, y - 1, z)
        if below_name and not is_air_name(below_name):
            flags["support"] = True
        # Check head clearance above
        # IMPORTANT: Only set head_clear=True when CONFIRMED air, not when unknown
        above_name = get_cell_name(executor, x, y + 1, z)
        flags["head_clear"] = above_name is not None and is_air_name(above_name)
    else:
        flags["passable"] = False
        flags["support"] = True  # Solid blocks provide support
    
    return flags


def _enrich_partial_grid(grid: Dict[str, Any], executor: Any) -> Dict[str, Any]:
    """
    Enhance partial grid with geometry flags and observation age.
    Returns enriched grid suitable for training input.
    """
    cells = grid.get("cells", {})
    center = grid.get("center", {})
    enriched_cells = {}
    
    current_time = time.time()
    
    for key, cell_data in cells.items():
        if not isinstance(cell_data, dict):
            continue
        
        xyz = _parse_key(key)
        if not xyz:
            continue
        x, y, z = xyz
        
        name = cell_data.get("name", "")
        
        # Compute geometry flags (may be incomplete if neighbors unknown)
        geometry = _compute_geometry_flags(executor, x, y, z, name)
        
        # Compute observation age (seconds since last seen)
        ts = cell_data.get("ts", current_time)
        observation_age_seconds = current_time - ts
        
        enriched_cell = cell_data.copy()
        enriched_cell.update({
            "geometry": geometry,
            "observation_age_seconds": observation_age_seconds,
        })
        
        enriched_cells[key] = enriched_cell
    
    return {
        "radius": grid.get("radius", DEFAULT_RADIUS),
        "center": center,
        "cells": enriched_cells,
    }


def _capture_agent_context(executor: Any, pose: Dict[str, Any]) -> Dict[str, Any]:
    """
    Capture agent context for training sample.
    Includes pose, movement state, inventory summary.
    """
    context = {
        "pose": {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "z": float(pose.get("z", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)),
            "pitch": float(pose.get("pitch", 0.0)),
        },
        "movement_state": "standing",  # TODO: track from nav tools
        "inventory_summary": {},  # TODO: fetch from mc-inventory if available
    }
    
    # Try to get inventory if available
    try:
        inv_state = executor.get_world_state("inventory")
        if isinstance(inv_state, dict):
            # Summarize inventory (coarse)
            items = inv_state.get("items", [])
            if isinstance(items, list):
                item_counts = {}
                for item in items:
                    if isinstance(item, dict):
                        item_name = item.get("name", "unknown")
                        item_counts[item_name] = item_counts.get(item_name, 0) + 1
                context["inventory_summary"] = item_counts
    except Exception:
        pass
    
    return context


def _fetch_ground_truth_grid(executor: Any, center: Dict[str, int], radius: int = 12, minecraft_url: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """
    Fetch ground truth grid from bridge /ground-truth endpoint.
    
    Args:
        executor: InfospaceExecutor instance
        center: Grid center {x, y, z}
        radius: Ground truth radius (default 12 for 25×25×25)
        minecraft_url: Optional bridge URL (defaults to MINECRAFT_URL env var)
    
    Returns:
        Ground truth grid dict or None if endpoint unavailable
    """
    try:
        import requests
        if minecraft_url is None:
            minecraft_url = os.getenv("MINECRAFT_URL", "http://localhost:3003")
        
        params = {
            "center_x": center.get("x", 0),
            "center_y": center.get("y", 0),
            "center_z": center.get("z", 0),
            "radius": radius,
        }
        
        url = f"{minecraft_url}/ground-truth"
        response = requests.get(url, params=params, timeout=30.0)
        response.raise_for_status()
        return response.json()
    except Exception:
        return None


def capture_training_sample(
    executor: Any,
    *,
    world_tick: Optional[int] = None,
    seed: Optional[int] = None,
    include_ground_truth: bool = False,
    minecraft_url: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
    """
    Capture one training example (partial observation grid + agent context + optional ground truth).
    
    Args:
        executor: InfospaceExecutor instance
        world_tick: Optional world tick (for observation age)
        seed: Optional world seed
        include_ground_truth: If True, fetch ground truth from bridge endpoint
        minecraft_url: Optional bridge URL for ground truth fetch
    
    Returns:
        Dict with:
        - partial_grid: Enhanced partial observation grid
        - ground_truth_grid: Ground truth grid (if include_ground_truth=True and endpoint available)
        - agent_context: Pose, movement, inventory
        - timestamp: Current time
        - seed: World seed if provided
    """
    grid = get_or_init(executor)
    if not isinstance(grid, dict) or "center" not in grid:
        return None
    
    center = grid.get("center", {})
    if not isinstance(center, dict):
        return None
    
    # Get current pose (from center + assume standing)
    # center is the feet block, so agent Y = center Y (feet are at block coordinate level)
    pose = {
        "x": float(center.get("x", 0)) + 0.5,
        "y": float(center.get("y", 0)),  # Agent feet at center Y (feet block coordinate level)
        "z": float(center.get("z", 0)) + 0.5,
        "yaw": 0.0,  # TODO: get from nav state
        "pitch": 0.0,
    }
    
    # Enrich partial grid
    partial_grid = _enrich_partial_grid(grid, executor)
    
    # Capture agent context
    agent_context = _capture_agent_context(executor, pose)
    
    # Fetch ground truth if requested
    ground_truth_grid = None
    if include_ground_truth:
        ground_truth_grid = _fetch_ground_truth_grid(executor, center, radius=12, minecraft_url=minecraft_url)
    
    sample = {
        "partial_grid": partial_grid,
        "ground_truth_grid": ground_truth_grid,
        "agent_context": agent_context,
        "timestamp": time.time(),
    }
    
    if world_tick is not None:
        sample["world_tick"] = world_tick
    if seed is not None:
        sample["seed"] = seed
    
    return sample


def _get_training_data_dir(executor: Any) -> Path:
    """Get training data storage directory."""
    # Try to get from executor's resource manager base_dir
    try:
        if hasattr(executor, "resource_manager") and executor.resource_manager:
            base_dir = executor.resource_manager.base_dir
            if base_dir:
                return Path(base_dir).parent / "training_data"
    except Exception:
        pass
    
    # Fallback: use scenarios/minecraft/training_data/
    current_dir = Path(__file__).parent
    project_root = current_dir.parent.parent  # src/world-tools/minecraft -> project root
    return project_root / "scenarios" / "minecraft" / "training_data"


def save_training_sample(executor: Any, sample: Dict[str, Any], *, compress: bool = True) -> Optional[Path]:
    """
    Save training sample to disk (JSON + optional Zstd compression).
    
    Args:
        executor: InfospaceExecutor instance
        sample: Training sample dict
        compress: If True, use Zstd compression (if available)
    
    Returns:
        Path to saved file, or None on error
    """
    try:
        data_dir = _get_training_data_dir(executor)
        data_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate filename from timestamp
        timestamp = sample.get("timestamp", time.time())
        filename = f"sample_{int(timestamp * 1000)}.json"
        if compress and ZSTD_AVAILABLE:
            filename += ".zst"
        
        filepath = data_dir / filename
        
        # Serialize to JSON
        json_bytes = json.dumps(sample, separators=(',', ':')).encode("utf-8")
        
        # Compress if requested and available
        if compress and ZSTD_AVAILABLE:
            cctx = zstd.ZstdCompressor(level=3)  # Level 3 = good balance
            compressed = cctx.compress(json_bytes)
            filepath.write_bytes(compressed)
        else:
            filepath.write_bytes(json_bytes)
        
        return filepath
    except Exception as e:
        import logging
        logger = logging.getLogger(__name__)
        logger.warning(f"Failed to save training sample: {e}")
        return None

