"""
Spatial Map - Cell-based persistent spatial memory for Minecraft.

Provides a hash-map structure keyed by (x, z) coordinates with cells
following the MAP_CELL_SCHEMA. Supports load/save persistence, cell
updates from observations, and spatial queries.
"""

import json
import logging
import math
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple, Callable

logger = logging.getLogger(__name__)

# Cell schema template (from map_core.py)
def empty_cell(x: int, z: int) -> Dict[str, Any]:
    """Create an empty cell with default values."""
    return {
        "cell_version": "1.0",
        "cell_id": {"x": x, "z": z},
            "support": {
            "walkable": False,
            "support_y": None,
            "movement_class": "unknown"
        },
        "surface": {
            "support_block": None,
            "surface_block_class": "unknown",
            "is_fluid": False,
            "is_partial_block": False
        },
        "obstructions": {
            # Store blocks at surface level for query-time blocking computation
            # Key: Y coordinate (int), Value: block name (str) or None
            # Used to compute if cell is blocked from a given position
            "blocks_at_y": {}  # Dict[int, Optional[str]]
        },
        "hazards": {
            "flags": [],
            "exposure_risk": "unknown",
            "escape_difficulty": "unknown"
        },
        "resources": {
            "resources_visible": [],
            "harvest_actions": [],
            "tool_required": "unknown"
        },
        "environment": {
            "light_level": "unknown",
            "spawn_risk": "unknown",
            "time_sensitive": False
        },
        "observability": {
            "observed_from": {"x": None, "y": None, "z": None},
            "observation_mode": "unknown",
            "confidence": 0.0,
            "last_observed_at": None
        },
        "provenance": {
            "updated_by": "unknown",
            "update_reason": "unknown"
        },
        "waypoints": [],  # List of waypoint names at this location
        "obstructions": {
            # Store blocks at surface level for query-time blocking computation
            # Key: Y coordinate (int), Value: block name (str) or None
            # Used to compute if cell is blocked from a given position
            "blocks_at_y": {}  # Dict[int, Optional[str]]
        }
    }


# Block classification helpers
FLUID_BLOCKS = {'water', 'lava', 'flowing_water', 'flowing_lava'}
HAZARD_BLOCKS = {'lava', 'fire', 'cactus', 'magma_block', 'soul_fire', 'wither_rose', 'sweet_berry_bush', 'campfire', 'soul_campfire'}
PARTIAL_BLOCKS = {'snow', 'carpet', 'pressure_plate', 'trapdoor', 'slab', 'stairs'}

SURFACE_BLOCK_CLASSES = {
    'stone': ['stone', 'cobblestone', 'granite', 'diorite', 'andesite', 'deepslate', 'blackstone', 'basalt'],
    'dirt': ['dirt', 'grass_block', 'podzol', 'mycelium', 'coarse_dirt', 'rooted_dirt', 'mud'],
    'sand': ['sand', 'red_sand', 'gravel', 'soul_sand', 'soul_soil'],
    'wood': ['oak_log', 'spruce_log', 'birch_log', 'jungle_log', 'acacia_log', 'dark_oak_log', 'planks'],
    'foliage': ['leaves', 'grass', 'fern', 'flower', 'mushroom', 'vine', 'moss'],
    'water': ['water', 'flowing_water'],
    'lava': ['lava', 'flowing_lava'],
    'ore': ['ore', 'coal', 'iron', 'gold', 'diamond', 'emerald', 'lapis', 'redstone', 'copper'],
}

def classify_block(block_name: str) -> str:
    """Classify a block into a surface class."""
    if not block_name:
        return 'unknown'
    
    block_lower = block_name.lower().replace('minecraft:', '')
    
    for class_name, keywords in SURFACE_BLOCK_CLASSES.items():
        if any(kw in block_lower for kw in keywords):
            return class_name
    
    return 'unknown'


def is_partial_block(block_name: str) -> bool:
    """Check if block is a partial block (snow layer, carpet, etc.)."""
    if not block_name:
        return False
    block_lower = block_name.lower().replace('minecraft:', '')
    return any(pb in block_lower for pb in PARTIAL_BLOCKS)


def is_fluid(block_name: str) -> bool:
    """Check if block is a fluid."""
    if not block_name:
        return False
    block_lower = block_name.lower().replace('minecraft:', '')
    return any(fb in block_lower for fb in FLUID_BLOCKS)


def extract_hazard_flags(blocks_data: Dict) -> List[str]:
    """Extract hazard flags from observation blocks data."""
    flags = []
    hazard_list = blocks_data.get('hazard', [])
    if isinstance(hazard_list, list):
        for block in hazard_list:
            block_lower = str(block).lower().replace('minecraft:', '')
            if 'lava' in block_lower:
                flags.append('lava')
            if 'fire' in block_lower:
                flags.append('fire')
            if 'cactus' in block_lower:
                flags.append('cactus')
            if 'magma' in block_lower:
                flags.append('magma')
            if 'wither_rose' in block_lower:
                flags.append('wither_rose')
            if 'berry' in block_lower:
                flags.append('sweet_berry')
    return list(set(flags))


class SpatialMap:
    """
    Cell-based spatial map with persistence.
    
    Stores cells keyed by (x, z) coordinates. Each cell follows the
    MAP_CELL_SCHEMA and contains support, surface, hazards, resources,
    environment, observability, and provenance information.
    """
    
    def __init__(self, agent_name: str, world_name: str, base_dir: Optional[Path] = None):
        """
        Initialize SpatialMap.
        
        Args:
            agent_name: Name of the agent owning this map
            world_name: World/scenario name (e.g., 'minecraft')
            base_dir: Base directory for persistence (defaults to scenarios/{world_name}/resources/)
        """
        self.agent_name = agent_name
        self.world_name = world_name
        
        # Determine base directory
        if base_dir:
            self.base_dir = Path(base_dir)
        else:
            # Default: find project root and use scenarios/<world_name>/resources/
            current_dir = Path(__file__).parent
            project_root = current_dir.parent.parent.parent  # world-tools/minecraft/ -> src/ -> project root
            self.base_dir = project_root / "scenarios" / world_name / "resources"
        
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.map_file = self.base_dir / f"{agent_name}_spatial_map.json"
        
        # In-memory storage
        self.cells: Dict[Tuple[int, int], Dict[str, Any]] = {}
        self.metadata: Dict[str, Any] = {
            "version": "1.0",
            "agent_name": agent_name,
            "world_name": world_name,
            "created_at": None,
            "updated_at": None,
            "cell_count": 0
        }
        
        # Load existing map
        self.load()
    
    def _cell_key(self, x: int, z: int) -> Tuple[int, int]:
        """Create cell key from coordinates."""
        return (int(round(x)), int(round(z)))
    
    def _key_to_str(self, key: Tuple[int, int]) -> str:
        """Convert cell key to string for JSON serialization."""
        return f"{key[0]},{key[1]}"
    
    def _str_to_key(self, s: str) -> Tuple[int, int]:
        """Convert string back to cell key."""
        parts = s.split(',')
        return (int(parts[0]), int(parts[1]))
    
    def load(self) -> bool:
        """Load spatial map from file."""
        if not self.map_file.exists():
            logger.info(f"No existing spatial map at {self.map_file}, starting fresh")
            self.metadata["created_at"] = datetime.utcnow().isoformat()
            return False
        
        try:
            with open(self.map_file, 'r') as f:
                data = json.load(f)
            
            # Load metadata
            self.metadata = data.get('metadata', self.metadata)
            
            # Load cells
            cells_data = data.get('cells', {})
            self.cells = {}
            for key_str, cell in cells_data.items():
                key = self._str_to_key(key_str)
                self.cells[key] = cell
            
            self.metadata["cell_count"] = len(self.cells)
            logger.info(f"📂 Loaded spatial map from {self.map_file}: {len(self.cells)} cells")
            return True
        except (json.JSONDecodeError, IOError) as e:
            logger.warning(f"Failed to load spatial map from {self.map_file}: {e}")
            self.metadata["created_at"] = datetime.utcnow().isoformat()
            return False
    
    def save(self) -> bool:
        """Save spatial map to file with backup."""
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            
            # Backup existing file
            if self.map_file.exists():
                try:
                    with open(self.map_file, 'r') as f:
                        existing = json.load(f)
                    backup_date = existing.get('metadata', {}).get('updated_at')
                    if backup_date:
                        formatted_date = backup_date.replace(':', '-').replace('.', '-').split('+')[0].split('Z')[0]
                        backup_filename = f"{self.agent_name}_spatial_map_{formatted_date}.json"
                        backup_path = self.base_dir / backup_filename
                        self.map_file.rename(backup_path)
                        logger.info(f"📦 Backed up spatial map to {backup_filename}")
                except Exception as e:
                    logger.warning(f"Failed to backup spatial map: {e}")
            
            # Update metadata
            if not self.metadata.get("created_at"):
                self.metadata["created_at"] = datetime.utcnow().isoformat()
            self.metadata["updated_at"] = datetime.utcnow().isoformat()
            self.metadata["cell_count"] = len(self.cells)
            
            # Serialize cells
            cells_data = {}
            for key, cell in self.cells.items():
                cells_data[self._key_to_str(key)] = cell
            
            data = {
                "metadata": self.metadata,
                "cells": cells_data
            }
            
            with open(self.map_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            logger.info(f"💾 Saved spatial map to {self.map_file}: {len(self.cells)} cells")
            return True
        except Exception as e:
            logger.error(f"Failed to save spatial map: {e}")
            return False
    
    def clear(self) -> bool:
        """Clear all cells and reset metadata."""
        self.cells = {}
        self.metadata = {
            "version": "1.0",
            "agent_name": self.agent_name,
            "world_name": self.world_name,
            "created_at": datetime.utcnow().isoformat(),
            "updated_at": datetime.utcnow().isoformat(),
            "cell_count": 0
        }
        
        # Delete the file if it exists
        if self.map_file.exists():
            try:
                self.map_file.unlink()
                logger.info(f"🗑️ Deleted spatial map file: {self.map_file}")
            except Exception as e:
                logger.warning(f"Failed to delete spatial map file: {e}")
        
        return True
    
    def get_cell(self, x: int, z: int) -> Optional[Dict[str, Any]]:
        """Get cell at coordinates, or None if not mapped."""
        key = self._cell_key(x, z)
        return self.cells.get(key)
    
    def set_cell(self, x: int, z: int, cell: Dict[str, Any]) -> None:
        """Set cell at coordinates."""
        key = self._cell_key(x, z)
        self.cells[key] = cell
    
    def is_blocked_from(self, current_x: int, current_z: int, current_y: int, target_x: int, target_z: int, target_y: int) -> bool:
        """
        Query-time computation: Is target cell blocked from current position?
        
        "Blocked" is relative - a cell may be blocked from position A but not from position B.
        This computes blocking based on stored obstruction data.
        
        Args:
            current_x, current_z, current_y: Current agent position
            target_x, target_z, target_y: Target cell position
            
        Returns:
            True if target cell is blocked from current position, False otherwise
        """
        # Get target cell
        target_cell = self.get_cell(target_x, target_z)
        if not target_cell:
            return False  # Unknown cell is not "blocked", it's "unknown"
        
        # Check if there's a solid obstruction block at target Y level
        obstructions = target_cell.get('obstructions', {})
        blocks_at_y = obstructions.get('blocks_at_y', {})
        block_at_target_y = blocks_at_y.get(target_y)
        
        if block_at_target_y:
            # Check if it's a solid block that would block movement
            block_class = classify_block(block_at_target_y)
            if block_class in ['stone', 'dirt', 'sand', 'wood', 'ore']:
                # There's a solid block at target Y - check if it blocks movement from current position
                # For cardinal movement, if there's a solid block at target Y, it's blocked
                dx = abs(target_x - current_x)
                dz = abs(target_z - current_z)
                
                # Cardinal movement (one cell away)
                if (dx == 1 and dz == 0) or (dx == 0 and dz == 1):
                    return True  # Solid block at target Y blocks cardinal movement
                
                # For diagonal movement, check if both cardinal paths are clear
                # (simplified - could be more sophisticated)
                if dx == 1 and dz == 1:
                    # Check intermediate cells
                    mid_x, mid_z = current_x + (1 if target_x > current_x else -1), current_z
                    mid_cell = self.get_cell(mid_x, mid_z)
                    if mid_cell:
                        mid_obstructions = mid_cell.get('obstructions', {})
                        mid_blocks = mid_obstructions.get('blocks_at_y', {})
                        if mid_blocks.get(target_y):
                            mid_block_class = classify_block(mid_blocks[target_y])
                            if mid_block_class in ['stone', 'dirt', 'sand', 'wood', 'ore']:
                                return True  # Blocked via intermediate cell
                    
                    mid_x, mid_z = current_x, current_z + (1 if target_z > current_z else -1)
                    mid_cell = self.get_cell(mid_x, mid_z)
                    if mid_cell:
                        mid_obstructions = mid_cell.get('obstructions', {})
                        mid_blocks = mid_obstructions.get('blocks_at_y', {})
                        if mid_blocks.get(target_y):
                            mid_block_class = classify_block(mid_blocks[target_y])
                            if mid_block_class in ['stone', 'dirt', 'sand', 'wood', 'ore']:
                                return True  # Blocked via intermediate cell
        
        return False  # No solid obstruction detected
    
    def add_waypoint(self, x: int, z: int, waypoint_name: str) -> bool:
        """
        Add a waypoint name to a cell.
        
        Args:
            x, z: Cell coordinates
            waypoint_name: Name of the waypoint to add
            
        Returns:
            True if waypoint was added, False if cell doesn't exist
        """
        key = self._cell_key(x, z)
        cell = self.cells.get(key)
        if not cell:
            return False
        
        if 'waypoints' not in cell:
            cell['waypoints'] = []
        
        if waypoint_name not in cell['waypoints']:
            cell['waypoints'].append(waypoint_name)
        
        return True
    
    def get_waypoints(self, x: int, z: int) -> List[str]:
        """
        Get waypoint names at a cell.
        
        Args:
            x, z: Cell coordinates
            
        Returns:
            List of waypoint names, empty list if cell doesn't exist or has no waypoints
        """
        key = self._cell_key(x, z)
        cell = self.cells.get(key)
        if not cell:
            return []
        return cell.get('waypoints', [])
    
    def update_cell_from_observation(
        self,
        observer_x: float,
        observer_y: float,
        observer_z: float,
        observation: Dict[str, Any],
        update_reason: str = "observation"
    ) -> int:
        """
        Update cells from mc-observe-blocks observation data.
        
        Args:
            observer_x, observer_y, observer_z: Observer position
            observation: Structured observation data from mc-observe-blocks
            update_reason: Reason for update (e.g., "observation", "inference")
            
        Returns:
            Number of cells updated
        """
        now = datetime.utcnow().isoformat()
        obs_x = int(round(observer_x))
        obs_y = int(round(observer_y))
        obs_z = int(round(observer_z))
        
        cells_updated = 0
        
        # Extract data from observation
        pose = observation.get('pose', {})
        yaw = pose.get('yaw', 0)
        support = observation.get('support', {})
        aff = observation.get('aff', {})
        blocks = observation.get('blocks', {})
        geom = observation.get('geom', {})
        conf = observation.get('conf', 'med')
        dirs = observation.get('dirs', {})
        
        # ---------------------------------------------------------------------
        # 1. Update observer's cell (Current Location)
        # ---------------------------------------------------------------------
        key = self._cell_key(obs_x, obs_z)
        cell = self.cells.get(key) or empty_cell(obs_x, obs_z)
        
        # Update support
        here_support = support.get('here', {})
        support_type = here_support.get('type')
        support_depth = here_support.get('depth')
        support_block = here_support.get('block')
        
        # Fallback to dirs.down when support.here is incomplete
        # This handles cases where support detection failed but dirs.down has reliable data
        dirs_down = dirs.get('down', {})
        dirs_down_dist = dirs_down.get('dist')
        dirs_down_blk = dirs_down.get('blk')
        
        # Infer support_y from support.here.depth OR dirs.down.dist
        if support_depth is not None:
            cell['support']['support_y'] = obs_y - support_depth
        elif dirs_down_dist is not None and dirs_down_dist > 0:
            # Fallback: use dirs.down.dist to infer support_y
            cell['support']['support_y'] = obs_y - dirs_down_dist
        
        # Infer walkable from support.here.type OR dirs.down.blk
        if support_type is not None:
            cell['support']['walkable'] = support_type == 'solid'
        elif dirs_down_blk and dirs_down_blk != 'air':
            # Fallback: if dirs.down shows a solid block, infer walkable
            # Check if it's a non-supporting block (snow layers, etc.)
            non_supporting_prefixes = ('minecraft:snow', 'minecraft:carpet', 'minecraft:pressure_plate', 
                                      'minecraft:farmland', 'minecraft:rail', 'minecraft:trapdoor')
            if not any(str(dirs_down_blk).startswith(ns) for ns in non_supporting_prefixes):
                # Check if it's solid
                block_class = classify_block(dirs_down_blk)
                if block_class in ['stone', 'dirt', 'sand', 'wood', 'ore']:
                    cell['support']['walkable'] = True
                    # Also infer support_type for movement_class computation
                    support_type = 'solid'
        
        # Note: delta_y_from_agent was removed - it's agent-relative data that becomes stale.
        # Use absolute support_y instead. Compute deltas at query time if needed.
        
        # Determine movement class only if we have support information
        if support_type is not None:
            if not cell['support']['walkable']:
                # Support detection failed or uncertain - mark as unknown, not blocked
                # "blocked" is reserved for detected obstructions (handled in section 2)
                cell['support']['movement_class'] = 'unknown'
            elif geom.get('pit'):
                # "drop" means cell has a pit/drop below it (absolute property)
                cell['support']['movement_class'] = 'drop'
            else:
                cell['support']['movement_class'] = 'flat'
        
        # Update surface from support.here.block OR dirs.down.blk
        if support_block is not None:
            cell['surface']['support_block'] = support_block
            cell['surface']['surface_block_class'] = classify_block(support_block)
            cell['surface']['is_fluid'] = is_fluid(support_block)
            cell['surface']['is_partial_block'] = is_partial_block(support_block)
        elif dirs_down_blk and dirs_down_blk != 'air':
            # Fallback: use dirs.down.blk for surface info
            # Only if it's not a non-supporting block (we want the actual support block)
            non_supporting_prefixes = ('minecraft:snow', 'minecraft:carpet', 'minecraft:pressure_plate', 
                                      'minecraft:farmland', 'minecraft:rail', 'minecraft:trapdoor')
            if not any(str(dirs_down_blk).startswith(ns) for ns in non_supporting_prefixes):
                cell['surface']['support_block'] = dirs_down_blk
                cell['surface']['surface_block_class'] = classify_block(dirs_down_blk)
                cell['surface']['is_fluid'] = is_fluid(dirs_down_blk)
                cell['surface']['is_partial_block'] = is_partial_block(dirs_down_blk)
        
        # Update environment only if we have sky information
        if 'sky' in aff:
            if aff.get('sky'):
                cell['environment']['light_level'] = 'high'
                cell['environment']['spawn_risk'] = 'low'
            else:
                cell['environment']['light_level'] = 'unknown'
                cell['environment']['spawn_risk'] = 'unknown'
        
        # Update observability
        cell['observability']['observed_from'] = {'x': obs_x, 'y': obs_y, 'z': obs_z}
        cell['observability']['observation_mode'] = 'direct'
        cell['observability']['confidence'] = {'high': 0.9, 'med': 0.7, 'low': 0.4}.get(conf, 0.5)
        cell['observability']['last_observed_at'] = now
        cell['provenance']['updated_by'] = self.agent_name
        cell['provenance']['update_reason'] = update_reason
        
        self.cells[key] = cell
        cells_updated += 1

        # ---------------------------------------------------------------------
        # 2. Update immediate neighbors from 'dirs' (Obstructions)
        # ---------------------------------------------------------------------
        # Determine cardinal facing to map 'left'/'right'/'back' to coordinates
        # 0=South(+Z), 90=West(-X), 180=North(-Z), 270=East(+X)
        norm_yaw = yaw % 360
        if norm_yaw < 0: norm_yaw += 360
        
        cardinal_offsets = {
            'fwd':   (0, 0), # Placeholder, handled by logic below
            'back':  (0, 0),
            'left':  (0, 0),
            'right': (0, 0)
        }
        
        if 315 <= norm_yaw or norm_yaw < 45:    # South (+Z)
            cardinal_offsets = {'fwd': (0, 1), 'back': (0, -1), 'left': (1, 0), 'right': (-1, 0)}
        elif 45 <= norm_yaw < 135:              # West (-X)
            cardinal_offsets = {'fwd': (-1, 0), 'back': (1, 0), 'left': (0, 1), 'right': (0, -1)}
        elif 135 <= norm_yaw < 225:             # North (-Z)
            cardinal_offsets = {'fwd': (0, -1), 'back': (0, 1), 'left': (-1, 0), 'right': (1, 0)}
        else:                                   # East (+X)
            cardinal_offsets = {'fwd': (1, 0), 'back': (-1, 0), 'left': (0, -1), 'right': (0, 1)}

        # Store obstruction data (blocks at positions) instead of marking cells as "blocked"
        # "Blocked" is now a query-time attribute computed relative to current position
        surface_y_for_obs = cell['support']['support_y']
        if surface_y_for_obs is not None:
            surface_y_for_obs = surface_y_for_obs + 1  # Surface = support_y + 1
        
        for dir_name in ['back', 'left', 'right']:
            dir_info = dirs.get(dir_name)
            if not dir_info: continue
            
            blk = dir_info.get('blk')
            # Store block information for query-time blocking computation
            if blk:
                dx, dz = cardinal_offsets[dir_name]
                nx, nz = obs_x + dx, obs_z + dz
                n_key = self._cell_key(nx, nz)
                
                n_cell = self.cells.get(n_key) or empty_cell(nx, nz)
                
                # Store block at surface Y level (or agent Y if surface_y unknown)
                # This allows query-time computation of "blocked" relative to any position
                block_y = surface_y_for_obs if surface_y_for_obs is not None else obs_y
                if 'obstructions' not in n_cell:
                    n_cell['obstructions'] = {'blocks_at_y': {}}
                n_cell['obstructions']['blocks_at_y'][block_y] = blk
                
                # If it's a solid block, we know there's an obstruction at this Y level
                # But don't mark as "blocked" - that's computed at query time
                if classify_block(blk) in ['stone', 'dirt', 'sand', 'wood', 'ore']:
                    # Store that this is a solid obstruction
                    n_cell['surface']['support_block'] = blk  # Store the wall block info
                
                # Metadata
                n_cell['observability']['observation_mode'] = 'inferred_obstruction'
                n_cell['observability']['last_observed_at'] = now
                n_cell['provenance']['update_reason'] = f"obstruction_{dir_name}"
                
                self.cells[n_key] = n_cell
                cells_updated += 1

        # ---------------------------------------------------------------------
        # 3. Update all visible cells (Resources & Hazards)
        # ---------------------------------------------------------------------
        # Iterate over all nearby blocks to populate resource/hazard maps
        # IMPORTANT: Only record resources/hazards that are on the surface (Y matches support_y)
        # This ensures we know the Y coordinate and maintain 3D surface view consistency
        nearby_blocks = blocks.get('nearby', [])
        for block in nearby_blocks:
            if not isinstance(block, dict): continue
            
            b_name = block.get('name', '')
            b_pos = block.get('position', {})
            
            # Normalize position
            bx, by, bz = 0, 0, 0
            if isinstance(b_pos, dict):
                bx, by, bz = b_pos.get('x'), b_pos.get('y'), b_pos.get('z')
            elif isinstance(b_pos, (list, tuple)) and len(b_pos) >= 3:
                bx, by, bz = b_pos[0], b_pos[1], b_pos[2]
            else:
                continue
                
            bx_int, bz_int = int(math.floor(bx)), int(math.floor(bz))
            b_key = self._cell_key(bx_int, bz_int)
            
            # Get or create cell
            b_cell = self.cells.get(b_key)
            
            # CRITICAL: Only record resources/hazards if we know the surface Y coordinate
            # Skip if cell doesn't exist or doesn't have support_y set
            # Note: With cone-based visibility, surface blocks behind taller obstacles may be occluded,
            # so some cells may not have support_y until observed from another angle. This is expected.
            if not b_cell or b_cell['support']['support_y'] is None:
                continue
            
            # Uniform surface definition:
            # - Surface Y = support_y + 1 (top face of the solid support block where agent stands)
            # - Resources on surface: blocks at support_y + 1 (crops, logs on surface)
            # - Resources embedded: blocks at support_y (ore in the support block itself)
            # - Do NOT record: blocks at support_y - 1 or below (underground resources)
            cell_support_y = b_cell['support']['support_y']
            by_int = int(math.floor(by))
            
            # Block must be at surface level (support_y + 1) or embedded in surface block (support_y)
            if by_int not in (cell_support_y, cell_support_y + 1):
                continue
            
            # Identify special properties
            is_hazard = b_name.replace('minecraft:', '') in HAZARD_BLOCKS
            
            # Identify resources
            resource_type = None
            harvest_action = None
            b_lower = b_name.lower()
            if 'ore' in b_lower:
                resource_type = 'ore'; harvest_action = 'mine'
            elif 'log' in b_lower or 'wood' in b_lower:
                resource_type = 'wood'; harvest_action = 'chop'
            elif 'crop' in b_lower or 'wheat' in b_lower or 'carrot' in b_lower:
                resource_type = 'food_source'; harvest_action = 'harvest'
                
            # Only update if we have something interesting
            if is_hazard or resource_type:
                modified = False
                
                if is_hazard:
                    # Map hazard name to flag
                    flag = 'unknown'
                    if 'lava' in b_lower: flag = 'lava'
                    elif 'fire' in b_lower: flag = 'fire'
                    elif 'magma' in b_lower: flag = 'magma'
                    elif 'berry' in b_lower: flag = 'sweet_berry'
                    elif 'cactus' in b_lower: flag = 'cactus'
                    
                    if flag not in b_cell['hazards']['flags']:
                        b_cell['hazards']['flags'].append(flag)
                        b_cell['hazards']['exposure_risk'] = 'high'
                        modified = True
                        
                if resource_type:
                    if resource_type not in b_cell['resources']['resources_visible']:
                        b_cell['resources']['resources_visible'].append(resource_type)
                        if harvest_action and harvest_action not in b_cell['resources']['harvest_actions']:
                            b_cell['resources']['harvest_actions'].append(harvest_action)
                        modified = True
                
                if modified:
                    # Don't overwrite higher quality observation data if existing
                    if b_cell['observability']['observation_mode'] == 'unknown':
                        b_cell['observability']['observation_mode'] = 'scanned'
                        b_cell['observability']['last_observed_at'] = now
                    
                    self.cells[b_key] = b_cell
        
        # ---------------------------------------------------------------------
        # 4. Forward Inference (Existing logic)
        # ---------------------------------------------------------------------
        fwd_support = support.get('fwd', {})
        if fwd_support:
            # Re-calculate fwd coords
            dx, dz = cardinal_offsets['fwd']
            fwd_x, fwd_z = obs_x + dx, obs_z + dz
            
            fwd_key = self._cell_key(fwd_x, fwd_z)
            fwd_cell = self.cells.get(fwd_key) or empty_cell(fwd_x, fwd_z)
            
            fwd_type = fwd_support.get('type')
            if fwd_type is not None:
                fwd_cell['support']['walkable'] = fwd_type == 'solid'
            
            fwd_block = fwd_support.get('block')
            
            # Forward support logic (depth calculation)
            # Preserve existing support_y if new data is missing
            fwd_depth = fwd_support.get('depth')
            if fwd_depth is not None:
                fwd_cell['support']['support_y'] = obs_y - fwd_depth
            elif fwd_type == 'solid' and cell['support'].get('support_y') is not None:
                # Only copy from observer cell if we have it
                fwd_cell['support']['support_y'] = cell['support'].get('support_y')
            
            # Note: delta_y_from_agent was removed - it's agent-relative data that becomes stale.
            # Use absolute support_y instead. Compute deltas at query time if needed.
            
            # Update surface only if we have block information
            if fwd_block is not None:
                fwd_cell['surface']['support_block'] = fwd_block
                fwd_cell['surface']['surface_block_class'] = classify_block(fwd_block)
                fwd_cell['surface']['is_fluid'] = is_fluid(fwd_block)
                fwd_cell['surface']['is_partial_block'] = is_partial_block(fwd_block)
            
            fwd_cell['observability']['observed_from'] = {'x': obs_x, 'y': obs_y, 'z': obs_z}
            fwd_cell['observability']['observation_mode'] = 'inferred'
            fwd_cell['observability']['confidence'] = max(0.3, cell['observability']['confidence'] - 0.2)
            fwd_cell['observability']['last_observed_at'] = now
            fwd_cell['provenance']['updated_by'] = self.agent_name
            fwd_cell['provenance']['update_reason'] = 'forward_inference'
            
            self.cells[fwd_key] = fwd_cell
            cells_updated += 1
        
        return cells_updated
    
    # =========================================================================
    # Query Methods
    # =========================================================================
    
    def get_all_cells(self) -> List[Dict[str, Any]]:
        """Get all cells as a list."""
        return list(self.cells.values())
    
    def cells_within_radius(self, cx: int, cz: int, radius: int) -> List[Dict[str, Any]]:
        """Get all cells within radius of center point."""
        results = []
        for key, cell in self.cells.items():
            x, z = key
            dist = math.sqrt((x - cx) ** 2 + (z - cz) ** 2)
            if dist <= radius:
                results.append(cell)
        return results
    
    def cells_matching(self, predicate: Callable[[Dict[str, Any]], bool]) -> List[Dict[str, Any]]:
        """Get all cells matching a predicate function."""
        return [cell for cell in self.cells.values() if predicate(cell)]
    
    # --- Coverage / Observability Queries ---
    
    def cells_unobserved(self, cx: int, cz: int, radius: int) -> List[Tuple[int, int]]:
        """
        Get coordinates of cells within radius that have no observation.
        Returns list of (x, z) tuples for unobserved positions.
        """
        unobserved = []
        for dx in range(-radius, radius + 1):
            for dz in range(-radius, radius + 1):
                x, z = cx + dx, cz + dz
                dist = math.sqrt(dx ** 2 + dz ** 2)
                if dist <= radius:
                    key = self._cell_key(x, z)
                    if key not in self.cells:
                        unobserved.append((x, z))
        return unobserved
    
    def cells_low_confidence(self, cx: int, cz: int, radius: int, threshold: float = 0.5) -> List[Dict[str, Any]]:
        """Get cells within radius with confidence below threshold."""
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            conf = cell.get('observability', {}).get('confidence', 0)
            if conf < threshold:
                results.append(cell)
        return results
    
    def frontier_cells(self, cx: int, cz: int, radius: int) -> List[Dict[str, Any]]:
        """Get observed cells within radius that are adjacent to unobserved cells."""
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            cell_id = cell.get('cell_id', {})
            x, z = cell_id.get('x', 0), cell_id.get('z', 0)
            
            # Check 4-connected neighbors
            neighbors = [(x-1, z), (x+1, z), (x, z-1), (x, z+1)]
            for nx, nz in neighbors:
                if self._cell_key(nx, nz) not in self.cells:
                    results.append(cell)
                    break
        return results
    
    def cells_observed_from_distance(self, min_dist: float) -> List[Dict[str, Any]]:
        """Get cells where observation was made from at least min_dist away."""
        results = []
        for cell in self.cells.values():
            cell_id = cell.get('cell_id', {})
            cx, cz = cell_id.get('x', 0), cell_id.get('z', 0)
            
            obs_from = cell.get('observability', {}).get('observed_from', {})
            ox = obs_from.get('x')
            oz = obs_from.get('z')
            
            if ox is not None and oz is not None:
                dist = math.sqrt((cx - ox) ** 2 + (cz - oz) ** 2)
                if dist >= min_dist:
                    results.append(cell)
        return results
    
    def cells_stale(self, max_age_seconds: float) -> List[Dict[str, Any]]:
        """Get cells with observations older than max_age_seconds."""
        now = datetime.utcnow()
        results = []
        for cell in self.cells.values():
            last_obs = cell.get('observability', {}).get('last_observed_at')
            if last_obs:
                try:
                    obs_time = datetime.fromisoformat(last_obs.replace('Z', '+00:00').replace('+00:00', ''))
                    age = (now - obs_time).total_seconds()
                    if age > max_age_seconds:
                        results.append(cell)
                except (ValueError, TypeError):
                    results.append(cell)  # Invalid timestamp = stale
            else:
                results.append(cell)  # No timestamp = stale
        return results
    
    # --- Reachability / Locomotion Queries ---
    
    def cells_reachable(self, cx: int, cz: int, radius: int, max_delta_y: int = 1, current_y: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Get walkable cells within radius and delta_y constraint.
        
        Note: "blocked" is now computed at query time. If current_y is provided,
        cells blocked from current position will be excluded.
        """
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            if not cell.get('support', {}).get('walkable'):
                continue
            
            movement_class = cell.get('support', {}).get('movement_class', 'unknown')
            # Don't filter by 'blocked' - that's query-time now
            if movement_class == 'unknown':
                continue
            
            # Check if blocked from current position (query-time)
            if current_y is not None:
                cell_id = cell.get('cell_id', {})
                cell_x, cell_z = cell_id.get('x', 0), cell_id.get('z', 0)
                support_y = cell.get('support', {}).get('support_y')
                if support_y is not None:
                    target_y = support_y + 1  # Surface Y
                    if self.is_blocked_from(cx, cz, current_y, cell_x, cell_z, target_y):
                        continue  # Blocked from current position
            
            # Check delta_y if current_y is provided (compute at query time)
            if current_y is not None:
                cell_support_y = cell.get('support', {}).get('support_y')
                if cell_support_y is not None:
                    cell_surface_y = cell_support_y + 1  # Surface = support_y + 1
                    delta_y = cell_surface_y - current_y
                    if abs(delta_y) > max_delta_y:
                        continue
            
            results.append(cell)
        return results
    
    def cells_blocked(self, from_x: Optional[int] = None, from_z: Optional[int] = None, from_y: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Get cells that are blocked or unwalkable.
        
        If from_x, from_z, from_y are provided, computes blocking relative to that position (query-time).
        Otherwise, returns all unwalkable cells (but not "blocked" since that's query-time now).
        """
        if from_x is not None and from_z is not None and from_y is not None:
            # Query-time blocking computation
            results = []
            for cell in self.cells.values():
                cell_id = cell.get('cell_id', {})
                cell_x, cell_z = cell_id.get('x', 0), cell_id.get('z', 0)
                support_y = cell.get('support', {}).get('support_y')
                if support_y is not None:
                    target_y = support_y + 1  # Surface Y
                    if self.is_blocked_from(from_x, from_z, from_y, cell_x, cell_z, target_y):
                        results.append(cell)
            return results
        else:
            # Return unwalkable cells (but "blocked" is now query-time, so don't check movement_class)
            return self.cells_matching(
                lambda c: not c.get('support', {}).get('walkable', False)
            )
    
    def cells_requiring_climb(self, from_y: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Get cells reachable only via upward movement.
        
        If from_y is provided, computes step_up relative to that Y coordinate (query-time).
        Otherwise, returns empty list (step_up is now query-time only).
        """
        if from_y is not None:
            # Query-time computation: check if cell surface is above from_y
            results = []
            for cell in self.cells.values():
                if not cell.get('support', {}).get('walkable'):
                    continue
                cell_support_y = cell.get('support', {}).get('support_y')
                if cell_support_y is not None:
                    cell_surface_y = cell_support_y + 1  # Surface = support_y + 1
                    if cell_surface_y > from_y:  # Requires upward movement
                        results.append(cell)
            return results
        else:
            # step_up is now query-time only - return empty
            return []
    
    def cells_with_drop_risk(self, from_y: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Get cells involving unsafe descent.
        
        If from_y is provided, computes drop risk relative to that Y coordinate (query-time).
        Otherwise, only checks movement_class == 'drop'.
        """
        if from_y is not None:
            # Query-time computation: check if cell surface is significantly below from_y
            results = []
            for cell in self.cells.values():
                if cell.get('support', {}).get('movement_class') == 'drop':
                    results.append(cell)
                    continue
                
                cell_support_y = cell.get('support', {}).get('support_y')
                if cell_support_y is not None:
                    cell_surface_y = cell_support_y + 1  # Surface = support_y + 1
                    delta_y = cell_surface_y - from_y
                    if delta_y < -1:  # More than 1 block below
                        results.append(cell)
            return results
        else:
            # Only check movement_class
            return self.cells_matching(
                lambda c: c.get('support', {}).get('movement_class') == 'drop'
            )
    
    # --- Safety / Survival Queries ---
    
    def cells_safe_to_stand(self, cx: int, cz: int, radius: int) -> List[Dict[str, Any]]:
        """Get cells within radius that are safe to stand on."""
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            # Must be walkable
            if not cell.get('support', {}).get('walkable'):
                continue
            
            # Must not have hazards
            hazard_flags = cell.get('hazards', {}).get('flags', [])
            if hazard_flags:
                continue
            
            # Must have low/medium exposure risk
            exposure = cell.get('hazards', {}).get('exposure_risk', 'unknown')
            if exposure == 'high':
                continue
            
            results.append(cell)
        return results
    
    def cells_high_hazard(self) -> List[Dict[str, Any]]:
        """Get all cells with hazards (lava, void risk, etc.)."""
        return self.cells_matching(
            lambda c: (
                len(c.get('hazards', {}).get('flags', [])) > 0 or
                c.get('hazards', {}).get('exposure_risk') == 'high'
            )
        )
    
    def cells_escape_nodes(self, cx: int, cz: int, radius: int) -> List[Dict[str, Any]]:
        """Get cells that are good retreat/recovery positions."""
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            # Must be walkable and safe
            if not cell.get('support', {}).get('walkable'):
                continue
            
            hazard_flags = cell.get('hazards', {}).get('flags', [])
            if hazard_flags:
                continue
            
            # Must have easy escape
            escape_diff = cell.get('hazards', {}).get('escape_difficulty', 'unknown')
            if escape_diff not in ['easy', 'medium']:
                continue
            
            # Prefer cells with good visibility (sky access)
            light = cell.get('environment', {}).get('light_level', 'unknown')
            if light == 'high':
                results.append(cell)
            elif escape_diff == 'easy':
                results.append(cell)
        
        return results
    
    # --- Resource Queries ---
    
    def cells_with_resource(self, resource_type: str) -> List[Dict[str, Any]]:
        """Get cells with visible resources of specified type."""
        return self.cells_matching(
            lambda c: resource_type in c.get('resources', {}).get('resources_visible', [])
        )
    
    def cells_harvestable_now(self, available_tools: List[str] = None) -> List[Dict[str, Any]]:
        """Get cells with resources harvestable with current tools."""
        if available_tools is None:
            available_tools = ['none', 'hand']
        
        return self.cells_matching(
            lambda c: (
                len(c.get('resources', {}).get('resources_visible', [])) > 0 and
                c.get('resources', {}).get('tool_required', 'unknown') in available_tools
            )
        )
    
    def cells_water_source(self) -> List[Dict[str, Any]]:
        """Get cells with water."""
        return self.cells_matching(
            lambda c: (
                c.get('surface', {}).get('surface_block_class') == 'water' or
                c.get('surface', {}).get('is_fluid') and 'water' in str(c.get('surface', {}).get('support_block', '')).lower()
            )
        )
    
    # --- Multi-Objective Queries ---
    
    def cells_candidate_waypoints(
        self,
        cx: int,
        cz: int,
        radius: int,
        require_safe: bool = True,
        require_reachable: bool = True
    ) -> List[Dict[str, Any]]:
        """Get cells that are good waypoint candidates."""
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            # Check walkable
            if not cell.get('support', {}).get('walkable'):
                continue
            
            # Check safety
            if require_safe:
                hazards = cell.get('hazards', {}).get('flags', [])
                if hazards:
                    continue
            
            # Check reachability
            # Note: "blocked" is now query-time, so we only check for 'unknown'
            if require_reachable:
                movement = cell.get('support', {}).get('movement_class', 'unknown')
                if movement == 'unknown':
                    continue
            
            results.append(cell)
        return results
    
    def cells_nearest(
        self,
        cx: int,
        cz: int,
        predicate: Callable[[Dict[str, Any]], bool],
        max_results: int = 10
    ) -> List[Tuple[float, Dict[str, Any]]]:
        """Get nearest cells matching predicate, sorted by distance."""
        matches = []
        for cell in self.cells.values():
            if predicate(cell):
                cell_id = cell.get('cell_id', {})
                x, z = cell_id.get('x', 0), cell_id.get('z', 0)
                dist = math.sqrt((x - cx) ** 2 + (z - cz) ** 2)
                matches.append((dist, cell))
        
        matches.sort(key=lambda x: x[0])
        return matches[:max_results]
    
    def cells_worth_revisit(self, cx: int, cz: int, radius: int, max_confidence: float = 0.6) -> List[Dict[str, Any]]:
        """Get cells that should be revisited (low confidence, inferred, or risky)."""
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            # Low confidence
            conf = cell.get('observability', {}).get('confidence', 0)
            if conf < max_confidence:
                results.append(cell)
                continue
            
            # Inferred (not direct observation)
            obs_mode = cell.get('observability', {}).get('observation_mode', 'unknown')
            if obs_mode in ['inferred', 'propagated', 'unknown']:
                results.append(cell)
                continue
            
            # Has hazards but uncertain
            if cell.get('hazards', {}).get('exposure_risk') == 'unknown':
                results.append(cell)
        
        return results
    
    # =========================================================================
    # Statistics
    # =========================================================================
    
    def get_stats(self) -> Dict[str, Any]:
        """Get map statistics."""
        if not self.cells:
            return {
                "cell_count": 0,
                "bounds": None,
                "walkable_count": 0,
                "hazard_count": 0,
                "resource_count": 0
            }
        
        min_x = min_z = float('inf')
        max_x = max_z = float('-inf')
        walkable = 0
        hazard = 0
        resource = 0
        
        for key, cell in self.cells.items():
            x, z = key
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_z = min(min_z, z)
            max_z = max(max_z, z)
            
            if cell.get('support', {}).get('walkable'):
                walkable += 1
            if cell.get('hazards', {}).get('flags'):
                hazard += 1
            if cell.get('resources', {}).get('resources_visible'):
                resource += 1
        
        return {
            "cell_count": len(self.cells),
            "bounds": {
                "min_x": min_x, "max_x": max_x,
                "min_z": min_z, "max_z": max_z,
                "range_x": max_x - min_x + 1,
                "range_z": max_z - min_z + 1
            },
            "walkable_count": walkable,
            "hazard_count": hazard,
            "resource_count": resource
        }
