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
            "delta_y_from_agent": None,
            "movement_class": "unknown"
        },
        "surface": {
            "support_block": None,
            "surface_block_class": "unknown",
            "is_fluid": False,
            "is_partial_block": False
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
        
        # Update observer's cell (primary observation)
        key = self._cell_key(obs_x, obs_z)
        cell = self.cells.get(key) or empty_cell(obs_x, obs_z)
        
        # Extract data from observation
        support = observation.get('support', {})
        aff = observation.get('aff', {})
        blocks = observation.get('blocks', {})
        geom = observation.get('geom', {})
        conf = observation.get('conf', 'med')
        
        # Update support
        here_support = support.get('here', {})
        cell['support']['walkable'] = here_support.get('type') == 'solid'
        cell['support']['support_y'] = obs_y - 1 if here_support.get('depth') == 1 else obs_y - (here_support.get('depth') or 1)
        cell['support']['delta_y_from_agent'] = 0  # Observer is at this cell
        
        # Determine movement class from affordances and geometry
        if not cell['support']['walkable']:
            cell['support']['movement_class'] = 'blocked'
        elif geom.get('stair'):
            cell['support']['movement_class'] = 'step_up'
        elif geom.get('pit'):
            cell['support']['movement_class'] = 'drop'
        else:
            cell['support']['movement_class'] = 'flat'
        
        # Update surface
        support_block = here_support.get('block')
        cell['surface']['support_block'] = support_block
        cell['surface']['surface_block_class'] = classify_block(support_block)
        cell['surface']['is_fluid'] = is_fluid(support_block)
        cell['surface']['is_partial_block'] = is_partial_block(support_block)
        
        # Update hazards
        cell['hazards']['flags'] = extract_hazard_flags(blocks)
        if cell['hazards']['flags']:
            cell['hazards']['exposure_risk'] = 'high'
        elif not cell['support']['walkable']:
            cell['hazards']['exposure_risk'] = 'medium'
        else:
            cell['hazards']['exposure_risk'] = 'low'
        
        # Escape difficulty based on clearance and affordances
        if aff.get('step') or aff.get('jump'):
            cell['hazards']['escape_difficulty'] = 'easy'
        elif aff.get('descend'):
            cell['hazards']['escape_difficulty'] = 'medium'
        else:
            cell['hazards']['escape_difficulty'] = 'hard'
        
        # Update resources (extract from seen blocks)
        seen_blocks = blocks.get('seen', [])
        resources = []
        harvest_actions = []
        for block in seen_blocks:
            block_lower = str(block).lower()
            if 'ore' in block_lower:
                resources.append('ore')
                harvest_actions.append('mine')
            if 'log' in block_lower or 'wood' in block_lower:
                resources.append('wood')
                harvest_actions.append('chop')
            if 'crop' in block_lower or 'wheat' in block_lower or 'carrot' in block_lower:
                resources.append('food_source')
                harvest_actions.append('harvest')
        cell['resources']['resources_visible'] = list(set(resources))
        cell['resources']['harvest_actions'] = list(set(harvest_actions))
        
        # Update environment
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
        
        # Update provenance
        cell['provenance']['updated_by'] = self.agent_name
        cell['provenance']['update_reason'] = update_reason
        
        self.cells[key] = cell
        cells_updated += 1
        
        # Update forward cell if we have forward support info
        fwd_support = support.get('fwd', {})
        if fwd_support:
            # Calculate forward cell based on yaw (simplified: assume cardinal)
            pose = observation.get('pose', {})
            yaw = pose.get('yaw', 0)
            
            # Cardinal direction offsets
            if -45 <= yaw < 45 or yaw >= 315 or yaw < -315:  # North (negative Z)
                fwd_x, fwd_z = obs_x, obs_z - 1
            elif 45 <= yaw < 135 or -315 <= yaw < -225:  # West (negative X)
                fwd_x, fwd_z = obs_x - 1, obs_z
            elif 135 <= yaw < 225 or -225 <= yaw < -135:  # South (positive Z)
                fwd_x, fwd_z = obs_x, obs_z + 1
            else:  # East (positive X)
                fwd_x, fwd_z = obs_x + 1, obs_z
            
            fwd_key = self._cell_key(fwd_x, fwd_z)
            fwd_cell = self.cells.get(fwd_key) or empty_cell(fwd_x, fwd_z)
            
            # Update forward cell with inferred data
            fwd_cell['support']['walkable'] = fwd_support.get('type') == 'solid'
            fwd_block = fwd_support.get('block')
            
            # Calculate forward cell support_y based on forward support depth
            # Forward support depth indicates how many blocks down from forward position to find solid support
            fwd_depth = fwd_support.get('depth')
            if fwd_depth is not None:
                # Estimate forward cell support_y: observer Y minus forward support depth
                # This assumes forward cell is approximately at observer's Y level
                fwd_cell['support']['support_y'] = obs_y - fwd_depth
            else:
                # If depth unknown, estimate based on forward support type
                if fwd_support.get('type') == 'solid':
                    # Assume same height as observer's cell if solid
                    fwd_cell['support']['support_y'] = cell['support'].get('support_y')
                else:
                    fwd_cell['support']['support_y'] = None
            
            # Calculate delta_y_from_agent for forward cell
            observer_support_y = cell['support'].get('support_y')
            if observer_support_y is not None and fwd_cell['support']['support_y'] is not None:
                fwd_cell['support']['delta_y_from_agent'] = fwd_cell['support']['support_y'] - observer_support_y
            else:
                fwd_cell['support']['delta_y_from_agent'] = None
            
            fwd_cell['surface']['support_block'] = fwd_block
            fwd_cell['surface']['surface_block_class'] = classify_block(fwd_block)
            fwd_cell['surface']['is_fluid'] = is_fluid(fwd_block)
            fwd_cell['surface']['is_partial_block'] = is_partial_block(fwd_block)
            
            # Inferred observation
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
    
    def cells_reachable(self, cx: int, cz: int, radius: int, max_delta_y: int = 1) -> List[Dict[str, Any]]:
        """Get walkable cells within radius and delta_y constraint."""
        results = []
        for cell in self.cells_within_radius(cx, cz, radius):
            if not cell.get('support', {}).get('walkable'):
                continue
            
            movement_class = cell.get('support', {}).get('movement_class', 'unknown')
            if movement_class in ['blocked', 'unknown']:
                continue
            
            # Check delta_y if available
            delta_y = cell.get('support', {}).get('delta_y_from_agent')
            if delta_y is not None and abs(delta_y) > max_delta_y:
                continue
            
            results.append(cell)
        return results
    
    def cells_blocked(self) -> List[Dict[str, Any]]:
        """Get all cells marked as blocked or unwalkable."""
        return self.cells_matching(
            lambda c: (
                c.get('support', {}).get('movement_class') == 'blocked' or
                not c.get('support', {}).get('walkable', False)
            )
        )
    
    def cells_requiring_climb(self) -> List[Dict[str, Any]]:
        """Get cells reachable only via upward movement."""
        return self.cells_matching(
            lambda c: c.get('support', {}).get('movement_class') == 'step_up'
        )
    
    def cells_with_drop_risk(self) -> List[Dict[str, Any]]:
        """Get cells involving unsafe descent."""
        return self.cells_matching(
            lambda c: (
                c.get('support', {}).get('movement_class') == 'drop' or
                (c.get('support', {}).get('delta_y_from_agent') or 0) < -1
            )
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
            if require_reachable:
                movement = cell.get('support', {}).get('movement_class', 'unknown')
                if movement in ['blocked', 'unknown']:
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
