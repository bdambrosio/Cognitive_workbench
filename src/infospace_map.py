"""
infospace_map.py - Information space map implementation

This module implements InfospaceMap, a semantic/abstract space for representing
conceptual resources like skills, knowledge, or information. Unlike physical 
geography, information spaces have:
- Uniform terrain (no elevation, water, mountains)
- No properties or ownership
- Minimal infrastructure
- Flat visibility (all resources visible from any position)
"""

from space_map import SpaceMap, Patch
from typing import List
import xml.etree.ElementTree as ET


class InfospaceMap(SpaceMap):
    """
    Information space map - a flat semantic space for conceptual resources.
    
    Designed for small grids where agents can "navigate" through information
    but visibility and movement have different semantics than physical space.
    """
    
    def __init__(self, scenario_module, width=10, height=10):
        """
        Initialize information space.
        
        Args:
            scenario_module: Infospace module with InfospaceTerrain, resources, etc.
            width: Grid width (default 10 - small for info spaces)
            height: Grid height (default 10)
        """
        # Initialize parent
        super().__init__(scenario_module, width, height)
    
    def generate_terrain(self):
        """
        Generate uniform InfoSpace terrain.
        
        Information spaces have no elevation variation, water, or geographic features.
        All patches are uniform InfoSpace type.
        """
        # Get the single InfoSpace terrain type
        infospace_terrain = None
        for member in self.terrain_types:
            if member.name == 'InfoSpace':
                infospace_terrain = member
                break
        
        if infospace_terrain is None:
            raise ValueError("InfospaceTerrain must define 'InfoSpace' terrain type")
        
        # Set all patches to InfoSpace terrain
        for x in range(self.width):
            for y in range(self.height):
                self.patches[x][y].terrain_type = infospace_terrain
                self.patches[x][y].elevation = 0.0  # Flat space
                self.patches[x][y].has_water = False
    
    def generate_properties(self):
        """
        Skip property generation.
        
        Information spaces have no property ownership concept.
        """
        # Check if properties are disabled
        if self._property_rules:
            max_size = self._property_rules.get('max_size', 0)
            if max_size == 0:
                # Properties explicitly disabled
                return
        
        # Otherwise do nothing - no properties in information space
        pass
    
    def generate_infrastructure(self):
        """
        Skip infrastructure generation.
        
        Information spaces have no roads or paths.
        Navigation is conceptual, not physical.
        """
        # Check if infrastructure is explicitly None
        if self._infrastructure_rules is None or self.infrastructure_types is None:
            return
        
        # Otherwise do nothing - no infrastructure in information space
        pass
    
    def get_visibility(self, x: int, y: int, observer_height: int) -> List[Patch]:
        """
        Get visible patches from a position in information space.
        
        In information space, "visibility" means conceptual accessibility.
        For simplicity, we implement local visibility: agent can see nearby
        patches in a radius, simulating "conceptual proximity".
        
        Args:
            x, y: Observer position
            observer_height: Ignored in information space
            
        Returns:
            List of conceptually "visible" patches
        """
        visible = []
        
        # Define visibility radius (conceptual proximity)
        # In a small info space, we might see everything
        visibility_radius = min(self.width, self.height, 8)
        
        for px in range(max(0, x - visibility_radius), min(self.width, x + visibility_radius + 1)):
            for py in range(max(0, y - visibility_radius), min(self.height, y + visibility_radius + 1)):
                visible.append(self.patches[px][py])
        
        return visible
    
    def get_map_summary(self) -> str:
        """
        Generate a summary of the information space.
        
        Returns:
            Human-readable summary string
        """
        summary_parts = []
        
        summary_parts.append(f"=== Information Space Map Summary ===")
        summary_parts.append(f"Dimensions: {self.width}x{self.height}")
        summary_parts.append(f"Type: Semantic/Abstract Space")
        summary_parts.append(f"")
        
        # Terrain summary
        summary_parts.append(f"Terrain: Uniform InfoSpace (no elevation, no water)")
        summary_parts.append(f"")
        
        # Resource summary
        summary_parts.append(f"Resources: {len(self.resource_registry)} total")
        
        # Group resources by type
        resource_types = {}
        for resource_id, resource in self.resource_registry.items():
            type_name = resource['type'].name if hasattr(resource['type'], 'name') else str(resource['type'])
            if type_name not in resource_types:
                resource_types[type_name] = []
            resource_types[type_name].append(resource)
        
        for type_name, resources in resource_types.items():
            summary_parts.append(f"  - {type_name}: {len(resources)} instances")
            
            # For tools, show names
            for resource in resources[:5]:  # Show first 5
                if 'tool_name' in resource['properties']:
                    tool_name = resource['properties']['tool_name']
                    summary_parts.append(f"    • {tool_name}")
            
            if len(resources) > 5:
                summary_parts.append(f"    ... and {len(resources) - 5} more")
        
        summary_parts.append(f"")
        
        # Agent summary
        summary_parts.append(f"Agents: {len(self.agents)}")
        for agent in self.agents:
            summary_parts.append(f"  - {agent.name} at ({agent.x}, {agent.y})")
        
        summary_parts.append(f"")
        summary_parts.append(f"Note: This is an abstract information space.")
        summary_parts.append(f"Movement and visibility have conceptual rather than physical semantics.")
        
        return "\n".join(summary_parts)
    
    def get_movement_cost(self, x, y):
        """
        Get movement cost for a patch in information space.
        
        In information space, movement is uniform and conceptual.
        All patches have the same traversal cost.
        
        Args:
            x, y: Patch coordinates
            
        Returns:
            Movement cost (always 1.0 for uniform space)
        """
        return 1.0
    
    def is_visible(self, x1, y1, x2, y2, observer_height):
        """
        Check if position 2 is visible from position 1.
        
        In information space, there's no line-of-sight blocking.
        Visibility is based only on conceptual distance.
        
        Args:
            x1, y1: Observer position
            x2, y2: Target position
            observer_height: Ignored in information space
            
        Returns:
            True if visible, False otherwise
        """
        # Simple distance-based visibility
        distance = abs(x2 - x1) + abs(y2 - y1)  # Manhattan distance
        visibility_range = min(self.width, self.height, 8)
        return distance <= visibility_range
    
    def get_visible_agents(self, x, y, observer_height):
        """
        Get agents visible from a position.
        
        Args:
            x, y: Observer position
            observer_height: Ignored in information space
            
        Returns:
            List of visible agents
        """
        visible_patches = self.get_visibility(x, y, observer_height)
        visible_patch_coords = {(p.x, p.y) for p in visible_patches}
        
        return [agent for agent in self.agents 
                if (agent.x, agent.y) in visible_patch_coords]
    
    def get_detailed_visibility_description(self, x: int, y: int, observer, observer_height: int) -> str:
        """
        Generate detailed visibility description for information space.
        
        This method delegates to the module-level function for backward compatibility.
        
        Args:
            x, y: Observer position
            observer: Observer agent (MapAgent instance)
            observer_height: Ignored in information space
            
        Returns:
            XML string describing visible resources and agents
        """
        return get_detailed_visibility_description_impl(self, x, y, observer, observer_height)


def manhattan_distance(x1, y1, x2, y2):
    """Calculate Manhattan distance between two points"""
    return abs(x2 - x1) + abs(y2 - y1)


def get_direction_name(dx, dy):
    """Get direction name from delta coordinates"""
    if dx == 0 and dy == 0:
        return "Current"
    
    # Normalize to -1, 0, 1
    ndx = 0 if dx == 0 else (1 if dx > 0 else -1)
    ndy = 0 if dy == 0 else (1 if dy > 0 else -1)
    
    direction_map = {
        (0, 1): "North",
        (1, 1): "Northeast",
        (1, 0): "East",
        (1, -1): "Southeast",
        (0, -1): "South",
        (-1, -1): "Southwest",
        (-1, 0): "West",
        (-1, 1): "Northwest"
    }
    
    return direction_map.get((ndx, ndy), "Current")


def get_detailed_visibility_description_impl(world: InfospaceMap, camera_x: int, camera_y: int, observer, observer_height: int):
    """
    Generate detailed visibility description for information space.
    
    This function produces XML output compatible with the physical map version,
    but adapted for information space semantics.
    
    Args:
        world: InfospaceMap instance
        camera_x, camera_y: Observer position
        observer: Observer agent (may be None)
        observer_height: Ignored in information space
        
    Returns:
        XML string describing visible resources and agents
    """
    from map_agent import Direction
    
    visible_patches = world.get_visibility(camera_x, camera_y, observer_height)
    if world.patches[camera_x][camera_y] not in visible_patches:
        visible_patches.append(world.patches[camera_x][camera_y])
    
    root = ET.Element("visibility_report")
    pos = ET.SubElement(root, "position")
    pos.set("x", str(camera_x))
    pos.set("y", str(camera_y))
    pos.text = ""
    
    for direction in Direction:
        direction_element = ET.SubElement(root, direction.name)
        
        if direction == Direction.Current:
            # Include current patch and immediate neighborhood
            direction_patches = [p for p in visible_patches 
                               if manhattan_distance(camera_x, camera_y, p.x, p.y) <= 1]
        else:
            # Get patches in this direction
            direction_patches = [p for p in visible_patches 
                               if (abs(p.x - camera_x) <= 16 and abs(p.y - camera_y) <= 16) 
                               and get_direction_name(p.x - camera_x, p.y - camera_y) == direction.name]
        
        if not direction_patches:
            ET.SubElement(direction_element, "visibility").text = "No clear visibility"
            continue
        
        max_distance = max(manhattan_distance(camera_x, camera_y, p.x, p.y) 
                          for p in direction_patches)
        adjacent_patch = min(direction_patches, 
                           key=lambda p: manhattan_distance(camera_x, camera_y, p.x, p.y))
        
        ET.SubElement(direction_element, "visibility").text = str(max_distance)
        ET.SubElement(direction_element, "terrain").text = "InfoSpace"
        ET.SubElement(direction_element, "property").text = "None"
        
        # No slope in information space
        slope_element = ET.SubElement(direction_element, "slope")
        slope_element.text = "flat"
        
        # List resources (skills) in this direction
        resources_element = ET.SubElement(direction_element, "resources")
        for patch in direction_patches:
            distance = manhattan_distance(camera_x, camera_y, patch.x, patch.y)
            for resource_id in patch.resources:
                resource_element = ET.SubElement(resources_element, "resource")
                resource_element.set("id", resource_id)
                resource_element.set("distance", str(distance))
                
                # Add tool name if available
                resource_data = world.resource_registry.get(resource_id)
                if resource_data and 'tool_name' in resource_data['properties']:
                    resource_element.set("tool_name", 
                                       resource_data['properties']['tool_name'])
                
                resource_element.text = ""
        
        # No water in information space
        # No paths in information space
        
        # List visible agents
        visible_agents = [agent for agent in world.agents
                         if world.patches[agent.x][agent.y] in direction_patches 
                         and agent != observer]
        if visible_agents:
            agents_element = ET.SubElement(direction_element, "characters")
            for agent in visible_agents:
                agent_element = ET.SubElement(agents_element, "character")
                agent_element.set("name", agent.name)
                agent_element.set("distance", 
                                str(manhattan_distance(camera_x, camera_y, agent.x, agent.y)))
                agent_element.text = ""
    
    return ET.tostring(root, encoding="unicode")


# For compatibility with map.py function calls
def get_slope_description(elevation_change):
    """Information space has no elevation"""
    return "flat"


# Backward compatibility: allow module-level function calls
def get_detailed_visibility_description(world: InfospaceMap, camera_x: int, camera_y: int, observer, observer_height: int):
    """
    Backward compatibility wrapper.
    
    Delegates to implementation function.
    """
    return get_detailed_visibility_description_impl(world, camera_x, camera_y, observer, observer_height)

