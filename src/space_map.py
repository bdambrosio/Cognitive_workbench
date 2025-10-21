"""
space_map.py - Base class for all spatial map types

This module provides the abstract SpaceMap base class that contains functionality
common to all spatial representations (physical geography, information spaces, 
social networks, etc.). It is designed to be extended by specific map types.

Architecture:
    SpaceMap (base - this file)
        ├── WorldMap (physical geography - map.py)
        └── InfospaceMap (semantic/abstract space - infospace_map.py)
"""

from datetime import datetime
import random
from enum import Enum
from typing import List, Tuple, Dict, Any, Optional
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod


class Patch:
    """
    Represents a single location/patch in any space.
    Shared by all map types - physical and abstract.
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.elevation = 0.0
        self.has_water = False
        self.has_path = False
        self.height = 0
        self.property_id = None
        self.resource_type = None
        self.terrain_type = None
        self.infrastructure_type = None
        self.property_type = None
        self.resources = {}  # Dict of resource_id -> resource data

    def get_slope(self):
        """Calculate slope - may not be meaningful for all space types"""
        if not hasattr(self, '_slope'):
            self._slope = 0.0
        return self._slope


class SpaceMap(ABC):
    """
    Abstract base class for all spatial map types.
    
    Provides common functionality:
    - Patch grid management
    - Agent registration
    - Resource management
    - Basic spatial queries
    
    Subclasses must implement:
    - generate_terrain()
    - generate_properties()
    - generate_infrastructure()
    - get_visibility()
    - get_map_summary()
    """
    
    def __init__(self, scenario_module, width=40, height=40):
        """
        Initialize the space with common attributes.
        
        Args:
            scenario_module: Module defining terrain_types, resource_types, rules, etc.
            width: Width of the space in patches
            height: Height of the space in patches
        """
        self.width = width
        self.height = height
        self.scenario_module = scenario_module
        self.datetime = datetime.now()
        
        # Get types from scenario module
        self.terrain_types = scenario_module.terrain_types
        self.infrastructure_types = getattr(scenario_module, 'infrastructure_types', None)
        self.property_types = getattr(scenario_module, 'property_types', None)
        self.resource_types = scenario_module.resource_types
        
        # Get rules from scenario module
        self._terrain_rules = scenario_module.terrain_rules
        self._infrastructure_rules = getattr(scenario_module, 'infrastructure_rules', None)
        self._property_rules = getattr(scenario_module, 'property_rules', None)
        self._resource_rules = scenario_module.resource_rules
        
        # Initialize core data structures
        self.resource_registry = {}
        self._resource_counters = {}  # Track counters for each resource type
        self.property_registry = {}  # Store property data
        self.agents = []
        
        # Create patch grid
        self.patches = [[Patch(x, y) for y in range(height)] for x in range(width)]
        
        # Call generation methods (subclasses implement these)
        self.generate_terrain()
        self.generate_properties()
        self.generate_resources()
        self.generate_infrastructure()
        
        print(f"{self.__class__.__name__} initialized")

    # =============================================================================
    # Abstract methods - must be implemented by subclasses
    # =============================================================================
    
    @abstractmethod
    def generate_terrain(self):
        """Generate terrain types for all patches"""
        pass
    
    @abstractmethod
    def generate_properties(self):
        """Generate properties/ownership zones"""
        pass
    
    @abstractmethod
    def generate_infrastructure(self):
        """Generate infrastructure (roads, paths, etc.)"""
        pass
    
    @abstractmethod
    def get_visibility(self, x: int, y: int, observer_height: int) -> List[Patch]:
        """
        Get list of patches visible from given position.
        
        Args:
            x, y: Observer position
            observer_height: Height of observer
            
        Returns:
            List of visible Patch objects
        """
        pass
    
    @abstractmethod
    def get_map_summary(self) -> str:
        """Return a summary description of the map"""
        pass
    
    @abstractmethod
    def get_detailed_visibility_description(self, x: int, y: int, observer, observer_height: int) -> str:
        """
        Generate detailed visibility description for agent at position.
        
        Args:
            x, y: Observer position
            observer: Observer agent (MapAgent instance)
            observer_height: Height of observer (affects visibility in physical spaces)
            
        Returns:
            XML string describing visible patches, resources, and agents
        """
        pass

    # =============================================================================
    # Agent management - common to all space types
    # =============================================================================
    
    def register_agent(self, agent):
        """Register an agent in this space"""
        if agent not in self.agents:
            self.agents.append(agent)
    
    def unregister_agent(self, agent):
        """Unregister an agent from this space"""
        if agent in self.agents:
            self.agents.remove(agent)
    
    def get_agent(self, name):
        """Get agent by name"""
        for agent in self.agents:
            if agent.name == name:
                return agent
        return None

    # =============================================================================
    # Resource management - common to all space types
    # =============================================================================
    
    def _generate_resource_id(self, resource_type, custom_name=None):
        """
        Generate unique ID for a resource.
        
        Args:
            resource_type: The type of resource (enum)
            custom_name: Optional custom name to use instead of TypeName+Number
            
        Returns:
            Resource ID string
        """
        # If custom name provided, use it directly
        if custom_name:
            return custom_name
        
        # Otherwise use TypeName+Number pattern
        if resource_type not in self._resource_counters:
            self._resource_counters[resource_type] = 0
        
        self._resource_counters[resource_type] += 1
        counter = self._resource_counters[resource_type]
        
        # Get resource type name
        type_name = resource_type.name if hasattr(resource_type, 'name') else str(resource_type)
        return f"{type_name}{counter}"
    
    def generate_resources(self):
        """
        Place resources according to resource_rules.
        This is mostly common across map types, but can be overridden.
        """
        if not self._resource_rules or 'allocations' not in self._resource_rules:
            return
        
        allocations = self._resource_rules['allocations']
        
        for allocation in allocations:
            resource_type = allocation['resource_type']
            count = allocation.get('count', 1)
            description = allocation.get('description', '')
            requires_property = allocation.get('requires_property', False)
            remove_on_take = allocation.get('remove_on_take', True)
            terrain_weights = allocation.get('terrain_weights', {})
            
            # Place each instance
            for i in range(count):
                # Find suitable location based on terrain weights
                x, y = self._find_resource_location(terrain_weights, requires_property)
                
                if x is None:
                    print(f"Warning: Could not place {resource_type}")
                    continue
                
                # Generate resource ID - use custom name if provided (e.g., skill_name for infospace)
                custom_name = allocation.get('skill_name') or allocation.get('custom_name')
                resource_id = self._generate_resource_id(resource_type, custom_name=custom_name)
                
                # Create resource entry
                resource_data = {
                    'name': resource_id,  # Store ID in the dict for self-contained access
                    'type': resource_type,
                    'location': (x, y),
                    'description': description,
                    'remove_on_take': remove_on_take,
                    'properties': {}
                }
                
                # Copy any additional fields from allocation
                for key, value in allocation.items():
                    if key not in ['resource_type', 'count', 'description', 
                                   'requires_property', 'remove_on_take', 'terrain_weights']:
                        resource_data['properties'][key] = value
                
                # Register resource
                self.resource_registry[resource_id] = resource_data
                
                # Add to patch
                self.patches[x][y].resources[resource_id] = resource_data
                self.patches[x][y].resource_type = resource_type
    
    def _find_resource_location(self, terrain_weights: Dict, requires_property: bool) -> Tuple[Optional[int], Optional[int]]:
        """
        Find a suitable location for a resource based on terrain weights.
        
        Args:
            terrain_weights: Dict mapping terrain types to weights
            requires_property: Whether resource must be on a property
            
        Returns:
            (x, y) tuple or (None, None) if no suitable location found
        """
        # Build weighted list of candidate locations
        candidates = []
        
        for x in range(self.width):
            for y in range(self.height):
                patch = self.patches[x][y]
                
                # Check property requirement
                if requires_property and patch.property_id is None:
                    continue
                
                # Check terrain weight
                if patch.terrain_type in terrain_weights:
                    weight = terrain_weights[patch.terrain_type]
                    candidates.extend([(x, y)] * int(weight * 10))
        
        if not candidates:
            # Fallback: any valid patch
            candidates = [(x, y) for x in range(self.width) for y in range(self.height)]
        
        if not candidates:
            return None, None
        
        return random.choice(candidates)
    
    def get_resource_by_id(self, resource_id):
        """Get resource data by ID"""
        return self.resource_registry.get(resource_id)
    
    def get_resource_by_name(self, name):
        """Get resource by name (searches descriptions and IDs)"""
        for resource_id, resource in self.resource_registry.items():
            if name.lower() in resource_id.lower():
                return resource  # Return full resource dict, not just ID
            if name.lower() in resource.get('description', '').lower():
                return resource  # Return full resource dict, not just ID
        return None
    
    def get_resource_list(self):
        """
        Get list of all resources with their data.
        
        Returns:
            List of dicts, each containing resource data:
            {'id': str, 'name': str, 'type': enum, 'location': tuple, 'description': str, ...}
        """
        resource_list = []
        for resource_id, resource_data in self.resource_registry.items():
            # Create a copy with the ID included
            resource_info = dict(resource_data)
            resource_info['id'] = resource_id
            resource_list.append(resource_info)
        return resource_list
    
    def get_resource_property(self, resource_id, property_name):
        """Get a property of a resource"""
        if resource_id in self.resource_registry:
            return self.resource_registry[resource_id]['properties'].get(property_name)
        return None
    
    def set_resource_property(self, resource_id, property_name, value):
        """Set a property of a resource"""
        if resource_id in self.resource_registry:
            self.resource_registry[resource_id]['properties'][property_name] = value
            return True
        return False
    
    def delete_resource_property(self, resource_id, property_name):
        """Delete a property from a resource"""
        if resource_id in self.resource_registry:
            if property_name in self.resource_registry[resource_id]['properties']:
                del self.resource_registry[resource_id]['properties'][property_name]
                return True
        return False
    
    def remove_resource(self, resource_id):
        """Remove a resource from the map"""
        if resource_id not in self.resource_registry:
            return False
        
        # Remove from patch
        x, y = self.resource_registry[resource_id]['location']
        if resource_id in self.patches[x][y].resources:
            del self.patches[x][y].resources[resource_id]
        
        # Remove from registry
        del self.resource_registry[resource_id]
        return True
    
    def place_resource(self, resource_id, x, y):
        """Move a resource to a new location"""
        if resource_id not in self.resource_registry:
            return False
        
        # Remove from old location
        old_x, old_y = self.resource_registry[resource_id]['location']
        if resource_id in self.patches[old_x][old_y].resources:
            del self.patches[old_x][old_y].resources[resource_id]
        
        # Add to new location
        self.resource_registry[resource_id]['location'] = (x, y)
        self.patches[x][y].resources[resource_id] = self.resource_registry[resource_id]
        
        return True

    # =============================================================================
    # Property management - common to all space types
    # =============================================================================
    
    def register_property(self, property_id, patches):
        """
        Register a property with its constituent patches.
        
        Args:
            property_id: Unique identifier for the property
            patches: List of (x, y) tuples defining the property
        """
        self.property_registry[property_id] = {
            'patches': patches,
            'owner': None
        }
    
    def set_property_owner(self, property_id, owner):
        """Set the owner of a property"""
        if property_id in self.property_registry:
            self.property_registry[property_id]['owner'] = owner
            return True
        return False
    
    def get_property_owner(self, property_id):
        """Get the owner of a property"""
        if property_id in self.property_registry:
            return self.property_registry[property_id]['owner']
        return None

    # =============================================================================
    # Spatial queries - common to all space types
    # =============================================================================
    
    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """Get neighboring patch coordinates (8-way adjacency)"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    neighbors.append((nx, ny))
        return neighbors
    
    def random_location_by_terrain(self, terrain_name):
        """Get a random location with specified terrain type"""
        # Find terrain type enum member
        terrain_type = None
        for member in self.terrain_types:
            if member.name == terrain_name:
                terrain_type = member
                break
        
        if terrain_type is None:
            return None, None
        
        # Find all patches with this terrain
        candidates = []
        for x in range(self.width):
            for y in range(self.height):
                if self.patches[x][y].terrain_type == terrain_type:
                    candidates.append((x, y))
        
        if not candidates:
            return None, None
        
        return random.choice(candidates)
    
    def random_location_by_resource(self, resource_name):
        """Get a random location with specified resource"""
        resource = self.get_resource_by_name(resource_name)
        if resource:
            return resource['location']
        return None, None

    # =============================================================================
    # Utility methods
    # =============================================================================
    
    def get_owned_resources(self):
        """Get all resources that are NPC-owned"""
        return [res for res in self.resource_registry.values() 
                if res['properties'].get('owner')]
    
    # =============================================================================
    # Visibility processing methods
    # =============================================================================
    
    def extract_direction_info(self, xml_string, direction_name):
        """
        Extract information about a specific direction from the visibility description XML.
        
        This method processes the XML output from get_detailed_visibility_description()
        and extracts structured data for a single direction.
        
        Args:
            xml_string: The XML string returned by get_detailed_visibility_description
            direction_name: The name of the direction to extract (e.g., "North", "Southeast")
            
        Returns:
            A dictionary containing the extracted information for that direction
        """
        root = ET.fromstring(xml_string)

        # Find the direction element
        direction_elem = root.find(f".//{direction_name}")

        if direction_elem is None:
            return f"No information found for direction: {direction_name}"

        info = {}

        # Extract basic information
        visibility = direction_elem.findtext('visibility')
        if visibility is not None:
            try:
                info['visibility'] = int(visibility) 
            except:
                info['visibility'] = visibility
        else:
            info['visibility'] = 0

        info['terrain'] = direction_elem.findtext('terrain')
        info['property'] = direction_elem.findtext('property')

        # Extract slope information if available
        slope_elem = direction_elem.find('slope')
        if slope_elem is not None:
            info['slope'] = {
                'description': slope_elem.text,
            }

        # Extract resource information
        resources = direction_elem.find('resources')
        if resources is not None:
            info['resources'] = [
                {'id': resource.get('id'), 'distance': int(resource.get('distance'))}
                for resource in resources.findall('resource')
            ]

        # Extract water information if available
        water_elem = direction_elem.find('water')
        if water_elem is not None:
            info['water'] = {
                'flow': water_elem.get('flow')
            } 
            try:
                info['water']['distances'] = water_elem.get('distances')
            except:
                pass

        # Extract path information if available (only if infrastructure exists)
        if self._infrastructure_rules is not None:
            path_name = self._infrastructure_rules.get('path_type', 'road')
            path_elem = direction_elem.find(path_name)
            if path_elem is not None:
                info[path_name] = {
                    'distances': path_elem.get('distances')
                } 
            paths_elem = direction_elem.find('paths')
            if paths_elem is not None:
                info['paths'] = [
                    {'id': path.get('id'), 'distance': int(path.get('distance'))}
                    for path in paths_elem.findall('path')
                ]

        # Extract agent information if available
        agents_elem = direction_elem.find('characters')
        if agents_elem is not None:
            try:
                info['characters'] = [
                    {'name': agent.get('name'), 'distance': int(agent.get('distance'))}
                    for agent in agents_elem.findall('character')
                ]
            except:
                info['characters'] = [
                    {'name': agent.get('name'), 'distance': agent.get('distance')}
                    for agent in agents_elem.findall('character')
                ]

        return info
    
    def hash_direction_info(self, direction_info, distance_threshold=10):
        """
        Convert structured direction info into text summary and extract key entities.
        
        This method processes the output of extract_direction_info() for all directions
        and creates a human-readable text description plus lists of visible entities.
        
        Args:
            direction_info: Dictionary mapping direction names to direction info dicts
            distance_threshold: Maximum distance to include in summaries (default 10)
            
        Returns:
            Tuple of (text_view, resources, characters, paths, percept_summary)
        """
        text_view = ""
        percept = ""
        percept_summary = ""
        resources = []
        characters = []
        paths = []
        
        for dir in direction_info.keys():
            if dir == 'Current':
                percept_summary = f'You are in {direction_info[dir]["terrain"]} terrain, viewing property: {direction_info[dir]["property"]}. '

            percept += f"#view {dir}:"
            if 'visibility' in direction_info[dir]:
                percept += f" visibility {direction_info[dir]['visibility']}"
            if 'terrain' in direction_info[dir]:
                percept += f", terrain {direction_info[dir]['terrain']}"
            if 'property' in direction_info[dir]:
                percept += f", property {direction_info[dir]['property']}"
            if 'slope' in direction_info[dir]:
                percept += f", slope {direction_info[dir]['slope']['description']} "
            percept += "; "
            
            # Extract resources
            if 'resources' in direction_info[dir] and len(direction_info[dir]['resources']) > 0:
                resource_label_added = False
                for resource in direction_info[dir]['resources']:
                    if resource['distance'] <= distance_threshold:
                        if not resource_label_added:
                            percept += f"resources: "
                            resource_label_added = True
                        percept += f"{resource['id']} distance {resource['distance']}, "
                        resources.append(resource['id'])
                percept = percept[:-2] + '; '
            
            # Extract path information only if infrastructure exists
            if self._infrastructure_rules is not None:
                path_name = self._infrastructure_rules.get('path_type', 'road').lower()
                # New per-path entities
                if 'paths' in direction_info[dir] and len(direction_info[dir]['paths']) > 0:
                    path_label_added = False
                    for path in direction_info[dir]['paths']:
                        if path['distance'] <= distance_threshold:
                            if not path_label_added:
                                percept += f"{path_name}s: "
                                path_label_added = True
                            percept += f"{path['id']} distance {path['distance']}, "
                            paths.append(path['id'])
                    percept = (percept[:-2] + '; ') if path_label_added else percept
                # Preserve legacy aggregated distances for compatibility
                elif path_name in direction_info[dir] and len(direction_info[dir][path_name]) > 0:
                    path_distances = direction_info[dir][path_name]['distances']
                    percept += f"{path_name}: distances {path_distances}; "
                    paths.append(path_name)

            # Extract characters
            if 'characters' in direction_info[dir] and len(direction_info[dir]['characters']) > 0:  
                character_label_added = False
                for character in direction_info[dir]['characters']:
                    if not character_label_added:
                        percept += f"characters: "
                        character_label_added = True
                    percept += f"{character['name']} distance {character['distance']}, "
                    if character['name'] not in characters:
                        characters.append(character['name'])
                percept = percept[:-2]
            percept += "\n"
        
        percept_summary += f"You see {', '.join(list(set(characters)))} and {', '.join(list(set(resources)))} resources{f' and paths' if len(paths) > 0 else ''}"
        return percept, resources, characters, paths, percept_summary

