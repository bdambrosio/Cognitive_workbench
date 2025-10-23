"""
Infospace cognition map module.

This module creates a map of skills discovered from a skills directory.
Each skill becomes a resource instance in the infospace.
"""

from enum import Enum, auto
from pathlib import Path
from typing import Dict, List, Optional
import logging
import re

# Import the dynamic resource system
from world_map import ResourceTypeRegistry

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Minimal enums for infospace
class InfospaceTerrain(Enum):
    """Single terrain type for skill space."""
    InfoSpace = 1


class InfospaceResources(Enum):
    """
    Resource types in infospace.
    
    - Skill: Static resources (cognitive tools loaded from skills directory)
    - Note: Dynamic resources (artifacts created by skill execution at runtime)
    - Collection: Dynamic resources (groups of related notes or structured data)
    """
    Skill = auto()
    Note = auto()
    Collection = auto()


# No infrastructure or properties needed
class InfospaceInfrastructure(Enum):
    pass


class InfospaceProperty(Enum):
    pass


def parse_yaml_frontmatter(content: str) -> Optional[Dict[str, str]]:
    """
    Extract YAML frontmatter from SKILL.md content.
    
    Args:
        content: Full file content
        
    Returns:
        Dictionary with frontmatter fields, or None if parsing fails
    """
    # Match content between --- markers at start of file
    pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.match(pattern, content, re.DOTALL)
    
    if not match:
        logger.warning("No YAML frontmatter found (missing --- markers)")
        return None
    
    frontmatter_text = match.group(1)
    
    # Simple YAML parser for our minimal needs (name: value format)
    result = {}
    for line in frontmatter_text.split('\n'):
        line = line.strip()
        if ':' in line and not line.startswith('#'):
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            result[key] = value
    
    return result if result else None


def scan_tools_directory(tools_path: str) -> List[Dict]:
    """
    Scan tools directory and create resource allocations for each tool.
    
    Args:
        tools_path: Absolute path to tools directory
        
    Returns:
        List of resource allocation dictionaries
    """
    tools_dir = Path(tools_path)
    
    if not tools_dir.exists():
        logger.error(f"Tools directory not found: {tools_path}")
        return []
    
    if not tools_dir.is_dir():
        logger.error(f"Tools path is not a directory: {tools_path}")
        return []
    
    allocations = []
    tool_names_seen = set()
    
    # Scan immediate subdirectories
    for tool_dir in sorted(tools_dir.iterdir()):
        if not tool_dir.is_dir():
            continue
            
        tool_md_path = tool_dir / "SKILL.md"
        
        if not tool_md_path.exists():
            logger.warning(f"No SKILL.md found in {tool_dir.name}, skipping")
            continue
        
        try:
            # Read SKILL.md
            with open(tool_md_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Parse frontmatter
            metadata = parse_yaml_frontmatter(content)
            
            if metadata is None:
                logger.error(f"Failed to parse YAML frontmatter in {tool_md_path}")
                continue
            
            # Extract required fields with defaults
            tool_name = metadata.get('name')
            if not tool_name:
                tool_name = tool_dir.name
                logger.warning(f"Missing 'name' in {tool_md_path}, using directory name: {tool_name}")
            
            tool_description = metadata.get('description')
            if not tool_description:
                tool_description = "No description available"
                logger.warning(f"Missing 'description' in {tool_md_path}")
            
            # Check for duplicate names
            if tool_name in tool_names_seen:
                logger.warning(f"Duplicate tool name '{tool_name}' in {tool_dir}, skipping")
                continue
            
            tool_names_seen.add(tool_name)
            
            # Collect additional files in the tool directory
            additional_files = [
                f.name for f in tool_dir.iterdir() 
                if f.is_file() and f.name != "SKILL.md"
            ]
            
            # Create resource allocation
            allocation = {
                'resource_type': None,  # Will be set after registry creation
                'description': tool_description,
                'count': 1,
                'requires_property': False,
                'remove_on_take': False,
                'terrain_weights': {
                    InfospaceTerrain.InfoSpace: 1.0
                },
                # Tool-specific metadata
                'tool_name': tool_name,
                'tool_path': str(tool_dir.absolute()),
                'tool_md_content': content,
                'additional_files': additional_files
            }
            
            allocations.append(allocation)
            logger.info(f"Loaded tool: {tool_name}")
            
        except Exception as e:
            logger.error(f"Error processing {tool_dir.name}: {e}")
            continue
    
    logger.info(f"Successfully loaded {len(allocations)} tools from {tools_path}")
    
    if len(allocations) == 0:
        logger.info("No tools found in directory")
    
    return allocations


def create_infospace_module(tools_path: str):
    """
    Create infospace module with tools from the given directory.
    
    Args:
        tools_path: Absolute path to tools directory
        
    Returns:
        Tuple of (terrain_types, infrastructure_types, property_types, 
                  resource_types, terrain_rules, infrastructure_rules,
                  property_rules, resource_rules, required_resource, required_resource_name)
    """
    # Scan tools directory
    tool_allocations = scan_tools_directory(tools_path)
    
    # Create resource type registry
    resource_types = ResourceTypeRegistry(InfospaceResources)
    
    # Set resource_type in allocations now that we have the registry
    for allocation in tool_allocations:
        allocation['resource_type'] = resource_types.Skill
    
    # Minimal terrain rules - uniform flat space
    terrain_rules = {
        'elevation_noise_scale': 0.0,  # No elevation variation
        'water_level': 0.0,
        'mountain_level': 1.0,
        'terrain_by_elevation': {},
        'lowland_distribution': {
            'InfoSpace': 1.0  # Everything is InfoSpace
        }
    }
    
    # No infrastructure
    infrastructure_rules = None
    
    # No properties
    property_rules = {
        'min_size': 0,
        'max_size': 0,
        'valid_terrain': ['InfoSpace']
    }
    
    # Resource rules with dynamically discovered tools
    resource_rules = {
        'allocations': tool_allocations
    }
    
    # Set required resource (first tool, or a specific one if present)
    required_resource = resource_types.Skill
    required_resource_name = "Skill"
    
    # Look for tool-creator or similar as the required resource
    for allocation in tool_allocations:
        if allocation['tool_name'] in ['tool-creator', 'tool_creator']:
            required_resource_name = allocation['tool_name']
            break
    
    return (
        InfospaceTerrain,
        None,  # No infrastructure types
        None,  # No property types
        resource_types,
        terrain_rules,
        infrastructure_rules,
        property_rules,
        resource_rules,
        required_resource,
        required_resource_name
    )


# Module initialization with tools path
# This should be called by the user with their specific tools directory path
def initialize(tools_path: str):
    """
    Initialize the infospace module with tools from the given directory.
    
    Usage:
        import infospace
        (terrain_types, infrastructure_types, property_types, 
         resource_types, terrain_rules, infrastructure_rules,
         property_rules, resource_rules, required_resource, 
         required_resource_name) = infospace.initialize('/path/to/tools')
    
    Args:
        tools_path: Absolute path to tools directory
    """
    return create_infospace_module(tools_path)


# For backwards compatibility, also support the lab.py pattern
# where module variables are set directly
def setup_module(tools_path: str):
    """
    Setup module-level variables (alternative initialization pattern).
    
    Usage:
        import infospace
        infospace.setup_module('/path/to/tools')
        # Now access infospace.terrain_types, infospace.resource_types, etc.
    """
    global terrain_types, infrastructure_types, property_types
    global resource_types, terrain_rules, infrastructure_rules
    global property_rules, resource_rules, required_resource, required_resource_name
    
    (terrain_types, infrastructure_types, property_types,
     resource_types, terrain_rules, infrastructure_rules,
     property_rules, resource_rules, required_resource,
     required_resource_name) = create_infospace_module(tools_path)

if __name__ == "__main__":

    # Initialize with tools directory path
    #(terrain_types, infrastructure_types, property_types,
    #  resource_types, terrain_rules, infrastructure_rules,
    #  property_rules, resource_rules, required_resource,
    #  required_resource_name) = initialize('/absolute/path/to/tools')

    # Or use module-level setup
    setup_module('/home/bruce/Downloads/Cognitive_workbench/src/maps/tools')

# Import InfospaceMap for map_node
from infospace_map import InfospaceMap

# Specify map class for map_node
map_class = InfospaceMap
