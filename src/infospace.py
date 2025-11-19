"""
Infospace cognition module.

Defines resource types for information/semantic operations:
- Note: Dynamic resources (artifacts created at runtime)
- Collection: Dynamic resources (groups of related notes or structured data)
"""

import logging
from enum import Enum

from infospace_types import InfospaceResources, InfospaceTerrain, ResourceTypeRegistry

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# No infrastructure or properties needed
class InfospaceInfrastructure(Enum):
    pass


class InfospaceProperty(Enum):
    pass


def create_infospace_module():
    """
    Create infospace module.
    
    Returns:
        Tuple of (terrain_types, infrastructure_types, property_types, 
                  resource_types, terrain_rules, infrastructure_rules,
                  property_rules, resource_rules, required_resource, required_resource_name)
    """
    # Create resource type registry
    resource_types = ResourceTypeRegistry(InfospaceResources)
    
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
    
    # Resource rules with no static allocations
    # Notes and Collections are created dynamically at runtime
    resource_rules = {
        'allocations': []
    }
    
    # Set required resource (Note)
    required_resource = resource_types.Note
    required_resource_name = "Note"
    
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


# Module initialization
def initialize():
    """
    Initialize the infospace module.
    
    Usage:
        import infospace
        (terrain_types, infrastructure_types, property_types, 
         resource_types, terrain_rules, infrastructure_rules,
         property_rules, resource_rules, required_resource, 
         required_resource_name) = infospace.initialize()
    """
    return create_infospace_module()


# For backwards compatibility, also support the lab.py pattern
# where module variables are set directly
def setup_module():
    """
    Setup module-level variables (alternative initialization pattern).
    
    Usage:
        import infospace
        infospace.setup_module()
        # Now access infospace.terrain_types, infospace.resource_types, etc.
    """
    global terrain_types, infrastructure_types, property_types
    global resource_types, terrain_rules, infrastructure_rules
    global property_rules, resource_rules, required_resource, required_resource_name
    
    (terrain_types, infrastructure_types, property_types,
     resource_types, terrain_rules, infrastructure_rules,
     property_rules, resource_rules, required_resource,
     required_resource_name) = create_infospace_module()

if __name__ == "__main__":
    # Use module-level setup
    setup_module()
