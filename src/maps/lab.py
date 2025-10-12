from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, List

# Import the new dynamic resource system
from map import ResourceTypeRegistry

# All enums for forest scenario
class LabTerrain(Enum):
    Water = 1
    Mountain = 2
    Campus = 4
    Town = 5
    Park = 6

class LabInfrastructure(Enum):
    Path = auto()    

class LabResources(Enum):
    Appletree = auto()
    Library = auto()  # 
    Fountain = auto()     # Water source
    Cafe = auto()      
    Bench = auto()   
    Lab = auto()    

class LabProperty(Enum):
    Lab = auto()
    pass  # Keeping property system but wilderness has no ownership

# Rules for terrain generation
terrain_rules = {
    'elevation_noise_scale': 50.0,
    'water_level': 0.2,
    'mountain_level': 0.8,
    'terrain_by_elevation': {
        'water': {'max': 0.2, 'type': 'Water'},
        'mountain': {'min': 0.8, 'type': 'Mountain'}
    },
    'lowland_distribution': {
        'Campus': 0.7,        # Much more forest
        'Town': 0.2,      # Some clearings
        'Park': 0.1         # Few meadows
    }
}

infrastructure_rules = {
    'road_density': 0.05,     # Fewer roads in forest
    'path_type': 'Path',     # Specify the path type
    'slope_factor': 2.5,      # Slightly higher penalty for slopes in forests
    'terrain_costs': {
        'Water': float('inf'),
        'Mountain': float('inf'),
        'Campus': 1.5,        # Forests are harder to traverse
        'Town': 1.2,      # Clearings are easy but not as easy as fields
        'Park': 2.0         # Meadows are easy to traverse
    }
}

property_rules = {
    'min_size': 0,    # No properties in wilderness
    'max_size': 0,
    'valid_terrain': ['Campus', 'Town']  # Add this even if no properties are used
}

# Standard interface names
terrain_types = LabTerrain
infrastructure_types = LabInfrastructure
property_types = LabProperty
resource_types = ResourceTypeRegistry(LabResources)  # Use dynamic registry

# Add at top with other interface names
required_resource = resource_types.Lab  # or Market, etc.
required_resource_name = "Lab"  # or "Market", etc.

resource_rules = {
    'allocations': [
        {
            'resource_type': resource_types.Appletree,
            'description': 'A large apple tree with apples',
            'count': 16,
            'requires_property': False,
            'remove_on_take': False,
            'terrain_weights': {
                terrain_types.Campus: 0.6,
                terrain_types.Park: 1.0
            }
        },
        {
            'resource_type': resource_types.Library,
            'description': 'the campus library',
            'count': 1,
            'requires_property': False,
            'remove_on_take': False,
            'terrain_weights': {
                terrain_types.Campus: 1.0
            }
        },
        {
            'resource_type': resource_types.Fountain,
            'description': 'A water fountain',
            'has_npc': False,
            'count': 10,
            'requires_property': False,
            'remove_on_take': False,
            'terrain_weights': {
                terrain_types.Campus: 2.0
            }
        },
        {
            'resource_type': resource_types.Lab,
            'description': 'A small university grad student office and lab',
            'has_npc': False,
            'count': 1,
            'requires_property': False,
            'remove_on_take': False,
            'terrain_weights': {
                terrain_types.Mountain: 2.0
            }
        },
        {
            'resource_type': resource_types.Cafe,
            'description': 'A small cafe serving coffee and light snacks',
            'count': 5,
            'requires_property': False,
            'remove_on_take': False,
            'terrain_weights': {
                terrain_types.Town: 2.0,
                terrain_types.Campus: 1.0
            }
        },
        {
            'resource_type': resource_types.Bench,
            'description': 'a bench to sit on',
            'count': 5,
            'requires_property': False,
            'remove_on_take': False,
            'terrain_weights': {
                terrain_types.Park: 2.0,
                terrain_types.Campus: 1.0
            }
        }

    ]
} 
