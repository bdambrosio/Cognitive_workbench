from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, List

# Import the new dynamic resource system
from map import ResourceTypeRegistry

# All enums for rural scenario
class RuralTerrain(Enum):
    Water = 1
    Mountain = 2
    Forest = 3
    Grassland = 4
    Field = 5

class RuralInfrastructure(Enum):
    Path = auto()

class RuralResources(Enum):
    Market = auto()
    Farmhouse = auto()
    Church = auto()
    Will = auto()
    Deed = auto()
    Well = auto()
    Mill = auto()
    Hoe = auto()
    Plow = auto()
    Scythe = auto()
    Spring = auto()
    Hay = auto()
    Bread = auto()
    Apple = auto()
    Chicken = auto()
    Cow = auto()
    Egg = auto()
    Milk = auto()
    Cheese = auto()
    Butter = auto()

class RuralProperty(Enum):
    Farm = auto()
    Village = auto()
    Blacksmith = auto()

# Rules for terrain generation
terrain_rules = {
    'elevation_noise_scale': 50.0,
    'water_level': 0.2,
    'mountain_level': 0.8,
    'terrain_by_elevation': {
        'water': {'max': 0.1, 'type': 'Water'},
        'mountain': {'min': 0.7, 'type': 'Mountain'}
    },
    'lowland_distribution': {
        'Forest': 0.4,
        'Grassland': 0.3,
        'Field': 0.3
    }
}

infrastructure_rules = {
    'road_density': 0.1,
    'path_type': 'Path',
    'slope_factor': 2.0,
    'terrain_costs': {
        'Water': float('inf'),
        'Mountain': float('inf'),
        'Forest': 2.0,
        'Grassland': 1.0,
        'Field': 1.0
    }
}

property_rules = {
    'min_size': 50,
    'max_size': 150,
    'valid_terrain': ['Field', 'Grassland']
}

# Standard interface names
terrain_types = RuralTerrain
infrastructure_types = RuralInfrastructure
property_types = RuralProperty
resource_types = ResourceTypeRegistry(RuralResources)  # Use dynamic registry

# Add at top with other interface names
required_resource = resource_types.Mill  # or Market, etc.
required_resource_name = "Mill"  # or "Market", etc.

resource_rules = {

    'allocations': [
        {
            'resource_type': resource_types.Will,
            'description': 'A will for land ownership',
            'count': 1,
            'has_npc': True,
            'remove_on_take': False,
            'requires_property': False,
            'use': [],
            'terrain_weights': {
                terrain_types.Grassland: 1.0,
                terrain_types.Field: 1.0
            }
        },
        {
            'resource_type': resource_types.Deed,
            'description': 'A deed for land ownership',
            'count': 1,
            'has_npc': True,
            'remove_on_take': True,
            'requires_property': False,
            'use': [],
            'terrain_weights': {
                terrain_types.Grassland: 1.0,
                terrain_types.Field: 1.0
            }
        },
        {
            'resource_type': resource_types.Well,
            'description': 'A deep well for water',
            'count': 10,
            'has_npc': True,
            'remove_on_take': False,
            'requires_property': True,
            'use': [{"need": 'thirst', "effect": -30}, {"need": 'fatigue', "effect": -10}],
            'terrain_weights': {
                terrain_types.Grassland: 1.0,
                terrain_types.Field: 1.0
            }
        },
        {
            'resource_type': resource_types.Farmhouse,
            'description': 'A farmhouse for housing',
            'count': 5,
            'has_npc': False,
            'remove_on_take': False,
            'requires_property': True,
            'use': [{"need": 'fatigue', "effect": -50}],
            'terrain_weights': {
                terrain_types.Grassland: 1.0,
                terrain_types.Field: 1.0
            }
        },
        {
            'resource_type': resource_types.Church,
            'description': 'A church for religious services',
            'count': 1,
            'has_npc': False,
            'remove_on_take': False,
            'requires_property': True,
            'use': [{"need": 'fatigue', "effect": -20}],
            'terrain_weights': {
                terrain_types.Grassland: 1.0,
                terrain_types.Field: 1.0
            }
        },
        {
            'resource_type': resource_types.Mill,
            'has_npc': True,
            'description': 'A grain mill',
            'count': 1,
            'remove_on_take': False,
            'requires_property': True,
            'use': [],
            'terrain_weights': {
                terrain_types.Field: 2.0,
                terrain_types.Grassland: 1.0
            }
        },
        {
            'resource_type': resource_types.Hoe,
            'has_npc': True,
            'description': 'A hoe for tilling fields',
            'count': 30,
            'remove_on_take': True,
            'requires_property': True,
            'use': [],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            }
        },
        {
            'resource_type': resource_types.Plow,
            'has_npc': True,
            'description': 'A plow for plowing fields',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            }
        },
        {
            'resource_type': resource_types.Scythe,
            'has_npc': True,
            'description': 'A scythe for harvesting grass',
            'count': 30,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'thirst', "effect": 10}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            }
        },
        {
            'resource_type': resource_types.Spring,
            'description': 'A natural spring of fresh water',
            'count': 100,
            'remove_on_take': False,
            'requires_property': True,
            'use': [{"need": 'thirst', "effect": -50}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0,
                terrain_types.Forest: 1.0
            },
        },
                {
            'resource_type': resource_types.Hay,
            'description': 'A bale of hay',
            'count': 60,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": 10}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        },
        {
            'resource_type': resource_types.Bread,
            'description': 'A loaf of bread',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -30}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        },
        {
            'resource_type': resource_types.Apple,
            'description': 'An apple',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -10}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },

        },
        {
            'resource_type': resource_types.Chicken,
            'description': 'A chicken',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -30}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        },
        {
            'resource_type': resource_types.Cow,
            'description': 'A cow for miling',
            'count': 10,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -5}, {"need": 'thirst', "effect": -20}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        },
        {
            'resource_type': resource_types.Egg,
            'description': 'An egg',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -10}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        },
        {
            'resource_type': resource_types.Milk,
            'description': 'A glass of milk',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -10}, {"need": 'thirst', "effect": -30}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        },
        {
            'resource_type': resource_types.Cheese,
            'description': 'A piece of cheese',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -10}, {"need": 'thirst', "effect": 5}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        },
        {
            'resource_type': resource_types.Butter,
            'description': 'A stick of butter',
            'count': 20,
            'remove_on_take': True,
            'requires_property': True,
            'use': [{"need": 'hunger', "effect": -10}],
            'terrain_weights': {
                terrain_types.Field: 1.0,
                terrain_types.Grassland: 1.0
            },
        }


    ]
}

