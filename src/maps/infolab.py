from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, List

# Import the new information space map
from infospace_map import InfospaceMap

import infospace
infospace.setup_module('/home/bruce/Downloads/Cognitive_workbench/src/maps/skills')

# Re-export the attributes that map_node expects
terrain_types = infospace.terrain_types
infrastructure_types = infospace.infrastructure_types
property_types = infospace.property_types
resource_types = infospace.resource_types
terrain_rules = infospace.terrain_rules
infrastructure_rules = infospace.infrastructure_rules
property_rules = infospace.property_rules
resource_rules = infospace.resource_rules
required_resource = infospace.required_resource
required_resource_name = infospace.required_resource_name

# Specify map class for map_node
map_class = InfospaceMap
