#!/usr/bin/env python3

import sys
import os
import argparse
import logging
import json
import importlib.util
import signal
from datetime import datetime
from typing import Dict, Any, Optional

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh
from map import WorldMap, Agent

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('map_node.log', mode='w')
    ],
    force=True
)
logger = logging.getLogger(__name__)

class MapNode:
    def __init__(self, map_file: str):
        self.map_file = map_file
        self.world_map = None
        self.session = None
        self.shutdown_requested = False
        
        # Agent registry: character_name -> Agent instance
        self.agent_registry = {}
        
        # Agent visibility tracking: agent_name -> set of visible agent names
        self.agent_visibility = {}
        
        # Load the map module
        self.load_map_module()
        
        # Initialize Zenoh session
        self.init_zenoh()
        
    def load_map_module(self):
        """Load the map module from the maps subdirectory"""
        try:
            maps_dir = os.path.join(os.path.dirname(__file__), 'maps')
            map_path = os.path.join(maps_dir, self.map_file)
            
            if not os.path.exists(map_path):
                raise FileNotFoundError(f"Map file not found: {map_path}")
            
            logger.info(f"Loading map module from: {map_path}")
            
            # Load the module dynamically
            spec = importlib.util.spec_from_file_location("map_module", map_path)
            map_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(map_module)
            
            # Create WorldMap instance
            logger.info("Creating WorldMap instance...")
            self.world_map = WorldMap(map_module)
            logger.info(f"WorldMap created successfully: {self.world_map.width}x{self.world_map.height}")
            
        except Exception as e:
            logger.error(f"Failed to load map module: {e}")
            raise
    
    def init_zenoh(self):
        """Initialize Zenoh session and set up queryables"""
        try:
            # Create Zenoh configuration
            conf = zenoh.Config()
            conf.insert_json5("listen/endpoints", '["tcp/localhost:7447"]')
            conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
            
            # Create Zenoh session
            logger.info("Creating Zenoh session...")
            self.session = zenoh.open(conf)
            
            # Set up queryables for map services
            self.setup_queryables()
            
            logger.info("Zenoh session initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize Zenoh: {e}")
            raise
    
    def setup_queryables(self):
        """Set up Zenoh queryables for map services"""
        
        # Map summary queryable
        self.map_summary_queryable = self.session.declare_queryable(
            "cognitive/map/summary",
            self.handle_map_summary
        )
        
        # Resource list queryable
        self.resources_queryable = self.session.declare_queryable(
            "cognitive/map/resources",
            self.handle_resources_query
        )
        
        # Resource by name queryable
        self.resource_by_name_queryable = self.session.declare_queryable(
            "cognitive/map/resource/*",
            self.handle_resource_by_name
        )
        
        # Terrain queryable
        self.terrain_queryable = self.session.declare_queryable(
            "cognitive/map/terrain",
            self.handle_terrain_query
        )
        
        # Random location by resource queryable
        self.random_location_resource_queryable = self.session.declare_queryable(
            "cognitive/map/random_location/resource/*",
            self.handle_random_location_by_resource
        )
        
        # Random location by terrain queryable
        self.random_location_terrain_queryable = self.session.declare_queryable(
            "cognitive/map/random_location/terrain/*",
            self.handle_random_location_by_terrain
        )
        
        # Agent registration queryable
        self.agent_register_queryable = self.session.declare_queryable(
            "cognitive/map/agent/register/*",
            self.handle_agent_register
        )
        
        # Agent look queryable
        self.agent_look_queryable = self.session.declare_queryable(
            "cognitive/map/agent/*/look",
            self.handle_agent_look
        )
        
        # Agent move queryable
        self.agent_move_queryable = self.session.declare_queryable(
            "cognitive/map/agent/*/move",
            self.handle_agent_move
        )
        
        logger.info("Map queryables set up successfully")
    
    def handle_map_summary(self, query):
        """Handle map summary queries"""
        try:
            summary = self.world_map.get_map_summary()
            response = {
                'success': True,
                'map_file': self.map_file,
                'summary': summary
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling map summary query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_resources_query(self, query):
        """Handle resources list queries"""
        try:
            resources = self.world_map.get_resource_list()
            response = {
                'success': True,
                'resources': resources
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling resources query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_resource_by_name(self, query):
        """Handle resource by name queries"""
        try:
            # Extract resource name from query key
            key_parts = str(query.key_expr).split('/')
            resource_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not resource_name:
                raise ValueError("No resource name provided")
            
            resource = self.world_map.get_resource_by_name(resource_name)
            if resource:
                response = {
                    'success': True,
                    'resource': resource
                }
            else:
                response = {
                    'success': False,
                    'error': f"Resource '{resource_name}' not found"
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling resource by name query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_terrain_query(self, query):
        """Handle terrain queries"""
        try:
            # Get terrain information
            terrain_info = {
                'width': self.world_map.width,
                'height': self.world_map.height,
                'terrain_types': [t.name for t in self.world_map.terrain_types]
            }
            
            response = {
                'success': True,
                'terrain': terrain_info
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling terrain query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_random_location_by_resource(self, query):
        """Handle random location by resource queries"""
        try:
            # Extract resource name from query key
            key_parts = str(query.key_expr).split('/')
            resource_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not resource_name:
                raise ValueError("No resource name provided")
            
            location = self.world_map.random_location_by_resource(resource_name)
            if location:
                response = {
                    'success': True,
                    'resource': resource_name,
                    'location': location
                }
            else:
                response = {
                    'success': False,
                    'error': f"No location found for resource '{resource_name}'"
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling random location query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_random_location_by_terrain(self, query):
        """Handle random location by terrain queries"""
        try:
            # Extract terrain name from query key
            key_parts = str(query.key_expr).split('/')
            terrain_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not terrain_name:
                raise ValueError("No terrain name provided")
            
            location = self.world_map.random_location_by_terrain(terrain_name)
            if location:
                response = {
                    'success': True,
                    'terrain': terrain_name,
                    'location': location
                }
            else:
                response = {
                    'success': False,
                    'error': f"No location found for terrain '{terrain_name}'"
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling random terrain location query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_agent_register(self, query):
        """Handle agent registration"""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not character_name:
                raise ValueError("No character name provided")
            
            # Check if agent already exists
            if character_name in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' already registered"
                }
            else:
                # Find a random valid location for the agent
                location = self.world_map.random_location_by_terrain("Clearing")
                if not location:
                    # Fallback to any valid location
                    location = (25, 25)  # Center of map
                
                # Create agent instance
                agent = Agent(location[0], location[1], self.world_map, character_name)
                
                # Register agent with world map
                self.world_map.register_agent(agent)
                
                # Store in our registry
                self.agent_registry[character_name] = agent
                
                # Initialize visibility tracking for this agent
                self.agent_visibility[character_name] = set()
                
                response = {
                    'success': True,
                    'character_name': character_name,
                    'location': location,
                    'message': f"Agent registered at {location}"
                }
                
                logger.info(f"Agent registered for character '{character_name}' at {location}")
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling agent registration: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_agent_look(self, query):
        """Handle agent look command"""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-2] if len(key_parts) > 1 else None
            
            if not character_name:
                raise ValueError("No character name provided")
            
            # Get agent from registry
            if character_name not in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' not found"
                }
            else:
                agent = self.agent_registry[character_name]
                
                # Call agent's look method
                look_result = agent.look()
                
                response = {
                    'success': True,
                    'character_name': character_name,
                    'look_result': look_result
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling agent look: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_agent_move(self, query):
        """Handle agent move command"""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-2] if len(key_parts) > 1 else None
            
            if not character_name:
                raise ValueError("No character name provided")
            
            # Parse direction from query payload
            direction = 'current'  # Default direction
            try:
                if query.payload:
                    payload = query.payload.to_bytes().decode('utf-8')
                    data = json.loads(payload) if payload else {}
                    direction = data.get('direction', 'current')
            except Exception as e:
                logger.warning(f"Could not parse move payload, using default direction: {e}")
            
            # Get agent from registry
            if character_name not in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' not found"
                }
            else:
                agent = self.agent_registry[character_name]
                
                # Call agent's move method
                move_result = agent.move(direction)
                
                # Check for visibility changes after movement
                self.check_visibility_changes(character_name)
                
                response = {
                    'success': True,
                    'character_name': character_name,
                    'direction': direction,
                    'move_result': move_result
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling agent move: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def check_visibility_changes(self, moved_agent_name):
        """Check if the moved agent is now visible to other agents"""
        try:
            if moved_agent_name not in self.agent_registry:
                return
            
            moved_agent = self.agent_registry[moved_agent_name]
            
            # Get current visibility from moved agent's perspective
            # This will reveal all agents that can see the moved agent
            current_visibility = set()
            
            # Check each other agent to see if they can see the moved agent
            for other_agent_name, other_agent in self.agent_registry.items():
                if other_agent_name == moved_agent_name:
                    continue
                
                # Check if other agent can see the moved agent
                # We'll use a simple distance-based check for now
                if self.can_agent_see_agent(other_agent, moved_agent):
                    current_visibility.add(other_agent_name)
            
            # Check for new detections
            previous_visibility = self.agent_visibility.get(moved_agent_name, set())
            new_detections = current_visibility - previous_visibility
            
            # Publish events for new detections
            for detecting_agent_name in new_detections:
                self.publish_agent_detected_event(detecting_agent_name, moved_agent_name)
            
            # Update visibility tracking
            self.agent_visibility[moved_agent_name] = current_visibility
            
            if new_detections:
                logger.info(f"Agent '{moved_agent_name}' is now visible to: {new_detections}")
                
        except Exception as e:
            logger.error(f"Error checking visibility changes for {moved_agent_name}: {e}")
    
    def can_agent_see_agent(self, observer_agent, target_agent):
        """Check if observer_agent can see target_agent"""
        try:
            # Get positions
            observer_pos = (observer_agent.x, observer_agent.y)
            target_pos = (target_agent.x, target_agent.y)
            
            # Calculate distance
            distance = self.calculate_distance(observer_pos, target_pos)
            
            # Simple visibility: within 10 units and line of sight
            if distance <= 10:
                # Check line of sight (simplified - just check if path is clear)
                return self.has_line_of_sight(observer_pos, target_pos)
            
            return False
            
        except Exception as e:
            logger.error(f"Error checking agent visibility: {e}")
            return False
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Manhattan distance between two positions"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def has_line_of_sight(self, pos1, pos2):
        """Check if there's a clear line of sight between two positions"""
        try:
            # Simple line of sight check - just make sure it's not blocked by water/mountains
            x1, y1 = pos1
            x2, y2 = pos2
            
            # Check points along the path
            steps = max(abs(x2 - x1), abs(y2 - y1))
            if steps == 0:
                return True
            
            for i in range(1, steps):
                t = i / steps
                x = int(x1 + t * (x2 - x1))
                y = int(y1 + t * (y2 - y1))
                
                # Check if this point is passable terrain
                if 0 <= x < self.world_map.width and 0 <= y < self.world_map.height:
                    patch = self.world_map.patches[x][y]
                    if patch.terrain_type and patch.terrain_type.name in ['Water', 'Mountain']:
                        return False
            
            return True
            
        except Exception as e:
            logger.error(f"Error checking line of sight: {e}")
            return True  # Default to visible if error
    
    def publish_agent_detected_event(self, observer_agent_name, detected_agent_name):
        """Publish agent detection event"""
        try:
            detected_agent = self.agent_registry[detected_agent_name]
            observer_agent = self.agent_registry[observer_agent_name]
            
            # Calculate distance
            distance = self.calculate_distance(
                (observer_agent.x, observer_agent.y),
                (detected_agent.x, detected_agent.y)
            )
            
            event_data = {
                'agent_name': detected_agent_name,
                'position': (detected_agent.x, detected_agent.y),
                'distance': distance,
                'timestamp': datetime.now().isoformat()
            }
            
            topic = f"cognitive/{observer_agent_name}/sense/visual/agent_detected"
            self.session.put(topic, json.dumps(event_data).encode('utf-8'))
            
            logger.info(f"Published visual event: {observer_agent_name} detected {detected_agent_name} at distance {distance}")
            
        except Exception as e:
            logger.error(f"Error publishing agent detected event: {e}")
    
    def run(self):
        """Run the map node"""
        logger.info(f"Map node started with map file: {self.map_file}")
        
        try:
            # Keep the node running
            while not self.shutdown_requested:
                import time
                time.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        except Exception as e:
            logger.error(f"Error in map node main loop: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown the map node"""
        logger.info("Shutting down map node...")
        self.shutdown_requested = True
        
        # Clean up agents
        for character_name, agent in self.agent_registry.items():
            try:
                self.world_map.unregister_agent(agent)
                logger.info(f"Unregistered agent for character: {character_name}")
            except Exception as e:
                logger.error(f"Error unregistering agent for {character_name}: {e}")
        
        # Clear visibility tracking
        self.agent_visibility.clear()
        
        if self.session:
            self.session.close()
            logger.info("Zenoh session closed")
        
        logger.info("Map node shutdown complete")

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info(f"Received signal {signum}")
    if hasattr(signal_handler, 'map_node'):
        signal_handler.map_node.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Shared Map Node')
    parser.add_argument('-m', '--map-file', required=True, 
                       help='Map file name (e.g., forest.py)')
    
    args = parser.parse_args()
    
    try:
        # Set up signal handlers
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        # Create and run map node
        map_node = MapNode(args.map_file)
        signal_handler.map_node = map_node  # Store reference for signal handler
        
        map_node.run()
        
    except Exception as e:
        logger.error(f"Failed to start map node: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 