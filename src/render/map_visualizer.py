"""
map_visualizer.py - Pygame visualization for WorldMap

Displays a cartoon-like god's-eye-view of the scenario map with:
- Colored terrain tiles
- Emoji characters for agents
- Speech/thought bubbles
"""

import pygame
import sys
import importlib
import random
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from world_map import WorldMap
from map_agent import MapAgent


# Terrain color palette - kid-friendly cartoon colors
TERRAIN_COLORS = {
    'Water': (100, 180, 255),      # Light blue
    'Mountain': (150, 150, 150),    # Gray
    'Forest': (34, 139, 34),        # Forest green
    'Grassland': (144, 238, 144),   # Light green
    'Field': (240, 230, 140),       # Khaki/wheat
    'Clearing': (200, 220, 160),    # Pale green
    'Meadow': (170, 220, 130),      # Medium green
}

DEFAULT_TERRAIN_COLOR = (200, 200, 200)  # Light gray

# UI Colors
BACKGROUND_COLOR = (240, 240, 240)
GRID_COLOR = (180, 180, 180)
SPEECH_BUBBLE_COLOR = (255, 255, 255)
SPEECH_BUBBLE_BORDER = (0, 0, 0)

# Window settings
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
MAP_MARGIN = 20  # Margin around the map


class MapVisualizer:
    """Pygame-based visualizer for WorldMap"""
    
    def __init__(self, world_map, window_width=WINDOW_WIDTH, window_height=WINDOW_HEIGHT):
        """
        Initialize the visualizer.
        
        Args:
            world_map: WorldMap instance to visualize
            window_width: Width of pygame window
            window_height: Height of pygame window
        """
        pygame.init()
        
        self.world = world_map
        self.window_width = window_width
        self.window_height = window_height
        
        # Calculate tile size based on map dimensions and window size
        available_width = window_width - (2 * MAP_MARGIN)
        available_height = window_height - (2 * MAP_MARGIN)
        
        self.tile_width = available_width // world_map.width
        self.tile_height = available_height // world_map.height
        
        # Use square tiles (smallest dimension)
        self.tile_size = min(self.tile_width, self.tile_height)
        
        # Calculate actual map size and centering offset
        self.map_pixel_width = self.tile_size * world_map.width
        self.map_pixel_height = self.tile_size * world_map.height
        
        self.map_offset_x = (window_width - self.map_pixel_width) // 2
        self.map_offset_y = (window_height - self.map_pixel_height) // 2
        
        # Create window
        self.screen = pygame.display.set_mode((window_width, window_height))
        pygame.display.set_caption("World Map Visualizer")
        
        # Load font for speech bubbles and emoji
        self.font = pygame.font.SysFont('segoe ui emoji', max(16, self.tile_size // 2))
        self.bubble_font = pygame.font.SysFont('arial', max(12, self.tile_size // 3))
        
        # Agent emoji mapping
        self.agent_emoji = {}
        self.available_emoji = ['😊', '🙂', '😄', '🧑', '👤', '🙋', '🧒', '👧', '👦']
        
    def assign_emoji(self, agent_name):
        """Assign an emoji to an agent if not already assigned"""
        if agent_name not in self.agent_emoji:
            emoji_idx = len(self.agent_emoji) % len(self.available_emoji)
            self.agent_emoji[agent_name] = self.available_emoji[emoji_idx]
    
    def get_terrain_color(self, terrain_type):
        """Get color for a terrain type"""
        if terrain_type is None:
            return DEFAULT_TERRAIN_COLOR
        
        terrain_name = terrain_type.name if hasattr(terrain_type, 'name') else str(terrain_type)
        return TERRAIN_COLORS.get(terrain_name, DEFAULT_TERRAIN_COLOR)
    
    def draw_terrain(self):
        """Draw the terrain grid"""
        for x in range(self.world.width):
            for y in range(self.world.height):
                patch = self.world.patches[x][y]
                color = self.get_terrain_color(patch.terrain_type)
                
                # Calculate pixel position
                px = self.map_offset_x + x * self.tile_size
                py = self.map_offset_y + y * self.tile_size
                
                # Draw tile
                pygame.draw.rect(self.screen, color, 
                               (px, py, self.tile_size, self.tile_size))
                
                # Draw grid lines
                pygame.draw.rect(self.screen, GRID_COLOR,
                               (px, py, self.tile_size, self.tile_size), 1)
    
    def draw_agents(self):
        """Draw agents as emoji on the map"""
        for agent in self.world.agents:
            self.assign_emoji(agent.name)
            emoji = self.agent_emoji[agent.name]
            
            # Calculate center of agent's tile
            px = self.map_offset_x + agent.x * self.tile_size + self.tile_size // 2
            py = self.map_offset_y + agent.y * self.tile_size + self.tile_size // 2
            
            # Render emoji
            emoji_surface = self.font.render(emoji, True, (0, 0, 0))
            emoji_rect = emoji_surface.get_rect(center=(px, py))
            self.screen.blit(emoji_surface, emoji_rect)
            
            # Draw name below emoji (smaller)
            name_surface = self.bubble_font.render(agent.name, True, (0, 0, 0))
            name_rect = name_surface.get_rect(center=(px, py + self.tile_size // 2))
            self.screen.blit(name_surface, name_rect)
    
    def draw_speech_bubble(self, agent, text):
        """
        Draw a speech bubble above an agent.
        
        Args:
            agent: MapAgent instance
            text: Text to display in bubble
        """
        # Calculate agent position
        agent_px = self.map_offset_x + agent.x * self.tile_size + self.tile_size // 2
        agent_py = self.map_offset_y + agent.y * self.tile_size + self.tile_size // 2
        
        # Render text
        text_surface = self.bubble_font.render(text, True, (0, 0, 0))
        text_width, text_height = text_surface.get_size()
        
        # Bubble dimensions with padding
        padding = 8
        bubble_width = text_width + padding * 2
        bubble_height = text_height + padding * 2
        
        # Position bubble above agent
        bubble_x = agent_px - bubble_width // 2
        bubble_y = agent_py - self.tile_size - bubble_height - 10
        
        # Ensure bubble stays on screen
        bubble_x = max(5, min(bubble_x, self.window_width - bubble_width - 5))
        bubble_y = max(5, bubble_y)
        
        # Draw bubble (rounded rectangle)
        bubble_rect = pygame.Rect(bubble_x, bubble_y, bubble_width, bubble_height)
        pygame.draw.rect(self.screen, SPEECH_BUBBLE_COLOR, bubble_rect, border_radius=8)
        pygame.draw.rect(self.screen, SPEECH_BUBBLE_BORDER, bubble_rect, 2, border_radius=8)
        
        # Draw speech bubble pointer (small triangle)
        pointer_tip = (agent_px, agent_py - self.tile_size // 4)
        pointer_left = (bubble_x + bubble_width // 2 - 8, bubble_y + bubble_height)
        pointer_right = (bubble_x + bubble_width // 2 + 8, bubble_y + bubble_height)
        pygame.draw.polygon(self.screen, SPEECH_BUBBLE_COLOR, 
                          [pointer_tip, pointer_left, pointer_right])
        pygame.draw.line(self.screen, SPEECH_BUBBLE_BORDER, pointer_left, pointer_tip, 2)
        pygame.draw.line(self.screen, SPEECH_BUBBLE_BORDER, pointer_right, pointer_tip, 2)
        
        # Draw text
        text_rect = text_surface.get_rect(center=(bubble_x + bubble_width // 2, 
                                                  bubble_y + bubble_height // 2))
        self.screen.blit(text_surface, text_rect)
    
    def render(self):
        """Render the entire map"""
        # Clear screen
        self.screen.fill(BACKGROUND_COLOR)
        
        # Draw layers
        self.draw_terrain()
        self.draw_agents()
        
        # Draw example speech bubble on first agent (Samantha)
        if self.world.agents:
            self.draw_speech_bubble(self.world.agents[0], "Hello World")
        
        # Update display
        pygame.display.flip()
    
    def run(self):
        """Main loop - display map until window is closed"""
        clock = pygame.time.Clock()
        running = True
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            self.render()
            clock.tick(30)  # 30 FPS
        
        pygame.quit()


def find_random_valid_position(world_map):
    """
    Find a random valid position for spawning an agent.
    
    Prefers land tiles (non-water, non-mountain).
    
    Args:
        world_map: WorldMap instance
        
    Returns:
        Tuple of (x, y) coordinates
    """
    # Collect valid positions (non-water, non-mountain)
    valid_positions = []
    
    for x in range(world_map.width):
        for y in range(world_map.height):
            patch = world_map.patches[x][y]
            terrain_name = patch.terrain_type.name if patch.terrain_type else ""
            
            # Skip water and mountains
            if terrain_name not in ['Water', 'Mountain']:
                valid_positions.append((x, y))
    
    # If no valid positions found, just use any position
    if not valid_positions:
        return (random.randint(0, world_map.width - 1), 
                random.randint(0, world_map.height - 1))
    
    return random.choice(valid_positions)


def create_example_visualization(scenario_name='rural', map_width=30, map_height=30):
    """
    Create and display a WorldMap with example agents.
    
    Args:
        scenario_name: Name of scenario module (e.g., 'rural', 'forest')
        map_width: Width of map in patches
        map_height: Height of map in patches
    """
    # Load scenario module
    scenario_module = importlib.import_module(f'maps.{scenario_name}')
    
    # Create WorldMap
    print(f"Creating {map_width}x{map_height} map from {scenario_name} scenario...")
    world_map = WorldMap(scenario_module, width=map_width, height=map_height)
    
    # Create and register agents
    print("Creating agents...")
    
    # Samantha
    sam_x, sam_y = find_random_valid_position(world_map)
    samantha = MapAgent(sam_x, sam_y, world_map, "Samantha")
    
    # Joe (ensure different position)
    joe_x, joe_y = find_random_valid_position(world_map)
    while (joe_x, joe_y) == (sam_x, sam_y):
        joe_x, joe_y = find_random_valid_position(world_map)
    joe = MapAgent(joe_x, joe_y, world_map, "Joe")
    
    print(f"Samantha spawned at ({sam_x}, {sam_y})")
    print(f"Joe spawned at ({joe_x}, {joe_y})")
    print(f"Total agents registered: {len(world_map.agents)}")
    
    # Create visualizer and run
    print("Launching visualizer...")
    visualizer = MapVisualizer(world_map)
    visualizer.run()


if __name__ == "__main__":
    # Example: create visualization with rural scenario
    # Can pass different scenario name as parameter
    scenario = sys.argv[1] if len(sys.argv) > 1 else 'rural'
    create_example_visualization(scenario_name=scenario, map_width=30, map_height=30)
