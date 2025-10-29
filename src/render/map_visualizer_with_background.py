"""
map_visualizer_with_background.py - Pygame visualization with optional painted background

Displays a WorldMap with either:
1. Simple colored terrain tiles (original style)
2. A beautiful Schnell-generated background image with transparent overlays

Usage:
    # Without background image (colored tiles)
    python map_visualizer_with_background.py rural
    
    # With background image
    python map_visualizer_with_background.py rural --background map_bg.png
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


# Terrain color palette - kid-friendly cartoon colors (fallback)
TERRAIN_COLORS = {
    'Water': (100, 180, 255),
    'Mountain': (150, 150, 150),
    'Forest': (34, 139, 34),
    'Grassland': (144, 238, 144),
    'Field': (240, 230, 140),
    'Clearing': (200, 220, 160),
    'Meadow': (170, 220, 130),
}

DEFAULT_TERRAIN_COLOR = (200, 200, 200)

# UI Colors
BACKGROUND_COLOR = (240, 240, 240)
GRID_COLOR = (100, 100, 100, 80)  # Semi-transparent gray for grid over image
SPEECH_BUBBLE_COLOR = (255, 255, 255)
SPEECH_BUBBLE_BORDER = (0, 0, 0)

# Window settings
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
MAP_MARGIN = 20


class MapVisualizer:
    """Pygame-based visualizer for WorldMap with optional background image"""
    
    def __init__(self, world_map, window_width=WINDOW_WIDTH, window_height=WINDOW_HEIGHT,
                 background_image_path=None):
        """
        Initialize the visualizer.
        
        Args:
            world_map: WorldMap instance to visualize
            window_width: Width of pygame window
            window_height: Height of pygame window
            background_image_path: Optional path to background image from Schnell
        """
        pygame.init()
        
        self.world = world_map
        self.window_width = window_width
        self.window_height = window_height
        
        # Calculate tile size
        available_width = window_width - (2 * MAP_MARGIN)
        available_height = window_height - (2 * MAP_MARGIN)
        
        self.tile_width = available_width // world_map.width
        self.tile_height = available_height // world_map.height
        self.tile_size = min(self.tile_width, self.tile_height)
        
        # Calculate map positioning
        self.map_pixel_width = self.tile_size * world_map.width
        self.map_pixel_height = self.tile_size * world_map.height
        self.map_offset_x = (window_width - self.map_pixel_width) // 2
        self.map_offset_y = (window_height - self.map_pixel_height) // 2
        
        # Create window
        self.screen = pygame.display.set_mode((window_width, window_height))
        pygame.display.set_caption("World Map Visualizer")
        
        # Load and scale background image if provided
        self.background_image = None
        if background_image_path:
            try:
                bg_img = pygame.image.load(background_image_path)
                # Scale to fit map area
                self.background_image = pygame.transform.scale(
                    bg_img, 
                    (self.map_pixel_width, self.map_pixel_height)
                )
                print(f"Loaded background image: {background_image_path}")
            except Exception as e:
                print(f"Failed to load background image: {e}")
                print("Falling back to colored tiles")
        
        # Load fonts
        self.font = pygame.font.SysFont('segoe ui emoji', max(16, self.tile_size // 2))
        self.bubble_font = pygame.font.SysFont('arial', max(12, self.tile_size // 3))
        
        # Agent emoji
        self.agent_emoji = {}
        self.available_emoji = ['😊', '🙂', '😄', '🧑', '👤', '🙋', '🧒', '👧', '👦']
        
        # UI options
        self.show_grid = True  # Toggle with 'G' key
    
    def assign_emoji(self, agent_name):
        """Assign an emoji to an agent"""
        if agent_name not in self.agent_emoji:
            emoji_idx = len(self.agent_emoji) % len(self.available_emoji)
            self.agent_emoji[agent_name] = self.available_emoji[emoji_idx]
    
    def get_terrain_color(self, terrain_type):
        """Get color for a terrain type"""
        if terrain_type is None:
            return DEFAULT_TERRAIN_COLOR
        terrain_name = terrain_type.name if hasattr(terrain_type, 'name') else str(terrain_type)
        return TERRAIN_COLORS.get(terrain_name, DEFAULT_TERRAIN_COLOR)
    
    def draw_background(self):
        """Draw either background image or colored terrain tiles"""
        if self.background_image:
            # Draw the Schnell-generated background
            self.screen.blit(self.background_image, 
                           (self.map_offset_x, self.map_offset_y))
        else:
            # Draw colored terrain tiles (fallback)
            for x in range(self.world.width):
                for y in range(self.world.height):
                    patch = self.world.patches[x][y]
                    color = self.get_terrain_color(patch.terrain_type)
                    
                    px = self.map_offset_x + x * self.tile_size
                    py = self.map_offset_y + y * self.tile_size
                    
                    pygame.draw.rect(self.screen, color,
                                   (px, py, self.tile_size, self.tile_size))
    
    def draw_grid(self):
        """Draw optional grid overlay"""
        if not self.show_grid:
            return
        
        # Create semi-transparent surface for grid
        grid_surface = pygame.Surface(
            (self.map_pixel_width, self.map_pixel_height), 
            pygame.SRCALPHA
        )
        
        for x in range(self.world.width + 1):
            px = x * self.tile_size
            pygame.draw.line(grid_surface, GRID_COLOR,
                           (px, 0), (px, self.map_pixel_height), 1)
        
        for y in range(self.world.height + 1):
            py = y * self.tile_size
            pygame.draw.line(grid_surface, GRID_COLOR,
                           (0, py), (self.map_pixel_width, py), 1)
        
        # Blit grid surface onto screen
        self.screen.blit(grid_surface, (self.map_offset_x, self.map_offset_y))
    
    def draw_agents(self):
        """Draw agents as emoji on the map"""
        for agent in self.world.agents:
            self.assign_emoji(agent.name)
            emoji = self.agent_emoji[agent.name]
            
            # Calculate center of agent's tile
            px = self.map_offset_x + agent.x * self.tile_size + self.tile_size // 2
            py = self.map_offset_y + agent.y * self.tile_size + self.tile_size // 2
            
            # Render emoji with white background circle for visibility
            circle_radius = self.tile_size // 3
            pygame.draw.circle(self.screen, (255, 255, 255), (px, py), circle_radius)
            pygame.draw.circle(self.screen, (0, 0, 0), (px, py), circle_radius, 2)
            
            # Render emoji
            emoji_surface = self.font.render(emoji, True, (0, 0, 0))
            emoji_rect = emoji_surface.get_rect(center=(px, py))
            self.screen.blit(emoji_surface, emoji_rect)
            
            # Draw name below with background
            name_surface = self.bubble_font.render(agent.name, True, (0, 0, 0))
            name_rect = name_surface.get_rect(center=(px, py + self.tile_size // 2 + 5))
            
            # Draw background rectangle for name
            name_bg_rect = name_rect.inflate(4, 2)
            pygame.draw.rect(self.screen, (255, 255, 255), name_bg_rect, border_radius=3)
            pygame.draw.rect(self.screen, (0, 0, 0), name_bg_rect, 1, border_radius=3)
            
            self.screen.blit(name_surface, name_rect)
    
    def draw_speech_bubble(self, agent, text):
        """Draw a speech bubble above an agent"""
        agent_px = self.map_offset_x + agent.x * self.tile_size + self.tile_size // 2
        agent_py = self.map_offset_y + agent.y * self.tile_size + self.tile_size // 2
        
        # Render text
        text_surface = self.bubble_font.render(text, True, (0, 0, 0))
        text_width, text_height = text_surface.get_size()
        
        # Bubble dimensions
        padding = 8
        bubble_width = text_width + padding * 2
        bubble_height = text_height + padding * 2
        
        # Position bubble above agent
        bubble_x = agent_px - bubble_width // 2
        bubble_y = agent_py - self.tile_size - bubble_height - 15
        
        # Keep bubble on screen
        bubble_x = max(5, min(bubble_x, self.window_width - bubble_width - 5))
        bubble_y = max(5, bubble_y)
        
        # Draw bubble
        bubble_rect = pygame.Rect(bubble_x, bubble_y, bubble_width, bubble_height)
        pygame.draw.rect(self.screen, SPEECH_BUBBLE_COLOR, bubble_rect, border_radius=8)
        pygame.draw.rect(self.screen, SPEECH_BUBBLE_BORDER, bubble_rect, 2, border_radius=8)
        
        # Draw pointer
        pointer_tip = (agent_px, agent_py - self.tile_size // 3)
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
    
    def draw_help_text(self):
        """Draw help text at bottom of screen"""
        help_text = "Press G to toggle grid | ESC to quit"
        help_surface = self.bubble_font.render(help_text, True, (100, 100, 100))
        help_rect = help_surface.get_rect(center=(self.window_width // 2, 
                                                  self.window_height - 15))
        self.screen.blit(help_surface, help_rect)
    
    def render(self):
        """Render the entire map"""
        # Clear screen
        self.screen.fill(BACKGROUND_COLOR)
        
        # Draw layers
        self.draw_background()
        self.draw_grid()
        self.draw_agents()
        
        # Draw example speech bubble on first agent
        if self.world.agents:
            self.draw_speech_bubble(self.world.agents[0], "Hello World")
        
        # Draw help
        self.draw_help_text()
        
        # Update display
        pygame.display.flip()
    
    def run(self):
        """Main loop"""
        clock = pygame.time.Clock()
        running = True
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_g:
                        self.show_grid = not self.show_grid
            
            self.render()
            clock.tick(30)
        
        pygame.quit()


def find_random_valid_position(world_map):
    """Find a random valid position for spawning an agent"""
    valid_positions = []
    
    for x in range(world_map.width):
        for y in range(world_map.height):
            patch = world_map.patches[x][y]
            terrain_name = patch.terrain_type.name if patch.terrain_type else ""
            
            if terrain_name not in ['Water', 'Mountain']:
                valid_positions.append((x, y))
    
    if not valid_positions:
        return (random.randint(0, world_map.width - 1),
                random.randint(0, world_map.height - 1))
    
    return random.choice(valid_positions)


def create_example_visualization(scenario_name='rural', map_width=30, map_height=30,
                                background_image=None):
    """
    Create and display a WorldMap with example agents.
    
    Args:
        scenario_name: Name of scenario module
        map_width: Width of map
        map_height: Height of map  
        background_image: Optional path to background image
    """
    # Load scenario
    scenario_module = importlib.import_module(f'maps.{scenario_name}')
    
    # Create WorldMap
    print(f"Creating {map_width}x{map_height} map from {scenario_name} scenario...")
    world_map = WorldMap(scenario_module, width=map_width, height=map_height)
    
    # Create agents
    print("Creating agents...")
    
    sam_x, sam_y = find_random_valid_position(world_map)
    samantha = MapAgent(sam_x, sam_y, world_map, "Samantha")
    
    joe_x, joe_y = find_random_valid_position(world_map)
    while (joe_x, joe_y) == (sam_x, sam_y):
        joe_x, joe_y = find_random_valid_position(world_map)
    joe = MapAgent(joe_x, joe_y, world_map, "Joe")
    
    print(f"Samantha at ({sam_x}, {sam_y})")
    print(f"Joe at ({joe_x}, {joe_y})")
    
    # Create visualizer
    print("Launching visualizer...")
    if background_image:
        print(f"Using background image: {background_image}")
    else:
        print("Using colored terrain tiles")
    
    visualizer = MapVisualizer(world_map, background_image_path=background_image)
    visualizer.run()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Visualize WorldMap with optional background')
    parser.add_argument('scenario', nargs='?', default='rural',
                       help='Scenario name (default: rural)')
    parser.add_argument('--background', '-b', type=str, default=None,
                       help='Path to background image from Schnell')
    parser.add_argument('--width', '-w', type=int, default=30,
                       help='Map width (default: 30)')
    parser.add_argument('--height', '-h', type=int, default=30,
                       help='Map height (default: 30)')
    
    args = parser.parse_args()
    
    create_example_visualization(
        scenario_name=args.scenario,
        map_width=args.width,
        map_height=args.height,
        background_image=args.background
    )
