"""
generate_layout_image.py - Generate smooth layout images for ComfyUI img2img

Creates a clean, smoothed terrain layout image that can be used as input for
ComfyUI img2img workflows with Schnell. The layout preserves the spatial
arrangement of terrain types while providing a good base for artistic rendering.

Output is a high-resolution image (1024x1024) with smooth color gradients
representing different terrain types.
"""

import sys
import numpy as np
from pathlib import Path
from PIL import Image, ImageDraw, ImageFilter, ImageFont
import importlib

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from world_map import WorldMap


# Terrain color palette - saturated colors for good img2img input
LAYOUT_COLORS = {
    'Water': (80, 150, 220),       # Blue
    'Mountain': (140, 140, 140),    # Gray
    'Forest': (60, 120, 60),        # Dark green
    'Grassland': (120, 180, 100),   # Light green
    'Field': (200, 180, 100),       # Yellow-tan
    'Clearing': (150, 170, 110),    # Olive green
    'Meadow': (140, 200, 120),      # Bright green
}

DEFAULT_COLOR = (160, 160, 160)  # Gray


def get_terrain_color(terrain_type):
    """Get color for a terrain type"""
    if terrain_type is None:
        return DEFAULT_COLOR
    terrain_name = terrain_type.name if hasattr(terrain_type, 'name') else str(terrain_type)
    return LAYOUT_COLORS.get(terrain_name, DEFAULT_COLOR)


def generate_layout_image(world_map, output_size=1024, smoothing=True, 
                          smoothing_radius=2, supersample=4):
    """
    Generate a smooth layout image from WorldMap.
    
    Args:
        world_map: WorldMap instance
        output_size: Output image size (square)
        smoothing: Whether to apply Gaussian blur for smooth transitions
        smoothing_radius: Blur radius in pixels
        supersample: Render at higher resolution then downscale for anti-aliasing
        
    Returns:
        PIL Image object
    """
    # Calculate render size (with supersampling for anti-aliasing)
    render_size = output_size * supersample
    
    # Calculate tile size
    tile_size = render_size // max(world_map.width, world_map.height)
    actual_width = tile_size * world_map.width
    actual_height = tile_size * world_map.height
    
    # Create image
    img = Image.new('RGB', (actual_width, actual_height))
    pixels = img.load()
    
    # Fill in terrain colors
    for x in range(world_map.width):
        for y in range(world_map.height):
            patch = world_map.patches[x][y]
            color = get_terrain_color(patch.terrain_type)
            
            # Fill the tile
            for px in range(x * tile_size, (x + 1) * tile_size):
                for py in range(y * tile_size, (y + 1) * tile_size):
                    pixels[px, py] = color
    
    # Apply smoothing for natural terrain transitions
    if smoothing:
        # Gaussian blur creates smooth color gradients between terrain types
        img = img.filter(ImageFilter.GaussianBlur(radius=smoothing_radius * supersample))
    
    # Resize to final output size (provides anti-aliasing)
    if actual_width != output_size or actual_height != output_size:
        img = img.resize((output_size, output_size), Image.Resampling.LANCZOS)
    
    return img


def generate_layout_image_with_elevation(world_map, output_size=1024, 
                                         elevation_influence=0.3):
    """
    Generate layout image with elevation-based shading for more realistic terrain.
    
    Higher elevation = lighter shade, lower elevation = darker shade.
    This helps Schnell understand the topography.
    
    Args:
        world_map: WorldMap instance
        output_size: Output image size (square)
        elevation_influence: How much elevation affects brightness (0.0-1.0)
        
    Returns:
        PIL Image object
    """
    supersample = 4
    render_size = output_size * supersample
    tile_size = render_size // max(world_map.width, world_map.height)
    actual_width = tile_size * world_map.width
    actual_height = tile_size * world_map.height
    
    # Find elevation range
    elevations = []
    for x in range(world_map.width):
        for y in range(world_map.height):
            elevations.append(world_map.patches[x][y].elevation)
    
    min_elev = min(elevations)
    max_elev = max(elevations)
    elev_range = max_elev - min_elev if max_elev > min_elev else 1.0
    
    # Create image
    img = Image.new('RGB', (actual_width, actual_height))
    pixels = img.load()
    
    # Fill with terrain colors adjusted by elevation
    for x in range(world_map.width):
        for y in range(world_map.height):
            patch = world_map.patches[x][y]
            base_color = get_terrain_color(patch.terrain_type)
            
            # Calculate elevation factor (0.0 = lowest, 1.0 = highest)
            elev_factor = (patch.elevation - min_elev) / elev_range
            
            # Adjust brightness based on elevation
            # Higher elevation = lighter, lower = darker
            brightness_mult = 1.0 + elevation_influence * (elev_factor - 0.5)
            
            adjusted_color = tuple(
                int(min(255, max(0, c * brightness_mult)))
                for c in base_color
            )
            
            # Fill the tile
            for px in range(x * tile_size, (x + 1) * tile_size):
                for py in range(y * tile_size, (y + 1) * tile_size):
                    pixels[px, py] = adjusted_color
    
    # Apply smoothing
    img = img.filter(ImageFilter.GaussianBlur(radius=2 * supersample))
    
    # Resize to final size
    img = img.resize((output_size, output_size), Image.Resampling.LANCZOS)
    
    return img


def generate_annotated_layout(world_map, output_size=1024, show_legend=True):
    """
    Generate layout image with optional legend showing terrain types.
    
    Useful for understanding what colors represent which terrain.
    
    Args:
        world_map: WorldMap instance
        output_size: Output image size
        show_legend: Whether to add a legend
        
    Returns:
        PIL Image object
    """
    # Generate base layout
    img = generate_layout_image(world_map, output_size, smoothing=True)
    
    if show_legend:
        # Get unique terrain types in the map
        terrain_types_present = set()
        for x in range(world_map.width):
            for y in range(world_map.height):
                terrain = world_map.patches[x][y].terrain_type
                if terrain:
                    terrain_types_present.add(terrain.name)
        
        # Create legend
        draw = ImageDraw.Draw(img)
        
        # Try to load a font
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
        except:
            font = None  # Will use default
        
        # Draw legend in top-right corner
        legend_x = output_size - 150
        legend_y = 10
        box_size = 15
        spacing = 25
        
        # Draw semi-transparent background
        legend_height = len(terrain_types_present) * spacing + 10
        draw.rectangle(
            [(legend_x - 10, legend_y - 5), 
             (output_size - 5, legend_y + legend_height)],
            fill=(255, 255, 255, 200)
        )
        
        # Draw each terrain type
        for terrain_name in sorted(terrain_types_present):
            color = LAYOUT_COLORS.get(terrain_name, DEFAULT_COLOR)
            
            # Draw color box
            draw.rectangle(
                [(legend_x, legend_y), (legend_x + box_size, legend_y + box_size)],
                fill=color,
                outline=(0, 0, 0)
            )
            
            # Draw label
            draw.text(
                (legend_x + box_size + 5, legend_y),
                terrain_name,
                fill=(0, 0, 0),
                font=font
            )
            
            legend_y += spacing
    
    return img


def generate_comfyui_prompt(world_map, style='pastel'):
    """
    Generate a simplified prompt for ComfyUI img2img workflow.
    
    Since the layout image already shows the spatial arrangement,
    the prompt focuses on artistic style and atmosphere.
    
    Args:
        world_map: WorldMap instance
        style: Art style preference
        
    Returns:
        String prompt for ComfyUI
    """
    style_prompts = {
        'pastel': (
            "Transform this terrain map into a soft pastel painting, "
            "bird's eye view, dreamy watercolor style, "
            "gentle color gradients, muted tones, "
            "artistic interpretation while preserving the layout, "
            "suitable for children's storybook, "
            "no text, no labels, no UI elements, "
            "high quality, 4k"
        ),
        'watercolor': (
            "Transform this terrain map into a watercolor painting, "
            "top-down aerial view, flowing colors, wet-on-wet technique, "
            "soft edges, artistic style, "
            "preserve the spatial layout while adding artistic beauty, "
            "no text, no labels, "
            "high quality"
        ),
        'cartoon': (
            "Transform this terrain map into a colorful cartoon landscape, "
            "bird's eye view, bright cheerful colors, "
            "clear but artistic outlines, playful style, "
            "maintain the terrain layout, "
            "no text, no characters, "
            "high quality"
        ),
        'realistic': (
            "Transform this terrain map into a realistic aerial photograph, "
            "natural colors, photorealistic style, "
            "detailed textures, satellite view aesthetic, "
            "preserve all terrain boundaries, "
            "no text, no labels, "
            "high quality, 4k"
        )
    }
    
    return style_prompts.get(style, style_prompts['pastel'])


def save_layout_with_metadata(world_map, output_path, style='pastel', 
                              output_size=1024, use_elevation=False):
    """
    Generate and save layout image with accompanying text file.
    
    Args:
        world_map: WorldMap instance
        output_path: Path to save image
        style: Style for ComfyUI prompt
        output_size: Image size
        use_elevation: Whether to include elevation shading
        
    Returns:
        Tuple of (image_path, prompt_path)
    """
    output_path = Path(output_path)
    
    # Generate layout image
    print(f"Generating layout image ({output_size}x{output_size})...")
    if use_elevation:
        img = generate_layout_image_with_elevation(world_map, output_size)
        print("  ✓ Using elevation-based shading")
    else:
        img = generate_layout_image(world_map, output_size, smoothing=True)
        print("  ✓ Using smooth color blending")
    
    # Save image
    img.save(output_path)
    print(f"  ✓ Saved layout image: {output_path}")
    
    # Generate prompt
    prompt = generate_comfyui_prompt(world_map, style)
    
    # Save prompt to text file
    prompt_path = output_path.with_suffix('.txt')
    with open(prompt_path, 'w') as f:
        f.write(prompt)
    print(f"  ✓ Saved prompt: {prompt_path}")
    
    # Also save annotated version for reference
    annotated_path = output_path.with_stem(output_path.stem + '_annotated')
    annotated_img = generate_annotated_layout(world_map, output_size)
    annotated_img.save(annotated_path)
    print(f"  ✓ Saved annotated version: {annotated_path}")
    
    return str(output_path), str(prompt_path)


def print_comfyui_instructions(image_path, prompt_path):
    """Print instructions for using the generated files in ComfyUI"""
    print("\n" + "="*80)
    print("COMFYUI WORKFLOW INSTRUCTIONS")
    print("="*80)
    print("\n1. Open ComfyUI and load an img2img workflow")
    print("   (or use the default workflow and add 'Load Image' node)")
    print()
    print("2. Load the layout image:")
    print(f"   → {image_path}")
    print()
    print("3. Load the prompt from:")
    print(f"   → {prompt_path}")
    print()
    print("4. Recommended settings:")
    print("   - Denoising strength: 0.6-0.8")
    print("   - Higher strength = more artistic, may lose layout")
    print("   - Lower strength = preserves layout, less artistic")
    print()
    print("5. If using ControlNet (advanced):")
    print("   - Use Canny or Tile ControlNet")
    print("   - This will better preserve the terrain boundaries")
    print()
    print("6. Generate!")
    print("   - The output should have the same terrain layout")
    print("   - But rendered in beautiful Schnell style")
    print()
    print("="*80)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Generate layout images for ComfyUI img2img workflow',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage - generate layout for rural scenario
  python generate_layout_image.py rural -o rural_layout.png
  
  # With elevation shading for more realistic topography
  python generate_layout_image.py forest -o forest_layout.png --elevation
  
  # Different art style prompts
  python generate_layout_image.py rural -o rural_layout.png --style watercolor
  
  # Higher resolution output
  python generate_layout_image.py forest -o forest_layout.png --size 2048
        """
    )
    
    parser.add_argument('scenario', help='Scenario name (e.g., rural, forest)')
    parser.add_argument('-o', '--output', required=True,
                       help='Output image path (e.g., layout.png)')
    parser.add_argument('--width', '-w', type=int, default=30,
                       help='Map width (default: 30)')
    parser.add_argument('--height', '-ht', type=int, default=30,
                       help='Map height (default: 30)')
    parser.add_argument('--size', '-s', type=int, default=1024,
                       help='Output image size in pixels (default: 1024)')
    parser.add_argument('--style', choices=['pastel', 'watercolor', 'cartoon', 'realistic'],
                       default='pastel', help='Art style for prompt (default: pastel)')
    parser.add_argument('--elevation', '-e', action='store_true',
                       help='Use elevation-based shading')
    
    args = parser.parse_args()
    
    # Load scenario
    print(f"Loading {args.scenario} scenario...")
    scenario_module = importlib.import_module(f'maps.{args.scenario}')
    
    # Create WorldMap
    print(f"Creating {args.width}x{args.height} map...")
    world_map = WorldMap(scenario_module, width=args.width, height=args.height)
    
    # Generate and save layout
    image_path, prompt_path = save_layout_with_metadata(
        world_map,
        args.output,
        style=args.style,
        output_size=args.size,
        use_elevation=args.elevation
    )
    
    # Print instructions
    print_comfyui_instructions(image_path, prompt_path)
    
    print("\n✓ All files generated successfully!")
    print(f"\nNext step: Load {image_path} into ComfyUI img2img workflow")
