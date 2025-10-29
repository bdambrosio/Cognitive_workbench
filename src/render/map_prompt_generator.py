"""
map_prompt_generator.py - Generate Schnell prompts for WorldMap visualization

Creates text-to-image prompts that describe the terrain layout of a WorldMap
in a way that produces beautiful pastel top-down landscape paintings.
"""

import sys
from pathlib import Path
from collections import Counter

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from world_map import WorldMap


def analyze_terrain_layout(world_map):
    """
    Analyze the spatial distribution of terrain types.
    
    Returns:
        dict with terrain statistics and spatial features
    """
    terrain_counts = Counter()
    terrain_regions = {}  # Track major regions of each terrain type
    
    for x in range(world_map.width):
        for y in range(world_map.height):
            terrain = world_map.patches[x][y].terrain_type
            terrain_name = terrain.name if terrain else "Unknown"
            terrain_counts[terrain_name] += 1
    
    total_patches = world_map.width * world_map.height
    terrain_percentages = {
        terrain: (count / total_patches) * 100 
        for terrain, count in terrain_counts.items()
    }
    
    # Identify major terrain features by quadrant
    quadrants = {
        'northwest': analyze_quadrant(world_map, 0, 0, world_map.width//2, world_map.height//2),
        'northeast': analyze_quadrant(world_map, world_map.width//2, 0, world_map.width, world_map.height//2),
        'southwest': analyze_quadrant(world_map, 0, world_map.height//2, world_map.width//2, world_map.height),
        'southeast': analyze_quadrant(world_map, world_map.width//2, world_map.height//2, world_map.width, world_map.height),
    }
    
    return {
        'terrain_counts': terrain_counts,
        'terrain_percentages': terrain_percentages,
        'quadrants': quadrants,
        'dominant_terrain': terrain_counts.most_common(1)[0][0] if terrain_counts else 'Unknown'
    }


def analyze_quadrant(world_map, x_start, y_start, x_end, y_end):
    """Analyze dominant terrain in a quadrant"""
    terrain_counts = Counter()
    
    for x in range(x_start, x_end):
        for y in range(y_start, y_end):
            if x < world_map.width and y < world_map.height:
                terrain = world_map.patches[x][y].terrain_type
                terrain_name = terrain.name if terrain else "Unknown"
                terrain_counts[terrain_name] += 1
    
    return terrain_counts.most_common(1)[0][0] if terrain_counts else 'Unknown'


def get_terrain_description(terrain_name):
    """Get artistic description for a terrain type"""
    descriptions = {
        'Water': 'calm blue water with gentle ripples',
        'Mountain': 'rocky gray mountains with snow-capped peaks',
        'Forest': 'lush green forest with dense tree canopy',
        'Grassland': 'rolling grassland with wildflowers',
        'Field': 'golden wheat fields with organized rows',
        'Clearing': 'open meadow clearings with soft grass',
        'Meadow': 'flowering meadows with colorful blooms',
    }
    return descriptions.get(terrain_name, f'{terrain_name.lower()} terrain')


def get_terrain_color_hint(terrain_name):
    """Get pastel color hints for terrain"""
    colors = {
        'Water': 'soft powder blue',
        'Mountain': 'pale gray and white',
        'Forest': 'sage green and olive',
        'Grassland': 'mint green and lime',
        'Field': 'pale yellow and cream',
        'Clearing': 'light sage and beige',
        'Meadow': 'pastel pink and lavender mixed with green',
    }
    return colors.get(terrain_name, 'muted earth tones')


def generate_schnell_prompt(world_map, style='pastel'):
    """
    Generate a Schnell prompt for the world map.
    
    Args:
        world_map: WorldMap instance
        style: Art style ('pastel', 'watercolor', 'cartoon')
        
    Returns:
        String prompt for text-to-image generation
    """
    analysis = analyze_terrain_layout(world_map)
    
    # Build prompt components
    components = []
    
    # Base description
    if style == 'pastel':
        components.append("A soft pastel painting of a bird's eye view landscape map")
    elif style == 'watercolor':
        components.append("A gentle watercolor painting showing an aerial view of a landscape")
    elif style == 'cartoon':
        components.append("A colorful cartoon-style top-down map of a fantasy landscape")
    else:
        components.append("An aerial view painting of a landscape map")
    
    # Add dominant terrain
    dominant = analysis['dominant_terrain']
    components.append(f"dominated by {get_terrain_description(dominant)}")
    
    # Add significant terrain features (>10% of map)
    significant_terrains = [
        terrain for terrain, pct in analysis['terrain_percentages'].items()
        if pct > 10 and terrain != dominant
    ]
    
    if significant_terrains:
        terrain_descs = [get_terrain_description(t) for t in significant_terrains[:3]]
        components.append(f"with patches of {', '.join(terrain_descs)}")
    
    # Add spatial layout hints from quadrants
    quadrants = analysis['quadrants']
    unique_quadrants = set(quadrants.values())
    
    if len(unique_quadrants) > 1:
        # Describe notable quadrant features
        if quadrants['northwest'] == 'Water':
            components.append("a lake in the upper left")
        elif quadrants['northwest'] == 'Mountain':
            components.append("mountains in the upper left")
        
        if quadrants['southeast'] == 'Forest':
            components.append("dense forest in the lower right")
    
    # Add color palette description
    color_hints = []
    for terrain, pct in sorted(analysis['terrain_percentages'].items(), 
                               key=lambda x: x[1], reverse=True)[:4]:
        if pct > 5:  # Only mention terrains covering >5%
            color_hints.append(get_terrain_color_hint(terrain))
    
    if color_hints:
        components.append(f"Colors: {', '.join(color_hints)}")
    
    # Style modifiers
    if style == 'pastel':
        components.append("soft lighting, muted colors, gentle gradients, dreamy atmosphere")
    elif style == 'watercolor':
        components.append("flowing colors, wet-on-wet technique, soft edges, artistic")
    elif style == 'cartoon':
        components.append("bright cheerful colors, clear outlines, playful style")
    
    # Quality and view modifiers
    components.append("top-down view, square composition, suitable for game map background")
    components.append("no text, no labels, no UI elements, no characters")
    components.append("high quality, detailed, 4k")
    
    # Combine into prompt
    prompt = ", ".join(components)
    
    return prompt


def generate_schnell_prompt_simple(world_map):
    """
    Generate a simpler, more concise prompt for faster generation.
    
    Args:
        world_map: WorldMap instance
        
    Returns:
        Concise prompt string
    """
    analysis = analyze_terrain_layout(world_map)
    
    # Get top 3 terrain types
    top_terrains = analysis['terrain_counts'].most_common(3)
    
    terrain_list = []
    for terrain, count in top_terrains:
        pct = analysis['terrain_percentages'][terrain]
        if pct > 5:  # Only mention if >5% of map
            terrain_list.append(get_terrain_description(terrain))
    
    prompt = (
        f"Pastel painting, bird's eye view map, "
        f"{', '.join(terrain_list)}, "
        f"soft colors, top-down perspective, square format, "
        f"no text, no labels, dreamy atmosphere, art style suitable for children's storybook"
    )
    
    return prompt


def print_map_statistics(world_map):
    """Print helpful statistics about the map"""
    analysis = analyze_terrain_layout(world_map)
    
    print(f"\n=== Map Analysis ===")
    print(f"Dimensions: {world_map.width}x{world_map.height}")
    print(f"Total patches: {world_map.width * world_map.height}")
    print(f"\nTerrain Distribution:")
    for terrain, pct in sorted(analysis['terrain_percentages'].items(), 
                              key=lambda x: x[1], reverse=True):
        print(f"  {terrain:12s}: {pct:5.1f}%")
    
    print(f"\nQuadrant Analysis:")
    for quadrant, terrain in analysis['quadrants'].items():
        print(f"  {quadrant:12s}: {terrain}")
    
    print(f"\n=== Generated Prompts ===\n")


if __name__ == "__main__":
    import importlib
    
    # Load scenario
    scenario_name = sys.argv[1] if len(sys.argv) > 1 else 'rural'
    scenario_module = importlib.import_module(f'maps.{scenario_name}')
    
    # Create map
    print(f"Generating map from {scenario_name} scenario...")
    world_map = WorldMap(scenario_module, width=30, height=30)
    
    # Print statistics
    print_map_statistics(world_map)
    
    # Generate prompts
    print("DETAILED PROMPT:")
    print("-" * 80)
    detailed_prompt = generate_schnell_prompt(world_map, style='pastel')
    print(detailed_prompt)
    print()
    
    print("\nSIMPLE PROMPT:")
    print("-" * 80)
    simple_prompt = generate_schnell_prompt_simple(world_map)
    print(simple_prompt)
    print()
    
    print("\nALTERNATE STYLE - WATERCOLOR:")
    print("-" * 80)
    watercolor_prompt = generate_schnell_prompt(world_map, style='watercolor')
    print(watercolor_prompt)
    print()
