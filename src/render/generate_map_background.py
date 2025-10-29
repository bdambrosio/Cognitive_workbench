"""
generate_map_background.py - Integrated workflow for creating Schnell backgrounds

This script:
1. Creates a WorldMap from a scenario
2. Generates a Schnell prompt describing the terrain
3. (Optionally) Calls Schnell to generate the image
4. Launches the visualizer with the background

Usage:
    # Generate prompt only
    python generate_map_background.py rural --prompt-only
    
    # Generate prompt and create background (requires Schnell API/tool)
    python generate_map_background.py rural --output map_bg.png
    
    # Use existing background
    python generate_map_background.py rural --use-existing map_bg.png
"""

import sys
import argparse
import importlib
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from world_map import WorldMap
from map_prompt_generator import (
    generate_schnell_prompt, 
    generate_schnell_prompt_simple,
    print_map_statistics
)


def generate_prompt_for_map(scenario_name, map_width=30, map_height=30, style='pastel'):
    """
    Generate WorldMap and return Schnell prompt.
    
    Args:
        scenario_name: Scenario module name
        map_width: Map width
        map_height: Map height
        style: Art style ('pastel', 'watercolor', 'cartoon')
        
    Returns:
        Tuple of (world_map, prompt_text)
    """
    # Load scenario
    scenario_module = importlib.import_module(f'maps.{scenario_name}')
    
    # Create map
    print(f"Creating {map_width}x{map_height} map from {scenario_name} scenario...")
    world_map = WorldMap(scenario_module, width=map_width, height=map_height)
    
    # Print statistics
    print_map_statistics(world_map)
    
    # Generate prompt
    print(f"\nGenerating {style} style prompt...")
    prompt = generate_schnell_prompt(world_map, style=style)
    
    return world_map, prompt


def call_schnell(prompt, output_path, width=1024, height=1024):
    """
    Call Schnell to generate image.
    
    NOTE: This is a placeholder - you'll need to integrate with your actual
    Schnell implementation. This could be:
    - A Python API call
    - A subprocess call to a CLI tool
    - An HTTP request to a server
    
    Args:
        prompt: Text prompt
        output_path: Where to save the image
        width: Image width
        height: Image height
        
    Returns:
        True if successful, False otherwise
    """
    print("\n" + "="*80)
    print("SCHNELL INTEGRATION REQUIRED")
    print("="*80)
    print("This is where you would call Schnell to generate the image.")
    print(f"\nPrompt: {prompt}")
    print(f"\nOutput path: {output_path}")
    print(f"Dimensions: {width}x{height}")
    print("\nPlease integrate with your Schnell implementation by modifying")
    print("the call_schnell() function in this script.")
    print("\nExample integrations:")
    print("  1. If Schnell is a Python module:")
    print("     import schnell")
    print("     image = schnell.generate(prompt, width=width, height=height)")
    print("     image.save(output_path)")
    print()
    print("  2. If Schnell is a CLI tool:")
    print("     import subprocess")
    print('     subprocess.run(["schnell", "--prompt", prompt, "--output", output_path])')
    print()
    print("  3. If Schnell is an API service:")
    print("     import requests")
    print('     response = requests.post("http://schnell-api/generate", ')
    print('                             json={"prompt": prompt})')
    print("     with open(output_path, 'wb') as f:")
    print("         f.write(response.content)")
    print("\n" + "="*80)
    
    return False


def launch_visualizer_with_background(scenario_name, background_path, 
                                      map_width=30, map_height=30):
    """Launch the visualizer with the generated background"""
    import map_visualizer_with_background
    
    map_visualizer_with_background.create_example_visualization(
        scenario_name=scenario_name,
        map_width=map_width,
        map_height=map_height,
        background_image=background_path
    )


def main():
    parser = argparse.ArgumentParser(
        description='Generate Schnell prompts and backgrounds for WorldMap visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Just show the prompt that would be generated
  python generate_map_background.py rural --prompt-only
  
  # Generate prompt and output path (for manual Schnell usage)
  python generate_map_background.py forest --output forest_bg.png --prompt-only
  
  # Generate prompt, create image with Schnell, and launch visualizer
  python generate_map_background.py rural --output rural_bg.png --generate
  
  # Use an existing background image
  python generate_map_background.py rural --use-existing my_background.png
        """
    )
    
    parser.add_argument('scenario', help='Scenario name (e.g., rural, forest)')
    parser.add_argument('--width', '-w', type=int, default=30,
                       help='Map width (default: 30)')
    parser.add_argument('--height', type=int, default=30,
                       help='Map height (default: 30)')
    parser.add_argument('--style', '-s', choices=['pastel', 'watercolor', 'cartoon'],
                       default='pastel', help='Art style (default: pastel)')
    parser.add_argument('--output', '-o', type=str,
                       help='Output path for generated image')
    parser.add_argument('--prompt-only', action='store_true',
                       help='Only generate and display the prompt (don\'t call Schnell)')
    parser.add_argument('--simple', action='store_true',
                       help='Use simpler, more concise prompt')
    parser.add_argument('--generate', '-g', action='store_true',
                       help='Call Schnell to generate the image (requires integration)')
    parser.add_argument('--use-existing', type=str,
                       help='Skip generation and use existing background image')
    parser.add_argument('--no-visualize', action='store_true',
                       help='Don\'t launch visualizer after generation')
    
    args = parser.parse_args()
    
    # If using existing background, skip straight to visualization
    if args.use_existing:
        if not Path(args.use_existing).exists():
            print(f"Error: Background image not found: {args.use_existing}")
            return 1
        
        print(f"Using existing background: {args.use_existing}")
        launch_visualizer_with_background(
            args.scenario, 
            args.use_existing,
            args.width, 
            args.height
        )
        return 0
    
    # Generate prompt
    world_map, prompt = generate_prompt_for_map(
        args.scenario, 
        args.width, 
        args.height, 
        args.style
    )
    
    # Display prompt
    print("\n" + "="*80)
    print("GENERATED SCHNELL PROMPT")
    print("="*80)
    print(prompt)
    print("="*80 + "\n")
    
    if args.simple:
        from map_prompt_generator import generate_schnell_prompt_simple
        simple = generate_schnell_prompt_simple(world_map)
        print("SIMPLE VERSION:")
        print("-"*80)
        print(simple)
        print("-"*80 + "\n")
    
    # If prompt-only, we're done
    if args.prompt_only:
        if args.output:
            print(f"\nTo generate the image with Schnell, save it to: {args.output}")
        return 0
    
    # If generate flag is set, call Schnell
    if args.generate:
        if not args.output:
            print("Error: --output required when using --generate")
            return 1
        
        print("\nAttempting to generate image with Schnell...")
        success = call_schnell(prompt, args.output, width=1024, height=1024)
        
        if not success:
            print("\nSchnell integration not configured.")
            print("Please manually generate the image using the prompt above,")
            print(f"then save it to: {args.output}")
            print(f"\nThen run:")
            print(f"  python generate_map_background.py {args.scenario} --use-existing {args.output}")
            return 1
        
        # If successful and visualization requested
        if not args.no_visualize:
            print(f"\nLaunching visualizer with background: {args.output}")
            launch_visualizer_with_background(
                args.scenario,
                args.output,
                args.width,
                args.height
            )
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
