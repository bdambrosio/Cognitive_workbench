"""
test_visualization.py - Quick test of map visualization system

Tests both colored tile mode and background image mode (if image provided).
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))


def test_basic_visualization():
    """Test basic colored tile visualization"""
    print("="*80)
    print("TEST 1: Basic Colored Tile Visualization")
    print("="*80)
    
    try:
        import map_visualizer
        print("✓ map_visualizer.py imported successfully")
        
        # Test with rural scenario
        print("\nCreating rural map with agents...")
        map_visualizer.create_example_visualization('rural', 30, 30)
        
        print("✓ Test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_prompt_generation():
    """Test Schnell prompt generation"""
    print("\n" + "="*80)
    print("TEST 2: Schnell Prompt Generation")
    print("="*80)
    
    try:
        import importlib
        from world_map import WorldMap
        from map_prompt_generator import (
            generate_schnell_prompt,
            generate_schnell_prompt_simple,
            analyze_terrain_layout
        )
        
        print("✓ Modules imported successfully")
        
        # Create a test map
        print("\nCreating test map (forest scenario)...")
        scenario = importlib.import_module('maps.forest')
        world_map = WorldMap(scenario, width=20, height=20)
        print(f"✓ Created {world_map.width}x{world_map.height} map")
        
        # Test terrain analysis
        print("\nAnalyzing terrain...")
        analysis = analyze_terrain_layout(world_map)
        print(f"✓ Found {len(analysis['terrain_counts'])} terrain types")
        print(f"  Dominant terrain: {analysis['dominant_terrain']}")
        
        # Test prompt generation
        print("\nGenerating prompts...")
        detailed = generate_schnell_prompt(world_map, style='pastel')
        simple = generate_schnell_prompt_simple(world_map)
        
        print(f"✓ Detailed prompt: {len(detailed)} characters")
        print(f"✓ Simple prompt: {len(simple)} characters")
        
        print("\nDetailed prompt preview:")
        print("-"*80)
        print(detailed[:200] + "...")
        print("-"*80)
        
        print("\n✓ Test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_background_mode():
    """Test background image mode (if test image exists)"""
    print("\n" + "="*80)
    print("TEST 3: Background Image Mode")
    print("="*80)
    
    # Check if test background exists
    test_bg_path = Path(__file__).parent / "test_background.png"
    
    if not test_bg_path.exists():
        print("ℹ Test background image not found")
        print(f"  Expected location: {test_bg_path}")
        print("  Skipping background image test")
        print("\nTo test background mode:")
        print("  1. Generate a test image with Schnell")
        print("  2. Save as test_background.png")
        print("  3. Run this test again")
        return None
    
    try:
        import map_visualizer_with_background
        print(f"✓ Found test background: {test_bg_path}")
        
        print("\nLaunching visualizer with background...")
        map_visualizer_with_background.create_example_visualization(
            'rural', 30, 30, 
            background_image=str(test_bg_path)
        )
        
        print("✓ Test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_agent_creation():
    """Test agent creation and registration"""
    print("\n" + "="*80)
    print("TEST 4: Agent Creation and Registration")
    print("="*80)
    
    try:
        import importlib
        from world_map import WorldMap
        from map_agent import MapAgent
        
        print("✓ Modules imported successfully")
        
        # Create map
        print("\nCreating map...")
        scenario = importlib.import_module('maps.rural')
        world_map = WorldMap(scenario, width=15, height=15)
        print(f"✓ Created map")
        
        # Verify no agents initially
        assert len(world_map.agents) == 0, "Map should start with no agents"
        print("✓ Map starts with no agents")
        
        # Create and register agents
        print("\nCreating agents...")
        samantha = MapAgent(5, 5, world_map, "Samantha")
        print(f"✓ Created Samantha at ({samantha.x}, {samantha.y})")
        
        joe = MapAgent(10, 10, world_map, "Joe")
        print(f"✓ Created Joe at ({joe.x}, {joe.y})")
        
        # Verify registration
        assert len(world_map.agents) == 2, "Should have 2 agents"
        print(f"✓ Both agents registered: {len(world_map.agents)} total")
        
        # Verify retrieval
        sam = world_map.get_agent("Samantha")
        assert sam is not None, "Should find Samantha"
        assert sam.name == "Samantha", "Name should match"
        print("✓ Agent retrieval works")
        
        print("\n✓ Test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_layout_generation():
    """Test layout image generation for ComfyUI"""
    print("\n" + "="*80)
    print("TEST 5: Layout Image Generation")
    print("="*80)
    
    try:
        from generate_layout_image import (
            generate_layout_image,
            generate_layout_image_with_elevation,
            generate_annotated_layout
        )
        import importlib
        from world_map import WorldMap
        
        print("✓ Modules imported successfully")
        
        # Create test map
        print("\nCreating test map...")
        scenario = importlib.import_module('maps.rural')
        world_map = WorldMap(scenario, width=20, height=20)
        print("✓ Map created")
        
        # Test basic layout generation
        print("\nGenerating basic layout image...")
        img = generate_layout_image(world_map, output_size=512)
        print(f"✓ Generated {img.size[0]}x{img.size[1]} image")
        
        # Test elevation-based layout
        print("\nGenerating elevation-based layout...")
        img_elev = generate_layout_image_with_elevation(world_map, output_size=512)
        print(f"✓ Generated elevation layout")
        
        # Test annotated layout
        print("\nGenerating annotated layout...")
        img_annot = generate_annotated_layout(world_map, output_size=512)
        print(f"✓ Generated annotated layout with legend")
        
        # Save test outputs
        test_dir = Path(__file__).parent / "test_outputs"
        test_dir.mkdir(exist_ok=True)
        
        img.save(test_dir / "test_layout.png")
        img_elev.save(test_dir / "test_layout_elevation.png")
        img_annot.save(test_dir / "test_layout_annotated.png")
        
        print(f"\n✓ Saved test images to {test_dir}/")
        print("  - test_layout.png (basic)")
        print("  - test_layout_elevation.png (with elevation)")
        print("  - test_layout_annotated.png (with legend)")
        
        print("\n✓ Test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_tests():
    """Run all tests"""
    print("\n" + "="*80)
    print("MAP VISUALIZATION TEST SUITE")
    print("="*80 + "\n")
    
    results = {
        'Agent Creation': test_agent_creation(),
        'Prompt Generation': test_prompt_generation(),
        'Layout Generation': test_layout_generation(),
        'Background Mode': test_background_mode(),
        'Basic Visualization': test_basic_visualization(),  # This should be last (opens window)
    }
    
    print("\n" + "="*80)
    print("TEST SUMMARY")
    print("="*80)
    
    for test_name, result in results.items():
        if result is True:
            status = "✓ PASS"
        elif result is False:
            status = "✗ FAIL"
        else:
            status = "⊘ SKIP"
        print(f"{status:10s} {test_name}")
    
    passed = sum(1 for r in results.values() if r is True)
    failed = sum(1 for r in results.values() if r is False)
    skipped = sum(1 for r in results.values() if r is None)
    
    print(f"\nResults: {passed} passed, {failed} failed, {skipped} skipped")
    
    if failed > 0:
        print("\n⚠ Some tests failed. Please check the output above.")
        return 1
    else:
        print("\n✓ All tests passed!")
        return 0


if __name__ == "__main__":
    if len(sys.argv) > 1:
        test_name = sys.argv[1].lower()
        
        if test_name == 'agent':
            test_agent_creation()
        elif test_name == 'prompt':
            test_prompt_generation()
        elif test_name == 'layout':
            test_layout_generation()
        elif test_name == 'background':
            test_background_mode()
        elif test_name == 'basic':
            test_basic_visualization()
        else:
            print(f"Unknown test: {test_name}")
            print("Available tests: agent, prompt, layout, background, basic")
            sys.exit(1)
    else:
        sys.exit(run_all_tests())
