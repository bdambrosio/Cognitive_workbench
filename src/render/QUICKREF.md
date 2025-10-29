# Quick Reference Guide

## Common Tasks

### 1. Quick Visualization Test
```bash
# Test that everything works
python test_visualization.py

# Test individual components
python test_visualization.py agent      # Agent creation
python test_visualization.py prompt     # Prompt generation
python test_visualization.py basic      # Visualization
```

### 2. Visualize with Colored Tiles
```bash
# Basic usage
python map_visualizer.py rural

# Different scenarios
python map_visualizer.py forest
python map_visualizer.py rural
```

### 3. Generate Schnell Prompt
```bash
# See what prompt would be generated
python generate_map_background.py rural --prompt-only

# Generate for different scenarios
python generate_map_background.py forest --prompt-only --style watercolor

# Get a simpler prompt
python generate_map_background.py rural --prompt-only --simple
```

### 4. Use Schnell Background
```bash
# Step 1: Generate prompt
python generate_map_background.py rural --prompt-only > prompt.txt

# Step 2: Use Schnell (external tool) to create image from prompt
#         Save as rural_bg.png (1024x1024 recommended)

# Step 3: Visualize with background
python map_visualizer_with_background.py rural --background rural_bg.png
```

### 5. Custom Map Sizes
```bash
# Smaller map (faster generation)
python map_visualizer.py rural --width 20 --height 20

# Larger map (more detail)
python map_visualizer.py forest --width 40 --height 40
```

## Workflow Summary

### Without Schnell (Fastest)
```
Create Map → Colored Tiles Visualization
(1 command, instant)
```

### With Schnell (Most Beautiful)
```
Create Map → Generate Prompt → Schnell → Background Visualization
(3 steps, ~1-2 minutes depending on Schnell)
```

## File Purposes

| File | Purpose |
|------|---------|
| `map_visualizer.py` | Basic colored tile visualization |
| `map_visualizer_with_background.py` | Visualization with optional background image |
| `map_prompt_generator.py` | Generate Schnell prompts from terrain |
| `generate_map_background.py` | Integrated workflow script |
| `test_visualization.py` | Test suite for all components |

## Key Features

### Visualizer Controls
- **ESC** - Quit
- **G** - Toggle grid (background mode only)

### Agent Display
- Emoji characters: 😊, 🙂, 😄, etc.
- White circle backgrounds for visibility
- Names displayed below each agent
- Speech bubbles with comic-style pointers

### Terrain Colors (Fallback Mode)
- Water: Light blue
- Mountain: Gray
- Forest: Forest green
- Grassland: Light green
- Field: Khaki/wheat
- Clearing: Pale green
- Meadow: Medium green

## Schnell Integration

To automate the full workflow, edit `call_schnell()` in `generate_map_background.py`:

```python
def call_schnell(prompt, output_path, width=1024, height=1024):
    # Add your Schnell integration here
    # Return True if successful, False otherwise
    pass
```

Example integrations provided in README_VISUALIZATION.md

## Troubleshooting

**Import errors:**
```bash
# Ensure src/ is in Python path
export PYTHONPATH="${PYTHONPATH}:./src"
```

**Pygame not installed:**
```bash
pip install pygame --break-system-packages
```

**No scenario found:**
- Scenarios must be in `src/maps/` directory
- Use filename without .py extension
- Example: `python map_visualizer.py rural` looks for `src/maps/rural.py`

**Background image issues:**
- Use square images (1024x1024 recommended)
- Supported formats: PNG, JPG
- Path can be relative or absolute

## Next Steps

1. Test basic visualization: `python test_visualization.py`
2. Generate a Schnell prompt: `python generate_map_background.py rural --prompt-only`
3. Use Schnell to create the background image
4. Visualize with background: `python map_visualizer_with_background.py rural --background your_image.png`

## Example Session

```bash
# Terminal 1: Generate prompt
$ python generate_map_background.py forest --prompt-only
Creating 30x30 map from forest scenario...
WorldMap initialized

=== Map Analysis ===
Dimensions: 30x30
Total patches: 900

Terrain Distribution:
  Forest      : 55.2%
  Clearing    : 18.3%
  Meadow      : 12.1%
  Water       :  8.4%
  Mountain    :  6.0%

GENERATED SCHNELL PROMPT
================================================================================
A soft pastel painting of a bird's eye view landscape map dominated by lush 
green forest with dense tree canopy, with patches of open meadow clearings 
with soft grass, flowering meadows with colorful blooms...
================================================================================

# Copy the prompt, use in Schnell to generate forest_bg.png

# Terminal 2: Visualize with background
$ python map_visualizer_with_background.py forest --background forest_bg.png
Creating 30x30 map from forest scenario...
WorldMap initialized
Creating agents...
Samantha at (12, 8)
Joe at (18, 22)
Launching visualizer...
Loaded background image: forest_bg.png

# Pygame window opens with beautiful forest background!
```
