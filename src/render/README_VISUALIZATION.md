# WorldMap Visualization with Schnell Backgrounds

Beautiful cartoon-style visualization of your WorldMap scenarios with optional AI-generated backgrounds.

## Overview

This visualization system provides three ways to display your maps:

1. **Simple colored tiles** - Quick, immediate visualization with color-coded terrain
2. **Schnell-generated backgrounds** - Beautiful pastel landscape paintings as backgrounds
3. **Hybrid approach** - AI background with optional grid overlay

## Files

- `map_visualizer.py` - Basic visualizer with colored terrain tiles
- `map_visualizer_with_background.py` - Enhanced visualizer supporting background images
- `map_prompt_generator.py` - Generates Schnell prompts from WorldMap terrain
- `generate_map_background.py` - Integrated workflow script

## Quick Start

### Option 1: Simple Visualization (No Schnell Required)

```bash
# Visualize with colored tiles
python map_visualizer.py rural

# Try different scenarios
python map_visualizer.py forest
```

### Option 2: With Schnell Background

**Step 1: Generate the prompt**
```bash
python generate_map_background.py rural --prompt-only
```

**Step 2: Create image with Schnell**
Use your Schnell implementation to generate an image from the prompt:
- Resolution: 1024x1024 (square format)
- Save as: `rural_bg.png`

**Step 3: Visualize with background**
```bash
python map_visualizer_with_background.py rural --background rural_bg.png
```

### Option 3: Integrated Workflow (Requires Schnell Integration)

After integrating Schnell into `generate_map_background.py`:

```bash
# One command to generate and visualize
python generate_map_background.py rural --output rural_bg.png --generate
```

## Schnell Prompt Examples

### Rural Scenario
```
A soft pastel painting of a bird's eye view landscape map dominated by 
rolling grassland with wildflowers, with patches of lush green forest 
with dense tree canopy, golden wheat fields with organized rows, 
Colors: mint green and lime, sage green and olive, pale yellow and cream, 
soft lighting, muted colors, gentle gradients, dreamy atmosphere, 
top-down view, square composition, suitable for game map background, 
no text, no labels, no UI elements, no characters, high quality, detailed, 4k
```

### Forest Scenario
```
A soft pastel painting of a bird's eye view landscape map dominated by 
lush green forest with dense tree canopy, with patches of open meadow 
clearings with soft grass, calm blue water with gentle ripples, 
Colors: sage green and olive, light sage and beige, soft powder blue, 
soft lighting, muted colors, gentle gradients, dreamy atmosphere, 
top-down view, square composition, suitable for game map background, 
no text, no labels, no UI elements, no characters, high quality, detailed, 4k
```

## Features

### Current Implementation

- ✅ Colored terrain tiles with kid-friendly palette
- ✅ Emoji characters (😊, 🙂, etc.) for agents
- ✅ Speech bubbles with comic-style pointers
- ✅ Support for Schnell-generated backgrounds
- ✅ Optional semi-transparent grid overlay (toggle with 'G' key)
- ✅ Automatic terrain analysis for prompt generation
- ✅ Multiple art styles (pastel, watercolor, cartoon)

### Interactive Controls

- **ESC** - Quit
- **G** - Toggle grid overlay (when using background image)

## Integrating Schnell

To fully automate the workflow, modify the `call_schnell()` function in `generate_map_background.py`.

### Example: Python API
```python
def call_schnell(prompt, output_path, width=1024, height=1024):
    import schnell
    image = schnell.generate(prompt, width=width, height=height)
    image.save(output_path)
    return True
```

### Example: CLI Tool
```python
def call_schnell(prompt, output_path, width=1024, height=1024):
    import subprocess
    result = subprocess.run([
        "schnell", 
        "--prompt", prompt,
        "--output", output_path,
        "--width", str(width),
        "--height", str(height)
    ])
    return result.returncode == 0
```

### Example: HTTP API
```python
def call_schnell(prompt, output_path, width=1024, height=1024):
    import requests
    response = requests.post(
        "http://localhost:8000/generate",
        json={"prompt": prompt, "width": width, "height": height}
    )
    if response.ok:
        with open(output_path, 'wb') as f:
            f.write(response.content)
        return True
    return False
```

## Architecture

```
WorldMap (terrain data)
    ↓
Terrain Analysis
    ↓
Schnell Prompt Generator
    ↓
Schnell (text-to-image)
    ↓
Background Image (1024x1024 PNG)
    ↓
Pygame Visualizer
    ├─ Background layer (image)
    ├─ Grid overlay (optional, semi-transparent)
    ├─ Agent layer (emoji + names)
    └─ UI layer (speech bubbles)
```

## Customization

### Terrain Colors (Fallback Mode)
Edit `TERRAIN_COLORS` in `map_visualizer_with_background.py`:
```python
TERRAIN_COLORS = {
    'Water': (100, 180, 255),
    'Forest': (34, 139, 34),
    # Add your terrain types...
}
```

### Agent Emoji
Edit `available_emoji` in the visualizer:
```python
self.available_emoji = ['😊', '🙂', '😄', '🧑', '👤', '🙋']
```

### Art Styles
Choose when generating prompts:
```bash
python generate_map_background.py rural --style watercolor
python generate_map_background.py forest --style cartoon
```

## Troubleshooting

**Problem:** Background image looks stretched or distorted
- Ensure Schnell generates square images (1024x1024)
- The visualizer auto-scales to fit the map area

**Problem:** Grid is too prominent over background
- Press 'G' to toggle grid off
- Adjust `GRID_COLOR` alpha value in the code

**Problem:** Agents hard to see on background
- White circle backgrounds are drawn behind emoji
- Adjust `circle_radius` in `draw_agents()` for larger halos

**Problem:** Schnell prompt doesn't match terrain well
- The analyzer looks at terrain percentages and quadrants
- For better results, ensure your scenario has varied terrain
- Try the `--simple` flag for more concise prompts

## Next Steps

This is the initial implementation. Potential enhancements:

- **Thought bubbles** - Cloud-shaped bubbles for internal thoughts
- **Action indicators** - Icons showing what agents are doing
- **Resource overlays** - Small icons or tooltips for resources
- **Animation** - Smooth movement between turns
- **Click interactions** - Click agents to see details
- **Turn tracking** - Display current turn number
- **Multiple bubbles** - Show speech/thoughts for all agents
- **Resource highlighting** - Glow or pulse effects on resources

## License

Part of the larger scenario simulation project.
