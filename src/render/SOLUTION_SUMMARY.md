# WorldMap Visualization - Complete System

## The Solution: Layout Image + ComfyUI img2img

You were absolutely right - the text-only prompts weren't encoding spatial information. The solution is to **generate a layout image** that shows the exact terrain positions, then use **ComfyUI's img2img** to transform it into a beautiful pastel painting while preserving the layout.

## The New Workflow

```
WorldMap → Layout Image Generator → ComfyUI img2img → Beautiful Background → Pygame Visualizer
```

### Why This Works

1. **Layout Image** - We generate a smooth, colored image where each region shows terrain type
2. **img2img** - ComfyUI transforms this into art while preserving the spatial structure
3. **Control** - You adjust "denoise strength" to balance layout preservation vs artistic freedom

## Quick Start Guide

### Step 1: Generate Layout Image

```bash
# Create a layout image from your map
python generate_layout_image.py rural -o rural_layout.png

# With elevation shading (recommended)
python generate_layout_image.py forest -o forest_layout.png --elevation
```

**Outputs:**
- `rural_layout.png` - Clean layout for ComfyUI input
- `rural_layout.txt` - Prompt for ComfyUI positive prompt
- `rural_layout_annotated.png` - Reference with legend showing terrain colors

### Step 2: Use in ComfyUI

1. Open ComfyUI with img2img workflow
2. Load `rural_layout.png` as input image
3. Copy prompt from `rural_layout.txt`
4. Set **denoise to 0.7** (start here, adjust as needed)
5. Generate!

**Key Settings:**
- **Denoise 0.5-0.6**: Very accurate layout, less artistic
- **Denoise 0.7**: Good balance (recommended starting point)
- **Denoise 0.8-0.9**: More artistic, may alter layout

### Step 3: Use in Visualizer

```bash
python map_visualizer_with_background.py rural --background final_background.png
```

## Files Reference

### Core Visualization
- **map_visualizer.py** - Basic colored tile visualization
- **map_visualizer_with_background.py** - With background image support
- **test_visualization.py** - Test suite

### Layout Generation (NEW!)
- **generate_layout_image.py** - Creates layout images for ComfyUI
- **COMFYUI_GUIDE.md** - Detailed ComfyUI workflow instructions

### Legacy (Text-only Prompts)
- **map_prompt_generator.py** - Text-only prompts (less accurate)
- **generate_map_background.py** - Integrated workflow (needs Schnell API)

### Documentation
- **README_VISUALIZATION.md** - Full documentation
- **QUICKREF.md** - Quick reference
- **COMFYUI_GUIDE.md** - ComfyUI-specific guide

## Example: Creating a Forest Map Background

```bash
# 1. Generate layout (30x30 map, 1024x1024 image)
python generate_layout_image.py forest -o forest_layout.png --elevation

# Output:
# ✓ Saved layout image: forest_layout.png
# ✓ Saved prompt: forest_layout.txt
# ✓ Saved annotated version: forest_layout_annotated.png

# 2. Open ComfyUI
#    - Load forest_layout.png in "Load Image" node
#    - Paste contents of forest_layout.txt in positive prompt
#    - Denoise: 0.7, Steps: 25, CFG: 7
#    - Generate and save as forest_bg.png

# 3. Visualize
python map_visualizer_with_background.py forest --background forest_bg.png
```

## How the Layout Image Works

The layout generator:
1. Analyzes your WorldMap terrain distribution
2. Renders each patch with its terrain color
3. Applies Gaussian blur for smooth transitions (mimics natural terrain blending)
4. Optionally adds elevation-based shading (lighter = higher, darker = lower)
5. Outputs at high resolution (1024x1024 or higher)

**The result:** A clean, spatially-accurate representation that ComfyUI can transform while preserving the layout.

## Advanced: Using ControlNet

For even better layout preservation:

1. In ComfyUI, add ControlNet nodes
2. Use **Canny** or **Tile** ControlNet model
3. Feed the layout image through ControlNet
4. Set ControlNet strength to 0.8-0.9

This preserves terrain boundaries perfectly while allowing full artistic transformation.

## Troubleshooting

**Layout gets lost in ComfyUI:**
- Lower denoise strength (try 0.6)
- Use ControlNet
- Generate layout at higher resolution (--size 2048)

**Not artistic enough:**
- Increase denoise strength (try 0.8)
- Make prompt more descriptive
- Try different art styles (--style watercolor)

**Colors don't match:**
- Check `forest_layout_annotated.png` to see color mapping
- Adjust colors in `LAYOUT_COLORS` if needed

## Testing

```bash
# Test layout generation
python test_visualization.py layout

# Test everything
python test_visualization.py
```

## What You Get

✅ **Spatially accurate** - Terrain layout matches your WorldMap exactly
✅ **Beautiful** - Schnell transforms it into pastel/watercolor art
✅ **Controllable** - Adjust denoise to balance accuracy vs artistry
✅ **Fast** - Layout generation is instant, ComfyUI takes ~10-30 seconds
✅ **Reproducible** - Same layout can be rendered in different styles

## Summary

The key insight: **Don't describe the layout in text - show it in an image.** The layout generator creates a visual representation of your map's terrain, which ComfyUI can then artistically transform while preserving the spatial structure.

This approach gives you the best of both worlds: precise spatial accuracy from the layout image, and beautiful artistic rendering from Schnell.

Ready to create your first spatially-accurate map background!
