# ComfyUI Workflow Guide for WorldMap Backgrounds

## Overview

This guide explains how to use the generated layout images with ComfyUI to create beautiful, spatially-accurate map backgrounds for your visualizations.

## The Process

```
WorldMap → Layout Image → ComfyUI img2img → Beautiful Background → Pygame Visualizer
```

## Step 1: Generate Layout Image

```bash
# Basic usage
python generate_layout_image.py rural -o rural_layout.png

# With elevation shading (recommended for realistic terrain)
python generate_layout_image.py forest -o forest_layout.png --elevation

# Different styles
python generate_layout_image.py rural -o rural_layout.png --style watercolor

# Higher resolution
python generate_layout_image.py forest -o forest_layout.png --size 2048
```

**Output files:**
- `rural_layout.png` - Clean layout image (for ComfyUI input)
- `rural_layout.txt` - Prompt text (for ComfyUI positive prompt)
- `rural_layout_annotated.png` - Reference image showing terrain legend

## Step 2: ComfyUI Basic Workflow (Recommended First Try)

### Option A: Simple img2img

**Basic workflow nodes:**
```
Load Image (layout.png)
    ↓
CLIP Text Encode (positive prompt from .txt file)
    ↓
CLIP Text Encode (negative prompt: "blurry, distorted, text, labels")
    ↓
Load Checkpoint (Schnell model)
    ↓
KSampler
  - denoise: 0.65-0.75 (start with 0.7)
  - steps: 20-30
  - cfg: 7-9
    ↓
VAE Decode
    ↓
Save Image
```

**Key settings:**
- **Denoise strength: 0.7** - Good balance between layout preservation and artistic style
  - Lower (0.5-0.6): Keeps layout very accurate, less artistic
  - Higher (0.8-0.9): More artistic, may alter layout
- **CFG Scale: 7-8** - Controls prompt adherence
- **Steps: 25** - Usually sufficient for Schnell

### Option B: Basic img2img with Upscaling

If you want higher resolution output:

```
[Same as above] → Upscale Image (2x) → VAE Encode → KSampler (denoise: 0.3) → Save
```

## Step 3: ComfyUI Advanced Workflow (Better Layout Preservation)

### Option C: Using ControlNet

ControlNet helps preserve the exact layout while allowing artistic transformation.

**Workflow additions:**
```
Load Image (layout.png)
    ↓
[Split into two paths]

Path 1 (ControlNet):
    Canny Edge Detection OR Color ControlNet
    ↓
    Apply ControlNet (strength: 0.7-0.9)
    ↓
    [feeds into KSampler]

Path 2 (Standard):
    [Normal img2img path as above]
```

**Recommended ControlNet models:**
- **Canny** - Preserves boundaries/edges between terrain types
- **Tile** - Good for texture preservation while allowing style change
- **Color** - Preserves color layout most faithfully

**ControlNet settings:**
- Strength: 0.8 (strong layout preservation)
- Start/End: 0.0 / 1.0 (full guidance throughout)

## Step 4: Load Result in Visualizer

```bash
# After generating final_background.png in ComfyUI
python map_visualizer_with_background.py rural --background final_background.png
```

## Recommended ComfyUI Settings by Style

### Pastel Style
```
Denoise: 0.7
CFG: 7
Positive: "soft pastel painting, bird's eye view, dreamy watercolor..."
Negative: "harsh colors, photorealistic, detailed textures"
```

### Watercolor Style
```
Denoise: 0.75
CFG: 8
Positive: "watercolor painting, flowing colors, wet-on-wet..."
Negative: "sharp edges, digital art, photograph"
```

### Cartoon Style
```
Denoise: 0.65
CFG: 8.5
Positive: "colorful cartoon landscape, bright cheerful colors..."
Negative: "realistic, photographic, muted colors"
```

### Realistic/Satellite Style
```
Denoise: 0.6
CFG: 7
Positive: "aerial photograph, natural colors, photorealistic..."
Negative: "painting, artistic, stylized, cartoon"
```

## Troubleshooting

### Problem: Layout gets distorted/lost
**Solutions:**
- Lower denoise strength (try 0.5-0.6)
- Use ControlNet with Canny or Tile
- Increase ControlNet strength to 0.9
- Use higher resolution layout image (--size 2048)

### Problem: Output too similar to input (not artistic enough)
**Solutions:**
- Increase denoise strength (try 0.8)
- Adjust prompt to be more descriptive of desired style
- Lower ControlNet strength if using it

### Problem: Colors don't match terrain types
**Solutions:**
- Use Color ControlNet instead of Canny
- Check the annotated layout image to verify color mapping
- Adjust base colors in `LAYOUT_COLORS` in generate_layout_image.py

### Problem: Terrain boundaries too sharp
**Solutions:**
- The layout image already has Gaussian blur applied
- In ComfyUI, you can add additional blur to the input image
- Or increase denoise slightly to soften transitions

## Advanced Tips

### Batch Processing Multiple Maps
```bash
# Generate layouts for multiple scenarios
for scenario in rural forest; do
    python generate_layout_image.py $scenario -o ${scenario}_layout.png --elevation
done
```

### Creating Variations
Keep the same layout image but try different prompts/settings in ComfyUI to get variations of the same map.

### Seasonal Variations
Modify the prompt for different seasons:
- Spring: "fresh green growth, blooming flowers, soft pastels"
- Summer: "lush vegetation, bright colors, golden hour lighting"
- Fall: "autumn colors, orange and red foliage, warm tones"
- Winter: "snow-covered, white and blue tones, frosty atmosphere"

## Example Full Workflow

```bash
# 1. Generate layout
python generate_layout_image.py forest -o forest_layout.png --elevation --style pastel

# 2. Open ComfyUI, load img2img workflow
# 3. Load forest_layout.png as input image
# 4. Copy prompt from forest_layout.txt
# 5. Set denoise to 0.7, CFG to 7, steps to 25
# 6. Generate → save as forest_background.png

# 7. Test in visualizer
python map_visualizer_with_background.py forest --background forest_background.png

# 8. If layout preserved well, use in final visualization
# 9. If not, adjust denoise and regenerate in ComfyUI
```

## Sample ComfyUI Workflows (JSON)

I can provide sample ComfyUI workflow JSON files for:
- Basic img2img
- img2img with ControlNet
- img2img with upscaling

Let me know if you'd like these exported!

## Next Steps

1. Generate your first layout image
2. Try the basic img2img workflow in ComfyUI
3. If layout is preserved well → great, you're done!
4. If not → try ControlNet workflow
5. Once you have a good workflow, save it as a template in ComfyUI for reuse

The key is finding the right denoise strength for your needs - start at 0.7 and adjust from there!
