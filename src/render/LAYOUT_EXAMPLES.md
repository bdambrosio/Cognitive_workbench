# Layout Image Examples

## What Does a Layout Image Look Like?

The layout generator creates a smooth, colored image representing your map's terrain. Here's what you'll see:

## Example 1: Rural Scenario (30x30 map)

### Layout Image Output
```
┌─────────────────────────────────────┐
│ 🟦🟦🟦                               │ Blue = Water
│ 🟦🟦🟦🟦   🟫🟫🟫                    │ Gray = Mountain  
│   🟦🟦    🟫🟫🟫🟫                   │ Dark Green = Forest
│          🟫🟫                        │ Light Green = Grassland
│    🟩🟩🟩🟩🟩🟩🟩🟩                  │ Yellow = Field
│  🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩                │
│🟩🟩🟩🟢🟢🟢🟢🟩🟩🟩🟩                 │
│🟢🟢🟢🟢🟢🟢🟢🟢🟩🟩                  │
│🟢🟢🟢🟢🟨🟨🟨🟨🟩                    │
│🟢🟢🟨🟨🟨🟨🟨🟨🟨                    │
│  🟨🟨🟨🟨🟨🟨🟨🟨🟨                  │
│🟨🟨🟨🟨🟨🟨🟨🟨🟨🟨                  │
│🟨🟨🟨🟨  🟨🟨🟨🟢🟢                  │
│🟨🟨🟨    🟢🟢🟢🟢🟢                  │
│🟨🟨     🟢🟢🟢🟢🟢🟢                 │
│       🟢🟢🟢🟢🟩🟩🟩                 │
│      🟢🟢🟢🟩🟩🟩🟩🟩                │
│     🟢🟢🟩🟩🟩🟩                     │
│    🟢🟩🟩🟩🟩  🟫🟫🟫                │
│   🟩🟩🟩     🟫🟫🟫🟫                │
└─────────────────────────────────────┘
```

**Key Features:**
- Smooth color transitions (Gaussian blur applied)
- Clear terrain regions
- No sharp pixel boundaries
- High resolution (1024x1024 pixels)

## Example 2: Forest Scenario with Elevation

### Layout Image Output (with elevation shading)
```
┌─────────────────────────────────────┐
│                    🟫🟫🟫🟫          │ Lighter = Higher elevation
│  🟦🟦🟦          🟫🟫🟫🟫🟫🟫        │ Darker = Lower elevation
│🟦🟦🟦🟦🟦          🟫🟫🟫🟫🟫        │
│🟦🟦🟦🟦                             │ Notice mountains are lighter
│  🟦🟦    🟢🟢🟢🟢                   │ Forest in valleys is darker
│        🟢🟢🟢🟢🟢🟢🟢                │
│      🟢🟢🟢🟢🟢🟢🟢🟢🟢               │
│    🟢🟢🟢🌲🌲🌲🌲🟢🟢🟢               │
│  🟢🟢🌲🌲🌲🌲🌲🌲🟢🟢                │
│  🌲🌲🌲🌲🌲🌲🌲🌲🟢                  │
│🌲🌲🌲🌲🌲🌲🌲🌲🌲🟢                  │
│🌲🌲🌲🌲🌲🌲🌲🌲🌲🌲                  │
│🌲🌲🌲🌲🌲🌲🌲🟢🟢🟢                  │
│🌲🌲🌲🌲🌲🟢🟢🟢🟢                    │
│🌲🌲🌲🌲🟢🟢🟢🟩🟩                    │
│  🌲🌲🟢🟢🟩🟩🟩🟩                    │
│    🟢🟢🟩🟩🟩🟩                      │
│      🟩🟩🟩🟩                        │
│      🟩🟩🟩    🟦🟦                  │
│      🟩🟩    🟦🟦🟦🟦                │
└─────────────────────────────────────┘
```

## Color Palette

The layout generator uses these base colors:

| Terrain    | RGB Color       | Visual     |
|------------|-----------------|------------|
| Water      | (80, 150, 220)  | Light blue |
| Mountain   | (140, 140, 140) | Gray       |
| Forest     | (60, 120, 60)   | Dark green |
| Grassland  | (120, 180, 100) | Light green|
| Field      | (200, 180, 100) | Yellow-tan |
| Clearing   | (150, 170, 110) | Olive      |
| Meadow     | (140, 200, 120) | Bright green|

**With Gaussian blur applied**, these colors blend smoothly at terrain boundaries, creating natural-looking transitions like real landscapes.

## Annotated Version Example

The annotated layout includes a legend:

```
┌─────────────────────────────────────┐
│ [Map content here]                  │
│                                     │
│                                     │
│                          ┌────────┐ │
│                          │Legend  │ │
│                          │■ Water │ │
│                          │■ Forest│ │
│                          │■ Field │ │
│                          │■ Grass │ │
│                          └────────┘ │
└─────────────────────────────────────┘
```

This helps you verify the color mapping before feeding to ComfyUI.

## How ComfyUI Transforms It

**Input (Layout Image):**
- Clean colored regions
- Smooth transitions
- Spatially accurate

**ComfyUI img2img with denoise 0.7:**
- Adds artistic texture and style
- Maintains spatial layout
- Transforms into pastel painting

**Result:**
- Beautiful, painterly appearance
- Terrain still in correct locations
- Natural-looking landscape

## Technical Details

### Resolution
- Default: 1024x1024 pixels
- Can generate up to 2048x2048 or higher
- Higher resolution = better preservation in ComfyUI

### Supersampling
- Renders at 4x target resolution
- Applies Gaussian blur
- Downsamples for anti-aliasing
- Result: Smooth, professional-looking layout

### Smoothing
- Gaussian blur radius: 2 pixels (at supersample resolution)
- Creates natural terrain transitions
- Mimics how real terrain blends together

## File Sizes

Typical output:
- `layout.png`: ~500KB - 2MB (depending on size)
- `layout_annotated.png`: Similar size
- `layout.txt`: <1KB (just the prompt text)

## What Makes This Work

The key insight: **The layout image encodes spatial information visually** rather than trying to describe it in text. ComfyUI's img2img can then:

1. **See** the exact layout of terrain types
2. **Preserve** the spatial structure (controlled by denoise strength)
3. **Transform** the artistic style while keeping positions

This is far more reliable than text prompts which struggle with spatial relationships like "forest in the northwest quadrant" or "water on the left side."

## Testing Your Layout

Before sending to ComfyUI, check the annotated version:
```bash
python generate_layout_image.py rural -o test.png
# Look at test_annotated.png to verify:
# - Colors match terrain types
# - Layout looks correct
# - Smooth transitions between regions
```

If the layout looks good, ComfyUI will preserve it!
