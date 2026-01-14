---
name: mc-map-visualize
type: python
description: "Generates zoomable HTML visualization of the spatial map showing walkability, hazards, resources, and observation confidence."
situational: false
---

# Minecraft Map Visualize Tool

Generates an interactive HTML visualization of the cell-based SpatialMap. Displays mapped cells with color-coded walkability, hazards, resources, and observation confidence.

## Purpose

Visualize spatial map for debugging, analysis, and planning review. Interactive HTML allows panning, zooming, and layer toggling. Shows cell details on hover.

## Input

- `target` or `map_name`: Map name (default: agent-specific)
- `output_file`: Optional output file path (default: auto-generated in /tmp)

## Output

Returns uniform_return format with:
- `value`: Summary text with file path
- `data`: `{success, map_name, file_path, cell_count, stats}`

Also opens generated HTML file in default browser.

## Visualization Features

Interactive canvas:
- Pan: Click and drag
- Zoom: Mouse wheel
- Hover: Shows cell details

Layer toggles:
- Walkable: Show walkability coloring
- Hazards: Highlight hazard cells (red)
- Resources: Highlight resource cells (gold)
- Confidence: Color by observation confidence

Color scheme:
- Green/surface colors: Walkable cells
- Gray: Blocked cells
- Red: Hazard cells
- Gold: Resource cells
- Blue: Water
- Orange: Lava

Cell borders:
- Green border: Step-up movement
- Yellow border: Drop risk
- Dashed border: Inferred (not directly observed)

Sidebar:
- Map statistics
- Bounds information
- Movement class distribution
- Surface type distribution

## Behavior & Performance

- Generates self-contained HTML file
- Auto-opens in default browser
- Exports JSON data for external analysis
- Works with maps of any size (canvas scales)

## Guidelines

- Use after mc-map-update to visualize mapped area
- Toggle layers to focus on specific aspects
- Hover over cells for detailed properties
- Export JSON for programmatic analysis

## Usage Examples

Visualize current map:
```json
{"type":"mc-map-visualize"}
```

With specific output file:
```json
{"type":"mc-map-visualize","output_file":"/tmp/my_map.html"}
```
