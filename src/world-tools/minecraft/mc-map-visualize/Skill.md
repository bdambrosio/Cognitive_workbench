---
name: mc-map-visualize
type: python
description: "Generates an HTML file with 2D visualization of the spatial map and opens it in browser. Developer tool for viewing explored locations, waypoints, and visit patterns"
---

# Minecraft Map Visualize Tool

Generates an HTML file with 2D visualization of the spatial map and opens it in browser. Developer tool for viewing explored locations, waypoints, and visit patterns.

## Purpose

Map visualization for debugging and analysis. Creates interactive HTML visualization showing explored locations, waypoints, and visit patterns from persistent spatial memory.

## Input

- `target`: Map Collection name or variable (default: `<agent_name>-minecraft_map`)
- `map_name`: Map Collection name (alternative to target)
- `output_file`: Optional output file path (default: auto-generated)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (success message with file path and location count)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `file_path`: String
  - `locations_count`: Integer
  - `waypoints_count`: Integer

## Behavior & Performance

- Generates HTML file with 2D map visualization
- Opens file in default browser
- Shows explored locations, waypoints, and visit patterns
- File path can be customized or auto-generated

## Guidelines

- Developer tool for debugging and analysis
- Use `load` to load map Collection first if needed
- Visualization shows spatial exploration patterns
- Waypoints are highlighted in visualization
- Visit counts shown as intensity

## Usage Examples

Visualize map:
```json
{"type":"load","target":"minecraft_map","out":"$map"}
{"type":"mc-map-visualize","target":"$map","out":"$result"}
```

Visualize with Collection name:
```json
{"type":"mc-map-visualize","target":"minecraft_map","out":"$result"}
```

Custom output file:
```json
{"type":"mc-map-visualize","target":"minecraft_map","output_file":"my_map.html","out":"$result"}
```
