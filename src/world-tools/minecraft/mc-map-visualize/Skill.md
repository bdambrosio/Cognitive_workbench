---
name: mc-map-visualize
type: python
description: "Generates an HTML file with 2D visualization of the spatial map and opens it in browser. Developer tool for viewing explored locations, waypoints, and visit patterns."
schema_hint:
  value: "ignored"
  target: "Optional Map Collection name or variable (default: '<agent_name>-minecraft_map', computed automatically)"
  map_name: "Optional explicit map name (alternative to target, same default)"
  output_file: "Optional output file path (default: auto-generated with timestamp)"
  out: "$variable"
examples:
  - '{"type":"load","target":"minecraft_map","out":"$map"}'
  - '{"type":"mc-map-visualize","target":"$map","out":"$result"}'
  - '{"type":"mc-map-visualize","target":"minecraft_map","out":"$result"}'
  - '{"type":"mc-map-visualize","target":"minecraft_map","output_file":"my_map.html","out":"$result"}'
---

# Minecraft Map Visualize Tool

## Input
- `target` or `map_name`: Optional Map Collection name (default: `<agent_name>-minecraft_map`, computed automatically)
  - Can be literal string: `"minecraft_map"` or `"custom_map"`
  - Can be variable: `"$map"` (if bound)
  - If omitted or unbound: defaults to `<agent_name>-minecraft_map`
- `output_file`: Optional output file path (default: auto-generated as `map_visualization_{map_name}_{timestamp}.html`)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: Success message with file path and location count
  - `metadata`: Map information including file_path, locations_count, waypoints_count

## How It Works

1. **Loads map Collection**: Retrieves the specified map Collection by name
2. **Generates HTML**: Creates standalone HTML file with:
   - 2D top-down view (X-Z plane, Y shown in details)
   - Interactive canvas (pan, zoom, click for details)
   - Location markers (color-coded by visit count)
   - Waypoint markers (gold stars with labels)
   - Sidebar with waypoints and recent locations
   - Export functionality (download as JSON)
3. **Saves file**: Writes HTML to specified or auto-generated file path
4. **Opens browser**: Automatically opens visualization in default browser

## Visualization Features

- **2D Map View**: Top-down perspective showing X-Z coordinates
- **Location Markers**: 
  - Size indicates visit count
  - Color intensity based on exploration frequency
  - Click to see details
- **Waypoint Markers**: Gold stars with labels
- **Interactive Controls**:
  - Pan (click and drag)
  - Zoom (mouse wheel)
  - Reset view
  - Fit all locations
  - Toggle waypoints
  - Export data as JSON
- **Sidebar**: Lists waypoints and recent locations with click-to-focus

## Common Workflow

**After exploration:**
```json
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-map-update","observation":"$obs","out":"$result"}
{"type":"mc-map-visualize","target":"minecraft_map","out":"$viz"}
```

**On demand:**
```json
{"type":"mc-map-visualize","target":"minecraft_map","out":"$viz"}
```

**With custom filename:**
```json
{"type":"mc-map-visualize","target":"minecraft_map","output_file":"exploration_map.html","out":"$viz"}
```

## Cognitive Contract

- **Developer tool**: Designed for developers, not end-users
- **Standalone HTML**: No server required, works offline
- **Auto-opens**: Automatically opens in browser for immediate viewing
- **Export capability**: Can export map data as JSON for external tools
- **Lightweight**: Minimal dependencies, fast generation

## Important Notes

- **Requires map Collection**: Map must exist (create with `mc-map-update` first)
- **File location**: Saves to current working directory by default
- **Browser required**: Opens in default browser (may fail in headless environments)
- **World-agnostic**: Works with any Collection containing spatial data (x, y, z fields)
- **Read-only**: Visualization is read-only, does not modify map data

## Use Cases

- **Debugging**: Visualize exploration patterns and coverage
- **Analysis**: Understand visit frequency and waypoint distribution
- **Planning**: Review explored areas before planning new routes
- **Documentation**: Generate visual maps for reports or documentation

