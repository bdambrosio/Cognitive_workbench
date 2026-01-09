---
name: mc-waypoint
type: python
description: "Labels a coordinate with a waypoint name for reasoning about spatial relationships. Learning requires labeled data"
---

# Minecraft Waypoint Tool

Labels a coordinate with a waypoint name for reasoning about spatial relationships. Stores waypoints in persistent spatial memory.

## Purpose

Spatial memory labeling for navigation and planning. Creates named waypoints that can be queried later for navigation and spatial reasoning.

## Input

- `name`: Waypoint name string (required)
- `x`, `y`, `z`: Coordinates (optional - if not provided, uses current position from mc-status)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (success message with waypoint name and location)
- `data`: Structured data dict (machine-readable). Key fields:
  - `waypoint`: String (waypoint name)
  - `location`: `{x: int, y: int, z: int}`
  - `all_waypoints`: List of all waypoint names at this location

## Behavior & Performance

- Stores waypoint in SpatialMap cell at specified coordinates
- Creates cell if it doesn't exist
- Multiple waypoints can be stored at the same location
- Waypoints persist in SpatialMap and can be queried via cell data
- Auto-saves SpatialMap after update
- If coordinates not provided, automatically queries mc-status for current position

## Guidelines

- Use meaningful waypoint names (e.g., "Base_Camp", "Pit_Exit_1")
- Waypoints enable spatial reasoning and navigation
- Waypoints are stored in SpatialMap cells and persist across sessions
- Use `mc-status` to get current coordinates, or omit coordinates to use current position

## Usage Examples

Create waypoint:
```json
{"type":"mc-waypoint","name":"Base_Camp","x":-112,"y":71,"z":-123,"out":"$result"}
```

Create multiple waypoints:
```json
{"type":"mc-waypoint","name":"Pit_Exit_1","x":-110,"y":65,"z":-120,"out":"$result"}
```
