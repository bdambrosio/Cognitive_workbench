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
- `x`, `y`, `z`: Coordinates (required)
- `map_name`: Optional map Collection name (default: `<agent_name>-minecraft_map`)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (success message with waypoint name and location)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `name`: String (waypoint name)
  - `location`: `{x: int, y: int, z: int}`
  - `map_name`: String

## Behavior & Performance

- Stores waypoint in persistent Collection
- Waypoints can be queried with `mc-map-query`
- Map name defaults to agent-specific name
- Learning requires labeled data - waypoints provide labels

## Guidelines

- Use meaningful waypoint names (e.g., "Base_Camp", "Pit_Exit_1")
- Waypoints enable spatial reasoning and navigation
- Query waypoints with `mc-map-query` using `query="waypoint"`
- Use `mc-status` to get current coordinates before creating waypoint

## Usage Examples

Create waypoint:
```json
{"type":"mc-waypoint","name":"Base_Camp","x":-112,"y":71,"z":-123,"out":"$result"}
```

Create multiple waypoints:
```json
{"type":"mc-waypoint","name":"Pit_Exit_1","x":-110,"y":65,"z":-120,"out":"$result"}
```
