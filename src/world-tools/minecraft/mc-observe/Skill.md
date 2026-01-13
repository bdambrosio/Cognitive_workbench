---
name: mc-observe
type: python
description: "Unified observation tool: fetches blocks, entities, and items in a single efficient call."
---

# Minecraft Unified Observe Tool

Fetches blocks, entities (including items), and navigation surface data in a single HTTP call to the bridge. More efficient than calling mc-observe-blocks, mc-observe-entities, and mc-observe-items separately.

## Purpose

Comprehensive observation for map-building and world awareness. Returns blocks, entities, items, navigation surface, and visibility data in one response.

## Input

- `radius`: Optional integer observation radius (default: `3`, max: `7`)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Multi-line SUMMARY text (human-readable, ≤ 1024 chars)
- `data`: Structured observation dict (machine-readable). Key fields:
  - `success`: Boolean
  - `pose`: `{x, y, z, yaw, pitch}` (floats)
  - `blocks`: `{count, nearby: [dict], nav_surface: [dict], adjacent: dict, visibility_distances: dict, complete: bool, elapsed_ms: int}`
  - `entities`: `{total, items, mobs, players, other, nearby: [dict], complete: bool, elapsed_ms: int, metadata_stats: {with_age, with_velocity, with_persistence}}`
  - `conf`: `"high"` | `"med"` | `"low"`

## Behavior & Performance

- Single HTTP call to `/observe` endpoint with `entity_filter=""` (all entities)
- Returns blocks, entities, items, nav_surface, adjacent_blocks, visibility_distances
- Entity metadata includes age_ticks, velocity, time_until_despawn (if available from Minescript API)
- Logs what data is successfully returned and what is missing (for debugging)
- Radius limits: Default 3 blocks (for testing), maximum 7 blocks

## Guidelines

- Use this tool for map-building and comprehensive world observation
- More efficient than calling mc-observe-blocks + mc-observe-entities + mc-observe-items separately
- Entity metadata (age, velocity) may not be available depending on Minescript API version
- Check logs for metadata availability statistics
