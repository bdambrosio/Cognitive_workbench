---
name: mc-observe-items
type: python
description: "Exhaustive enumeration of ALL item entities within radius R. Only reports item entities (dropped items), not blocks or other entities."
schema_hint:
  value: "ignored"
  radius: "optional observation radius for items (default: 5, max: 12)"
  entities_radius: "optional entities observation radius (default: 5, max: 12)"
  out: "$variable"
examples:
  - '{"type":"mc-observe-items","out":"$items"}'
  - '{"type":"mc-observe-items","radius":5,"out":"$items"}'
---

# Minecraft Observe-Items Tool

## Input
- `value`: ignored
- `radius`: optional - observation radius for items (default: 5, max: 12)
- `entities_radius`: optional - entities observation radius (default: 5, max: 12)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: structured observation summary following the schema below
  - `metadata`: raw observation data from Minecraft bot API

### Structured Summary Format

The `text` field contains a structured summary in the following format:

```
SUMMARY:

pose: (x.xx,y.yy,z.zz, yaw:y.yy, pitch:p.pp)

items:
  total: N
  types: {item_name: count, ...}
  nearest: {item_name: distance, ...}
  by_distance:
    - item_name (count) at (x,y,z) dist: d.d
    - ...

pickup:
  in_range: [item_name, ...]
  nearby: [item_name, ...]
  far: [item_name, ...]

conf: high|med|low
note: "Human-readable summary"
```

### Field Descriptions

**pose**: Bot position (x, y, z) and orientation (yaw, pitch in radians)

**items**: Item summary information
- `total`: Total number of item entities found
- `types`: Dictionary mapping item name to total count across all instances
- `nearest`: Dictionary mapping item name to distance of nearest instance
- `by_distance`: List of items sorted by distance (closest first), showing:
  - Item name
  - Count (if > 1)
  - Position coordinates
  - Distance from bot

**pickup**: Pickup feasibility categorization
- `in_range`: Item types within 1.5 blocks (automatic pickup range)
- `nearby`: Item types within 3 blocks (easy to reach with small movement)
- `far`: Item types beyond 3 blocks (require more movement)

**conf**: Confidence level
- `high`: Complete enumeration, fast response, reasonable item count
- `med`: Complete but near timeout, or many items (complex environment)
- `low`: Incomplete enumeration (timeout occurred)

**note**: Human-readable summary combining key observations

### Metadata

The `metadata` field contains the raw API response with:
- `status`: Bot status (position, yaw, pitch, health, etc.)
- `perception`: Raw perception data
  - `nearby_entities`: Complete list of ALL item entities within radius (exhaustive enumeration)
  - `entities_complete`: Boolean indicating if enumeration completed (false if timeout)
  - `entities_elapsed_ms`: Time taken for entity enumeration
  - `visibility_distances`: Distance data for all directions (same as mc-observe-blocks)

**IMPORTANT**: The structured summary in `text` is optimized for LLM parsing and decision-making. For complete raw data including all item positions and details, access the `metadata` field or use the `load` primitive to load the full Note content.

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-dig","forward":1,"up":0,"right":0,"out":"$dig"}
{"type":"mc-observe-items","out":"$items"}
{"type":"mc-move","forward":true,"duration":0.5,"out":"$nudge"}
{"type":"mc-inventory","out":"$inv"}
```

## Cognitive Contract

### Exhaustive Enumeration Guarantee
- **ALL item entities within radius R are returned** - this is exhaustive, not sampled
- **Safe negation**: If an item type is not listed within radius R, Jill can safely infer it is not present nearby
- Items are entities, not blocks - they can be detected through walls (no line-of-sight required)

### Performance
- Default radius: 5 blocks (max: 12 blocks)
- Timeout protection: Enumeration stops if it exceeds ~10ms to prevent blocking
- If timeout occurs, `entities_complete` will be `false` and results are partial (confidence will be `low`)

### Structured Output Benefits
- **Precomputed pickup feasibility**: Items categorized by pickup range (in_range/nearby/far)
- **Item grouping**: Items grouped by type with counts for easy scanning
- **Distance sorting**: Items sorted by distance for efficient pickup planning
- **Confidence levels**: Indicates data quality for decision-making
- **Consistent format**: Matches mc-observe-blocks structure for familiarity

### Pickup Mechanics
- **Automatic pickup**: Items within 1.5 blocks are automatically collected when bot moves
- **Movement required**: Bot must move close to items for automatic collection
- **Verification**: Use `mc-inventory` after movement to confirm collection
- **No explicit pickup action**: Minecraft handles pickup automatically when in range

### Other Notes
- **Only reports dropped items** (item entities) - does NOT report blocks or other entities
- Use `mc-observe-blocks` to see blocks and non-item entities
- Items appear after digging blocks or dropping items from inventory
- The structured format makes it easy to quickly identify what items are available and their pickup feasibility
- This enables reliable item detection: "If diamond is not listed within radius 5, there are no diamonds nearby"
