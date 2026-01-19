---
name: mc-dig
type: python
description: "Removes a block from the Minecraft world at a specified location. SYNCHRONOUS - waits for dig completion. May cause an item entity to spawn, but item spawning, item type, and item location are not guaranteed and are not directly reported by this tool"
---

# Minecraft Dig Tool

Removes a block from the Minecraft world at a specified location. Synchronous operation - bridge waits for dig completion (or timeout).

## Purpose

Block removal for mining, excavation, and terrain modification. Synchronous operation - bridge waits for dig completion before returning. May cause item entities to spawn, but spawning details are not guaranteed or directly reported.

## Input

- Position: `dx`, `dy`, `dz` (world-relative offsets from agent, floats, all required)
  - See coordinate system documentation in jill-minecraft.yaml for details
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (e.g., "Dig request accepted: stone at (x, y, z)")
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `status`: `"accepted"` (request accepted, but digging may still fail asynchronously)
  - `dug`: `{name: string, position: {x, y, z}}`

## Behavior & Performance

- Synchronous: Bridge waits for dig completion (or timeout)
- Item spawning: May cause item entities to spawn, but details not guaranteed
- Use `mc-observe` after digging to check for spawned item entities
- **Automatically updates persistent spatial map** after successful dig (calls mc-map-update internally, non-fatal if fails)

## Guidelines

- Status `"accepted"` means request was accepted, not that digging succeeded
- Check for spawned item entities using `mc-observe` after digging completes
- All positions use world-relative coordinates `dx, dy, dz` (see coordinate system in jill-minecraft.yaml)
- Spatial map is automatically updated after successful dig (no need to call mc-map-update separately)

## Usage Examples

Dig block directly east:
```json
{"type":"mc-dig","dx":1,"dy":0,"dz":0,"out":"$dig"}
```

Dig block below:
```json
{"type":"mc-dig","dx":0,"dy":-1,"dz":0,"out":"$dig"}
```

Dig block directly south:
```json
{"type":"mc-dig","dx":0,"dy":0,"dz":1,"out":"$dig"}
```
