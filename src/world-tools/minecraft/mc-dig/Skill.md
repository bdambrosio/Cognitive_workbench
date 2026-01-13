---
name: mc-dig
type: python
description: "Removes a block from the Minecraft world at a specified location. ASYNCHRONOUS. Returns request acceptance status. May cause an item entity to spawn, but item spawning, item type, and item location are not guaranteed and are not directly reported by this tool"
---

# Minecraft Dig Tool

Removes a block from the Minecraft world at a specified location. Asynchronous operation - returns request acceptance status immediately.

## Purpose

Block removal for mining, excavation, and terrain modification. Asynchronous operation means the tool returns immediately after accepting the request, but actual digging may complete later. May cause item entities to spawn, but spawning details are not guaranteed or directly reported.

## Input

- Egocentric position (preferred): `forward`, `right`, `up` (relative to agent facing, floats)
- Absolute position: `x`, `y`, `z` (legacy/global, floats, all required if using absolute)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (e.g., "Dig request accepted: stone at (x, y, z)")
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `status`: `"accepted"` (request accepted, but digging may still fail asynchronously)
  - `dug`: `{name: string, position: {x, y, z}}`

## Behavior & Performance

- Asynchronous: Returns immediately after accepting request
- Actual digging may complete or fail later
- Item spawning: May cause item entities to spawn, but details not guaranteed
- Use `mc-observe` after digging to check for spawned item entities

## Guidelines

- Status `"accepted"` means request was accepted, not that digging succeeded
- Check for spawned item entities using `mc-observe` after digging completes
- Use egocentric coordinates (forward, right, up) for relative positioning
- Use absolute coordinates (x, y, z) for precise global positioning

## Usage Examples

Dig forward one block:
```json
{"type":"mc-dig","forward":1,"up":0,"right":0,"out":"$dig"}
```

Dig down one block:
```json
{"type":"mc-dig","forward":0,"up":-1,"right":0,"out":"$dig"}
```

Dig at absolute coordinates:
```json
{"type":"mc-dig","x":100,"y":64,"z":200,"out":"$dig"}
```
