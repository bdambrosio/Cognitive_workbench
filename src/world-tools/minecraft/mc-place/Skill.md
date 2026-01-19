---
name: mc-place
type: python
description: "Build / modify world - place blocks. ASYNCHRONOUS. Returns request acceptance status"
---

# Minecraft Place Tool

Places blocks in the Minecraft world at a specified location. Asynchronous operation - returns request acceptance status immediately.

## Purpose

Block placement for building, construction, and terrain modification. Asynchronous operation means the tool returns immediately after accepting the request, but actual placement may complete later.

## Input

- `value`: Item/block name to place (preferred)
- `item`: Item/block name to place (alternative to value)
- Reference position: `dx`, `dy`, `dz` (world-relative offsets from agent, floats, all required)
  - See coordinate system documentation in jill-minecraft.yaml for details
- `face`: String - face of reference block to place against (absolute: "top", "bottom", "north", "south", "east", "west") - REQUIRED

## Output

Returns uniform_return format with:
- `value`: Text summary (e.g., "Place request accepted: dirt at (x, y, z) (face=north)")
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `status`: `"accepted"` (request accepted, but placement may still fail asynchronously)
  - `placed`: `{name: string, position: {x, y, z}, face: string}`

## Behavior & Performance

- Asynchronous: Returns immediately after accepting request
- Actual placement may complete or fail later
- Face parameter determines which side of reference block to place against
- **Automatically updates persistent spatial map** after successful placement (calls mc-map-update internally, non-fatal if fails)
- Note: Due to async nature, observation may capture intermediate state; next observation will correct inconsistencies

## Guidelines

- Status `"accepted"` means request was accepted, not that placement succeeded
- Face parameter is required and determines placement orientation (absolute cardinal directions)
- All positions use world-relative coordinates `dx, dy, dz` (see coordinate system in jill-minecraft.yaml)
- Spatial map is automatically updated after successful placement (no need to call mc-map-update separately)

## Important: Face Semantics

**The `face` parameter specifies which side of the REFERENCE block to place against, NOT the final placement location.**

- `face="top"` or `face="up"`: Places block **one block above** the reference block (at `ref_y + 1`)
- `face="bottom"` or `face="down"`: Places block **one block below** the reference block (at `ref_y - 1`)
- `face="north"`/`"south"`/`"east"`/`"west"`: Places block adjacent in that direction

**Example**: To place a block at y=55 that you can stand on:
- Specify reference block at y=54 with `face="top"` → places at y=55 ✓
- NOT reference block at y=55 with `face="top"` → places at y=56 ✗

This matches standard Minecraft block placement semantics.

## Usage Examples

Place block directly south with face:
```json
{"type":"mc-place","value":"dirt","dx":0,"dy":0,"dz":1,"face":"north","out":"$place"}
```

Place block above with face:
```json
{"type":"mc-place","item":"cobblestone","dx":0,"dy":1,"dz":0,"face":"top","out":"$build"}
```
