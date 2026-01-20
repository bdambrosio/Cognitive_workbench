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
- Reference block position: `dx`, `dy`, `dz` (world-relative offsets to **anchor block**, floats, all required)
  - These specify the **anchor block** (existing solid block), NOT the placement destination
  - See coordinate system documentation in jill-minecraft.yaml for details
- `face`: String - face of anchor block to place against (absolute: "top", "bottom", "north", "south", "east", "west") - REQUIRED

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
- `dx, dy, dz` specify anchor block position (existing solid block), NOT destination
- Face parameter determines which side of anchor block to place against
- All positions use world-relative coordinates `dx, dy, dz` (see coordinate system in jill-minecraft.yaml)
- Spatial map is automatically updated after successful placement (no need to call mc-map-update separately)

**Failure Interpretation:**
- If placement fails with "invalid placement": anchor block missing/invalid OR destination occupied
- Do not retry with same `(dx, dy, dz, face)` pair
- After two failures: re-observe anchor blocks, recompute anchor position

## Placement Invariants

**Reference Frame:**
- `dx, dy, dz` specify the **anchor block** (existing solid block), NOT the placement destination
- All coordinates are relative to agent at execution time
- Placement is always adjacent to an existing solid block face

**Valid Placement Conditions:**
- Anchor block must exist and be solid (air/void/water cannot be anchors)
- Destination cell must be empty (placement fails if occupied)
- Block is placed adjacent to anchor block face

**Face Semantics:**
- `face="top"`: Block placed **above** anchor → destination at `(anchor.x, anchor.y + 1, anchor.z)`
- `face="bottom"`: Block placed **below** anchor → destination at `(anchor.x, anchor.y - 1, anchor.z)`
- `face="north"`: Block placed north of anchor → destination at `(anchor.x, anchor.y, anchor.z - 1)`
- `face="south"`: Block placed south of anchor → destination at `(anchor.x, anchor.y, anchor.z + 1)`
- `face="east"`: Block placed east of anchor → destination at `(anchor.x + 1, anchor.y, anchor.z)`
- `face="west"`: Block placed west of anchor → destination at `(anchor.x - 1, anchor.y, anchor.z)`

**Common Patterns:**

**Step up (place block at agent_y):**
- Anchor: `dx=0, dy=-1, dz=0` (block below agent)
- Face: `"top"`
- Result: Block at agent_y (can stand on)

**Bridge forward:**
- Anchor: `dx=1, dy=-1, dz=0` (block forward and below)
- Face: `"top"`
- Result: Block at forward position, agent_y

**Prohibited:**
- ❌ Cannot place "at a coordinate" (must specify anchor + face)
- ❌ Cannot place relative to air (anchor must be solid)
- ❌ Cannot place without valid face parameter

## Usage Examples

**Step up (place block at agent_y to stand on):**
```json
{"type":"mc-place","value":"dirt","dx":0,"dy":-1,"dz":0,"face":"top","out":"$step"}
```
Anchor: block below agent. Face: top. Result: block at agent_y.

**Bridge forward:**
```json
{"type":"mc-place","value":"cobblestone","dx":1,"dy":-1,"dz":0,"face":"top","out":"$bridge"}
```
Anchor: block forward and below. Face: top. Result: block at forward position, agent_y.

**Place adjacent (lateral):**
```json
{"type":"mc-place","value":"stone","dx":1,"dy":0,"dz":0,"face":"west","out":"$wall"}
```
Anchor: block east of agent. Face: west. Result: block at agent position (east face of anchor).
