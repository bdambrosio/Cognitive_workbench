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
- Reference block position: `dx`, `dy`, `dz` (agent-relative offsets to **anchor block**, floats, all required)
  - `dx`: Right (+) / Left (-), `dy`: Up (+) / Down (-), `dz`: Forward (+) / Back (-)
  - These specify the **anchor block** (existing solid block), NOT the placement destination
- `face`: String - face of anchor block to place against - REQUIRED
  - **Agent-relative (recommended)**: `"forward"`, `"back"`, `"left"`, `"right"`, `"up"`, `"down"`
  - **Absolute (legacy)**: `"top"`, `"bottom"`, `"north"`, `"south"`, `"east"`, `"west"`
  - Agent-relative faces are automatically converted based on current yaw

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
- All positions use agent-relative coordinates: `dx` (right/left), `dy` (up/down), `dz` (forward/back)
- Spatial map is automatically updated after successful placement (no need to call mc-map-update separately)

**Failure Interpretation:**
- If placement fails with "invalid placement": anchor block missing/invalid OR destination occupied
- If placement fails with "placement_overlaps_agent": target position is inside agent's hitbox (feet or body block). To pillar up, jump first or place forward then walk onto it.
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
- Destination must NOT overlap agent's hitbox (feet block or body block above)
- Block is placed adjacent to anchor block face

**Face Semantics (Agent-Relative - Recommended):**
- `face="up"`: Block placed **above** anchor
- `face="down"`: Block placed **below** anchor
- `face="forward"`: Block placed in agent's forward direction from anchor
- `face="back"`: Block placed behind anchor (opposite of forward)
- `face="right"`: Block placed to agent's right from anchor
- `face="left"`: Block placed to agent's left from anchor

**Face Semantics (Absolute - Legacy):**
- `face="top"`: Same as `"up"` → destination at `(anchor.x, anchor.y + 1, anchor.z)`
- `face="bottom"`: Same as `"down"` → destination at `(anchor.x, anchor.y - 1, anchor.z)`
- `face="north"`: Block placed north of anchor → destination at `(anchor.x, anchor.y, anchor.z - 1)`
- `face="south"`: Block placed south of anchor → destination at `(anchor.x, anchor.y, anchor.z + 1)`
- `face="east"`: Block placed east of anchor → destination at `(anchor.x + 1, anchor.y, anchor.z)`
- `face="west"`: Block placed west of anchor → destination at `(anchor.x - 1, anchor.y, anchor.z)`

**Note:** Agent-relative faces are converted to absolute faces using current yaw:
- Yaw 0° (South): forward=south, right=west, back=north, left=east
- Yaw 90° (West): forward=west, right=north, back=east, left=south
- Yaw 180° (North): forward=north, right=east, back=south, left=west
- Yaw 270° (East): forward=east, right=south, back=west, left=north

**Common Patterns:**

**Vertical ascent:**
- For pure vertical ascent (pillar up), use **`mc-pillar-up`** tool instead of mc-place
- `mc-pillar-up` handles jump+place timing atomically

**Bridge forward (horizontal with mc-place):**
1. Place block forward at feet level: anchor `dx=0, dy=-1, dz=1`, face `"top"` → block at `(0, 0, 1)`
2. Walk onto the placed block (nav-move forward)
3. Repeat to continue bridging

**Step up (ONLY while jumping/in-air):**
- ⚠️ **WARNING**: This pattern fails with `placement_overlaps_agent` if agent is standing!
- Use **`mc-pillar-up`** instead for vertical ascent
- Anchor: `dx=0, dy=-1, dz=0` (block below agent)
- Face: `"top"`
- Result: Block at agent_y (only works if agent's feet are above that level, e.g., mid-jump)

**Bridge forward:**
- Anchor: `dx=0, dy=-1, dz=1` (block forward and below)
- Face: `"top"`
- Result: Block at forward position, agent_y level
- ✅ Safe for standing agents (destination is forward, not at agent position)

**Prohibited:**
- ❌ Cannot place "at a coordinate" (must specify anchor + face)
- ❌ Cannot place relative to air (anchor must be solid)
- ❌ Cannot place without valid face parameter
- ❌ Cannot place where agent is standing (causes `placement_overlaps_agent`)

## Usage Examples

**Pillar up (safe for standing agent):**
```json
{"type":"mc-place","value":"cobblestone","dx":0,"dy":-1,"dz":1,"face":"up","out":"$pillar_step"}
```
Anchor: block forward and below. Face: up. Result: block at `(0, 0, 1)` - forward at feet level.
Then walk onto it with nav-move, repeat to ascend. ✅ Safe approach for gaining height.

**Bridge forward:**
```json
{"type":"mc-place","value":"cobblestone","dx":0,"dy":-1,"dz":1,"face":"up","out":"$bridge"}
```
Anchor: block forward and below. Face: up. Result: block at forward position, agent_y level.
✅ Safe for standing agents.

**Step up (⚠️ ONLY while jumping):**
```json
{"type":"mc-place","value":"dirt","dx":0,"dy":-1,"dz":0,"face":"up","out":"$step"}
```
Anchor: block below agent. Face: up. Result: block at agent_y.
⚠️ **Fails with `placement_overlaps_agent` if agent is standing!** Only use mid-jump.

**Place adjacent (lateral) - using agent-relative face:**
```json
{"type":"mc-place","value":"stone","dx":1,"dy":0,"dz":0,"face":"left","out":"$wall"}
```
Anchor: block to the right of agent. Face: left. Result: block placed between agent and anchor.
✅ Works regardless of yaw - "left" is always toward the agent from the anchor at dx=+1.
