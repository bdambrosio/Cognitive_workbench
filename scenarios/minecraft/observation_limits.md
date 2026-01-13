# Observation Limits and Epistemic Constraints

Defines what can and cannot be inferred from observation tools.

---

## Observation Characteristics (Updated 2026-01)

### mc-observe (unified)
- **Reports**: A *cone-limited*, *work-capped* view of nearby blocks + entities, plus navigation-first surface samples.
- **Radius**:
  - Bridge `/observe` defaults to 9 if omitted, clamps to max 10
  - Tool `mc-observe` defaults smaller (typically 3) and may clamp to its own max for performance
- **Visibility criteria**:
  - Forward cone: yaw ±60°
  - Vertical: up 60° / down 90° relative to pitch=0
  - Distance ≤ radius (distance measured from player position; LOS uses eye height)
- **Navigation surface** (`nav_surface`): 2D cone scan with downward probes to reliably recover `support_y` and walkability.
- **Important**: block reporting is **not guaranteed exhaustive**; it is optimized for navigation and performance.

### Entities and items
- `mc-observe` includes entities (items are a subset of entities) when entity scanning is enabled.
- Same cone + LOS filtering as above.
- **Distance** is computed from player position (not eye height).

---

## Cone-Based Visibility Limits

### What IS Reported
- Blocks/entities within the forward cone (120° horizontal, up 60° / down 90°)
- Only if visible (not occluded by opaque blocks)
- Only within radius limit (default 7 blocks)

### What IS NOT Reported
- Blocks/entities outside the cone (behind or to the sides)
- Blocks/entities beyond radius limit
- Occluded blocks/entities (hidden behind walls/terrain)
- Blocks/entities outside vertical range (too high/low)

### Important Note
**Absence from observation ≠ non-existence**. A block may exist but:
- Be outside the observation cone
- Be occluded by other blocks
- Be beyond the radius limit

---

## Navigation Surface vs Visual Observation

### Navigation Surface (`nav_surface`)
- **Purpose**: Determine where agent can stand (support_y, walkability)
- **Method**: 2D (x,z) cell scan + downward probe
- **Ignores**: Non-supporting cover blocks (snow layers, grass) for occlusion
- **Reliable**: Provides `support_y` even when visual surface is occluded

### Visual Blocks (`nearby_blocks`)
- **Purpose**: Visual awareness of block types present
- **Method**: Cone + LOS filtering
- **Occlusion**: Snow layers and other cover blocks can occlude distant blocks
- **Use case**: Resource detection, hazard awareness

---

## Correct Usage

- Use `mc-observe` to:
  - Determine navigation surface (`nav_surface`)
  - Identify a navigation-relevant set of nearby blocks (`blocks.nearby`)
  - See entities/items (`entities.nearby`)
  - Confirm presence when observed (absence is never definitive)

- Use `mc-map-update` to:
  - Persist navigation surface data
  - Build spatial map for path planning

- Use coordinate reasoning and prior knowledge to:
  - Infer block existence outside observation cone
  - Choose dig targets beyond visible range

---

## Agent Rule (Strong)

Observation provides **high-value but cone-limited and work-capped** data.
It supplements reasoning but does not replace it.
Absence from observation is never proof of absence.
