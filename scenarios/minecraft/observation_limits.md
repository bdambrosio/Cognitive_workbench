# Observation Limits and Epistemic Constraints

Defines what can and cannot be inferred from observation tools.

---

## Observation Characteristics (Updated 2026-01)

### mc-observe-blocks
- **Reports**: Exhaustive enumeration of visible non-air blocks within radius
- **Visibility criteria**:
  - Within radius R (default: 7 blocks)
  - Within forward cone (yaw ±60°, pitch -60° to +90°)
  - Line-of-sight from agent eye position (occlusion-aware)
- **Navigation surface**: Uses `nav_surface` scan (2D cone + downward probe) for reliable terrain mapping
- **Does NOT report**: Blocks outside cone, occluded blocks, blocks beyond radius

### mc-observe-entities / mc-observe-items
- **Reports**: Exhaustive enumeration of visible entities/items within radius
- **Same visibility criteria** as blocks (cone + LOS)
- **Distance**: Measured from agent foot position

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

- Use `mc-observe-blocks` to:
  - Determine navigation surface (`nav_surface`)
  - Identify visible block types (`nearby_blocks`)
  - Confirm presence when observed

- Use `mc-map-update` to:
  - Persist navigation surface data
  - Build spatial map for path planning

- Use coordinate reasoning and prior knowledge to:
  - Infer block existence outside observation cone
  - Choose dig targets beyond visible range

---

## Agent Rule (Strong)

Observation provides **exhaustive but cone-limited** data.
It supplements reasoning but does not replace it.
Blocks outside the cone or beyond radius still exist and can be reasoned about.
