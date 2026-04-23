# Desktop / operator console — change spec: local 2.5D map

**Date:** 2026-04-17  
**Audience:** Body desktop dev UI implementers.  
**Normative payload:** `schemas.local_map_2p5d` in [body/lib/schemas.py](../body/lib/schemas.py); behavior: [local_map_spec.md](local_map_spec.md).

---

## Summary

The Pi may run **`body.local_map`**, which publishes a fused **egocentric height grid** on Zenoh for navigation debugging.

| Item | Value |
|------|--------|
| **Topic** | `body/map/local_2p5d` |
| **Transport** | Zenoh JSON (`put`), same session as other `body/**` topics |
| **Cadence** | ~`publish_hz` from Pi `config.json` → `local_map.publish_hz` (default **2 Hz** when enabled; raise in config if you need faster updates) |
| **Gated by** | `local_map.enabled: true` on Pi; process still runs when false but **does not** publish (idle) |

---

## Subscribe

Add **`body/map/local_2p5d`** to the console’s subscriber list (exact key or `body/map/**` if your client already uses prefixes).

---

## Message shape (JSON)

All keys below are **stable for v1**; ignore unknown future keys.

| Field | Type | Description |
|-------|------|-------------|
| `ts` | number | Host wall time when the map was emitted (seconds, same style as other Body topics). |
| `frame` | string | `"body"` — robot body frame: +x forward, +y left, +z up. |
| `kind` | string | `"max_height_grid"` for this version. |
| `resolution_m` | number | Cell size (m), square cells. |
| `origin_x_m` | number | World **x** of grid corner (minimum x, smallest forward index). |
| `origin_y_m` | number | World **y** of grid corner (minimum y, smallest left index). |
| `nx` | int | Cells along **+x** (forward). |
| `ny` | int | Cells along **+y** (left). |
| `max_height_m` | array | Length `nx`; each element is length `ny`. Values are **body-frame** \(z\) (m) of the **max** fused sample per cell, or JSON **`null`** (no sample). See [local_map_spec.md](local_map_spec.md). |
| `driveable` | array? | Same shape as `max_height_m` when Pi has `driveable_enabled`: **`true`** / **`false`** / **`null`** per cell. |
| `driveable_clearance_height_m` | number? | Slab top (m) when `driveable` is published. |
| `sources` | object? | Optional: `lidar_ts`, `depth_ts` — timestamps of inputs fused into this frame (may differ). |

**Indexing:** `max_height_m[i][j]` = cell at body position approximately  
`x ≈ origin_x_m + (i + 0.5) * resolution_m`,  
`y ≈ origin_y_m + (j + 0.5) * resolution_m`.

Robot origin **(0,0)** is the body-frame reference used on the Pi (typically between wheels / CAD base). The grid **rectangle** is configured on the Pi (`extent_forward_m`, etc.); the corner `(origin_x_m, origin_y_m)` is **not** necessarily `(0,0)`.

---

## Visualization (recommended)

- **Top-down 2D:** color = `max_height_m[i][j]`; treat **`null`** as transparent or “unknown” (distinct from height 0 if you ever use ground at 0). Optional second layer for **`driveable`** (`true` / `false` / `null`).
- **Scale:** fixed color range e.g. **0–2.2 m** for quick sanity, or auto per frame with a “dynamic scale” badge.
- **Robot:** mark **(0,0)** and draw **+x forward** arrow; align with lidar “forward up” convention after your polar plot fix.
- **Metadata line:** `nx×ny @ resolution_m`, age since `ts`, optional `sources` ages.
- **Stale:** if no sample for **> 2×** expected period, gray out or show “no map” (same spirit as other Body streams).

Large grids (fine `resolution_m`) can be **big JSON**; if the UI stutters, allow downsampling for display only.

---

## Console acceptance / tests

1. **Pi:** `local_map.enabled: true` in `config.json`; `zenohd` + launcher running; **`lidar_driver`** and **`oakd_driver`** publishing real (or stub) data.  
2. **Subscribe** on desktop to `body/map/local_2p5d`; confirm JSON parses and **`kind`** is `max_height_grid`.  
3. **Sanity:** with robot facing a wall, cells in front should show **non-null** heights roughly consistent with **lidar height** (~0.10 m; see `lidar.height_above_ground_m`) and **depth** hits (variable by range).  
4. **Disabled:** `local_map.enabled: false` — topic may **not** update (no publishes); UI should tolerate **missing** topic.

---

## References

- [local_map_spec.md](local_map_spec.md) — Pi config and fusion rules.  
- [agent_desktop_stub_spec.md](agent_desktop_stub_spec.md) — overall console contract.
