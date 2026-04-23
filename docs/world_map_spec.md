# Desktop world driveable map — specification

**Version:** 0.1 draft
**Date:** 2026-04-22
**Audience:** Desktop implementer of the continuous world-map / SLAM fuser.
**Status:** Specification only. No Pi-side changes are required for v1. Optional Pi follow-ups listed in §12.
**Normative references:**
[body_project_spec.md](body_project_spec.md) (wire contract + safety),
[local_map_spec.md](local_map_spec.md) (Pi-side driveable grid),
[sweep360_spec.md](sweep360_spec.md) (prior stop-and-shoot fuser — superseded for general mapping; retained for calibration, see §11),
[agent_desktop_stub_spec.md](agent_desktop_stub_spec.md) (commanding-client discipline — does NOT apply here; see §2).

---

## 1. Purpose

Fuse the Pi-published `body/map/local_2p5d` stream with `body/odom` into a **persistent, continuously-updated world-frame driveable map** while the robot drives freely. Supersedes `sweep_360` as the primary mapping surface for planners.

Goals:

- Dense `max_height_m` + `driveable` raster in a fixed world frame, at the same resolution as `local_2p5d`.
- Update continuously at the `local_map` publish rate (currently 5 Hz).
- Publish a wire message shape-compatible with `body/map/local_2p5d` so existing desktop viewers render it without code changes.
- Expose an evidence-rich grid (per-cell observation counts, traversal stamps) so the body_stub native planner can reason about confidence, not just driveability.
- Keep the pose source swappable behind one interface: odom-only in v1, odom + scan-match correction in v1.1. The v1 code path must not need rewriting to admit the v1.1 corrector.
- Use **traversal** (robot physically covered a cell) as a first-class driveability signal. This is the only signal available for cells directly under the robot body.

Non-goals:

- Loop closure / pose-graph global optimization. Deferred to v2.
- Multi-session persistence (save/load to disk across power cycles). Deferred to v2.
- Path planning or obstacle-aware control. Consumers do their own planning.
- Replacing `local_map` or `sweep_360` inputs — both remain on the bus as-is.

---

## 2. Where this runs

**Desktop**, as a **standalone sibling process** to `body_stub` (the operator console / dev UI). Separate process specifically so it can take its own core under load — this fuser is expected to grow into the home of higher-level navigation and planning work, and should not compete with the UI thread.

`body_stub` consumes `body/map/world_driveable` like any other published map topic; the fuser and the UI share nothing but the Zenoh bus.

**This process is consumer-only.** It never publishes `body/cmd_vel`, `body/heartbeat`, or `body/cmd_direct`. The commanding-client rules of [agent_desktop_stub_spec.md](agent_desktop_stub_spec.md) §4.0 therefore do not apply. An operator can drive the robot via `body_stub` (or any other commander) with the fuser running passively.

Suggested codebase location: `dev/world_map/` as a sibling to `dev/body_stub/`. Entry point `python -m dev.world_map`. Implementer's choice.

---

## 3. Architecture

```mermaid
flowchart TD
    odom["body/odom (50 Hz, wheel-tick integrated)"] --> pose[PoseSource interface]
    lmap["body/map/local_2p5d (5 Hz)"] --> fuse[Grid fusion: rototranslate into world, per-cell update]
    lidar["body/lidar/scan (10 Hz)"] -.v1.1.-> pose
    pose --> fuse
    odom --> trav[Traversal stamper: footprint disk at current pose]
    trav --> grid
    fuse --> grid[World grid: max_height + votes + traversed_ts + observed_ts + obs_count]
    grid --> pub["body/map/world_driveable (≤ 2 Hz, cropped to bounds)"]
    grid --> status["body/world_map/status (≥ 1 Hz)"]
    cmd["body/world_map/cmd"] --> reset[Reset / relocate]
    reset --> grid
```

---

## 4. Wire contract

All JSON over Zenoh, same style as the rest of `body/**`. New topics live under `body/world_map/` for control/status; the map output lives under `body/map/` to match the `local_2p5d` / `sweep_360` pattern.

### 4.1 `body/map/world_driveable` — primary output (fuser → bus)

Same JSON shape as `body/map/local_2p5d` (see `schemas.local_map_2p5d` in [body/lib/schemas.py](../body/lib/schemas.py) and [local_map_spec.md](local_map_spec.md) §Wire message), with `frame = "world"` and the following additions:

```json
{
  "ts": 1713264000.123,
  "frame": "world",
  "kind": "max_height_grid",
  "resolution_m": 0.08,
  "origin_x_m": -20.0,
  "origin_y_m": -20.0,
  "nx": 500,
  "ny": 500,
  "max_height_m": [[null, 0.12, ...], ...],
  "driveable": [[null, true, ...], ...],
  "driveable_clearance_height_m": 0.35,

  "session_id": "c2a8f1...",
  "world_anchor_pose": { "x_m": 0.0, "y_m": 0.0, "theta_rad": 0.0 },
  "bounds_m": { "min_x": -3.2, "max_x": 4.4, "min_y": -1.6, "max_y": 2.1 },
  "pose_source": "odom",
  "observation_count": [[0, 3, ...], ...],
  "last_observed_ts": [[null, 1713263990.1, ...], ...],
  "traversed_ts": [[null, 1713263995.3, ...], ...]
}
```

Additions vs. `local_2p5d`:

| Field | Type | Meaning |
|-------|------|---------|
| `session_id` | string | UUID minted on fuser start or `world_cmd action=reset`. Consumers detect a fresh world by change of `session_id`. |
| `world_anchor_pose` | object | Body pose in world frame at session start — identity `(0,0,0)` by construction. Present for forward compatibility with v2 multi-session anchoring. |
| `bounds_m` | object | Tight bounding box (in world frame) of cells that have any evidence. Consumers can crop rendering to this box. |
| `pose_source` | string | `"odom"` (v1) or `"odom+scanmatch"` (v1.1). Informational. |
| `observation_count` | int grid? | Optional per-cell count of fusion touches. `null` in place of 0 is legal. |
| `last_observed_ts` | float grid? | Optional wall-clock ts of most recent perception touch; `null` where never observed. |
| `traversed_ts` | float grid? | Optional wall-clock ts of most recent traversal touch; `null` where never traversed. |

Ignore unknown fields on consume (forward-compatible).

**Cropping rule — mandatory.** The fuser MUST publish only the sub-grid covering `bounds_m` extended by `publish_margin_cells` (default 4). `origin_x_m`, `origin_y_m`, `nx`, `ny` refer to the cropped grid, not the allocated one. Rationale: a 40 m × 40 m world at 0.08 m is 250 k cells × multiple layers — uncropped JSON publishes are several MB each. Early exploration sessions touch a few thousand cells; the full grid is wasteful.

**Cadence:** ≤ `publish_hz` (default 2.0 Hz) regardless of the 5 Hz `local_map` input rate. Internal fusion runs at input rate; publish is throttled. Also publish immediately on `session_id` change.

### 4.2 `body/world_map/cmd` (any client → fuser, inbound)

```json
{ "action": "reset", "reason": "manual_move", "request_id": "..." }
```

```json
{ "action": "relocate", "pose": { "x_m": 0.0, "y_m": 0.0, "theta_rad": 0.0 }, "request_id": "..." }
```

| `action` | Meaning |
|----------|---------|
| `reset` | Clear all grid layers. Rebind world frame to current body pose (new identity). Mint a new `session_id`. v1 supports this. |
| `relocate` | Teleport the body's pose in the world frame to `pose`. Grid intact. For "operator picked up the robot and put it back roughly here." v1.1. |

Unknown actions ignored. Fire-and-forget — response is the resulting `world_map/status` publish.

### 4.3 `body/world_map/status` (fuser → bus, ≥ 1 Hz while process is alive)

```json
{
  "ts": 1713264000.123,
  "session_id": "c2a8f1...",
  "pose_source": "odom",
  "pose_world": { "x_m": 1.23, "y_m": -0.45, "theta_rad": 0.12 },
  "input_rates_hz": { "local_map": 4.9, "odom": 49.6, "scan": 9.8 },
  "input_age_s": { "local_map": 0.22, "odom": 0.02, "scan": 0.11 },
  "grid_cells_allocated": 250000,
  "grid_cells_observed": 3124,
  "grid_cells_traversed": 214,
  "last_correction": { "dx_m": 0.0, "dy_m": 0.0, "dtheta_rad": 0.0 },
  "notes": null
}
```

Operational telemetry. Consumed by the operator console to show "fuser is alive, getting fresh inputs, here's where it thinks you are, here's how hard scan-match is working" (v1.1).

`notes` is a short free-form string for transient conditions: `"stall:local_map"`, `"stall:odom"`, `"pose_unavailable"`, `"world_bounds_exceeded"`, or `null`.

### 4.4 No motion topics

The fuser does not publish `body/cmd_vel`, `body/cmd_direct`, or `body/heartbeat`. An operator-driven or agent-driven commander elsewhere owns motion; the fuser observes.

---

## 5. Inputs consumed

| Topic | Role | Required? |
|-------|------|-----------|
| `body/map/local_2p5d` | Primary fusion source. Each frame is rototranslated into world using the pose at the frame's effective capture timestamp. | Yes |
| `body/odom` | Primary pose source. 50 Hz wheel-encoder-integrated pose (post the Pi work that shipped on 2026-04-22). Interpolated to match `local_map` capture ts. | Yes |
| `body/lidar/scan` | v1 unused. v1.1 scan-match correction input. Subscribe from v1 regardless so the buffer is warm. | Yes (buffered v1; consumed v1.1) |
| `body/status` | Soft health gate. Fuser does not stop fusing on e-stop — mapping continues, motion is the Pi's concern. | No |
| `body/world_map/cmd` | Control input (reset / relocate). | Yes |

**Pose-timestamp alignment.** Each `local_map` message carries `sources.lidar_ts` / `depth_ts`. The fuser takes the **newer** of the two as the effective capture time, then looks up pose by linear interpolation across the two `body/odom` samples bracketing that ts. θ is wrap-safe (unwrap then interpolate, or interpolate sin/cos). If the latency between the chosen capture ts and the newest available `body/odom` exceeds `stale_odom_s` (default 0.25 s), the `local_map` frame is dropped and the skip logged.

---

## 6. Algorithm

### 6.1 Pose source (swappable)

**Interface** — normative, so v1.1 can drop in without rewriting the fuser:

```
class PoseSource:
    def pose_at(ts: float) -> Optional[Pose]   # (x_m, y_m, theta_rad)
    def cov_at(ts: float) -> Optional[np.ndarray]  # 3x3; v1 may always return None
    def notify_correction(dx, dy, dtheta, ts)  # v1.1 hook; no-op in v1
```

**v1 implementation — `OdomPose`:**
- Subscribe to `body/odom`, keep a 2 s ring buffer of `(ts, x, y, theta)`.
- `pose_at(ts)` linearly interpolates `x`, `y`. For `theta`: unwrap the two bracketing samples to a common branch, linearly interpolate, re-normalize to `[-π, π]`.
- Returns `None` if `ts` is outside the buffer or only one sample is available.

**v1.1 implementation — `OdomPlusScanMatch`:**
- Wraps `OdomPose`. On each `local_map` arrival, predicts pose from odom, then runs a 2D ICP of the freshest `body/lidar/scan` against the world grid's current obstacle set (`block_votes > clear_votes`). The ICP-corrected pose is returned.
- Correction is a delta on top of odom, not a replacement. Correction magnitudes are published in `world_status.last_correction` — if these grow large over time, odom drift is real and visible.
- ICP failure → fall through to pure odom for that frame, increment a `"scanmatch_fail"` counter exposed in status.

### 6.2 Grid storage

Dense arrays in world frame, pre-allocated at fuser start (or on `reset`) from `world_extent_m`. At `world_extent_m = 40.0`, `world_resolution_m = 0.08`, that is 500 × 500 = 250 k cells. With five layers it is ~6 MB total — negligible.

Layers:

| Name | Dtype | Meaning | Empty value |
|------|-------|---------|-------------|
| `max_height_m` | float32 | Per-cell running max of body-frame z samples. | NaN |
| `clear_votes` | int32 | Count of `driveable=true` observations + traversal increments. | 0 |
| `block_votes` | int32 | Count of `driveable=false` observations. Never incremented by traversal. | 0 |
| `traversed_ts` | float32 | Wall-clock ts of most recent traversal coverage. | NaN |
| `last_observed_ts` | float32 | Wall-clock ts of most recent perception touch. | NaN |
| `observation_count` | int32 | Total fusion touches (perception + traversal). | 0 |

World origin `(0, 0)` maps to the grid's center cell. `grid_origin_x_m = -world_extent_m / 2`, same for y.

Resolution must match `local_map.resolution_m`. If the first incoming `local_map` has a different resolution, the fuser enters the `notes = "resolution_mismatch"` error state and stops fusing. Restart with the matching value. (This is why §12 asks Pi to document the "resolution is fixed for a session" contract.)

Growable / tiled storage is a v2 optimization.

### 6.3 Fusion step (per `local_map` arrival)

1. Resolve capture ts `t_cap = max(sources.lidar_ts, sources.depth_ts, msg.ts)`.
2. `pose = pose_source.pose_at(t_cap)`. If `None`, drop frame, `notes = "pose_unavailable"`, increment skip counter.
3. Build rigid transform `T = [[cos θ, -sin θ, x], [sin θ, cos θ, y], [0, 0, 1]]`.
4. Forward-project every observed source cell `(i_b, j_b)`:
   - Body-frame center: `(x_b, y_b) = (lm.origin_x_m + (i_b + 0.5) * res, lm.origin_y_m + (j_b + 0.5) * res)`.
   - World-frame center: `(x_w, y_w) = T · (x_b, y_b, 1)`.
   - World cell: `(i_w, j_w) = (floor((x_w - grid_origin_x_m) / res + 1e-9), floor((y_w - grid_origin_y_m) / res + 1e-9))`.
   - If out of world bounds, clip and set `notes = "world_bounds_exceeded"`; continue.
   - Update layers:
     - `max_height_m[i_w, j_w] = nanmax(existing, source.max_height_m)`.
     - If `source.driveable == True`: `clear_votes += 1`.
     - If `source.driveable == False`: `block_votes += 1`.
     - `observation_count += 1`, `last_observed_ts = t_cap`.
5. Update `bounds_m` to include the set of world cells touched this frame.

Forward-project only (no world-to-body reverse lookup). Aliasing — adjacent body cells mapping to the same world cell at oblique yaws — is absorbed by `nanmax` for heights and by the vote accumulator for driveability. Same behavior as `sweep_mission.py:443-519`.

The `1e-9` epsilon in `floor` matches the convention used in `sweep_mission.py:500-501` — float64 index math, nudge boundary cells consistently.

### 6.4 Traversal stamping

Independent of `local_map` fusion. Runs at `traversal_stamp_hz` (default 10 Hz — every 5th `body/odom` message).

On each tick:
- `(x, y, θ) = latest_odom_pose_world`.
- For every world cell `(i_w, j_w)` whose center is within `footprint_radius_m` (default 0.15 m — slightly larger than half the 0.19 m wheel-base) of `(x, y)`:
  - `traversed_ts = nanmax(existing, current_ts)`.
  - `clear_votes += traversal_vote_weight` (default 3).
  - `observation_count += 1`.
  - `last_observed_ts = nanmax(existing, current_ts)`.

**Traversal must never increment `block_votes`.**

Rationale: the robot physically occupying a cell is the strongest possible driveability signal, and it's the **only** signal for cells under the robot body (lidar and depth cannot see there). Weight 3 survives two contradictory perception observations, roughly the amount of depth speckle we expect per cell in a short window.

Gating: if `pose_source` has not produced a valid pose in the last `stale_odom_s`, do not stamp. A stale pose could stamp the wrong cells.

### 6.5 Driveable verdict (for publish)

Per cell, published `driveable` value:

- `True` if `clear_votes > block_votes + vote_margin` (default `vote_margin = 1`).
- `False` if `block_votes > clear_votes + vote_margin`.
- `None` otherwise.

This is the same rule as `sweep_mission.py:521-529`. The traversal weight tilts the margin decisively for cells the robot has driven, which is intentional.

No decay in v1. v2 will add a last-observed freshness cutoff.

### 6.6 Reset / relocate

`reset`:
1. Re-allocate all layers to empty.
2. Rebind world frame: the current body pose becomes `(0, 0, 0)` in the new world.
3. Mint new `session_id`.
4. Publish `world_map/status` and `map/world_driveable` (latter will be empty but carry the new `session_id`).

`relocate` (v1.1): leave all layers intact; store a pose offset so future pose lookups report the new pose while the world grid stays fixed. Effect: "I manually moved the robot; keep the map."

---

## 7. Safety

- **Consumer only.** The fuser publishes only `body/map/world_driveable` and `body/world_map/status`. It never publishes motion. The safety envelope is the Pi's and the active commander's.
- **Input-stall detection.** If `local_map` or `odom` goes silent for > `input_timeout_s` (default 2.0 s), the fuser keeps the accumulator intact and sets `world_status.notes = "stall:<topic>"`. It does not reset the world.
- **Pose-source failure.** If `pose_at` returns `None` for 10 consecutive `local_map` frames, the fuser sets `notes = "pose_unavailable"` and stops fusing until pose recovers. Accumulator preserved.
- **Grid overflow.** If a `local_map` projection tries to write outside allocated world bounds, the frame is clipped to bounds and `notes = "world_bounds_exceeded"`. Operator manually `reset`s with a larger `world_extent_m`.
- **E-stop.** Fusion continues unaffected during e-stop. The robot is stationary; there is no reason to discard perception. Motion safety is not this process's concern.

---

## 8. Configuration

Desktop-side only; no Pi config changes.

| Knob | Default | Purpose |
|------|---------|---------|
| `zenoh_connect` | `tcp/<pi-ip>:7447` | Same endpoint as body_stub. Respect `ZENOH_CONNECT`. |
| `world_extent_m` | 40.0 | Square world side length, centered on world origin. |
| `world_resolution_m` | 0.08 | Must match `local_map.resolution_m`; refuse to run on mismatch. |
| `publish_hz` | 2.0 | §4.1 cap. |
| `publish_margin_cells` | 4 | §4.1 crop padding around `bounds_m`. |
| `stale_odom_s` | 0.25 | §5 alignment tolerance. |
| `input_timeout_s` | 2.0 | §7 stall detection. |
| `pose_source` | `"odom"` | `"odom"` (v1) or `"odom+scanmatch"` (v1.1). |
| `vote_margin` | 1 | §6.5. |
| `traversal_stamp_hz` | 10.0 | §6.4. |
| `traversal_vote_weight` | 3 | §6.4. |
| `footprint_radius_m` | 0.15 | §6.4. |
| `scanmatch_min_inliers` | 60 | v1.1 ICP quality gate. |

---

## 9. Acceptance criteria

- [ ] Fuser process runs independently of `body_stub`; `body_stub` subscribes to `body/map/world_driveable` and renders it in the existing local_map viewer without code changes.
- [ ] On a hand-driven straight-line run of 2 m, the world map extends forward by ~2 m and walls observed in `local_map` remain straight in the fused grid (no visible shear).
- [ ] On a hand-driven 2 m × 2 m square back to origin, walls at the starting pose align within ~20 cm of the second pass (v1, odom-only). v1.1 scan-match should tighten this substantially.
- [ ] Cells the robot has physically driven through are marked `driveable = True` in the published grid, even where `local_map` shows them as `null` (under-robot blind spot).
- [ ] `body/world_map/cmd action=reset` clears the grid, increments `session_id`, and starts fusion fresh from the current body pose.
- [ ] `body/world_map/status` publishes at ≥ 1 Hz and reflects input-age values consistent with Pi rates (`local_map` ≤ 0.3 s in steady state, `odom` ≤ 0.05 s).
- [ ] Fuser never publishes `body/cmd_vel`, `body/cmd_direct`, or `body/heartbeat`.
- [ ] Published map is cropped to `bounds_m + publish_margin_cells`; `nx * ny` scales with explored area, not allocated extent.
- [ ] `local_map` stall sets `notes = "stall:local_map"` and preserves the accumulator.
- [ ] `odom` stall sets `notes = "stall:odom"` and suspends traversal stamping; accumulator preserved.
- [ ] Swapping `pose_source` from `"odom"` to `"odom+scanmatch"` (v1.1) does not require changes to the fusion or publish paths.
- [ ] With `pose_source = "odom"`, produces a usable map over ≤ 20 m of driving before drift becomes visually intolerable.

---

## 10. Out-of-scope / deferred

- **Scan-match correction.** Interface lands in v1 (PoseSource), implementation is v1.1.
- **`world_cmd action=relocate`.** v1.1.
- **Loop closure / pose-graph SLAM.** v2.
- **Multi-session persistence** (save/load to disk, re-anchoring a stored map on new session). v2. Requires a stable cross-session world anchor — either a "where I was last shutdown" pose saved next to the map, or a scan-match bootstrap against the saved map on startup. Both are real work.
- **Growable / tiled grid storage.** v2, when 40 m × 40 m shows real resource pressure.
- **Decay policies** for stale perception evidence. v2.
- **Cost-grid / navigable-graph outputs.** Depends on planner requirements; current consumer (body_stub native planner via LLM) consumes the raw raster.
- **Operator UI panel for the fuser** (`reset` button, drift monitor, correction visualizer). Belongs in body_stub, specified separately when needed.

---

## 11. Relationship to sweep_360

[`sweep_360`](sweep360_spec.md) is retained as a **calibration / debug mission**. Its explicit 360° loop-closure residual is useful for sanity-checking the perception pipeline with a stationary robot ("is my scan-matcher getting close to zero here?"). It is no longer the primary mapping surface and should not receive new feature work. The world-map fuser supersedes it for any mapping consumed by a planner.

Practical consequence: once the world-map fuser is shipping, `sweep_360` becomes a tool that runs occasionally, not a thing running in steady state.

---

## 12. Pi-side notes — optional follow-ups

No Pi changes are required for v1. The following would simplify desktop work but are not blocking:

- **Stamp `local_map` with `odom_pose_at_capture`.** A field `odom_pose_at_capture: {x_m, y_m, theta_rad}` on each `local_map` message would eliminate the desktop-side timestamp-alignment step entirely. Pi already has both sources; it is the authority on their clock relationship. Would collapse §5 and §6.1 into a single lookup.
- **Odom covariance or velocity-variance estimate.** A `body/odom.covariance` or `body/odom.vel_std` field gives the v1.1 scan-match correction a principled prior. Without it, the scan-matcher hard-codes a guess.
- **Document resolution / extent stability** in [local_map_spec.md](local_map_spec.md). Imply: `resolution_m` and `extent_*_m` are fixed for the lifetime of a Pi session; changes require a Pi restart. If Pi ever chooses to change them mid-session, a `session_id`-style bump on the `local_map` message would let the desktop re-initialize cleanly.
- **Odom source tag** (`body/odom.source: "commanded" | "encoder" | "encoder+imu"`). Lets the desktop weight scan-match correction differently when odom quality changes. Cheap to add when encoders go live tomorrow.

---

## 13. References

- [body_project_spec.md](body_project_spec.md)
- [local_map_spec.md](local_map_spec.md)
- [sweep360_spec.md](sweep360_spec.md)
- [agent_desktop_stub_spec.md](agent_desktop_stub_spec.md)
- [desktop_change_spec_local_map.md](desktop_change_spec_local_map.md)
- [body/lib/schemas.py](../body/lib/schemas.py)
