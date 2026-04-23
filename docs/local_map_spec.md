# Local 2.5D map (`body/map/local_2p5d`)

**Process:** `python -m body.local_map` (see `body/launcher.py` when `local_map.enabled` is true).

## Purpose

Egocentric **max height above ground** grid in the **body frame** (+x forward, +y left, +z up). Each cell stores the highest obstacle sample from:

- **`body/lidar/scan`** — horizontal slice at `lidar_z_body_m` (default from `lidar.height_above_ground_m`).
- **`body/oakd/depth`** with `format: depth_uint16_mm` — stereo points unprojected with approximate intrinsics and `depth_*` extrinsics.

This matches the navigation intent: lidar sees a **low, horizontal ring** (~10 cm above the floor on this build); depth fills **near-ground** structure in the camera wedge. Fusion rule for `max_height_m` is **per-cell maximum** of valid body-frame \(z\) samples (no temporal decay).

**Driveable grid (optional):** When `driveable_enabled` is true, the message may include `driveable` (same shape as `max_height_m`) with `true` / `false` / `null` per cell. A cell is **blocked** if lidar or depth places **any** sample whose **height above the fitted floor plane** lies between `driveable_floor_band_m` and `driveable_clearance_height_m` (exclusive of the floor band, inclusive of clearance top). Returns above the clearance height are ignored for driveability. The floor plane is **RANSAC-fit** to depth points in a configurable image ROI every **`floor_fit_interval_s`**; between updates the last good plane is held. Until `driveable_clear_frames` consecutive observed frames have no slab hit, a cell stays **not** driveable (sticky clear). Unobserved cells keep the previous driveable verdict.

**Depth presmoothing:** Before unprojection, the depth image can be passed through a **median filter** over valid pixels only (`depth_median_kernel`, default **3**; set **0** or **1** to disable). That cuts stereo speckle with modest cost; like any spatial filter it slightly softens depth discontinuities. If that blur is noticeable at your current `oakd.depth_out_width` / `depth_out_height`, raising resolution (more pixels per radian) shrinks the **angular** size of a fixed \(k \times k\) kernel, so the same median hurts fine detail less—at the cost of larger Zenoh payloads and more projection work. Trade bandwidth/CPU against noise.

## Configuration (`config.json` → `local_map`)

| Key | Meaning |
|-----|--------|
| `enabled` | If false, `local_map` exits immediately (launcher still starts the module; use launcher edit or systemd to omit). |
| `publish_hz` | Fusion/publish rate (default **2 Hz** in code and sample `config.json` — large JSON payloads; increase only if needed). |
| `resolution_m` | Square cell size (m). |
| `extent_*_m` | Rectangle around robot: forward/back/left/right from body origin. |
| `ground_z_body_m` | Ground plane z (usually 0). Samples at or below are ignored. |
| `lidar_x_body_m`, `lidar_y_body_m`, `lidar_yaw_rad` | Lidar origin and yaw offset (rad) for scan angles (0 = forward per spec, CCW). |
| `lidar_z_body_m` | Optional override; else `lidar.height_above_ground_m`. |
| `depth_x_body_m`, `depth_y_body_m`, `depth_z_body_m` | Camera optical center in body; z defaults to `oakd.depth_camera_height_above_ground_m`. |
| `depth_yaw_rad`, `depth_pitch_rad`, `depth_roll_rad` | Euler (Z–Y–X) after fixed OpenCV→body axis fix (see code). |
| `depth_hfov_deg`, `depth_vfov_deg` | Approximate pinhole FOV for resized depth (`oakd.depth_out_width` × `height`). |
| `depth_median_kernel` | Odd kernel size (e.g. **3**) for median filter on depth **before** unprojection; invalid pixels (`0` mm) skipped. **0** or **1** = off. |
| `driveable_enabled` | If true and `driveable_clearance_height_m` > 0, publish `driveable` + `driveable_clearance_height_m`. |
| `driveable_clearance_height_m` | Slab top (m): plane-relative height above which samples do not block. |
| `driveable_floor_band_m` | Thickness (m) around the plane treated as floor (not an obstacle). |
| `driveable_slab_min_pixels` | Minimum depth pixels in a cell whose plane-relative height lands in the obstacle slab before the cell is flagged `slab_hit` this frame. **1** = any speckle blocks. Default **2**. |
| `driveable_floor_min_pixels` | Minimum depth pixels classified as floor (plane-relative height ≤ `driveable_floor_band_m`) in a cell before it counts as **observed** for the driveable clear-streak. Default **2**. |
| `driveable_clear_frames` | Observed frames with no slab hit before a cell becomes `driveable` true. |
| `driveable_unobs_decay_frames` | Per-tick decrement applied to the clear-streak counter of any cell **not** observed this frame. **0** disables decay (cells stay `_D_OK` forever until re-observed; old behavior). **1** (default) gives a memoryless half-life of roughly `driveable_clear_frames` publish ticks before a previously-clear cell drifts back to `null` (unknown). Speckle-induced false clears fade instead of sticking. |
| `floor_fit_interval_s` | Minimum time between RANSAC floor fits on depth. |
| `floor_fit_ransac_iters`, `floor_fit_inlier_m`, `floor_fit_min_inliers`, `floor_fit_max_samples` | RANSAC subsample and quality. |
| `floor_roi_u0`, `floor_roi_u1`, `floor_roi_v0`, `floor_roi_v1` | Normalized depth image ROI \([0,1]\) for floor points (defaults bias the lower band of the image — tighten if door frames / walls pollute the fit). |
| `floor_fit_log_interval_s` | Seconds between stdout logs of the fitted plane (`n`, `d`, implied pitch / roll). Set **0** to disable. Use the printed `depth_pitch_rad` / `depth_roll_rad` suggestions to baseline your camera extrinsics. |
| `depth_roi_u0`, `depth_roi_u1`, `depth_roi_v0`, `depth_roi_v1` | Normalized depth image ROI \([0,1]\) for **all** unprojected points feeding `max_height_m`, `floor_seen`, and the obstacle slab. Defaults drop the outer 5 % on each side to discard stereo edge speckle. Wider = more coverage + more edge noise; tighter = cleaner but narrower effective FOV. Independent of the floor-fit ROI above. |

Tune **`lidar_yaw_rad`** / **`depth_*`** to match your CAD mount; defaults assume sensors on the body origin.

## Wire message (`schemas.local_map_2p5d`)

- `frame`: `"body"`.
- `kind`: `"max_height_grid"`.
- `origin_x_m`, `origin_y_m`: grid corner (minimum x, minimum y).
- `nx`, `ny`: cell counts; index **i** along +x, **j** along +y.
- `max_height_m`: `list` length `nx`, each row length `ny`, entries `null` or **body-frame** \(z\) (m) of the highest sample (not plane-relative).
- `driveable`: optional same shape; `true` / `false` / `null` if enabled.
- `driveable_clearance_height_m`: optional scalar matching config slab top.
- `sources`: optional `lidar_ts`, `depth_ts` of inputs used.

## Extrinsics you still owe the stack

Horizontal **offsets** (lidar vs body origin, camera vs body), **yaw** alignment between lidar 0° and body +x, and **camera pitch/roll** if the OAK is not level. Heights alone are not enough; fill the `*_x_body_m`, `*_y_body_m`, and Euler fields from CAD or calibration.

## Related

- [body_project_spec.md](body_project_spec.md) — `body/lidar/scan`, `body/oakd/depth`.
