# Body Operator Console — Zenoh Client & Visualizer

**Version:** 0.3 draft  
**Date:** 2026-04-21  
**Status:** Specification only (not yet implemented in this repo)

This document specifies a **desktop operator / dev tool** for the Body stack. It is **not** the Cognitive Workbench agent (Jill) and **not** bound to product agent design gates (e.g. D1); it exists to drive and visualize the same **Zenoh topics** as a real agent would, for integration testing and field debugging.

---

## 1. Purpose

Goals:

- **Publish** `body/heartbeat` and `body/cmd_vel` when the operator explicitly takes control (see §4).
- **Subscribe** to Body outputs for situational awareness and **visualize** OAK-D depth (false-color) and on-request RGB.
- Stay aligned with [body_project_spec.md](body_project_spec.md) for wire semantics and safety (e.g. heartbeat timeout, `cmd_vel` re-engagement after e-stop).

Non-goals: mapping, planning, LLM / “cognitive” behavior, or replacing Jill.

**Normative contract:** [body_project_spec.md](body_project_spec.md). This document adds UI/UX and desktop wiring only.

---

## 2. Scope: OAK-D Capabilities (Current)

The console exercises **two** OAK-D paths implemented on the Pi:

| Capability | Direction | Topics | Notes |
|------------|-----------|--------|--------|
| **Depth stream** | Pi → desktop | `body/oakd/depth` | **`format: "depth_uint16_mm"`** is **owned and shipped by Body** (`oakd_driver` on the Pi). Payload: base64 **uint16** depth in **mm**, row-major; **`width` and `height` are always taken from the message** (sample Pi config uses **120×90**). If the Pi still publishes `format: "placeholder"`, the console shows a textual placeholder only—**readiness of `depth_uint16_mm` is a Body-side deliverable**; the console is the consumer that assumes it once available. |
| **RGB snapshot (on-request)** | Desktop → Pi → desktop | `body/oakd/config` → `body/oakd/rgb` | `{"action": "capture_rgb", "request_id": "<uuid>"}`; JPEG in `body/oakd/rgb`. |

Other OAK topics (`body/oakd/imu`): subscribe; **text / compact numeric** panel, not image tiles.

Future on-request actions may follow the same `body/oakd/config` pattern.

---

## 3. Transport & Session

- **Zenoh** only, same as [body.teleop](../body/teleop.py) and future Jill bridges.
- **Endpoint configuration:** User-editable router endpoint (e.g. `tcp/192.168.1.50:7447`).
- **Environment override (canonical for this repo and the console):** **`ZENOH_CONNECT`** — single endpoint string, same convention as Body’s [config.json](../config.json) / [README](../README.md). Do not introduce a second env name for the same override in this spec.
- **Session lifecycle:** On disconnect, **stop** heartbeat and command publishing so the stack behaves like “no commanding client” for watchdog tests when desired.

---

## 4. Publishing & Subscribing (Wire Contract)

### 4.0 Single commanding client (normative)

**Operational rule:** Run **exactly one** commanding client against a given robot session: **this console**, **Jill’s bridge** (or equivalent agent), **or** `body.teleop` — **never two at once**.

Rationale: Zenoh has **no controller-of-record**; `cmd_vel` and heartbeat are **last-writer-wins**. Two live publishers produce ambiguous motion and ambiguous watchdog behavior.

This rule **replaces** a “competing heartbeat” subsection: the fix is **process discipline**, not UI heuristics.

### 4.1 Default mode: monitor-only (normative)

On connect, the console defaults to **monitor-only**:

- **Does not** publish `body/heartbeat`.
- **Does not** publish `body/cmd_vel` (or publishes explicit zeros only if a future mode requires it—prefer **no publish** until live command is enabled).

The operator must **explicitly** enable **“Live command”** (or equivalent) to start heartbeat (≥ **2 Hz**) and to allow non-zero `cmd_vel`. Label that control clearly so operators know they have taken ownership of the commanding-client role.

### 4.2 Publish (console → Body) when live command is enabled

| Topic | Requirement | Console behavior |
|-------|-------------|-------------------|
| `body/heartbeat` | ≥ **2 Hz** while live command is on | Timer-driven; monotonic `seq`, `ts`. |
| `body/cmd_vel` | Per spec; respect motor **timeout_ms** | Sliders/keys; **linear=0**, **angular=0** at rest; “all stop”. |
| `body/oakd/config` | On-request actions | At minimum **RGB capture** per [schemas](../body/lib/schemas.py) (`action`, `request_id`). |

**`body/cmd_direct`:** **v1 UI omits** direct wheel commands to avoid low-level footguns in a general operator UI. This is a **scope choice for this tool**, not a statement about product agent (D1) policy. Dev-only builds **may** add `cmd_direct` later behind a separate “calibration” entry point; not required for v1.

### 4.3 Subscribe (Body → console)

| Topic | Use |
|-------|-----|
| `body/status` | Processes, `heartbeat_ok`, `e_stop_active`, `uptime_s`; optional **`host`** — see [body_status_host_spec.md](body_status_host_spec.md) |
| `body/emergency_stop` | Prominent alert + `reason` |
| `body/odom` | Pose / velocity readout |
| `body/motor_state` | PWM, dirs, flags |
| `body/lidar/scan` | **v1:** simple **polar** render (matplotlib or equivalent; small implementation cost, high diagnostic value) plus last-update time |
| `body/oakd/imu` | Compact numeric / JSON |
| `body/oakd/depth` | False-color view (§5.1) |
| `body/oakd/rgb` | JPEG from capture responses |
| `body/map/local_2p5d` | Optional: fused **2.5D** grid for navigation debug — `max_height_m` plus optional **`driveable`** / **`driveable_clearance_height_m`** when enabled on the Pi — see [desktop_change_spec_local_map.md](desktop_change_spec_local_map.md) and [local_map_spec.md](local_map_spec.md) |

**Optional (recommended):** subscribe to **`body/cmd_vel`** in monitor-only (and when live) to show **last twist seen on the bus**—useful when another process is driving (should not happen per §4.0, but invaluable when debugging mistaken double-client setups).

---

## 5. Visualization Requirements

### 5.1 Depth (`body/oakd/depth`)

For `format == "depth_uint16_mm"`:

- Decode base64 → `uint16` array, shape `(height, width)`; **0** = invalid before colormap.
- **Default normalization:** map depth **0.2 m – 5.0 m** (200–5000 mm) to the colormap; clamp out-of-range valid pixels to the ends. **Fixed range** is default so consecutive frames are visually comparable (operator sanity check).
- **Optional toggle:** percentile-based normalization per frame (off by default); when on, show a short UI warning that scale **varies frame-to-frame** and can mislead.
- Colormap: e.g. **TURBO** or **inferno** (implementation choice).
- Show **age** of last frame (wall time − `ts`).

For `format == "placeholder"`: textual state only (“Pi not streaming depth yet”).

### 5.2 RGB (`body/oakd/rgb`)

- `ok: true`: decode JPEG; show `width`×`height`.
- `ok: false`: show `error`.
- Match `request_id`; ignore stale IDs.
- **Client-side timeout:** default **3.0 s** from publish of `capture_rgb` to matching reply. On timeout: show **“Capture timed out”**, clear in-flight `request_id`, do not block the UI indefinitely.

### 5.3 Lidar

**v1:** polar render of `ranges` (invalid entries as gaps or max-range), plus scan metadata (count, `scan_time_ms`, last update).

**Angle convention (normative, matches [body_project_spec.md](body_project_spec.md) §5.5):** `angle_min` / `angle_increment` define beam directions in radians with **0 = robot forward** and **increasing CCW** when viewed from above (same as robot frame: +x forward, +y left).

**Polar plot / matplotlib (implementation note):** By default, `matplotlib` polar axes put **θ = 0 at the right** (3 o’clock), not at the top. If the UI caption says **“forward is up”** but forward (index 0 / angle 0) appears **to the right**, that is a **pure UI bug**: either call **`ax.set_theta_zero_location('N')`** so θ = 0 is at the **top**, or equivalently offset angles by **+π/2** relative to matplotlib’s default zero, and verify **`set_theta_direction`** so CCW matches the spec. The STL-19P housing arrow and Body’s scan angles are aligned; fix the console, not `lidar_driver`.

### 5.4 Local 2.5D map (`body/map/local_2p5d`)

Normative JSON shape and indexing: [desktop_change_spec_local_map.md](desktop_change_spec_local_map.md). Pi behavior and tuning: [local_map_spec.md](local_map_spec.md).

- **`max_height_m`:** top-down colormap of stored values; treat **`null`** as unknown. Values are **body-frame** \(z\) of the **highest** fused sample per cell (not plane-relative height).
- **`driveable` (optional):** when present, same `nx`×`ny` layout as `max_height_m`; entries are **`true`**, **`false`**, or **`null`**. **`false`** means the Pi judged the cell **not** passable for the configured clearance slab; **`null`** means unknown or held state—**do not** treat as free space. Recommended: second layer (outline, tint, or toggle) so operators can see **driveable** vs **height** without conflating the two.
- **`driveable_clearance_height_m` (optional):** scalar; top of the obstacle slab used on the Pi (meters, plane-relative). Show in the map metadata when present so the console matches robot config.
- **Ignore unknown keys** in the message for forward compatibility.

---

## 6. Commands, E-Stop Semantics & Operator Actions

| Action | Effect |
|--------|--------|
| **Connect / Disconnect** | Zenoh session; on disconnect, stop publishing commands/heartbeat |
| **Live command** | Off = monitor-only (§4.1); On = heartbeat + cmd_vel allowed |
| **Drive controls** | `body/cmd_vel` when live command on |
| **`body/emergency_stop` display** | Show reason; **no wire “acknowledge_estop”** — this console is not a cognitive agent. **Acknowledgement is UI copy only** (explain that recovery is per Body spec: when inhibit conditions clear, operator must **re-publish `body/cmd_vel`** (and heartbeat if required) to re-engage motion). If future Body topics distinguish **MOTION_INHIBITED** vs **WATCHDOG_ESTOP**, surface them as **separate displayed states**; the console still performs **no** automated cognitive ack. |
| **Request RGB capture** | Publish `body/oakd/config` with new `request_id`; enforce §5.2 timeout |
| **Clear view** | Optional reset of image panels |

Stretch: rolling log of raw JSON samples for debug.

---

## 7. Implementation Notes (Non-Normative)

- **Stack:** Unspecified; Python + Qt/Tk/Dear PyGui or local web UI is fine.
- **Deps:** `eclipse-zenoh`, `numpy`, OpenCV or Pillow, matplotlib for lidar.
- **Redraw:** Decimate if depth > UI refresh needs.
- **Security:** Trusted LAN; TCP unless Zenoh TLS is configured elsewhere.
- **Recording (v1.1 suggestion):** dump last **N** seconds of selected topics to **JSONL** for bug reports—cheap to add early, painful to retrofit; not required for v1.0.

---

## 8. Acceptance Criteria

- [ ] Connects to Pi router; tolerates Wi-Fi jitter without crashing.
- [ ] **Monitor-only by default:** no heartbeat until live command enabled.
- [ ] With live command: heartbeat ≥ 2 Hz; `cmd_vel` with zero-default and stop control.
- [ ] Subscribes to: `status`, `emergency_stop`, `odom`, `motor_state`, `lidar/scan`, `oakd/imu`, `oakd/depth`, `oakd/rgb`; optional `cmd_vel` echo recommended.
- [ ] Depth: false-color for `depth_uint16_mm`; placeholder text otherwise. **Assumes Pi eventually publishes `depth_uint16_mm`** (Body owns that publisher).
- [ ] RGB: request + display JPEG; **3 s client timeout** with clear error.
- [ ] Lidar: **v1 polar** render + metadata.
- [ ] **Optional:** `body/map/local_2p5d` — height grid; if **`driveable`** is present, visualize it (e.g. overlay) and show **`driveable_clearance_height_m`** when present.
- [ ] E-stop: visible alert; **no fake wire ack**; UI text explains cmd_vel re-engagement.
- [ ] **Stale data:** if no `body/status` (or chosen liveness sample) within **~2 s** of wall clock, or Zenoh session loss, mark depth/status/sensor panels **stale** (do not silently show frozen frames as fresh). Exact heuristic implementation-defined but must avoid indefinite silent freeze after Pi reboot or network drop.

---

## 9. References

- [body_project_spec.md](body_project_spec.md)
- [body/lib/schemas.py](../body/lib/schemas.py)
- [body/teleop.py](../body/teleop.py)
- [local_map_spec.md](local_map_spec.md)
- [desktop_change_spec_local_map.md](desktop_change_spec_local_map.md)
