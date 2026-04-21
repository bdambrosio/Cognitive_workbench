# Body: Robot Body Software Project Specification

**Version:** 0.2 draft
**Date:** 2026-04-16
**Author:** Bruce + Claude (initial spec generation)
**Hardware target:** Raspberry Pi 5 (assumed), differential drive chassis
**Language:** Python 3.11+
**Transport:** Zenoh (pyzenoh)

---

## 1. Purpose

Body is the onboard software stack for a differential-drive robot chassis. It provides sensor acquisition, motor control, and safety supervision as a set of independent communicating processes linked by Zenoh. The desktop agent (Jill, running on the CW workstation) connects to the same Zenoh network and interacts with Body exclusively through published topics. Body has no knowledge of Jill's internals; Jill has no knowledge of Body's process structure. The Zenoh topic schema defined in this document is the contract between them.

## 2. Hardware Inventory

| Component | Interface | Notes |
|---|---|---|
| Cytron MDD10A dual 10A motor driver | GPIO: 2x PWM + 2x DIR | Channel A = left motor, Channel B = right motor |
| Pololu #4752 30:1 gearmotor w/ encoder (x2) | GPIO: 2x quadrature (A/B per motor) | 64 CPR motor shaft, 1920 CPR output shaft. 330 RPM no-load @ 12V. Stall: 14 kg·cm / 5.5A |
| LDROBOT STL-19P lidar | USB serial | 360° 2D scan, ~10Hz typical |
| OAK-D-Lite | USB3 (DepthAI) | Stereo depth, RGB, onboard IMU (BNO086) |
| Raspberry Pi 5 | — | 4-core, 8GB assumed. Runs all Body processes |

### GPIO Pin Assignments (provisional, confirm during wiring)

| Function | BCM Pin | Notes |
|---|---|---|
| Motor A PWM | 12 | Hardware PWM channel 0 |
| Motor A DIR | 5 | HIGH = forward, LOW = reverse (verify with MDD10A datasheet) |
| Motor B PWM | 13 | Hardware PWM channel 1 |
| Motor B DIR | 6 | Same convention |
| Encoder L channel A | 23 | Pull-up enabled |
| Encoder L channel B | 24 | Pull-up enabled |
| Encoder R channel A | 27 | Pull-up enabled |
| Encoder R channel B | 22 | Pull-up enabled |

These assignments avoid SPI/I2C/UART pins. Adjust as needed during physical build.

## 3. Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│  Raspberry Pi                                       │
│                                                     │
│  ┌──────────────┐  ┌─────────────┐  ┌───────────┐  │
│  │ motor_       │  │ lidar_      │  │ oakd_     │  │
│  │ controller   │  │ driver      │  │ driver    │  │
│  └──────┬───────┘  └──────┬──────┘  └─────┬─────┘  │
│         │                 │               │         │
│         └────────┬────────┴───────┬───────┘         │
│                  │   Zenoh bus    │                  │
│            ┌─────┴─────┐         │                  │
│            │ watchdog   │         │                  │
│            └─────┬─────┘         │                  │
│                  │               │                  │
└──────────────────┼───────────────┼──────────────────┘
                   │               │
              ─────┴───────────────┴───── network
                          │
              ┌───────────┴───────────┐
              │  Desktop workstation  │
              │  (Jill / CW)          │
              └───────────────────────┘
```

All inter-process communication, both local (Pi-to-Pi) and remote (Pi-to-desktop), uses the same Zenoh topics. There is no separate local IPC mechanism.

### 3.1 Process Inventory

| Process | Owns | Publishes | Subscribes |
|---|---|---|---|
| `motor_controller` | MDD10A GPIO, encoder GPIO | `body/odom`, `body/motor_state` | `body/cmd_vel`, `body/cmd_direct`, `body/emergency_stop`, `body/status` |
| `lidar_driver` | STL-19P USB | `body/lidar/scan` | — |
| `oakd_driver` | OAK-D-Lite USB | `body/oakd/depth`, `body/oakd/imu`, `body/oakd/rgb` (optional) | `body/oakd/config` (optional) |
| `watchdog` | safety authority | `body/status` | `body/heartbeat`, all `body/*` for monitoring |
| `launcher` | process lifecycle | — | — |

### 3.2 Design Principles

- Each process is a standalone Python script with its own `main()` and Zenoh session.
- No shared memory, no threading across process boundaries.
- Processes do not import each other. The Zenoh topic schema is the only coupling.
- Every process logs to stdout. The launcher captures and tags output.
- Every process handles SIGTERM gracefully: releases hardware, closes Zenoh session, exits.
- Motor output defaults to zero (stopped) on startup and on any error.

## 4. Zenoh Configuration

### 4.1 Transport

Body processes use Zenoh in **peer mode** for local communication. The desktop agent connects via Zenoh **router mode** or direct peer over TCP.

Recommended: run a `zenohd` router on the Pi that both local processes and the remote desktop connect to. This simplifies discovery and avoids multicast issues over wifi.

```
# /etc/zenoh/config.json (Pi router)
{
  "mode": "router",
  "listen": {
    "endpoints": ["tcp/0.0.0.0:7447"]
  }
}
```

Each Body process connects as a peer to `tcp/localhost:7447`. The desktop agent connects to `tcp://<pi-ip>:7447`.

### 4.2 Key Expression Namespace

All Body topics live under the `body/` prefix.

```
body/
  cmd_vel            # commanded twist velocity (from Jill)
  cmd_direct         # direct wheel velocities (from Jill, low-level override)
  heartbeat          # periodic heartbeat from Jill
  odom               # encoder-derived odometry
  motor_state        # motor driver status
  lidar/
    scan             # 2D laser scan
  oakd/
    depth            # depth frame or point cloud
    imu              # IMU readings
    rgb              # color frame (optional, high bandwidth)
    config           # runtime pipeline reconfiguration (optional)
  status             # system-level health from watchdog
  emergency_stop     # published by watchdog on safety violation
```

## 5. Message Schemas

All messages are JSON-encoded UTF-8 strings published as Zenoh values. Timestamps are Unix epoch float (seconds with microsecond precision). All spatial units are SI: meters, radians, meters/second, radians/second.

### 5.1 `body/cmd_vel` (Jill → motor_controller)

Twist command in robot body frame. Linear is forward/back, angular is rotation (positive = counterclockwise viewed from above).

```json
{
  "ts": 1713264000.123456,
  "linear": 0.2,
  "angular": 0.0,
  "timeout_ms": 500
}
```

- `linear`: m/s, positive = forward. Clamped by motor_controller to hardware limits.
- `angular`: rad/s, positive = CCW.
- `timeout_ms`: if no new `cmd_vel` arrives within this window, motor_controller sets output to zero. Safety backstop independent of watchdog.

### 5.2 `body/cmd_direct` (Jill → motor_controller)

Direct wheel velocity command. Bypasses twist-to-differential math. For calibration and testing.

```json
{
  "ts": 1713264000.123456,
  "left": 0.15,
  "right": 0.15,
  "timeout_ms": 500
}
```

- `left`, `right`: m/s at wheel surface. Sign convention: positive = forward.

### 5.3 `body/odom` (motor_controller → Jill)

Dead-reckoned pose from encoder integration. Frame: robot starting position at origin, x-forward, y-left, theta CCW from x.

```json
{
  "ts": 1713264000.123456,
  "x": 0.0,
  "y": 0.0,
  "theta": 0.0,
  "vx": 0.0,
  "vtheta": 0.0,
  "left_ticks": 0,
  "right_ticks": 0,
  "dt_ms": 20
}
```

- `x`, `y`: meters from origin.
- `theta`: radians, normalized to [-π, π].
- `vx`, `vtheta`: instantaneous velocity estimates. When encoder motion is present for the cycle (non-zero tick deltas), these are derived from wheel motion; otherwise the stub reports commanded velocities from the active `cmd_vel` / `cmd_direct`. They are zero when e-stop, command timeout, or software stall applies.
- `left_ticks`, `right_ticks`: raw cumulative encoder counts. Allows Jill to do her own integration if desired.
- `dt_ms`: time since last odom publication.

### 5.4 `body/motor_state` (motor_controller → all)

```json
{
  "ts": 1713264000.123456,
  "left_pwm": 0.0,
  "right_pwm": 0.0,
  "left_dir": "fwd",
  "right_dir": "fwd",
  "e_stop_active": false,
  "cmd_timeout_active": false,
  "stall_detected": false
}
```

- PWM values are duty cycle 0.0–1.0.
- `e_stop_active`: true if the motor controller is holding motion for e-stop (watchdog `body/status` `e_stop_active` and/or latched `body/emergency_stop`).
- `cmd_timeout_active`: true if no `cmd_vel`/`cmd_direct` received within timeout window.
- `stall_detected`: true when software stall protection has tripped (commanded PWM above threshold but encoder-indicated wheel velocity near zero for a sustained interval; see [motor_controller_spec.md](motor_controller_spec.md) §4.9). Cleared after an all-stop command (`linear`/`angular` or `left`/`right` all zero), then motion may be retried. Disabled when `motor.stall_detect_enabled` is false in config (default for stub / no encoders).

### 5.5 `body/lidar/scan` (lidar_driver → Jill)

```json
{
  "ts": 1713264000.123456,
  "angle_min": 0.0,
  "angle_max": 6.2832,
  "angle_increment": 0.01745,
  "range_min": 0.05,
  "range_max": 12.0,
  "ranges": [1.2, 1.3, 1.1, "..."],
  "intensities": [200, 195, 210, "..."],
  "scan_time_ms": 100
}
```

- Angles in radians. 0 = forward, increasing CCW.
- Invalid/out-of-range readings: `null` in the array.
- `intensities`: included if the STL-19P provides them, omit field otherwise.

### 5.6 `body/oakd/imu` (oakd_driver → Jill)

```json
{
  "ts": 1713264000.123456,
  "accel": {"x": 0.0, "y": 0.0, "z": 9.81},
  "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
  "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
}
```

- `accel`: m/s², sensor frame.
- `gyro`: rad/s.
- `mag`: if available from BNO086, omit if not configured.
- `orientation`: quaternion from BNO086 onboard fusion, if available.

### 5.7 `body/oakd/depth` (oakd_driver → Jill)

**TBD.** This is the highest-bandwidth topic and the one most likely to need a non-JSON encoding or compression strategy. Options include publishing a downsampled depth map as a flat array, publishing only a local obstacle summary, or using Zenoh's raw bytes mode with a compact binary format. Defer detailed schema until the OAK-D pipeline design is done. For the initial skeleton, publish a placeholder.

### 5.8 `body/heartbeat` (Jill → watchdog)

```json
{
  "ts": 1713264000.123456,
  "seq": 42
}
```

Jill publishes at 2 Hz minimum. Watchdog triggers safety stop if no heartbeat received for `HEARTBEAT_TIMEOUT_MS` (default 2000ms).

### 5.9 `body/status` (watchdog → Jill)

```json
{
  "ts": 1713264000.123456,
  "processes": {
    "motor_controller": "ok",
    "lidar_driver": "ok",
    "oakd_driver": "ok"
  },
  "heartbeat_ok": true,
  "e_stop_active": false,
  "uptime_s": 123.4,
  "host": {
    "ts": 1713264000.123456,
    "cpu_temp_c": 52.3,
    "core_volts": 0.8563,
    "throttled": "0x0",
    "under_voltage_now": false,
    "throttled_now": false
  }
}
```

- Process state: `"ok"`, `"missing"`, `"restarting"`.
- Published at 1 Hz.
- **`host`** (optional): Raspberry Pi–oriented telemetry when enabled in watchdog config. Includes **`cpu_temp_c`** (°C) from thermal sysfs when available; **`core_volts`** (SoC core rail ~0.8–0.95 V, **not** the 5 V input—see [body_status_host_spec.md](body_status_host_spec.md)); **`throttled`** (hex) and boolean flags from `vcgencmd get_throttled`. **5 V input problems** are indicated by **`under_voltage_*`** flags, not by `core_volts`. Omitted entirely if `watchdog.host_metrics` is false.

### 5.10 `body/emergency_stop` (watchdog → motor_controller)

```json
{
  "ts": 1713264000.123456,
  "reason": "heartbeat_timeout",
  "source": "watchdog"
}
```

Motor controller immediately sets PWM to zero on receipt. Latched: motor output stays at zero until a new `cmd_vel` or `cmd_direct` is received AND `e_stop_active` is cleared by the watchdog (which requires heartbeat recovery).

## 6. Process Specifications

### 6.1 `motor_controller`

**Responsibility:** Translate velocity commands into PWM output. Read encoders. Publish odometry.

**Hardware ownership:** MDD10A (4 GPIO pins), encoder inputs (4 GPIO pins).

**Loop structure:**
- Main loop at 50 Hz (20ms cycle).
- Each cycle: read encoders → compute odometry → check for cmd timeout → check software stall (when enabled) → compute PWM from latest command → write GPIO → publish odom and motor_state.
- Zenoh: subscribe to `body/cmd_vel`, `body/cmd_direct`, `body/emergency_stop`, and `body/status` (for `e_stop_active`). Callbacks set shared state (protected by a lock since they run on Zenoh's internal thread).

**Differential drive math:**
```
wheel_base = W  (meters, measured)
wheel_radius = R  (meters, measured)

From cmd_vel (linear v, angular ω):
  v_left  = v - (ω * W / 2)
  v_right = v + (ω * W / 2)

PWM mapping:
  pwm = clamp(velocity / max_wheel_velocity, -1.0, 1.0)
  direction = "fwd" if pwm >= 0 else "rev"
  duty_cycle = abs(pwm)
```

**Odometry integration:**
```
From encoder ticks since last cycle:
  d_left  = left_ticks * (2π * R / ticks_per_revolution)
  d_right = right_ticks * (2π * R / ticks_per_revolution)
  d_center = (d_left + d_right) / 2
  d_theta  = (d_right - d_left) / W

  x     += d_center * cos(theta + d_theta/2)
  y     += d_center * sin(theta + d_theta/2)
  theta += d_theta
```

**Configuration (`config.json` → `motor` and env overrides):**
```
wheel_base_m, wheel_radius_m   # measure during build
ticks_per_rev = 1920           # Pololu 4752: 64 CPR × 30:1 gear ratio
max_wheel_vel_ms               # determine experimentally
loop_hz = 50
cmd_timeout_ms = 500
pwm_frequency_hz = 1000
stall_detect_enabled = false   # true when encoders + stall logic are active
stall_detect_ms = 1000
```

**Encoder counting (software, initial implementation):**
Uses `lgpio` edge callbacks on both A and B channels per motor for quadrature decoding. At full speed (330 RPM), edge rate is ~10.5 kHz per motor. Python callback overhead may cause missed counts at high speeds. Acceptable for initial development at typical indoor speeds (50–150 RPM, ~1.6–4.8 kHz). Monitor odometry drift under load as a health indicator.

**Known upgrade path:** If software counting proves lossy, add a Raspberry Pi Pico as a dedicated encoder counter (and optionally PWM driver). The Pico communicates cumulative tick counts to the Pi over USB serial. The motor_controller process swaps GPIO reads for serial reads; the Zenoh interface does not change. Chassis design should reserve physical space for a Pico on the compute layer.

**Stub deliverable:** A Python script that connects to Zenoh, subscribes to `body/cmd_vel`, `body/cmd_direct`, `body/emergency_stop`, and `body/status`, prints received commands, publishes synthetic odom at 50 Hz with incrementing timestamps and zero motion (or encoder-derived when present), publishes `motor_state`. No GPIO access.

### 6.2 `lidar_driver`

**Responsibility:** Acquire scans from STL-19P, publish to Zenoh.

**Hardware ownership:** USB serial port (typically `/dev/ttyUSB0`).

**Loop structure:**
- Blocking read on serial port to accumulate a complete 360° scan.
- Parse ranges and (if available) intensities.
- Publish to `body/lidar/scan`.
- The STL-19P drives the loop rate (~10 Hz). No sleep needed.

**Configuration:**
```
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 230400           # verify against STL-19P datasheet
```

**Stub deliverable:** A Python script that connects to Zenoh, publishes a synthetic scan at 10 Hz with plausible ranges (e.g., constant 2.0m in all directions). No serial access.

### 6.3 `oakd_driver`

**Responsibility:** Configure and run DepthAI pipeline on OAK-D-Lite, publish IMU and depth to Zenoh.

**Hardware ownership:** OAK-D-Lite USB3.

**Loop structure:**
- Build DepthAI pipeline on startup (depth + IMU nodes, optionally RGB).
- Main loop calls `device.getOutputQueue().get()` for each enabled stream.
- Repackage into Zenoh messages and publish.
- IMU at ~100 Hz, depth at ~15-30 Hz depending on resolution config.

**Configuration:**
```
DEPTH_RESOLUTION = "400p"     # 640x400 stereo
DEPTH_FPS = 15
IMU_ENABLED = True
RGB_ENABLED = False           # keep off unless needed, saves bandwidth
```

**Stub deliverable:** A Python script that connects to Zenoh, publishes synthetic IMU data at 100 Hz (gravity vector only) and a placeholder depth message at 15 Hz. No DepthAI access.

### 6.4 `watchdog`

**Responsibility:** Monitor system health, enforce safety stops.

**Behavior:**
- Subscribes to `body/heartbeat`. If no message received within `HEARTBEAT_TIMEOUT_MS`, publishes `body/emergency_stop` with reason `heartbeat_timeout`.
- Periodically checks that all expected Zenoh topics are being published. If a process appears dead (no publications for `PROCESS_TIMEOUT_MS`), updates `body/status` accordingly.
- On emergency stop: publishes `body/emergency_stop`. Clears stop condition only when heartbeat resumes AND Jill sends a new `cmd_vel` or `cmd_direct` (explicit re-engagement, not automatic).
- Publishes `body/status` at 1 Hz.

**Configuration:**
```
HEARTBEAT_TIMEOUT_MS = 2000
PROCESS_TIMEOUT_MS = 5000
STATUS_PUBLISH_HZ = 1
MONITORED_TOPICS = [
    "body/odom",
    "body/lidar/scan",
    "body/oakd/imu"
]
```

**Stub deliverable:** A Python script that connects to Zenoh, subscribes to `body/heartbeat`, publishes `body/status` at 1 Hz. Implements the heartbeat timeout logic. Publishes `body/emergency_stop` when heartbeat is lost. No dependency on other Body processes running.

### 6.5 `launcher`

**Responsibility:** Start all Body processes, monitor them, restart on crash, provide orderly shutdown.

**Behavior:**
- Reads a process list from a config file or hardcoded list.
- Starts each process as a subprocess.
- Captures stdout/stderr with process-name prefixes.
- Monitors subprocess health. On unexpected exit, restarts with backoff (1s, 2s, 4s, max 30s).
- On SIGTERM/SIGINT: sends SIGTERM to all children, waits up to 5s, then SIGKILL.

**Process list (default):**
```
PROCESSES = [
    {"name": "watchdog",         "cmd": ["python3", "watchdog.py"]},
    {"name": "motor_controller", "cmd": ["python3", "motor_controller.py"]},
    {"name": "lidar_driver",     "cmd": ["python3", "lidar_driver.py"]},
    {"name": "oakd_driver",      "cmd": ["python3", "oakd_driver.py"]},
]
```

Watchdog starts first. Motor controller before sensors (so it's listening for emergency stop before anything else is live).

**Stub deliverable:** Fully functional launcher. This is not a stub — the launcher itself is hardware-independent and can be implemented completely in the first pass.

## 7. Project Structure

```
body/
  README.md
  config.json              # zenoh connection, hardware params
  launcher.py
  watchdog.py
  motor_controller.py
  lidar_driver.py
  oakd_driver.py
  lib/
    zenoh_helpers.py       # session setup, common publish/subscribe patterns
    schemas.py             # message construction/validation helpers
    diff_drive.py          # kinematic math (twist↔wheel, odometry integration)
  requirements.txt         # zenoh, gpiozero, depthai, pyserial
```

## 8. Desktop-Side Assumptions

The following assumptions apply to the desktop agent (Jill / CW) and are **out of scope** for this project but must be satisfied for the system to function:

1. **Zenoh connectivity.** The desktop must connect to the Zenoh router running on the Pi at `tcp://<pi-ip>:7447`. This may require a new Zenoh session configuration in the CW infrastructure or a new "robot" tool/skill that manages the connection.

2. **Heartbeat publication.** Jill (or a CW-side robot interface module) must publish `body/heartbeat` at ≥2 Hz whenever she expects the robot to be active. If she stops publishing, the robot stops moving. This is by design.

3. **Command publication.** Jill publishes `body/cmd_vel` to drive the robot. She is responsible for rate-limiting her own commands sensibly (10–20 Hz is reasonable). She must respect the timeout semantics — if she wants the robot to keep moving, she must keep publishing.

4. **Subscription and interpretation.** Jill subscribes to `body/odom`, `body/lidar/scan`, `body/oakd/imu`, `body/oakd/depth`, `body/status`, and `body/motor_state` as needed. She is responsible for interpreting these in her OODA loop.

5. **Coordinate frame.** The robot's coordinate frame is: x-forward, y-left, z-up, angles CCW from x-axis. Odometry origin is the robot's position at boot time. Jill must manage any world-frame transforms on her side.

6. **No guaranteed delivery.** Zenoh pub/sub is best-effort. Messages can be lost, especially over wifi. Jill should not depend on receiving every scan or odom update. The `cmd_vel` timeout mechanism handles the inverse case (lost commands → robot stops).

7. **Emergency stop acknowledgment.** When `body/emergency_stop` fires, the robot will not move until heartbeat recovers AND a new `cmd_vel` or `cmd_direct` is received. Jill should treat emergency stop as a state requiring explicit re-engagement, not automatic recovery.

## 9. Implementation Sequence

| Phase | Deliverable | Validates |
|---|---|---|
| 1 | Zenoh topic spec (this document) | Contract between Body and desktop |
| 2 | Launcher + all process stubs publishing synthetic data | Zenoh communication, process lifecycle, watchdog logic |
| 3 | Desktop-side subscriber (minimal script or CW tool) confirming receipt of all topics | End-to-end network path |
| 4 | Motor controller with real GPIO (no encoders yet) | PWM output, direction control, cmd_vel → wheel mapping |
| 5 | Encoder reading + odometry | Closed-loop velocity, dead reckoning |
| 6 | Lidar driver with real STL-19P | 2D scan data flowing to desktop |
| 7 | OAK-D pipeline (IMU first, then depth) | 9-axis IMU, depth data |
| 8 | Jill integration: robot body API as CW tool/skill | Agent-driven motion |

Phase 2 is the critical milestone. Once it's running, every subsequent phase is swapping a stub for real hardware behind the same Zenoh interface.

## 10. Open Questions

- **Encoder software counting viability:** Pololu 4752 delivers 1920 CPR at output shaft, ~10.5 kHz edge rate at full speed. Software counting via `lgpio` is the initial approach. Monitor for missed counts at speed. Pico upgrade path documented in §6.1 if needed.
- **OAK-D depth format:** JSON is impractical for full depth maps. Likely needs Zenoh raw bytes with a compact encoding. Decide during Phase 7.
- **Config file format:** JSON for now. TOML or YAML if it gets complex. Don't over-think this.
- **WiFi latency:** Zenoh over WiFi may introduce 5–50ms jitter. Acceptable for Jill's planning loop but worth measuring. If problematic, ethernet or a USB WiFi adapter with better antenna may help.
- **Pi 5 GPIO library:** `gpiozero` backed by `lgpio` is the current recommendation for Pi 5. `RPi.GPIO` does not work on Pi 5. Confirm during Phase 4.
- **Motor driver current sensing:** The MDD10A does not provide current feedback. If motor stall detection is needed later, it will require additional hardware (e.g., INA219 on I2C).
- **Stall protection:** Pololu rates continuous load limit at 10 kg·cm. Stall current is 5.5A per motor. The MDD10A can handle this (10A per channel) but prolonged stall will damage the motors. Without current sensing, software stall detection (encoder velocity drops to zero while PWM is nonzero) is the available option.
