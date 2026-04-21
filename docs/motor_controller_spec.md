# motor_controller.py — Informal Specification

**Project:** Body (robot body software stack)
**Date:** 2026-04-19
**Hardware:** Cytron MDD10A Rev2.0, 2x Pololu #4752 30:1 37D gearmotor w/ 64 CPR encoder, Raspberry Pi 5

---

## 1. MDD10A Pinout

The MDD10A has a 5-pin input header and a 6-position terminal block.

### Input Header (5-pin, 0.1" pitch, left to right facing the board)

| Pin | Name | Description |
|-----|------|-------------|
| 1   | GND  | Ground (shared with Pi) |
| 2   | PWM2 | PWM input, motor 2 speed. NOT RC PWM — raw TTL PWM. |
| 3   | DIR2 | Direction input, motor 2. LOW = Output A high; HIGH = Output B high |
| 4   | PWM1 | PWM input, motor 1 speed |
| 5   | DIR1 | Direction input, motor 1. Same convention as DIR2 |

Logic levels: 3.3V and 5V compatible. Pi 5 GPIO (3.3V) works directly, no level shifter needed.

### Control Truth Table (per channel, sign-magnitude mode)

| PWM | DIR | Output A | Output B | Motor |
|-----|-----|----------|----------|-------|
| LOW | X   | LOW      | LOW      | Brake (regenerative) |
| HIGH| LOW | HIGH     | LOW      | Forward (A→B) |
| HIGH| HIGH| LOW      | HIGH     | Reverse (B→A) |

PWM frequency: up to 20 kHz. We will use 1 kHz initially (safe, audible but functional; increase to 16–20 kHz later to eliminate audible whine).

### Power Terminal Block (6-position screw terminal)

| Pos | Name | Description |
|-----|------|-------------|
| 1   | M1B  | Motor 1 output B |
| 2   | M1A  | Motor 1 output A |
| 3   | PWR+ | Battery positive (12V) |
| 4   | PWR- | Battery negative / ground |
| 5   | M2A  | Motor 2 output A |
| 6   | M2B  | Motor 2 output B |

**Critical:** Use battery, not bench supply. The MDD10A's regenerative braking produces back-EMF that trips switching power supply protection circuits.

---

## 2. Pololu #4752 Motor/Encoder Pinout

Each motor has a 6-wire harness terminated in a 1×6 female header (0.1" pitch).

| Wire Color | Function | Connect To |
|------------|----------|------------|
| Red        | Motor power (+) | MDD10A motor output A |
| Black      | Motor power (-) | MDD10A motor output B |
| Green      | Encoder GND | Pi GND (shared ground) |
| Blue       | Encoder Vcc | Pi 3.3V (acceptable range: 3.5–20V per datasheet, but 3.3V works — see note) |
| Yellow     | Encoder output A | Pi GPIO input |
| White      | Encoder output B | Pi GPIO input |

**Encoder Vcc note:** The Hall sensor datasheet says 3.5V minimum. In practice, 3.3V from the Pi works reliably with these Pololu encoders — this is widely reported. If edges look noisy on a scope, move to 5V Vcc and add a voltage divider or level shifter on the A/B outputs. Start with 3.3V.

**Encoder output:** Open-drain with 10kΩ pull-up to Vcc on the encoder board. Outputs swing 0V to Vcc. At 3.3V Vcc, directly compatible with Pi GPIO. At 5V Vcc, you need level shifting.

---

## 3. Raspberry Pi 5 GPIO Assignments

Pi 5 uses the RP1 I/O controller. GPIO library: `lgpio` (the only fully supported option on Pi 5; `RPi.GPIO` does not work).

### Pin Assignments

| Function | BCM GPIO | Physical Pin | Notes |
|----------|----------|-------------|-------|
| Motor L PWM | 12 | 32 | Hardware PWM0 (channel 0) |
| Motor L DIR | 5 | 29 | Digital output |
| Motor R PWM | 13 | 33 | Hardware PWM0 (channel 1) |
| Motor R DIR | 6 | 31 | Digital output |
| Encoder L ch A | 23 | 16 | Input, pull-up, edge callbacks |
| Encoder L ch B | 24 | 18 | Input, pull-up, edge callbacks |
| Encoder R ch A | 27 | 13 | Input, pull-up, edge callbacks |
| Encoder R ch B | 22 | 15 | Input, pull-up, edge callbacks |

**Rationale:** BCM 12 and 13 are the two hardware PWM channels on the Pi 5 (PWM0_CH0 and PWM0_CH1). Using hardware PWM gives clean, jitter-free output independent of Python timing. All other pins avoid SPI0, I2C1, and UART0 so those buses remain available for future peripherals.

### Wiring Summary

```
Pi 5                    MDD10A Input Header
────                    ───────────────────
GND  ──────────────────── Pin 1 (GND)
GPIO 12 (PWM) ─────────── Pin 4 (PWM1) ── left motor
GPIO 5  (DIR) ─────────── Pin 5 (DIR1) ── left motor
GPIO 13 (PWM) ─────────── Pin 2 (PWM2) ── right motor
GPIO 6  (DIR) ─────────── Pin 3 (DIR2) ── right motor

Pi 5                    Left Motor Encoder
────                    ──────────────────
3.3V ──────────────────── Blue  (Vcc)
GND  ──────────────────── Green (GND)
GPIO 23 ───────────────── Yellow (ch A)
GPIO 24 ───────────────── White  (ch B)

Pi 5                    Right Motor Encoder
────                    ───────────────────
3.3V ──────────────────── Blue  (Vcc)
GND  ──────────────────── Green (GND)
GPIO 27 ───────────────── Yellow (ch A)
GPIO 22 ───────────────── White  (ch B)

Motor power wires (Red/Black) go to MDD10A terminal block,
NOT to the Pi. Pi and MDD10A share ground only.
```

---

## 4. motor_controller.py — Functional Spec

### 4.1 Responsibility

Sole owner of MDD10A GPIO (PWM + DIR) and encoder GPIO. Translates velocity commands from Zenoh into PWM output. Reads encoders. Publishes odometry and motor state to Zenoh. Enforces command timeouts and emergency stops.

### 4.2 Zenoh Interface

**Subscribes to:**
- `body/cmd_vel` — twist commands (linear m/s, angular rad/s)
- `body/cmd_direct` — direct wheel velocities (left m/s, right m/s)
- `body/emergency_stop` — immediate halt

**Publishes to:**
- `body/odom` — dead-reckoned pose + raw encoder ticks, at 50 Hz
- `body/motor_state` — PWM duty cycles, directions, timeout/estop flags, at 50 Hz

See [body_project_spec.md](body_project_spec.md) for message schemas.

### 4.3 Main Loop (50 Hz / 20ms cycle)

```
every 20ms:
    1. Read encoder counters (ticks since last cycle)
    2. Compute instantaneous wheel velocities from ticks + dt
    3. Integrate odometry (x, y, theta)
    4. Check command timeout (no cmd_vel/cmd_direct within timeout_ms → stop)
    5. Check emergency_stop flag
    6. If stopped: set PWM to 0 on both channels
       Else: compute PWM from latest velocity command
    7. Write GPIO (PWM duty cycle + DIR pin)
    8. Publish body/odom
    9. Publish body/motor_state
```

### 4.4 Differential Drive Math

**cmd_vel → wheel velocities:**
```
v_left  = linear - (angular * WHEEL_BASE_M / 2)
v_right = linear + (angular * WHEEL_BASE_M / 2)
```

**wheel velocity → PWM:**
```
duty = clamp(abs(velocity) / MAX_WHEEL_VEL_MS, 0.0, 1.0)
direction = "fwd" if velocity >= 0 else "rev"
```

**encoder ticks → odometry:**
```
d_left  = delta_ticks_left  * (2π * WHEEL_RADIUS_M / TICKS_PER_REV)
d_right = delta_ticks_right * (2π * WHEEL_RADIUS_M / TICKS_PER_REV)
d_center = (d_left + d_right) / 2
d_theta  = (d_right - d_left) / WHEEL_BASE_M

x     += d_center * cos(theta + d_theta / 2)
y     += d_center * sin(theta + d_theta / 2)
theta += d_theta
theta  = normalize_angle(theta)   # keep in [-π, π]
```

### 4.5 Encoder Reading Strategy

**Initial implementation: software quadrature decoding via lgpio edge callbacks.**

Register `BOTH_EDGES` callbacks on all four encoder GPIO pins. Each callback updates a shared counter (one counter per motor). Use a simple state machine per motor:

```
state = (A_level, B_level)
On any edge:
    new_state = (read_A, read_B)
    direction = QUADRATURE_TABLE[(old_state, new_state)]
    if direction == +1: ticks += 1
    elif direction == -1: ticks -= 1
    old_state = new_state
```

Standard quadrature lookup table handles all 16 transitions (4 valid forward, 4 valid reverse, 4 no-change, 4 error/missed).

**Counters are accessed from the main loop.** The lgpio callback runs on a background thread. Protect counters with a simple threading.Lock or use atomic-style read (read + zero in main loop while holding lock briefly).

**Performance risk:** At 330 RPM, edge rate is ~10.5 kHz per motor. Python callback overhead may cause missed counts. Monitor for drift. Pico upgrade path documented in Body spec if this proves problematic.

### 4.6 GPIO Setup

```python
import lgpio

h = lgpio.gpiochip_open(0)

# Motor PWM outputs (hardware PWM on BCM 12, 13)
# lgpio hardware PWM: lgpio.tx_pwm(handle, gpio, freq_hz, duty_percent)
lgpio.tx_pwm(h, 12, PWM_FREQ_HZ, 0)   # left motor, start stopped
lgpio.tx_pwm(h, 13, PWM_FREQ_HZ, 0)   # right motor, start stopped

# Motor DIR outputs
lgpio.gpio_claim_output(h, 5, 0)   # left DIR, default LOW (forward)
lgpio.gpio_claim_output(h, 6, 0)   # right DIR, default LOW (forward)

# Encoder inputs with pull-ups
for pin in [23, 24, 27, 22]:
    lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)

# Encoder edge callbacks
lgpio.callback(h, 23, lgpio.BOTH_EDGES, left_encoder_callback)
lgpio.callback(h, 24, lgpio.BOTH_EDGES, left_encoder_callback)
lgpio.callback(h, 27, lgpio.BOTH_EDGES, right_encoder_callback)
lgpio.callback(h, 22, lgpio.BOTH_EDGES, right_encoder_callback)
```

### 4.7 Command Timeout

Each `cmd_vel` or `cmd_direct` message carries a `timeout_ms` field (default 500ms). If no new command is received within this window, motor output is set to zero. This is independent of the watchdog process — it's a local safety measure.

### 4.8 Emergency Stop

On receiving `body/emergency_stop`:
- Immediately set both PWM channels to 0% duty
- Set `e_stop_active = True`
- Remain stopped until:
  1. No `emergency_stop` has been received for `ESTOP_CLEAR_MS` (default 2000ms), AND
  2. A new `cmd_vel` or `cmd_direct` is received (explicit re-engagement)

### 4.9 Software Stall Detection

If PWM duty > 10% but encoder velocity reads zero for `STALL_DETECT_MS` (default 1000ms):
- Set PWM to 0 (protect motors from overheating)
- Publish `motor_state` with a `stall_detected` flag
- Resume normal operation on next `cmd_vel` (allow retry)

Not a substitute for current sensing, but prevents prolonged stalls from burning out the motors (Pololu rates 5.5A stall, MDD10A can deliver 10A).

### 4.10 Startup and Shutdown

**Startup:**
1. Open lgpio handle
2. Configure all GPIO (PWM at 0%, DIR LOW, encoder inputs with callbacks)
3. Open Zenoh session, subscribe to cmd topics
4. Enter main loop

**Shutdown (SIGTERM):**
1. Set both PWM to 0%
2. Close Zenoh session
3. Release all GPIO
4. Close lgpio handle
5. Exit

Motors are always safe-off by default. If the process crashes, lgpio releases the pins and PWM output stops.

### 4.11 Configuration Constants

```python
# Physical measurements — fill in during build
WHEEL_BASE_M = 0.0           # center-to-center wheel distance (meters)
WHEEL_RADIUS_M = 0.0         # wheel radius (meters)

# Pololu #4752 encoder
TICKS_PER_REV = 1920          # 64 CPR motor × 30:1 gear ratio

# Motor performance (Pololu #4752 at 12V)
MAX_WHEEL_VEL_MS = 0.0       # max wheel surface velocity (m/s)
                              # = (330 RPM / 60) × 2π × WHEEL_RADIUS_M
                              # fill in once wheel radius is measured

# Control loop
LOOP_HZ = 50                 # 20ms cycle
PWM_FREQ_HZ = 1000           # increase to 16000+ to eliminate whine
CMD_TIMEOUT_MS = 500          # stop if no command received
ESTOP_CLEAR_MS = 2000         # time after last e-stop to allow clearing
STALL_DETECT_MS = 1000        # PWM on but no encoder motion → stall

# GPIO assignments (BCM numbering)
MOTOR_L_PWM = 12
MOTOR_L_DIR = 5
MOTOR_R_PWM = 13
MOTOR_R_DIR = 6
ENC_L_A = 23
ENC_L_B = 24
ENC_R_A = 27
ENC_R_B = 22

# Zenoh
ZENOH_CONNECT = "tcp/localhost:7447"
```

### 4.12 Dependencies

- `lgpio` — GPIO access on Pi 5 (pre-installed on Raspberry Pi OS)
- `zenoh` — pip install eclipse-zenoh
- Python 3.11+ standard library (threading, signal, time, json, math)

No other dependencies. No ROS. No frameworks. One file.

---

## 5. Stub vs. Real Implementation

**Phase 2 (stub):** No GPIO access. Subscribes to cmd topics, logs received commands to stdout, publishes synthetic odom (zero motion, incrementing timestamps) and motor_state at 50 Hz.

**Phase 4 (real PWM, no encoders):** GPIO active for PWM and DIR. Encoders not read. Open-loop velocity from cmd_vel → PWM mapping. Odom publishes zeros. Verify motors spin in correct directions, tune DIR polarity if needed.

**Phase 5 (full):** Encoder callbacks active. Closed-loop odometry. Stall detection enabled. Calibrate MAX_WHEEL_VEL_MS experimentally by commanding full PWM and reading encoder velocity.

---

## 6. What This Spec Does NOT Cover

- PID velocity control (cmd_vel asks for m/s, we map linearly to PWM — no feedback loop yet). Add later if odometry shows the linear mapping is too coarse.
- Acceleration limiting / ramp-up. Currently jumps to commanded velocity instantly. Add if mechanical shock is a problem.
- Pico-based encoder counting (upgrade path documented in Body spec).
- Zenoh session configuration details (see Body spec §4).
