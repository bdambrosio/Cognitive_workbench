**Recommendation: Conditional** (of 72 claims examined individually out of 118 identified).

Material gaps are addressable (dependency list, IMU firmware default, IMU/depth rates, unused `restarting` state). Safety latch, heartbeat timeout, cmd timeout, launcher lifecycle, and Phase-2 stub framing hold. The seller’s own phase table (spec:483,491) states Phase 2 — launcher + stubs — is the critical milestone; later phases swap stubs for hardware. That framing is the correct reading of current code. Desktop §8 (S64–S70) is out of scope for Body implementation and was not treated as a Body-code promise.

---

**Finding 1: requirements.txt omits gpiozero and pyserial — [delta]**

Claim (body_project_spec.md:457): `requirements.txt         # zenoh, gpiozero, depthai, pyserial`

Evidence: requirements.txt:1-3 — `eclipse-zenoh~=1.9.0`, `depthai>=2.24,<4`, `opencv-python-headless>=4.8,<5`. Lexical search of `body/` for `gpiozero`, `pyserial`, `lgpio`, `RPi.GPIO`: no imports. Structural: no tests/; motor_controller.py:1 and lidar_driver.py:1 are stubs with no GPIO/serial.

Gap: Spec names gpiozero and pyserial as project requirements; the pip file and tree do not include them. Consistent with Phase 2 stubs, inconsistent with the requirements comment as written.

**Finding 2: IMU firmware-update default — [delta]**

Claim (README.md:9): DepthAI may require `oakd.imu_enable_firmware_update: true` (default in config.json).

Evidence: config.json oakd section — `imu_enable_firmware_update` is `false` (prior read of config.json:36).

Gap: README asserts the default is true; the shipped config is false.

**Finding 3: OAK stub IMU/depth rates — [delta]**

Claim (body_project_spec.md:391): stub publishes synthetic IMU at 100 Hz and placeholder depth at 15 Hz.

Evidence: oakd_driver.py:635 `interval_s = float(oakd_cfg.get("imu_aggregate_interval_s", 1.0))`; config.json `imu_aggregate_interval_s`: 1.0; oakd_driver.py:3-4 publishes means over that interval; config.json:21 `depth_fps`: 10; oakd_driver.py:636 default `depth_fps` 15 if unset, but shipped config is 10.

Gap: Stub IMU is ~1 Hz aggregated means, not 100 Hz; shipped depth is 10 Hz, not 15 Hz.

**Finding 4: Hardware OAK rates — [partial]**

Claim (body_project_spec.md:381): IMU at ~100 Hz, depth at ~15-30 Hz.

Evidence: oakd_driver.py:3-4,278-289 — high-rate packets are aggregated and published at `imu_aggregate_interval_s` (1.0 s). config.json:21 `depth_fps`: 10. Spec DEPTH_FPS = 15 at body_project_spec.md:386.

Gap: Wire IMU is 1 Hz means, not ~100 Hz samples; depth config is 10 fps, below the 15–30 Hz band.

**Finding 5: status never uses `restarting` — [partial]**

Claim (body_project_spec.md:277): Process state: `"ok"`, `"missing"`, `"restarting"`.

Evidence: body/watchdog.py:89-93 assigns only `"missing"` or `"ok"`.

Gap: `restarting` is specified and never emitted. Launcher restarts exist (launcher.py:16-21,49-52) but are not reflected in status.

**Finding 6: motor/lidar hardware ownership vs current stubs — [real, operational caveat]**

Claim (body_project_spec.md:349, S47): motor stub, no GPIO; (lidar_driver.py:1 / S49) lidar stub ~10 Hz no serial. S16/S17 name hardware ownership. S71 (body_project_spec.md:483,491): Phase 2 is launcher + stubs; later phases swap hardware.

Evidence: motor_controller.py:1 stub; lidar_driver.py:1 stub; config.json motor wheel_base_m/wheel_radius_m/max_wheel_vel_ms 0.0; ticks_per_rev 1920, loop_hz 50, cmd_timeout_ms 500 (S45 holds in config).

Gap: None against the Phase-2 claim. R1 (README.md:3) is a product description of onboard software for a differential-drive chassis; it is not a claim that GPIO motors are live today. Buyer should not price a driving chassis.

**Finding 7: heartbeat e-stop path — [real]**

Claim (README.md R30; body_project_spec.md:259,405, S39/S52/S56): HEARTBEAT_TIMEOUT_MS = 2000; watchdog publishes body/emergency_stop reason heartbeat_timeout.

Evidence: body/watchdog.py:23-29 hb_timeout_ms default 2000; :75-80,104-105 publish emergency_stop on first hb timeout; last_hb init 0.0 so hb_ok is false until first heartbeat.

Gap: None.

**Finding 8: e-stop latch and re-engage — [real]**

Claim (body_project_spec.md ~290-310,400; S41/S54; README R42): latched until new cmd_vel AND heartbeat recovered.

Evidence: watchdog.py:42-47 clears e_stop_active only if hb_ok; motor_controller.py:44-51,59-74 e_stop_latched / awaiting_cmd_vel_after_clear.

Gap: None on the stated protocol.

**Finding 9: cmd_vel timeout zeros motors — [real]**

Claim (S32/R41): timeout_ms default 500; motors zero if no new cmd_vel.

Evidence: config.json:11 `cmd_timeout_ms`: 500; motor_controller.py:100-108 PWM zero on e_stop/stale; teleop.py:25 default 500.

Gap: None. PWM is computed in software only (stub).

**Finding 10: launcher lifecycle — [real]**

Claim (R26–R29, S58–S61): order watchdog→motor→lidar→oakd; log prefixes; SIGTERM wait then SIGKILL; backoff cap 30 s; launcher not a stub.

Evidence: launcher.py:16-21 PROCESSES; :43 prefix; :49-52 `min(30, 2**(n-1))`; :116-122 SIGTERM wait 5s then kill; PYTHONPATH :57-58.

Gap: None. Spec “backoff 1/2/4” matches 2**(n-1) for n=1,2,3.

**Finding 11: Zenoh router/connect — [real]**

Claim (R18–R20, R25, S27–S28): ZENOH_CONNECT override; Pi connect tcp/127.0.0.1:7447; deploy router 0.0.0.0:7447.

Evidence: zenoh_helpers.py:23-26 ZENOH_CONNECT; config.json:3 `tcp/127.0.0.1:7447`; deploy/zenohd-router.json listen 0.0.0.0:7447.

Gap: None from files. Live router process was not observed (limited assurance).

**Finding 12: teleop defaults — [real]**

Claim (R31–R38): standalone teleop; heartbeat-hz 2; cmd-vel-hz 20; timeout-ms 500.

Evidence: teleop.py:1-4,23-25; periods teleop.py:56-57.

Gap: None.

**Finding 13: schemas odom/motor_state — [real]**

Claim (S34, S35): odom fields and theta wrap; motor_state PWM, e_stop_active, cmd_timeout_active.

Evidence: schemas.py:22-45 atan2 wrap; :48-65 motor_state fields.

Gap: None.

**Finding 14: lgpio quadrature / stall detect — [non-delta] / remaining**

S46 (body_project_spec.md:344-347) and S74 (500-501) describe future/initial hardware approaches and an open stall option. Current motor is the S47 stub. No encoder-vel stall code. Not a broken current-feature promise; not counted as [delta].

**Finding 15: S63/S46 vs Phase 2 — consistency**

S63 is still a [delta] (Finding 1) because the comment names libraries the file does not list. S46 is not current Phase-2 deliverable.

---

**Coverage.** 118 claims frozen (README 44, spec 74). 72 examined with file:line evidence. Unchecked remainder: hardware inventory figures not measurable from a repo (S9–S14 motor stall kg·cm, Pi 5 8GB, OAK BNO086 presence), Linux udev (R7), Kickstarter IMU epidemiology (R8,R10), live zenohd install path (R22–R24), SIGTERM hardware-release on real GPIO (S25 — no GPIO), WiFi latency open question (S73), desktop §8 S64–S70 (explicitly out of scope for Body). Those are low-risk relative to a Phase-2 software acquisition, or unverifiable without hardware. No tests/ directory.

**Ask the seller before closing.** (1) Confirm the deal is Phase-2 software, not a driving chassis. (2) Whether gpiozero/pyserial/lgpio will be added before Phase 4 or the spec comment will be corrected. (3) Intended IMU publish rate: 100 Hz samples vs 1 Hz means. (4) Whether `imu_enable_firmware_update` should default true. (5) Whether `restarting` will be implemented. (6) As-of date of this tree vs any uncommitted hardware work.

=== LIMITATIONS ===
Materials: repository /home/bruce/projects/Body/Body (README.md, body_project_spec.md, body/, config.json, requirements.txt, deploy/), as-of 2026-08-25. Seller was not consulted and has not confirmed this reading of their claims. Limited assurance (§1a): 72 of 118 claims examined; no hardware, no pen-test, no live robot.

=== GAP MAP ===

**Body** — Raspberry Pi differential-drive stack (Zenoh JSON processes); current tree is Phase-2 launcher + stubs.

**Conditional** — 72 of 118 claims examined.

- Safety heartbeat / e-stop latch / 500 ms cmd timeout hold in code.
- Launcher order, backoff cap 30 s, SIGTERM+kill hold.
- requirements.txt does not list gpiozero or pyserial as the spec comment does.
- README says IMU firmware-update default true; config.json is false.
- IMU/depth rates: spec 100 Hz / 15–30 Hz vs ~1 Hz means and depth_fps 10; status never emits `restarting`.
- Motor and lidar are stubs (also claimed); do not price a driving chassis.

Coverage: 72/118 checked; remainder mostly hardware inventory, desktop §8, or live-router ops — low-risk if the purchase is Phase-2 software. Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice