Claim sources (named by the engagement): README.md and body_project_spec.md. Every other file is evidence and was not used to add claims.

=== CLAIM SURFACE ===
164 claims

Per-document: README.md 45; body_project_spec.md 119.

README.md:3 — Independent Python processes on a Raspberry Pi communicate over Zenoh using JSON messages.
README.md:3 — The contract with the desktop agent is defined in body_project_spec.md.
README.md:7 — Requires Python 3.11+.
README.md:8 — Requires eclipse-zenoh and, for oakd_driver, depthai.
README.md:8 — Linux udev rules for Movidius (03e7) are required to open the OAK from a non-root user.
README.md:9 — Retail OAK-D-Lite units usually include a BNO IMU.
README.md:9 — DepthAI may require oakd.imu_enable_firmware_update: true (default in config.json) on first use.
README.md:9 — Some Kickstarter OAK-D-Lite boards have no IMU.
README.md:9 — Setting imu_hardware_present: false runs oakd_driver with synthetic body/oakd/imu so the launcher does not crash.
README.md:10 — A Zenoh router (zenohd) is reachable by every Body process and every client.
README.md:10 — On the robot, the router runs on the Pi and listens on TCP 7447.
README.md:14 — Install is from the repository root that contains config.json and the body/ package.
README.md:23 — The launcher sets PYTHONPATH for child processes.
README.md:29 — config.json holds Zenoh connect_endpoints and motor/lidar/oakd/watchdog tuning.
README.md:30 — ZENOH_CONNECT optionally replaces zenoh.connect_endpoints for all processes.
README.md:32 — The Pi router listens on 0.0.0.0:7447 so LAN peers can connect.
README.md:41 — Processes on the Pi connect to tcp/127.0.0.1:7447 (default in config.json).
README.md:41 — A laptop running teleop uses tcp/<pi-ip>:7447 via ZENOH_CONNECT or edited connect_endpoints.
README.md:45 — Body expects a router already running before body.launcher or teleop.
README.md:47 — zenohd is not installed by pip or the .venv.
README.md:47 — The Python package eclipse-zenoh is only the client library.
README.md:60 — deploy/zenohd-router.json listens on TCP 0.0.0.0:7447.
README.md:97 — body.launcher starts motor, lidar, oakd, and watchdog processes.
README.md:98 — Without heartbeats, the watchdog treats the robot as not under command and can trigger body/emergency_stop.
README.md:110 — Startup order is watchdog → motor_controller → lidar_driver → oakd_driver.
README.md:110 — Logs are prefixed by process name.
README.md:112 — SIGTERM/Ctrl+C to the launcher sends SIGTERM to children, waits, then SIGKILL if needed.
README.md:114 — If a child exits unexpectedly, the launcher restarts it with exponential backoff capped at 30 s.
README.md:118 — Until something publishes body/heartbeat, the watchdog may emit body/emergency_stop (heartbeat_timeout).
README.md:122 — Standalone mode runs Body on the Pi with the repo teleop client; Cognitive Workbench does not need to run.
README.md:137 — Default config.json uses tcp/127.0.0.1:7447.
README.md:156 — Teleop vel latches body/cmd_vel as linear m/s and angular rad/s (CCW positive); angular defaults to 0.
README.md:157 — Teleop stop latches zero velocity.
README.md:158 — Teleop status prints last body/status JSON if any.
README.md:160 — Exiting teleop stops heartbeat and cmd_vel; the robot may e-stop if nothing else publishes heartbeat.
README.md:162 — Teleop --heartbeat-hz defaults to 2.
README.md:162 — Teleop --cmd-vel-hz defaults to 20.
README.md:162 — Teleop --timeout-ms defaults to 500.
README.md:164 — Running teleop and another publisher both commanding body/cmd_vel makes the motor side see interleaved commands.
README.md:170 — A desktop agent should publish body/heartbeat at ≥ 2 Hz while the robot is expected to accept motion.
README.md:171 — A desktop agent should publish body/cmd_vel often enough to satisfy timeout_ms (default 500 ms in the spec) while moving or holding speed.
README.md:174 — After a heartbeat fault, recovery follows §5.10 in body_project_spec.md as implemented on the Pi.
README.md:178 — With the stack running, body/odom, body/lidar/scan, body/oakd/imu, and body/status carry traffic; body/heartbeat and body/cmd_vel when teleop or Jill is active.
README.md:182 — body/ is the package: launcher, drivers, teleop, lib/ (zenoh_helpers, schemas, diff_drive).
README.md:183 — deploy/ holds optional ops files (e.g. zenohd-router.json).

body_project_spec.md:3 — Spec version is 0.2 draft.
body_project_spec.md:4 — Spec date is 2026-04-16.
body_project_spec.md:6 — Hardware target is Raspberry Pi 5 (assumed), differential drive chassis.
body_project_spec.md:7 — Language is Python 3.11+.
body_project_spec.md:8 — Transport is Zenoh (pyzenoh).
body_project_spec.md:14 — Body is the onboard software stack for a differential-drive chassis providing sensor acquisition, motor control, and safety supervision as independent processes linked by Zenoh.
body_project_spec.md:14 — The desktop agent connects to the same Zenoh network and interacts with Body exclusively through published topics.
body_project_spec.md:14 — Body has no knowledge of Jill's internals; Jill has no knowledge of Body's process structure.
body_project_spec.md:14 — The Zenoh topic schema in this document is the contract between them.
body_project_spec.md:20 — Cytron MDD10A is a dual 10A motor driver on GPIO (2× PWM + 2× DIR); Channel A = left, Channel B = right.
body_project_spec.md:21 — Two Pololu #4752 30:1 gearmotors with encoders: 64 CPR motor shaft, 1920 CPR output shaft, 330 RPM no-load @ 12V, stall 14 kg·cm / 5.5A.
body_project_spec.md:22 — LDROBOT STL-19P lidar is USB serial, 360° 2D scan, ~10 Hz typical.
body_project_spec.md:23 — OAK-D-Lite is USB3 (DepthAI) with stereo depth, RGB, and onboard IMU (BNO086).
body_project_spec.md:24 — Raspberry Pi 5 is 4-core, 8GB assumed, and runs all Body processes.
body_project_spec.md:30 — Motor A PWM is BCM 12 (hardware PWM channel 0).
body_project_spec.md:31 — Motor A DIR is BCM 5; HIGH = forward, LOW = reverse.
body_project_spec.md:32 — Motor B PWM is BCM 13 (hardware PWM channel 1).
body_project_spec.md:33 — Motor B DIR is BCM 6 with the same convention.
body_project_spec.md:34 — Encoder L channel A is BCM 23 with pull-up enabled.
body_project_spec.md:35 — Encoder L channel B is BCM 24 with pull-up enabled.
body_project_spec.md:36 — Encoder R channel A is BCM 27 with pull-up enabled.
body_project_spec.md:37 — Encoder R channel B is BCM 22 with pull-up enabled.
body_project_spec.md:39 — These assignments avoid SPI/I2C/UART pins.
body_project_spec.md:68 — All inter-process communication, local and remote, uses the same Zenoh topics; there is no separate local IPC mechanism.
body_project_spec.md:74 — motor_controller owns MDD10A GPIO and encoder GPIO, publishes body/odom and body/motor_state, subscribes to body/cmd_vel and body/cmd_direct.
body_project_spec.md:75 — lidar_driver owns STL-19P USB and publishes body/lidar/scan.
body_project_spec.md:76 — oakd_driver owns OAK-D-Lite USB, publishes body/oakd/depth, body/oakd/imu, and optionally body/oakd/rgb, and optionally subscribes to body/oakd/config.
body_project_spec.md:77 — watchdog is the safety authority, publishes body/status, and subscribes to body/heartbeat and all body/* for monitoring.
body_project_spec.md:78 — launcher owns process lifecycle and publishes/subscribes nothing.
body_project_spec.md:82 — Each process is a standalone Python script with its own main() and Zenoh session.
body_project_spec.md:83 — No shared memory, and no threading across process boundaries.
body_project_spec.md:84 — Processes do not import each other; the Zenoh topic schema is the only coupling.
body_project_spec.md:85 — Every process logs to stdout; the launcher captures and tags output.
body_project_spec.md:86 — Every process handles SIGTERM by releasing hardware, closing the Zenoh session, and exiting.
body_project_spec.md:87 — Motor output defaults to zero (stopped) on startup and on any error.
body_project_spec.md:93 — Body processes use Zenoh in peer mode for local communication.
body_project_spec.md:93 — The desktop agent connects via Zenoh router mode or direct peer over TCP.
body_project_spec.md:95 — Recommended: run a zenohd router on the Pi that local processes and the remote desktop connect to.
body_project_spec.md:107 — Each Body process connects as a peer to tcp/localhost:7447.
body_project_spec.md:107 — The desktop agent connects to tcp://<pi-ip>:7447.
body_project_spec.md:111 — All Body topics live under the body/ prefix.
body_project_spec.md:133 — All messages are JSON-encoded UTF-8 strings published as Zenoh values.
body_project_spec.md:133 — Timestamps are Unix epoch float (seconds with microsecond precision).
body_project_spec.md:133 — All spatial units are SI: meters, radians, meters/second, radians/second.
body_project_spec.md:137 — body/cmd_vel is a twist in the robot body frame; linear is forward/back, angular is rotation (positive = CCW from above).
body_project_spec.md:148 — linear is m/s, positive = forward, clamped by motor_controller to hardware limits.
body_project_spec.md:149 — angular is rad/s, positive = CCW.
body_project_spec.md:150 — If no new cmd_vel arrives within timeout_ms, motor_controller sets output to zero; this safety backstop is independent of the watchdog.
body_project_spec.md:154 — body/cmd_direct is a direct wheel-velocity command that bypasses twist-to-differential math.
body_project_spec.md:165 — cmd_direct left and right are m/s at the wheel surface; positive = forward.
body_project_spec.md:169 — body/odom is dead-reckoned pose from encoder integration; frame is start-at-origin, x-forward, y-left, theta CCW from x.
body_project_spec.md:185 — odom x, y are meters from origin.
body_project_spec.md:186 — odom theta is radians, normalized to [-π, π].
body_project_spec.md:187 — odom vx, vtheta are instantaneous velocity estimates.
body_project_spec.md:188 — odom left_ticks, right_ticks are raw cumulative encoder counts.
body_project_spec.md:189 — odom dt_ms is time since last odom publication.
body_project_spec.md:205 — motor_state PWM values are duty cycle 0.0–1.0.
body_project_spec.md:206 — e_stop_active is true if the watchdog has triggered emergency stop.
body_project_spec.md:207 — cmd_timeout_active is true if no cmd_vel/cmd_direct was received within the timeout window.
body_project_spec.md:225 — Lidar angles are radians; 0 = forward, increasing CCW.
body_project_spec.md:226 — Invalid/out-of-range lidar readings are null in the array.
body_project_spec.md:227 — intensities is included if the STL-19P provides them, otherwise omitted.
body_project_spec.md:248 — body/oakd/depth schema is TBD; the initial skeleton publishes a placeholder.
body_project_spec.md:259 — Jill publishes heartbeat at 2 Hz minimum.
body_project_spec.md:259 — Watchdog triggers a safety stop if no heartbeat is received for HEARTBEAT_TIMEOUT_MS (default 2000 ms).
body_project_spec.md:277 — Process state in status is "ok", "missing", or "restarting".
body_project_spec.md:278 — body/status is published at 1 Hz.
body_project_spec.md:290 — On emergency_stop the motor controller immediately sets PWM to zero; output stays zero until a new cmd_vel is received AND e_stop_active is cleared by the watchdog (which requires heartbeat recovery).
body_project_spec.md:296 — motor_controller translates velocity commands into PWM, reads encoders, and publishes odometry.
body_project_spec.md:298 — motor_controller owns MDD10A (4 GPIO) and encoder inputs (4 GPIO).
body_project_spec.md:301 — motor_controller main loop runs at 50 Hz (20 ms cycle).
body_project_spec.md:302 — Each cycle: read encoders → compute odometry → check cmd timeout → compute PWM from latest command → write GPIO → publish odom and motor_state.
body_project_spec.md:303 — Zenoh callbacks for cmd_vel, cmd_direct, and emergency_stop set shared state protected by a lock.
body_project_spec.md:337 — TICKS_PER_REV is 1920 (Pololu 4752: 64 CPR × 30:1).
body_project_spec.md:339 — LOOP_HZ is 50.
body_project_spec.md:340 — CMD_TIMEOUT_MS is 500.
body_project_spec.md:341 — PWM_FREQUENCY_HZ is 1000.
body_project_spec.md:345 — Encoder counting uses lgpio edge callbacks on both A and B per motor; at full speed (330 RPM) edge rate is ~10.5 kHz per motor.
body_project_spec.md:345 — Python callback overhead may miss counts at high speeds; indoor 50–150 RPM is ~1.6–4.8 kHz.
body_project_spec.md:347 — If software counting is lossy, a Pico can count encoders over USB serial without changing the Zenoh interface.
body_project_spec.md:349 — Motor stub connects to Zenoh, subscribes to cmd_vel and cmd_direct, publishes synthetic odom at 50 Hz with zero motion and motor_state, with no GPIO access.
body_project_spec.md:353 — lidar_driver acquires scans from the STL-19P and publishes to Zenoh.
body_project_spec.md:355 — Lidar hardware ownership is a USB serial port, typically /dev/ttyUSB0.
body_project_spec.md:358 — Lidar loop is a blocking serial read to accumulate a complete 360° scan.
body_project_spec.md:361 — The STL-19P drives the loop rate (~10 Hz); no sleep is needed.
body_project_spec.md:365 — Lidar SERIAL_PORT is /dev/ttyUSB0.
body_project_spec.md:366 — Lidar BAUD_RATE is 230400.
body_project_spec.md:369 — Lidar stub publishes a synthetic scan at 10 Hz (e.g. constant 2.0 m) with no serial access.
body_project_spec.md:373 — oakd_driver configures and runs a DepthAI pipeline on OAK-D-Lite and publishes IMU and depth to Zenoh.
body_project_spec.md:375 — oakd_driver owns OAK-D-Lite USB3.
body_project_spec.md:378 — Pipeline on startup includes depth + IMU nodes, optionally RGB.
body_project_spec.md:381 — IMU is published at ~100 Hz; depth at ~15–30 Hz depending on resolution.
body_project_spec.md:385 — DEPTH_RESOLUTION is "400p" (640×400 stereo).
body_project_spec.md:386 — DEPTH_FPS is 15.
body_project_spec.md:387 — IMU_ENABLED is True.
body_project_spec.md:388 — RGB_ENABLED is False by default.
body_project_spec.md:391 — OAK stub publishes synthetic IMU at 100 Hz (gravity only) and a placeholder depth message at 15 Hz, with no DepthAI access.
body_project_spec.md:395 — watchdog monitors system health and enforces safety stops.
body_project_spec.md:398 — If no heartbeat arrives within HEARTBEAT_TIMEOUT_MS, watchdog publishes body/emergency_stop with reason heartbeat_timeout.
body_project_spec.md:399 — If a process appears dead (no publications for PROCESS_TIMEOUT_MS), watchdog updates body/status accordingly.
body_project_spec.md:400 — Emergency stop clears only when heartbeat resumes AND Jill sends a new cmd_vel.
body_project_spec.md:401 — watchdog publishes body/status at 1 Hz.
body_project_spec.md:405 — HEARTBEAT_TIMEOUT_MS is 2000.
body_project_spec.md:406 — PROCESS_TIMEOUT_MS is 5000.
body_project_spec.md:407 — STATUS_PUBLISH_HZ is 1.
body_project_spec.md:408 — MONITORED_TOPICS are body/odom, body/lidar/scan, and body/oakd/imu.
body_project_spec.md:415 — Watchdog stub implements heartbeat timeout and publishes emergency_stop when heartbeat is lost, with no dependency on other Body processes running.
body_project_spec.md:419 — launcher starts all Body processes, monitors them, restarts on crash, and provides orderly shutdown.
body_project_spec.md:422 — launcher reads a process list from a config file or a hardcoded list.
body_project_spec.md:423 — launcher starts each process as a subprocess.
body_project_spec.md:424 — launcher captures stdout/stderr with process-name prefixes.
body_project_spec.md:425 — On unexpected exit, launcher restarts with backoff 1s, 2s, 4s, max 30s.
body_project_spec.md:426 — On SIGTERM/SIGINT, launcher sends SIGTERM to all children, waits up to 5s, then SIGKILL.
body_project_spec.md:430 — Default process list is watchdog, motor_controller, lidar_driver, oakd_driver via python3 *.py scripts.
body_project_spec.md:438 — Watchdog starts first; motor controller starts before sensors.
body_project_spec.md:440 — The launcher itself is hardware-independent and can be implemented completely (not a stub).
body_project_spec.md:445 — Project tree is body/ with README.md, config.json, launcher.py, watchdog.py, motor_controller.py, lidar_driver.py, oakd_driver.py, lib/, and requirements.txt.
body_project_spec.md:454 — lib/ includes zenoh_helpers.py, schemas.py, and diff_drive.py.
body_project_spec.md:457 — requirements.txt lists zenoh, gpiozero, depthai, pyserial.
body_project_spec.md:464 — The desktop must connect to the Zenoh router on the Pi at tcp://<pi-ip>:7447.
body_project_spec.md:466 — Jill (or a CW-side module) must publish body/heartbeat at ≥2 Hz whenever the robot is expected to be active; if she stops, the robot stops moving.
body_project_spec.md:468 — Jill publishes body/cmd_vel to drive; she must keep publishing if she wants continued motion (timeout semantics).
body_project_spec.md:470 — Jill subscribes to odom, lidar/scan, oakd/imu, oakd/depth, status, and motor_state as needed.
body_project_spec.md:472 — Robot frame is x-forward, y-left, z-up, angles CCW from x; odometry origin is boot position.
body_project_spec.md:474 — Zenoh pub/sub is best-effort; messages can be lost; cmd_vel timeout handles lost commands by stopping the robot.
body_project_spec.md:476 — After emergency_stop the robot will not move until heartbeat recovers AND a new cmd_vel is received.
body_project_spec.md:491 — After Phase 2, later phases swap a stub for real hardware behind the same Zenoh interface.
body_project_spec.md:495 — Pololu 4752 delivers 1920 CPR at the output shaft, ~10.5 kHz edge rate at full speed; software counting via lgpio is the initial approach.
body_project_spec.md:498 — Zenoh over WiFi may introduce 5–50 ms jitter.
body_project_spec.md:499 — gpiozero backed by lgpio is the current recommendation for Pi 5; RPi.GPIO does not work on Pi 5.
body_project_spec.md:500 — The MDD10A does not provide current feedback.
body_project_spec.md:501 — Pololu continuous load limit is 10 kg·cm; stall current is 5.5A per motor; MDD10A can handle 10A per channel; software stall detection (encoder velocity zero while PWM nonzero) is the available option without current sensing.

Surface is frozen at 164. I did not verify any claim.