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
