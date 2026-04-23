"""Stub configuration.

Precedence: CLI --router > ZENOH_CONNECT env > default.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional


DEFAULT_ROUTER = "tcp/127.0.0.1:7447"
ENV_VAR = "ZENOH_CONNECT"


@dataclass
class StubConfig:
    router: str = DEFAULT_ROUTER
    heartbeat_hz: float = 5.0
    cmd_vel_hz: float = 10.0
    cmd_vel_timeout_ms: int = 500
    rgb_request_timeout_s: float = 3.0
    ui_redraw_hz: float = 10.0
    # Motor-test dock: UI-side clamp on per-wheel slider range (m/s).
    # Independent of Pi motor.max_wheel_vel_ms — belt-and-suspenders so a
    # mis-configured Pi can't be commanded past the tester's own ceiling.
    max_wheel_vel_default: float = 0.3
    # local_map: how long without a sample before we treat the panel as
    # stale ("no map / disabled?"). Pi default cadence is moving downward
    # from 5 Hz; set this to ~3× the Pi's configured period.
    map_stale_s: float = 2.0

    # Jill chat routing (used only when the Vision dock's "Jill" mode is
    # selected). jill_router=None means reuse the body stub's router.
    jill_character: str = "Jill"
    jill_router: Optional[str] = None

    topics: "Topics" = field(default_factory=lambda: Topics())


@dataclass
class Topics:
    # Publish
    heartbeat: str = "body/heartbeat"
    cmd_vel: str = "body/cmd_vel"
    cmd_direct: str = "body/cmd_direct"
    oakd_config: str = "body/oakd/config"
    # Subscribe
    status: str = "body/status"
    emergency_stop: str = "body/emergency_stop"
    odom: str = "body/odom"
    motor_state: str = "body/motor_state"
    lidar_scan: str = "body/lidar/scan"
    oakd_imu: str = "body/oakd/imu"
    oakd_depth: str = "body/oakd/depth"
    oakd_rgb: str = "body/oakd/rgb"
    local_map: str = "body/map/local_2p5d"
    sweep_cmd: str = "body/sweep/cmd"


def resolve_router(cli_value: Optional[str]) -> str:
    if cli_value:
        return cli_value
    env = os.environ.get(ENV_VAR)
    if env:
        return env
    return DEFAULT_ROUTER
