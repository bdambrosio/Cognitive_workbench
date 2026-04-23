"""World-map fuser configuration.

Precedence: CLI --router > ZENOH_CONNECT env > default.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional


DEFAULT_ROUTER = "tcp/127.0.0.1:7447"
ENV_VAR = "ZENOH_CONNECT"


@dataclass
class Topics:
    local_map: str = "body/map/local_2p5d"
    odom: str = "body/odom"
    lidar_scan: str = "body/lidar/scan"
    status_in: str = "body/status"
    world_cmd: str = "body/world_map/cmd"
    world_status: str = "body/world_map/status"
    world_driveable: str = "body/map/world_driveable"


@dataclass
class FuserConfig:
    router: str = DEFAULT_ROUTER

    world_extent_m: float = 40.0
    world_resolution_m: float = 0.08
    publish_hz: float = 2.0
    publish_margin_cells: int = 4
    status_hz: float = 1.0

    stale_odom_s: float = 0.25
    input_timeout_s: float = 2.0

    pose_source: str = "odom"  # v1.1: "odom+scanmatch"

    vote_margin: int = 1
    traversal_stamp_hz: float = 10.0
    traversal_vote_weight: int = 3
    footprint_radius_m: float = 0.15

    ui_redraw_hz: float = 5.0
    map_stale_s: float = 2.0

    topics: Topics = field(default_factory=Topics)


def resolve_router(cli_value: Optional[str]) -> str:
    if cli_value:
        return cli_value
    env = os.environ.get(ENV_VAR)
    if env:
        return env
    return DEFAULT_ROUTER
