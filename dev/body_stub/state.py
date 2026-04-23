"""Shared state the stub accumulates from Body topics.

BodyState is mutated by the controller (under a lock) from Zenoh
subscribe callbacks and by UI actions. The UI reads a snapshot.

Decoders here are pure: bytes in, typed dict (or ndarray for depth) out.
They swallow malformed messages and return None so a single bad publisher
can't take the stub down.
"""
from __future__ import annotations

import base64
import json
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from threading import RLock
from typing import Any, Deque, Dict, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class BodyState:
    # Connection / control
    connected: bool = False
    live_command: bool = False
    router: str = ""
    # Command mode selects which topic the publisher loop drives:
    # "cmd_vel" → body/cmd_vel (linear/angular twist)
    # "cmd_direct" → body/cmd_direct (per-wheel m/s, for bring-up)
    cmd_mode: str = "cmd_vel"
    last_cmd_vel: Tuple[float, float] = (0.0, 0.0)
    last_cmd_direct: Tuple[float, float] = (0.0, 0.0)  # (left, right) m/s
    heartbeat_seq: int = 0
    pending_rgb_request_id: Optional[str] = None
    pending_rgb_ts: float = 0.0

    # Subscribed topics: (latest_payload, wall-clock ts of receipt)
    status: Optional[Dict[str, Any]] = None
    status_ts: float = 0.0
    emergency_stop: Optional[Dict[str, Any]] = None
    emergency_ts: float = 0.0
    odom: Optional[Dict[str, Any]] = None
    odom_ts: float = 0.0
    motor_state: Optional[Dict[str, Any]] = None
    motor_ts: float = 0.0
    lidar_scan: Optional[Dict[str, Any]] = None
    lidar_ts: float = 0.0
    oakd_imu: Optional[Dict[str, Any]] = None
    oakd_imu_ts: float = 0.0

    # Depth: decoded ndarray + metadata
    depth_image: Optional[np.ndarray] = None
    depth_width: int = 0
    depth_height: int = 0
    depth_format: str = ""
    depth_ts: float = 0.0

    # RGB: raw JPEG bytes + metadata
    rgb_jpeg: Optional[bytes] = None
    rgb_width: int = 0
    rgb_height: int = 0
    rgb_ts: float = 0.0
    rgb_request_id: str = ""
    rgb_error: Optional[str] = None

    # Local 2.5D map: nan-filled float32 grid + frame metadata
    local_map_grid: Optional[np.ndarray] = None
    local_map_meta: Optional[Dict[str, Any]] = None
    local_map_ts: float = 0.0
    # Driveable layer rides on the same message; int8 (-1 unknown, 0 blocked, 1 clear)
    local_map_driveable: Optional[np.ndarray] = None
    # Wall-clock arrival ts of the last few local_map messages, for period
    # estimation. Sweep-360 sizes its settle from this.
    local_map_arrivals: Deque[float] = field(
        default_factory=lambda: deque(maxlen=8), repr=False,
    )

    lock: RLock = field(default_factory=RLock, repr=False)

    def snapshot_lidar(self) -> Tuple[Optional[Dict[str, Any]], float]:
        """Return (scan_dict, ts) under the state lock.

        The dict reference is shared, not deep-copied — the controller
        swaps in a fresh dict on every message rather than mutating in
        place, so the returned reference is safe to read from another
        thread for the lifetime of one mission step.
        """
        with self.lock:
            return self.lidar_scan, self.lidar_ts

    def local_map_period_s(self) -> Optional[float]:
        """Median inter-arrival of recent local_map messages, or None
        if too few have arrived to estimate.
        """
        with self.lock:
            ts = list(self.local_map_arrivals)
        if len(ts) < 2:
            return None
        deltas = sorted(b - a for a, b in zip(ts[:-1], ts[1:]) if (b - a) > 0)
        if not deltas:
            return None
        n = len(deltas)
        return deltas[n // 2] if n % 2 else 0.5 * (deltas[n // 2 - 1] + deltas[n // 2])


def _decode_json(payload: bytes) -> Optional[Dict[str, Any]]:
    try:
        return json.loads(payload.decode("utf-8"))
    except Exception as e:
        logger.debug(f"json decode failed: {e}")
        return None


def decode_status(payload: bytes) -> Optional[Dict[str, Any]]:
    return _decode_json(payload)


def decode_emergency_stop(payload: bytes) -> Optional[Dict[str, Any]]:
    return _decode_json(payload)


def decode_odom(payload: bytes) -> Optional[Dict[str, Any]]:
    return _decode_json(payload)


def decode_motor_state(payload: bytes) -> Optional[Dict[str, Any]]:
    return _decode_json(payload)


def decode_lidar_scan(payload: bytes) -> Optional[Dict[str, Any]]:
    return _decode_json(payload)


def decode_oakd_imu(payload: bytes) -> Optional[Dict[str, Any]]:
    return _decode_json(payload)


def decode_depth(payload: bytes) -> Optional[Dict[str, Any]]:
    """Decode body/oakd/depth. For format=='depth_uint16_mm', returns
    {'format','width','height','image': np.ndarray(uint16,(h,w))}.
    Other formats (e.g. 'placeholder') return {'format',...} without image.
    """
    msg = _decode_json(payload)
    if msg is None:
        return None
    fmt = msg.get("format", "")
    if fmt != "depth_uint16_mm":
        return {"format": fmt, "width": msg.get("width", 0),
                "height": msg.get("height", 0), "image": None}
    try:
        width = int(msg["width"])
        height = int(msg["height"])
        raw = base64.b64decode(msg["data"])
        arr = np.frombuffer(raw, dtype=np.uint16)
        if arr.size != width * height:
            logger.warning(
                f"depth size mismatch: got {arr.size}, expected {width*height}"
            )
            return {"format": fmt, "width": width, "height": height, "image": None}
        layout = msg.get("layout", "row_major")
        if layout != "row_major":
            logger.warning(f"unexpected depth layout '{layout}', treating as row_major")
        img = arr.reshape((height, width))
        return {"format": fmt, "width": width, "height": height, "image": img}
    except Exception as e:
        logger.warning(f"depth decode failed: {e}")
        return None


def decode_rgb(payload: bytes) -> Optional[Dict[str, Any]]:
    """Decode body/oakd/rgb. Returns dict with keys
    ok, request_id, width, height, jpeg (bytes or None), error (str or None).
    """
    msg = _decode_json(payload)
    if msg is None:
        return None
    ok = bool(msg.get("ok", False))
    req_id = str(msg.get("request_id", ""))
    width = int(msg.get("width", 0) or 0)
    height = int(msg.get("height", 0) or 0)
    error = msg.get("error")
    jpeg: Optional[bytes] = None
    if ok:
        data = msg.get("data")
        if data:
            try:
                jpeg = base64.b64decode(data)
            except Exception as e:
                logger.warning(f"rgb base64 decode failed: {e}")
                ok = False
                error = error or f"base64: {e}"
    return {"ok": ok, "request_id": req_id, "width": width,
            "height": height, "jpeg": jpeg, "error": error}


def decode_local_map(payload: bytes) -> Optional[Dict[str, Any]]:
    """Decode body/map/local_2p5d.

    Returns dict with keys:
        meta: dict (everything except max_height_m / driveable)
        grid: np.ndarray float32 shape (nx, ny), unmeasured cells as NaN
        driveable: np.ndarray int8 shape (nx, ny) or None if layer absent
                   (-1 unknown/null, 0 blocked, 1 clear)
    Returns None if the payload is malformed or kind is unsupported.
    """
    msg = _decode_json(payload)
    if msg is None:
        return None
    if msg.get("kind") != "max_height_grid":
        logger.debug(f"local_map: unsupported kind {msg.get('kind')!r}")
        return None
    try:
        nx = int(msg["nx"])
        ny = int(msg["ny"])
        rows = msg["max_height_m"]
        if not isinstance(rows, list) or len(rows) != nx:
            logger.warning(f"local_map: outer len {len(rows)} != nx {nx}")
            return None
        # Build flat float32 buffer; NaN for nulls. ~6 ns/cell at 100x100.
        flat = np.empty(nx * ny, dtype=np.float32)
        nan = np.float32("nan")
        idx = 0
        for r in rows:
            if not isinstance(r, list) or len(r) != ny:
                logger.warning(
                    f"local_map: inner len mismatch at row {idx//ny}"
                )
                return None
            for v in r:
                flat[idx] = nan if v is None else float(v)
                idx += 1
        grid = flat.reshape((nx, ny))

        driveable: Optional[np.ndarray] = None
        drows = msg.get("driveable")
        if isinstance(drows, list) and len(drows) == nx:
            dflat = np.empty(nx * ny, dtype=np.int8)
            j = 0
            ok = True
            for r in drows:
                if not isinstance(r, list) or len(r) != ny:
                    logger.warning(
                        f"local_map: driveable row len mismatch at {j//ny}"
                    )
                    ok = False
                    break
                for v in r:
                    if v is True:
                        dflat[j] = 1
                    elif v is False:
                        dflat[j] = 0
                    else:
                        dflat[j] = -1  # null / unknown
                    j += 1
            if ok:
                driveable = dflat.reshape((nx, ny))

        meta = {
            k: v for k, v in msg.items()
            if k not in ("max_height_m", "driveable")
        }
        return {"meta": meta, "grid": grid, "driveable": driveable}
    except (KeyError, TypeError, ValueError) as e:
        logger.warning(f"local_map decode failed: {e}")
        return None


def now_ts() -> float:
    return time.time()
