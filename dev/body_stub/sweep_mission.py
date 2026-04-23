"""Sweep-360 mission worker. Implements docs/sweep360_spec.md.

Drives the robot through a stepped 360° rotation, fuses the post-settle
local_2p5d frames into a sweep-frame accumulator, and reports a loop-
closure residual at completion.

Threading:
- One QThread (`SweepMission`) running `run()`.
- Drives motion via `controller.set_cmd_vel` + forced `live_command`,
  letting the existing `_cmd_loop` handle the wire publish at cmd_vel_hz.
- Publishes `body/sweep/status` and `body/map/sweep_360` directly through
  the controller's shared Zenoh session.
- Emits Qt signals for the dock to update UI (auto-queued across threads).
"""
from __future__ import annotations

import enum
import json
import logging
import math
import time
import uuid
from typing import Any, Dict, Optional

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

from .yaw_estimator import estimate_lidar_corr

logger = logging.getLogger(__name__)


class SweepState(str, enum.Enum):
    IDLE = "idle"
    PRECHECK = "precheck"
    ROTATING = "rotating"
    SETTLING = "settling"
    FUSING = "fusing"
    DONE = "done"
    ABORTED = "aborted"
    ESTOP = "estop"
    ERROR = "error"


_TERMINAL = (SweepState.DONE, SweepState.ABORTED, SweepState.ESTOP, SweepState.ERROR)


DEFAULT_PARAMS: Dict[str, Any] = {
    "step_deg": 30.0,
    "total_deg": 390.0,
    "angular_rate_dps": 30.0,
    "settle_ms": 1500,
    "direction": "ccw",
}


class SweepMission(QThread):
    """One-shot sweep mission. Reusable: call `start_mission()` again
    after a previous run terminated.
    """

    # Signals (auto-queued across threads when receiver is in GUI thread)
    state_changed = pyqtSignal(str)         # SweepState.value
    step_complete = pyqtSignal(dict)         # last_step debug info
    mission_done = pyqtSignal(dict)          # final accumulator dict
    accumulator_updated = pyqtSignal()       # UI repaint hook

    STATUS_TOPIC = "body/sweep/status"
    MAP_TOPIC = "body/map/sweep_360"

    SCAN_MATCH_MIN_CONFIDENCE = 0.35
    FUSE_VOTE_MARGIN = 1

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.controller = controller
        self._params = dict(DEFAULT_PARAMS)
        self._request_id: Optional[str] = None
        self._abort_requested = False
        self._state = SweepState.IDLE
        # Publishers (declared on entering run, undeclared on exit)
        self._status_pub: Optional[Any] = None
        self._map_pub: Optional[Any] = None
        # Mission-local state
        self._anchor_pose: Dict[str, float] = {"x_m": 0.0, "y_m": 0.0, "theta_rad": 0.0}
        self._first_scan: Optional[Dict[str, Any]] = None
        self._last_scan: Optional[Dict[str, Any]] = None
        self._yaw_accum_deg = 0.0
        self._step_index = 0
        self._step_count = 0
        self._loop_closure_deg: Optional[float] = None
        self._last_step_info: Dict[str, Any] = {}
        self._reason: Optional[str] = None
        # Accumulator (allocated on first local_map fusion)
        self._acc_max_h: Optional[np.ndarray] = None
        self._acc_clear_votes: Optional[np.ndarray] = None
        self._acc_block_votes: Optional[np.ndarray] = None
        self._acc_resolution_m = 0.0
        self._acc_extent_m = 0.0
        self._clearance_m: Optional[float] = None

    # ── Public API ───────────────────────────────────────────────────

    def state(self) -> SweepState:
        return self._state

    def is_active(self) -> bool:
        return self._state not in (SweepState.IDLE,) + _TERMINAL

    def start_mission(
        self,
        params: Optional[Dict[str, Any]] = None,
        request_id: Optional[str] = None,
    ) -> bool:
        """Begin a new sweep. Returns False if a sweep is already active."""
        if self.is_active() or self.isRunning():
            logger.warning("SweepMission.start_mission called while active")
            return False
        merged = dict(DEFAULT_PARAMS)
        if params:
            for k in DEFAULT_PARAMS:
                if k in params and params[k] is not None:
                    merged[k] = params[k]
        if merged["direction"] not in ("ccw", "cw"):
            merged["direction"] = "ccw"
        merged["step_deg"] = max(1.0, float(merged["step_deg"]))
        merged["total_deg"] = max(merged["step_deg"], float(merged["total_deg"]))
        merged["angular_rate_dps"] = max(1.0, float(merged["angular_rate_dps"]))
        merged["settle_ms"] = max(100, int(merged["settle_ms"]))
        self._params = merged
        self._request_id = request_id or str(uuid.uuid4())
        self._abort_requested = False
        self._reason = None
        self._first_scan = None
        self._last_scan = None
        self._yaw_accum_deg = 0.0
        self._step_index = 0
        self._step_count = int(math.ceil(merged["total_deg"] / merged["step_deg"]))
        self._loop_closure_deg = None
        self._last_step_info = {}
        self._acc_max_h = None
        self._acc_clear_votes = None
        self._acc_block_votes = None
        self._clearance_m = None
        self.start()  # QThread.start() → run()
        return True

    def request_abort(self) -> None:
        self._abort_requested = True

    def snapshot_accumulator(self) -> Optional[Dict[str, Any]]:
        """Return a render-ready snapshot of the accumulator, or None
        if nothing has been fused yet. Shape matches local_2p5d so
        existing LocalMapView/DriveableView widgets render it directly.
        """
        if self._acc_max_h is None:
            return None
        n = self._acc_max_h.shape[0]
        meta: Dict[str, Any] = {
            "resolution_m": self._acc_resolution_m,
            "origin_x_m": -self._acc_extent_m,
            "origin_y_m": -self._acc_extent_m,
            "nx": n,
            "ny": n,
            "frame": "sweep",
        }
        if self._clearance_m is not None:
            meta["driveable_clearance_height_m"] = self._clearance_m
        drive = self._driveable_from_votes()
        return {
            "grid": self._acc_max_h.copy(),
            "meta": meta,
            "driveable": drive,
        }

    # ── Main loop ────────────────────────────────────────────────────

    def run(self) -> None:
        try:
            self._declare_publishers()
            self._set_state(SweepState.PRECHECK)
            if not self._precheck():
                return

            self._take_anchor()

            # Capture the very first scan for loop-closure at the end.
            self._first_scan, _ = self.controller.state.snapshot_lidar()
            if self._first_scan is None:
                # Wait briefly for one to land
                self._first_scan = self._wait_for_scan(time.time() - 1.0, timeout_s=2.0)
            if self._first_scan is None:
                self._fail("stale_lidar_at_start")
                return
            pre_step_scan = self._first_scan

            for i in range(self._step_count):
                if self._abort_or_estop_check():
                    return
                self._step_index = i
                pre_step_scan = self._do_step(i, pre_step_scan)
                if pre_step_scan is None:
                    return  # step set the terminal state
                self._last_scan = pre_step_scan

            # Loop closure: compare first scan to last post-settle scan.
            if self._first_scan is not None and self._last_scan is not None:
                deg, _ = estimate_lidar_corr(self._first_scan, self._last_scan)
                if deg is not None:
                    residual = deg - self._yaw_accum_deg
                    residual = ((residual + 180.0) % 360.0) - 180.0
                    self._loop_closure_deg = residual

            self._publish_map(final=True)
            self._set_state(SweepState.DONE)
            self.mission_done.emit(self._build_map_dict())
        except Exception as e:
            logger.exception("SweepMission.run crashed")
            self._fail(f"{type(e).__name__}: {e}")
        finally:
            self._stop_motion()
            self._undeclare_publishers()

    # ── State machine helpers ────────────────────────────────────────

    def _set_state(self, new_state: SweepState, *, reason: Optional[str] = None) -> None:
        self._state = new_state
        if reason is not None:
            self._reason = reason
        self.state_changed.emit(new_state.value)
        # Spec §4.2: silent in idle. Publish on every transition otherwise.
        if new_state != SweepState.IDLE:
            self._publish_status()

    def _fail(self, reason: str) -> None:
        self._set_state(SweepState.ERROR, reason=reason)

    def _precheck(self) -> bool:
        with self.controller.state.lock:
            connected = self.controller.state.connected
        if not connected:
            self._fail("not_connected")
            return False
        if self._estop_active():
            self._set_state(SweepState.ESTOP, reason="estop_at_start")
            return False
        return True

    def _estop_active(self) -> bool:
        with self.controller.state.lock:
            es = self.controller.state.emergency_stop
            st = self.controller.state.status or {}
            ms = self.controller.state.motor_state or {}
        if isinstance(es, dict) and (es.get("active") or es.get("e_stop_active")):
            return True
        if st.get("e_stop_active"):
            return True
        if ms.get("e_stop_active"):
            return True
        return False

    def _abort_or_estop_check(self) -> bool:
        if self._abort_requested:
            self._set_state(SweepState.ABORTED, reason="abort_requested")
            return True
        if self._estop_active():
            self._set_state(SweepState.ESTOP, reason="estop")
            return True
        return False

    def _take_anchor(self) -> None:
        with self.controller.state.lock:
            odom = self.controller.state.odom
        if isinstance(odom, dict):
            self._anchor_pose = {
                "x_m": float(odom.get("x", 0.0) or 0.0),
                "y_m": float(odom.get("y", 0.0) or 0.0),
                "theta_rad": float(odom.get("theta", 0.0) or 0.0),
            }
        else:
            self._anchor_pose = {"x_m": 0.0, "y_m": 0.0, "theta_rad": 0.0}

    # ── Per-step logic ───────────────────────────────────────────────

    def _do_step(
        self, i: int, pre_step_scan: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        params = self._params
        rate_rad_s = math.radians(params["angular_rate_dps"])
        sign = 1.0 if params["direction"] == "ccw" else -1.0
        t_step = math.radians(params["step_deg"]) / rate_rad_s

        # ── ROTATING ────────────────────────────────────────────────
        self._set_state(SweepState.ROTATING)
        # Make sure we're commanding the twist topic, not direct.
        self.controller.set_cmd_mode("cmd_vel")
        self.controller.set_cmd_vel(0.0, sign * rate_rad_s)
        self.controller.set_live_command(True)
        deadline = time.monotonic() + t_step
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            if self._abort_or_estop_check():
                return None
            self._publish_status()
            time.sleep(min(0.1, deadline - now))
        # Stop motion; keep live_command so cmd_loop emits zeros (and so
        # the watchdog stays happy) through settling and fusing.
        self.controller.set_cmd_vel(0.0, 0.0)

        # ── SETTLING ────────────────────────────────────────────────
        self._set_state(SweepState.SETTLING)
        scan_period_s = self._scan_period_s()
        local_map_period_s = self._local_map_period_s_or(default=1.0)
        spec_min_s = scan_period_s + local_map_period_s
        settle_s = max(params["settle_ms"] / 1000.0, spec_min_s)
        settle_started_wall = time.time()
        deadline = time.monotonic() + settle_s
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            if self._abort_or_estop_check():
                return None
            self._publish_status()
            time.sleep(min(0.1, deadline - now))

        # ── FUSING ──────────────────────────────────────────────────
        self._set_state(SweepState.FUSING)
        scan_threshold_ts = settle_started_wall + scan_period_s
        scan_post = self._wait_for_scan(scan_threshold_ts, timeout_s=2.0)
        if scan_post is None:
            self._fail("stale_lidar")
            return None
        # local_map threshold uses its own source.lidar_ts when available,
        # falling back to receipt ts.
        local_map_post = self._wait_for_local_map(scan_threshold_ts, timeout_s=4.0)
        if local_map_post is None:
            self._fail("stale_local_map")
            return None

        commanded_deg = sign * params["step_deg"]
        lidar_deg, lidar_conf = estimate_lidar_corr(pre_step_scan, scan_post)
        imu_deg = None  # current hardware has no IMU
        cmd_deg = commanded_deg

        if lidar_deg is not None and lidar_conf >= self.SCAN_MATCH_MIN_CONFIDENCE:
            fused = lidar_deg
        elif imu_deg is not None:
            fused = imu_deg
        else:
            fused = cmd_deg
        self._yaw_accum_deg += fused

        self._fuse_local_map(local_map_post, self._yaw_accum_deg)

        self._last_step_info = {
            "commanded_deg": commanded_deg,
            "yaw_sources": {"lidar": lidar_deg, "imu": imu_deg, "cmd": cmd_deg},
            "fused_deg": fused,
            "residual_xy_m": [0.0, 0.0],  # correlation gives no translation
            "settle_ms": int(settle_s * 1000),
            "lidar_confidence": lidar_conf,
        }
        self.step_complete.emit(dict(self._last_step_info))
        self.accumulator_updated.emit()

        self._publish_map(final=False)
        self._publish_status()
        return scan_post

    # ── Sensor waits ─────────────────────────────────────────────────

    def _scan_period_s(self) -> float:
        with self.controller.state.lock:
            scan = self.controller.state.lidar_scan
        if isinstance(scan, dict):
            stm = scan.get("scan_time_ms")
            if isinstance(stm, (int, float)) and stm > 0:
                return float(stm) / 1000.0
        return 0.2  # safe default ~5 Hz

    def _local_map_period_s_or(self, *, default: float) -> float:
        period = self.controller.state.local_map_period_s()
        if period is None or period <= 0:
            return default
        return period

    def _scan_msg_ts(self, scan: Dict[str, Any], recv_ts: float) -> float:
        ts = scan.get("ts")
        if isinstance(ts, (int, float)) and ts > 0:
            return float(ts)
        return recv_ts

    def _wait_for_scan(
        self, threshold_ts: float, timeout_s: float,
    ) -> Optional[Dict[str, Any]]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._abort_or_estop_check():
                return None
            scan, recv_ts = self.controller.state.snapshot_lidar()
            if scan is not None:
                if self._scan_msg_ts(scan, recv_ts) > threshold_ts:
                    return scan
            time.sleep(0.05)
        return None

    def _wait_for_local_map(
        self, threshold_ts: float, timeout_s: float,
    ) -> Optional[Dict[str, Any]]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._abort_or_estop_check():
                return None
            with self.controller.state.lock:
                grid = self.controller.state.local_map_grid
                meta = self.controller.state.local_map_meta
                drive = self.controller.state.local_map_driveable
                map_recv_ts = self.controller.state.local_map_ts
            if grid is not None and meta is not None:
                # local_map carries sources.lidar_ts / depth_ts; honor whichever
                # the Pi has marked as freshest, fall back to recv ts.
                src = meta.get("sources") or {}
                src_ts = 0.0
                for k in ("lidar_ts", "depth_ts"):
                    v = src.get(k)
                    if isinstance(v, (int, float)) and v > src_ts:
                        src_ts = float(v)
                eff_ts = src_ts if src_ts > 0 else map_recv_ts
                if eff_ts > threshold_ts:
                    return {
                        "grid": grid.copy(),
                        "meta": dict(meta),
                        "driveable": drive.copy() if drive is not None else None,
                    }
            time.sleep(0.05)
        return None

    # ── Grid fusion ──────────────────────────────────────────────────

    def _fuse_local_map(
        self, local_map: Dict[str, Any], theta_accum_deg: float,
    ) -> None:
        meta = local_map["meta"]
        grid = local_map["grid"]            # (nx, ny) max_height_m, NaN unknown
        drive = local_map.get("driveable")  # (nx, ny) int8 or None
        res = float(meta.get("resolution_m", 0.0))
        if res <= 0:
            return
        origin_x = float(meta.get("origin_x_m", 0.0))
        origin_y = float(meta.get("origin_y_m", 0.0))
        nx_b, ny_b = grid.shape

        if self._acc_max_h is None:
            # Allocate a square accumulator big enough to hold the source
            # extent rotated through any yaw. Take the worst-case half-extent
            # (max distance from anchor any source-cell center can reach).
            corners = [
                (origin_x, origin_y),
                (origin_x + nx_b * res, origin_y),
                (origin_x, origin_y + ny_b * res),
                (origin_x + nx_b * res, origin_y + ny_b * res),
            ]
            half = max(math.hypot(x, y) for x, y in corners)
            n_side = 2 * int(math.ceil(half / res))
            self._acc_max_h = np.full((n_side, n_side), np.nan, dtype=np.float32)
            self._acc_clear_votes = np.zeros((n_side, n_side), dtype=np.int32)
            self._acc_block_votes = np.zeros((n_side, n_side), dtype=np.int32)
            self._acc_resolution_m = res
            self._acc_extent_m = (n_side / 2.0) * res

        if abs(res - self._acc_resolution_m) > 1e-6:
            logger.warning(
                f"local_map resolution changed mid-sweep: {res} vs {self._acc_resolution_m}"
            )
            return

        clr = meta.get("driveable_clearance_height_m")
        if isinstance(clr, (int, float)):
            self._clearance_m = float(clr)

        n = self._acc_max_h.shape[0]
        # Map every accumulator cell back into the body frame of this
        # local_map by rotating by -theta_accum, then look up the source
        # cell. Vectorized.
        theta_rad = math.radians(theta_accum_deg)
        c, s = math.cos(theta_rad), math.sin(theta_rad)
        # float64 throughout the index math; float32 lets `floor((0.20)/0.1)`
        # land at 1.999… → wrong cell. Tiny epsilon nudges boundary cells
        # consistently into the next cell, matching the local_map convention.
        ii = np.arange(n, dtype=np.float64)
        jj = np.arange(n, dtype=np.float64)
        x_s = -self._acc_extent_m + (ii + 0.5) * res
        y_s = -self._acc_extent_m + (jj + 0.5) * res
        Xs, Ys = np.meshgrid(x_s, y_s, indexing="ij")
        Xb = c * Xs + s * Ys
        Yb = -s * Xs + c * Ys
        ib = np.floor((Xb - origin_x) / res + 1e-9).astype(np.int32)
        jb = np.floor((Yb - origin_y) / res + 1e-9).astype(np.int32)
        valid = (ib >= 0) & (ib < nx_b) & (jb >= 0) & (jb < ny_b)
        if not np.any(valid):
            return

        ib_v = ib[valid]
        jb_v = jb[valid]
        src_h = grid[ib_v, jb_v]
        target_h = self._acc_max_h[valid]
        src_has = ~np.isnan(src_h)
        # Fold in source heights: max if both, source if target NaN, target otherwise.
        merged = np.where(np.isnan(target_h), src_h, np.maximum(target_h, src_h))
        merged = np.where(src_has, merged, target_h)
        self._acc_max_h[valid] = merged

        if drive is not None:
            src_d = drive[ib_v, jb_v]
            self._acc_clear_votes[valid] += (src_d == 1).astype(np.int32)
            self._acc_block_votes[valid] += (src_d == 0).astype(np.int32)

    def _driveable_from_votes(self) -> np.ndarray:
        n = self._acc_max_h.shape[0]
        out = np.full((n, n), -1, dtype=np.int8)
        if self._acc_clear_votes is None or self._acc_block_votes is None:
            return out
        margin = self.FUSE_VOTE_MARGIN
        out[self._acc_clear_votes > self._acc_block_votes + margin] = 1
        out[self._acc_block_votes > self._acc_clear_votes + margin] = 0
        return out

    # ── Output building ──────────────────────────────────────────────

    def _build_map_dict(self) -> Dict[str, Any]:
        if self._acc_max_h is None:
            return {}
        n = self._acc_max_h.shape[0]
        drive = self._driveable_from_votes()
        max_h_rows = [
            [None if math.isnan(v) else float(v) for v in row]
            for row in self._acc_max_h
        ]
        drive_rows = [
            [True if v == 1 else (False if v == 0 else None) for v in row]
            for row in drive
        ]
        out: Dict[str, Any] = {
            "ts": time.time(),
            "frame": "sweep",
            "kind": "max_height_grid",
            "resolution_m": self._acc_resolution_m,
            "origin_x_m": -self._acc_extent_m,
            "origin_y_m": -self._acc_extent_m,
            "nx": n,
            "ny": n,
            "max_height_m": max_h_rows,
            "driveable": drive_rows,
            "anchor_pose": dict(self._anchor_pose),
            "step_count": self._step_count,
            "loop_closure_deg": self._loop_closure_deg,
            "request_id": self._request_id,
        }
        if self._clearance_m is not None:
            out["driveable_clearance_height_m"] = self._clearance_m
        return out

    def _publish_map(self, *, final: bool) -> None:
        if self._map_pub is None or self._acc_max_h is None:
            return
        try:
            self._map_pub.put(
                json.dumps(self._build_map_dict()).encode("utf-8")
            )
        except Exception:
            logger.exception("sweep_360 publish failed")

    def _publish_status(self) -> None:
        if self._status_pub is None or self._state == SweepState.IDLE:
            return
        payload = {
            "ts": time.time(),
            "state": self._state.value,
            "request_id": self._request_id,
            "step_index": self._step_index,
            "step_count": self._step_count,
            "yaw_accum_deg": self._yaw_accum_deg,
            "coverage_deg": self._yaw_accum_deg,
            "last_step": self._last_step_info or None,
            "loop_closure_deg": self._loop_closure_deg,
            "reason": self._reason,
        }
        try:
            self._status_pub.put(json.dumps(payload).encode("utf-8"))
        except Exception:
            logger.exception("sweep status publish failed")

    # ── Publisher lifecycle ──────────────────────────────────────────

    def _declare_publishers(self) -> None:
        sess = self.controller._session
        if sess is None:
            return
        try:
            self._status_pub = sess.declare_publisher(self.STATUS_TOPIC)
            self._map_pub = sess.declare_publisher(self.MAP_TOPIC)
        except Exception:
            logger.exception("sweep publishers declare failed")
            self._status_pub = None
            self._map_pub = None

    def _undeclare_publishers(self) -> None:
        for pub in (self._status_pub, self._map_pub):
            if pub is not None:
                try:
                    pub.undeclare()
                except Exception:
                    pass
        self._status_pub = None
        self._map_pub = None

    # ── Motion shutdown ──────────────────────────────────────────────

    def _stop_motion(self) -> None:
        try:
            self.controller.set_cmd_vel(0.0, 0.0)
            self.controller.set_live_command(False)
        except Exception:
            logger.exception("sweep stop_motion raised")
