"""Fuser controller: owns Zenoh session, subscribers, fusion worker
thread, and publish loops.

Threading model
---------------
- Zenoh subscribe callbacks land on Zenoh's own threads. They normalize
  the payload and either update the PoseSource directly (cheap) or hand
  off heavier work via a thread-safe input queue.
- One worker thread (`_fusion_loop`) drains the local_map queue,
  resolves pose, and folds frames into the WorldGrid.
- A second worker thread (`_traversal_loop`) ticks the traversal stamp
  at `traversal_stamp_hz`.
- A third worker thread (`_publish_loop`) publishes
  body/map/world_driveable at `publish_hz` and body/world_map/status at
  `status_hz`.
- The UI thread polls snapshot_for_ui() on its own redraw timer.
"""
from __future__ import annotations

import json
import logging
import math
import queue
import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

from .config import FuserConfig
from .pose_source import OdomPose, PoseSource
from .transport import open_session
from .world_grid import WorldGrid, encode_for_publish

logger = logging.getLogger(__name__)


def _now() -> float:
    return time.time()


# ── Helpers shared with body_stub-style decoders ────────────────────

def _decode_json(payload: bytes) -> Optional[Dict[str, Any]]:
    try:
        return json.loads(payload.decode("utf-8"))
    except Exception as e:
        logger.debug(f"json decode failed: {e}")
        return None


def _decode_local_map(msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Reduce parsed local_map JSON to {meta, grid (np.ndarray), driveable}.

    Returns None on schema problems. NaN cells in grid for `null` heights;
    int8 (-1/0/1) for driveable; None if driveable layer missing.
    """
    if msg.get("kind") != "max_height_grid":
        return None
    try:
        nx = int(msg["nx"])
        ny = int(msg["ny"])
        rows = msg["max_height_m"]
        if not isinstance(rows, list) or len(rows) != nx:
            return None
        flat = np.empty(nx * ny, dtype=np.float32)
        nan = np.float32("nan")
        idx = 0
        for r in rows:
            if not isinstance(r, list) or len(r) != ny:
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
                    ok = False
                    break
                for v in r:
                    if v is True:
                        dflat[j] = 1
                    elif v is False:
                        dflat[j] = 0
                    else:
                        dflat[j] = -1
                    j += 1
            if ok:
                driveable = dflat.reshape((nx, ny))

        meta = {
            k: v for k, v in msg.items()
            if k not in ("max_height_m", "driveable")
        }
        return {"meta": meta, "grid": grid, "driveable": driveable}
    except (KeyError, TypeError, ValueError):
        return None


def _odom_xyt(msg: Dict[str, Any]) -> Optional[Tuple[float, float, float, float]]:
    """Extract (ts, x, y, theta) from a parsed body/odom message.

    Tolerates both 'theta' and 'yaw' / 'theta_rad' field spellings — the
    Pi schema has wobbled here in the past.
    """
    try:
        ts = float(msg.get("ts") or _now())
        x = float(msg.get("x") or 0.0)
        y = float(msg.get("y") or 0.0)
        if "theta" in msg:
            th = float(msg["theta"])
        elif "theta_rad" in msg:
            th = float(msg["theta_rad"])
        elif "yaw" in msg:
            th = float(msg["yaw"])
        else:
            th = 0.0
        return ts, x, y, th
    except (TypeError, ValueError):
        return None


def _local_map_capture_ts(meta: Dict[str, Any], fallback: float) -> float:
    """Pick the freshest sensor ts from a local_map's `sources` block;
    fall back to the message ts or the receipt ts.
    """
    src = meta.get("sources") or {}
    best = 0.0
    for k in ("lidar_ts", "depth_ts"):
        v = src.get(k)
        if isinstance(v, (int, float)) and v > best:
            best = float(v)
    if best > 0:
        return best
    msg_ts = meta.get("ts")
    if isinstance(msg_ts, (int, float)) and msg_ts > 0:
        return float(msg_ts)
    return fallback


# ── Controller ──────────────────────────────────────────────────────


class FuserController:
    """Owns all Zenoh I/O and fusion threads. UI-agnostic."""

    def __init__(self, config: FuserConfig):
        self.config = config
        self.pose_source: PoseSource = OdomPose()
        self.grid = WorldGrid(
            extent_m=config.world_extent_m,
            resolution_m=config.world_resolution_m,
            vote_margin=config.vote_margin,
            traversal_vote_weight=config.traversal_vote_weight,
            footprint_radius_m=config.footprint_radius_m,
        )

        # Zenoh handles
        self._session: Optional[Any] = None
        self._subscribers: List[Any] = []
        self._pub_world_driveable: Optional[Any] = None
        self._pub_world_status: Optional[Any] = None

        # Threads
        self._stop_event = threading.Event()
        self._fusion_thread: Optional[threading.Thread] = None
        self._traversal_thread: Optional[threading.Thread] = None
        self._publish_thread: Optional[threading.Thread] = None

        # Inputs
        self._local_map_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=8)

        # Telemetry / state
        self._lock = threading.RLock()
        self._connected = False
        self._notes: Optional[str] = None
        self._last_pose_unavail_streak = 0
        # Per-topic arrival ts ring buffers for rate estimation.
        self._arr_local_map: Deque[float] = deque(maxlen=32)
        self._arr_odom: Deque[float] = deque(maxlen=64)
        self._arr_lidar: Deque[float] = deque(maxlen=32)
        # Most recent receipt ts per topic for staleness reporting.
        self._last_local_map_ts = 0.0
        self._last_odom_ts = 0.0
        self._last_lidar_ts = 0.0
        self._last_correction = (0.0, 0.0, 0.0)
        # Optional UI hooks.
        self._on_session_change: Optional[Callable[[str], None]] = None
        self._on_grid_update: Optional[Callable[[], None]] = None

    # ── UI hooks ─────────────────────────────────────────────────────

    def set_on_session_change(self, cb: Optional[Callable[[str], None]]) -> None:
        self._on_session_change = cb

    def set_on_grid_update(self, cb: Optional[Callable[[], None]]) -> None:
        self._on_grid_update = cb

    # ── Lifecycle ────────────────────────────────────────────────────

    def connect(self) -> Tuple[bool, Optional[str]]:
        if self._session is not None:
            return True, None
        try:
            self._session = open_session(self.config.router)
        except Exception as e:
            logger.exception("zenoh open failed")
            return False, f"{type(e).__name__}: {e}"
        try:
            self._declare()
        except Exception as e:
            logger.exception("declare failed")
            self._teardown_zenoh()
            return False, f"{type(e).__name__}: {e}"

        self._stop_event.clear()
        self._fusion_thread = threading.Thread(
            target=self._fusion_loop, name="wm-fusion", daemon=True,
        )
        self._traversal_thread = threading.Thread(
            target=self._traversal_loop, name="wm-traversal", daemon=True,
        )
        self._publish_thread = threading.Thread(
            target=self._publish_loop, name="wm-publish", daemon=True,
        )
        self._fusion_thread.start()
        self._traversal_thread.start()
        self._publish_thread.start()
        with self._lock:
            self._connected = True
        return True, None

    def shutdown(self) -> None:
        self._stop_event.set()
        for t in (self._fusion_thread, self._traversal_thread,
                  self._publish_thread):
            if t is not None and t.is_alive():
                t.join(timeout=1.0)
        self._fusion_thread = None
        self._traversal_thread = None
        self._publish_thread = None
        self._teardown_zenoh()
        with self._lock:
            self._connected = False

    def _teardown_zenoh(self) -> None:
        for sub in self._subscribers:
            try:
                sub.undeclare()
            except Exception:
                pass
        self._subscribers.clear()
        for pub in (self._pub_world_driveable, self._pub_world_status):
            if pub is not None:
                try:
                    pub.undeclare()
                except Exception:
                    pass
        self._pub_world_driveable = None
        self._pub_world_status = None
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None

    # ── Zenoh wiring ─────────────────────────────────────────────────

    def _declare(self) -> None:
        assert self._session is not None
        t = self.config.topics
        pairs = [
            (t.local_map, self._on_local_map),
            (t.odom, self._on_odom),
            (t.lidar_scan, self._on_lidar_scan),
            (t.world_cmd, self._on_world_cmd),
        ]
        for key, cb in pairs:
            sub = self._session.declare_subscriber(key, cb)
            self._subscribers.append(sub)
        self._pub_world_driveable = self._session.declare_publisher(
            t.world_driveable
        )
        self._pub_world_status = self._session.declare_publisher(
            t.world_status
        )

    def _payload_bytes(self, sample: Any) -> bytes:
        try:
            return bytes(sample.payload.to_bytes())
        except AttributeError:
            return bytes(sample.payload)

    # ── Subscribe callbacks (Zenoh threads) ──────────────────────────

    def _on_local_map(self, sample: Any) -> None:
        msg = _decode_json(self._payload_bytes(sample))
        if msg is None:
            return
        decoded = _decode_local_map(msg)
        if decoded is None:
            return
        recv_ts = _now()
        with self._lock:
            self._arr_local_map.append(recv_ts)
            self._last_local_map_ts = recv_ts
        # Drop oldest if queue full so we don't accumulate latency.
        try:
            self._local_map_q.put_nowait({
                "meta": decoded["meta"],
                "grid": decoded["grid"],
                "driveable": decoded["driveable"],
                "recv_ts": recv_ts,
            })
        except queue.Full:
            try:
                self._local_map_q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._local_map_q.put_nowait({
                    "meta": decoded["meta"],
                    "grid": decoded["grid"],
                    "driveable": decoded["driveable"],
                    "recv_ts": recv_ts,
                })
            except queue.Full:
                logger.warning("local_map queue still full after drop; skipping")

    def _on_odom(self, sample: Any) -> None:
        msg = _decode_json(self._payload_bytes(sample))
        if msg is None:
            return
        triple = _odom_xyt(msg)
        if triple is None:
            return
        ts, x, y, theta = triple
        # Use Pi-supplied ts for the pose buffer; recv ts only for rates.
        if isinstance(self.pose_source, OdomPose):
            self.pose_source.update(ts, x, y, theta)
        with self._lock:
            self._arr_odom.append(_now())
            self._last_odom_ts = _now()

    def _on_lidar_scan(self, sample: Any) -> None:
        # v1: only used for telemetry; v1.1 will feed the scan-matcher.
        with self._lock:
            self._arr_lidar.append(_now())
            self._last_lidar_ts = _now()

    def _on_world_cmd(self, sample: Any) -> None:
        msg = _decode_json(self._payload_bytes(sample))
        if msg is None or not isinstance(msg, dict):
            return
        action = msg.get("action")
        if action == "reset":
            self._do_reset(reason=str(msg.get("reason") or "cmd_reset"))
        elif action == "relocate":
            logger.info("world_cmd action=relocate is v1.1; ignoring")
        else:
            logger.debug(f"world_cmd: ignoring action {action!r}")

    # ── Reset ────────────────────────────────────────────────────────

    def _do_reset(self, *, reason: str) -> None:
        new_id = self.grid.reset((0.0, 0.0, 0.0))
        if isinstance(self.pose_source, OdomPose):
            self.pose_source.rebind_world_to_current()
        with self._lock:
            self._notes = f"reset:{reason}"
            self._last_pose_unavail_streak = 0
        cb = self._on_session_change
        if cb is not None:
            try:
                cb(new_id)
            except Exception:
                logger.exception("on_session_change callback raised")
        logger.info(f"world reset; new session_id={new_id} reason={reason}")

    def request_reset(self, *, reason: str = "ui_reset") -> None:
        """UI entry point; safe to call from any thread."""
        self._do_reset(reason=reason)

    # ── Worker threads ───────────────────────────────────────────────

    def _fusion_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                item = self._local_map_q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                meta = item["meta"]
                grid = item["grid"]
                drive = item["driveable"]
                recv_ts = item["recv_ts"]
                cap_ts = _local_map_capture_ts(meta, recv_ts)

                # Honor anchor_pose if Pi stamps it on the frame.
                ap = meta.get("odom_pose_at_capture") or meta.get("anchor_pose")
                if isinstance(ap, dict) and "x_m" in ap:
                    pose_in_odom = (
                        float(ap.get("x_m", 0.0)),
                        float(ap.get("y_m", 0.0)),
                        float(ap.get("theta_rad", ap.get("theta", 0.0))),
                    )
                    if isinstance(self.pose_source, OdomPose):
                        # Convert odom-frame anchor to world via the
                        # current offset.
                        pose = self.pose_source._to_world(*pose_in_odom)
                    else:
                        pose = pose_in_odom
                else:
                    pose = self.pose_source.pose_at(cap_ts)

                if pose is None:
                    with self._lock:
                        self._last_pose_unavail_streak += 1
                        if self._last_pose_unavail_streak >= 10:
                            self._notes = "pose_unavailable"
                    # Check stale_odom budget: if odom is fresh but ts far
                    # outside buffer, still drop. Either way, don't fuse.
                    continue

                with self._lock:
                    self._last_pose_unavail_streak = 0

                n_in, n_out = self.grid.fuse_local_map(
                    grid=grid,
                    driveable=drive,
                    meta=meta,
                    pose_world=pose,
                    capture_ts=cap_ts,
                )
                if n_out > 0 and n_in == 0:
                    with self._lock:
                        self._notes = "world_bounds_exceeded"
                if n_in > 0:
                    cb = self._on_grid_update
                    if cb is not None:
                        try:
                            cb()
                        except Exception:
                            logger.exception("on_grid_update raised")
            except Exception:
                logger.exception("fusion step crashed; continuing")

    def _traversal_loop(self) -> None:
        period = 1.0 / max(0.5, self.config.traversal_stamp_hz)
        while not self._stop_event.is_set():
            start = time.monotonic()
            try:
                latest = self.pose_source.latest_pose()
                if latest is not None:
                    pose, sample_ts = latest
                    age = _now() - sample_ts
                    if age <= self.config.stale_odom_s:
                        self.grid.stamp_traversal(
                            x_w=pose[0], y_w=pose[1], ts=_now(),
                        )
            except Exception:
                logger.exception("traversal stamp failed; continuing")
            elapsed = time.monotonic() - start
            self._stop_event.wait(max(0.0, period - elapsed))

    def _publish_loop(self) -> None:
        publish_period = 1.0 / max(0.1, self.config.publish_hz)
        status_period = 1.0 / max(0.1, self.config.status_hz)
        next_publish = time.monotonic()
        next_status = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_status:
                try:
                    self._publish_status()
                except Exception:
                    logger.exception("status publish failed")
                next_status = now + status_period
            if now >= next_publish:
                try:
                    self._publish_world_driveable()
                except Exception:
                    logger.exception("world_driveable publish failed")
                next_publish = now + publish_period
            sleep_for = min(
                max(0.0, next_publish - time.monotonic()),
                max(0.0, next_status - time.monotonic()),
                0.2,
            )
            self._stop_event.wait(sleep_for)

    # ── Publish helpers ──────────────────────────────────────────────

    def _publish_world_driveable(self) -> None:
        if self._pub_world_driveable is None:
            return
        crop = self.grid.crop_for_publish(self.config.publish_margin_cells)
        if crop is None:
            return
        payload = encode_for_publish(
            crop, pose_source_name=self.pose_source.source_name(),
        )
        self._pub_world_driveable.put(json.dumps(payload).encode("utf-8"))

    def _publish_status(self) -> None:
        if self._pub_world_status is None:
            return
        with self._lock:
            arr_lm = list(self._arr_local_map)
            arr_od = list(self._arr_odom)
            arr_li = list(self._arr_lidar)
            last_lm = self._last_local_map_ts
            last_od = self._last_odom_ts
            last_li = self._last_lidar_ts
            notes = self._notes
            corr = self._last_correction

        latest = self.pose_source.latest_pose()
        if latest is not None:
            pose_world = {
                "x_m": float(latest[0][0]),
                "y_m": float(latest[0][1]),
                "theta_rad": float(latest[0][2]),
            }
        else:
            pose_world = None

        payload = {
            "ts": _now(),
            "session_id": self.grid.session_id,
            "pose_source": self.pose_source.source_name(),
            "pose_world": pose_world,
            "input_rates_hz": {
                "local_map": _rate_from_arrivals(arr_lm),
                "odom": _rate_from_arrivals(arr_od),
                "scan": _rate_from_arrivals(arr_li),
            },
            "input_age_s": {
                "local_map": _age(last_lm),
                "odom": _age(last_od),
                "scan": _age(last_li),
            },
            "grid_cells_allocated": self.grid.n_cells * self.grid.n_cells,
            "grid_cells_observed": self.grid.cells_observed(),
            "grid_cells_traversed": self.grid.cells_traversed(),
            "last_correction": {
                "dx_m": float(corr[0]),
                "dy_m": float(corr[1]),
                "dtheta_rad": float(corr[2]),
            },
            "notes": notes,
        }
        # Stall annotations override any sticky note.
        stall_topic = _stall_topic(
            self.config.input_timeout_s, last_lm, last_od,
        )
        if stall_topic is not None:
            payload["notes"] = f"stall:{stall_topic}"
        self._pub_world_status.put(json.dumps(payload).encode("utf-8"))

    # ── Snapshot for UI ─────────────────────────────────────────────

    def snapshot_for_ui(self) -> Optional[Dict[str, Any]]:
        return self.grid.snapshot_for_ui()

    def status_summary(self) -> Dict[str, Any]:
        with self._lock:
            arr_lm = list(self._arr_local_map)
            arr_od = list(self._arr_odom)
            last_lm = self._last_local_map_ts
            last_od = self._last_odom_ts
            notes = self._notes
        latest = self.pose_source.latest_pose()
        pose = latest[0] if latest is not None else None
        return {
            "session_id": self.grid.session_id,
            "pose": pose,
            "rates": {
                "local_map": _rate_from_arrivals(arr_lm),
                "odom": _rate_from_arrivals(arr_od),
            },
            "ages": {
                "local_map": _age(last_lm),
                "odom": _age(last_od),
            },
            "cells_observed": self.grid.cells_observed(),
            "cells_traversed": self.grid.cells_traversed(),
            "notes": notes,
            "pose_source": self.pose_source.source_name(),
        }


def _rate_from_arrivals(ts_list: List[float]) -> Optional[float]:
    if len(ts_list) < 2:
        return None
    span = ts_list[-1] - ts_list[0]
    if span <= 0:
        return None
    return (len(ts_list) - 1) / span


def _age(ts: float) -> Optional[float]:
    if ts <= 0:
        return None
    return _now() - ts


def _stall_topic(
    timeout_s: float, last_lm: float, last_od: float,
) -> Optional[str]:
    now = _now()
    if last_lm > 0 and (now - last_lm) > timeout_s:
        return "local_map"
    if last_od > 0 and (now - last_od) > timeout_s:
        return "odom"
    return None
