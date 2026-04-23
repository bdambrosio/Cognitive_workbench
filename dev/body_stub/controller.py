"""Controller for the body stub: owns Zenoh session, threads, and state.

Controller exposes a thread-safe, UI-agnostic API. Subscribe callbacks
(invoked on Zenoh's own threads) and the heartbeat/cmd_vel publisher
threads all coordinate through BodyState.lock. The UI polls
snapshot() on its redraw timer; it does not share threads with Zenoh.
"""
from __future__ import annotations

import json
import logging
import threading
import time
import uuid
from typing import Any, Callable, List, Optional, Tuple

from .config import StubConfig
from .state import (
    BodyState, decode_depth, decode_emergency_stop, decode_lidar_scan,
    decode_local_map, decode_motor_state, decode_oakd_imu, decode_odom,
    decode_rgb, decode_status, now_ts,
)
from .transport import open_session

logger = logging.getLogger(__name__)


class StubController:
    def __init__(self, config: StubConfig):
        self.config = config
        self.state = BodyState(router=config.router)
        self._session: Optional[Any] = None
        self._heartbeat_pub: Optional[Any] = None
        self._cmd_vel_pub: Optional[Any] = None
        self._cmd_direct_pub: Optional[Any] = None
        self._subscribers: List[Any] = []

        self._stop_event = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None
        self._cv_thread: Optional[threading.Thread] = None

        # Optional handler for body/sweep/cmd. Set by SweepDock at startup;
        # Zenoh thread invokes it with the parsed JSON dict.
        self._sweep_cmd_handler: Optional[Callable[[dict], None]] = None

        # Monotonic-ts generator for outgoing cmd_vel/cmd_direct payloads.
        # The Pi motor_controller uses payload ts to pick between the two
        # last commands (direct_ts >= twist_ts → direct wins). We must
        # never emit a ts that goes backwards, or a stale stored command
        # on the other topic could re-win precedence. Protected by
        # self.state.lock so publisher thread and supersede calls can't
        # race.
        self._last_sent_ts: float = 0.0

    # ── Connection lifecycle ─────────────────────────────────────────

    def connect(self) -> Tuple[bool, Optional[str]]:
        if self._session is not None:
            return True, None
        try:
            self._session = open_session(self.config.router)
        except Exception as e:
            logger.exception("zenoh open failed")
            return False, f"{type(e).__name__}: {e}"

        try:
            self._declare_publishers()
            self._declare_subscribers()
        except Exception as e:
            logger.exception("publisher/subscriber setup failed")
            self._teardown_zenoh()
            return False, f"{type(e).__name__}: {e}"

        self._stop_event.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, name="body-stub-heartbeat", daemon=True,
        )
        self._cv_thread = threading.Thread(
            target=self._cmd_loop, name="body-stub-cmd", daemon=True,
        )
        self._hb_thread.start()
        self._cv_thread.start()

        with self.state.lock:
            self.state.connected = True
        return True, None

    def disconnect(self) -> None:
        # Best-effort neutralize any stored command on the Pi before the
        # publisher thread stops. Matters when the stub is the only
        # commander (no watchdog to e-stop on heartbeat loss).
        self._supersede_both_zero()
        with self.state.lock:
            self.state.live_command = False
            self.state.connected = False
        self._stop_event.set()
        for t in (self._hb_thread, self._cv_thread):
            if t is not None and t.is_alive():
                t.join(timeout=1.0)
        self._hb_thread = None
        self._cv_thread = None
        self._teardown_zenoh()

    def shutdown(self) -> None:
        self.disconnect()

    def _teardown_zenoh(self) -> None:
        for sub in self._subscribers:
            try:
                sub.undeclare()
            except Exception:
                pass
        self._subscribers.clear()
        for pub in (
            self._heartbeat_pub, self._cmd_vel_pub, self._cmd_direct_pub,
        ):
            if pub is not None:
                try:
                    pub.undeclare()
                except Exception:
                    pass
        self._heartbeat_pub = None
        self._cmd_vel_pub = None
        self._cmd_direct_pub = None
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None

    # ── Zenoh wiring ─────────────────────────────────────────────────

    def _declare_publishers(self) -> None:
        assert self._session is not None
        t = self.config.topics
        self._heartbeat_pub = self._session.declare_publisher(t.heartbeat)
        self._cmd_vel_pub = self._session.declare_publisher(t.cmd_vel)
        self._cmd_direct_pub = self._session.declare_publisher(t.cmd_direct)

    def _declare_subscribers(self) -> None:
        assert self._session is not None
        t = self.config.topics
        pairs = [
            (t.status, self._on_status),
            (t.emergency_stop, self._on_emergency_stop),
            (t.odom, self._on_odom),
            (t.motor_state, self._on_motor_state),
            (t.lidar_scan, self._on_lidar_scan),
            (t.oakd_imu, self._on_oakd_imu),
            (t.oakd_depth, self._on_oakd_depth),
            (t.oakd_rgb, self._on_oakd_rgb),
            (t.local_map, self._on_local_map),
            (t.sweep_cmd, self._on_sweep_cmd),
        ]
        for key, cb in pairs:
            sub = self._session.declare_subscriber(key, cb)
            self._subscribers.append(sub)

    # ── Subscribe callbacks (Zenoh threads) ──────────────────────────

    def _payload_bytes(self, sample: Any) -> bytes:
        try:
            return bytes(sample.payload.to_bytes())
        except AttributeError:
            return bytes(sample.payload)

    def _on_status(self, sample: Any) -> None:
        msg = decode_status(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.status = msg
            self.state.status_ts = now_ts()

    def _on_emergency_stop(self, sample: Any) -> None:
        msg = decode_emergency_stop(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.emergency_stop = msg
            self.state.emergency_ts = now_ts()

    def _on_odom(self, sample: Any) -> None:
        msg = decode_odom(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.odom = msg
            self.state.odom_ts = now_ts()

    def _on_motor_state(self, sample: Any) -> None:
        msg = decode_motor_state(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.motor_state = msg
            self.state.motor_ts = now_ts()

    def _on_lidar_scan(self, sample: Any) -> None:
        msg = decode_lidar_scan(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.lidar_scan = msg
            self.state.lidar_ts = now_ts()

    def _on_oakd_imu(self, sample: Any) -> None:
        msg = decode_oakd_imu(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.oakd_imu = msg
            self.state.oakd_imu_ts = now_ts()

    def _on_oakd_depth(self, sample: Any) -> None:
        msg = decode_depth(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.depth_format = msg.get("format", "")
            self.state.depth_width = int(msg.get("width", 0) or 0)
            self.state.depth_height = int(msg.get("height", 0) or 0)
            self.state.depth_image = msg.get("image")
            self.state.depth_ts = now_ts()

    def _on_local_map(self, sample: Any) -> None:
        msg = decode_local_map(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            self.state.local_map_grid = msg["grid"]
            self.state.local_map_meta = msg["meta"]
            self.state.local_map_driveable = msg.get("driveable")
            ts = now_ts()
            self.state.local_map_ts = ts
            self.state.local_map_arrivals.append(ts)

    def _on_sweep_cmd(self, sample: Any) -> None:
        try:
            data = json.loads(self._payload_bytes(sample).decode("utf-8"))
        except Exception:
            logger.warning("sweep_cmd: bad JSON payload")
            return
        if not isinstance(data, dict):
            return
        handler = self._sweep_cmd_handler
        if handler is None:
            logger.debug("sweep_cmd received but no handler registered")
            return
        try:
            handler(data)
        except Exception:
            logger.exception("sweep_cmd handler raised")

    def set_sweep_cmd_handler(self, handler: Optional[Callable[[dict], None]]) -> None:
        """Register a callback for body/sweep/cmd messages. Invoked from
        the Zenoh subscribe thread; the callee is responsible for
        marshalling to its own thread (e.g. via a Qt queued signal).
        """
        self._sweep_cmd_handler = handler

    def _on_oakd_rgb(self, sample: Any) -> None:
        msg = decode_rgb(self._payload_bytes(sample))
        if msg is None:
            return
        with self.state.lock:
            # Correlate against in-flight request; ignore unknown ids.
            pending = self.state.pending_rgb_request_id
            if pending and msg["request_id"] and msg["request_id"] != pending:
                logger.debug(
                    f"rgb reply id {msg['request_id']} != pending {pending}; dropping"
                )
                return
            self.state.rgb_request_id = msg["request_id"]
            self.state.rgb_width = msg["width"]
            self.state.rgb_height = msg["height"]
            self.state.rgb_jpeg = msg["jpeg"]
            self.state.rgb_error = msg["error"]
            self.state.rgb_ts = now_ts()
            self.state.pending_rgb_request_id = None

    # ── Publisher threads ────────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        """Publish body/heartbeat whenever connected.

        Deliberately not gated on live_command: per body_project_spec
        §8.2, the desktop agent publishes heartbeat "whenever she
        expects the robot to be active" — i.e. whenever connected.
        Gating on live_command created a deadlock where the watchdog
        e-stopped the robot for lack of heartbeat, which in turn
        disabled the UI's drive controls.
        """
        period = 1.0 / max(0.5, self.config.heartbeat_hz)
        while not self._stop_event.is_set():
            start = time.monotonic()
            try:
                with self.state.lock:
                    connected = self.state.connected
                    self.state.heartbeat_seq += 1
                    seq = self.state.heartbeat_seq
                if connected and self._heartbeat_pub is not None:
                    payload = json.dumps({"seq": seq, "ts": now_ts()}).encode("utf-8")
                    self._heartbeat_pub.put(payload)
            except Exception:
                logger.exception("heartbeat publish failed")
            elapsed = time.monotonic() - start
            self._stop_event.wait(max(0.0, period - elapsed))

    def _next_ts_locked(self) -> float:
        """Return a ts strictly greater than any previously emitted ts.

        Must be called with self.state.lock held. Protects the Pi's
        ts-based precedence logic against our own local clock going
        backwards (NTP step, VM suspend, etc).
        """
        t = now_ts()
        if t <= self._last_sent_ts:
            t = self._last_sent_ts + 1e-6
        self._last_sent_ts = t
        return t

    def _cmd_loop(self) -> None:
        """Publish the active command topic at cmd_vel_hz.

        Mode dispatch is read from state.cmd_mode each cycle; this makes
        the publisher the single source of truth for which topic is
        "live" at any instant. Supersession of the inactive topic on
        mode switch is handled out-of-band by _supersede_both_locked.
        """
        period = 1.0 / max(0.5, self.config.cmd_vel_hz)
        while not self._stop_event.is_set():
            start = time.monotonic()
            try:
                with self.state.lock:
                    live = self.state.live_command
                    connected = self.state.connected
                    mode = self.state.cmd_mode
                    lin, ang = self.state.last_cmd_vel
                    left, right = self.state.last_cmd_direct
                    timeout_ms = self.config.cmd_vel_timeout_ms
                    ts = self._next_ts_locked() if (live and connected) else None
                if ts is None:
                    pass
                elif mode == "cmd_direct" and self._cmd_direct_pub is not None:
                    payload = json.dumps({
                        "ts": ts,
                        "left": left, "right": right,
                        "timeout_ms": timeout_ms,
                    }).encode("utf-8")
                    self._cmd_direct_pub.put(payload)
                elif self._cmd_vel_pub is not None:
                    payload = json.dumps({
                        "ts": ts,
                        "linear": lin, "angular": ang,
                        "timeout_ms": timeout_ms,
                    }).encode("utf-8")
                    self._cmd_vel_pub.put(payload)
            except Exception:
                logger.exception("cmd publish failed")
            elapsed = time.monotonic() - start
            self._stop_event.wait(max(0.0, period - elapsed))

    # ── UI-facing API ────────────────────────────────────────────────

    def set_live_command(self, enable: bool) -> None:
        with self.state.lock:
            self.state.live_command = bool(enable)
            if not enable:
                self.state.last_cmd_vel = (0.0, 0.0)
                self.state.last_cmd_direct = (0.0, 0.0)

    def set_cmd_vel(self, linear: float, angular: float) -> None:
        with self.state.lock:
            self.state.last_cmd_vel = (float(linear), float(angular))

    def set_cmd_direct(self, left: float, right: float) -> None:
        with self.state.lock:
            self.state.last_cmd_direct = (float(left), float(right))

    def set_cmd_mode(self, mode: str) -> None:
        """Switch active publisher between 'cmd_vel' and 'cmd_direct'.

        Supersedes the leaving topic with an explicit zero so the Pi's
        ts-based precedence cannot keep a stale command from the old
        topic winning over fresh commands on the new one.
        """
        if mode not in ("cmd_vel", "cmd_direct"):
            raise ValueError(f"unknown cmd mode: {mode!r}")
        with self.state.lock:
            prev = self.state.cmd_mode
            if prev == mode:
                return
            self.state.cmd_mode = mode
            # Clear both sets on mode switch; the user re-enters values
            # on the new slider set.
            self.state.last_cmd_vel = (0.0, 0.0)
            self.state.last_cmd_direct = (0.0, 0.0)
        self._supersede_both_zero()

    def stop_all(self) -> None:
        """ALL-STOP. Zero both command sets, drop live_command, and
        publish zeros on *both* cmd topics so neither can win precedence
        on the Pi after this call returns.
        """
        with self.state.lock:
            self.state.last_cmd_vel = (0.0, 0.0)
            self.state.last_cmd_direct = (0.0, 0.0)
            self.state.live_command = False
        self._supersede_both_zero()

    def _supersede_both_zero(self) -> None:
        """Publish a zero command on both cmd_vel and cmd_direct with a
        shared monotonic ts. At tie ts, the Pi's `direct_ts >= twist_ts`
        rule makes direct win — but both payloads are zero so the motion
        outcome is identical, and the next live publish gets a strictly
        larger ts that resolves cleanly.
        """
        if self._cmd_vel_pub is None or self._cmd_direct_pub is None:
            return
        with self.state.lock:
            ts = self._next_ts_locked()
            timeout_ms = self.config.cmd_vel_timeout_ms
        try:
            self._cmd_vel_pub.put(json.dumps({
                "ts": ts, "linear": 0.0, "angular": 0.0,
                "timeout_ms": timeout_ms,
            }).encode("utf-8"))
            self._cmd_direct_pub.put(json.dumps({
                "ts": ts, "left": 0.0, "right": 0.0,
                "timeout_ms": timeout_ms,
            }).encode("utf-8"))
        except Exception:
            logger.exception("supersede publish failed")

    def request_rgb(self) -> Optional[str]:
        """Publish body/oakd/config capture_rgb; returns request_id or None."""
        if self._session is None:
            return None
        req_id = str(uuid.uuid4())
        payload = json.dumps({
            "action": "capture_rgb", "request_id": req_id,
        }).encode("utf-8")
        try:
            self._session.put(self.config.topics.oakd_config, payload)
        except Exception:
            logger.exception("oakd_config put failed")
            return None
        with self.state.lock:
            self.state.pending_rgb_request_id = req_id
            self.state.pending_rgb_ts = now_ts()
        return req_id
