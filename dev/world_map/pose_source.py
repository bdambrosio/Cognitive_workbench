"""PoseSource interface + v1 odom-only implementation.

Designed so v1.1 can drop in an `OdomPlusScanMatch` wrapper without
changes to fusion or publish paths. See docs/world_map_spec.md §6.1.
"""
from __future__ import annotations

import logging
import math
from collections import deque
from threading import RLock
from typing import Deque, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


Pose = Tuple[float, float, float]  # (x_m, y_m, theta_rad)


class PoseSource:
    """Abstract pose source. Implementations MUST be thread-safe."""

    def pose_at(self, ts: float) -> Optional[Pose]:
        raise NotImplementedError

    def latest_pose(self) -> Optional[Tuple[Pose, float]]:
        """Return (pose, ts) of the newest sample, or None if none yet."""
        raise NotImplementedError

    def cov_at(self, ts: float) -> Optional[np.ndarray]:
        return None

    def notify_correction(
        self, dx: float, dy: float, dtheta: float, ts: float,
    ) -> None:
        """v1.1 hook for scan-match corrections. v1: no-op."""
        pass

    def source_name(self) -> str:
        return "unknown"


def _unwrap_pair(a: float, b: float) -> Tuple[float, float]:
    """Return (a, b') where b' is within π of a. Preserves a."""
    d = b - a
    while d > math.pi:
        b -= 2.0 * math.pi
        d = b - a
    while d < -math.pi:
        b += 2.0 * math.pi
        d = b - a
    return a, b


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class OdomPose(PoseSource):
    """Linear-interpolated pose from body/odom.

    Keeps a small ring buffer of recent odom samples. pose_at(ts) bisects
    for the two bracketing samples and linearly interpolates. θ is
    interpolated wrap-safely by unwrapping the two bracketing angles to
    a common branch first.

    Also accepts a world-frame offset (set on reset) so that "current
    body pose at reset" becomes identity in the world frame.
    """

    def __init__(self, buffer_seconds: float = 2.0):
        self._lock = RLock()
        # (ts, x, y, theta) tuples, sorted ascending by ts.
        self._buf: Deque[Tuple[float, float, float, float]] = deque(maxlen=256)
        self._buf_seconds = buffer_seconds
        # World = Odom after (inv) offset: (x_w, y_w, θ_w) = T_offset · (x_o, y_o, θ_o)
        # Stored as an SE(2) triple applied as a body-frame rototranslation:
        # world_pose = offset ⊕ odom_pose.
        self._off_x = 0.0
        self._off_y = 0.0
        self._off_theta = 0.0

    # ── Ingest ───────────────────────────────────────────────────────

    def update(self, ts: float, x: float, y: float, theta: float) -> None:
        with self._lock:
            if self._buf and ts <= self._buf[-1][0]:
                # Ignore out-of-order samples; ts from a lagging publisher
                # would otherwise corrupt interpolation.
                return
            self._buf.append((ts, float(x), float(y), float(theta)))
            # Trim by time window to keep RAM bounded under fast pubs.
            cutoff = self._buf[-1][0] - self._buf_seconds
            while len(self._buf) > 2 and self._buf[0][0] < cutoff:
                self._buf.popleft()

    def rebind_world_to_current(self) -> Optional[Pose]:
        """Set world origin = current body pose. Returns the new offset
        as (off_x, off_y, off_theta) in odom frame, or None if no sample.
        """
        with self._lock:
            if not self._buf:
                return None
            _ts, x, y, theta = self._buf[-1]
            # We want world_pose(odom_pose_now) = (0, 0, 0).
            # Apply world_pose = inv(offset_SE2) · odom_pose, with the
            # offset stored as the current odom pose:
            self._off_x = x
            self._off_y = y
            self._off_theta = theta
            return (x, y, theta)

    # ── Query ────────────────────────────────────────────────────────

    def latest_pose(self) -> Optional[Tuple[Pose, float]]:
        with self._lock:
            if not self._buf:
                return None
            ts, x, y, theta = self._buf[-1]
            return self._to_world(x, y, theta), ts

    def pose_at(self, ts: float) -> Optional[Pose]:
        with self._lock:
            n = len(self._buf)
            if n == 0:
                return None
            if n == 1:
                _, x, y, theta = self._buf[0]
                return self._to_world(x, y, theta)

            # Clamp at ends to the nearest sample if within a small grace
            # window (half the mean inter-arrival). Outside, return None.
            first_ts = self._buf[0][0]
            last_ts = self._buf[-1][0]
            if ts < first_ts:
                grace = (last_ts - first_ts) / max(1, n - 1) * 0.5
                if first_ts - ts <= grace:
                    _, x, y, theta = self._buf[0]
                    return self._to_world(x, y, theta)
                return None
            if ts > last_ts:
                grace = (last_ts - first_ts) / max(1, n - 1) * 0.5
                if ts - last_ts <= grace:
                    _, x, y, theta = self._buf[-1]
                    return self._to_world(x, y, theta)
                return None

            # Binary search for bracketing pair.
            lo, hi = 0, n - 1
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if self._buf[mid][0] <= ts:
                    lo = mid
                else:
                    hi = mid
            t0, x0, y0, th0 = self._buf[lo]
            t1, x1, y1, th1 = self._buf[hi]
            if t1 == t0:
                return self._to_world(x1, y1, th1)
            alpha = (ts - t0) / (t1 - t0)
            x = x0 + alpha * (x1 - x0)
            y = y0 + alpha * (y1 - y0)
            ua, ub = _unwrap_pair(th0, th1)
            theta = _wrap(ua + alpha * (ub - ua))
            return self._to_world(x, y, theta)

    # ── World frame ──────────────────────────────────────────────────

    def _to_world(
        self, x_o: float, y_o: float, th_o: float,
    ) -> Pose:
        """Transform an odom-frame pose into world frame.

        world_pose = inv(T_offset_in_odom) · odom_pose
        """
        dx = x_o - self._off_x
        dy = y_o - self._off_y
        c, s = math.cos(-self._off_theta), math.sin(-self._off_theta)
        x_w = c * dx - s * dy
        y_w = s * dx + c * dy
        th_w = _wrap(th_o - self._off_theta)
        return (x_w, y_w, th_w)

    def source_name(self) -> str:
        return "odom"

    def buffer_span(self) -> Optional[Tuple[float, float]]:
        with self._lock:
            if not self._buf:
                return None
            return self._buf[0][0], self._buf[-1][0]
