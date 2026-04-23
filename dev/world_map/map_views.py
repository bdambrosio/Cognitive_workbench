"""Map view widgets. Copied verbatim from dev/body_stub/ui_qt.py with
minor renaming so the world-map fuser is self-contained.

If a third consumer ever appears, promote these to dev/shared/widgets/.
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QSizePolicy, QWidget


def _turbo_rgb(x: np.ndarray) -> np.ndarray:
    """x: float32 in [0,1], shape (h,w). Returns uint8 (h,w,3) RGB.
    Anton Mikhailov's polynomial Turbo approximation, Apache 2.0.
    """
    r = 0.1357 + x*(4.5744 + x*(-42.3335 + x*(130.8988 + x*(-152.6574 + x*59.9032))))
    g = 0.0914 + x*(2.1915 + x*(  4.9271 + x*(-14.1846 + x*(  4.2755 + x* 2.8289))))
    b = 0.1067 + x*(12.5989 + x*(-60.1846 + x*(109.2364 + x*(-88.7840 + x*27.0060))))
    rgb = np.stack([r, g, b], axis=-1)
    return np.clip(rgb * 255.0, 0, 255).astype(np.uint8)


class WorldHeightView(QWidget):
    """Top-down render of a height grid in world frame.

    Coordinate convention: world +x → up (forward), world +y → left.
    Robot pose is drawn as a triangle pointing in the heading direction.
    """

    DEFAULT_MAX_HEIGHT_M = 2.2

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        stale_s: float = 2.0,
        max_height_m: float = DEFAULT_MAX_HEIGHT_M,
    ):
        super().__init__(parent)
        self.setMinimumSize(280, 280)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        self._grid: Optional[np.ndarray] = None
        self._meta: Optional[dict] = None
        self._ts: float = 0.0
        self._stale_s = stale_s
        self._max_height_m = max_height_m
        self._pose: Optional[tuple] = None  # (x_m, y_m, theta_rad)

    def update_map(
        self, grid: Optional[np.ndarray],
        meta: Optional[dict], ts: float,
        pose: Optional[tuple] = None,
    ) -> None:
        self._grid = grid
        self._meta = meta
        self._ts = ts
        self._pose = pose
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor(10, 10, 10))

            if self._grid is None or self._meta is None:
                p.setPen(QColor(160, 160, 160))
                p.drawText(10, 16, "no world map (drive to populate)")
                return

            grid = self._grid
            nx, ny = grid.shape
            res = float(self._meta.get("resolution_m", 0.0))
            origin_x = float(self._meta.get("origin_x_m", 0.0))
            origin_y = float(self._meta.get("origin_y_m", 0.0))
            if res <= 0.0:
                p.setPen(QColor(255, 100, 100))
                p.drawText(10, 16, f"bad resolution_m={res}")
                return

            display = grid[::-1, ::-1]
            valid = ~np.isnan(display)
            norm = np.zeros_like(display, dtype=np.float32)
            np.divide(display, self._max_height_m, out=norm, where=valid)
            np.clip(norm, 0.0, 1.0, out=norm)
            rgb = _turbo_rgb(norm)
            rgb[~valid] = (16, 16, 16)
            rgb = np.ascontiguousarray(rgb)
            qimg = QImage(rgb.data, ny, nx, 3 * ny,
                          QImage.Format.Format_RGB888).copy()

            map_w_m = ny * res
            map_h_m = nx * res
            margin = 6
            avail_w = max(1, w - 2 * margin)
            avail_h = max(1, h - 2 * margin)
            scale = min(avail_w / map_w_m, avail_h / map_h_m)
            draw_w = max(1, int(map_w_m * scale))
            draw_h = max(1, int(map_h_m * scale))
            ox = margin + (avail_w - draw_w) // 2
            oy = margin + (avail_h - draw_h) // 2
            scaled = qimg.scaled(
                draw_w, draw_h,
                Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
            p.drawImage(ox, oy, scaled)

            p.setPen(QPen(QColor(70, 70, 70), 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(ox, oy, draw_w, draw_h)

            # World origin (anchor) marker.
            self._draw_world_marker(
                p, 0.0, 0.0, 0.0, ox, oy, draw_w, draw_h,
                nx, ny, res, origin_x, origin_y,
                color=QColor(140, 140, 200), label="anchor",
            )

            # Robot pose marker.
            if self._pose is not None:
                self._draw_world_marker(
                    p, self._pose[0], self._pose[1], self._pose[2],
                    ox, oy, draw_w, draw_h,
                    nx, ny, res, origin_x, origin_y,
                    color=QColor(255, 255, 255), label=None,
                )

            age = time.time() - self._ts if self._ts > 0 else 0.0
            if age > self._stale_s:
                p.fillRect(ox, oy, draw_w, draw_h, QColor(0, 0, 0, 140))
                p.setPen(QColor(255, 200, 80))
                p.drawText(
                    ox + 6, oy + 16,
                    f"stale ({age:.1f}s) — fuser idle?",
                )

            p.setPen(QColor(180, 180, 180))
            p.drawText(
                margin, margin + 10,
                f"0–{self._max_height_m:.1f} m (turbo)",
            )
        finally:
            p.end()

    def _draw_world_marker(
        self, p, x_w, y_w, theta_w, ox, oy, draw_w, draw_h,
        nx, ny, res, origin_x, origin_y, *, color, label,
    ):
        # World (x_w, y_w) → cell (i, j) in the (unflipped) world grid.
        i = (x_w - origin_x) / res  # along +x (forward)
        j = (y_w - origin_y) / res  # along +y (left)
        # Display is flipped: r_disp = (nx-1) - i, c_disp = (ny-1) - j.
        r_disp = (nx - 1) - i
        c_disp = (ny - 1) - j
        cell_px_w = draw_w / ny
        cell_px_h = draw_h / nx
        rx = ox + (c_disp + 0.5) * cell_px_w
        ry = oy + (r_disp + 0.5) * cell_px_h
        off_grid = (i < 0 or i >= nx or j < 0 or j >= ny)
        if off_grid:
            rx = max(ox, min(ox + draw_w - 1, rx))
            ry = max(oy, min(oy + draw_h - 1, ry))
            color = QColor(255, 200, 80)
        p.setPen(QPen(color, 1))
        p.setBrush(color)
        # Triangle pointing in direction theta. In display coords:
        # forward (theta=0, world +x) is up (-y in screen). theta CCW
        # in world rotates the triangle CCW in display.
        size = 9.0
        # Local triangle (forward = (0,-size)), rotated by -theta to
        # match display rotation (since +x maps to -y_screen).
        ct, st = np.cos(-theta_w), np.sin(-theta_w)
        def rot(px, py):
            return (ct * px - st * py, st * px + ct * py)
        p0 = rot(0.0, -size)
        p1 = rot(-size * 0.7, size * 0.7)
        p2 = rot(size * 0.7, size * 0.7)
        tri = QPolygonF([
            QPointF(rx + p0[0], ry + p0[1]),
            QPointF(rx + p1[0], ry + p1[1]),
            QPointF(rx + p2[0], ry + p2[1]),
        ])
        p.drawPolygon(tri)
        if label:
            p.setPen(color)
            p.drawText(int(rx) + 12, int(ry) + 4, label)


class WorldDriveableView(QWidget):
    """Top-down render of the driveable layer in world frame."""

    COLOR_CLEAR = (60, 170, 90)
    COLOR_BLOCKED = (180, 60, 60)
    COLOR_UNKNOWN = (60, 60, 60)
    COLOR_ABSENT_BG = (10, 10, 10)

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        stale_s: float = 2.0,
    ):
        super().__init__(parent)
        self.setMinimumSize(280, 280)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        self._drive: Optional[np.ndarray] = None
        self._meta: Optional[dict] = None
        self._ts: float = 0.0
        self._stale_s = stale_s
        self._pose: Optional[tuple] = None

    def update_map(
        self, drive: Optional[np.ndarray],
        meta: Optional[dict], ts: float,
        pose: Optional[tuple] = None,
    ) -> None:
        self._drive = drive
        self._meta = meta
        self._ts = ts
        self._pose = pose
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor(*self.COLOR_ABSENT_BG))

            if self._drive is None or self._meta is None:
                p.setPen(QColor(160, 160, 160))
                p.drawText(10, 16, "no driveable layer (drive to populate)")
                return

            drive = self._drive
            nx, ny = drive.shape
            res = float(self._meta.get("resolution_m", 0.0))
            origin_x = float(self._meta.get("origin_x_m", 0.0))
            origin_y = float(self._meta.get("origin_y_m", 0.0))
            if res <= 0.0:
                p.setPen(QColor(255, 100, 100))
                p.drawText(10, 16, f"bad resolution_m={res}")
                return

            display = drive[::-1, ::-1]
            rgb = np.empty((nx, ny, 3), dtype=np.uint8)
            rgb[...] = self.COLOR_UNKNOWN
            rgb[display == 1] = self.COLOR_CLEAR
            rgb[display == 0] = self.COLOR_BLOCKED
            rgb = np.ascontiguousarray(rgb)
            qimg = QImage(rgb.data, ny, nx, 3 * ny,
                          QImage.Format.Format_RGB888).copy()

            map_w_m = ny * res
            map_h_m = nx * res
            margin = 6
            avail_w = max(1, w - 2 * margin)
            avail_h = max(1, h - 2 * margin)
            scale = min(avail_w / map_w_m, avail_h / map_h_m)
            draw_w = max(1, int(map_w_m * scale))
            draw_h = max(1, int(map_h_m * scale))
            ox = margin + (avail_w - draw_w) // 2
            oy = margin + (avail_h - draw_h) // 2
            scaled = qimg.scaled(
                draw_w, draw_h,
                Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
            p.drawImage(ox, oy, scaled)

            p.setPen(QPen(QColor(70, 70, 70), 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(ox, oy, draw_w, draw_h)

            # Anchor + robot markers (world frame).
            self._draw_world_marker(
                p, 0.0, 0.0, 0.0, ox, oy, draw_w, draw_h,
                nx, ny, res, origin_x, origin_y,
                color=QColor(140, 140, 200), label="anchor",
            )
            if self._pose is not None:
                self._draw_world_marker(
                    p, self._pose[0], self._pose[1], self._pose[2],
                    ox, oy, draw_w, draw_h,
                    nx, ny, res, origin_x, origin_y,
                    color=QColor(255, 255, 255), label=None,
                )

            age = time.time() - self._ts if self._ts > 0 else 0.0
            if age > self._stale_s:
                p.fillRect(ox, oy, draw_w, draw_h, QColor(0, 0, 0, 140))
                p.setPen(QColor(255, 200, 80))
                p.drawText(
                    ox + 6, oy + 16,
                    f"stale ({age:.1f}s) — fuser idle?",
                )

            p.setPen(QColor(180, 180, 180))
            p.drawText(margin, margin + 10, "clear / blocked / unknown")
        finally:
            p.end()

    _draw_world_marker = WorldHeightView._draw_world_marker
