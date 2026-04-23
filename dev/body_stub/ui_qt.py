"""PyQt6 implementation of the body-stub UI.

Layout:
    [router] [Connect] [Live cmd] [status-line]
    +---------------+---------------+
    | Status text   | RGB (on req.) |
    +---------------+---------------+
    | Depth (color) | Lidar (polar) |
    +---------------+---------------+
    [linear] [angular] [Apply] [All-Stop] [Request RGB]

Depth is colormapped with a compact turbo-polynomial approximation
(no matplotlib dep). Lidar is rendered on a custom QWidget with
QPainter; angle_min/angle_increment are taken from the message if
present, otherwise a uniform 0..2π distribution is assumed.
"""
from __future__ import annotations

import json
import logging
import math
import os
import sys
import tempfile
import time
from typing import Optional

import numpy as np

from PyQt6.QtCore import Qt, QPointF, QProcess, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import (
    QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF,
)
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDockWidget, QDoubleSpinBox,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QPlainTextEdit, QPushButton, QSizePolicy, QSpinBox,
    QTabWidget, QTextBrowser, QVBoxLayout, QWidget,
)

from .jill_client import JillClient
from .ui_base import StubUI
# sweep_dock is imported lazily inside BodyStubWindow._build_ui to break
# the circular import: sweep_dock pulls in LocalMapView/DriveableView
# from this module.

logger = logging.getLogger(__name__)


# ── Depth colormap (turbo polynomial, Anton Mikhailov, Apache 2.0) ──

def _turbo_rgb(x: np.ndarray) -> np.ndarray:
    """x: float32 in [0,1], shape (h,w). Returns uint8 (h,w,3) RGB."""
    r = 0.1357 + x*(4.5744 + x*(-42.3335 + x*(130.8988 + x*(-152.6574 + x*59.9032))))
    g = 0.0914 + x*(2.1915 + x*(  4.9271 + x*(-14.1846 + x*(  4.2755 + x* 2.8289))))
    b = 0.1067 + x*(12.5989 + x*(-60.1846 + x*(109.2364 + x*(-88.7840 + x*27.0060))))
    rgb = np.stack([r, g, b], axis=-1)
    return np.clip(rgb * 255.0, 0, 255).astype(np.uint8)


def depth_to_pixmap(
    depth_mm: np.ndarray,
    *,
    max_range_mm: int = 6000,
    target_w: int = 320,
) -> QPixmap:
    """Colormap a uint16 depth image (mm). Zeros treated as invalid (black)."""
    valid = depth_mm > 0
    norm = np.zeros(depth_mm.shape, dtype=np.float32)
    if valid.any():
        clipped = np.clip(depth_mm.astype(np.float32), 0, max_range_mm)
        norm = clipped / float(max_range_mm)
    rgb = _turbo_rgb(norm)
    rgb[~valid] = 0
    h, w, _ = rgb.shape
    # Pack contiguously and build QImage; copy() so we don't hand Qt a
    # view into a numpy buffer that may be freed.
    rgb = np.ascontiguousarray(rgb)
    img = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
    if w < target_w:
        scale = target_w / float(w)
        img = img.scaled(
            int(w * scale), int(h * scale),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        )
    return QPixmap.fromImage(img)


# ── Host-metrics panel (body/status.host extension) ────────────────

_CHIP_DEFS = [
    ("under_voltage_now",       "UV now",  True),
    ("throttled_now",           "THR now", True),
    ("arm_freq_capped_now",     "CAP now", True),
    ("soft_temp_limit_now",     "STL now", True),
    ("under_voltage_occurred",  "UV ever", False),
    ("throttled_occurred",      "THR ever", False),
    ("arm_freq_capped_occurred","CAP ever", False),
    ("soft_temp_limit_occurred","STL ever", False),
]


class HostPanel(QWidget):
    """Renders body/status.host (Pi thermal/power hints; diagnostic only)."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(2)

        mono = QFont("Monospace", 9)
        self.temp_label = QLabel("CPU: —")
        self.volts_label = QLabel("SoC: —")
        self.volts_label.setToolTip(
            "SoC core voltage (vcgencmd measure_volts core). "
            "Not the 5 V USB input."
        )
        self.throttled_label = QLabel("throttled: —")
        for w in (self.temp_label, self.volts_label, self.throttled_label):
            w.setFont(mono)
        row1 = QHBoxLayout()
        row1.setSpacing(12)
        row1.addWidget(self.temp_label)
        row1.addWidget(self.volts_label)
        row1.addStretch(1)
        row1.addWidget(self.throttled_label)
        v.addLayout(row1)

        self.chips = {}
        row2 = QHBoxLayout()
        row2.setSpacing(4)
        chip_font = QFont("Monospace", 8)
        for key, label, _severe in _CHIP_DEFS:
            chip = QLabel(label)
            chip.setAlignment(Qt.AlignmentFlag.AlignCenter)
            chip.setFont(chip_font)
            chip.setMinimumWidth(58)
            self.chips[key] = chip
            row2.addWidget(chip)
        row2.addStretch(1)
        v.addLayout(row2)

        self.set_absent()

    def _style_chip(self, chip: QLabel, active: bool, *, severe: bool) -> None:
        if active:
            bg = "#aa2222" if severe else "#d08a2e"
            fg = "white"
        else:
            bg = "#2a2a2a"
            fg = "#888"
        chip.setStyleSheet(
            f"background:{bg};color:{fg};"
            f"padding:1px 6px;border-radius:4px;"
        )

    def set_absent(self) -> None:
        self.temp_label.setText("host metrics disabled")
        self.volts_label.setText("")
        self.throttled_label.setText("")
        for chip in self.chips.values():
            self._style_chip(chip, False, severe=False)

    def update_host(self, host: Optional[dict]) -> None:
        if not isinstance(host, dict):
            self.set_absent()
            return
        t = host.get("cpu_temp_c")
        v = host.get("core_volts")
        thr = host.get("throttled")
        self.temp_label.setText(
            f"CPU: {t:5.1f} °C" if isinstance(t, (int, float)) else "CPU: —"
        )
        self.volts_label.setText(
            f"SoC: {v:6.3f} V" if isinstance(v, (int, float)) else "SoC: —"
        )
        self.throttled_label.setText(
            f"throttled: {thr}" if thr else "throttled: —"
        )
        for key, _label, severe in _CHIP_DEFS:
            active = bool(host.get(key, False))
            self._style_chip(self.chips[key], active, severe=severe)


# ── Lidar polar view ────────────────────────────────────────────────

class LidarView(QWidget):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(240, 240)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._scan: Optional[dict] = None
        # Display cap, not a validity cap: the scan's own range_max (if
        # smaller) still wins for sample validity. Set lower than typical
        # hardware range_max so close features render ~2x larger.
        self._max_range_m: float = 3.0

    def update_scan(self, scan: Optional[dict]) -> None:
        self._scan = scan
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor(12, 12, 12))
            cx, cy = w / 2, h / 2
            radius = min(w, h) / 2 - 6

            # range rings
            p.setPen(QPen(QColor(60, 60, 60), 1))
            for frac in (0.25, 0.5, 0.75, 1.0):
                r = radius * frac
                p.drawEllipse(int(cx - r), int(cy - r), int(2 * r), int(2 * r))
            # heading tick (+x = forward)
            p.setPen(QPen(QColor(90, 90, 90), 1))
            p.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))
            p.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))

            if self._scan is None:
                p.setPen(QColor(160, 160, 160))
                p.drawText(10, 16, "no lidar data")
                return

            ranges = self._scan.get("ranges") or []
            n = len(ranges)
            if n == 0:
                p.setPen(QColor(160, 160, 160))
                p.drawText(10, 16, "empty scan")
                return
            angle_min = float(self._scan.get("angle_min", 0.0))
            angle_inc = self._scan.get("angle_increment")
            if angle_inc is None:
                angle_inc = (2.0 * math.pi) / n
            else:
                angle_inc = float(angle_inc)
            scan_max = float(self._scan.get("range_max", self._max_range_m))
            if scan_max <= 0.0:
                scan_max = self._max_range_m
            display_max = min(scan_max, self._max_range_m)

            p.setPen(QPen(QColor(120, 220, 255), 2))
            for i, r in enumerate(ranges):
                try:
                    rv = float(r)
                except Exception:
                    continue
                if not math.isfinite(rv) or rv <= 0.0 or rv > scan_max:
                    continue
                a = angle_min + i * angle_inc
                # Body frame: 0 rad = forward (+x), +π/2 = robot-left (+y).
                # Screen: +y is down. Bird's-eye view → rotate 90° CCW so
                # forward points UP on screen and robot-left points LEFT.
                # Cells beyond display_max clamp to the outer ring rather
                # than vanishing, so you can still see where things are,
                # just not how far.
                scale = min(rv / display_max, 1.0) * radius
                px = cx - scale * math.sin(a)
                py = cy - scale * math.cos(a)
                p.drawPoint(int(px), int(py))
            # forward tick marker
            p.setPen(QPen(QColor(180, 220, 255), 1))
            p.drawText(int(cx) + 4, int(cy - radius) + 12, "F")
        finally:
            p.end()


# ── Local 2.5D map view ─────────────────────────────────────────────

class LocalMapView(QWidget):
    """Top-down render of body/map/local_2p5d height grid.

    Coordinate convention matches LidarView: forward=up, robot-left=left.
    NaN cells render as background, robot at body (0,0) drawn as a small
    forward-pointing triangle (clamped to edge + amber if off-grid).
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
        self.setMinimumSize(240, 240)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        self._grid: Optional[np.ndarray] = None
        self._meta: Optional[dict] = None
        self._ts: float = 0.0
        self._stale_s = stale_s
        self._max_height_m = max_height_m

    def update_map(
        self, grid: Optional[np.ndarray],
        meta: Optional[dict], ts: float,
    ) -> None:
        self._grid = grid
        self._meta = meta
        self._ts = ts
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor(10, 10, 10))

            if self._grid is None or self._meta is None:
                p.setPen(QColor(160, 160, 160))
                p.drawText(10, 16, "no map (Pi local_map disabled?)")
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

            # Orient: forward (i=nx-1) → top of image, left (j=ny-1) → left.
            display = grid[::-1, ::-1]
            valid = ~np.isnan(display)
            norm = np.zeros_like(display, dtype=np.float32)
            np.divide(display, self._max_height_m, out=norm, where=valid)
            np.clip(norm, 0.0, 1.0, out=norm)
            rgb = _turbo_rgb(norm)
            rgb[~valid] = (16, 16, 16)
            rgb = np.ascontiguousarray(rgb)
            qimg = QImage(
                rgb.data, ny, nx, 3 * ny, QImage.Format.Format_RGB888,
            ).copy()

            # Fit while preserving square cells.
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

            # Frame outline
            p.setPen(QPen(QColor(70, 70, 70), 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(ox, oy, draw_w, draw_h)

            # Robot marker at body (0,0).
            i_robot = -origin_x / res  # cell index along +x (forward)
            j_robot = -origin_y / res  # cell index along +y (left)
            r_disp = (nx - 1) - i_robot  # display row (top→bottom)
            c_disp = (ny - 1) - j_robot  # display col (left→right)
            cell_px_w = draw_w / ny
            cell_px_h = draw_h / nx
            rx = ox + (c_disp + 0.5) * cell_px_w
            ry = oy + (r_disp + 0.5) * cell_px_h

            off_grid = (
                i_robot < 0 or i_robot >= nx
                or j_robot < 0 or j_robot >= ny
            )
            rx = max(ox, min(ox + draw_w - 1, rx))
            ry = max(oy, min(oy + draw_h - 1, ry))

            color = QColor(255, 200, 80) if off_grid else QColor(255, 255, 255)
            p.setPen(QPen(color, 1))
            p.setBrush(color)
            tri_size = 9.0
            tri = QPolygonF([
                QPointF(rx, ry - tri_size),
                QPointF(rx - tri_size * 0.7, ry + tri_size * 0.7),
                QPointF(rx + tri_size * 0.7, ry + tri_size * 0.7),
            ])
            p.drawPolygon(tri)
            if off_grid:
                p.setPen(QColor(255, 200, 80))
                p.drawText(int(rx) + 12, int(ry) + 4, "robot off-grid")

            # Stale dimming overlay
            age = time.time() - self._ts if self._ts > 0 else 0.0
            if age > self._stale_s:
                p.fillRect(ox, oy, draw_w, draw_h, QColor(0, 0, 0, 140))
                p.setPen(QColor(255, 200, 80))
                p.drawText(
                    ox + 6, oy + 16,
                    f"stale ({age:.1f}s) — Pi local_map disabled?",
                )

            # Top-left scale legend
            p.setPen(QColor(180, 180, 180))
            p.drawText(
                margin, margin + 10,
                f"0–{self._max_height_m:.1f} m (turbo)",
            )
        finally:
            p.end()


class DriveableView(QWidget):
    """Top-down render of the driveable layer on body/map/local_2p5d.

    Cell encoding (int8):  1 = clear, 0 = blocked, -1 = unknown/null.
    Fixed three-color palette — no autoscale: a single outlier pixel cannot
    wash the rest out.
    """

    COLOR_CLEAR = (60, 170, 90)      # green
    COLOR_BLOCKED = (180, 60, 60)    # red
    COLOR_UNKNOWN = (60, 60, 60)     # dark gray
    COLOR_ABSENT_BG = (10, 10, 10)

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        stale_s: float = 2.0,
    ):
        super().__init__(parent)
        self.setMinimumSize(240, 240)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        self._drive: Optional[np.ndarray] = None
        self._meta: Optional[dict] = None
        self._ts: float = 0.0
        self._stale_s = stale_s

    def update_map(
        self, drive: Optional[np.ndarray],
        meta: Optional[dict], ts: float,
    ) -> None:
        self._drive = drive
        self._meta = meta
        self._ts = ts
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor(*self.COLOR_ABSENT_BG))

            if self._drive is None or self._meta is None:
                p.setPen(QColor(160, 160, 160))
                msg = (
                    "driveable N/A (Pi driveable_enabled off?)"
                    if self._meta is not None
                    else "no map (Pi local_map disabled?)"
                )
                p.drawText(10, 16, msg)
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

            # Orient: forward (i=nx-1) → top, left (j=ny-1) → left.
            display = drive[::-1, ::-1]
            rgb = np.empty((nx, ny, 3), dtype=np.uint8)
            rgb[...] = self.COLOR_UNKNOWN
            rgb[display == 1] = self.COLOR_CLEAR
            rgb[display == 0] = self.COLOR_BLOCKED
            rgb = np.ascontiguousarray(rgb)
            qimg = QImage(
                rgb.data, ny, nx, 3 * ny, QImage.Format.Format_RGB888,
            ).copy()

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

            # Robot marker, same convention as LocalMapView.
            i_robot = -origin_x / res
            j_robot = -origin_y / res
            r_disp = (nx - 1) - i_robot
            c_disp = (ny - 1) - j_robot
            cell_px_w = draw_w / ny
            cell_px_h = draw_h / nx
            rx = ox + (c_disp + 0.5) * cell_px_w
            ry = oy + (r_disp + 0.5) * cell_px_h
            off_grid = (
                i_robot < 0 or i_robot >= nx
                or j_robot < 0 or j_robot >= ny
            )
            rx = max(ox, min(ox + draw_w - 1, rx))
            ry = max(oy, min(oy + draw_h - 1, ry))
            color = QColor(255, 200, 80) if off_grid else QColor(255, 255, 255)
            p.setPen(QPen(color, 1))
            p.setBrush(color)
            tri_size = 9.0
            tri = QPolygonF([
                QPointF(rx, ry - tri_size),
                QPointF(rx - tri_size * 0.7, ry + tri_size * 0.7),
                QPointF(rx + tri_size * 0.7, ry + tri_size * 0.7),
            ])
            p.drawPolygon(tri)
            if off_grid:
                p.setPen(QColor(255, 200, 80))
                p.drawText(int(rx) + 12, int(ry) + 4, "robot off-grid")

            age = time.time() - self._ts if self._ts > 0 else 0.0
            if age > self._stale_s:
                p.fillRect(ox, oy, draw_w, draw_h, QColor(0, 0, 0, 140))
                p.setPen(QColor(255, 200, 80))
                p.drawText(
                    ox + 6, oy + 16,
                    f"stale ({age:.1f}s) — Pi local_map disabled?",
                )

            # Legend
            p.setPen(QColor(180, 180, 180))
            p.drawText(margin, margin + 10, "clear / blocked / unknown")
        finally:
            p.end()


# ── Vision dock (VLM chat + detect on current frame) ────────────────

class _VisionWorker(QThread):
    """One-shot thread running a vision_service call; emits result on done."""

    chat_result = pyqtSignal(str)
    detect_result = pyqtSignal(object)  # vision_service.DetectResult
    error = pyqtSignal(str)

    def __init__(self, mode: str, kwargs: dict, parent=None):
        super().__init__(parent)
        self._mode = mode
        self._kwargs = kwargs

    def run(self) -> None:
        try:
            import vision_service as vs
            if self._mode == "chat":
                self.chat_result.emit(vs.chat(**self._kwargs))
            elif self._mode == "detect":
                self.detect_result.emit(vs.detect(**self._kwargs))
            else:
                self.error.emit(f"unknown mode {self._mode!r}")
        except Exception as e:
            self.error.emit(f"{type(e).__name__}: {e}")


class _TTSWorker(QThread):
    """Fetches mp3 bytes from the xAI TTS API in a background thread."""

    audio_ready = pyqtSignal(bytes)
    error = pyqtSignal(str)

    def __init__(self, text: str, api_key: str, parent=None):
        super().__init__(parent)
        self._text = text
        self._api_key = api_key

    def run(self) -> None:
        try:
            import requests
            res = requests.post(
                "https://api.x.ai/v1/tts",
                headers={
                    "Authorization": f"Bearer {self._api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "text": self._text,
                    "voice_id": "Eve",
                    "output_format": {
                        "codec": "mp3",
                        "sample_rate": 44100,
                        "bit_rate": 128000,
                    },
                    "language": "en",
                },
                timeout=30,
            )
            if res.status_code != 200:
                self.error.emit(
                    f"HTTP {res.status_code}: {res.text[:200]}"
                )
                return
            self.audio_ready.emit(res.content)
        except Exception as e:
            self.error.emit(f"{type(e).__name__}: {e}")


class VisionDock(QDockWidget):
    """Chat + detect pane talking to a local VLM via src/vision_service.py."""

    send_chat = pyqtSignal(str, bool)   # text, attach_frame
    run_detect = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Vision", parent)
        self.setAllowedAreas(
            Qt.DockWidgetArea.RightDockWidgetArea
            | Qt.DockWidgetArea.LeftDockWidgetArea
        )
        body = QWidget()
        v = QVBoxLayout(body)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("route:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Direct VLM", "direct")
        self.mode_combo.addItem("Jill", "jill")
        mode_row.addWidget(self.mode_combo)
        mode_row.addStretch(1)
        v.addLayout(mode_row)

        self.transcript = QTextBrowser()
        self.transcript.setOpenExternalLinks(False)
        self.transcript.setMinimumWidth(320)
        v.addWidget(self.transcript, 1)

        self.input = QLineEdit()
        self.input.setPlaceholderText("Ask about the scene, or just chat…")
        v.addWidget(self.input)

        self.attach_box = QCheckBox("attach current frame")
        v.addWidget(self.attach_box)

        self.speak_box = QCheckBox("Speak replies (xAI TTS)")
        v.addWidget(self.speak_box)

        self._speak_queue: list[str] = []
        self._tts_worker: Optional[_TTSWorker] = None
        self._play_process: Optional[QProcess] = None
        self._audio_tmp_path: Optional[str] = None

        btn_row = QHBoxLayout()
        self.send_btn = QPushButton("Send")
        self.detect_btn = QPushButton("Detect frame")
        btn_row.addWidget(self.send_btn)
        btn_row.addWidget(self.detect_btn)
        btn_row.addStretch(1)
        v.addLayout(btn_row)

        self.status = QLabel("idle")
        self.status.setStyleSheet("color:#888;")
        v.addWidget(self.status)

        self.setWidget(body)

        self.send_btn.clicked.connect(self._emit_send)
        self.input.returnPressed.connect(self._emit_send)
        self.detect_btn.clicked.connect(self.run_detect)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        self._on_mode_changed()

    def mode(self) -> str:
        return self.mode_combo.currentData() or "direct"

    def _emit_send(self) -> None:
        text = self.input.text().strip()
        if not text:
            return
        self.send_chat.emit(text, self.attach_box.isChecked())
        self.input.clear()

    def _on_mode_changed(self) -> None:
        # Detect is a VLM-specific operation; in Jill mode Jill decides
        # when to call vision-query herself.
        self.detect_btn.setEnabled(self.mode() == "direct")

    def set_busy(self, busy: bool, label: str = "") -> None:
        self.send_btn.setEnabled(not busy)
        self.detect_btn.setEnabled(not busy)
        self.input.setEnabled(not busy)
        self.status.setText(label or ("waiting…" if busy else "idle"))

    def append_turn(self, role: str, text: str) -> None:
        from html import escape
        color = {"user": "#4a9", "assistant": "#ddd", "error": "#c66"}.get(role, "#888")
        prefix = {"user": "you", "assistant": "vlm", "error": "error"}.get(role, role)
        html = (
            f'<div style="margin:4px 0;">'
            f'<span style="color:{color};font-weight:bold;">{prefix}:</span> '
            f'<span style="white-space:pre-wrap;">{escape(text)}</span>'
            f'</div>'
        )
        self.transcript.append(html)
        if role == "assistant":
            self.speak(text)

    # ── TTS / speak ──────────────────────────────────────────────────

    def speak(self, text: str) -> None:
        if not self.speak_box.isChecked():
            return
        text = (text or "").strip()
        if not text:
            return
        self._speak_queue.append(text)
        self._pump_speak_queue()

    def _pump_speak_queue(self) -> None:
        if self._tts_worker is not None or self._play_process is not None:
            return
        if not self._speak_queue:
            return
        api_key = os.environ.get("GROK_API_KEY")
        if not api_key:
            self.append_turn("error", "GROK_API_KEY not set; disabling speak.")
            self._speak_queue.clear()
            self.speak_box.setChecked(False)
            return
        text = self._speak_queue.pop(0)
        worker = _TTSWorker(text, api_key, parent=self)
        worker.audio_ready.connect(self._on_tts_ready)
        worker.error.connect(self._on_tts_error)
        worker.finished.connect(self._on_tts_finished)
        self._tts_worker = worker
        worker.start()

    def _on_tts_ready(self, data: bytes) -> None:
        try:
            fd, path = tempfile.mkstemp(suffix=".mp3", prefix="bodystub_tts_")
            with os.fdopen(fd, "wb") as f:
                f.write(data)
        except Exception as e:
            self.append_turn("error", f"TTS write failed: {e}")
            return
        self._audio_tmp_path = path
        proc = QProcess(self)
        proc.finished.connect(self._on_play_finished)
        proc.errorOccurred.connect(self._on_play_error)
        self._play_process = proc
        proc.start(
            "ffplay",
            ["-nodisp", "-autoexit", "-loglevel", "quiet", path],
        )

    def _on_tts_error(self, msg: str) -> None:
        self.append_turn("error", f"TTS: {msg}")

    def _on_tts_finished(self) -> None:
        worker = self._tts_worker
        self._tts_worker = None
        if worker is not None:
            worker.deleteLater()
        if self._play_process is None:
            self._pump_speak_queue()

    def _on_play_finished(self, *_args) -> None:
        proc = self._play_process
        self._play_process = None
        if proc is not None:
            proc.deleteLater()
        if self._audio_tmp_path:
            try:
                os.unlink(self._audio_tmp_path)
            except OSError:
                pass
            self._audio_tmp_path = None
        self._pump_speak_queue()

    def _on_play_error(self, _err) -> None:
        self.append_turn("error", "ffplay failed (is ffmpeg installed?)")


# ── Motor test dock ─────────────────────────────────────────────────

# Heartbeat freshness gate: body/status must have landed within this
# window, and status.heartbeat_ok must be true. 2 s matches the spec's
# HEARTBEAT_TIMEOUT_MS default; a stale status probably means the
# watchdog itself isn't running.
_STATUS_FRESH_S = 2.0


class DifferentialPad(QWidget):
    """Skid-steer 2D pad for driving both wheels with one mouse.

    Mouse position inside the unit circle maps to (left, right) wheel
    velocities via the standard tank mapping:

        nx, ny ∈ [-1, +1]   (y up; clamped to unit circle)
        left  = clamp(ny + nx, -1, +1) * max_wheel
        right = clamp(ny - nx, -1, +1) * max_wheel

    Consequences:
      * Center = stop.
      * Pure up = straight forward at max_wheel.
      * Pure right = L fwd, R rev → spin right (CW) in place.
      * Top-right corner (projected onto circle) = tight right turn
        while moving forward (L full, R slow/zero).
      * Drag outside the circle projects back onto the edge.
      * Mouse release snaps the handle to center → zero command.

    Widget is always enabled; disabling is done by the parent dock
    (engage toggle) via setEnabled().
    """

    # (left_mps, right_mps) — emitted on every position change.
    cmd_changed = pyqtSignal(float, float)

    # Expo curve on radius: r_out = (1-EXPO)*r + EXPO*r^3. 0=linear,
    # 1=pure cubic. Softens response near center so small mouse nudges
    # produce small commands; outer-edge full-stick still hits ±1.
    EXPO: float = 0.7

    def __init__(self, max_wheel: float, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._max_wheel = float(max_wheel)
        self._nx: float = 0.0  # normalized [-1, 1] (raw mouse position)
        self._ny: float = 0.0
        self._dragging: bool = False
        self.setMinimumSize(240, 240)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        self.setMouseTracking(False)  # only update while button is down

    # ── Public API ──────────────────────────────────────────────────

    def set_max_wheel(self, v: float) -> None:
        self._max_wheel = float(v)
        self._emit()  # same position, new scale

    def _curved(self) -> tuple[float, float]:
        """Apply expo curve on radius; angle unchanged. Keeps center soft
        without losing full-stick range.
        """
        nx, ny = self._nx, self._ny
        r = math.hypot(nx, ny)
        if r <= 0.0:
            return 0.0, 0.0
        r_out = (1.0 - self.EXPO) * r + self.EXPO * (r ** 3)
        s = r_out / r
        return nx * s, ny * s

    def current_mps(self) -> tuple[float, float]:
        nx, ny = self._curved()
        L = max(-1.0, min(1.0, ny + nx)) * self._max_wheel
        R = max(-1.0, min(1.0, ny - nx)) * self._max_wheel
        return L, R

    def recenter(self) -> None:
        self._nx = 0.0
        self._ny = 0.0
        self._dragging = False
        self._emit()
        self.update()

    # ── Geometry ────────────────────────────────────────────────────

    def _center_radius(self) -> tuple[float, float, float]:
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        r = min(w, h) / 2.0 - 10.0
        return cx, cy, max(r, 1.0)

    def _pixel_to_norm(self, px: float, py: float) -> tuple[float, float]:
        cx, cy, r = self._center_radius()
        nx = (px - cx) / r
        ny = -(py - cy) / r  # screen y is down; make up positive
        # Project onto unit circle if outside.
        d = math.hypot(nx, ny)
        if d > 1.0 and d > 0.0:
            nx /= d
            ny /= d
        return nx, ny

    def _norm_to_pixel(self, nx: float, ny: float) -> tuple[float, float]:
        cx, cy, r = self._center_radius()
        return cx + nx * r, cy - ny * r

    # ── Events ──────────────────────────────────────────────────────

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        self._dragging = True
        self._nx, self._ny = self._pixel_to_norm(
            event.position().x(), event.position().y(),
        )
        self._emit()
        self.update()

    def mouseMoveEvent(self, event) -> None:
        if not self._dragging:
            return
        self._nx, self._ny = self._pixel_to_norm(
            event.position().x(), event.position().y(),
        )
        self._emit()
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        self.recenter()

    def leaveEvent(self, _event) -> None:
        # Safety: if the pointer exits the widget mid-drag (e.g. user
        # drags off the window), treat as release.
        if self._dragging:
            self.recenter()

    def _emit(self) -> None:
        L, R = self.current_mps()
        self.cmd_changed.emit(L, R)

    # ── Painting ────────────────────────────────────────────────────

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            w, h = self.width(), self.height()
            enabled = self.isEnabled()
            p.fillRect(0, 0, w, h,
                       QColor(18, 18, 18) if enabled else QColor(28, 28, 28))
            cx, cy, r = self._center_radius()

            # Outer circle
            ring = QColor(120, 160, 200) if enabled else QColor(70, 70, 70)
            p.setPen(QPen(ring, 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(cx, cy), r, r)

            # Inner rings at 0.5 and 0.25 for speed reference
            p.setPen(QPen(QColor(60, 60, 60), 1))
            for frac in (0.25, 0.5, 0.75):
                p.drawEllipse(QPointF(cx, cy), r * frac, r * frac)

            # Crosshairs (F=forward up, R=spin right, B=back, L=spin left)
            p.setPen(QPen(QColor(70, 70, 70), 1))
            p.drawLine(int(cx - r), int(cy), int(cx + r), int(cy))
            p.drawLine(int(cx), int(cy - r), int(cx), int(cy + r))
            tag = QColor(130, 130, 130) if enabled else QColor(60, 60, 60)
            p.setPen(tag)
            p.setFont(QFont("Sans", 9, QFont.Weight.Bold))
            p.drawText(int(cx - 5), int(cy - r - 2), "F")
            p.drawText(int(cx - 5), int(cy + r + 14), "B")
            p.drawText(int(cx - r - 14), int(cy + 5), "L")
            p.drawText(int(cx + r + 4), int(cy + 5), "R")

            # Current position dot + line from center
            if enabled:
                hx, hy = self._norm_to_pixel(self._nx, self._ny)
                p.setPen(QPen(QColor(255, 200, 60), 2))
                p.drawLine(int(cx), int(cy), int(hx), int(hy))
                p.setPen(QPen(QColor(255, 220, 100), 1))
                p.setBrush(QColor(255, 220, 100))
                p.drawEllipse(QPointF(hx, hy), 7, 7)

            # Readout text (L/R in m/s)
            L, R = self.current_mps()
            txt = f"L {L:+.2f}   R {R:+.2f}  m/s"
            p.setPen(QColor(220, 220, 220) if enabled else QColor(100, 100, 100))
            p.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
            p.drawText(8, h - 10, txt)
        finally:
            p.end()


class _Pill(QLabel):
    """Small colored chip used for motion-gate indicators."""

    def __init__(self, text: str, parent: Optional[QWidget] = None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFont(QFont("Monospace", 8))
        self.setMinimumWidth(70)
        self.set_state(False)

    def set_state(self, ok: bool) -> None:
        bg = "#2c7a2c" if ok else "#5a2a2a"
        fg = "white" if ok else "#bbb"
        self.setStyleSheet(
            f"background:{bg};color:{fg};"
            f"padding:1px 6px;border-radius:4px;"
        )


class MotorTestDock(QDockWidget):
    """Per-wheel motor test panel (body/cmd_direct).

    Purpose: bring-up + calibration tool. Bypasses the twist→differential
    math so left/right can be tested independently.

      * Engage button arms the tester. Engage ON → mode=cmd_direct +
        live_command=True; publisher emits cmd_direct continuously with
        whatever the sliders read. Engage OFF → stop_all() +
        mode=cmd_vel. No separate "hold-to-drive" dead-man button —
        single-mouse UX can't hold a button AND drag a slider.
      * Sliders are center-snap, release-to-zero. Mouse-up on either
        slider snaps it back to 0.0 m/s (immediate published zero).
      * STOP button is the panic path: full stop and disengage.
      * Gate pills are diagnostic only — they do NOT block commands.
        The negative gates (e-stop, timeout, stall) are all cleared on
        the Pi by a fresh command, so blocking commands on their
        account would be a deadlock.
      * Feedback readouts come straight from body/motor_state (PWM,
        direction, flags) and body/odom (left_ticks/right_ticks,
        diffed locally for ticks/s). No wheel_base_m dependency.
    """

    # (mode,) — emitted when user toggles engage. "cmd_direct" or "cmd_vel".
    mode_change_requested = pyqtSignal(str)
    # (left_mps, right_mps)
    cmd_direct_changed = pyqtSignal(float, float)
    # panic stop
    stop_requested = pyqtSignal()

    def __init__(self, max_wheel_default: float, timeout_ms_default: int,
                 parent: Optional[QWidget] = None):
        super().__init__("Motor Test (cmd_direct)", parent)
        self.setAllowedAreas(
            Qt.DockWidgetArea.RightDockWidgetArea
            | Qt.DockWidgetArea.LeftDockWidgetArea
        )
        self._max_wheel = float(max_wheel_default)
        self._tick_hist: dict[str, tuple[int, float]] = {}  # side → (ticks, ts)
        self._tick_rate: dict[str, float] = {"left": 0.0, "right": 0.0}

        body = QWidget()
        v = QVBoxLayout(body)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(6)

        # --- engage row ---
        engage_row = QHBoxLayout()
        self.engage_btn = QPushButton("Engage direct control")
        self.engage_btn.setCheckable(True)
        self.engage_btn.setStyleSheet(
            "QPushButton:checked{background:#2c7a2c;color:white;"
            "font-weight:bold;}"
        )
        engage_row.addWidget(self.engage_btn)
        engage_row.addStretch(1)
        v.addLayout(engage_row)

        # --- limits row ---
        limits_row = QHBoxLayout()
        limits_row.addWidget(QLabel("Max wheel (m/s):"))
        self.max_wheel_box = QDoubleSpinBox()
        self.max_wheel_box.setRange(0.05, 1.0)
        self.max_wheel_box.setSingleStep(0.05)
        self.max_wheel_box.setDecimals(2)
        self.max_wheel_box.setValue(self._max_wheel)
        limits_row.addWidget(self.max_wheel_box)
        limits_row.addSpacing(12)
        limits_row.addWidget(QLabel("Timeout (ms):"))
        self.timeout_box = QSpinBox()
        self.timeout_box.setRange(100, 5000)
        self.timeout_box.setSingleStep(50)
        self.timeout_box.setValue(int(timeout_ms_default))
        self.timeout_box.setToolTip(
            "timeout_ms field sent with each cmd_direct payload"
        )
        limits_row.addWidget(self.timeout_box)
        limits_row.addStretch(1)
        v.addLayout(limits_row)

        # --- differential pad (replaces left/right sliders) ---
        self.pad = DifferentialPad(self._max_wheel)
        v.addWidget(self.pad, 1)

        # --- stop ---
        btn_row = QHBoxLayout()
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet(
            "QPushButton{background:#aa2222;color:white;font-weight:bold;"
            "padding:8px;}"
        )
        btn_row.addWidget(self.stop_btn, 1)
        v.addLayout(btn_row)

        # --- gate pills ---
        gate_frame = QFrame()
        gate_frame.setFrameShape(QFrame.Shape.StyledPanel)
        gv = QVBoxLayout(gate_frame)
        gv.setContentsMargins(6, 4, 6, 4)
        gv.setSpacing(4)
        gates_row = QHBoxLayout()
        gates_row.setSpacing(4)
        self.pills = {
            "connected": _Pill("conn"),
            "live": _Pill("live"),
            "hb_ok": _Pill("hb"),
            "no_estop": _Pill("no-stop"),
            "no_timeout": _Pill("no-to"),
            "no_stall": _Pill("no-stall"),
        }
        for pill in self.pills.values():
            gates_row.addWidget(pill)
        gates_row.addStretch(1)
        gv.addLayout(gates_row)
        self.status_line = QLabel("disconnected")
        self.status_line.setStyleSheet("color:#bbb;")
        gv.addWidget(self.status_line)
        v.addWidget(gate_frame)

        # --- measured readouts ---
        meas_frame = QFrame()
        meas_frame.setFrameShape(QFrame.Shape.StyledPanel)
        mv = QVBoxLayout(meas_frame)
        mv.setContentsMargins(6, 4, 6, 4)
        mv.setSpacing(2)
        mono = QFont("Monospace", 9)
        self.meas_header = QLabel("Measured (motor_state + odom):")
        mv.addWidget(self.meas_header)
        self.meas_left = QLabel("L: —")
        self.meas_right = QLabel("R: —")
        for lbl in (self.meas_left, self.meas_right):
            lbl.setFont(mono)
            mv.addWidget(lbl)
        v.addWidget(meas_frame)

        v.addStretch(1)
        self.setWidget(body)

        # --- wiring ---
        self.engage_btn.toggled.connect(self._on_engage_toggled)
        self.max_wheel_box.valueChanged.connect(self._on_max_wheel_changed)
        self.pad.cmd_changed.connect(self._on_pad_changed)
        self.stop_btn.clicked.connect(self.stop_requested)

        # Pad is disabled until engaged.
        self._set_controls_enabled(False)

    # ── User actions ─────────────────────────────────────────────────

    def _on_pad_changed(self, left: float, right: float) -> None:
        # The pad produces m/s directly (it was given max_wheel at
        # construction and kept in sync via set_max_wheel).
        self.cmd_direct_changed.emit(left, right)

    def _on_engage_toggled(self, on: bool) -> None:
        self.engage_btn.setText(
            "Release direct control (→ cmd_vel)" if on
            else "Engage direct control"
        )
        self._set_controls_enabled(on)
        if not on:
            self.pad.recenter()
        self.mode_change_requested.emit("cmd_direct" if on else "cmd_vel")

    def _on_max_wheel_changed(self, v: float) -> None:
        self._max_wheel = float(v)
        self.pad.set_max_wheel(self._max_wheel)

    def _set_controls_enabled(self, on: bool) -> None:
        self.pad.setEnabled(on)

    def timeout_ms(self) -> int:
        return int(self.timeout_box.value())

    def engaged(self) -> bool:
        return self.engage_btn.isChecked()

    # ── Redraw (called from BodyStubWindow._tick) ────────────────────

    def update_state(self, snap: dict, now: float) -> None:
        motor = snap.get("motor") or {}
        status = snap.get("status") or {}
        status_ts = snap.get("status_ts") or 0.0

        # Tick-rate derivation from odom.left_ticks / right_ticks.
        odom = snap.get("odom") or {}
        odom_ts = snap.get("odom_ts") or 0.0
        for side, key in (("left", "left_ticks"), ("right", "right_ticks")):
            ticks = odom.get(key)
            if not isinstance(ticks, (int, float)) or odom_ts <= 0.0:
                continue
            prev = self._tick_hist.get(side)
            self._tick_hist[side] = (int(ticks), float(odom_ts))
            if prev is None:
                continue
            dt = odom_ts - prev[1]
            if dt <= 0.0:
                continue
            self._tick_rate[side] = (int(ticks) - prev[0]) / dt

        # --- gates ---
        connected = bool(snap.get("connected"))
        live = bool(snap.get("live"))
        hb_ok = (
            bool(status.get("heartbeat_ok", False))
            and status_ts > 0.0
            and (now - status_ts) <= _STATUS_FRESH_S
        )
        # motor_state.e_stop_active already ORs status.e_stop_active on Pi,
        # per Body-side review. Use it as the authoritative motion gate.
        e_stop = bool(motor.get("e_stop_active", False))
        cmd_timeout = bool(motor.get("cmd_timeout_active", False))
        stall = bool(motor.get("stall_detected", False))

        gates = {
            "connected": connected,
            "live": connected and live,
            "hb_ok": connected and hb_ok,
            "no_estop": connected and not e_stop,
            "no_timeout": connected and not cmd_timeout,
            "no_stall": connected and not stall,
        }
        for key, ok in gates.items():
            self.pills[key].set_state(ok)

        reason = self._reason_string(
            connected=connected, live=live, hb_ok=hb_ok,
            status_seen=(status_ts > 0.0),
            status_e_stop=bool(status.get("e_stop_active", False)),
            motor_seen=(snap.get("motor_ts") or 0.0) > 0.0,
            e_stop=e_stop, cmd_timeout=cmd_timeout, stall=stall,
            engaged=self.engaged(),
        )
        self.status_line.setText(reason)

        # --- measured readouts ---
        if not motor:
            self.meas_left.setText("L: no motor_state yet")
            self.meas_right.setText("R: no motor_state yet")
        else:
            def fmt(side_key_pwm: str, side_key_dir: str, rate_key: str) -> str:
                pwm = motor.get(side_key_pwm)
                dire = motor.get(side_key_dir) or "?"
                pwm_s = f"{float(pwm):+.2f}" if isinstance(pwm, (int, float)) else "  — "
                rate = self._tick_rate.get(rate_key, 0.0)
                return f"PWM {pwm_s}  dir {dire:<3}  ticks/s {rate:+8.1f}"
            self.meas_left.setText("L: " + fmt("left_pwm", "left_dir", "left"))
            self.meas_right.setText("R: " + fmt("right_pwm", "right_dir", "right"))

    def _reason_string(
        self, *, connected: bool, live: bool, hb_ok: bool, status_seen: bool,
        status_e_stop: bool, motor_seen: bool,
        e_stop: bool, cmd_timeout: bool, stall: bool, engaged: bool,
    ) -> str:
        """Human-readable status.

        Pre-engage states report setup problems (disconnected, missing
        topics). Once engaged, reports whatever condition on the Pi is
        inhibiting actual motion — but only as a diagnostic; the dock
        keeps publishing regardless so a stuck latch can self-clear on
        the next fresh command.
        """
        if not connected:
            return "disconnected — press Connect"
        if not engaged:
            return "not engaged — main window cmd_vel controls active"
        if not motor_seen:
            return "engaged — waiting for motor_state…"
        if not status_seen:
            return "engaged — no body/status seen (watchdog running?)"
        if not hb_ok:
            return "engaged — heartbeat not ok at watchdog yet"
        if status_e_stop:
            return "engaged — watchdog holds e-stop (status.e_stop_active)"
        if e_stop:
            return "engaged — motor holds e-stop (motor_state.e_stop_active)"
        if cmd_timeout:
            return "engaged — motor sees cmd_timeout_active"
        if stall:
            return "engaged — stall_detected; return to zero to clear"
        if not live:
            return "engaged — publisher not live (should not happen)"
        return "driving"


# ── Main window ─────────────────────────────────────────────────────

class BodyStubWindow(QMainWindow):
    def __init__(self, controller, config):
        super().__init__()
        self.controller = controller
        self.config = config
        self.setWindowTitle("Body Stub — dev tool (do not run alongside Jill)")
        self.resize(1340, 720)
        # Vision state — transcript and last detect result tied to the rgb_ts
        # they were computed from, so boxes auto-clear when a newer frame lands.
        self._vision_transcript: list[dict] = []
        self._vision_boxes: list = []
        self._vision_boxes_for_ts: float = 0.0
        self._vision_worker: Optional[_VisionWorker] = None
        # Jill client is lazy-connected on first use (Jill mode).
        self._jill_client = JillClient(
            router=config.jill_router or config.router,
            character=config.jill_character,
        )
        self._build_ui()
        self._wire_signals()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        period_ms = max(50, int(1000.0 / max(1.0, self.config.ui_redraw_hz)))
        self._timer.start(period_ms)

    def _build_ui(self) -> None:
        central = QWidget(self)
        root = QVBoxLayout(central)

        # --- connection row ---
        conn_row = QHBoxLayout()
        conn_row.addWidget(QLabel("Router:"))
        self.router_edit = QLineEdit(self.config.router)
        self.router_edit.setMinimumWidth(240)
        conn_row.addWidget(self.router_edit)
        self.connect_btn = QPushButton("Connect")
        conn_row.addWidget(self.connect_btn)
        self.live_box = QCheckBox("Live command (publish cmd_vel)")
        self.live_box.setEnabled(False)
        conn_row.addWidget(self.live_box)
        conn_row.addStretch(1)
        self.conn_status = QLabel("disconnected")
        conn_row.addWidget(self.conn_status)
        root.addLayout(conn_row)

        # --- grid: status / rgb / depth / lidar ---
        grid = QGridLayout()

        status_box = QGroupBox("Body status")
        sv = QVBoxLayout(status_box)
        self.host_panel = HostPanel()
        sv.addWidget(self.host_panel)
        self.status_text = QPlainTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setFont(QFont("Monospace", 9))
        sv.addWidget(self.status_text)
        grid.addWidget(status_box, 0, 0)

        rgb_box = QGroupBox("OAK-D RGB (on request)")
        rv = QVBoxLayout(rgb_box)
        self.rgb_label = QLabel("no image")
        self.rgb_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.rgb_label.setMinimumSize(320, 240)
        self.rgb_label.setStyleSheet("background-color:#111;color:#aaa;")
        rv.addWidget(self.rgb_label)
        self.rgb_meta = QLabel("—")
        rv.addWidget(self.rgb_meta)
        grid.addWidget(rgb_box, 0, 1)

        depth_box = QGroupBox("OAK-D depth")
        dv = QVBoxLayout(depth_box)
        self.depth_label = QLabel("no depth")
        self.depth_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.depth_label.setMinimumSize(320, 240)
        self.depth_label.setStyleSheet("background-color:#111;color:#aaa;")
        dv.addWidget(self.depth_label)
        self.depth_meta = QLabel("—")
        dv.addWidget(self.depth_meta)
        grid.addWidget(depth_box, 1, 0)

        maps_tabs = QTabWidget()

        lidar_tab = QWidget()
        lv = QVBoxLayout(lidar_tab)
        lv.setContentsMargins(4, 4, 4, 4)
        self.lidar_view = LidarView()
        lv.addWidget(self.lidar_view)
        self.lidar_meta = QLabel("—")
        lv.addWidget(self.lidar_meta)
        maps_tabs.addTab(lidar_tab, "Lidar (forward = up)")

        map_tab = QWidget()
        mv = QVBoxLayout(map_tab)
        mv.setContentsMargins(4, 4, 4, 4)
        self.local_map_view = LocalMapView(stale_s=self.config.map_stale_s)
        mv.addWidget(self.local_map_view)
        self.local_map_meta = QLabel("—")
        mv.addWidget(self.local_map_meta)
        maps_tabs.addTab(map_tab, "Local map (2.5D)")

        drive_tab = QWidget()
        dv2 = QVBoxLayout(drive_tab)
        dv2.setContentsMargins(4, 4, 4, 4)
        self.driveable_view = DriveableView(stale_s=self.config.map_stale_s)
        dv2.addWidget(self.driveable_view)
        self.driveable_meta = QLabel("—")
        dv2.addWidget(self.driveable_meta)
        maps_tabs.addTab(drive_tab, "Driveable")

        grid.addWidget(maps_tabs, 1, 1)

        root.addLayout(grid)

        # --- command row ---
        cmd_row = QHBoxLayout()
        cmd_row.addWidget(QLabel("linear (m/s):"))
        self.linear_box = QDoubleSpinBox()
        self.linear_box.setRange(-1.0, 1.0)
        self.linear_box.setSingleStep(0.05)
        self.linear_box.setDecimals(2)
        cmd_row.addWidget(self.linear_box)
        cmd_row.addWidget(QLabel("angular (rad/s):"))
        self.angular_box = QDoubleSpinBox()
        self.angular_box.setRange(-2.0, 2.0)
        self.angular_box.setSingleStep(0.1)
        self.angular_box.setDecimals(2)
        cmd_row.addWidget(self.angular_box)
        self.apply_btn = QPushButton("Apply cmd_vel")
        cmd_row.addWidget(self.apply_btn)
        self.stop_btn = QPushButton("ALL STOP")
        self.stop_btn.setStyleSheet(
            "QPushButton{background:#aa2222;color:white;font-weight:bold;}"
        )
        cmd_row.addWidget(self.stop_btn)
        cmd_row.addStretch(1)
        self.rgb_btn = QPushButton("Request RGB")
        cmd_row.addWidget(self.rgb_btn)
        root.addLayout(cmd_row)

        self.setCentralWidget(central)

        self.vision_dock = VisionDock(self)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.vision_dock)

        self.motor_dock = MotorTestDock(
            max_wheel_default=self.config.max_wheel_vel_default,
            timeout_ms_default=self.config.cmd_vel_timeout_ms,
            parent=self,
        )
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.motor_dock)
        # Tab the motor dock behind vision so both can be raised independently.
        self.tabifyDockWidget(self.vision_dock, self.motor_dock)

        from .sweep_dock import SweepDock
        self.sweep_dock = SweepDock(self.controller, parent=self)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.sweep_dock)
        self.tabifyDockWidget(self.vision_dock, self.sweep_dock)

    def _wire_signals(self) -> None:
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.live_box.toggled.connect(self._on_live_toggled)
        self.apply_btn.clicked.connect(self._on_apply_clicked)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
        self.rgb_btn.clicked.connect(self._on_rgb_clicked)
        self.vision_dock.send_chat.connect(self._on_vision_send)
        self.vision_dock.run_detect.connect(self._on_vision_detect)
        self.motor_dock.mode_change_requested.connect(self._on_motor_mode_change)
        self.motor_dock.cmd_direct_changed.connect(self._on_cmd_direct_changed)
        self.motor_dock.stop_requested.connect(self._on_stop_clicked)
        self.sweep_dock.mission_active.connect(self._on_sweep_active)

    # ── Signal handlers ──────────────────────────────────────────────

    def _on_connect_clicked(self) -> None:
        snap = self._snapshot()
        if snap["connected"]:
            self.controller.disconnect()
            self.connect_btn.setText("Connect")
            self.live_box.setChecked(False)
            self.live_box.setEnabled(False)
            self.conn_status.setText("disconnected")
            return
        endpoint = self.router_edit.text().strip() or self.config.router
        self.controller.config.router = endpoint
        self.conn_status.setText(f"connecting to {endpoint}…")
        QApplication.processEvents()
        ok, err = self.controller.connect()
        if ok:
            self.connect_btn.setText("Disconnect")
            self.live_box.setEnabled(True)
            self.conn_status.setText(f"connected {endpoint}")
        else:
            self.conn_status.setText(f"connect failed: {err}")

    def _on_live_toggled(self, on: bool) -> None:
        self.controller.set_live_command(bool(on))

    def _on_apply_clicked(self) -> None:
        self.controller.set_cmd_vel(
            self.linear_box.value(), self.angular_box.value(),
        )

    def _on_stop_clicked(self) -> None:
        self.linear_box.setValue(0.0)
        self.angular_box.setValue(0.0)
        # Disengage the motor dock first if it's active. Unchecking
        # engage_btn triggers _on_motor_mode_change('cmd_vel'), which
        # calls stop_all() itself — so we don't need to call it twice
        # in that branch.
        if self.motor_dock.engage_btn.isChecked():
            self.motor_dock.engage_btn.setChecked(False)
        else:
            self.controller.stop_all()
        self.live_box.setChecked(False)

    def _on_rgb_clicked(self) -> None:
        req = self.controller.request_rgb()
        if req is None:
            self.rgb_meta.setText("request failed (not connected?)")
        else:
            self.rgb_meta.setText(f"request_id {req[:8]}… pending")

    # ── Motor-test dock handlers ────────────────────────────────────

    def _on_motor_mode_change(self, mode: str) -> None:
        """Dock engage toggled → switch controller mode and arm/disarm.

        Engage ON → mode=cmd_direct, live_command=True. Publisher
        emits cmd_direct at cmd_vel_hz with current slider values.
        Main window's cmd_vel live_box is unchecked so the two paths
        don't race (only one publisher emits per cycle, but the
        user-facing state should be unambiguous).

        Engage OFF → stop_all() (zero both topics, drop live, supersede),
        then switch stored mode back to cmd_vel. User can drive cmd_vel
        again from the main row by re-checking live_box.
        """
        # Push the dock's timeout value into the config so the next
        # published command carries it.
        self.controller.config.cmd_vel_timeout_ms = self.motor_dock.timeout_ms()
        if mode == "cmd_direct":
            if self.live_box.isChecked():
                self.live_box.setChecked(False)
            self.controller.set_cmd_mode("cmd_direct")
            self.controller.set_live_command(True)
        else:
            self.controller.stop_all()
            self.controller.set_cmd_mode("cmd_vel")

    def _on_cmd_direct_changed(self, left: float, right: float) -> None:
        self.controller.set_cmd_direct(left, right)

    # ── Sweep mission lockout ────────────────────────────────────────

    def _on_sweep_active(self, active: bool) -> None:
        """Lock out competing commanding controls while sweep is running.

        Sweep owns cmd_vel for the duration; the Live cmd checkbox and
        the motor-test dock would race. Abort + ALL STOP stay live.
        """
        if active:
            # Disengage motor dock if user had it engaged.
            if self.motor_dock.engage_btn.isChecked():
                self.motor_dock.engage_btn.setChecked(False)
            # Drop the main-window live toggle; sweep manages live_command itself.
            if self.live_box.isChecked():
                self.live_box.setChecked(False)
        self.live_box.setEnabled(not active and self._snapshot()["connected"])
        self.motor_dock.engage_btn.setEnabled(not active)
        self.apply_btn.setEnabled(not active)

    # ── Vision handlers ──────────────────────────────────────────────

    def _current_frame(self) -> tuple[Optional[bytes], float]:
        snap = self._snapshot()
        return snap["rgb_jpeg"], snap["rgb_ts"]

    def _on_vision_send(self, text: str, attach_frame: bool) -> None:
        if self.vision_dock.mode() == "jill":
            self._send_to_jill(text, attach_frame)
            return
        if self._vision_worker is not None:
            return
        jpeg, _ts = self._current_frame()
        if attach_frame and not jpeg:
            self.vision_dock.append_turn("error", "No RGB frame to attach (request one first).")
            return
        self._vision_transcript.append({"role": "user", "content": text})
        self.vision_dock.append_turn("user", text + (" [+frame]" if attach_frame else ""))
        images = [jpeg] if (attach_frame and jpeg) else None
        self._start_vision_worker(
            "chat",
            {"messages": list(self._vision_transcript), "images": images},
        )

    def _send_to_jill(self, text: str, attach_frame: bool) -> None:
        if not self._jill_client.connected:
            err = self._jill_client.connect()
            if err is not None:
                self.vision_dock.append_turn("error", f"Jill connect failed: {err}")
                return
        image_path: Optional[str] = None
        if attach_frame:
            jpeg, _ts = self._current_frame()
            if not jpeg:
                self.vision_dock.append_turn("error", "No RGB frame to attach (request one first).")
                return
            try:
                import vision_service
                image_path = vision_service.cache_jpeg(jpeg)
            except Exception as e:
                self.vision_dock.append_turn("error", f"cache write failed: {e}")
                return
        ok = self._jill_client.publish_chat(text, image_path=image_path)
        if not ok:
            self.vision_dock.append_turn("error", "Jill publish failed.")
            return
        suffix = f" [+frame:{image_path}]" if image_path else ""
        self.vision_dock.append_turn("user", text + suffix)

    def _on_vision_detect(self) -> None:
        if self._vision_worker is not None:
            return
        jpeg, ts = self._current_frame()
        if not jpeg:
            self.vision_dock.append_turn("error", "No RGB frame — click Request RGB first.")
            return
        self._vision_pending_ts = ts
        self.vision_dock.append_turn("user", "[detect objects in current frame]")
        self._start_vision_worker("detect", {"jpeg_bytes": jpeg})

    def _start_vision_worker(self, mode: str, kwargs: dict) -> None:
        worker = _VisionWorker(mode, kwargs, parent=self)
        worker.chat_result.connect(self._on_vision_chat_result)
        worker.detect_result.connect(self._on_vision_detect_result)
        worker.error.connect(self._on_vision_error)
        worker.finished.connect(self._on_vision_finished)
        self._vision_worker = worker
        self.vision_dock.set_busy(True, f"{mode}…")
        worker.start()

    def _on_vision_chat_result(self, text: str) -> None:
        self._vision_transcript.append({"role": "assistant", "content": text})
        self.vision_dock.append_turn("assistant", text)

    def _on_vision_detect_result(self, result) -> None:
        self._vision_transcript.append({"role": "assistant", "content": result.text})
        if result.boxes:
            summary = "detected: " + ", ".join(
                f"{b.label}" + (f" ({b.confidence:.2f})" if b.confidence is not None else "")
                for b in result.boxes
            )
            self.vision_dock.append_turn("assistant", summary)
        else:
            self.vision_dock.append_turn("assistant", result.text)
        self._vision_boxes = result.boxes
        self._vision_boxes_for_ts = getattr(self, "_vision_pending_ts", 0.0)

    def _on_vision_error(self, msg: str) -> None:
        self.vision_dock.append_turn("error", msg)

    def _on_vision_finished(self) -> None:
        worker = self._vision_worker
        self._vision_worker = None
        self.vision_dock.set_busy(False)
        if worker is not None:
            worker.deleteLater()

    # ── Redraw tick ──────────────────────────────────────────────────

    def _snapshot(self) -> dict:
        s = self.controller.state
        with s.lock:
            snap = dict(
                connected=s.connected, live=s.live_command,
                status=s.status, status_ts=s.status_ts,
                emergency=s.emergency_stop, emergency_ts=s.emergency_ts,
                odom=s.odom, odom_ts=s.odom_ts,
                motor=s.motor_state, motor_ts=s.motor_ts,
                lidar=s.lidar_scan, lidar_ts=s.lidar_ts,
                imu=s.oakd_imu, imu_ts=s.oakd_imu_ts,
                depth_image=s.depth_image, depth_width=s.depth_width,
                depth_height=s.depth_height, depth_format=s.depth_format,
                depth_ts=s.depth_ts,
                rgb_jpeg=s.rgb_jpeg, rgb_width=s.rgb_width,
                rgb_height=s.rgb_height, rgb_ts=s.rgb_ts,
                rgb_error=s.rgb_error, rgb_request_id=s.rgb_request_id,
                pending_rgb=s.pending_rgb_request_id,
                heartbeat_seq=s.heartbeat_seq,
                last_cmd=s.last_cmd_vel,
                local_map_grid=s.local_map_grid,
                local_map_meta=s.local_map_meta,
                local_map_ts=s.local_map_ts,
                local_map_driveable=s.local_map_driveable,
            )
        return snap

    def _tick(self) -> None:
        snap = self._snapshot()
        self._render_status(snap)
        self._render_depth(snap)
        self._render_rgb(snap)
        self._render_lidar(snap)
        self._render_local_map(snap)
        self._render_driveable(snap)
        self._drain_jill_replies()
        self.motor_dock.update_state(snap, time.time())

    def _drain_jill_replies(self) -> None:
        if not self._jill_client.connected:
            return
        for text, _ts in self._jill_client.drain_replies():
            self.vision_dock.append_turn("assistant", text)

    def _render_status(self, snap: dict) -> None:
        now = time.time()
        def age(ts: float) -> str:
            return "—" if ts <= 0 else f"{now - ts:5.2f}s ago"
        status = snap["status"]
        host = status.get("host") if isinstance(status, dict) else None
        self.host_panel.update_host(host)
        lines = []
        lines.append(f"connected    : {snap['connected']}")
        lines.append(f"live command : {snap['live']}")
        lines.append(f"heartbeat seq: {snap['heartbeat_seq']}")
        lines.append(f"last cmd_vel : lin={snap['last_cmd'][0]:+.2f} "
                     f"ang={snap['last_cmd'][1]:+.2f}")
        lines.append("")
        lines.append(f"status       [{age(snap['status_ts'])}]: "
                     f"{_brief(snap['status'])}")
        lines.append(f"emergency    [{age(snap['emergency_ts'])}]: "
                     f"{_brief(snap['emergency'])}")
        lines.append(f"odom         [{age(snap['odom_ts'])}]: "
                     f"{_brief(snap['odom'])}")
        lines.append(f"motor_state  [{age(snap['motor_ts'])}]: "
                     f"{_brief(snap['motor'])}")
        lines.append(f"oakd/imu     [{age(snap['imu_ts'])}]: "
                     f"{_brief(snap['imu'])}")
        self.status_text.setPlainText("\n".join(lines))

    def _render_depth(self, snap: dict) -> None:
        img = snap["depth_image"]
        fmt = snap["depth_format"]
        if img is None:
            msg = f"no depth (format={fmt!r})" if fmt else "no depth"
            self.depth_label.setText(msg)
            self.depth_meta.setText("—")
            return
        try:
            pm = depth_to_pixmap(img, target_w=max(320, self.depth_label.width()))
        except Exception as e:
            logger.exception("depth render failed")
            self.depth_label.setText(f"render error: {e}")
            return
        self.depth_label.setPixmap(pm)
        age_s = time.time() - snap["depth_ts"] if snap["depth_ts"] else 0.0
        valid_frac = float((img > 0).mean()) if img.size else 0.0
        self.depth_meta.setText(
            f"{snap['depth_width']}×{snap['depth_height']} "
            f"valid={valid_frac*100:4.1f}%  age={age_s:4.2f}s"
        )

    def _render_rgb(self, snap: dict) -> None:
        pending = snap["pending_rgb"]
        err = snap["rgb_error"]
        jpeg = snap["rgb_jpeg"]
        if err:
            self.rgb_label.setText(f"error: {err}")
        elif jpeg:
            pm = QPixmap()
            if not pm.loadFromData(jpeg):
                self.rgb_label.setText("jpeg decode failed")
            else:
                scaled = pm.scaled(
                    max(320, self.rgb_label.width()),
                    max(240, self.rgb_label.height()),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
                if (
                    self._vision_boxes
                    and snap["rgb_ts"] == self._vision_boxes_for_ts
                ):
                    scaled = _overlay_boxes(
                        scaled, self._vision_boxes,
                        snap["rgb_width"], snap["rgb_height"],
                    )
                self.rgb_label.setPixmap(scaled)
        elif pending:
            self.rgb_label.setText("awaiting RGB reply…")
        if snap["rgb_ts"] > 0 and jpeg:
            age_s = time.time() - snap["rgb_ts"]
            self.rgb_meta.setText(
                f"{snap['rgb_width']}×{snap['rgb_height']}  "
                f"req={snap['rgb_request_id'][:8]}…  age={age_s:4.2f}s"
            )

    def _render_local_map(self, snap: dict) -> None:
        grid = snap["local_map_grid"]
        meta = snap["local_map_meta"]
        ts = snap["local_map_ts"]
        self.local_map_view.update_map(grid, meta, ts)
        if grid is None or meta is None:
            self.local_map_meta.setText("—")
            return
        nx, ny = grid.shape
        res = float(meta.get("resolution_m", 0.0))
        age = time.time() - ts if ts > 0 else 0.0
        valid_frac = float((~np.isnan(grid)).mean()) if grid.size else 0.0
        parts = [
            f"{nx}×{ny} @ {int(round(res*100))}cm",
            f"valid={valid_frac*100:4.1f}%",
            f"age={age:4.2f}s",
        ]
        sources = meta.get("sources") or {}
        if isinstance(sources, dict):
            now = time.time()
            for k in ("lidar_ts", "depth_ts"):
                v = sources.get(k)
                if isinstance(v, (int, float)):
                    parts.append(f"{k.replace('_ts','')}={now - float(v):4.2f}s")
        self.local_map_meta.setText("  ".join(parts))

    def _render_driveable(self, snap: dict) -> None:
        drive = snap.get("local_map_driveable")
        meta = snap["local_map_meta"]
        ts = snap["local_map_ts"]
        self.driveable_view.update_map(drive, meta, ts)
        if meta is None:
            self.driveable_meta.setText("—")
            return
        if drive is None:
            self.driveable_meta.setText("driveable layer absent (Pi driveable_enabled off?)")
            return
        nx, ny = drive.shape
        total = drive.size
        clear = int((drive == 1).sum())
        blocked = int((drive == 0).sum())
        unknown = total - clear - blocked
        parts = [
            f"{nx}×{ny}",
            f"clear={clear*100.0/total:4.1f}%",
            f"blocked={blocked*100.0/total:4.1f}%",
            f"unknown={unknown*100.0/total:4.1f}%",
        ]
        clr = meta.get("driveable_clearance_height_m")
        if isinstance(clr, (int, float)):
            parts.append(f"clearance={float(clr):.2f}m")
        self.driveable_meta.setText("  ".join(parts))

    def _render_lidar(self, snap: dict) -> None:
        scan = snap["lidar"]
        self.lidar_view.update_scan(scan)
        if scan is None:
            self.lidar_meta.setText("—")
            return
        ranges = scan.get("ranges") or []
        age_s = time.time() - snap["lidar_ts"] if snap["lidar_ts"] else 0.0
        self.lidar_meta.setText(
            f"n={len(ranges)}  "
            f"scan_time_ms={scan.get('scan_time_ms', '?')}  "
            f"age={age_s:4.2f}s"
        )

    def closeEvent(self, event) -> None:
        # Halt any in-flight sweep before tearing down. The mission worker
        # publishes cmd_vel through the controller's session, so we have
        # to wait for it to release that responsibility before disconnect.
        try:
            sweep = getattr(self, "sweep_dock", None)
            if sweep is not None:
                sweep.request_abort()
                sweep.wait_for_mission(timeout_ms=2000)
        except Exception:
            logger.exception("sweep abort raised on close")
        try:
            self._jill_client.close()
        except Exception:
            logger.exception("jill_client close raised")
        try:
            self.controller.shutdown()
        except Exception:
            logger.exception("shutdown raised on close")
        super().closeEvent(event)


def _overlay_boxes(pm: QPixmap, boxes, src_w: int, src_h: int) -> QPixmap:
    """Draw detect boxes on a scaled pixmap, scaling coords from source dims."""
    if not src_w or not src_h:
        return pm
    sx = pm.width() / float(src_w)
    sy = pm.height() / float(src_h)
    out = QPixmap(pm)
    p = QPainter(out)
    try:
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        pen = QPen(QColor(255, 220, 60), 2)
        p.setPen(pen)
        p.setFont(QFont("Sans", 9, QFont.Weight.Bold))
        for b in boxes:
            try:
                x1, y1, x2, y2 = b.bbox
            except (TypeError, ValueError):
                continue
            rx, ry = int(x1 * sx), int(y1 * sy)
            rw, rh = int((x2 - x1) * sx), int((y2 - y1) * sy)
            p.drawRect(rx, ry, rw, rh)
            label = b.label if b.confidence is None else f"{b.label} {b.confidence:.2f}"
            p.drawText(rx + 2, max(ry - 2, 10), label)
    finally:
        p.end()
    return out


def _brief(obj) -> str:
    if obj is None:
        return "—"
    try:
        s = json.dumps(obj, separators=(",", ":"))
    except Exception:
        s = str(obj)
    if len(s) > 120:
        return s[:117] + "..."
    return s


# ── StubUI adapter ──────────────────────────────────────────────────

class QtUI(StubUI):
    def run(self) -> int:
        app = QApplication.instance() or QApplication(sys.argv)
        win = BodyStubWindow(self.controller, self.config)
        win.show()
        return app.exec()
