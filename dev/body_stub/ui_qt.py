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
import sys
import time
from typing import Optional

import numpy as np

from PyQt6.QtCore import Qt, QPointF, QTimer
from PyQt6.QtGui import (
    QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF,
)
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPlainTextEdit,
    QPushButton, QSizePolicy, QTabWidget, QVBoxLayout, QWidget,
)

from .ui_base import StubUI

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
        self._max_range_m: float = 6.0

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
            range_max = float(self._scan.get("range_max", self._max_range_m))
            if range_max <= 0.0:
                range_max = self._max_range_m

            p.setPen(QPen(QColor(120, 220, 255), 2))
            for i, r in enumerate(ranges):
                try:
                    rv = float(r)
                except Exception:
                    continue
                if not math.isfinite(rv) or rv <= 0.0 or rv > range_max:
                    continue
                a = angle_min + i * angle_inc
                # Body frame: 0 rad = forward (+x), +π/2 = robot-left (+y).
                # Screen: +y is down. Bird's-eye view → rotate 90° CCW so
                # forward points UP on screen and robot-left points LEFT.
                scale = (rv / range_max) * radius
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


# ── Main window ─────────────────────────────────────────────────────

class BodyStubWindow(QMainWindow):
    def __init__(self, controller, config):
        super().__init__()
        self.controller = controller
        self.config = config
        self.setWindowTitle("Body Stub — dev tool (do not run alongside Jill)")
        self.resize(980, 720)
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
        self.live_box = QCheckBox("Live command (publish heartbeat + cmd_vel)")
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

    def _wire_signals(self) -> None:
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.live_box.toggled.connect(self._on_live_toggled)
        self.apply_btn.clicked.connect(self._on_apply_clicked)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
        self.rgb_btn.clicked.connect(self._on_rgb_clicked)

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
        self.controller.stop_all()
        self.live_box.setChecked(False)

    def _on_rgb_clicked(self) -> None:
        req = self.controller.request_rgb()
        if req is None:
            self.rgb_meta.setText("request failed (not connected?)")
        else:
            self.rgb_meta.setText(f"request_id {req[:8]}… pending")

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
            )
        return snap

    def _tick(self) -> None:
        snap = self._snapshot()
        self._render_status(snap)
        self._render_depth(snap)
        self._render_rgb(snap)
        self._render_lidar(snap)
        self._render_local_map(snap)

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
        try:
            self.controller.shutdown()
        except Exception:
            logger.exception("shutdown raised on close")
        super().closeEvent(event)


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
