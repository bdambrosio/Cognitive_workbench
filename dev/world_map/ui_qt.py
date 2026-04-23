"""Qt main window for the world-map fuser.

Layout:
    [router] [Connect] [Reset world] [status-line]
    +---------------------+---------------------+
    | World height (turbo)| World driveable     |
    +---------------------+---------------------+
    [pose] [rates] [cells observed/traversed] [notes]
"""
from __future__ import annotations

import logging
import math
import time
from typing import Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPushButton, QVBoxLayout, QWidget,
)

from .config import FuserConfig
from .controller import FuserController
from .map_views import WorldDriveableView, WorldHeightView

logger = logging.getLogger(__name__)


class WorldMapWindow(QMainWindow):
    def __init__(self, controller: FuserController, config: FuserConfig):
        super().__init__()
        self.setWindowTitle("World map fuser")
        self.controller = controller
        self.config = config

        self._build_ui()
        self._build_timer()

    # ── Layout ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        central = QWidget(self)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)

        # Top bar: router + connect + reset + status hint
        top = QHBoxLayout()
        top.addWidget(QLabel("Router:"))
        self._router_edit = QLineEdit(self.config.router)
        self._router_edit.setMinimumWidth(220)
        top.addWidget(self._router_edit)
        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        top.addWidget(self._connect_btn)
        self._reset_btn = QPushButton("Reset world")
        self._reset_btn.setToolTip(
            "Clear the accumulated world map and re-anchor to the "
            "robot's current pose. Use after you physically move the "
            "robot or want to start a fresh map."
        )
        self._reset_btn.clicked.connect(self._on_reset_clicked)
        self._reset_btn.setEnabled(False)
        top.addWidget(self._reset_btn)
        self._top_status = QLabel("disconnected")
        self._top_status.setStyleSheet("color: #aaa;")
        top.addWidget(self._top_status, stretch=1)
        outer.addLayout(top)

        # Map row
        maps = QHBoxLayout()
        self._height_view = WorldHeightView(stale_s=self.config.map_stale_s)
        self._drive_view = WorldDriveableView(stale_s=self.config.map_stale_s)
        maps.addWidget(self._height_view, stretch=1)
        maps.addWidget(self._drive_view, stretch=1)
        outer.addLayout(maps, stretch=1)

        # Bottom status strip
        bot = QHBoxLayout()
        self._pose_lbl = QLabel("pose: —")
        self._rates_lbl = QLabel("rates: —")
        self._cells_lbl = QLabel("cells: —")
        self._session_lbl = QLabel("session: —")
        self._notes_lbl = QLabel("")
        self._notes_lbl.setStyleSheet("color: #e8a; font-weight: bold;")
        for w in (self._pose_lbl, self._rates_lbl,
                  self._cells_lbl, self._session_lbl):
            w.setStyleSheet("color: #ccc;")
            bot.addWidget(w)
        bot.addWidget(self._notes_lbl, stretch=1)
        outer.addLayout(bot)

        self.setCentralWidget(central)
        self.resize(1000, 720)

    def _build_timer(self) -> None:
        self._redraw_timer = QTimer(self)
        self._redraw_timer.setInterval(
            int(1000.0 / max(1.0, self.config.ui_redraw_hz))
        )
        self._redraw_timer.timeout.connect(self._on_redraw_tick)
        self._redraw_timer.start()

    # ── Slots ────────────────────────────────────────────────────────

    def _on_connect_clicked(self) -> None:
        new_router = self._router_edit.text().strip()
        if new_router:
            self.config.router = new_router
            self.controller.config.router = new_router
        ok, err = self.controller.connect()
        if not ok:
            self._top_status.setText(f"connect failed: {err}")
            self._top_status.setStyleSheet("color: #f88;")
            return
        self._top_status.setText(f"connected → {self.config.router}")
        self._top_status.setStyleSheet("color: #8c8;")
        self._connect_btn.setEnabled(False)
        self._router_edit.setEnabled(False)
        self._reset_btn.setEnabled(True)

    def _on_reset_clicked(self) -> None:
        self.controller.request_reset(reason="ui_reset")

    def _on_redraw_tick(self) -> None:
        snap = self.controller.snapshot_for_ui()
        latest = self.controller.pose_source.latest_pose()
        pose = latest[0] if latest is not None else None
        ts = time.time()
        if snap is not None:
            self._height_view.update_map(
                snap["grid"], snap["meta"], ts, pose=pose,
            )
            self._drive_view.update_map(
                snap["driveable"], snap["meta"], ts, pose=pose,
            )
        else:
            # Empty state — let the views render their "no map" hint.
            self._height_view.update_map(None, None, 0.0, pose=pose)
            self._drive_view.update_map(None, None, 0.0, pose=pose)

        st = self.controller.status_summary()
        if pose is not None:
            self._pose_lbl.setText(
                f"pose: x={pose[0]:+.2f} y={pose[1]:+.2f} "
                f"θ={math.degrees(pose[2]):+.1f}°"
            )
        else:
            self._pose_lbl.setText("pose: (no odom)")

        rates = st["rates"]
        ages = st["ages"]
        rate_parts = []
        for name, hz, age in (
            ("lm", rates["local_map"], ages["local_map"]),
            ("od", rates["odom"], ages["odom"]),
        ):
            hz_s = f"{hz:.1f}Hz" if hz is not None else "—"
            age_s = f"{age:.2f}s" if age is not None else "—"
            rate_parts.append(f"{name} {hz_s}/{age_s}")
        self._rates_lbl.setText("rates: " + "  ".join(rate_parts))

        self._cells_lbl.setText(
            f"cells: obs={st['cells_observed']} trav={st['cells_traversed']}"
        )
        self._session_lbl.setText(
            f"session: {st['session_id'][:8]}  ({st['pose_source']})"
        )
        self._notes_lbl.setText(st["notes"] or "")

    # ── Lifecycle ────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        try:
            self.controller.shutdown()
        except Exception:
            logger.exception("controller shutdown raised")
        super().closeEvent(event)


def run_app(controller: FuserController, config: FuserConfig) -> int:
    app = QApplication.instance() or QApplication([])
    win = WorldMapWindow(controller, config)
    win.show()
    return app.exec()
