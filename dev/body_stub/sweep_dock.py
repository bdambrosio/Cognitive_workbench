"""SweepDock — controls + status + embedded preview for the sweep-360 mission.

Self-contained QDockWidget. Owns the SweepMission worker thread; signals
`mission_active(bool)` to the main window so it can disable competing
commanding controls (Live cmd checkbox, MotorTestDock) for the duration.
Abort button is always live, regardless of mission state.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QComboBox, QDockWidget, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QSizePolicy, QSpinBox, QVBoxLayout,
    QWidget,
)

from .sweep_mission import DEFAULT_PARAMS, SweepMission, SweepState

logger = logging.getLogger(__name__)


# Imported lazily inside __init__ to avoid a circular import with ui_qt
# (LocalMapView/DriveableView live there today).
LocalMapView = None  # type: ignore
DriveableView = None  # type: ignore


class SweepDock(QDockWidget):
    """Controls + status + small embedded preview for the sweep mission."""

    # Emitted when the mission becomes active/inactive — main window uses
    # this to disable the Live cmd checkbox and MotorTestDock engage btn.
    mission_active = pyqtSignal(bool)
    # Internal: marshals body/sweep/cmd payloads from the Zenoh thread to
    # the GUI thread via a queued connection.
    _external_cmd = pyqtSignal(dict)

    def __init__(self, controller, parent=None):
        super().__init__("Sweep-360", parent)
        self.controller = controller
        self.mission = SweepMission(controller, parent=self)

        # Lazy import to break cycle: ui_qt imports SweepDock.
        global LocalMapView, DriveableView
        if LocalMapView is None:
            from .ui_qt import LocalMapView as _L, DriveableView as _D
            LocalMapView = _L
            DriveableView = _D

        self._build_ui()
        self._wire_signals()
        # Repaint preview at the same rate as the main window.
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_preview)
        self._refresh_timer.start(200)  # 5 Hz; the accumulator only ticks
                                        # once per step so this is plenty.

    # ── UI build ─────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        body = QWidget(self)
        v = QVBoxLayout(body)
        v.setContentsMargins(6, 6, 6, 6)

        # ── Parameters group ────────────────────────────────────────
        params_box = QGroupBox("Sweep parameters")
        pl = QFormLayout(params_box)
        pl.setContentsMargins(6, 4, 6, 4)

        self.step_deg_box = QDoubleSpinBox()
        self.step_deg_box.setRange(1.0, 90.0)
        self.step_deg_box.setSingleStep(5.0)
        self.step_deg_box.setDecimals(1)
        self.step_deg_box.setValue(float(DEFAULT_PARAMS["step_deg"]))
        pl.addRow("step_deg:", self.step_deg_box)

        self.total_deg_box = QDoubleSpinBox()
        self.total_deg_box.setRange(30.0, 720.0)
        self.total_deg_box.setSingleStep(30.0)
        self.total_deg_box.setDecimals(1)
        self.total_deg_box.setValue(float(DEFAULT_PARAMS["total_deg"]))
        pl.addRow("total_deg:", self.total_deg_box)

        self.rate_box = QDoubleSpinBox()
        self.rate_box.setRange(5.0, 90.0)
        self.rate_box.setSingleStep(5.0)
        self.rate_box.setDecimals(1)
        self.rate_box.setValue(float(DEFAULT_PARAMS["angular_rate_dps"]))
        pl.addRow("rate (dps):", self.rate_box)

        self.settle_box = QSpinBox()
        self.settle_box.setRange(100, 10000)
        self.settle_box.setSingleStep(100)
        self.settle_box.setValue(int(DEFAULT_PARAMS["settle_ms"]))
        pl.addRow("settle (ms):", self.settle_box)

        self.dir_box = QComboBox()
        self.dir_box.addItems(["ccw", "cw"])
        pl.addRow("direction:", self.dir_box)
        v.addWidget(params_box)

        # ── Buttons ────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("Start sweep")
        btn_row.addWidget(self.start_btn)
        self.abort_btn = QPushButton("Abort")
        self.abort_btn.setStyleSheet(
            "QPushButton{background:#aa2222;color:white;font-weight:bold;}"
        )
        self.abort_btn.setEnabled(False)
        btn_row.addWidget(self.abort_btn)
        v.addLayout(btn_row)

        # ── Status block ───────────────────────────────────────────
        status_box = QGroupBox("Status")
        sv = QVBoxLayout(status_box)
        sv.setContentsMargins(6, 4, 6, 4)
        sv.setSpacing(2)
        mono = QFont("Monospace", 9)
        self.state_label = QLabel("state: idle")
        self.state_label.setFont(mono)
        sv.addWidget(self.state_label)
        self.progress_label = QLabel("step: —")
        self.progress_label.setFont(mono)
        sv.addWidget(self.progress_label)
        self.yaw_label = QLabel("yaw_accum: —")
        self.yaw_label.setFont(mono)
        sv.addWidget(self.yaw_label)
        self.sources_label = QLabel("yaw_sources: —")
        self.sources_label.setFont(mono)
        sv.addWidget(self.sources_label)
        self.fused_label = QLabel("last fused: —")
        self.fused_label.setFont(mono)
        sv.addWidget(self.fused_label)
        self.closure_label = QLabel("loop_closure: —")
        self.closure_label.setFont(mono)
        sv.addWidget(self.closure_label)
        self.reason_label = QLabel("")
        self.reason_label.setFont(mono)
        self.reason_label.setStyleSheet("color:#daa;")
        sv.addWidget(self.reason_label)
        v.addWidget(status_box)

        # ── Preview ────────────────────────────────────────────────
        self.preview_local = LocalMapView()
        self.preview_local.setMinimumSize(220, 220)
        self.preview_local.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        v.addWidget(QLabel("Sweep map (height)"))
        v.addWidget(self.preview_local)

        self.preview_drive = DriveableView()
        self.preview_drive.setMinimumSize(220, 220)
        self.preview_drive.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        v.addWidget(QLabel("Sweep map (driveable)"))
        v.addWidget(self.preview_drive)

        v.addStretch(1)
        self.setWidget(body)

    def _wire_signals(self) -> None:
        self.start_btn.clicked.connect(self._on_start_clicked)
        self.abort_btn.clicked.connect(self._on_abort_clicked)
        self.mission.state_changed.connect(self._on_state_changed)
        self.mission.step_complete.connect(self._on_step_complete)
        self.mission.mission_done.connect(self._on_mission_done)
        self.mission.accumulator_updated.connect(self._refresh_preview)
        # External body/sweep/cmd → controller (Zenoh thread) → signal → here.
        self._external_cmd.connect(self._on_external_cmd)
        self.controller.set_sweep_cmd_handler(self._external_cmd.emit)

    # ── User actions ─────────────────────────────────────────────────

    def current_params(self) -> Dict[str, Any]:
        return {
            "step_deg": float(self.step_deg_box.value()),
            "total_deg": float(self.total_deg_box.value()),
            "angular_rate_dps": float(self.rate_box.value()),
            "settle_ms": int(self.settle_box.value()),
            "direction": self.dir_box.currentText(),
        }

    def _on_start_clicked(self) -> None:
        self.start_external(params=self.current_params(), request_id=None)

    def start_external(
        self,
        params: Optional[Dict[str, Any]] = None,
        request_id: Optional[str] = None,
    ) -> bool:
        """Trigger a sweep from outside the dock (e.g. body/sweep/cmd)."""
        if self.mission.is_active():
            return False
        ok = self.mission.start_mission(params=params, request_id=request_id)
        if ok:
            # Push spinner values to match what the mission accepted, so
            # the UI doesn't lie when start_external used different params.
            if params is not None:
                _set = lambda b, k, conv: (
                    b.setValue(conv(params[k])) if k in params and params[k] is not None else None
                )
                _set(self.step_deg_box, "step_deg", float)
                _set(self.total_deg_box, "total_deg", float)
                _set(self.rate_box, "angular_rate_dps", float)
                _set(self.settle_box, "settle_ms", int)
                if params.get("direction") in ("ccw", "cw"):
                    self.dir_box.setCurrentText(params["direction"])
        return ok

    def _on_external_cmd(self, data: dict) -> None:
        """Handle a body/sweep/cmd payload routed from the Zenoh thread.

        Per spec §4.1: action ∈ {start, abort}; unknown actions ignored.
        """
        action = (data.get("action") or "").strip().lower()
        request_id = data.get("request_id")
        if action == "start":
            params = {
                k: data.get(k) for k in (
                    "step_deg", "total_deg", "angular_rate_dps",
                    "settle_ms", "direction",
                )
                if data.get(k) is not None
            }
            ok = self.start_external(params=params, request_id=request_id)
            if not ok:
                logger.info("external sweep/cmd start ignored: mission active")
        elif action == "abort":
            self.request_abort()
        else:
            logger.debug(f"sweep/cmd: unknown action {action!r}; ignoring")

    def _on_abort_clicked(self) -> None:
        if self.mission.is_active():
            self.mission.request_abort()
        else:
            # Treat as a clear of the last status text.
            self.reason_label.setText("")

    def request_abort(self) -> None:
        """External abort entry point (close, e-stop UI command, etc.)."""
        if self.mission.is_active():
            self.mission.request_abort()

    def wait_for_mission(self, timeout_ms: int = 3000) -> bool:
        """Block until the mission thread exits, or timeout. Returns True
        if the thread is no longer running.
        """
        if not self.mission.isRunning():
            return True
        return self.mission.wait(timeout_ms)

    # ── Mission signal handlers ──────────────────────────────────────

    def _on_state_changed(self, state_str: str) -> None:
        self.state_label.setText(f"state: {state_str}")
        active = self.mission.is_active()
        self.start_btn.setEnabled(not active)
        # Lock parameter editing while running.
        for w in (
            self.step_deg_box, self.total_deg_box, self.rate_box,
            self.settle_box, self.dir_box,
        ):
            w.setEnabled(not active)
        # Abort always live during non-idle; disabled when truly idle.
        self.abort_btn.setEnabled(active)
        self.mission_active.emit(active)
        if state_str in ("aborted", "estop", "error"):
            self.reason_label.setText(f"{state_str}: {self.mission._reason or ''}")
        elif state_str == "done":
            self.reason_label.setText("done")
        elif state_str == "idle":
            self.reason_label.setText("")

    def _on_step_complete(self, info: Dict[str, Any]) -> None:
        sources = info.get("yaw_sources") or {}
        def fmt(x):
            return "—" if x is None else f"{x:+.2f}°"
        self.sources_label.setText(
            f"yaw_sources: lidar={fmt(sources.get('lidar'))}  "
            f"imu={fmt(sources.get('imu'))}  cmd={fmt(sources.get('cmd'))}"
        )
        fused = info.get("fused_deg")
        conf = info.get("lidar_confidence", 0.0)
        self.fused_label.setText(
            f"last fused: {fmt(fused)}  (lidar conf={conf:.2f})"
        )

    def _on_mission_done(self, _final: Dict[str, Any]) -> None:
        # progress + closure populated by the periodic _refresh_preview tick
        self._refresh_preview()

    def _refresh_preview(self) -> None:
        # Update progress + closure + yaw_accum from the worker (cheap reads).
        m = self.mission
        if m.is_active() or m.state() in (
            SweepState.DONE, SweepState.ABORTED, SweepState.ESTOP, SweepState.ERROR,
        ):
            self.progress_label.setText(
                f"step: {m._step_index + 1}/{m._step_count}"
            )
            self.yaw_label.setText(f"yaw_accum: {m._yaw_accum_deg:+.2f}°")
            if m._loop_closure_deg is not None:
                self.closure_label.setText(
                    f"loop_closure: {m._loop_closure_deg:+.2f}°"
                )
        snap = m.snapshot_accumulator()
        if snap is None:
            return
        now = time.time()
        self.preview_local.update_map(snap["grid"], snap["meta"], now)
        self.preview_drive.update_map(snap["driveable"], snap["meta"], now)
