"""Dense world grid: layered evidence accumulator + fusion of incoming
local_2p5d frames + traversal stamping. See docs/world_map_spec.md §6.

Threading: all public mutating methods take an internal lock. Snapshot
methods return copies and are safe to call from a UI thread while the
fusion thread mutates.
"""
from __future__ import annotations

import logging
import math
import time
import uuid
from threading import RLock
from typing import Any, Dict, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


def _new_session_id() -> str:
    return uuid.uuid4().hex[:12]


class WorldGrid:
    """Six-layer dense world grid centered on world origin (0, 0).

    Layers:
        max_height_m       float32  NaN = unobserved
        clear_votes        int32    0   = no clear evidence
        block_votes        int32    0   = no block evidence
        traversed_ts       float32  NaN = never traversed
        last_observed_ts   float32  NaN = never observed
        observation_count  int32    0   = never touched
    """

    def __init__(
        self,
        *,
        extent_m: float,
        resolution_m: float,
        vote_margin: int,
        traversal_vote_weight: int,
        footprint_radius_m: float,
    ):
        if resolution_m <= 0:
            raise ValueError("resolution_m must be > 0")
        if extent_m <= 0:
            raise ValueError("extent_m must be > 0")
        self._lock = RLock()
        self._res = float(resolution_m)
        self._extent = float(extent_m)
        n = 2 * int(math.ceil(extent_m / resolution_m / 2.0))
        # Origin (world (0,0)) lands at the grid center cell boundary.
        self._n = n
        self._origin_x_m = -self._extent / 2.0
        self._origin_y_m = -self._extent / 2.0
        self._vote_margin = int(vote_margin)
        self._traversal_vote_weight = int(traversal_vote_weight)
        self._footprint_radius_m = float(footprint_radius_m)

        # Pre-compute footprint cell offsets (square mask, then circular).
        r_cells = int(math.ceil(self._footprint_radius_m / self._res))
        offs = []
        for di in range(-r_cells, r_cells + 1):
            for dj in range(-r_cells, r_cells + 1):
                if (di * self._res) ** 2 + (dj * self._res) ** 2 <= (
                    self._footprint_radius_m ** 2
                ):
                    offs.append((di, dj))
        self._footprint_offsets = offs

        self._allocate()
        self._session_id = _new_session_id()
        self._world_anchor_pose = (0.0, 0.0, 0.0)
        self._driveable_clearance_m: Optional[float] = None
        # Tight bounds of touched cells (i_min, i_max, j_min, j_max). None
        # until first touch.
        self._bounds_ij: Optional[Tuple[int, int, int, int]] = None

    # ── Allocation / reset ───────────────────────────────────────────

    def _allocate(self) -> None:
        n = self._n
        self.max_height_m = np.full((n, n), np.nan, dtype=np.float32)
        self.clear_votes = np.zeros((n, n), dtype=np.int32)
        self.block_votes = np.zeros((n, n), dtype=np.int32)
        self.traversed_ts = np.full((n, n), np.nan, dtype=np.float32)
        self.last_observed_ts = np.full((n, n), np.nan, dtype=np.float32)
        self.observation_count = np.zeros((n, n), dtype=np.int32)

    def reset(self, anchor_pose_in_old_world: Tuple[float, float, float]) -> str:
        """Clear all layers; mint new session_id. anchor_pose_in_old_world
        is informational — recorded in published messages so a consumer
        knows where the new world origin sits relative to the old.
        """
        with self._lock:
            self._allocate()
            self._bounds_ij = None
            self._session_id = _new_session_id()
            self._world_anchor_pose = (0.0, 0.0, 0.0)
            self._driveable_clearance_m = None
            return self._session_id

    # ── Geometry helpers ─────────────────────────────────────────────

    @property
    def session_id(self) -> str:
        return self._session_id

    @property
    def resolution_m(self) -> float:
        return self._res

    @property
    def n_cells(self) -> int:
        return self._n

    def world_to_cell(
        self, x_w: float, y_w: float,
    ) -> Tuple[int, int]:
        i = int(math.floor((x_w - self._origin_x_m) / self._res + 1e-9))
        j = int(math.floor((y_w - self._origin_y_m) / self._res + 1e-9))
        return i, j

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self._n and 0 <= j < self._n

    # ── Fusion ───────────────────────────────────────────────────────

    def fuse_local_map(
        self,
        *,
        grid: np.ndarray,
        driveable: Optional[np.ndarray],
        meta: Dict[str, Any],
        pose_world: Tuple[float, float, float],
        capture_ts: float,
    ) -> Tuple[int, int]:
        """Fold a single local_2p5d frame into the world grid using the
        supplied pose. Returns (cells_written, cells_clipped).
        """
        with self._lock:
            res_in = float(meta.get("resolution_m", 0.0))
            if res_in <= 0:
                logger.warning("fuse: bad resolution_m on local_map")
                return 0, 0
            if abs(res_in - self._res) > 1e-6:
                logger.warning(
                    f"fuse: resolution mismatch (local={res_in}, "
                    f"world={self._res}); skipping frame"
                )
                return 0, 0

            origin_x = float(meta.get("origin_x_m", 0.0))
            origin_y = float(meta.get("origin_y_m", 0.0))
            nx_b, ny_b = grid.shape

            clr = meta.get("driveable_clearance_height_m")
            if isinstance(clr, (int, float)):
                self._driveable_clearance_m = float(clr)

            # Body-frame cell centers.
            ii = np.arange(nx_b, dtype=np.float64)
            jj = np.arange(ny_b, dtype=np.float64)
            xb = origin_x + (ii + 0.5) * self._res
            yb = origin_y + (jj + 0.5) * self._res
            Xb, Yb = np.meshgrid(xb, yb, indexing="ij")

            # Apply T_world_body.
            x_w_pose, y_w_pose, theta = pose_world
            c, s = math.cos(theta), math.sin(theta)
            Xw = c * Xb - s * Yb + x_w_pose
            Yw = s * Xb + c * Yb + y_w_pose

            iw = np.floor((Xw - self._origin_x_m) / self._res + 1e-9).astype(np.int32)
            jw = np.floor((Yw - self._origin_y_m) / self._res + 1e-9).astype(np.int32)

            in_world = (iw >= 0) & (iw < self._n) & (jw >= 0) & (jw < self._n)
            n_in = int(np.count_nonzero(in_world))
            n_out = int(in_world.size - n_in)
            if n_in == 0:
                return 0, n_out

            iw_v = iw[in_world]
            jw_v = jw[in_world]
            src_h = grid[in_world]
            src_has = ~np.isnan(src_h)

            # max_height_m: nanmax with existing.
            tgt_h = self.max_height_m[iw_v, jw_v]
            merged_h = np.where(np.isnan(tgt_h), src_h, np.maximum(tgt_h, src_h))
            merged_h = np.where(src_has, merged_h, tgt_h)
            self.max_height_m[iw_v, jw_v] = merged_h

            # Driveable votes.
            if driveable is not None:
                src_d = driveable[in_world]
                clear_mask = src_d == 1
                block_mask = src_d == 0
                # np.add.at handles repeated indices correctly when several
                # source cells map to the same target cell.
                if np.any(clear_mask):
                    np.add.at(
                        self.clear_votes,
                        (iw_v[clear_mask], jw_v[clear_mask]),
                        1,
                    )
                if np.any(block_mask):
                    np.add.at(
                        self.block_votes,
                        (iw_v[block_mask], jw_v[block_mask]),
                        1,
                    )

            # observation_count and last_observed_ts.
            np.add.at(self.observation_count, (iw_v, jw_v), 1)
            ts32 = np.float32(capture_ts)
            cur = self.last_observed_ts[iw_v, jw_v]
            self.last_observed_ts[iw_v, jw_v] = np.where(
                np.isnan(cur) | (ts32 > cur), ts32, cur
            )

            self._extend_bounds(int(iw_v.min()), int(iw_v.max()),
                                int(jw_v.min()), int(jw_v.max()))
            return n_in, n_out

    def stamp_traversal(
        self,
        *,
        x_w: float,
        y_w: float,
        ts: float,
    ) -> int:
        """Mark a footprint disk around (x_w, y_w) as traversed."""
        with self._lock:
            ic, jc = self.world_to_cell(x_w, y_w)
            i_arr = np.array([ic + di for di, _dj in self._footprint_offsets],
                             dtype=np.int32)
            j_arr = np.array([jc + dj for _di, dj in self._footprint_offsets],
                             dtype=np.int32)
            in_world = (
                (i_arr >= 0) & (i_arr < self._n)
                & (j_arr >= 0) & (j_arr < self._n)
            )
            if not np.any(in_world):
                return 0
            iw_v = i_arr[in_world]
            jw_v = j_arr[in_world]
            ts32 = np.float32(ts)
            cur = self.traversed_ts[iw_v, jw_v]
            self.traversed_ts[iw_v, jw_v] = np.where(
                np.isnan(cur) | (ts32 > cur), ts32, cur
            )
            np.add.at(self.clear_votes, (iw_v, jw_v),
                      self._traversal_vote_weight)
            np.add.at(self.observation_count, (iw_v, jw_v), 1)
            cur_obs = self.last_observed_ts[iw_v, jw_v]
            self.last_observed_ts[iw_v, jw_v] = np.where(
                np.isnan(cur_obs) | (ts32 > cur_obs), ts32, cur_obs
            )
            self._extend_bounds(int(iw_v.min()), int(iw_v.max()),
                                int(jw_v.min()), int(jw_v.max()))
            return int(iw_v.size)

    # ── Read-side helpers ────────────────────────────────────────────

    def driveable_grid(self) -> np.ndarray:
        """int8 grid: 1=clear, 0=blocked, -1=unknown."""
        with self._lock:
            out = np.full((self._n, self._n), -1, dtype=np.int8)
            m = self._vote_margin
            out[self.clear_votes > self.block_votes + m] = 1
            out[self.block_votes > self.clear_votes + m] = 0
            return out

    def cells_observed(self) -> int:
        with self._lock:
            return int(np.count_nonzero(self.observation_count))

    def cells_traversed(self) -> int:
        with self._lock:
            return int(np.count_nonzero(~np.isnan(self.traversed_ts)))

    def bounds_world(self) -> Optional[Dict[str, float]]:
        with self._lock:
            if self._bounds_ij is None:
                return None
            i0, i1, j0, j1 = self._bounds_ij
            return {
                "min_x": self._origin_x_m + i0 * self._res,
                "max_x": self._origin_x_m + (i1 + 1) * self._res,
                "min_y": self._origin_y_m + j0 * self._res,
                "max_y": self._origin_y_m + (j1 + 1) * self._res,
            }

    def crop_for_publish(
        self, margin_cells: int,
    ) -> Optional[Dict[str, Any]]:
        """Return cropped layers + meta dict, or None if nothing to publish."""
        with self._lock:
            if self._bounds_ij is None:
                return None
            i0, i1, j0, j1 = self._bounds_ij
            i0 = max(0, i0 - margin_cells)
            j0 = max(0, j0 - margin_cells)
            i1 = min(self._n - 1, i1 + margin_cells)
            j1 = min(self._n - 1, j1 + margin_cells)
            sl = (slice(i0, i1 + 1), slice(j0, j1 + 1))
            crop = {
                "max_height_m": self.max_height_m[sl].copy(),
                "driveable": self._driveable_from_votes_locked()[sl].copy(),
                "observation_count": self.observation_count[sl].copy(),
                "last_observed_ts": self.last_observed_ts[sl].copy(),
                "traversed_ts": self.traversed_ts[sl].copy(),
                "origin_x_m": self._origin_x_m + i0 * self._res,
                "origin_y_m": self._origin_y_m + j0 * self._res,
                "nx": (i1 - i0 + 1),
                "ny": (j1 - j0 + 1),
                "resolution_m": self._res,
                "session_id": self._session_id,
                "world_anchor_pose": self._world_anchor_pose,
                "driveable_clearance_height_m": self._driveable_clearance_m,
                "bounds_m": self._bounds_world_locked(),
            }
            return crop

    def snapshot_for_ui(self) -> Optional[Dict[str, Any]]:
        """Cheap snapshot for UI rendering: only what the map widgets need."""
        with self._lock:
            if self._bounds_ij is None:
                return None
            return {
                "grid": self.max_height_m.copy(),
                "driveable": self._driveable_from_votes_locked(),
                "meta": {
                    "resolution_m": self._res,
                    "origin_x_m": self._origin_x_m,
                    "origin_y_m": self._origin_y_m,
                    "nx": self._n,
                    "ny": self._n,
                    "frame": "world",
                    "driveable_clearance_height_m": self._driveable_clearance_m,
                },
                "session_id": self._session_id,
            }

    # ── Internal ─────────────────────────────────────────────────────

    def _driveable_from_votes_locked(self) -> np.ndarray:
        out = np.full((self._n, self._n), -1, dtype=np.int8)
        m = self._vote_margin
        out[self.clear_votes > self.block_votes + m] = 1
        out[self.block_votes > self.clear_votes + m] = 0
        return out

    def _extend_bounds(self, i0: int, i1: int, j0: int, j1: int) -> None:
        if self._bounds_ij is None:
            self._bounds_ij = (i0, i1, j0, j1)
            return
        a0, a1, b0, b1 = self._bounds_ij
        self._bounds_ij = (
            min(a0, i0), max(a1, i1), min(b0, j0), max(b1, j1),
        )

    def _bounds_world_locked(self) -> Optional[Dict[str, float]]:
        if self._bounds_ij is None:
            return None
        i0, i1, j0, j1 = self._bounds_ij
        return {
            "min_x": self._origin_x_m + i0 * self._res,
            "max_x": self._origin_x_m + (i1 + 1) * self._res,
            "min_y": self._origin_y_m + j0 * self._res,
            "max_y": self._origin_y_m + (j1 + 1) * self._res,
        }


def encode_for_publish(
    crop: Dict[str, Any],
    *,
    pose_source_name: str,
    include_evidence: bool = True,
) -> Dict[str, Any]:
    """Convert a cropped grid into the JSON-serializable wire shape.

    Driveable is encoded as nested booleans/None, max_height as floats/None,
    matching the local_2p5d schema so existing viewers can render it.
    """
    nx = int(crop["nx"])
    ny = int(crop["ny"])
    mh = crop["max_height_m"]
    dr = crop["driveable"]
    max_h_rows = [
        [None if math.isnan(v) else float(v) for v in row]
        for row in mh
    ]
    drive_rows = [
        [True if v == 1 else (False if v == 0 else None) for v in row]
        for row in dr
    ]
    out: Dict[str, Any] = {
        "ts": time.time(),
        "frame": "world",
        "kind": "max_height_grid",
        "resolution_m": float(crop["resolution_m"]),
        "origin_x_m": float(crop["origin_x_m"]),
        "origin_y_m": float(crop["origin_y_m"]),
        "nx": nx,
        "ny": ny,
        "max_height_m": max_h_rows,
        "driveable": drive_rows,
        "session_id": crop["session_id"],
        "world_anchor_pose": {
            "x_m": float(crop["world_anchor_pose"][0]),
            "y_m": float(crop["world_anchor_pose"][1]),
            "theta_rad": float(crop["world_anchor_pose"][2]),
        },
        "bounds_m": crop["bounds_m"],
        "pose_source": pose_source_name,
    }
    clr = crop.get("driveable_clearance_height_m")
    if isinstance(clr, (int, float)):
        out["driveable_clearance_height_m"] = float(clr)
    if include_evidence:
        oc = crop["observation_count"]
        lo = crop["last_observed_ts"]
        tr = crop["traversed_ts"]
        out["observation_count"] = [
            [int(v) if v > 0 else 0 for v in row] for row in oc
        ]
        out["last_observed_ts"] = [
            [None if math.isnan(v) else float(v) for v in row] for row in lo
        ]
        out["traversed_ts"] = [
            [None if math.isnan(v) else float(v) for v in row] for row in tr
        ]
    return out
