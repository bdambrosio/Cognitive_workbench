"""Lidar yaw estimation by 1-D angular cross-correlation.

Pure numpy. No dependencies on Zenoh or Qt — unit-testable with synthetic
scans. Used by sweep_mission to recover the per-step delta-yaw between
the pre-rotation and post-settle scans.

Convention (matches docs/sweep360_spec.md §4.1 and the body lidar frame):
- Body frame: 0 rad = forward (+x); +π/2 = robot-left (+y).
- Returned `deg` is positive when the robot rotated CCW (angular_z > 0)
  between scan_a and scan_b.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

import numpy as np


def _scan_to_vector(scan: Dict[str, Any], n_bins: int) -> Optional[np.ndarray]:
    """Resample a lidar scan into a fixed-length range vector indexed by
    angle bin in [0, 2π). Returns None if the scan is unusable.
    Empty bins remain NaN; multi-sample bins are averaged.
    """
    if not isinstance(scan, dict):
        return None
    ranges = scan.get("ranges") or []
    n = len(ranges)
    if n == 0:
        return None
    angle_min = float(scan.get("angle_min", 0.0))
    angle_inc = scan.get("angle_increment")
    if angle_inc is None:
        angle_inc = (2.0 * math.pi) / n
    else:
        angle_inc = float(angle_inc)
    if angle_inc <= 0.0:
        return None
    range_max = scan.get("range_max")
    range_max = float(range_max) if isinstance(range_max, (int, float)) and range_max > 0 else math.inf

    bin_step = (2.0 * math.pi) / n_bins
    out = np.full(n_bins, np.nan, dtype=np.float32)
    counts = np.zeros(n_bins, dtype=np.int32)
    two_pi = 2.0 * math.pi
    for i, r in enumerate(ranges):
        try:
            rv = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(rv) or rv <= 0.0 or rv > range_max:
            continue
        a = (angle_min + i * angle_inc) % two_pi
        b = int(a / bin_step) % n_bins
        if counts[b] == 0:
            out[b] = rv
        else:
            out[b] = (out[b] * counts[b] + rv) / (counts[b] + 1)
        counts[b] += 1
    if not np.any(counts):
        return None
    return out


def _zero_mean_filled(v: np.ndarray) -> np.ndarray:
    """Fill NaNs with the per-vector mean and subtract the mean. Returns
    a zero-mean signal suitable for circular correlation.
    """
    mask = np.isnan(v)
    if mask.all():
        return np.zeros_like(v)
    m = float(np.nanmean(v))
    out = v.copy()
    out[mask] = m
    return out - m


def estimate_lidar_corr(
    scan_a: Dict[str, Any],
    scan_b: Dict[str, Any],
    *,
    n_bins: int = 360,
) -> Tuple[Optional[float], float]:
    """Estimate Δyaw between two scans by circular cross-correlation.

    Returns (deg_or_none, confidence).
        deg ∈ (-180, 180]; positive = CCW rotation from scan_a to scan_b.
        confidence ∈ [0, 1]; higher means a sharper correlation peak.
    Returns (None, 0.0) if either scan is unusable.
    """
    va = _scan_to_vector(scan_a, n_bins=n_bins)
    vb = _scan_to_vector(scan_b, n_bins=n_bins)
    if va is None or vb is None:
        return None, 0.0
    a = _zero_mean_filled(va)
    b = _zero_mean_filled(vb)

    # Circular cross-correlation via FFT.
    # corr[k] = sum_i a[i] * b[(i + k) mod N]
    A = np.fft.rfft(a)
    B = np.fft.rfft(b)
    corr = np.fft.irfft(np.conj(A) * B, n=n_bins)

    k = int(np.argmax(corr))
    bin_step_deg = 360.0 / n_bins
    # When the robot rotates CCW by Δθ, the angular bin at index i in
    # scan_a corresponds to bin (i - Δθ_bins) in scan_b. The argmax of
    # the cross-correlation above gives k such that scan_b best matches
    # scan_a shifted by +k → Δθ_bins = -k. A CCW rotation thus shows up
    # as k = N - Δθ_bins, i.e. a wrap-around to the high end. Map back
    # to the (-180, 180] range:
    deg = -k * bin_step_deg
    deg = ((deg + 180.0) % 360.0) - 180.0

    peak = float(corr[k])
    mean_abs = float(np.mean(np.abs(corr)))
    if mean_abs <= 0.0:
        confidence = 0.0
    else:
        ratio = peak / mean_abs
        # tanh-squash so featureless rooms (ratio ≈ 1) score near zero
        # and well-textured rooms (ratio ≳ 5) approach 1.
        confidence = float(max(0.0, math.tanh((ratio - 1.0) / 4.0)))
    return float(deg), confidence
