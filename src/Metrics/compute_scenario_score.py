#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import math
import argparse
import shutil
import subprocess
from typing import Any, Dict, List, Optional

def run_jq_digest(line: str, jq_path: str) -> Optional[Dict[str, Any]]:
    """Run digest.jq on a single JSON line and return parsed digest dict or None on failure."""
    try:
        p = subprocess.run(
            ["jq", "-f", jq_path],
            input=line.encode("utf-8"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if p.returncode != 0:
            return None
        out = p.stdout.decode("utf-8").strip()
        if not out:
            return None
        return json.loads(out)
    except Exception:
        return None

def clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))

def linear_declining_weights(n: int, last_fraction: float = 0.5) -> List[float]:
    """Return n weights where w_n = last_fraction * w_1, linear decline, normalized to sum 1."""
    if n <= 0:
        return []
    if n == 1:
        return [1.0]
    alpha = (1.0 - last_fraction) / (n - 1)
    raw = [max(0.0, 1.0 - alpha * i) for i in range(n)]
    s = sum(raw) or 1.0
    return [w / s for w in raw]

def survival_penalty(max_state: Optional[float], soft_start: float = 70.0, danger: float = 80.0, hard: float = 100.0) -> float:
    """Map a max physiological state [0..100] to [0..1] penalty.
    - <= soft_start: 1.0
    - danger..hard: linearly down to 0.0
    - between soft_start and danger: gentle linear ramp from 1.0 to 0.75
    """
    if max_state is None:
        return 1.0
    x = float(max_state)
    x = max(0.0, min(100.0, x))
    if x <= soft_start:
        return 1.0
    if x < danger:
        return 1.0 - 0.25 * ((x - soft_start) / (danger - soft_start))
    if x >= hard:
        return 0.0
    return 0.75 * (1.0 - (x - danger) / (hard - danger))

def time_weight(actual_minutes: Optional[float], fallback: float = 1.0) -> float:
    try:
        m = float(actual_minutes) if actual_minutes is not None else float(fallback)
        return max(0.0, m)
    except Exception:
        return float(fallback)

def extract_from_raw(line_obj: Dict[str, Any]) -> Dict[str, Any]:
    """Fallback extraction when jq digest isn't available."""
    metrics = line_obj.get("metrics", {}) if isinstance(line_obj, dict) else {}
    time_m = metrics.get("time", {}) if isinstance(metrics, dict) else {}
    drives = (metrics.get("drive_fulfillment") or metrics.get("drive_fullfillment") or {}).get("drives", [])
    gs = metrics.get("goal_satisfaction", None)

    hunger = (metrics.get("hunger") or {})
    fatigue = (metrics.get("fatigue") or {})
    thirst  = (metrics.get("thirst")  or {})

    return {
        "time": {"actual_minutes": time_m.get("actual_minutes", time_m.get("minutes_advanced"))},
        "drive_fulfillment": {"drives": drives},
        "goal_satisfaction": gs,
        "hunger": {"max": hunger.get("max"), "end": hunger.get("end")},
        "fatigue": {"max": fatigue.get("max"), "end": fatigue.get("end")},
        "thirst": {"max": thirst.get("max"), "end": thirst.get("end")},
    }

def max_state_from_entry(d: Dict[str, Any]) -> Optional[float]:
    """Take max across hunger/fatigue/thirst using 'max' then 'end'."""
    vals = []
    for key in ["hunger", "fatigue", "thirst"]:
        blk = d.get(key) or {}
        for k in ["max", "end"]:
            v = blk.get(k)
            try:
                if v is not None:
                    vals.append(float(v))
            except Exception:
                pass
    if not vals:
        return None
    return max(vals)

def drive_vector_from_entry(d: Dict[str, Any]) -> List[float]:
    """Extract drive scores (0..1) in listed order."""
    drives = (((d.get("drive_fulfillment") or {}).get("drives")) or [])
    vec = []
    for dr in drives:
        try:
            vec.append(float(dr.get("score", 0.0)))
        except Exception:
            vec.append(0.0)
    return vec

def scenario_score_for_file(
    jsonl_path: str,
    jq_path: Optional[str],
    last_fraction: float = 0.5,
    soft_start: float = 70.0,
    danger: float = 80.0,
    hard: float = 100.0,
    goal_sat_weight: float = 0.0,
) -> Dict[str, Any]:
    """Compute scenario score and return a dict report."""
    use_jq = jq_path is not None and shutil.which("jq") is not None and os.path.exists(jq_path)

    total_time = 0.0
    accum = 0.0
    n_lines = 0

    per_plan: List[Dict[str, Any]] = []
    drive_dim = None
    avg_goal_sat_num = 0.0
    avg_goal_sat_den = 0.0
    avg_drive_num = 0.0  # NEW: time-weighted sum of drive-only (pre-survival)

    with open(jsonl_path, "r", encoding="utf-8") as f:
        for raw in f:
            raw = raw.strip()
            if not raw:
                continue
            n_lines += 1

            d = None
            if use_jq:
                d = run_jq_digest(raw, jq_path)
            if d is None:
                try:
                    d = extract_from_raw(json.loads(raw))
                except Exception:
                    continue

            tmin = d.get("time", {}).get("actual_minutes")
            wt = time_weight(tmin, fallback=1.0)

            dv = drive_vector_from_entry(d)
            if drive_dim is None:
                drive_dim = len(dv)
            elif drive_dim != len(dv) and len(dv) > 0:
                if len(dv) < drive_dim:
                    dv = dv + [0.0] * (drive_dim - len(dv))
                else:
                    dv = dv[:drive_dim]

            if drive_dim is None or drive_dim == 0:
                w = []
                dv_weighted = 0.0
            else:
                w = linear_declining_weights(drive_dim, last_fraction=last_fraction)
                dv_weighted = sum((dv[i] if i < len(dv) else 0.0) * w[i] for i in range(drive_dim))

            avg_drive_num += dv_weighted * wt
            mstate = max_state_from_entry(d)
            pen = survival_penalty(mstate, soft_start=soft_start, danger=danger, hard=hard)

            contrib = dv_weighted * pen
            accum += contrib * wt
            total_time += wt

            gs = d.get("goal_satisfaction", None)
            if isinstance(gs, (int, float)):
                avg_goal_sat_num += float(gs) * wt
                avg_goal_sat_den += wt

            per_plan.append({
                "minutes": wt,
                "drive_weighted": round(dv_weighted, 4),
                "survival_penalty": round(pen, 4),
                "contribution": round(contrib, 4),
                "goal_satisfaction": gs if gs is None else round(float(gs), 4),
                "max_state": mstate if mstate is None else round(float(mstate), 2),
            })

    base = (accum / total_time) if total_time > 0 else 0.0
    base = clamp(base, 0.0, 1.0)


    avg_drive = (avg_drive_num / total_time) if total_time > 0 else 0.0
    avg_drive = clamp(avg_drive, 0.0, 1.0)
    if goal_sat_weight > 0.0 and avg_goal_sat_den > 0:
        avg_gs = clamp(avg_goal_sat_num / avg_goal_sat_den, 0.0, 1.0)
        blended = (1.0 - goal_sat_weight) * base + goal_sat_weight * (base * avg_gs)
    else:
        avg_gs = None
        blended = base

    return {
        "file": jsonl_path,
        "used_digest_jq": bool(use_jq),
        "total_plans": n_lines,
        "total_minutes": total_time,
        "drive_dimensions": drive_dim or 0,
        "weights_last_fraction": last_fraction,
        "survival_params": {"soft_start": soft_start, "danger": danger, "hard": hard},
        "goal_sat_weight": goal_sat_weight,
        "avg_goal_satisfaction": avg_gs if avg_gs is None else round(float(avg_gs), 4),
        "avg_drive_fulfillment": round(avg_drive, 4),
        "scenario_score_base": round(base, 4),
        "scenario_score_final": round(blended, 4),
        "per_plan": per_plan,
    }

def main():
    ap = argparse.ArgumentParser(description="Compute scenario score from plans.jsonl using digest.jq (optional).")
    ap.add_argument("--plans", required=True, help="Path to plans.jsonl")
    ap.add_argument("--digest", default=None, help="Path to digest.jq (optional but recommended)")
    ap.add_argument("--last-fraction", type=float, default=0.5, help="Weight of last drive relative to first (default 0.5)")
    ap.add_argument("--soft-start", type=float, default=70.0, help="Survival soft start threshold (default 70)")
    ap.add_argument("--danger", type=float, default=80.0, help="Survival danger threshold (default 80)")
    ap.add_argument("--hard", type=float, default=100.0, help="Survival hard cutoff (default 100)")
    ap.add_argument("--goal-sat-weight", type=float, default=0.0, help="Blend factor [0..1] to include avg goal satisfaction multiplicatively")
    ap.add_argument("--out", default=None, help="Write JSON report to this file")
    args = ap.parse_args()

    report = scenario_score_for_file(
        jsonl_path=args.plans,
        jq_path=args.digest,
        last_fraction=args.last_fraction,
        soft_start=args.soft_start,
        danger=args.danger,
        hard=args.hard,
        goal_sat_weight=args.goal_sat_weight,
    )

    out = json.dumps(report, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(out + "\n")
        print(f"Wrote report to {args.out}")
    else:
        print(out)

if __name__ == "__main__":
    main()
