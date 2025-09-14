#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import math
import argparse
import shutil
import subprocess
import hashlib
from datetime import datetime, timezone
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

    # Counts: prefer metrics.steps if present, otherwise derive from actions
    steps_m = metrics.get("steps", {}) if isinstance(metrics, dict) else {}
    actions = line_obj.get("actions", []) if isinstance(line_obj, dict) else []

    # Initialize counts from metrics if available
    steps_total = steps_m.get("total") if isinstance(steps_m, dict) else None
    moves = steps_m.get("moves") if isinstance(steps_m, dict) else None
    takes = steps_m.get("takes") if isinstance(steps_m, dict) else None
    uses = steps_m.get("uses") if isinstance(steps_m, dict) else None
    inspects = steps_m.get("inspects") if isinstance(steps_m, dict) else None
    scans = steps_m.get("scans") if isinstance(steps_m, dict) else None
    failures = steps_m.get("failures") if isinstance(steps_m, dict) else None

    # Derive from actions where missing
    if (steps_total is None) and isinstance(actions, list):
        steps_total = len(actions)
    def _count_type(t: str) -> int:
        if not isinstance(actions, list):
            return 0
        return sum(1 for a in actions if isinstance(a, dict) and isinstance(a.get("action"), dict) and str(a.get("action", {}).get("type", "")).lower() == t)
    if moves is None:
        moves = _count_type("move")
    if takes is None:
        takes = _count_type("take")
    if uses is None:
        uses = _count_type("use")
    if inspects is None:
        inspects = _count_type("inspect")
    if scans is None:
        scans = _count_type("scan")
    if failures is None:
        if not isinstance(actions, list):
            failures = 0
        else:
            failures = sum(1 for a in actions if isinstance(a, dict) and (a.get("failure_code") is not None))

    return {
        "time": {"actual_minutes": time_m.get("actual_minutes", time_m.get("minutes_advanced"))},
        "drive_fulfillment": {"drives": drives},
        "goal_satisfaction": gs,
        # Prefer root-level plan_score, then metrics.plan_score for parity with digest
        "summary_score": (
            (line_obj.get("plan_score") if isinstance(line_obj, dict) else None)
            or (metrics.get("plan_score") if isinstance(metrics, dict) else None)
        ),
        "counts": {
            "steps_total": steps_total or 0,
            "moves": moves or 0,
            "takes": takes or 0,
            "uses": uses or 0,
            "inspects": inspects or 0,
            "scans": scans or 0,
            "failures": failures or 0,
        },
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

def state_geomean_from_entry(d: Dict[str, Any]) -> Optional[float]:
    """Geometric mean over hunger/fatigue/thirst using 'max' then 'end', scaled to [0,1]."""
    vals = []
    for key in ["hunger", "fatigue", "thirst"]:
        blk = d.get(key) or {}
        v = blk.get("max")
        if v is None:
            v = blk.get("end")
        if v is None:
            continue
        try:
            x = float(v)
            # scale to [0,1]
            x = max(0.0, min(100.0, x)) / 100.0
            vals.append(x)
        except Exception:
            continue
    if not vals:
        return None
    # geometric mean
    try:
        prod = 1.0
        for x in vals:
            # ensure non-negative
            prod *= max(0.0, x)
        gm = prod ** (1.0 / len(vals))
        return gm
    except Exception:
        return None

def drive_vector_from_entry(d: Dict[str, Any]) -> List[float]:
    """Extract drive scores (0..1) in listed order."""
    drives = (((d.get("drive_fulfillment") or {}).get("drives")) or [])
    vec = []
    for dr in drives:
        try:
            # Back-compat: prefer plan_score, fallback to score
            v = dr.get("plan_score")
            if v is None:
                v = dr.get("score", 0.0)
            vec.append(float(v))
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
    plan_score_sum = 0.0
    plan_score_den = 0.0
    drive_score_sum = 0.0
    drive_score_den = 0.0
    state_gm_sum = 0.0
    state_gm_den = 0.0
    failure_rate_sum = 0.0
    failure_rate_den = 0.0
    activity_present_count = 0
    activity_total_count = 0
    total_actions = 0

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

            # Accumulate activity presence (treat None, False, and "None" string as no activity)
            try:
                raw_obj = json.loads(raw)
                activity_total_count += 1
                activity_val = raw_obj.get("activity") if isinstance(raw_obj, dict) else None
                is_present = not (
                    activity_val is None
                    or activity_val is False
                    or (isinstance(activity_val, str) and activity_val.strip().lower() == "none")
                )
                if is_present:
                    activity_present_count += 1
            except Exception:
                pass

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


            gs = d.get("goal_satisfaction", None)
            if isinstance(gs, (int, float)):
                avg_goal_sat_num += float(gs) * wt
                avg_goal_sat_den += wt

            # Per-plan drive_score: prefer digest-provided drive_score, else dv_weighted * goal_satisfaction
            ds_val = d.get("drive_score")
            if ds_val is None and isinstance(gs, (int, float)):
                try:
                    ds_val = float(dv_weighted) * float(gs)
                except Exception:
                    ds_val = None
            if isinstance(ds_val, (int, float)):
                drive_score_sum += float(ds_val)
                drive_score_den += 1.0

            # Per-plan extras
            # plan_score from digest summary_score (already 0..1)
            ps_raw = d.get("summary_score")
            ps_scaled = None
            try:
                if ps_raw is not None:
                    ps_scaled = clamp(float(ps_raw), 0.0, 1.0)
                    plan_score_sum += ps_scaled
                    plan_score_den += 1.0
            except Exception:
                ps_scaled = None

            contrib = dv_weighted * (ps_scaled + gs)/2 * pen
            accum += contrib * wt
            total_time += wt

            # State geomean (0..1)
            st_gm = state_geomean_from_entry(d)
            if isinstance(st_gm, (int, float)):
                state_gm_sum += float(st_gm)
                state_gm_den += 1.0

            # Failure rate per plan
            try:
                counts = d.get("counts", {}) or {}
                steps_total = counts.get("steps_total")
                failures = counts.get("failures")
                if steps_total and float(steps_total) > 0:
                    fr = max(0.0, float(failures or 0) / float(steps_total))
                    failure_rate_sum += fr
                    failure_rate_den += 1.0
                else:
                    fr = 0.0
            except Exception:
                fr = None

            # Actions per plan and total (steps_total)
            actions_val = None
            counts2 = d.get("counts", {}) or {}
            st = counts2.get("steps_total")
            if isinstance(st, (int, float)):
                actions_val = int(st)
                total_actions += int(st)

            per_plan.append({
                "minutes": wt,
                "drive_weighted": round(dv_weighted, 4),
                "drive_fulfillment": round(dv_weighted, 4),
                "survival_penalty": round(pen, 4),
                "contribution": round(contrib, 4),
                "goal_satisfaction": gs if gs is None else round(float(gs), 4),
                "plan_score": (None if ps_scaled is None else round(float(ps_scaled), 4)),
                "drive_score": (None if ds_val is None else round(float(ds_val), 4)),
                "state_geomean": (None if st_gm is None else round(float(st_gm), 4)),
                "max_state": mstate if mstate is None else round(float(mstate), 2),
                "failure_rate": fr if fr is None else round(float(fr), 4),
                "actions": actions_val,
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

    # Scenario-level new aggregates (unweighted means over plans with values)
    avg_plan_score = (plan_score_sum / plan_score_den) if plan_score_den > 0 else None
    avg_drive_score = (drive_score_sum / drive_score_den) if drive_score_den > 0 else None
    avg_state_geomean = (state_gm_sum / state_gm_den) if state_gm_den > 0 else None
    avg_failure_rate = (failure_rate_sum / failure_rate_den) if failure_rate_den > 0 else None

    return {
        "file": jsonl_path,
        "used_digest_jq": bool(use_jq),
        "total_plans": n_lines,
        "activity_present_percent": (None if activity_total_count == 0 else round(100.0 * activity_present_count / activity_total_count, 1)),
        "total_minutes": total_time,
        "drive_dimensions": drive_dim or 0,
        "weights_last_fraction": last_fraction,
        "survival_params": {"soft_start": soft_start, "danger": danger, "hard": hard},
        "goal_sat_weight": goal_sat_weight,
        "avg_goal_satisfaction": avg_gs if avg_gs is None else round(float(avg_gs), 4),
        "avg_drive_alignment": round(avg_drive, 4),
        "scenario_score_base": round(base, 4),
        "scenario_score_final": round(blended, 4),
        "avg_plan_score": (None if avg_plan_score is None else round(float(avg_plan_score), 4)),
        "avg_drive_score": (None if avg_drive_score is None else round(float(avg_drive_score), 4)),
        "avg_state_geomean": (None if avg_state_geomean is None else round(float(avg_state_geomean), 4)),
        "avg_failure_rate": (None if avg_failure_rate is None else round(float(avg_failure_rate), 4)),
        "total_actions": int(total_actions),
        "per_plan": per_plan,
    }

def main():
    ap = argparse.ArgumentParser(description="Compute scenario score from plans.jsonl using digest.jq (optional).")
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument("--plans", help="Path to plans.jsonl")
    grp.add_argument("--character", help="Character name to resolve input as data/<Character>-plans.jsonl")
    ap.add_argument("--digest", default=None, help="Path to digest.jq (optional but recommended)")
    ap.add_argument("--last-fraction", type=float, default=0.5, help="Weight of last drive relative to first (default 0.5)")
    ap.add_argument("--soft-start", type=float, default=70.0, help="Survival soft start threshold (default 70)")
    ap.add_argument("--danger", type=float, default=80.0, help="Survival danger threshold (default 80)")
    ap.add_argument("--hard", type=float, default=100.0, help="Survival hard cutoff (default 100)")
    ap.add_argument("--goal-sat-weight", type=float, default=0.25, help="Blend factor [0..1] to include avg goal satisfaction multiplicatively")
    ap.add_argument("--out", default=None, help="Write JSON report to this file (overrides analysis dir write)")
    ap.add_argument("--yes", action="store_true", help="Overwrite scenario_score.json without prompting if it exists")
    ap.add_argument("-m", "--manifest-note", dest="manifest_note", default=None, help="Freeform note stored in manifest.json (non --out mode)")
    args = ap.parse_args()

    # Resolve input path and character
    if args.plans:
        plans_path = args.plans
        character = None
        base = os.path.basename(plans_path)
        if base.endswith("-plans.jsonl"):
            character = base[:-len("-plans.jsonl")]
    else:
        character = (args.character or "").strip()
        character = character.capitalize() if character else character
        plans_path = os.path.join("..", "data", f"{character}-plans.jsonl")

    if not os.path.exists(plans_path):
        print(f"ERROR: plans file not found: {plans_path}", file=sys.stderr)
        sys.exit(2)

    # Compute report
    report = scenario_score_for_file(
        jsonl_path=plans_path,
        jq_path=args.digest,
        last_fraction=args.last_fraction,
        soft_start=args.soft_start,
        danger=args.danger,
        hard=args.hard,
        goal_sat_weight=args.goal_sat_weight,
    )

    out_json = json.dumps(report, indent=2)

    # If explicit output path provided, honor it for backward-compat
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(out_json + "\n")
        print(f"Wrote report to {args.out}")
        return

    # Create analysis directory: analysis/<Character>-<YYYYMMDD-HHMMSS>
    # Derive character if still unknown
    if not character:
        base = os.path.basename(plans_path)
        character = base[:-len("-plans.jsonl")] if base.endswith("-plans.jsonl") else os.path.splitext(base)[0]

    st = os.stat(plans_path)
    mtime = datetime.fromtimestamp(st.st_mtime, tz=timezone.utc)
    ts_str = mtime.strftime("%Y%m%d-%H%M%S")
    run_dir = os.path.join("analysis", f"{character}-{ts_str}")
    os.makedirs(run_dir, exist_ok=True)

    # Copy input plans into run directory
    dst_plans = os.path.join(run_dir, os.path.basename(plans_path))
    try:
        shutil.copy2(plans_path, dst_plans)
    except Exception:
        # Fallback to copy without metadata
        shutil.copy(plans_path, dst_plans)

    # Prepare output path
    score_path = os.path.join(run_dir, "scenario_score.json")
    if os.path.exists(score_path) and not args.yes:
        try:
            ans = input(f"File exists: {score_path}. Overwrite? [y/N]: ").strip().lower()
        except Exception:
            ans = "n"
        if ans not in ("y", "yes"):
            print("Aborted by user (won't overwrite).", file=sys.stderr)
            sys.exit(3)

    with open(score_path, "w", encoding="utf-8") as f:
        f.write(out_json + "\n")
    print(f"Wrote scenario score to {score_path}")

    # Write manifest.json
    try:
        # Compute SHA256 of input
        sha256 = hashlib.sha256()
        with open(plans_path, "rb") as pf:
            for chunk in iter(lambda: pf.read(8192), b""):
                sha256.update(chunk)
        manifest = {
            "character_name": character,
            "input_path": os.path.abspath(plans_path),
            "input_basename": os.path.basename(plans_path),
            "input_mtime_utc": mtime.isoformat(),
            "input_sha256": sha256.hexdigest(),
            "analysis_dir": os.path.abspath(run_dir),
            "scenario_score_path": os.path.abspath(score_path),
            "used_digest_jq": bool(args.digest and shutil.which("jq") and os.path.exists(args.digest)),
            "digest_cli": (["jq", "-f", args.digest] if args.digest else None),
            "compute_cli": sys.argv,
            "note": args.manifest_note,
        }
        with open(os.path.join(run_dir, "manifest.json"), "w", encoding="utf-8") as mf:
            json.dump(manifest, mf, indent=2)
        print(f"Wrote manifest to {os.path.join(run_dir, 'manifest.json')}")
    except Exception as e:
        print(f"WARNING: failed to write manifest: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()
