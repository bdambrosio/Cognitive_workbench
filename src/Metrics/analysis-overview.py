#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
List scenario summaries from src/Metrics/analysis (top level only).

Displays a table with:
- Directory name (subdir under analysis)
- Note (first 8 chars of manifest note if present)
- Total number of plans
- Total actions (if available)
- Duration (total minutes)
- Activity% (percent of plans with activity present)
- Score (scenario_score_final)

KISS: no recursion, no arguments, no try/except.
"""

import os
import sys
import json
import argparse
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from scipy import stats


def _is_number(value: Any) -> bool:
    if isinstance(value, (int, float)):
        return True
    return False


def _read_json(path: Path) -> Optional[Dict[str, Any]]:
    # Preconditions only; no exceptions are caught
    if not path.exists() or not path.is_file():
        return None
    with path.open("r", encoding="utf-8") as f:
        text = f.read()
    if not text or not text.strip().startswith("{"):
        return None
    data = json.loads(text)
    if not isinstance(data, dict):
        return None
    return data


def scan_analysis(analysis_dir: Path) -> List[Tuple[str, str, Optional[int], Optional[int], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float]]]:
    rows: List[Tuple[str, str, Optional[int], Optional[int], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float]]] = []
    if not analysis_dir.exists() or not analysis_dir.is_dir():
        return rows

    # Top-level subdirectories only
    for entry in sorted(analysis_dir.iterdir(), key=lambda p: p.name):
        if not entry.is_dir():
            continue
        name = entry.name
        score_path = entry / "scenario_score.json"
        if not score_path.exists() or not score_path.is_file():
            # Missing summary file; log and skip
            print(f"warning: missing scenario_score.json in {name}", file=sys.stderr)
            continue
        data = _read_json(score_path)
        if data is None:
            # Unreadable or invalid json; log and skip
            print(f"warning: invalid JSON in {score_path}", file=sys.stderr)
            continue

        total_plans_raw = data.get("total_plans")
        total_plans: Optional[int]
        if isinstance(total_plans_raw, int):
            total_plans = total_plans_raw
        elif isinstance(total_plans_raw, float):
            total_plans = int(total_plans_raw)
        else:
            total_plans = None

        total_actions_raw = data.get("total_actions")
        total_actions: Optional[int]
        if isinstance(total_actions_raw, int):
            total_actions = total_actions_raw
        elif isinstance(total_actions_raw, float):
            total_actions = int(total_actions_raw)
        else:
            total_actions = None

        total_minutes_raw = data.get("total_minutes")
        total_minutes: Optional[float]
        if _is_number(total_minutes_raw):
            total_minutes = float(total_minutes_raw)
        else:
            total_minutes = None

        actpct_raw = data.get("activity_present_percent")
        actpct: Optional[float]
        if _is_number(actpct_raw):
            actpct = float(actpct_raw)
        else:
            actpct = None

        score_raw = data.get("scenario_score_final")
        score: Optional[float]
        if _is_number(score_raw):
            score = float(score_raw)
        else:
            score = None

        # Manifest note prefix (first 8 chars if present)
        manifest_path = entry / "manifest.json"
        note8 = ""
        man = _read_json(manifest_path)
        if isinstance(man, dict):
            note = man.get("note")
            if isinstance(note, str) and len(note) > 0:
                note8 = note[:8]

        # Additional summary metrics
        plan_avg_raw = data.get("avg_plan_score")
        plan_avg: Optional[float]
        if _is_number(plan_avg_raw):
            plan_avg = float(plan_avg_raw)
        else:
            plan_avg = None

        goal_avg_raw = data.get("avg_goal_satisfaction")
        goal_avg: Optional[float]
        if _is_number(goal_avg_raw):
            goal_avg = float(goal_avg_raw)
        else:
            goal_avg = None

        drive_avg_raw = data.get("avg_drive_score")
        drive_avg: Optional[float]
        if _is_number(drive_avg_raw):
            drive_avg = float(drive_avg_raw)
        else:
            drive_avg = None

        overall = score

        rows.append((name, note8, total_plans, total_actions, total_minutes, actpct, score, plan_avg, goal_avg, drive_avg, overall))

    return rows


def display_table(rows: List[Tuple[str, str, Optional[int], Optional[int], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float]]]) -> None:
    # Headers
    header_idx = "Idx"
    header_dir = "Directory"
    header_note = "Note"
    header_plans = "Plans"
    header_actions = "Actions"
    header_minutes = "Duration"
    header_act = "Activity%"
    header_score = "Score"
    header_plan = "plan"
    header_goal = "goal"
    header_drive = "drive"
    header_overall = "overall"

    # Pre-format row values to compute widths including data
    formatted: List[Tuple[str, str, str, str, str, str, str, str, str, str, str]] = []
    for name, note8, plans, actions, minutes, actpct, score, plan_avg, goal_avg, drive_avg, overall in rows:
        plans_str = str(plans) if isinstance(plans, int) else "N/A"
        actions_str = str(actions) if isinstance(actions, int) else "N/A"
        minutes_str = (f"{minutes:.1f}" if isinstance(minutes, float) else "N/A")
        act_str = (f"{actpct:.1f}%" if isinstance(actpct, float) else "N/A")
        score_str = (f"{score:.3f}" if isinstance(score, float) else "N/A")
        plan_str = (f"{plan_avg:.3f}" if isinstance(plan_avg, float) else "N/A")
        goal_str = (f"{goal_avg:.3f}" if isinstance(goal_avg, float) else "N/A")
        drive_str = (f"{drive_avg:.3f}" if isinstance(drive_avg, float) else "N/A")
        overall_str = (f"{overall:.3f}" if isinstance(overall, float) else "N/A")
        formatted.append((name, note8, plans_str, actions_str, minutes_str, act_str, score_str, plan_str, goal_str, drive_str, overall_str))

    # Compute column widths (max of header and data lengths)
    idx_width = max(len(header_idx), len(str(len(rows)))) if rows else len(header_idx)
    dir_width = max(len(header_dir), *(len(f[0]) for f in formatted)) if formatted else len(header_dir)
    note_width = max(len(header_note), *(len(f[1]) for f in formatted)) if formatted else len(header_note)
    plans_width = max(len(header_plans), *(len(f[2]) for f in formatted)) if formatted else len(header_plans)
    actions_width = max(len(header_actions), *(len(f[3]) for f in formatted)) if formatted else len(header_actions)
    minutes_width = max(len(header_minutes), *(len(f[4]) for f in formatted)) if formatted else len(header_minutes)
    act_width = max(len(header_act), *(len(f[5]) for f in formatted)) if formatted else len(header_act)
    score_width = max(len(header_score), *(len(f[6]) for f in formatted)) if formatted else len(header_score)
    plan_width = max(len(header_plan), *(len(f[7]) for f in formatted)) if formatted else len(header_plan)
    goal_width = max(len(header_goal), *(len(f[8]) for f in formatted)) if formatted else len(header_goal)
    drive_width = max(len(header_drive), *(len(f[9]) for f in formatted)) if formatted else len(header_drive)
    overall_width = max(len(header_overall), *(len(f[10]) for f in formatted)) if formatted else len(header_overall)

    # Header
    print(f"{header_idx:>{idx_width}}  {header_dir:<{dir_width}}  {header_note:<{note_width}}  {header_plans:>{plans_width}}  {header_actions:>{actions_width}}  {header_minutes:>{minutes_width}}  {header_act:>{act_width}}  {header_score:>{score_width}}  {header_plan:>{plan_width}}  {header_goal:>{goal_width}}  {header_drive:>{drive_width}}  {header_overall:>{overall_width}}")

    # Rows
    idx = 1
    for (name, note8, plans_str, actions_str, minutes_str, act_str, score_str, plan_str, goal_str, drive_str, overall_str) in formatted:
        print(f"{idx:>{idx_width}}  {name:<{dir_width}}  {note8:<{note_width}}  {plans_str:>{plans_width}}  {actions_str:>{actions_width}}  {minutes_str:>{minutes_width}}  {act_str:>{act_width}}  {score_str:>{score_width}}  {plan_str:>{plan_width}}  {goal_str:>{goal_width}}  {drive_str:>{drive_width}}  {overall_str:>{overall_width}}")
        idx += 1


def parse_selection_from_rows(sel: str, rows: List[Tuple[str, Optional[int], Optional[int], Optional[float], Optional[float], Optional[float]]]) -> List[str]:
    names: List[str] = []
    if not sel:
        return names
    tokens = [t.strip() for t in sel.split(",") if t.strip()]
    index_to_name = [r[0] for r in rows]
    for tok in tokens:
        if ("-" in tok) and tok.replace("-", "").isdigit():
            parts = tok.split("-")
            if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                start = int(parts[0])
                end = int(parts[1])
                if start <= end:
                    i = start
                    while i <= end:
                        if 1 <= i <= len(index_to_name):
                            names.append(index_to_name[i - 1])
                        i += 1
            continue
        if tok.isdigit():
            i = int(tok)
            if 1 <= i <= len(index_to_name):
                names.append(index_to_name[i - 1])
            continue
        if tok in index_to_name:
            names.append(tok)
    # de-dup preserve order
    seen = set()
    uniq: List[str] = []
    for n in names:
        if n not in seen:
            uniq.append(n)
            seen.add(n)
    return uniq


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    analysis_dir = script_dir / "analysis"
    if not analysis_dir.exists() or not analysis_dir.is_dir():
        print(f"error: analysis directory not found at {analysis_dir}", file=sys.stderr)
        sys.exit(1)

    ap = argparse.ArgumentParser(description="Overview and utilities for analysis summaries.")
    ap.add_argument("--rerun", default=None, help="Comma/range selection of dirs or names to recompute (e.g., '1,3-5' or 'Name1,Name2')")
    ap.add_argument("--pair-a", dest="pair_a", default=None, help="Selection for Pair A (indices/ranges or names)")
    ap.add_argument("--pair-b", dest="pair_b", default=None, help="Selection for Pair B (indices/ranges or names)")
    ap.add_argument("--metrics", default="avg_goal_satisfaction,avg_drive_alignment,scenario_score_base,scenario_score_final,avg_plan_score,avg_drive_score,avg_state_geomean,avg_failure_rate", help="Comma-separated metrics to export/test")
    ap.add_argument("--out", dest="out_csv", default=None, help="Write pair export CSV to this path")
    ap.add_argument("--t-test", dest="do_ttest", action="store_true", help="Run Welch's t-tests for selected metrics over A vs B")
    args = ap.parse_args()

    rows = scan_analysis(analysis_dir)
    display_table(rows)

    # Interactive loop
    sel_a: List[str] = []
    sel_b: List[str] = []

    def show_sel() -> None:
        print("A:", ", ".join(sel_a) if sel_a else "(empty)")
        print("B:", ", ".join(sel_b) if sel_b else "(empty)")

    while True:
        line = input("cmd> ").strip()
        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            break
        if line.lower() in ("help", "h", "?"):
            print("commands: rerun LIST | a LIST | b LIST | ttest | show | clear a|b|both | q")
            continue
        if line.lower().startswith("show"):
            show_sel()
            continue
        if line.lower().startswith("clear "):
            which = line.split(None, 1)[1].strip().lower()
            if which == "a":
                sel_a = []
            elif which == "b":
                sel_b = []
            elif which == "both":
                sel_a = []
                sel_b = []
            else:
                print("error: use clear a|b|both", file=sys.stderr)
            continue
        if line.lower().startswith("rerun"):
            arg = line.split(None, 1)[1].strip() if len(line.split(None, 1)) > 1 else ""
            targets = parse_selection_from_rows(arg, rows)
            cs_path = script_dir / "compute_scenario_score.py"
            for name in targets:
                run_dir = analysis_dir / name
                character = name.split("-")[0] if "-" in name else name
                plans_path = run_dir / f"{character}-plans.jsonl"
                out_path = run_dir / "scenario_score.json"
                if plans_path.exists() and plans_path.is_file():
                    cmd = [sys.executable or "python3", str(cs_path), "--plans", str(plans_path), "--out", str(out_path)]
                    subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, check=False)
                else:
                    print(f"warning: missing plans file for {name}: {plans_path}", file=sys.stderr)
            rows = scan_analysis(analysis_dir)
            display_table(rows)
            continue
        if line.lower().startswith("a "):
            arg = line.split(None, 1)[1].strip()
            targets = parse_selection_from_rows(arg, rows)
            # Remove from B, add to A, de-dup
            sel_b = [n for n in sel_b if n not in targets]
            for n in targets:
                if n not in sel_a:
                    sel_a.append(n)
            show_sel()
            continue
        if line.lower().startswith("b "):
            arg = line.split(None, 1)[1].strip()
            targets = parse_selection_from_rows(arg, rows)
            # Remove from A, add to B, de-dup
            sel_a = [n for n in sel_a if n not in targets]
            for n in targets:
                if n not in sel_b:
                    sel_b.append(n)
            show_sel()
            continue
        if line.lower().startswith("ttest"):
            # Auto-select groups if none provided: A = activity% > 75, B = activity% < 10
            if len(sel_a) == 0 and len(sel_b) == 0:
                for row in rows:
                    name, _plans, _actions, _minutes, actpct, _score = row
                    if isinstance(actpct, float):
                        if actpct > 75.0 and name not in sel_a:
                            sel_a.append(name)
                        elif actpct < 10.0 and name not in sel_b:
                            sel_b.append(name)
                if len(sel_a) == 0 or len(sel_b) == 0:
                    print("warning: auto-selection produced empty A or B; add selections with 'a'/'b' first", file=sys.stderr)
                    continue

            metrics_list = [m.strip() for m in ("avg_goal_satisfaction,avg_drive_alignment,scenario_score_base,scenario_score_final,avg_plan_score,avg_drive_score,avg_state_geomean,avg_failure_rate").split(",") if m.strip()]

            # Collect Welch rows
            welch_rows: List[Tuple[str, str, str, str, str, str, str, str, str, str, str]] = []
            for metric in metrics_list:
                a_vals: List[float] = []
                b_vals: List[float] = []
                for name in sel_a:
                    d = _read_json(analysis_dir / name / "scenario_score.json")
                    v = d.get(metric) if isinstance(d, dict) else None
                    if isinstance(v, (int, float)):
                        a_vals.append(float(v))
                for name in sel_b:
                    d = _read_json(analysis_dir / name / "scenario_score.json")
                    v = d.get(metric) if isinstance(d, dict) else None
                    if isinstance(v, (int, float)):
                        b_vals.append(float(v))
                nA = len(a_vals)
                nB = len(b_vals)
                if nA >= 2 and nB >= 2:
                    t_res = stats.ttest_ind(a_vals, b_vals, equal_var=False, nan_policy="omit")
                    meanA = sum(a_vals) / nA
                    meanB = sum(b_vals) / nB
                    varA = sum((x - meanA) * (x - meanA) for x in a_vals) / (nA - 1)
                    varB = sum((x - meanB) * (x - meanB) for x in b_vals) / (nB - 1)
                    stdA = varA ** 0.5
                    stdB = varB ** 0.5
                    meanD = meanA - meanB
                    seD = (varA / nA + varB / nB) ** 0.5
                    tval = float(t_res.statistic) if t_res.statistic is not None else float('nan')
                    pval = float(t_res.pvalue) if t_res.pvalue is not None else float('nan')
                    welch_rows.append((metric, str(nA), f"{meanA:.4f}", f"{stdA:.4f}", str(nB), f"{meanB:.4f}", f"{stdB:.4f}", f"{meanD:.4f}", f"{seD:.4f}", f"{tval:.4f}", f"{pval:.4g}"))
                else:
                    welch_rows.append((metric, str(nA), "N/A", "N/A", str(nB), "N/A", "N/A", "N/A", "N/A", "N/A", "N/A"))

            # Compute widths and print aligned Welch table
            headers = ("metric", "nA", "meanA", "stdA", "nB", "meanB", "stdB", "meanDiff", "seDiff", "t", "p")
            widths = [len(h) for h in headers]
            for row in welch_rows:
                for i, col in enumerate(row):
                    if len(col) > widths[i]:
                        widths[i] = len(col)
            print("Welch's t-tests (A vs B):")
            print(" ".join(h.ljust(widths[i]) for i, h in enumerate(headers)))
            for row in welch_rows:
                print(" ".join(row[i].rjust(widths[i]) if i in (1,3,4,7,8,9,10) else row[i].ljust(widths[i]) for i in range(len(row))))

            # Paired t-tests if same selection lengths
            if len(sel_a) == len(sel_b) and len(sel_a) >= 2:
                paired_rows: List[Tuple[str, str, str, str, str, str, str, str, str, str, str]] = []
                for metric in metrics_list:
                    paired_a: List[float] = []
                    paired_b: List[float] = []
                    i = 0
                    while i < len(sel_a):
                        name_a = sel_a[i]
                        name_b = sel_b[i]
                        d_a = _read_json(analysis_dir / name_a / "scenario_score.json")
                        d_b = _read_json(analysis_dir / name_b / "scenario_score.json")
                        v_a = d_a.get(metric) if isinstance(d_a, dict) else None
                        v_b = d_b.get(metric) if isinstance(d_b, dict) else None
                        if isinstance(v_a, (int, float)) and isinstance(v_b, (int, float)):
                            paired_a.append(float(v_a))
                            paired_b.append(float(v_b))
                        i += 1
                    n = len(paired_a)
                    if n >= 2 and n == len(paired_b):
                        t_res = stats.ttest_rel(paired_a, paired_b, nan_policy="omit")
                        meanA = sum(paired_a) / n
                        meanB = sum(paired_b) / n
                        varA = sum((x - meanA) * (x - meanA) for x in paired_a) / (n - 1)
                        varB = sum((x - meanB) * (x - meanB) for x in paired_b) / (n - 1)
                        stdA = varA ** 0.5
                        stdB = varB ** 0.5
                        diffs = [paired_a[j] - paired_b[j] for j in range(n)]
                        meanD = sum(diffs) / n
                        varD = sum((d - meanD) * (d - meanD) for d in diffs) / (n - 1)
                        stdD = varD ** 0.5
                        tval = float(t_res.statistic) if t_res.statistic is not None else float('nan')
                        pval = float(t_res.pvalue) if t_res.pvalue is not None else float('nan')
                        paired_rows.append((metric, str(n), f"{meanA:.4f}", f"{stdA:.4f}", str(n), f"{meanB:.4f}", f"{stdB:.4f}", f"{meanD:.4f}", f"{stdD:.4f}", f"{tval:.4f}", f"{pval:.4g}"))
                    else:
                        paired_rows.append((metric, str(n), "N/A", "N/A", str(n), "N/A", "N/A", "N/A", "N/A", "N/A", "N/A"))

                headers2 = ("metric", "n", "meanA", "stdA", "n", "meanB", "stdB", "meanDiff", "stdDiff", "t", "p")
                widths2 = [len(h) for h in headers2]
                for row in paired_rows:
                    for i, col in enumerate(row):
                        if len(col) > widths2[i]:
                            widths2[i] = len(col)
                print()
                print("Paired t-tests (A vs B):")
                print(" ".join(h.ljust(widths2[i]) for i, h in enumerate(headers2)))
                for row in paired_rows:
                    print(" ".join(row[i].rjust(widths2[i]) if i in (1,3,4,7,8,9,10) else row[i].ljust(widths2[i]) for i in range(len(row))))
                continue
        print("error: unknown command. Try 'help'", file=sys.stderr)

    # Helper to map selection strings to directory names
    def parse_selection(sel: str) -> List[str]:
        names: List[str] = []
        if not sel:
            return names
        tokens = [t.strip() for t in sel.split(",") if t.strip()]
        index_to_name = [r[0] for r in rows]
        for tok in tokens:
            if "-" in tok and tok.replace("-", "").isdigit():
                parts = tok.split("-")
                if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                    start = int(parts[0])
                    end = int(parts[1])
                    if start <= end:
                        for i in range(start, end + 1):
                            if 1 <= i <= len(index_to_name):
                                names.append(index_to_name[i - 1])
                continue
            if tok.isdigit():
                i = int(tok)
                if 1 <= i <= len(index_to_name):
                    names.append(index_to_name[i - 1])
                continue
            # treat as literal name
            if tok in index_to_name:
                names.append(tok)
        # de-dup preserve order
        seen = set()
        uniq: List[str] = []
        for n in names:
            if n not in seen:
                uniq.append(n)
                seen.add(n)
        return uniq

    # Rerun compute for selected directories
    if isinstance(args.rerun, str) and len(args.rerun) > 0:
        sel_names = parse_selection(args.rerun)
        cs_path = script_dir / "compute_scenario_score.py"
        for name in sel_names:
            run_dir = analysis_dir / name
            character = name.split("-")[0] if "-" in name else name
            plans_path = run_dir / f"{character}-plans.jsonl"
            out_path = run_dir / "scenario_score.json"
            if plans_path.exists() and plans_path.is_file():
                cmd = [
                    sys.executable or "python3",
                    str(cs_path),
                    "--plans", str(plans_path),
                    "--out", str(out_path),
                ]
                subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, check=False)
            else:
                print(f"warning: missing plans file for {name}: {plans_path}", file=sys.stderr)

    # Pair selections and optional export / t-tests
    if (isinstance(args.pair_a, str) and len(args.pair_a) > 0) or (isinstance(args.pair_b, str) and len(args.pair_b) > 0):
        names_a = parse_selection(args.pair_a or "")
        names_b = parse_selection(args.pair_b or "")

        # Load metrics dicts by dir
        def load_metrics(name: str) -> Optional[Dict[str, Any]]:
            score_path = analysis_dir / name / "scenario_score.json"
            return _read_json(score_path)

        metrics_list = [m.strip() for m in (args.metrics or "").split(",") if m.strip()]

        # Export CSV
        if args.out_csv:
            lines: List[str] = []
            header = ["group", "dir", "metric", "value"]
            lines.append(",".join(header))
            for group, sel in (("A", names_a), ("B", names_b)):
                for name in sel:
                    d = load_metrics(name)
                    if isinstance(d, dict):
                        for metric in metrics_list:
                            v = d.get(metric)
                            if isinstance(v, (int, float)):
                                lines.append(f"{group},{name},{metric},{float(v)}")
            outp = Path(args.out_csv)
            outp.parent.mkdir(parents=True, exist_ok=True)
            with outp.open("w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            print(f"wrote pairs CSV to {outp}")

        # Welch's t-tests
        if args.do_ttest:
            print()
            print("Welch's t-tests (A vs B):")
            print("metric,nA,meanA,stdA,nB,meanB,stdB,t,p")
            for metric in metrics_list:
                a_vals: List[float] = []
                b_vals: List[float] = []
                for name in names_a:
                    d = _read_json(analysis_dir / name / "scenario_score.json")
                    v = d.get(metric) if isinstance(d, dict) else None
                    if isinstance(v, (int, float)):
                        a_vals.append(float(v))
                for name in names_b:
                    d = _read_json(analysis_dir / name / "scenario_score.json")
                    v = d.get(metric) if isinstance(d, dict) else None
                    if isinstance(v, (int, float)):
                        b_vals.append(float(v))
                nA = len(a_vals)
                nB = len(b_vals)
                if nA >= 2 and nB >= 2:
                    t_res = stats.ttest_ind(a_vals, b_vals, equal_var=False, nan_policy="omit")
                    meanA = sum(a_vals) / nA if nA > 0 else 0.0
                    meanB = sum(b_vals) / nB if nB > 0 else 0.0
                    varA = sum((x - meanA) * (x - meanA) for x in a_vals) / (nA - 1) if nA > 1 else 0.0
                    varB = sum((x - meanB) * (x - meanB) for x in b_vals) / (nB - 1) if nB > 1 else 0.0
                    stdA = varA ** 0.5
                    stdB = varB ** 0.5
                    tval = float(t_res.statistic) if t_res.statistic is not None else float('nan')
                    pval = float(t_res.pvalue) if t_res.pvalue is not None else float('nan')
                    print(f"{metric},{nA},{meanA:.4f},{stdA:.4f},{nB},{meanB:.4f},{stdB:.4f},{tval:.4f},{pval:.4g}")
                else:
                    print(f"{metric},{nA},N/A,N/A,{nB},N/A,N/A,N/A,N/A")


if __name__ == "__main__":
    main()


