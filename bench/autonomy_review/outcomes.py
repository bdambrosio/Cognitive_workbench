#!/usr/bin/env python3
"""autonomy_review/outcomes — M1 fire-outcome readout (docs/harness-roadmap.md).

Aggregates the `fire_outcome` records that fire-outcome capture Phase 1
(docs/fire-outcome-capture.md, reflection stage 6) appends to
`autonomy.jsonl` into the baseline table the roadmap's M1 asks for:

  - outcome distribution (helped/neutral/hindered/ignored/unobserved/
    unobservable), overall and per concern
  - triage precision — judged, user-visible fires that landed
    helped-or-neutral (triage *recall* — good fires wrongly deferred — is
    unmeasurable without counterfactuals and is deliberately not faked)
  - coverage — of observable fires, how many got judged before aging out
    (validates the stage-6-rides-user-turns design)
  - valence / user_impact means among judged records
  - the capability_proposal fire stream, separated

Read-only with respect to agent state (reads autonomy.jsonl and, for the
pending count, pending_fire_outcomes.json; writes nothing).

Usage:
    python bench/autonomy_review/outcomes.py                # table, default world/agent
    python bench/autonomy_review/outcomes.py --since 2026-07-01
    python bench/autonomy_review/outcomes.py --json         # machine-readable
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

from review import _brief, _default_log, _filter, _load, _ts

M1_TARGET = 50  # roadmap M1 exit criterion: ≥50 outcome-labeled fires

JUDGED = ("helped", "neutral", "hindered", "ignored")
OUTCOMES = JUDGED + ("unobserved", "unobservable")


def _pending_count(log: Path, concern=None, since=None):
    """(count, oldest fired_at ISO) from the pending registry beside the
    log, constrained by the same --concern/--since filters as the rest
    of the analysis; (0, None) when absent/unreadable — absence just
    means nothing is awaiting judgment."""
    path = log.parent / "pending_fire_outcomes.json"
    try:
        records = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return 0, None
    except Exception as e:
        # stderr: --json mode's stdout must stay machine-parseable
        print(f"(pending registry unreadable: {e})", file=sys.stderr)
        return 0, None
    if not isinstance(records, list):
        return 0, None
    kept = []
    for r in records:
        if not isinstance(r, dict):
            continue
        if concern and r.get("concern_id") != concern:
            continue
        if since is not None:
            try:
                fired = datetime.fromisoformat(str(r.get("fired_at", "")))
            except ValueError:
                continue
            if fired.replace(tzinfo=None) < since:
                continue
        kept.append(r)
    fired = sorted(str(r.get("fired_at", "")) for r in kept
                   if r.get("fired_at"))
    return len(kept), (fired[0] if fired else None)


def _mean(vals):
    vals = [v for v in vals if isinstance(v, (int, float))]
    return round(statistics.mean(vals), 3) if vals else None


def analyze(records: list, all_records: list, log: Path,
            concern=None, since=None) -> dict:
    """`records` is the filtered view every table is computed over;
    `all_records` is the unfiltered log, used only for cross-record
    joins (an outcome's fire event may fall outside the --since window
    while the outcome itself is inside it)."""
    fire_events = [r for r in records if r.get("event") == "fire"]
    outcomes = [r for r in records if r.get("event") == "fire_outcome"]
    triages = Counter(r.get("verdict") for r in records
                      if r.get("event") == "triage")

    by_outcome = Counter(r.get("outcome") for r in outcomes)
    judged = [r for r in outcomes if r.get("outcome") in JUDGED]
    unobserved = by_outcome.get("unobserved", 0)
    observable = len(judged) + unobserved

    landed_ok = sum(1 for r in judged
                    if r.get("outcome") in ("helped", "neutral"))

    # Join on concern_id, not concern text: a fire's concern text can be
    # rewritten between dispatch and outcome (WIP/successor edits), which
    # would split one concern across two text-keyed rows.
    per_concern: dict = defaultdict(
        lambda: {"text": "", "fires": 0, "outcomes": Counter()})
    for r in fire_events:
        row = per_concern[r.get("concern_id") or "?"]
        row["fires"] += 1
        row["text"] = r.get("concern_text") or row["text"]
    for r in outcomes:
        row = per_concern[r.get("concern_id") or "?"]
        row["outcomes"][r.get("outcome")] += 1
        row["text"] = r.get("concern_text") or row["text"]

    # capability_proposal stream: fires tagged by kind, outcomes joined
    # on fire_id (outcome records don't carry kind themselves; fire_id
    # exists only on post-Phase-1 fire events, which is fine — outcomes
    # only exist for those too). The kind index comes from ALL records:
    # an outcome inside the --since window must still join to its fire
    # event even when the fire fell before the cutoff.
    cap_fires = [r for r in fire_events
                 if r.get("kind") == "capability_proposal"]
    cap_fire_ids = {r.get("fire_id") for r in all_records
                    if r.get("event") == "fire"
                    and r.get("kind") == "capability_proposal"
                    and r.get("fire_id")}
    cap_outcomes = Counter(r.get("outcome") for r in outcomes
                           if r.get("fire_id") in cap_fire_ids)

    pending, oldest_pending = _pending_count(log, concern, since)

    return {
        "log": str(log),
        "fires_total": len(fire_events),
        "outcome_records": len(outcomes),
        "m1_target": M1_TARGET,
        "by_outcome": {k: by_outcome.get(k, 0) for k in OUTCOMES},
        "judged": len(judged),
        "coverage": round(len(judged) / observable, 3) if observable else None,
        "triage_verdicts": dict(triages),
        # of judged, user-visible fires: landed helped-or-neutral
        "triage_precision": round(landed_ok / len(judged), 3) if judged else None,
        "helped_rate": round(by_outcome.get("helped", 0) / len(judged), 3)
                       if judged else None,
        "hindered_ignored_rate": round(
            (by_outcome.get("hindered", 0) + by_outcome.get("ignored", 0))
            / len(judged), 3) if judged else None,
        "valence_mean": _mean(r.get("valence") for r in judged),
        "user_impact_mean": _mean(r.get("user_impact") for r in judged),
        "latency_turns_mean": _mean(r.get("latency_turns") for r in judged),
        "pending": pending,
        "oldest_pending": oldest_pending,
        "capability_proposal": {
            "fires": len(cap_fires), "outcomes": dict(cap_outcomes)},
        "per_concern": {
            k: {"text": _brief(v["text"], 60), "fires": v["fires"],
                "outcomes": dict(v["outcomes"])}
            for k, v in sorted(per_concern.items(),
                               key=lambda kv: -kv[1]["fires"])},
    }


def render(a: dict) -> None:
    n = a["outcome_records"]
    target = a["m1_target"]
    status = "MET" if n >= target else f"{n}/{target}"
    print(f"\nFIRE OUTCOMES   {n} record(s)   (M1 target ≥{target}: {status})")
    print(f"  fires logged {a['fires_total']}   pending judgment {a['pending']}"
          + (f" (oldest {a['oldest_pending'][:10]})" if a["oldest_pending"] else ""))
    print("  " + "  ".join(f"{k}={v}" for k, v in a["by_outcome"].items()))

    if a["judged"]:
        print(f"\n  judged (user-visible, outcome known)  {a['judged']}")
        print(f"    helped_rate            {a['helped_rate']}")
        print(f"    hindered+ignored_rate  {a['hindered_ignored_rate']}")
        print(f"    triage_precision       {a['triage_precision']}   "
              f"(judged fires landing helped-or-neutral)")
        print(f"    valence mean {a['valence_mean']}   "
              f"user_impact mean {a['user_impact_mean']}   "
              f"latency {a['latency_turns_mean']} turn(s)")
        cov = a["coverage"]
        print(f"    coverage {cov}   (judged / (judged + aged-out unobserved))")
    else:
        print("\n  no judged outcomes yet — run with --autonomy live and let "
              "reflection stage 6 accumulate data (roadmap M1).")

    tv = a["triage_verdicts"]
    if tv:
        print("\nTRIAGE VERDICTS  " +
              "  ".join(f"{k}={v}" for k, v in sorted(tv.items())))
        print("  (defer↔ignored calibration needs judged data per concern)")

    cap = a["capability_proposal"]
    if cap["fires"] or cap["outcomes"]:
        print(f"\nCAPABILITY-PROPOSAL FIRES  {cap['fires']}  " +
              "  ".join(f"{k}={v}" for k, v in cap["outcomes"].items()))

    rows = [(k, v) for k, v in a["per_concern"].items()
            if v["fires"] or v["outcomes"]]
    if rows:
        print("\nPER CONCERN (by fires)")
        for cid, v in rows[:12]:
            mix = "  ".join(f"{k}={n}" for k, n in v["outcomes"].items())
            print(f"  {v['fires']:4d}  [{cid}] {v['text']}"
                  + (f"   [{mix}]" if mix else ""))
    print()


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Aggregate fire-outcome records (roadmap M1 readout).")
    ap.add_argument("--world", default="jill_chat")
    ap.add_argument("--agent", default="Jill")
    ap.add_argument("--log", type=Path, help="explicit path to autonomy.jsonl")
    ap.add_argument("--concern", help="filter to one concern_id")
    ap.add_argument("--since", help="ISO date floor, e.g. 2026-07-01")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    log = args.log or _default_log(args.world, args.agent)
    all_records = _load(log)
    records = _filter(all_records, args)
    since = None
    if args.since:
        try:
            since = datetime.fromisoformat(args.since)
        except ValueError:
            raise SystemExit(f"--since must be ISO date, got {args.since!r}")
    analysis = analyze(records, all_records, log,
                       concern=args.concern, since=since)
    if args.json:
        print(json.dumps(analysis, indent=2, ensure_ascii=False))
    else:
        print(f"# {log}")
        render(analysis)


if __name__ == "__main__":
    main()
