#!/usr/bin/env python3
"""autonomy_review — observe and score Jill's autonomous concern fires.

Reads a world/agent's `autonomy.jsonl` (written by the concerns layer) and
renders what autonomy actually did: which concerns fired, the triage verdicts
and reasons, outcomes, and timing. The point is to make autonomy *visible and
judgeable* — the missing piece before turning the autonomy dial up.

Read-only with respect to agent state. The only thing it writes is an opt-in
score sidecar (`autonomy_scores.jsonl`) next to the log — never the resource
manager, never the concern notes.

Usage:
    python bench/autonomy_review/review.py                 # summary, default world/agent
    python bench/autonomy_review/review.py --list          # chronological digest
    python bench/autonomy_review/review.py --world jill_chat --agent Jill
    python bench/autonomy_review/review.py --log <path/to/autonomy.jsonl>
    python bench/autonomy_review/review.py --concern Note_812   # filter one concern
    python bench/autonomy_review/review.py --since 2026-06-01
    python bench/autonomy_review/review.py --score          # interactively rate fires
    python bench/autonomy_review/review.py --scores         # summarize ratings
"""
from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]


def _default_log(world: str, agent: str) -> Path:
    return _REPO / "scenarios" / world / agent / "memory" / "autonomy.jsonl"


def _load(path: Path) -> list:
    if not path.is_file():
        raise SystemExit(f"no autonomy log at {path}")
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError as e:
            print(f"  (skipped malformed line: {e})")
    return out


def _ts(rec: dict):
    raw = rec.get("ts") or rec.get("started_at") or ""
    try:
        return datetime.fromisoformat(raw)
    except (ValueError, TypeError):
        return None


def _brief(s: str, n: int = 100) -> str:
    s = " ".join((s or "").split())
    return s if len(s) <= n else s[: n - 1] + "…"


def _filter(records: list, args) -> list:
    out = records
    if args.concern:
        out = [r for r in out if r.get("concern_id") == args.concern]
    if args.since:
        try:
            cutoff = datetime.fromisoformat(args.since)
        except ValueError:
            raise SystemExit(f"--since must be ISO date, got {args.since!r}")
        kept = []
        for r in out:
            t = _ts(r)
            # fromisoformat with tz vs naive: compare on date if needed
            if t is None:
                continue
            if t.replace(tzinfo=None) >= cutoff:
                kept.append(r)
        out = kept
    return out


# ── summary ────────────────────────────────────────────────────────────

def summarize(records: list) -> None:
    by_event = Counter(r.get("event") for r in records)
    times = [t for t in (_ts(r) for r in records) if t is not None]
    span = ""
    if times:
        span = f"{min(times):%Y-%m-%d} → {max(times):%Y-%m-%d}"
    print(f"\n{len(records)} events   {span}")
    print(f"  fire={by_event.get('fire',0)}  "
          f"defer={by_event.get('deferred',0)}  "
          f"triage={by_event.get('triage',0)}\n")

    fires = [r for r in records if r.get("event") == "fire"]
    if fires:
        days = {(_ts(r) or datetime.min).date() for r in fires}
        per_day = len(fires) / max(1, len(days))
        durs = [r["duration_s"] for r in fires if isinstance(r.get("duration_s"), (int, float))]
        term = Counter(r.get("terminated") for r in fires)
        exits = Counter(r.get("react_exit_reason") for r in fires)
        print("FIRES")
        print(f"  {len(fires)} over {len(days)} day(s)  (~{per_day:.1f}/day)")
        if durs:
            print(f"  duration  mean {statistics.mean(durs):.1f}s  "
                  f"median {statistics.median(durs):.1f}s  max {max(durs):.1f}s")
        print(f"  terminated  " + "  ".join(f"{k}={v}" for k, v in term.most_common()))
        print(f"  react_exit  " + "  ".join(f"{k}={v}" for k, v in exits.most_common()))
        top = Counter(_brief(r.get("concern_text", ""), 60) for r in fires)
        print("  by concern:")
        for txt, n in top.most_common(10):
            print(f"    {n:4d}  {txt}")

    triages = [r for r in records if r.get("event") == "triage"]
    if triages:
        verd = Counter(r.get("verdict") for r in triages)
        print("\nTRIAGE")
        print(f"  {len(triages)} judgements  " +
              "  ".join(f"{k}={v}" for k, v in verd.most_common()))

    defers = [r for r in records if r.get("event") == "deferred"]
    if defers:
        print(f"\nDEFERRED (cap overflow)  {len(defers)}")
    print()


# ── chronological digest ────────────────────────────────────────────────

def digest(records: list) -> None:
    for r in sorted(records, key=lambda r: _ts(r) or datetime.min):
        t = _ts(r)
        stamp = f"{t:%Y-%m-%d %H:%M}" if t else "?"
        ev = r.get("event")
        cid = r.get("concern_id", "?")
        if ev == "fire":
            tail = (f"{r.get('terminated','?')}/{r.get('react_exit_reason','?')} "
                    f"{r.get('duration_s','?')}s")
            print(f"{stamp}  FIRE   [{cid}] {_brief(r.get('concern_text',''),60)}  ({tail})")
            resp = _brief(r.get("response_brief", ""), 120)
            if resp:
                print(f"                  → {resp}")
        elif ev == "triage":
            print(f"{stamp}  TRIAGE [{cid}] {r.get('verdict','?').upper()}  "
                  f"— {_brief(r.get('reason',''),90)}")
        elif ev == "deferred":
            print(f"{stamp}  DEFER  [{cid}] {_brief(r.get('concern_text',''),60)}")
    print()


# ── scoring ──────────────────────────────────────────────────────────────

def _score_path(log: Path) -> Path:
    return log.parent / "autonomy_scores.jsonl"


def _load_scores(path: Path) -> dict:
    scores = {}
    if path.is_file():
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line:
                s = json.loads(line)
                scores[(s.get("concern_id"), s.get("fire_ts"))] = s
    return scores


def score(records: list, log: Path) -> None:
    sidecar = _score_path(log)
    done = _load_scores(sidecar)
    fires = [r for r in records if r.get("event") == "fire"]
    todo = [r for r in fires if (r.get("concern_id"), r.get("ts")) not in done]
    if not todo:
        print(f"all {len(fires)} fires already scored ({sidecar.name})")
        return
    print(f"scoring {len(todo)} unrated fire(s). "
          f"[w]arranted [u]nwarranted [m]aybe [s]kip [q]uit\n")
    with open(sidecar, "a", encoding="utf-8") as f:
        for r in sorted(todo, key=lambda r: _ts(r) or datetime.min):
            t = _ts(r)
            print(f"── {t:%Y-%m-%d %H:%M}  [{r.get('concern_id')}]")
            print(f"   concern: {_brief(r.get('concern_text',''),100)}")
            print(f"   did:     {_brief(r.get('response_brief',''),200) or '(no response captured)'}")
            ans = input("   verdict [w/u/m/s/q]: ").strip().lower()
            if ans == "q":
                break
            if ans == "s" or ans not in ("w", "u", "m"):
                print()
                continue
            note = input("   note (optional): ").strip()
            rec = {"concern_id": r.get("concern_id"), "fire_ts": r.get("ts"),
                   "verdict": {"w": "warranted", "u": "unwarranted",
                               "m": "maybe"}[ans],
                   "note": note,
                   "scored_at": datetime.now().isoformat()}
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            f.flush()
            print()
    print(f"saved → {sidecar}")


def show_scores(log: Path) -> None:
    sidecar = _score_path(log)
    scores = _load_scores(sidecar)
    if not scores:
        print(f"no scores yet ({sidecar}). Run with --score.")
        return
    verd = Counter(s.get("verdict") for s in scores.values())
    print(f"\n{len(scores)} scored fires  " +
          "  ".join(f"{k}={v}" for k, v in verd.most_common()))
    bad = [s for s in scores.values() if s.get("verdict") == "unwarranted"]
    if bad:
        print("\nunwarranted fires (the ones to tune away):")
        by_concern = defaultdict(int)
        for s in bad:
            by_concern[s.get("concern_id")] += 1
        for cid, n in sorted(by_concern.items(), key=lambda x: -x[1]):
            print(f"  {n:3d}  {cid}")
    print()


def main() -> None:
    ap = argparse.ArgumentParser(description="Review/score Jill's autonomous fires.")
    ap.add_argument("--world", default="jill_chat")
    ap.add_argument("--agent", default="Jill")
    ap.add_argument("--log", type=Path, help="explicit path to autonomy.jsonl")
    ap.add_argument("--list", action="store_true", help="chronological digest")
    ap.add_argument("--concern", help="filter to one concern_id")
    ap.add_argument("--since", help="ISO date floor, e.g. 2026-06-01")
    ap.add_argument("--score", action="store_true", help="interactively rate fires")
    ap.add_argument("--scores", action="store_true", help="summarize ratings")
    args = ap.parse_args()

    log = args.log or _default_log(args.world, args.agent)
    records = _filter(_load(log), args)
    print(f"# {log}")

    if args.scores:
        show_scores(log)
    elif args.score:
        score(records, log)
    elif args.list:
        digest(records)
    else:
        summarize(records)


if __name__ == "__main__":
    main()
