#!/usr/bin/env python3
"""venture/arc — reconstruct the reasoning arc of one venture-memo run.

The object of study is the ARC, not the ideas: how the run decomposed the
brief, when it gathered evidence versus reasoned over it versus wrote, whether
it revised, and whether it stopped. So this emits a per-turn timeline and the
shape statistics over it, NOT a quality score. What a judge is later asked to
grade sits in `judge_material`, kept separate so the mechanical numbers cannot
quietly become opinions.

Read-only. Reads a run's reasoning_trace.jsonl; writes nothing to agent state.

    python3 bench/venture/arc.py --world venture_solo --agent Jill
    python3 bench/venture/arc.py --world venture_solo --agent Jill --json out.json
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO = Path(__file__).resolve().parent.parent.parent

# Tools grouped by what calling one MEANS for the arc. Grouping is by tool
# identity, not by reading the text — a turn that called search-web went and
# got evidence whatever it said it was doing.
GATHER = {"search-web", "tavily", "fetch-text", "browse", "semantic-scholar",
          "obsidian", "extract", "extract-references", "check-x-feed",
          "stock-price", "get_financial_statements"}
INSPECT = {"inspect", "exec-script", "claude-code", "security", "text-find"}
REASON = {"process_text", "synthesize", "recall", "remember", "justify"}
COORD = {"agent-say", "agent-activity"}

ACTION_RE = re.compile(r'"tool"\s*:\s*"([^"]+)"')

# Terminal ReAct actions. They appear in the action stream exactly where a
# tool name does, and counting them as tools makes every turn that merely
# answered look like it went and did something — which inverted the
# evidence-before-authoring number on the first run of this script.
ACTIONS = {"respond", "yield"}


def norm(name: str) -> str:
    """`fetch_text` and `fetch-text` are one tool; the trace holds both, as it
    does for search-web/search and semantic-scholar/semantic_scholar."""
    n = str(name).strip().lower().replace("_", "-")
    return {"search": "search-web", "fetch-text": "fetch-text"}.get(n, n)


def read_trace(world: str, agent: str) -> List[Dict[str, Any]]:
    p = REPO / "scenarios" / world / agent / "memory" / "reasoning_trace.jsonl"
    if not p.exists():
        raise SystemExit(f"no trace at {p}")
    rows = []
    for line in p.open():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue          # truncated tail row; the run may still be live
    return rows


def tools_of(row: Dict[str, Any]) -> List[str]:
    """Ordered tool calls for a turn, from tool_meta when present and the
    working log otherwise — tool_meta is authoritative but only records steps
    that produced an observation."""
    meta = row.get("tool_meta") or {}
    if isinstance(meta, dict) and meta:
        out = []
        for k in sorted(meta, key=lambda s: (len(s), s)):
            t = (meta[k] or {}).get("tool")
            if t:
                out.append(str(t))
        if out:
            return [norm(x) for x in out if norm(x) not in ACTIONS]
    return [norm(x) for x in ACTION_RE.findall(str(row.get("working_log") or ""))
            if norm(x) not in ACTIONS]


def phase_of(tools: List[str], produced: int) -> str:
    """What this turn mostly DID. Deliberately coarse: the interesting signal
    is the sequence of phases, and a finer taxonomy would need to interpret
    the text rather than observe the actions."""
    s = set(tools)
    if s & GATHER:
        return "gather"
    if s & INSPECT:
        return "inspect"
    if s & COORD:
        return "coordinate"
    if s & REASON:
        return "reason"
    return "author" if produced >= 400 else "idle"


def parse_ts(v: Any) -> Optional[datetime]:
    try:
        return datetime.fromisoformat(str(v))
    except (TypeError, ValueError):
        return None


def build(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    turns, t0 = [], None
    for r in rows:
        tools = tools_of(r)
        produced = len(str(r.get("raw_response") or ""))
        ts = parse_ts(r.get("ts") or r.get("timestamp"))
        t0 = t0 or ts
        turns.append({
            "seq": r.get("turn_seq"),
            "source": r.get("source"),
            "autonomous": bool(r.get("autonomous")),
            "exit_reason": r.get("exit_reason"),
            "tools": tools,
            "n_tools": len(tools),
            "produced_chars": produced,
            "phase": phase_of(tools, produced),
            "minutes": (round((ts - t0).total_seconds() / 60, 1)
                        if ts and t0 else None),
        })

    stamped = [t["minutes"] for t in turns if t["minutes"] is not None]
    phases = [t["phase"] for t in turns]
    # Collapse runs of the same phase: the SHAPE of the arc, not its length.
    shape, last = [], None
    for p in phases:
        if p != last:
            shape.append(p)
            last = p

    tool_counts = Counter(t for turn in turns for t in turn["tools"])
    exits = Counter(str(t["exit_reason"]) for t in turns)
    first_author = next((i for i, t in enumerate(turns)
                         if t["phase"] == "author"), None)
    gathered_before_authoring = sum(
        1 for t in turns[:first_author] if t["phase"] == "gather"
    ) if first_author is not None else 0
    authoring_after_first = sum(
        1 for t in turns[first_author + 1:] if t["phase"] == "author"
    ) if first_author is not None else 0

    return {
        "turns": turns,
        "summary": {
            "n_turns": len(turns),
            "wall_clock_min": max(stamped) if stamped else None,
            "timestamp_coverage": f"{len(stamped)}/{len(turns)}",
            "phase_histogram": dict(Counter(phases)),
            "arc_shape": " → ".join(shape),
            "phase_transitions": len(shape),
            "tool_calls": dict(tool_counts.most_common()),
            "exit_reasons": dict(exits),
            "yields": exits.get("yield", 0),
            "autonomous_turns": sum(1 for t in turns if t["autonomous"]),
            # Did evidence precede the writing, or did it write first and
            # look things up afterwards (or never)?
            "gather_turns_before_first_authoring": gathered_before_authoring,
            # Re-authoring after a first artifact exists is what the product
            # spec's no-reopen rule forbids. High here = the 2026-08-20 shape.
            "authoring_turns_after_first": authoring_after_first,
        },
        # For the judge pass; kept out of the numbers above on purpose.
        "judge_material": [
            {"seq": t["seq"], "phase": t["phase"],
             "text": str(r.get("raw_response") or "")[:1500]}
            for t, r in zip(turns, rows) if t["produced_chars"] > 200
        ],
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="venture_solo")
    ap.add_argument("--agent", default="Jill")
    ap.add_argument("--json", help="also write the full structure here")
    a = ap.parse_args()

    arc = build(read_trace(a.world, a.agent))
    s = arc["summary"]
    print(f"=== {a.agent} @ {a.world}: {s['n_turns']} turns, "
          f"{s['wall_clock_min']} min "
          f"(timestamps on {s['timestamp_coverage']} turns)")
    shape = s["arc_shape"].split(" → ")
    shown = (" → ".join(shape[:14]) + f"  …({len(shape) - 28} more)…  "
             + " → ".join(shape[-14:])) if len(shape) > 30 else s["arc_shape"]
    print(f"shape      : {shown}")
    print(f"transitions: {s['phase_transitions']}")
    print(f"phases     : {s['phase_histogram']}")
    print(f"exits      : {s['exit_reasons']}  (yields={s['yields']})")
    print(f"tools      : {s['tool_calls']}")
    print(f"evidence   : {s['gather_turns_before_first_authoring']} gather "
          f"turns before the first authoring turn")
    print(f"re-author  : {s['authoring_turns_after_first']} authoring turns "
          f"after the first (spec forbids reopening an accepted section)")
    print("\nseq  min   phase       exit        tools")
    for t in arc["turns"]:
        print(f"{str(t['seq']):>4} {str(t['minutes']):>5}  {t['phase']:<11}"
              f" {str(t['exit_reason']):<11} {','.join(t['tools'])[:60]}")
    if a.json:
        Path(a.json).write_text(json.dumps(arc, indent=1))
        print(f"\nwrote {a.json}")


if __name__ == "__main__":
    main()
