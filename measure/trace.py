#!/usr/bin/env python3
"""Turn-level view of a run's trace. The substrate every metric reads.

Nothing here scores anything. It loads `reasoning_trace.jsonl` for one
(world, agent), normalises the parts every metric needs, and hands back a
list of turns.

TWO TRAPS THIS FILE EXISTS TO ABSORB, both found on live data 2026-08-22:

1. `turn_seq` IS NOT UNIQUE. jill_chat/Jill holds 3,009 trace rows carrying
   921 distinct turn_seq values, one of them repeating 235 times. Anything
   keying a join on turn_seq alone silently collapses turns. Within the
   trace, `ts` is the identity. Joining to `claims.jsonl` needs BOTH, because
   claims carry the GRADING time rather than the turn time — see
   `measure.provenance.join_turn`.

2. Terminal ReAct actions (`respond`, `yield`) appear in the action stream
   exactly where a tool name does. Counting them as tools makes a turn that
   merely answered look like it went and gathered evidence — which inverted
   the evidence-before-authoring number the first time this was written
   (see the retired bench/venture/arc.py).
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional

REPO = Path(__file__).resolve().parent.parent

# Tools grouped by what calling one MEANS for the arc. Grouped by tool
# identity — an identifier match, not a reading of the turn's prose. A turn
# that called search-web went and got evidence whatever it said it was doing.
GATHER = {"search-web", "tavily", "fetch-text", "browse", "semantic-scholar",
          "obsidian", "extract", "extract-references", "check-x-feed",
          "stock-price", "get-financial-statements", "doc-read"}
INSPECT = {"inspect", "exec-script", "claude-code", "security", "text-find",
           "system-info"}
REASON = {"process-text", "synthesize", "recall", "remember", "justify",
          "assess", "filter-semantic"}
COORD = {"agent-say", "agent-activity"}

# Terminal actions, never tools. See trap 2 above.
ACTIONS = {"respond", "yield"}

_ACTION_RE = re.compile(r'"tool"\s*:\s*"([^"]+)"')
_ITER_RE = re.compile(r'--- iter (\d+) ---')


def norm_tool(name: str) -> str:
    """`fetch_text` and `fetch-text` are one tool; the trace holds both, as
    it does for search-web/search and semantic-scholar/semantic_scholar."""
    n = str(name).strip().lower().replace("_", "-")
    return {"search": "search-web"}.get(n, n)


def parse_ts(v: Any) -> Optional[datetime]:
    """Always tz-aware. Trace timestamps carry an offset but CLI bounds like
    `--since 2026-08-14` do not, and comparing the two raises. Naive input is
    read as UTC, which is what the trace writes."""
    try:
        dt = datetime.fromisoformat(str(v))
    except (TypeError, ValueError):
        return None
    return dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)


@dataclass
class Turn:
    """One turn, normalised. `ts` is the identity; `seq` is advisory only."""
    ts: Optional[datetime]
    seq: Optional[int]
    source: Optional[str]
    autonomous: bool
    exit_reason: Optional[str]
    tools: List[str] = field(default_factory=list)
    iterations: int = 0
    produced_chars: int = 0
    working_log: str = ""
    raw: Dict[str, Any] = field(default_factory=dict)

    @property
    def phase(self) -> str:
        """What this turn mostly DID. Deliberately coarse: the signal is the
        sequence of phases, and a finer taxonomy would have to interpret text
        rather than observe actions."""
        s = set(self.tools)
        if s & GATHER:
            return "gather"
        if s & INSPECT:
            return "inspect"
        if s & COORD:
            return "coordinate"
        if s & REASON:
            return "reason"
        return "author" if self.produced_chars >= 400 else "idle"

    @property
    def is_toolless_reply(self) -> bool:
        """Ended at iteration 1 with content, having called no tool.

        NOT named `is_stall`, deliberately. On a research fixture this is
        the stall — claiming progress without doing work, which is what
        coord_search built it to catch. In conversation it is just answering
        a question, and 175 of jill_chat's 443 turns in one window are this.
        Whether it is a failure depends on whether the task required work,
        so this reports the fact and leaves the judgement to the fixture."""
        return (not self.tools and self.iterations <= 1
                and self.produced_chars > 0)

    @property
    def is_silent(self) -> bool:
        """Ended at iteration 1 with zero content — the deliberate
        don't-acknowledge idiom. Scored apart from toolless replies so correct
        behaviour is never counted as a failure."""
        return (not self.tools and self.iterations <= 1
                and self.produced_chars == 0)


def tools_of(row: Dict[str, Any]) -> List[str]:
    """Ordered tool calls, from `tool_meta` when present and the working log
    otherwise. tool_meta is authoritative but only records steps that
    produced an observation, so the log is the fallback, not the reverse."""
    meta = row.get("tool_meta") or {}
    if isinstance(meta, dict) and meta:
        out = []
        for k in sorted(meta, key=lambda s: (len(s), s)):
            t = (meta[k] or {}).get("tool")
            if t:
                out.append(norm_tool(t))
        out = [t for t in out if t not in ACTIONS]
        if out:
            return out
    return [t for t in
            (norm_tool(x) for x in
             _ACTION_RE.findall(str(row.get("working_log") or "")))
            if t not in ACTIONS]


def trace_path(world: str, agent: str) -> Path:
    return REPO / "scenarios" / world / agent / "memory" / "reasoning_trace.jsonl"


def load_turns(world: str, agent: str) -> List[Turn]:
    """All turns for one (world, agent), oldest first, from the live world."""
    return load_trace(trace_path(world, agent))


def load_trace(p: Path) -> List[Turn]:
    """All turns in one trace file, oldest first.

    Split out from load_turns so a caller can read an ARCHIVED trace — the copy
    a run leaves in its working record — rather than only the live world. A run
    that cannot be read after its world is discarded is not an archive, and
    METHOD §14 says the world is discarded.

    A truncated tail row is skipped rather than fatal — the run may still be
    live and writing.
    """
    if not p.exists():
        raise SystemExit(f"no trace at {p}")

    turns: List[Turn] = []
    for line in p.open(errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        wl = str(row.get("working_log") or "")
        # `iters` is authoritative where the record carries it; the log regex
        # is the fallback for older rows written before that field existed.
        iters = row.get("iters")
        if not isinstance(iters, int):
            seen = [int(m) for m in _ITER_RE.findall(wl)]
            iters = max(seen) if seen else 0
        turns.append(Turn(
            ts=parse_ts(row.get("ts") or row.get("timestamp")),
            seq=row.get("turn_seq"),
            source=row.get("source"),
            autonomous=bool(row.get("autonomous")),
            exit_reason=row.get("exit_reason"),
            tools=tools_of(row),
            iterations=iters,
            produced_chars=len(str(row.get("raw_response") or "")),
            working_log=wl,
            raw=row,
        ))
    turns.sort(key=lambda t: (t.ts is None, t.ts))
    return turns


def iter_agents(world: str) -> Iterator[str]:
    """Agent names in a world that have a trace."""
    root = REPO / "scenarios" / world
    if not root.is_dir():
        return
    for d in sorted(root.iterdir()):
        if (d / "memory" / "reasoning_trace.jsonl").exists():
            yield d.name


def window(turns: List[Turn], since: Optional[str] = None,
           until: Optional[str] = None) -> List[Turn]:
    """Restrict to a time window. Bounds are ISO strings; either may be None."""
    lo, hi = parse_ts(since) if since else None, parse_ts(until) if until else None
    out = []
    for t in turns:
        if t.ts is None:
            continue
        if lo and t.ts < lo:
            continue
        if hi and t.ts > hi:
            continue
        out.append(t)
    return out
