#!/usr/bin/env python3
"""Process-shape metrics — what the run DID, in what order.

Ported from the retired `bench/venture/arc.py`, generalised from one
fixture to any (world, agent). Phase comes from which tools were called,
never from reading the turn's prose: a turn that called search-web went and
got evidence whatever it said it was doing.

The metrics that earned their place, and the failure each detects:

  re_authoring          the runaway shape. The 2026-08-20 exchange ran 130
                        unattended turns re-writing the same document.
  gather_before_author  did evidence precede the writing, or did it write
                        first and look things up after (or never)?
  toolless vs silent    kept SEPARATE, and neither is called a failure
                        here. A silent turn is the deliberate
                        don't-acknowledge idiom. A toolless reply is a
                        stall only where the task required work — in
                        conversation it is just answering. Naming it
                        `stalls` scored 40% of live jill_chat turns as
                        failures, which is the metric-punishes-correct-
                        behaviour bug six times over in the retired suite.
  exit_reasons          yield / respond / max_iters. A max_iters turn burned
                        its whole budget without answering.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from measure.trace import Turn


@dataclass
class ProcessReport:
    turns: int = 0
    shape: List[str] = field(default_factory=list)
    transitions: int = 0
    phases: Counter = field(default_factory=Counter)
    exit_reasons: Counter = field(default_factory=Counter)
    tools: Counter = field(default_factory=Counter)
    toolless_replies: int = 0
    silent: int = 0
    autonomous: int = 0
    gather_before_author: Optional[int] = None
    re_authoring: Optional[int] = None
    total_iterations: int = 0
    produced_chars: int = 0
    minutes: Optional[float] = None


def build(turns: List[Turn]) -> ProcessReport:
    rep = ProcessReport(turns=len(turns))
    if not turns:
        return rep

    phases = []
    for t in turns:
        p = t.phase
        phases.append(p)
        rep.phases[p] += 1
        rep.exit_reasons[str(t.exit_reason)] += 1
        rep.tools.update(t.tools)
        rep.toolless_replies += int(t.is_toolless_reply)
        rep.silent += int(t.is_silent)
        rep.autonomous += int(t.autonomous)
        rep.total_iterations += t.iterations
        rep.produced_chars += t.produced_chars

    # Collapse runs of the same phase: the SHAPE of the arc, not its length.
    shape: List[str] = []
    for p in phases:
        if not shape or shape[-1] != p:
            shape.append(p)
    rep.shape = shape
    rep.transitions = max(0, len(shape) - 1)

    first_author = next((i for i, p in enumerate(phases) if p == "author"),
                        None)
    if first_author is not None:
        rep.gather_before_author = sum(
            1 for p in phases[:first_author] if p == "gather")
        # Re-authoring after a first artifact exists is the runaway shape.
        rep.re_authoring = sum(
            1 for p in phases[first_author + 1:] if p == "author")

    stamped = [t.ts for t in turns if t.ts is not None]
    if len(stamped) >= 2:
        rep.minutes = round(
            (max(stamped) - min(stamped)).total_seconds() / 60.0, 1)

    return rep


def render(rep: ProcessReport, label: str, shape_cap: int = 24) -> str:
    out = [f"\n=== process — {label} ==="]
    out.append(f"  turns             {rep.turns}  "
               f"({rep.autonomous} autonomous)"
               + (f"   span {rep.minutes} min" if rep.minutes else ""))
    shape = rep.shape[:shape_cap]
    tail = " …" if len(rep.shape) > shape_cap else ""
    out.append(f"  arc shape         {' -> '.join(shape)}{tail}")
    out.append(f"  transitions       {rep.transitions}")
    out.append(f"  phases            "
               + "  ".join(f"{k}={v}" for k, v in rep.phases.most_common()))
    out.append(f"  exits             "
               + "  ".join(f"{k}={v}" for k, v in rep.exit_reasons.most_common()))
    if rep.gather_before_author is not None:
        out.append(f"  evidence first    {rep.gather_before_author} gather "
                   f"turns before the first authoring turn")
        out.append(f"  re-authoring      {rep.re_authoring}  "
                   f"(authoring turns after the first)")
    out.append(f"  toolless replies  {rep.toolless_replies}      "
               f"silent {rep.silent}")
    out.append("                    (neither is a failure by itself — a "
               "toolless reply is a stall\n                     only where "
               "the task required work; silent is the don't-acknowledge idiom)")
    out.append(f"  iterations        {rep.total_iterations} total")
    if rep.tools:
        top = "  ".join(f"{k}={v}" for k, v in rep.tools.most_common(8))
        out.append(f"  tools             {top}")
    return "\n".join(out)
