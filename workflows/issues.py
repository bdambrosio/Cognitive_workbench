#!/usr/bin/env python3
"""One place a run records what a person still has to look at.

WHY A SINGLE LOG. The record was spread across five artifacts — run_meta.json,
WARNING lines in run.log, review/conformance.json, the reviewer's exceptions,
and delivery/editor_notes.md — and only the last was aimed at a person. Nobody
reads run_meta. A defect noticed by the audit reached a human only if somebody
happened to open the right file.

APPEND-ONLY, WRITTEN WHERE IT IS NOTICED. Each stage appends at the moment it
finds something, so a later stage cannot overwrite an earlier stage's finding
and no stage has to parse another's outputs to learn what happened. The log
survives even when a later stage never runs, which matters because an audit can
be delivered days after it was produced.

NOT A LOG OF WHAT HAPPENED. Only what somebody must decide or repair. A run
that worked writes nothing here.
"""
from __future__ import annotations

import datetime
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

FILENAME = "issues.jsonl"

# Severity is about what it costs to ship the run as it stands, not about how
# interesting the finding is.
#   blocking  a client must not see this — a wrong assurance figure, a report
#             whose citations do not resolve
#   check     a person decides; it may be fine — a flagged citation the index
#             could have misread, an observation with nowhere to go
#   note      recorded so it is not lost; no action expected
SEVERITIES = ("blocking", "check", "note")


def note(run: Path, stage: str, code: str, text: str,
         severity: str = "check", **fields: Any) -> None:
    """Append one issue. Never raises: recording a problem must not cause one."""
    if severity not in SEVERITIES:
        severity = "check"
    row: Dict[str, Any] = {
        "ts": datetime.datetime.now(
            datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ"),
        "stage": stage, "code": code, "severity": severity, "text": text,
    }
    row.update(fields)
    try:
        with (Path(run) / FILENAME).open("a") as fh:
            fh.write(json.dumps(row) + "\n")
    except OSError:
        pass


def read(run: Path) -> List[Dict[str, Any]]:
    """Every issue recorded for this run, in the order it was noticed."""
    p = Path(run) / FILENAME
    if not p.is_file():
        return []
    out = []
    for line in p.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


def render(run: Path) -> str:
    """The log as a person reads it, worst first."""
    rows = read(run)
    if not rows:
        return "No issues were recorded for this run."
    order = {s: i for i, s in enumerate(SEVERITIES)}
    rows.sort(key=lambda r: order.get(r.get("severity"), 9))
    out: List[str] = []
    for sev in SEVERITIES:
        group = [r for r in rows if r.get("severity") == sev]
        if not group:
            continue
        label = {"blocking": "Must be resolved before this goes to a client",
                 "check": "A person should decide",
                 "note": "Recorded, no action expected"}[sev]
        out += [f"**{label}**", ""]
        for r in group:
            out.append(f"- `{r.get('stage')}` · {r.get('text')}")
        out.append("")
    return "\n".join(out).rstrip() + "\n"
