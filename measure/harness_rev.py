#!/usr/bin/env python3
"""Join a turn to the harness revision that was live when it ran.

This is what makes harness-change regression measurable at all. The v2
suite only ever varied the backend model, so a harness edit could not be
seen. Every metric here is tagged (model x harness revision); this supplies
the second half.

Retrospectively the live trace is a natural experiment: jill_chat spans
2026-05-03 to 2026-08-22 against 121 commits touching src/ since July. Ask
whether a metric moved at a change already known to have had an effect.
"""

from __future__ import annotations

import bisect
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Tuple

REPO = Path(__file__).resolve().parent.parent


def harness_commits(paths: str = "src/") -> List[Tuple[datetime, str, str]]:
    """(committed_at, sha, subject) for commits touching `paths`, oldest
    first. Uses committer date, not author date — a rebased or
    cherry-picked commit became live when it landed, not when it was
    written."""
    out = subprocess.run(
        ["git", "log", "--reverse", "--format=%ct%x09%h%x09%s", "--", paths],
        cwd=REPO, capture_output=True, text=True, check=True,
    ).stdout
    rows: List[Tuple[datetime, str, str]] = []
    for line in out.splitlines():
        parts = line.split("\t", 2)
        if len(parts) < 3:
            continue
        try:
            when = datetime.fromtimestamp(int(parts[0]), tz=timezone.utc)
        except ValueError:
            continue
        rows.append((when, parts[1], parts[2]))
    return rows


class RevisionIndex:
    """Which harness commit was live at a given instant."""

    def __init__(self, paths: str = "src/") -> None:
        self.rows = harness_commits(paths)
        self._keys = [r[0] for r in self.rows]

    def at(self, ts: Optional[datetime]) -> Optional[Tuple[str, str]]:
        """(sha, subject) of the newest commit at or before `ts`."""
        if ts is None or not self.rows:
            return None
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)
        i = bisect.bisect_right(self._keys, ts) - 1
        if i < 0:
            return None
        _, sha, subject = self.rows[i]
        return sha, subject

    def span(self, lo: Optional[datetime], hi: Optional[datetime]
             ) -> List[Tuple[datetime, str, str]]:
        """Commits that landed inside a window — the candidate causes for
        any metric movement across it."""
        if lo is None or hi is None:
            return []
        if lo.tzinfo is None:
            lo = lo.replace(tzinfo=timezone.utc)
        if hi.tzinfo is None:
            hi = hi.replace(tzinfo=timezone.utc)
        return [r for r in self.rows if lo <= r[0] <= hi]
