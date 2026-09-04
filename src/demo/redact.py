"""Withhold the demo target's identity from everything the page shows.

A LIST OF LITERAL IDENTIFIERS, WRITTEN BY THE PRACTICE. `demo.yaml` names the
strings that identify the target — the repository's name in its spellings,
the author's name and handle, the commit — and this module replaces each
wherever it appears in text that leaves the process: the banner, the report,
the findings, the history, every reply. It is substitution over a known
list, not detection: a name the practice did not list is not withheld, and
the practice reads the demo page before publishing it.

Three fixed rules ride along because the record produces them
mechanically: an absolute path under the repository or the engagement
becomes a relative one; a GitHub URL becomes a marker; a run directory's
timestamp label is dropped from the banner.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

GITHUB = re.compile(r"https?://(?:www\.)?github\.com/[^\s)\]>\"'`]*")
LINK_WITHHELD = "[repository link withheld]"


class Redactor:
    def __init__(self, display_name: str, identifiers: Iterable[str],
                 roots: Iterable[Path] = (), name_marker: Optional[str] = None) -> None:
        self.display_name = display_name
        self.marker = name_marker or display_name
        ids = sorted({i.strip() for i in identifiers if i and i.strip()},
                     key=len, reverse=True)           # longest first
        self._ids = [re.compile(re.escape(i), re.IGNORECASE) for i in ids]
        self._roots = [str(Path(r).resolve()) for r in roots]

    def text(self, s: Any) -> Any:
        if not isinstance(s, str) or not s:
            return s
        out = s
        for root in self._roots:
            out = out.replace(root + "/", "").replace(root, ".")
        out = GITHUB.sub(LINK_WITHHELD, out)
        for rx in self._ids:
            out = rx.sub(self.marker, out)
        return out

    def deep(self, obj: Any) -> Any:
        """The same substitution over every string in a JSON-like value."""
        if isinstance(obj, str):
            return self.text(obj)
        if isinstance(obj, list):
            return [self.deep(x) for x in obj]
        if isinstance(obj, dict):
            return {k: self.deep(v) for k, v in obj.items()}
        return obj

    def leaks(self, s: str) -> List[str]:
        """The identifiers still present in `s`, for a check."""
        found = [rx.pattern for rx in self._ids if rx.search(s or "")]
        if GITHUB.search(s or ""):
            found.append("github.com URL")
        return found


def banner(display_name: str, claims: int, findings: int, sources: int,
           model_label: str) -> str:
    """The demo's own banner: what the visitor is looking at, with nothing
    that identifies the target."""
    return "\n".join([
        f"  target       {display_name} — a real public repository, identity withheld",
        f"  reviewed     {sources} claim document{'s' if sources != 1 else ''}, "
        f"{claims} claims, {findings} findings, reviewed and rated",
        f"  model        {model_label}, on a hosted inference service",
        "  this page    your conversation is private to your browser and kept for a few days"])
