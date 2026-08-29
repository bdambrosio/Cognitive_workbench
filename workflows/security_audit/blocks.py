"""Self-delimiting deliverable blocks for the security audit.

A COPY OF `workflows/blocks.py`, DELIBERATELY. The security workflow is being
built alongside the claims audit and review without touching either, so it gets
its own module rather than adding names to that one's `_ALL` — which builds its
marker regexes at import, so a name added from outside would have no regex and
`opened()` would raise instead of returning False.

Only the name tuples below differ from the original. Everything else is
identical and should be reconciled, not diverged: when the three workflows are
generalised, this file is the one to delete.

WHY BLOCKS AT ALL. A turn boundary proves a turn ended. It does not prove a
document was written. The markers do, and they are what the runner counts.
"""

from __future__ import annotations

import re
from typing import Dict, Optional, Tuple

BLOCKS: Tuple[str, ...] = ("ATTACK SURFACE", "REPORT", "LIMITATIONS",
                           "GAP MAP")

# No review workflow yet. Not every workflow needs one: a review holds a
# deliverable to a standard few workflows are held to, and adding one before
# the audit itself runs would be building the check before the thing checked.
_ALL: Tuple[str, ...] = BLOCKS

# What `report.md` is assembled from, in this order. LIMITATIONS is a separate
# block and still part of the report — the client receives one document with
# those lines in it. Separate block, same document.
REPORT_BLOCKS: Tuple[str, ...] = ("REPORT", "LIMITATIONS")


def open_mark(name: str) -> str:
    return f"=== {name} ==="


def close_mark(name: str) -> str:
    return f"=== END {name} ==="


def marker_re(mark: str) -> re.Pattern:
    """A marker AT THE START OF A LINE, tolerant of whitespace between tokens.

    ANCHORED 2026-08-29, tracking `workflows/blocks.py`. The header above says
    this file must be reconciled and not diverged, and an unanchored copy would
    be exactly that. There it cost a run: prose naming three blocks it was
    about to write counted as delivering them, and the engagement ended with a
    fifteen-word report. A marker mid-sentence is commentary; one opening a
    line is a block.

    The whitespace tolerance is unchanged — the anchor binds the first `===` to
    the line start and a wrap inside the marker still matches.
    """
    return re.compile(r"(?m)^[ \t]*"
                      + r"\s+".join(re.escape(t) for t in mark.split()))


# An opener never matches inside its own closer: `=== END REPORT ===` puts
# `END` where `REPORT` would have to be, so `===\s+REPORT\s+===` cannot match.
_OPEN = {n: marker_re(open_mark(n)) for n in _ALL}
_CLOSE = {n: marker_re(close_mark(n)) for n in _ALL}


def opened(text: str, name: str) -> bool:
    """Has this block been delivered? The opener is the delivery signal."""
    return bool(_OPEN[name].search(text or ""))


def closed(text: str, name: str) -> bool:
    """Recorded, never gated. A closer says the model finished writing rather
    than ran out of room; a missing one is a truncation signal and must not by
    itself reject a run that delivered."""
    return bool(_CLOSE[name].search(text or ""))


def content(text: str, name: str,
            blocks: Tuple[str, ...] = BLOCKS) -> Optional[str]:
    """The block's body, or None if its opener is absent.

    Runs from the opener to its closer, or — when the closer is missing — to
    the next block's opener, or to the end of the text. Truncation loses the
    closer, and losing the body as well would turn one defect into two.
    """
    text = text or ""
    hit = _OPEN[name].search(text)
    if not hit:
        return None
    body = text[hit.end():]
    end = _CLOSE[name].search(body)
    if end:
        return body[:end.start()].strip()
    nxt = [m.start() for other in blocks if other != name
           for m in [_OPEN[other].search(body)] if m]
    return (body[:min(nxt)] if nxt else body).strip()


def span(text: str, name: str,
         blocks: Tuple[str, ...] = BLOCKS) -> Optional[str]:
    """The block INCLUDING its markers, or None if the opener is absent.

    `report.md` is assembled from spans rather than stripped bodies so the file
    is a faithful copy of what the model emitted, and so every downstream
    check that looks for `=== LIMITATIONS ===` keeps finding it. Stripping the
    markers on the way out would have made the limitations criterion fail on
    reports that satisfied it.
    """
    body = content(text, name, blocks)
    if body is None:
        return None
    tail = f"\n{close_mark(name)}" if closed(text, name) else ""
    return f"{open_mark(name)}\n{body}{tail}"


def present(text: str,
            blocks: Tuple[str, ...] = BLOCKS) -> Dict[str, bool]:
    return {n: opened(text, n) for n in blocks}


def missing(delivered: Dict[str, bool],
            blocks: Tuple[str, ...] = BLOCKS) -> List[str]:
    """Undelivered blocks, in emission order."""
    return [n for n in blocks if not delivered.get(n)]


def rejection(name: str, section: str = "METHOD §15") -> str:
    """What the runner says when a block has not arrived.

    STATES WHAT WAS OBSERVED, NEVER WHAT IS INFERRED. "No report received" is
    an inference and can be false — a model that wrote a good report and forgot
    the marker would be told something untrue. The runner can check whether a
    token is present; it cannot check whether a document was written. The same
    discipline removed "The report is received." from the Gap Map request on
    2026-08-27, where it was false.
    """
    return (f"That turn carried no `{open_mark(name)}` line, so the "
            f"{name.lower()} block has not been delivered. Its contents and "
            f"order are specified in {section}. Continue the engagement and "
            f"emit the block, opening with the marker on its own line.")
