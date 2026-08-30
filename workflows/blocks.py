"""The delivery-block vocabulary shared by the workflow runners and the scorer.

A workflow's deliverables are emitted as FLAT, SELF-DELIMITING BLOCKS: each
opens with `=== NAME ===` on its own line and closes with `=== END NAME ===`.
Blocks never nest. METHOD §16 is the agent-facing statement of the same thing.

WHY BLOCKS AND NOT TURN BOUNDARIES. Until 2026-08-27 the runner read a turn
that did not `yield` as the engagement being finished, and took whatever that
turn contained as the report. `b2_glm_2` ended a turn with its claim
enumeration and the sentence "I'll work the priority order straight through
now"; the runner had nothing it could test, filed the enumeration as
`report.md`, and asked for a Gap Map about it. **A turn boundary proves a turn
ended. It does not prove a document was written.** A block says what it is.

WHY THIS FILE EXISTS RATHER THAN THREE COPIES. `=== GAP MAP ===` was declared
twice — in `claims_audit/runner.py` for its salvage path and in `score.py` for
parsing — and specified to the agent in neither METHOD nor the brief. Code
depending on a token nobody was told to emit is how a contract resolves against
a referent that was never declared.

THE MARKER REGEXES ARE WHITESPACE-TOLERANT, and that is not cosmetic. On
2026-08-25 a model emitted a complete limitations statement under
`===\\nLIMITATIONS ===` — a line wrap inside the literal — and exact matching
reported the requirement absent. Tolerance here matches `claims.py`, which
normalises whitespace before its verbatim check for the same reason.
"""
from __future__ import annotations

import re
from typing import Dict, List, Optional, Tuple

# In emission order. The order is not presentation: the claim surface is the
# denominator every coverage figure divides by, so a surface closed after the
# findings is not frozen, it is a count made to fit.
BLOCKS: Tuple[str, ...] = ("CLAIM SURFACE", "REPORT", "LIMITATIONS", "GAP MAP")

# The review's four, mirroring the audit's: a frozen surface, the document, its
# limitations, and the one-page read. Same shapes, same rules, because the
# review runner had the defect this module was written to remove — it read a
# turn that did not yield as the review, and told the reviewer "The review is
# received" whatever the turn contained.
REVIEW_BLOCKS: Tuple[str, ...] = ("REVIEW SURFACE", "REVIEW", "LIMITATIONS",
                                  "SUMMARY")

# The delivery stage's three, and they are deliberately NOT a document. The
# agent there writes only the connecting prose — a cover, a passage per finding
# group, and the coverage statement in sentences — and the script fits those
# around findings it copies verbatim. It never emits a finding, so it cannot
# alter one: the guarantee is structural rather than a rule it is asked to obey.
DELIVERY_BLOCKS: Tuple[str, ...] = ("COVER", "SECTION NOTES", "COVERAGE")

# Every block name this module knows, for the marker regexes below.
_ALL: Tuple[str, ...] = tuple(dict.fromkeys(BLOCKS + REVIEW_BLOCKS
                                              + DELIVERY_BLOCKS))

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

    ANCHORED, BECAUSE A MENTION IS NOT AN EMISSION. This module's opening line
    already specifies "each opens with `=== NAME ===` on its own line"; the
    pattern searched anywhere until 2026-08-29, so prose naming a block counted
    as delivering it. The `cs2_flashnext_med` run yielded with the status line
    "next I'll emit === REPORT === (Material conclusion, findings worst-first
    with both citations) then === LIMITATIONS === then === GAP MAP ===", which
    marked three blocks delivered and ended the engagement. `report.md` was
    fifteen words, cut from the middle of that sentence.

    The agent was doing as it was told. `tools.py` asks a yield's `text` for "a
    brief status line ... so they hear where things stand", and METHOD §16
    makes those same strings the proof of delivery — so naming a step it still
    owes and taking that step are the same characters. Nothing the agent can
    write resolves that; the reader has to. A marker in the middle of a
    sentence is commentary, a marker opening a line is a block.

    Not a complete guard: blocks listed one per line still read as delivered.
    That is why the runner also refuses to end an engagement on a yield.

    The whitespace tolerance is unchanged and load-bearing — on 2026-08-25 a
    model emitted `===\nLIMITATIONS ===`, a line wrap inside the literal, and
    exact matching reported the requirement absent. The anchor binds the first
    `===` to the line start and leaves the wrap inside the marker matchable.
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


def rejection(name: str, section: str = "METHOD §16") -> str:
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
