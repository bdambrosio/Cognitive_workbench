"""Rebuild the exact prompt of a past ReAct call from chat_trace.txt.

chat_trace.txt holds one block per turn: a header line
`[YYYY-MM-DD HH:MM:SS UTC] source=<who> iters=N exit=<reason>`, then
SYSTEM:, USER: and ASSISTANT: sections, one USER/ASSISTANT pair per
iteration. The first USER is written in full. Each later one is written as
`USER: [+M chars; first N unchanged from previous iteration]` followed by the
M new characters: the new prompt is the first N characters of the previous
one plus those M. `turn_calls` rebuilds every iteration's (system, user,
assistant) and checks each rebuilt length against N + M.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, Iterator, List, Optional

CHAT_TRACE = (Path(__file__).resolve().parents[2]
              / "scenarios/jill_chat/Jill/memory/chat_trace.txt")
RULE = "=" * 80
HEADER = re.compile(r"^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d) UTC\] source=(\S+) iters=(\d+) exit=(\S+)$")
# A section marker: a blank line, then USER: or ASSISTANT:, the USER one
# optionally carrying the delta note.
MARK = re.compile(r"\n\n(USER|ASSISTANT):(?: \[\+(\d+) chars; first (\d+) unchanged "
                  r"from previous iteration\])?\n")


def blocks(path: Path = CHAT_TRACE) -> Iterator[Dict[str, str]]:
    """Each turn block as {ts, source, iters, exit, body}, in file order."""
    head, body = None, []
    with path.open(encoding="utf-8", errors="replace") as fh:
        prev = None
        for line in fh:
            m = HEADER.match(line.rstrip("\n"))
            if m and prev is not None and prev.rstrip("\n") == RULE:
                if head:
                    yield {**head, "body": "".join(body[:-1])}   # drop the rule line before this header
                head = dict(zip(("ts", "source", "iters", "exit"), m.groups()))
                body = []
            elif head is not None:
                body.append(line)
            prev = line
    if head:
        yield {**head, "body": "".join(body)}


def turn_calls(block_body: str) -> List[Dict[str, str]]:
    """The block's iterations as [{system, user, assistant}], prompts rebuilt.
    Raises ValueError when a rebuilt length disagrees with the trace."""
    start = block_body.find("SYSTEM:\n")
    if start < 0:
        raise ValueError("block has no SYSTEM: section")
    body = block_body[start + len("SYSTEM:\n"):]
    marks = list(MARK.finditer(body))
    if not marks:
        raise ValueError("block has no USER/ASSISTANT sections")
    system = body[:marks[0].start()]
    calls, user = [], None
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(body)
        text = body[m.end():end]
        if m.group(1) == "USER":
            if m.group(2):
                add, keep = int(m.group(2)), int(m.group(3))
                user = user[:keep] + text
                if len(user) != keep + add:
                    raise ValueError(f"rebuilt user is {len(user)} chars, trace says {keep}+{add}")
            else:
                user = text
        else:
            calls.append({"system": system, "user": user, "assistant": text.strip()})
    return calls


def find(ts: str, source: str = "User", path: Path = CHAT_TRACE) -> Optional[Dict[str, str]]:
    """The block whose header has this UTC timestamp (YYYY-MM-DD HH:MM:SS)."""
    for b in blocks(path):
        if b["ts"] == ts and b["source"] == source:
            return b
    return None
