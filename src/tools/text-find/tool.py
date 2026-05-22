"""text-find — find a substring or regex pattern in text and return matches
with surrounding context. Deterministic; no LLM call.

Designed for the chat ReAct loop only — exposes `react_invoke(args, *, ...)`
as the entry-point. No OODA-style `tool()` wrapper (the OODA-era version
was prompt-augmentation only, with no Python implementation).
"""

from __future__ import annotations

import logging
import re
from typing import Any, Dict


_DEFAULT_CONTEXT_LINES = 1
_MAX_MATCHES = 100        # cap to keep observation size sane
_MAX_OBS_CHARS = 8000     # cap final observation


def _format_match(lines: list, line_idx: int, char_idx: int, match_text: str,
                  context_lines: int) -> str:
    start_line = max(0, line_idx - context_lines)
    end_line = min(len(lines), line_idx + context_lines + 1)
    out = [f"line {line_idx + 1}, col {char_idx + 1}: {match_text!r}"]
    for j in range(start_line, end_line):
        marker = "→ " if j == line_idx else "  "
        out.append(f"  {marker}{j + 1}: {lines[j]}")
    return "\n".join(out)


def react_invoke(args: Dict[str, Any], *, character_name=None, backend=None,
                 logger=None) -> Dict[str, Any]:
    """ReAct entry-point for chat-mode dispatch.

    Args (per Skill.md):
      source: required string — text to search
      pattern: required string — substring or Python regex
      context_lines: optional int (default 1) — lines of surrounding context

    Returns {'status': 'ok'|'empty'|'error', 'text': str}.
    """
    log = logger or logging.getLogger(__name__)
    source = args.get("source")
    pattern = args.get("pattern")
    context_lines = args.get("context_lines", _DEFAULT_CONTEXT_LINES)

    if not isinstance(source, str) or not source:
        return {"status": "error", "text": "text-find requires non-empty `source`"}
    if not isinstance(pattern, str) or not pattern:
        return {"status": "error", "text": "text-find requires non-empty `pattern`"}
    try:
        context_lines = int(context_lines)
    except (TypeError, ValueError):
        context_lines = _DEFAULT_CONTEXT_LINES
    context_lines = max(0, min(context_lines, 10))

    try:
        regex = re.compile(pattern)
    except re.error as e:
        return {"status": "error", "text": f"invalid regex pattern: {e}"}

    lines = source.split("\n")
    matches = []
    for line_idx, line in enumerate(lines):
        for m in regex.finditer(line):
            matches.append(_format_match(lines, line_idx, m.start(), m.group(0),
                                         context_lines))
            if len(matches) >= _MAX_MATCHES:
                break
        if len(matches) >= _MAX_MATCHES:
            break

    if not matches:
        return {"status": "empty", "text": f"no matches for {pattern!r}"}

    header = f"{len(matches)} match(es) for {pattern!r}:"
    body = "\n\n".join(matches)
    out = header + "\n\n" + body
    if len(out) > _MAX_OBS_CHARS:
        out = out[:_MAX_OBS_CHARS].rstrip() + f"\n…[truncated at {_MAX_OBS_CHARS} chars]"
    return {"status": "ok", "text": out}
