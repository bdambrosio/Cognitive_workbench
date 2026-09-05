"""`extract_external`: verbatim evidence from the bound external repository,
with no model in the loop.

WHY A SECOND TOOL BESIDE `inspect_external`. The subagent behind
`inspect_external` runs a model loop of up to twelve emissions to answer a
request, and its answer is a model's retelling of what it read. Measured on
the chhoto engagement (2026-09-04): ten of sixteen requests were "quote
lines a to b of file f", each cost about twelve sequential emissions, and one
quote came back paraphrased. A request that names the file and the lines
does not need a model to fulfil it. This tool reads the lines, numbers them
as the subagent's `read` does, and returns them unchanged, so the text the
auditor cites is the text in the file.

WHAT IT WRITES. One trace file per call, in the same shape as an
`inspect_external` trace: the `[claims …]` prefix the claims-audit runner
reads back (`trace_claims`), one iteration whose ACTION carries `"file"` or
`"pattern"` (so `files_read` and `files_matched` credit the file), and a
FINAL ANSWER that opens with `file:a-b` so `trim_trace` keeps the span. The
runner takes both trace prefixes as evidence requests.

WHAT IT IS NOT. Not a search over unknown files: `pattern` returns matching
lines, capped, and says where to read next. Finding which file holds the
answer is still `inspect_external`'s job.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

from chat.subagents.code_subagent import _tool_grep, _tool_read
from utils.subagent_trace import write_subagent_trace

logger = logging.getLogger(__name__)

LABEL = 'extract_external'
#: The most lines one call returns. A span this long is a file, not a
#: citation; the cap keeps a whole-file read out of the main loop's context.
MAX_SPAN_LINES = 200


def _claim_ids(claims: Any) -> List[int]:
    if not isinstance(claims, list):
        return []
    out = set()
    for c in claims:
        if isinstance(c, int):
            out.add(c)
        elif isinstance(c, str) and c.strip().isdigit():
            out.add(int(c.strip()))
    return sorted(out)


def extract_external(repo_root: Path, trace_dir: Path, *,
                     file: Optional[str] = None,
                     lines: Any = None,
                     pattern: Optional[str] = None,
                     claims: Any = None) -> str:
    """Return the requested lines verbatim and record the request.

    Two forms. `file` with `lines=[a, b]` returns lines a..b of that file,
    numbered `N|text`. `pattern`, with `file` optional, returns the matching
    lines as `path:line:text`. Either form takes `claims`, the ids of the
    frozen claims the request serves, which is written into the trace.
    """
    ids = _claim_ids(claims)
    prefix = f"[claims {', '.join(str(c) for c in ids)}] " if ids else ""
    pattern = (pattern or '').strip() if isinstance(pattern, str) else ''
    file = (file or '').strip() if isinstance(file, str) else ''

    if pattern:
        obs = _tool_grep(repo_root, pattern, file or None)
        query = prefix + f"grep {pattern!r}" + (f" in {file}" if file else " across the tree")
        action: Dict[str, Any] = {"tool": "grep", "pattern": pattern}
        if file:
            action["file"] = file
        answer = obs
    elif file:
        try:
            if isinstance(lines, list) and len(lines) == 2:
                s, e = int(lines[0]), int(lines[1])
            elif isinstance(lines, int):
                s = e = int(lines)
            else:
                return ("ERROR: extract_external needs `lines` as [start, end] "
                        "with `file`; a whole file is not a citation")
        except (TypeError, ValueError):
            return f"ERROR: extract_external `lines` must be two integers, got {lines!r}"
        if s < 1 or e < s:
            return f"ERROR: extract_external lines {s}-{e} is not a range"
        if e - s + 1 > MAX_SPAN_LINES:
            return (f"ERROR: extract_external span of {e - s + 1} lines exceeds "
                    f"the cap of {MAX_SPAN_LINES}; ask for the lines you will cite")
        obs = _tool_read(repo_root, file, s, e)
        query = prefix + f"read {file} lines {s}-{e}"
        action = {"tool": "read", "file": file, "start_line": s, "end_line": e}
        # The reference line is what trim_trace keys on; the lines follow it
        # exactly as read.
        answer = f"{file}:{s}-{e}\n{obs}"
    else:
        return ("ERROR: extract_external needs `file` with `lines` [start, end], "
                "or `pattern` (optionally with `file`)")

    write_subagent_trace(trace_dir, LABEL, query,
                         [{"action": action, "observation": obs}],
                         answer, 'respond')
    return answer
