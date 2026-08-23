"""Load a workflow document into the static portion of the ReAct prompt.

A workflow is the procedure a task-specific agent executes: its phases, the
vocabulary its output must use, and what a finished deliverable looks like.
`audit/METHOD.md` is the first one.

WHY THIS IS NOT A TOOL READ. The audit ran for a day reading its own method
with `inspect`, and the recommendation taxonomy never survived to the turn
that needed it. Tool observations are *evidence*, and evidence is designed to
decay — capped at 1000 chars in the stored trace, and dropped entirely once a
turn falls out of the history window. That decay is correct for evidence and
fatal for procedure. Measured 2026-08-23: across four runs the words
"Conditional", "Material" and "Walk" appear zero times in any working log,
and the surviving fragment of the method ended mid-sentence in §2.

So procedure goes where nothing truncates it: the system prompt, built once
per turn and reused byte-identically across every iteration (see react.py's
store-and-append note). The document stays a single reviewable file in git —
this loads it, it does not copy it.

AUDIENCE MARKERS. A workflow document is written for two readers: the agent
executing the work, and the human running the practice. Liability posture and
commercial notes are the second reader's. They are marked in the source and
stripped here — not to save tokens (it is ~13% of this file) but because
"here is how you would be sued" in an auditor's system prompt invites
defensive hedging in a document whose value is plain statement.

Marked by a line reading `<!-- audience: practice -->` anywhere in a `## `
section; the whole section is dropped. Anything unmarked loads.
"""

from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import List

logger = logging.getLogger('chat_loop')

PRACTICE_MARKER = '<!-- audience: practice -->'

# Split before each `## ` heading, keeping the heading with its body. The
# preamble (anything before the first heading) is section 0 and always loads.
_SECTION_RE = re.compile(r'(?m)^(?=## )')


def load_workflow(path: Path) -> str:
    """Return the executing-audience text of the workflow document at `path`.

    Raises SystemExit if the file is missing or empty. A workflow that
    silently fails to load leaves an agent improvising the procedure it was
    supposed to be given, which is indistinguishable from the bug this
    module exists to fix.
    """
    path = Path(path)
    try:
        raw = path.read_text(encoding='utf-8')
    except OSError as e:
        raise SystemExit(
            f"workflow document {path} could not be read ({e}). Fix the "
            f"scenario rather than running without the procedure — an agent "
            f"missing its method improvises one, and the output looks "
            f"plausible either way.")
    if not raw.strip():
        raise SystemExit(f"workflow document {path} is empty.")

    sections: List[str] = _SECTION_RE.split(raw)
    kept = [s for s in sections if PRACTICE_MARKER not in s]
    dropped = len(sections) - len(kept)
    text = ''.join(kept).rstrip() + '\n'

    logger.info(
        "workflow loaded: %s — %d chars, %d of %d sections (%d practice-only "
        "dropped)", path, len(text), len(kept), len(sections), dropped)
    return text
