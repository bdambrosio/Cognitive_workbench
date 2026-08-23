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


# ---------------------------------------------------------------------------
# Workflow mode
# ---------------------------------------------------------------------------

# Subsystems an agent executing a procedure does not need. Each entry is
# (ChatLoop attribute, scenario key that overrides it, what it costs).
#
# MEASURED on an audit run, 2026-08-23, by leg 2: 1,252 chars of discourse
# state and 2,791 of companion state per turn — relationship modelling for an
# engagement with no relationship yet — plus two extra LLM calls per turn for
# the updates.
#
# NOT IN THIS LIST, deliberately: concerns. `autonomy_enabled=False` already
# gates concern FIRING; concern CREATION is how a `yield` carries its
# remainder to the next leg, and an audit runs on yields. Suppressing them
# would not produce a leaner audit, it would produce one that cannot continue
# itself.
#
# The membership of this list is a judgement, and judgements rot quietly — a
# subsystem added later is silently not covered. That is why applying it
# returns what it touched, for the caller to log.
_SUPPRESSED = [
    ('discourse_enabled',   'discourse',
     'discourse state + the per-turn companion model update'),
    ('orientation_enabled', 'orientation',
     'the per-turn orientation block'),
]


def apply_workflow_mode(loop, character_config: dict) -> List[str]:
    """Switch off relationship machinery for an agent executing a procedure.

    Returns the human-readable descriptions of what was suppressed, or []
    when workflow_mode is off. An explicit scenario setting always wins: a
    config that says `discourse: {enabled: true}` keeps discourse, so the
    comparison "same workflow, machinery on" stays available.
    """
    if not bool(character_config.get('workflow_mode')):
        return []
    suppressed: List[str] = []
    for attr, key, what in _SUPPRESSED:
        block = character_config.get(key)
        if isinstance(block, dict) and 'enabled' in block:
            continue                      # explicit scenario setting wins
        setattr(loop, attr, False)
        suppressed.append(what)
    return suppressed
