"""Per-call trace writer shared by the ReAct-shaped tool subagents.

`security`, `recall`, and `inspect`/`inspect_external` each wrote a
byte-identical version of this, differing only in the label used for the
filename prefix, the header tag, and the failure log line. Consolidated
here so the trace format has one definition.

Not used by the main chat ReAct loop: `chat_loop._write_react_trace`
records a different structure (a bound working log, recall block, and
image ref alongside the iterations) and is deliberately left alone.
"""

import json
import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


def write_subagent_trace(trace_dir: Path, label: str, query: str,
                         iters: List[Dict[str, Any]], answer: str,
                         exit_reason: str) -> Optional[Path]:
    """Write one trace file for a subagent call; return its path.

    Args:
        trace_dir: directory to write into; created if absent.
        label: subagent name. Used three ways — `<label>_<ts>.txt` as the
            filename, `[<label>]` in the header, and the prefix on the
            warning if the write fails.
        query: the natural-language question the subagent was given.
        iters: per-iteration records, each optionally carrying 'action',
            'raw', and 'observation'.
        answer: the synthesized final answer.
        exit_reason: how the loop ended (respond / max_iters / llm_error).

    Never raises: a failed trace write must not take down the call whose
    work it was recording. Returns None in that case.
    """
    try:
        trace_dir = Path(trace_dir)
        trace_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H-%M-%SZ')
        path = trace_dir / f'{label}_{ts}.txt'
        lines = [
            '=' * 80,
            f'[{label}] {ts} exit={exit_reason} iters={len(iters)}',
            '=' * 80,
            f'Query: {query}',
            '',
        ]
        for i, it in enumerate(iters, start=1):
            lines.append(f'--- iter {i} ---')
            lines.append('ACTION:')
            if it.get('action') is not None:
                lines.append(json.dumps(it['action'], indent=2))
            else:
                lines.append('(unparseable; raw follows)')
                lines.append(it.get('raw', ''))
            obs = it.get('observation', '')
            if obs:
                lines.append('OBSERVATION:')
                lines.append(obs)
            lines.append('')
        lines.append('FINAL ANSWER:')
        lines.append(answer)
        path.write_text('\n'.join(lines), encoding='utf-8')
        return path
    except Exception as e:
        logger.warning(f"{label}: trace write failed: {e}")
        return None
