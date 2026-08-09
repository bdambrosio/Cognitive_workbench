"""Browser visits sensor — drains the url_listener queue.

Emits a prose situation report, not structured content: non-tick sensors
reach the chat loop through the `text` field of the sense_data envelope,
and a {'summary': ..., 'data': ...} payload has no `text` for it to read.

Unlike the other polling sensors this one holds no per-character state —
pop_visits() drains a single shared queue, so whichever character polls
first consumes the batch. Mounting it on more than one character at a time
would split the stream between them.
"""

import logging
import sys
from pathlib import Path

# Ensure src/ is importable (sensor_runner may not set this)
_src = str(Path(__file__).resolve().parent.parent.parent)
if _src not in sys.path:
    sys.path.insert(0, _src)

from url_listener import pop_visits  # noqa: E402

logger = logging.getLogger(__name__)

_NOTHING = {'status': 'nothing', 'content': '', 'metadata': {}}


def _describe(visits: list) -> str:
    """A self-contained report of what the user just looked at.

    Non-tick sensors arrive as user-like turns, so this has to read as an
    event the agent is noticing rather than as something anyone said.
    """
    n = len(visits)
    lines = [f"The user opened {n} page{'s' if n != 1 else ''} "
             f"in the browser."]
    lines.append("")

    for v in visits:
        title = v['title'] or '(untitled)'
        lines.append(f"  \"{title}\"")
        if v['url']:
            lines.append(f"    {v['url']}")
        if v['timestamp']:
            lines.append(f"    at {v['timestamp']}")

    lines.append("")
    lines.append("Nobody has said anything — you noticed this yourself. "
                 "Browsing is not a request for help, and most of it is "
                 "none of your business. Reply only if it bears directly "
                 "on work you and the user have in hand; otherwise stay "
                 "silent.")
    return "\n".join(lines)


def run(context):
    visits = pop_visits()
    if not visits:
        return _NOTHING

    data = []
    for v in visits:
        data.append({
            'url': v.get('url', ''),
            'title': v.get('title', ''),
            'timestamp': v.get('timestamp', ''),
        })

    n = len(data)
    logger.info(f"browser-visits: {n} visit(s) drained")
    return {
        'status': 'ok',
        'content': _describe(data),
        'metadata': {'visit_count': n,
                     'urls': [d['url'] for d in data if d['url']]},
    }
