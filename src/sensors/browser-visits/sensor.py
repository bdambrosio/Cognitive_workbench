"""Browser visits sensor — drains the url_listener queue."""

import sys
from pathlib import Path

# Ensure src/ is importable (sensor_runner may not set this)
_src = str(Path(__file__).resolve().parent.parent.parent)
if _src not in sys.path:
    sys.path.insert(0, _src)

from url_listener import pop_visits


def run(context):
    visits = pop_visits()
    if not visits:
        return {'status': 'nothing', 'content': '', 'metadata': {}}

    data = []
    for v in visits:
        data.append({
            'url': v.get('url', ''),
            'title': v.get('title', ''),
            'timestamp': v.get('timestamp', ''),
        })

    n = len(data)
    return {
        'status': 'ok',
        'content': {
            'summary': f"User opened {n} page{'s' if n != 1 else ''} in the browser",
            'data': data,
        },
        'metadata': {'visit_count': n},
    }
