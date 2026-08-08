"""Entry point for the world surface: `python -m world.display`.

Matches how the launcher spawns the affect and canvas bridges. Unlike
those, this is one process rather than a publisher plus a viewer — the
world is authoritative state, not a latest-wins payload, so the thing
serving the browser and the thing answering agent queries have to be the
same object.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from world.server import main  # noqa: E402

if __name__ == '__main__':
    main()
