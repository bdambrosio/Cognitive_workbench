"""Entry point: python -m dev.body_stub [--router tcp/HOST:PORT]

Precedence for router: --router > $ZENOH_CONNECT > tcp/127.0.0.1:7447.
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Make src/ importable so the vision dock can reach src/vision_service.py
# and src/utils/*. Same pattern used by bench scripts.
_SRC = Path(__file__).resolve().parents[2] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from .config import ENV_VAR, StubConfig, resolve_router
from .controller import StubController
from .ui_qt import QtUI


def _parse_args(argv):
    p = argparse.ArgumentParser(
        prog="body_stub",
        description="Desktop Zenoh stub for the Body robot (dev tool). "
                    "Do NOT run alongside Jill — single client at a time.",
    )
    p.add_argument(
        "--router", default=None,
        help=f"Zenoh router endpoint (overrides ${ENV_VAR}); "
             f"e.g. tcp/192.168.1.50:7447",
    )
    p.add_argument(
        "--heartbeat-hz", type=float, default=5.0,
        help="heartbeat rate when Live command is enabled (default 5)",
    )
    p.add_argument(
        "--map-stale-s", type=float, default=2.0,
        help="local_map staleness threshold in seconds; bump for low Pi "
             "publish_hz (e.g. 6.0 for 0.5 Hz). Default 2.0.",
    )
    p.add_argument(
        "-v", "--verbose", action="store_true",
        help="debug logging",
    )
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config = StubConfig(
        router=resolve_router(args.router),
        heartbeat_hz=args.heartbeat_hz,
        map_stale_s=args.map_stale_s,
    )
    logging.getLogger(__name__).info(
        f"body_stub starting; router={config.router} "
        f"(override via --router or ${ENV_VAR})"
    )
    controller = StubController(config)
    ui = QtUI(controller, config)
    try:
        return ui.run()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    sys.exit(main())
