"""Entry point: python -m dev.world_map [--router tcp/HOST:PORT]

Precedence for router: --router > $ZENOH_CONNECT > tcp/127.0.0.1:7447.

Spec: docs/world_map_spec.md.
"""
from __future__ import annotations

import argparse
import logging
import sys

from .config import ENV_VAR, FuserConfig, resolve_router
from .controller import FuserController
from .ui_qt import run_app


def _parse_args(argv):
    p = argparse.ArgumentParser(
        prog="world_map",
        description="Desktop world-map fuser (consumer-only). "
                    "Subscribes to body/map/local_2p5d + body/odom; "
                    "publishes body/map/world_driveable.",
    )
    p.add_argument(
        "--router", default=None,
        help=f"Zenoh router endpoint (overrides ${ENV_VAR}); "
             f"e.g. tcp/192.168.1.50:7447",
    )
    p.add_argument(
        "--world-extent-m", type=float, default=40.0,
        help="Square world side length in meters (default 40).",
    )
    p.add_argument(
        "--world-resolution-m", type=float, default=0.08,
        help="Cell size; must match Pi local_map.resolution_m (default 0.08).",
    )
    p.add_argument(
        "--publish-hz", type=float, default=2.0,
        help="World-driveable publish rate cap (default 2 Hz).",
    )
    p.add_argument(
        "--no-autoconnect", action="store_true",
        help="Don't connect on startup; wait for the user to click Connect.",
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
    config = FuserConfig(
        router=resolve_router(args.router),
        world_extent_m=args.world_extent_m,
        world_resolution_m=args.world_resolution_m,
        publish_hz=args.publish_hz,
    )
    logging.getLogger(__name__).info(
        f"world_map starting; router={config.router} "
        f"(override via --router or ${ENV_VAR})"
    )
    controller = FuserController(config)
    if not args.no_autoconnect:
        ok, err = controller.connect()
        if not ok:
            logging.getLogger(__name__).warning(
                f"autoconnect failed ({err}); UI will let you retry"
            )
    try:
        return run_app(controller, config)
    finally:
        controller.shutdown()


if __name__ == "__main__":
    sys.exit(main())
