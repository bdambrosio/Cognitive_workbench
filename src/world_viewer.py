"""Read-only resource browser over a world that is NOT running.

    python3 src/world_viewer.py --world cs1_flashnext_med --port 3002

A workflow run leaves a complete world on disk and no process behind it.
`resource_browser.py` is a live-agent client — every one of its routes is a
zenoh query answered by queryables that `ChatLoop._open_zenoh` declares — so
by the time you want to look at an audit, there is nothing to answer. This
serves those queries from the saved files instead.

WHY NOT HOLD THE RUN OPEN. `_open_zenoh` also declares the `sense_data`
subscriber, so a held-open engagement is a live agent accepting input, which
is the isolation the fresh-world-per-target rule exists to provide. It would
also only help runs someone decided to hold in advance, and every finished run
is already complete on disk. This works on all of them, afterwards.

READ-ONLY BY CONSTRUCTION, not by policy. Only the four reading queryables are
declared; `resource/remove/*`, `resource/update/*` and `control/concern_manage`
are simply absent, so the browser's delete and edit controls get no responder
and fail rather than mutating an engagement you are inspecting. Nothing here
ever calls _persist_to_disk, so the world files are opened and not written.

A DISTINCT ZENOH IDENTITY, and this is not cosmetic. Queryables are declared on
`cognitive/<character>/...`. A viewer that called itself `Jill` while a live
Jill was running would answer the same keys as she does, and the browser —
which queries BEST_MATCHING — would show a silent mixture of the two. So the
viewer serves under `<agent>_at_<world>`, which cannot collide and names on
screen which world is being read. Override with --identity.

The offline-shell pattern (an InfospaceResourceManager loaded from file, wrapped
in a bare ChatLoop carrying only the attributes the mixins touch) is the one
src/concern_cleanup.py already uses for supervised concern cleanup.
"""
from __future__ import annotations

import argparse
import logging
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import zenoh                                                   # noqa: E402

from chat.chat_loop import ChatLoop                            # noqa: E402
from chat.concerns import (                                    # noqa: E402
    _AGENT_CONCERNS_COLLECTION_NAME, _USER_CONCERNS_COLLECTION_NAME)
from infospace_resource_manager import InfospaceResourceManager  # noqa: E402
from utils.zenoh_utils import make_localhost_config            # noqa: E402

logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
logger = logging.getLogger('world_viewer')


def build_shell(world: str, agent: str, identity: str) -> ChatLoop:
    """A ChatLoop with exactly the attributes the read handlers touch.

    `agent` selects which character's files to load; `identity` is the zenoh
    name served under. They are deliberately separate — see the module note on
    collisions with a live session of the same character.
    """
    mgr = InfospaceResourceManager(
        world_name=world, world_config={'world_name': world}, agent_name=agent)
    mgr.load_from_file()

    shell = object.__new__(ChatLoop)
    shell.character_name = identity
    shell.resource_manager = mgr
    shell._agent_concerns_collection_id = mgr.named_collections.get(
        _AGENT_CONCERNS_COLLECTION_NAME)
    shell._user_concerns_collection_id = mgr.named_collections.get(
        _USER_CONCERNS_COLLECTION_NAME)
    return shell


class WorldViewer:
    """Serves the reading half of a ChatLoop's resource queryables."""

    def __init__(self, world: str, agent: str, identity: str):
        self.world, self.agent, self.identity = world, agent, identity
        self.shell = build_shell(world, agent, identity)
        self.session = zenoh.open(make_localhost_config())
        base = f"cognitive/{identity}"
        # Held so the queryables outlive this call; zenoh undeclares on drop.
        self._q = [
            self.session.declare_queryable(
                f"{base}/resources", self.shell._handle_resources_list_query),
            # Single-chunk wildcard. It would also match resource/remove and
            # resource/update if those had no extra segment — they do, so a
            # mutation query finds no responder here, which is the point.
            self.session.declare_queryable(
                f"{base}/resource/*", self.shell._handle_resource_by_id_query),
            self.session.declare_queryable(
                f"{base}/concerns", self.shell._handle_concerns_query),
            self.session.declare_queryable(f"{base}/status", self._status),
        ]

    def _status(self, query) -> None:
        """The live handler reports whether a turn is in flight. There is no
        loop here, so it answers with the same shape and says what this is —
        `ready: True` would claim a viewer accepts input."""
        self.shell._reply(query, {
            'success': True,
            'ready': False,
            'action': f"offline viewer — read-only snapshot of world "
                      f"{self.world!r} (agent {self.agent})",
            'current_turn': None,
            'post_turn_busy': False,
            'inbox_backlog': 0,
            'last_reply_at': None,
            'character': self.identity,
            'read_only': True,
        })

    def counts(self) -> str:
        user, agent = self.shell._all_concerns_split()
        n_res = len(self.shell.resource_manager.get_resource_list())
        return (f"{n_res} resources, {len(agent)} agent_concerns, "
                f"{len(user)} user_concerns")

    def close(self) -> None:
        try:
            self.session.close()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("zenoh close failed: %s", e)


def main() -> int:
    ap = argparse.ArgumentParser(
        description='Read-only resource browser over a saved world.')
    ap.add_argument('--world', required=True,
                    help='world name, e.g. cs1_flashnext_med (a run\'s '
                         '`world` field in run_meta.json)')
    ap.add_argument('--agent', default='Jill', help='character to load')
    ap.add_argument('--port', type=int, default=3002,
                    help='browser port (default 3002; the launcher pins a '
                         'live session to 3001)')
    ap.add_argument('--identity', default=None,
                    help='zenoh name to serve under '
                         '(default <agent>_at_<world>)')
    ap.add_argument('--no-browser', action='store_true',
                    help='serve the queryables only; start the web UI '
                         'yourself')
    args = ap.parse_args()

    identity = args.identity or f"{args.agent}_at_{args.world}"
    viewer = WorldViewer(args.world, args.agent, identity)
    logger.info("world %s loaded read-only: %s", args.world, viewer.counts())
    logger.info("serving cognitive/%s/{resources,resource/*,concerns,status}",
                identity)

    proc = None
    if not args.no_browser:
        proc = subprocess.Popen(
            [sys.executable, 'resource_browser.py', '--map', args.world,
             '--character', identity, '--port', str(args.port),
             '--no-browser'],
            cwd=str(Path(__file__).parent))
        logger.info("resource browser on http://127.0.0.1:%d "
                    "(character=%s)", args.port, identity)

    stop = {'now': False}

    def _sig(_signum, _frame):
        stop['now'] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    logger.info("Ctrl-C to stop.")
    try:
        while not stop['now']:
            if proc is not None and proc.poll() is not None:
                logger.error("resource browser exited (%s)", proc.returncode)
                break
            time.sleep(0.3)
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        viewer.close()
        logger.info("viewer stopped; world %s was not modified", args.world)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
