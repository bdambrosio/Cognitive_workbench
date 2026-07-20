"""Supervised batch cleanup of dead agent_concerns.

Applies the same disposition rules as the in-session sweep
(ConcernsMixin._delete_dead_agent_concerns): abandoned concerns and
satisfied one-shots past the grace period are tombstoned verbatim to
<memory>/concerns_graveyard.jsonl and deleted; seeds and satisfied
durable concerns are never touched.

Run with the chat session DOWN — this loads and saves the world's
resource files directly:

    python3 src/concern_cleanup.py --world jill_chat --agent Jill            # dry run
    python3 src/concern_cleanup.py --world jill_chat --agent Jill --execute
"""
import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from chat.chat_loop import ChatLoop  # noqa: E402
from chat.concerns import _AGENT_CONCERNS_COLLECTION_NAME  # noqa: E402
from infospace_resource_manager import InfospaceResourceManager  # noqa: E402

logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')


def main() -> None:
    ap = argparse.ArgumentParser(
        description='Batch cleanup of dead agent_concerns (dry-run by default).')
    ap.add_argument('--world', required=True, help='world name, e.g. jill_chat')
    ap.add_argument('--agent', default='Jill', help='character name')
    ap.add_argument('--execute', action='store_true',
                    help='tombstone + delete and save the world '
                         '(default: print what would be deleted)')
    args = ap.parse_args()

    mgr = InfospaceResourceManager(
        world_name=args.world,
        world_config={'world_name': args.world},
        agent_name=args.agent)
    mgr.load_from_file()

    shell = object.__new__(ChatLoop)
    shell.character_name = args.agent
    shell.resource_manager = mgr
    cid = mgr.named_collections.get(_AGENT_CONCERNS_COLLECTION_NAME)
    if not cid:
        raise SystemExit(
            f"no agent_concerns collection in world {args.world!r} "
            f"for agent {args.agent!r}")
    shell._agent_concerns_collection_id = cid

    doomed = shell._delete_dead_agent_concerns(dry_run=not args.execute)
    for nid, reason, text in doomed:
        print(f"{nid:<14} {reason:<18} {text[:90]!r}")
    verb = 'deleted' if args.execute else 'would delete'
    print(f"\n{verb}: {len(doomed)} concern(s)")
    if args.execute and doomed:
        if not mgr.save_to_file():
            raise SystemExit('world save FAILED — inspect before restarting '
                             'the session (graveyard entries were written)')
        print(f"world saved; tombstones in "
              f"{shell._memory_dir() / 'concerns_graveyard.jsonl'}")


if __name__ == '__main__':
    main()
