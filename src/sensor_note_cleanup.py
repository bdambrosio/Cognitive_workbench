"""Supervised deletion of notes misfiled under a sensor as their `entity`.

Before sensor turns became their own class (`is_sensor_source`), a sensor
push ran the whole user-turn machinery, so reflection and the discourse
pass attributed their writes to the machine that pushed the text. In
`jill_chat` that left memories, user_concerns and discourse segments filed
under `entity: sensor:factorio-telemetry` — including standing
instructions the user had actually given.

The code no longer produces these; this removes the ones already written.
Tombstoned verbatim to <memory>/sensor_misfiled_graveyard.jsonl first, so
a fact deleted here is still recoverable by hand.

Selects on the note's `entity` property carrying the `sensor:` source
prefix that sensor_runner stamps — the same structural marker
is_sensor_source() reads, not a guess about content.

Run with the chat session DOWN — this loads and saves the world's
resource files directly:

    python3 src/sensor_note_cleanup.py --world jill_chat --agent Jill
    python3 src/sensor_note_cleanup.py --world jill_chat --agent Jill --execute
"""
import argparse
import json
import logging
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from chat.chat_loop import SENSOR_SOURCE_PREFIX  # noqa: E402
from infospace_resource_manager import InfospaceResourceManager  # noqa: E402

logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')


def _misfiled(mgr) -> list:
    """(note_id, kind, entity, text) for every note whose entity is a
    sensor source. Sorted by id so dry run and execute agree."""
    out = []
    for nid, note in sorted(mgr.resource_registry.items()):
        if not nid.startswith('Note_') or not isinstance(note, dict):
            continue
        props = note.get('properties') or {}
        entity = str(props.get('entity') or '')
        if not entity.startswith(SENSOR_SOURCE_PREFIX):
            continue
        out.append((nid, str(props.get('kind') or '?'), entity,
                    str(props.get('content') or '')))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description='Delete notes misfiled under a sensor entity '
                    '(dry-run by default).')
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

    doomed = _misfiled(mgr)
    for nid, kind, entity, text in doomed:
        print(f"{nid:<12} {kind:<14} {entity:<28} {text[:80]!r}")
    verb = 'deleted' if args.execute else 'would delete'
    by_kind: dict = {}
    for _n, kind, _e, _t in doomed:
        by_kind[kind] = by_kind.get(kind, 0) + 1
    print(f"\n{verb}: {len(doomed)} note(s) — "
          + ", ".join(f"{v} {k}" for k, v in sorted(by_kind.items())))
    if not args.execute or not doomed:
        return

    # Tombstone BEFORE deleting: these hold true facts, just filed against
    # the wrong subject, and the graveyard is the only way back.
    grave = (mgr.base_dir.parent.parent / args.agent / 'memory'
             / 'sensor_misfiled_graveyard.jsonl')
    grave.parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).isoformat()
    with open(grave, 'a', encoding='utf-8') as f:
        for nid, kind, entity, _text in doomed:
            f.write(json.dumps({
                'deleted_at': stamp,
                'reason': 'entity was a sensor source, not a person',
                'note_id': nid,
                'kind': kind,
                'entity': entity,
                'note': mgr.resource_registry.get(nid),
            }, default=str) + '\n')

    failed = []
    for nid, _kind, _entity, _text in doomed:
        ok, err = mgr.delete_resource(nid)
        if not ok:
            failed.append((nid, err))
    for nid, err in failed:
        print(f"FAILED {nid}: {err}")
    if not mgr.save_to_file():
        raise SystemExit('world save FAILED — inspect before restarting the '
                         'session (graveyard entries were written)')
    print(f"world saved; {len(doomed) - len(failed)} deleted, "
          f"tombstones in {grave}")


if __name__ == '__main__':
    main()
