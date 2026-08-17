#!/usr/bin/env python3
"""One-time offline retirement of redundant memories.

Companion to the write-time gate in chat/memories.py, which stops new
duplicates but cannot touch the ones already stored. Run with the agent
DOWN: it edits resources.json directly rather than going through a live
ResourceManager, so a concurrent write would lose one side.

What it does NOT do: delete anything. A memory is retired by stamping
`superseded_by` on it, which `_recall` honours at the dereference it
already performs. Actual removal is not available — delete_resource only
marks the note-level indexer, while recall reads the per-collection chunk
store that nothing removes from. Retired chunks stay on disk until a
separate index rebuild reclaims them; they are invisible to recall either
way.

Clusters are found by embedding-free text similarity and then JUDGED by
the same restatement / revision / distinct question the write-time gate
asks, because similarity cannot separate "User is 79" from "User is 80"
— the pair scores like a verbatim duplicate and wants the opposite
handling. Anything the judge declines to call redundant is left alone.

Usage:
    python3 src/scripts/memory_cleanup.py            # dry run, prints a plan
    python3 src/scripts/memory_cleanup.py --apply    # writes the plan
    python3 src/scripts/memory_cleanup.py --undo     # clears every stamp
"""

import argparse
import difflib
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from utils.file_utils import atomic_write_json  # noqa: E402

RESOURCES = Path('scenarios/jill_chat/resources/Jill/resources.json')
MEM_LOG = Path('scenarios/jill_chat/Jill/memory/memories.jsonl')
BACKEND = 'http://127.0.0.1:5000/v1/chat/completions'

# Below this, two memories are not the same fact and the judge is not
# consulted. Matches the write-time gate's 0.80 in spirit; text ratio and
# cosine are different scales, so this is tuned to the observed clusters.
SIMILARITY = 0.82


def norm(s):
    return ' '.join(str(s).lower().replace('.', '').split())


def load_memories(resources):
    """Every live memory note, oldest first, as (note_id, text, created_at,
    entity)."""
    notes = resources['note_instances']
    out = []
    for nid, note in notes.items():
        props = note.get('properties') or {}
        if props.get('kind') != 'memory':
            continue
        if props.get('superseded_by') or props.get('retired'):
            continue
        text = str(props.get('content', '') or '').strip()
        if text:
            out.append((nid, text, str(props.get('created_at') or ''),
                        str(props.get('entity') or '')))
    out.sort(key=lambda t: t[2])
    return out


def cluster(memories):
    """Group memories that are the same fact about the same subject.

    Entity is a hard partition, not a tiebreak. "The user's name is Bruce"
    (entity=Sentinel) and "The user's name is Jack" (entity=Jack) differ by
    one word, cluster on any text measure, and are facts about two
    different people — and the judge, asked which supersedes which, picks
    the newer. That is not hypothetical: it is what this script did on its
    first run before this partition existed.
    """
    seen, clusters = set(), []
    for i, (nid, text, _created, entity) in enumerate(memories):
        if nid in seen:
            continue
        group = [memories[i]]
        for j in range(i + 1, len(memories)):
            if memories[j][0] in seen or memories[j][3] != entity:
                continue
            if difflib.SequenceMatcher(
                    None, norm(text), norm(memories[j][1])).ratio() > SIMILARITY:
                group.append(memories[j])
                seen.add(memories[j][0])
        if len(group) > 1:
            seen.add(nid)
            clusters.append(group)
    return clusters


def judge(held, new):
    """restatement / revision / distinct — the write-time gate's question,
    asked of the same local backend. Any failure returns 'distinct', which
    retires nothing."""
    import requests
    payload = {
        'model': 'google/gemma-4-31B-it',
        'messages': [
            {'role': 'system', 'content':
             'You compare two statements a system has recorded about the '
             'same subject. Answer with exactly one word.'},
            {'role': 'user', 'content':
             f"HELD is a fact already stored. NEW is about to be stored.\n\n"
             f"HELD:\n{held}\n\nNEW:\n{new}\n\n"
             "Answer with one word:\n"
             "restatement — NEW says the same thing as HELD and adds "
             "nothing; storing it would only duplicate.\n"
             "revision — NEW is the same fact with a changed or corrected "
             "value, so HELD is now out of date (an age that went up, a "
             "setting that changed, a plan replaced).\n"
             "distinct — NEW carries something HELD does not; both are "
             "worth keeping."}],
        'max_tokens': 8, 'temperature': 0.0,
    }
    try:
        r = requests.post(BACKEND, json=payload, timeout=60)
        r.raise_for_status()
        raw = r.json()['choices'][0]['message']['content']
    except Exception as e:
        print(f"    ! judge failed ({e}); keeping both")
        return 'distinct'
    verdict = (raw or '').strip().strip('.`"\'').lower()
    for known in ('restatement', 'revision', 'distinct'):
        if verdict.startswith(known):
            return known
    print(f"    ! judge said {verdict[:40]!r}, no verdict; keeping both")
    return 'distinct'


def build_plan(clusters):
    """Within each cluster the newest is the survivor; every older member
    is judged against it and retired only on restatement or revision."""
    plan = []
    for group in clusters:
        survivor = group[-1]          # created_at ascending, so last is newest
        print(f"\n  cluster of {len(group)} [{survivor[3] or '-'}] — keeping "
              f"{survivor[0]} ({survivor[2][:10]}): {survivor[1][:90]}")
        for nid, text, created, _entity in group[:-1]:
            verdict = judge(text, survivor[1])
            mark = {'restatement': 'retire', 'revision': 'retire'}.get(verdict)
            print(f"    {nid} ({created[:10]}) {verdict:12s}"
                  f"{'→ retire' if mark else '→ keep'}: {text[:80]}")
            if mark:
                plan.append({'note_id': nid, 'superseded_by': survivor[0],
                             'verdict': verdict, 'text': text,
                             'survivor_text': survivor[1]})
    return plan


def apply_plan(resources, plan):
    stamp = datetime.now(timezone.utc).isoformat()
    for item in plan:
        props = resources['note_instances'][item['note_id']]['properties']
        props['superseded_by'] = item['superseded_by']
        props['superseded_at'] = stamp
        props['superseded_by_pass'] = 'memory_cleanup'
    atomic_write_json(RESOURCES, resources)
    with MEM_LOG.open('a') as f:
        for item in plan:
            f.write(json.dumps({
                'event': 'retired',
                'note_id': item['note_id'],
                'text': item['text'],
                'superseded_by': item['superseded_by'],
                'verdict': item['verdict'],
                'ts': stamp,
                'character': 'Jill',
                'trigger': 'memory_cleanup',
            }) + '\n')


def undo(resources):
    n = 0
    for note in resources['note_instances'].values():
        props = note.get('properties') or {}
        if props.pop('superseded_by_pass', None) == 'memory_cleanup':
            props.pop('superseded_by', None)
            props.pop('superseded_at', None)
            n += 1
    atomic_write_json(RESOURCES, resources)
    print(f"cleared {n} stamps written by this pass")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true',
                    help='write the plan (default is a dry run)')
    ap.add_argument('--undo', action='store_true',
                    help='clear every stamp this pass wrote')
    ap.add_argument('--again', action='store_true',
                    help='allow a second --apply over an already-cleaned store')
    args = ap.parse_args()

    if not RESOURCES.exists():
        sys.exit(f"not found: {RESOURCES} — run from the repo root")
    resources = json.loads(RESOURCES.read_text())

    if args.undo:
        undo(resources)
        return

    # Re-running compounds: each pass retires a cluster's losers, which
    # re-partitions what is left and can pair survivors that were never
    # meant to meet. The second --apply of the first session did precisely
    # that. This is a one-time pass; a second one needs a deliberate flag.
    already = sum(1 for n in resources['note_instances'].values()
                  if (n.get('properties') or {}).get(
                      'superseded_by_pass') == 'memory_cleanup')
    if already and args.apply and not args.again:
        sys.exit(f"this pass has already retired {already} memories. "
                 f"Re-running compounds — inspect the result first, then "
                 f"--apply --again if you really mean it (or --undo).")

    memories = load_memories(resources)
    print(f"live memories: {len(memories)}")
    clusters = cluster(memories)
    redundant = sum(len(g) - 1 for g in clusters)
    print(f"clusters above {SIMILARITY} similarity: {len(clusters)} "
          f"({redundant} candidate retirements)")

    plan = build_plan(clusters)
    print(f"\n{'APPLYING' if args.apply else 'DRY RUN'} — "
          f"{len(plan)} of {redundant} candidates judged redundant, "
          f"{len(memories) - len(plan)} memories remain live")
    if args.apply:
        apply_plan(resources, plan)
        print(f"stamped {len(plan)} notes; logged to {MEM_LOG}")
        print("undo with: python3 src/scripts/memory_cleanup.py --undo")
    else:
        print("re-run with --apply to write it")


if __name__ == '__main__':
    main()
