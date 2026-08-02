#!/usr/bin/env python3
"""trace_claim — provenance audit walker (Level 1 verifiability).

Walks a stored resource back to its origin: note → source turn →
reasoning_trace record (raw tool observations, structured tool_meta).
Read-only, stdlib only.

Usage:
    python tools/trace_claim.py --world <world> [--agent <Agent>] Note_42
    python tools/trace_claim.py --world <world> [--agent <Agent>] turn:17
    ... [--full]   # don't truncate working_log lines

Exit status: 0 if the walk reached a reasoning_trace record (or the
resource legitimately has no turn provenance and says so), 1 on a broken
hop (missing file / dangling pointer) — a broken hop IS the audit finding.
"""

import argparse
import json
import sys
from pathlib import Path

TRUNC = 200


def die(msg: str) -> None:
    print(f"BROKEN HOP: {msg}", file=sys.stderr)
    sys.exit(1)


def clip(s: str, full: bool) -> str:
    s = str(s)
    if full or len(s) <= TRUNC:
        return s
    return s[:TRUNC] + f" …[+{len(s) - TRUNC} chars]"


def resolve_agent(world_dir: Path, agent: str | None) -> str:
    if agent:
        return agent
    res_root = world_dir / "resources"
    if not res_root.is_dir():
        die(f"no resources dir at {res_root}")
    agents = [p.name for p in res_root.iterdir() if p.is_dir()]
    if len(agents) != 1:
        die(f"can't autodetect agent (found {agents or 'none'}); pass --agent")
    return agents[0]


def load_resources(world_dir: Path, agent: str) -> dict:
    path = world_dir / "resources" / agent / "resources.json"
    if not path.is_file():
        die(f"no resources file at {path}")
    with open(path, encoding="utf-8") as f:
        return json.load(f).get("note_instances", {}) or {}


def find_turn_record(memory_dir: Path, turn_seq: int) -> dict | None:
    """Last match wins: turn_seq restarts each session, so the same seq
    can occur once per session — the latest occurrence is the one the
    sidecars (memories.jsonl, claims.jsonl) most recently joined to.
    CAVEAT: a cross-session join (an old memory's source_turn_seq) can
    therefore resolve to the wrong session's record; a session-unique
    seq is future work."""
    path = memory_dir / "reasoning_trace.jsonl"
    if not path.is_file():
        die(f"no reasoning trace at {path}")
    found = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("turn_seq") == turn_seq:
                found = rec
    return found


def memories_jsonl_lookup(memory_dir: Path, note_id: str) -> dict | None:
    path = memory_dir / "memories.jsonl"
    if not path.is_file():
        return None
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("note_id") == note_id:
                return rec
    return None


def claims_lookup(memory_dir: Path, turn_seq: int) -> dict | None:
    path = memory_dir / "claims.jsonl"
    if not path.is_file():
        return None
    found = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("turn_seq") == turn_seq:
                found = rec  # last record wins if ever re-attributed
    return found


def print_claims(claims_rec: dict, turn_rec: dict, notes: dict | None,
                 full: bool) -> bool:
    """Render the turn's attributed claims and structurally resolve each
    ref: $stepN must appear as a binding in the turn's working log,
    Note_N must exist in resources.json (when available), user_input is
    trivially present. Returns True if every ref resolved."""
    import re
    bindings = set(re.findall(r"^\$step\d+", turn_rec.get("working_log") or "",
                              re.MULTILINE))
    tool_meta = turn_rec.get("tool_meta") or {}
    all_ok = True
    print(f"\n== claims (turn {claims_rec.get('turn_seq')}, "
          f"reply_sha1 {str(claims_rec.get('reply_sha1'))[:12]}…) ==")
    for c in claims_rec.get("claims") or []:
        print(f"  [{c.get('grounding')}] {clip(c.get('claim', ''), full)}")
        for ref in c.get("refs") or []:
            if ref == "user_input":
                status = "ok (this turn's input)"
            elif ref.startswith("$step"):
                if ref in bindings:
                    n_src = len(((tool_meta.get(ref) or {}).get("meta") or []))
                    status = "ok" + (f" (+{n_src} structured source entr"
                                     f"{'y' if n_src == 1 else 'ies'})"
                                     if n_src else "")
                else:
                    status, all_ok = "DANGLING — no such binding", False
            elif ref.startswith("Note_"):
                if notes is None:
                    status = "unchecked (no resources.json)"
                elif ref in notes:
                    status = "ok"
                else:
                    status, all_ok = "DANGLING — note missing", False
            else:
                status, all_ok = "DANGLING — unknown ref form", False
            print(f"      ref {ref}: {status}")
    return all_ok


def print_turn(rec: dict, full: bool) -> None:
    print(f"\n== reasoning_trace turn {rec.get('turn_seq')} ==")
    for k in ("ts", "source", "autonomous", "exit_reason", "iters", "fire_id",
              "image_ref"):
        if k in rec:
            print(f"  {k}: {rec[k]}")
    print(f"  user_input: {clip(rec.get('user_input', ''), full)}")
    tool_meta = rec.get("tool_meta")
    if tool_meta:
        print("  tool_meta (structured provenance):")
        print("    " + json.dumps(tool_meta, indent=2).replace("\n", "\n    "))
    print("  working_log:")
    for line in (rec.get("working_log") or "").splitlines():
        print(f"    {clip(line, full)}")


def print_note(notes: dict, note_id: str, full: bool) -> dict:
    note = notes.get(note_id)
    if note is None:
        die(f"{note_id} not found in resources.json (deleted or never persisted; "
            f"conversation turn notes are trimmed at 50)")
    props = note.get("properties") or {}
    print(f"== {note_id} ==")
    for k in ("kind", "source_skill", "source_value", "created_by",
              "created_at", "fingerprint", "category", "polarity", "entity",
              "provenance", "creation_provenance", "triage_verdict",
              "source_turn_seq", "successor_of", "successor_depth"):
        if props.get(k) not in (None, ""):
            print(f"  {k}: {props[k]}")
    print(f"  content: {clip(props.get('content', ''), full)}")
    return props


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("target", help="Note_<n>, or turn:<seq>")
    ap.add_argument("--world", required=True)
    ap.add_argument("--agent", default=None)
    ap.add_argument("--full", action="store_true",
                    help="don't truncate content / working_log lines")
    ap.add_argument("--scenarios", default=None,
                    help="scenarios root (default: <repo>/scenarios)")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    scenarios = Path(args.scenarios) if args.scenarios else repo_root / "scenarios"
    world_dir = scenarios / args.world
    if not world_dir.is_dir():
        die(f"no world dir at {world_dir}")
    agent = resolve_agent(world_dir, args.agent)
    memory_dir = world_dir / agent / "memory"

    if args.target.startswith("turn:"):
        seq = int(args.target.split(":", 1)[1])
        rec = find_turn_record(memory_dir, seq)
        if rec is None:
            die(f"turn_seq {seq} not found in {memory_dir / 'reasoning_trace.jsonl'}")
        print_turn(rec, args.full)
        claims_rec = claims_lookup(memory_dir, seq)
        if claims_rec:
            res_path = world_dir / "resources" / agent / "resources.json"
            notes = load_resources(world_dir, agent) if res_path.is_file() else None
            if not print_claims(claims_rec, rec, notes, args.full):
                die(f"turn {seq} has claims with dangling refs (see above)")
        return

    notes = load_resources(world_dir, agent)
    props = print_note(notes, args.target, args.full)

    # Concern lineage: walk successor chain up to the root.
    parent = props.get("successor_of")
    seen = {args.target}
    while parent:
        if parent in seen:
            die(f"successor_of cycle at {parent}")
        seen.add(parent)
        print(f"\n-- successor_of --> {parent}")
        parent = print_note(notes, parent, args.full).get("successor_of")

    # Turn provenance: on-note pointer first, memories.jsonl sidecar second.
    turn_seq = props.get("source_turn_seq")
    if turn_seq is None:
        sidecar = memories_jsonl_lookup(memory_dir, args.target)
        if sidecar is not None:
            turn_seq = sidecar.get("source_turn_seq")
            print(f"\n(source_turn_seq via memories.jsonl sidecar: {turn_seq})")
    if turn_seq is None:
        kind = props.get("kind")
        if kind == "memory":
            die(f"memory {args.target} has no source_turn_seq on the note or "
                f"in memories.jsonl")
        print(f"\n(no turn provenance recorded for kind={kind!r} — "
              f"pre-Level-1 record or non-turn-derived note)")
        return
    rec = find_turn_record(memory_dir, int(turn_seq))
    if rec is None:
        die(f"source_turn_seq {turn_seq} not found in reasoning_trace.jsonl")
    print_turn(rec, args.full)
    claims_rec = claims_lookup(memory_dir, int(turn_seq))
    if claims_rec:
        if not print_claims(claims_rec, rec, notes, args.full):
            die(f"turn {turn_seq} has claims with dangling refs (see above)")


if __name__ == "__main__":
    main()
