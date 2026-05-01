#!/usr/bin/env python3
"""Trace-Grounded Introspective Fidelity Benchmark — runner (v0.1).

Drives a ChatLoop in-process through bench/introspective_fidelity/primer.yaml,
captures per-turn replies, and at each P-probe records a state snapshot
(concerns, reasoning_history, discourse, companion). Emits transcript.md
and snapshots/<probe_id>.json. Scoring is deferred to v0.2 — for v0.1,
read the transcript manually.

Usage:
    cd src
    python ../bench/introspective_fidelity/runner.py \\
        --scenario ../scenarios/jill-benchmark-chat.yaml

The runner overrides world_config.world_name with a per-run timestamped
value so each run starts with a clean state (scenarios/<world>/resources/
is created fresh under the timestamped name). It instantiates ChatLoop
directly rather than calling .run(), so no Zenoh session is opened —
_publish_say is a no-op when self._action_pub is None.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

# Paths
HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"

# Bring the chat package onto the import path. ChatLoop's siblings
# (infospace_resource_manager, conversation_store, character_evaluator,
# etc.) live in src/ as top-level modules, so src/ must be on sys.path.
sys.path.insert(0, str(SRC_DIR))

# Imports below depend on src/ being importable.
from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("bench.introspective_fidelity")


# ---------------------------------------------------------------------------
# YAML loading + per-run scenario mutation
# ---------------------------------------------------------------------------

def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> tuple[str, dict]:
    """Load scenario YAML and return (character_name, merged_config) for
    the single chat character. Replicates launcher.parse_characters'
    merge so per-character llm_config wins over top-level, then forces
    world_config.world_name to the override."""
    scenario = _load_yaml(scenario_path)
    llm_cfg = scenario.get("llm_config") or {}
    alt_llm = scenario.get("alt_llm_config") or {}
    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world_name_override
    setting = scenario.get("setting", "")
    chars = parse_characters(scenario, llm_cfg, world_cfg, setting, alt_llm)
    chat_chars = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat_chars) != 1:
        raise RuntimeError(
            f"expected exactly 1 chat character in {scenario_path}, found "
            f"{len(chat_chars)}"
        )
    return chat_chars[0]


# ---------------------------------------------------------------------------
# State snapshots
# ---------------------------------------------------------------------------

def _snapshot_concerns(loop: ChatLoop) -> List[Dict[str, Any]]:
    """Dump all concerns (active, satisfied, abandoned) with full
    metadata. Ground truth for P2/P3/P4."""
    cid = loop._concerns_collection_id
    if not cid:
        return []
    coll = loop.resource_manager.get_resource(cid)
    if not coll:
        return []
    note_ids = (coll.get("properties") or {}).get("content", []) or []
    out: List[Dict[str, Any]] = []
    for nid in note_ids:
        note = loop.resource_manager.get_resource(nid)
        if not note:
            continue
        props = note.get("properties") or {}
        out.append({
            "note_id": nid,
            "text": props.get("content", ""),
            "category": props.get("category", "durable"),
            "status": props.get("status", "active"),
            "provenance": props.get("provenance"),
            "seed": bool(props.get("seed", False)),
            "cadence_hours": loop._resolve_cadence_hours(props),
            "lifetime_days": loop._resolve_lifetime_days(props),
            "instruction": props.get("instruction"),
            "weight": loop._concern_decayed_weight(props),
            "created_at": props.get("created_at"),
            "last_engaged_at": props.get("last_engaged_at"),
            "last_acted_at": props.get("last_acted_at"),
        })
    return out


def _snapshot_reasoning_history(loop: ChatLoop) -> List[Dict[str, Any]]:
    """Dump the full reasoning_history collection. Ground truth for
    P1/P3/P5/P6/P7/P9/P11."""
    rid = getattr(loop, "_reasoning_history_collection_id", None)
    if not rid:
        return []
    coll = loop.resource_manager.get_resource(rid)
    if not coll:
        return []
    note_ids = (coll.get("properties") or {}).get("content", []) or []
    out: List[Dict[str, Any]] = []
    for nid in note_ids:
        note = loop.resource_manager.get_resource(nid)
        if not note:
            continue
        props = note.get("properties") or {}
        out.append({
            "note_id": nid,
            "content": props.get("content", ""),
            "compressed": props.get("compressed", ""),
            "timestamp": props.get("timestamp"),
            "source": props.get("source"),
        })
    return out


def _snapshot_discourse_and_companion(loop: ChatLoop, source: str
                                       ) -> Dict[str, str]:
    """Dump per-source discourse and companion strings. Ground truth
    for P6/P8/P11."""
    return {
        "discourse": loop._discourse_state.get(source, ""),
        "companion": loop._companion_state.get(source, ""),
    }


def _snapshot_all(loop: ChatLoop, source: str) -> Dict[str, Any]:
    return {
        "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "concerns": _snapshot_concerns(loop),
        "reasoning_history": _snapshot_reasoning_history(loop),
        "discourse_and_companion": _snapshot_discourse_and_companion(loop, source),
    }


# ---------------------------------------------------------------------------
# Reply capture
# ---------------------------------------------------------------------------

def _latest_reply(loop: ChatLoop, source: str) -> str:
    """Read the most recent outgoing turn for `source` from the
    conversation store. _process_user_turn ends with record_outgoing,
    so the last 'out' turn is our reply."""
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def run_benchmark(scenario_path: Path, primer_path: Path,
                  output_dir: Path, source: str = "User",
                  world_name: Optional[str] = None) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    snapshots_dir = output_dir / "snapshots"
    snapshots_dir.mkdir(parents=True, exist_ok=True)

    primer = _load_yaml(primer_path)
    turns = primer.get("turns") or []
    if not turns:
        raise RuntimeError(f"no turns found in {primer_path}")

    if world_name:
        # Caller supplied an existing or specific world to load. Useful for
        # isolated probe replays against a populated baseline world dir.
        # The caller is responsible for copying the baseline before mutating
        # if repeatability matters — this run will append to whatever's there.
        logger.info(f"world_name (provided): {world_name}")
    else:
        # Default: per-run timestamped name, ISO-ish + filesystem-safe.
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H-%M-%SZ")
        world_name = f"bench-introspective-{stamp}"
        logger.info(f"world_name (fresh): {world_name}")

    char_name, char_config = _build_character_config(scenario_path, world_name)
    if not char_config.get("chat", {}).get("benchmark_mode"):
        logger.warning(
            "scenario does not set chat.benchmark_mode=true; reflection "
            "will run on the background executor and probe-time snapshots "
            "may miss in-flight writes")

    # Instantiate ChatLoop without calling .run() — Zenoh stays closed.
    loop = ChatLoop(character_name=char_name, character_config=char_config)

    transcript_lines: List[str] = []
    transcript_lines.append(f"# Introspective Fidelity Benchmark — transcript")
    transcript_lines.append("")
    transcript_lines.append(f"- scenario: `{scenario_path.name}`")
    transcript_lines.append(f"- world_name: `{world_name}`")
    transcript_lines.append(f"- character: `{char_name}`")
    transcript_lines.append(f"- backend: `{(char_config.get('llm_config') or {}).get('server', '?')}` "
                            f"`{(char_config.get('llm_config') or {}).get('model', '')}`")
    transcript_lines.append(f"- started: {datetime.datetime.now(datetime.timezone.utc).isoformat()}")
    transcript_lines.append("")

    try:
        for turn in turns:
            tid = turn.get("id", "?")
            phase = turn.get("phase", "?")
            text = (turn.get("text") or "").strip()
            if not text:
                continue
            print(f"\n=== {tid} [{phase}] ===", flush=True)
            print(f"User: {text}", flush=True)

            loop._process_user_turn(source=source, text=text, close=False)
            reply = _latest_reply(loop, source)
            print(f"\n{char_name}: {reply}", flush=True)

            transcript_lines.append(f"## {tid} [{phase}]")
            transcript_lines.append("")
            transcript_lines.append(f"**User:**")
            transcript_lines.append("")
            transcript_lines.append(text)
            transcript_lines.append("")
            transcript_lines.append(f"**{char_name}:**")
            transcript_lines.append("")
            transcript_lines.append(reply)
            transcript_lines.append("")

            if phase == "probe":
                snap = _snapshot_all(loop, source)
                snap["probe"] = {
                    "id": tid,
                    "axes": turn.get("axes"),
                    "intent": turn.get("intent"),
                    "ground_truth_sources": turn.get("ground_truth_sources"),
                    "user_text": text,
                    "agent_reply": reply,
                }
                with open(snapshots_dir / f"{tid}.json", "w") as f:
                    json.dump(snap, f, indent=2, default=str)
                transcript_lines.append(f"_(state snapshot: snapshots/{tid}.json)_")
                transcript_lines.append("")
    finally:
        # Cleanly shut down the post-turn executor and persist final
        # state so the world dir is complete on disk.
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning(f"executor shutdown failed: {e}")
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning(f"final persist failed: {e}")

    transcript_path = output_dir / "transcript.md"
    transcript_path.write_text("\n".join(transcript_lines))
    print(f"\nTranscript: {transcript_path}", flush=True)
    print(f"Snapshots:  {snapshots_dir}", flush=True)
    print(f"World dir:  scenarios/{world_name}/", flush=True)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Drive the trace-grounded introspective fidelity benchmark "
                    "in-process against a chat scenario.")
    parser.add_argument(
        "--scenario", type=Path, required=True,
        help="Path to a chat-mode scenario YAML (e.g. "
             "scenarios/jill-benchmark-chat.yaml).")
    parser.add_argument(
        "--primer", type=Path, default=HERE / "primer.yaml",
        help="Path to primer YAML. Defaults to bench/introspective_fidelity/primer.yaml.")
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Where to write transcript.md and snapshots/. Defaults to "
             "bench/introspective_fidelity/results/<timestamp>_<scenario_stem>.")
    parser.add_argument(
        "--world-name", type=str, default=None,
        help="Override the world_name (and thus the on-disk state directory) "
             "used by the chat loop. Default behavior generates a fresh "
             "timestamped name per run. Pass an existing world_name to "
             "resume / probe an existing populated baseline (recommended: "
             "copy the baseline dir first so the test doesn't mutate it).")
    args = parser.parse_args()

    scenario = args.scenario.resolve()
    primer = args.primer.resolve()
    if args.output_dir:
        out = args.output_dir.resolve()
    else:
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H-%M-%SZ")
        out = HERE / "results" / f"{stamp}_{scenario.stem}"

    run_benchmark(scenario_path=scenario, primer_path=primer, output_dir=out,
                  world_name=args.world_name)


if __name__ == "__main__":
    main()
