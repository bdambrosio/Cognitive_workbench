#!/usr/bin/env python3
"""Counterfactual Self-Prediction Benchmark — runner (v0.1).

Tier-4 benchmark for the operational-self-awareness ladder. Drives, for
each pair in primer.yaml, two independent ChatLoop instances:
  - predict-arm: shared prefix turns + predict_probe → records prediction
  - enact-arm:   shared prefix turns + enact_stimulus → records actual
                 behavior

Each arm runs in its own fresh world directory (per-run timestamp + pair
id + arm tag) so the arms don't contaminate each other. State snapshots
are captured in both arms at the divergence turn for the judge to score.

Usage:
    python bench/counterfactual_self_prediction/runner.py \\
        --scenario scenarios/jill-benchmark-chat.yaml

The runner reuses the snapshot helpers from
bench/introspective_fidelity/runner.py in spirit (copy, not import) — once
both benches are stable we can extract a shared bench/_chat_harness.py.
For now duplication beats premature abstraction.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"

sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("bench.counterfactual_self_prediction")


# ---------------------------------------------------------------------------
# YAML loading + scenario config
# ---------------------------------------------------------------------------

def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> tuple[str, dict]:
    """Load scenario YAML and return (character_name, merged_config) for
    the single chat character. Replicates launcher.parse_characters' merge
    so per-character llm_config wins over top-level, then forces
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
# Snapshots — copied from bench/introspective_fidelity/runner.py
# ---------------------------------------------------------------------------

def _snapshot_concerns(loop: ChatLoop) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for cid in (getattr(loop, "_user_concerns_collection_id", None),
                getattr(loop, "_agent_concerns_collection_id", None)):
        if not cid:
            continue
        coll = loop.resource_manager.get_resource(cid)
        if not coll:
            continue
        note_ids = (coll.get("properties") or {}).get("content", []) or []
        for nid in note_ids:
            note = loop.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get("properties") or {}
            kind = props.get("kind", "concern")
            weight_src = (props.get("activation") if kind == "agent_concern"
                          else props.get("strength"))
            cadence = (loop._resolve_rhythm_hours(props)
                       if hasattr(loop, "_resolve_rhythm_hours")
                       else props.get("rhythm_hours"))
            out.append({
                "note_id": nid,
                "text": props.get("content", ""),
                "kind": kind,
                "category": kind,
                "status": props.get("status", "active"),
                "provenance": props.get("provenance"),
                "seed": bool(props.get("seed", False)),
                "cadence_hours": cadence,
                "rhythm_hours": cadence,
                "instruction": props.get("instruction"),
                "weight": float(weight_src or 0.0),
                "created_at": props.get("created_at"),
                "last_engaged_at": props.get("last_engaged_at"),
                "last_acted_at": (props.get("last_fired_at")
                                  or props.get("last_acted_at")),
            })
    return out


def _snapshot_reasoning_history(loop: "Any") -> List[Dict[str, Any]]:
    try:
        mem_dir_fn = getattr(loop, "_memory_dir", None)
        if mem_dir_fn is None:
            return []
        path = mem_dir_fn() / "reasoning_trace.jsonl"
    except Exception as e:
        logger.warning(f"reasoning_history snapshot path resolution failed: {e}")
        return []
    if not path.is_file():
        return []
    out: List[Dict[str, Any]] = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                s = line.strip()
                if not s:
                    continue
                try:
                    out.append(json.loads(s))
                except json.JSONDecodeError as e:
                    logger.warning(f"reasoning_history line skipped (bad JSON): {e}")
                    continue
    except Exception as e:
        logger.warning(f"reasoning_history read failed: {e}")
        return []
    return out


def _snapshot_discourse_and_companion(loop: ChatLoop, source: str
                                      ) -> Dict[str, str]:
    return {
        "discourse": loop._discourse_state.get(source, ""),
        "companion": loop._companion_state.get(source, ""),
    }


def _snapshot_conversation_history(loop: "Any", source: str
                                   ) -> List[Dict[str, str]]:
    try:
        store = getattr(loop, "store", None)
        if store is None:
            return []
        turns = store.get_recent_turns(source, limit=999, scope="all") or []
    except Exception as e:
        logger.warning(f"conversation_history snapshot failed: {e}")
        return []
    out: List[Dict[str, str]] = []
    for t in turns:
        out.append({
            "direction": str(t.get("direction", "")),
            "text": str(t.get("text", "")),
        })
    return out


def _snapshot_all(loop: ChatLoop, source: str) -> Dict[str, Any]:
    return {
        "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "concerns": _snapshot_concerns(loop),
        "reasoning_history": _snapshot_reasoning_history(loop),
        "discourse_and_companion": _snapshot_discourse_and_companion(loop, source),
        "conversation_history": _snapshot_conversation_history(loop, source),
    }


def _latest_reply(loop: ChatLoop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


# ---------------------------------------------------------------------------
# Single-arm driver
# ---------------------------------------------------------------------------

def _run_arm(scenario_path: Path,
             world_name: str,
             prefix_turns: List[str],
             divergence_turn: str,
             arm_dir: Path,
             source: str,
             arm_label: str,
             pair_id: str,
             divergence_id: str) -> Dict[str, Any]:
    """Run one arm: instantiate ChatLoop, send prefix turns, send the
    divergence turn, snapshot at divergence-time, write transcript.

    Returns the divergence-turn snapshot (with `probe` block populated)
    so the caller can persist it where convenient."""
    arm_dir.mkdir(parents=True, exist_ok=True)

    char_name, char_config = _build_character_config(scenario_path, world_name)
    if not char_config.get("chat", {}).get("benchmark_mode"):
        logger.warning(
            "scenario does not set chat.benchmark_mode=true; reflection runs "
            "on the background executor and the divergence-time snapshot "
            "may miss in-flight writes")

    loop = ChatLoop(character_name=char_name, character_config=char_config)

    transcript_lines: List[str] = []
    transcript_lines.append(f"# Counterfactual Self-Prediction — {pair_id} / {arm_label}")
    transcript_lines.append("")
    transcript_lines.append(f"- scenario: `{scenario_path.name}`")
    transcript_lines.append(f"- world_name: `{world_name}`")
    transcript_lines.append(f"- character: `{char_name}`")
    transcript_lines.append(f"- backend: `{(char_config.get('llm_config') or {}).get('server', '?')}` "
                            f"`{(char_config.get('llm_config') or {}).get('model', '')}`")
    transcript_lines.append(f"- arm: `{arm_label}`")
    transcript_lines.append(f"- started: {datetime.datetime.now(datetime.timezone.utc).isoformat()}")
    transcript_lines.append("")

    snapshot: Dict[str, Any] = {}
    try:
        for i, text in enumerate(prefix_turns, start=1):
            text = (text or "").strip()
            if not text:
                continue
            tid = f"PREFIX-{i:02d}"
            print(f"\n=== {pair_id}/{arm_label}/{tid} ===", flush=True)
            print(f"User: {text}", flush=True)
            loop._process_user_turn(source=source, text=text, close=False)
            reply = _latest_reply(loop, source)
            print(f"\n{char_name}: {reply}", flush=True)

            transcript_lines.append(f"## {tid} (prefix)")
            transcript_lines.append("")
            transcript_lines.append("**User:**")
            transcript_lines.append("")
            transcript_lines.append(text)
            transcript_lines.append("")
            transcript_lines.append(f"**{char_name}:**")
            transcript_lines.append("")
            transcript_lines.append(reply)
            transcript_lines.append("")

        # Divergence turn — predict_probe or enact_stimulus.
        text = (divergence_turn or "").strip()
        print(f"\n=== {pair_id}/{arm_label}/{divergence_id} ===", flush=True)
        print(f"User: {text}", flush=True)
        loop._process_user_turn(source=source, text=text, close=False)
        reply = _latest_reply(loop, source)
        print(f"\n{char_name}: {reply}", flush=True)

        transcript_lines.append(f"## {divergence_id} ({arm_label})")
        transcript_lines.append("")
        transcript_lines.append("**User:**")
        transcript_lines.append("")
        transcript_lines.append(text)
        transcript_lines.append("")
        transcript_lines.append(f"**{char_name}:**")
        transcript_lines.append("")
        transcript_lines.append(reply)
        transcript_lines.append("")

        snapshot = _snapshot_all(loop, source)
        snapshot["probe"] = {
            "id": divergence_id,
            "arm": arm_label,
            "pair_id": pair_id,
            "user_text": text,
            "agent_reply": reply,
        }
        with open(arm_dir / "snapshot.json", "w") as f:
            json.dump(snapshot, f, indent=2, default=str)
        transcript_lines.append("_(state snapshot: snapshot.json)_")
        transcript_lines.append("")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning(f"executor shutdown failed: {e}")
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning(f"final persist failed: {e}")

    (arm_dir / "transcript.md").write_text("\n".join(transcript_lines))
    return snapshot


# ---------------------------------------------------------------------------
# Pair driver
# ---------------------------------------------------------------------------

def _run_pair(scenario_path: Path,
              pair: Dict[str, Any],
              run_stamp: str,
              pair_dir: Path,
              source: str) -> Dict[str, Any]:
    """Run both arms of a pair and write a pair-level summary.json
    convenient for the judge."""
    pair_id = pair.get("id", "PAIR-?")
    intent = (pair.get("intent") or "").strip()
    axes = pair.get("axes")
    option_set = pair.get("option_set") or []
    notes_for_judge = (pair.get("notes_for_judge") or "").strip()

    prefix_blocks = pair.get("prefix") or []
    prefix_turns = [(b.get("text") or "") for b in prefix_blocks]
    predict_probe = pair.get("predict_probe") or ""
    enact_stimulus = pair.get("enact_stimulus") or ""
    if not predict_probe.strip() or not enact_stimulus.strip():
        raise RuntimeError(
            f"{pair_id}: both predict_probe and enact_stimulus are required"
        )

    pair_dir.mkdir(parents=True, exist_ok=True)
    predict_dir = pair_dir / "predict"
    enact_dir = pair_dir / "enact"

    safe_pair = pair_id.replace("/", "-").replace(" ", "_")
    predict_world = f"bench-cspred-{run_stamp}-{safe_pair}-predict"
    enact_world = f"bench-cspred-{run_stamp}-{safe_pair}-enact"

    print(f"\n>>> Pair {pair_id} — predict-arm world={predict_world}", flush=True)
    predict_snap = _run_arm(
        scenario_path=scenario_path,
        world_name=predict_world,
        prefix_turns=prefix_turns,
        divergence_turn=predict_probe,
        arm_dir=predict_dir,
        source=source,
        arm_label="predict",
        pair_id=pair_id,
        divergence_id="PRED",
    )

    print(f"\n>>> Pair {pair_id} — enact-arm world={enact_world}", flush=True)
    enact_snap = _run_arm(
        scenario_path=scenario_path,
        world_name=enact_world,
        prefix_turns=prefix_turns,
        divergence_turn=enact_stimulus,
        arm_dir=enact_dir,
        source=source,
        arm_label="enact",
        pair_id=pair_id,
        divergence_id="ENACT",
    )

    summary = {
        "pair_id": pair_id,
        "axes": axes,
        "intent": intent,
        "option_set": option_set,
        "notes_for_judge": notes_for_judge,
        "predict": {
            "world_name": predict_world,
            "user_text": predict_snap.get("probe", {}).get("user_text", ""),
            "agent_reply": predict_snap.get("probe", {}).get("agent_reply", ""),
        },
        "enact": {
            "world_name": enact_world,
            "user_text": enact_snap.get("probe", {}).get("user_text", ""),
            "agent_reply": enact_snap.get("probe", {}).get("agent_reply", ""),
        },
    }
    with open(pair_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    return summary


# ---------------------------------------------------------------------------
# Top-level driver
# ---------------------------------------------------------------------------

def run_benchmark(scenario_path: Path, primer_path: Path, output_dir: Path,
                  source: str = "User",
                  only_pairs: Optional[List[str]] = None) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    primer = _load_yaml(primer_path)
    pairs = primer.get("pairs") or []
    if not pairs:
        raise RuntimeError(f"no pairs found in {primer_path}")

    if only_pairs:
        wanted = set(only_pairs)
        pairs = [p for p in pairs if p.get("id") in wanted]
        if not pairs:
            raise RuntimeError(
                f"--pairs filter {only_pairs} matched none of the primer's pairs"
            )

    run_stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    summaries: List[Dict[str, Any]] = []
    for pair in pairs:
        pair_id = pair.get("id", "PAIR-?")
        pair_dir = output_dir / pair_id.replace("/", "-").replace(" ", "_")
        summary = _run_pair(scenario_path, pair, run_stamp, pair_dir, source)
        summaries.append(summary)

    index = {
        "run_stamp": run_stamp,
        "scenario": scenario_path.name,
        "primer": primer_path.name,
        "n_pairs": len(summaries),
        "pairs": [s["pair_id"] for s in summaries],
    }
    with open(output_dir / "run_index.json", "w") as f:
        json.dump(index, f, indent=2, default=str)
    print(f"\nRun complete: {output_dir}")
    print(f"  pairs: {[s['pair_id'] for s in summaries]}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Drive the counterfactual-self-prediction benchmark "
                    "in-process against a chat scenario.")
    parser.add_argument(
        "--scenario", type=Path, required=True,
        help="Path to a chat-mode scenario YAML (e.g. "
             "scenarios/jill-benchmark-chat.yaml).")
    parser.add_argument(
        "--primer", type=Path, default=HERE / "primer.yaml",
        help="Path to primer YAML. Defaults to "
             "bench/counterfactual_self_prediction/primer.yaml.")
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Where to write per-pair output. Defaults to "
             "bench/counterfactual_self_prediction/results/<timestamp>_<scenario_stem>.")
    parser.add_argument(
        "--pairs", nargs="+", default=None,
        help="Subset of pair IDs to run (e.g. --pairs PAIR-01 PAIR-03). "
             "Default: all pairs in the primer.")
    parser.add_argument(
        "--runs", type=int, default=1,
        help="Number of independent runs of the full pair set. N=1 (default) "
             "produces the flat results/<stamp>_<scenario>/ layout. N>1 "
             "wraps each run in results/<stamp>_<scenario>_xN/run-NN/ and "
             "writes a parent run_index.json — judge.py auto-detects the "
             "multi-run layout and aggregates.")
    args = parser.parse_args()

    if args.runs < 1:
        parser.error("--runs must be >= 1")

    scenario = args.scenario.resolve()
    primer = args.primer.resolve()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    if args.runs == 1:
        if args.output_dir:
            out = args.output_dir.resolve()
        else:
            out = HERE / "results" / f"{stamp}_{scenario.stem}"
        run_benchmark(scenario_path=scenario, primer_path=primer,
                      output_dir=out, only_pairs=args.pairs)
        return

    # Multi-run: parent dir contains run-01/, run-02/, … and a parent
    # run_index.json. judge.py walks this structure when scoring.
    if args.output_dir:
        parent = args.output_dir.resolve()
    else:
        parent = HERE / "results" / f"{stamp}_{scenario.stem}_x{args.runs}"
    parent.mkdir(parents=True, exist_ok=True)

    run_dirs: List[str] = []
    for i in range(1, args.runs + 1):
        run_dir = parent / f"run-{i:02d}"
        print(f"\n========== run {i}/{args.runs}  ({run_dir.name}) ==========",
              flush=True)
        run_benchmark(scenario_path=scenario, primer_path=primer,
                      output_dir=run_dir, only_pairs=args.pairs)
        run_dirs.append(run_dir.name)

    parent_index = {
        "parent_stamp": stamp,
        "scenario": scenario.name,
        "primer": primer.name,
        "n_runs": args.runs,
        "runs": run_dirs,
        "only_pairs": args.pairs,
    }
    with open(parent / "run_index.json", "w") as f:
        json.dump(parent_index, f, indent=2, default=str)
    print(f"\nMulti-run complete: {parent}")
    print(f"  runs: {run_dirs}")


if __name__ == "__main__":
    main()
