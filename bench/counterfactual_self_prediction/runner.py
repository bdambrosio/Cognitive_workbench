#!/usr/bin/env python3
"""Counterfactual Self-Prediction Benchmark — runner (v0.3).

Tier-4 benchmark for the operational-self-awareness ladder. Drives, for
each pair in primer.yaml, FOUR independent ChatLoop instances (one per
cell):
  - predict_base: default state; probe asks the question.
  - predict_cf:   default state; probe wraps the question with the
                  perturbation description.
  - enact_base:   default state; stimulus is the direct request.
  - enact_cf:     perturbed state per `perturbation.op`+`params`;
                  stimulus identical to enact_base.

Each cell runs in its own fresh world directory (per-run timestamp +
pair id + cell label) so cells don't contaminate each other. State
snapshots are captured at probe/stimulus time for the judge to score.
The judge computes Δ-match and Δ-specificity over the four cells; see
README and primer.yaml header.

Supported perturbation ops:
  - omit_tools         (params.tools=[<tool names>]) — filters the
                        ReAct catalog at the schema level via
                        chat.omitted_tools.
  - drop_prefix_turn   (params.turn_index=N) — drops the Nth prefix
                        turn (0-indexed).
  - strip_self_model   (no params) — clears character.self_model field
                        before ChatLoop construction.
  - disable_concern_pathway / suppress_companion_model — pending; pair
                        should be marked pending:true in primer.yaml
                        until the runtime hook lands. Runner skips
                        pending pairs with a warning.

Usage:
    python bench/counterfactual_self_prediction/runner.py \\
        --scenario scenarios/jill-benchmark-chat.yaml

The runner reuses the snapshot helpers from
bench/introspective_fidelity/runner.py in spirit (copy, not import) —
once both benches are stable we can extract a shared
bench/_chat_harness.py.
"""

from __future__ import annotations

import argparse
import copy
import datetime
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"

sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("bench.counterfactual_self_prediction")


CELL_LABELS = ("predict_base", "predict_cf", "enact_base", "enact_cf")


# ---------------------------------------------------------------------------
# YAML loading + scenario config
# ---------------------------------------------------------------------------

def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> Tuple[str, dict]:
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
# Perturbations
# ---------------------------------------------------------------------------

def _apply_perturbation(char_config: dict,
                        prefix_turns: List[str],
                        perturbation: Dict[str, Any]
                        ) -> Tuple[dict, List[str]]:
    """Return (modified_char_config, modified_prefix_turns) reflecting
    the perturbation's op + params. char_config is mutated in place
    (caller is expected to pass a deepcopy when isolation matters)."""
    op = perturbation.get("op")
    params = perturbation.get("params") or {}

    if op == "omit_tools":
        tools = list(params.get("tools") or [])
        if not tools:
            raise ValueError(
                f"omit_tools requires non-empty params.tools (got {params!r})")
        chat_block = dict(char_config.get("chat") or {})
        existing = list(chat_block.get("omitted_tools") or [])
        chat_block["omitted_tools"] = existing + [
            t for t in tools if t not in existing]
        char_config["chat"] = chat_block
        return char_config, prefix_turns

    if op == "drop_prefix_turn":
        idx = params.get("turn_index")
        if not isinstance(idx, int):
            raise ValueError(
                f"drop_prefix_turn requires int params.turn_index "
                f"(got {params!r})")
        if idx < 0 or idx >= len(prefix_turns):
            raise ValueError(
                f"drop_prefix_turn: turn_index={idx} out of range "
                f"(have {len(prefix_turns)} prefix turns)")
        new_prefix = list(prefix_turns)
        new_prefix.pop(idx)
        return char_config, new_prefix

    if op == "strip_self_model":
        char_config["self_model"] = ""
        return char_config, prefix_turns

    if op in ("disable_concern_pathway", "suppress_companion_model"):
        raise NotImplementedError(
            f"perturbation op '{op}' requires a runtime hook that has not "
            f"yet been implemented; mark the pair pending:true in "
            f"primer.yaml until the hook lands")

    raise ValueError(f"unknown perturbation op: {op!r}")


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
# Single-cell driver
# ---------------------------------------------------------------------------

def _run_cell(scenario_path: Path,
              world_name: str,
              prefix_turns: List[str],
              probe_or_stimulus: str,
              cell_dir: Path,
              source: str,
              cell_label: str,
              pair_id: str,
              perturbation: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Run one cell of a pair: instantiate ChatLoop (with perturbation
    applied to char_config + prefix when perturbation is not None), send
    prefix turns, send the probe/stimulus, snapshot at probe time, write
    transcript.

    Returns the snapshot (with `probe` block populated) so the caller
    can persist it where convenient."""
    cell_dir.mkdir(parents=True, exist_ok=True)

    char_name, char_config = _build_character_config(scenario_path, world_name)

    cell_prefix = list(prefix_turns)
    if perturbation is not None:
        # parse_characters returns a fresh tree per call, but deepcopy
        # to be defensive against future caching.
        char_config = copy.deepcopy(char_config)
        char_config, cell_prefix = _apply_perturbation(
            char_config, cell_prefix, perturbation)

    if not (char_config.get("chat") or {}).get("benchmark_mode"):
        logger.warning(
            "scenario does not set chat.benchmark_mode=true; reflection runs "
            "on the background executor and the divergence-time snapshot "
            "may miss in-flight writes")

    loop = ChatLoop(character_name=char_name, character_config=char_config)

    transcript_lines: List[str] = []
    transcript_lines.append(
        f"# Counterfactual Self-Prediction — {pair_id} / {cell_label}")
    transcript_lines.append("")
    transcript_lines.append(f"- scenario: `{scenario_path.name}`")
    transcript_lines.append(f"- world_name: `{world_name}`")
    transcript_lines.append(f"- character: `{char_name}`")
    transcript_lines.append(
        f"- backend: `{(char_config.get('llm_config') or {}).get('server', '?')}` "
        f"`{(char_config.get('llm_config') or {}).get('model', '')}`")
    transcript_lines.append(f"- cell: `{cell_label}`")
    if perturbation is not None:
        transcript_lines.append(
            f"- perturbation: `{perturbation.get('name')}` "
            f"(op=`{perturbation.get('op')}`, params=`{perturbation.get('params')}`)")
    transcript_lines.append(
        f"- started: {datetime.datetime.now(datetime.timezone.utc).isoformat()}")
    transcript_lines.append("")

    snapshot: Dict[str, Any] = {}
    try:
        for i, text in enumerate(cell_prefix, start=1):
            text = (text or "").strip()
            if not text:
                continue
            tid = f"PREFIX-{i:02d}"
            print(f"\n=== {pair_id}/{cell_label}/{tid} ===", flush=True)
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

        # Probe / stimulus turn.
        text = (probe_or_stimulus or "").strip()
        probe_id = cell_label.upper()
        print(f"\n=== {pair_id}/{cell_label}/{probe_id} ===", flush=True)
        print(f"User: {text}", flush=True)
        loop._process_user_turn(source=source, text=text, close=False)
        reply = _latest_reply(loop, source)
        print(f"\n{char_name}: {reply}", flush=True)

        transcript_lines.append(f"## {probe_id} ({cell_label})")
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
            "id": probe_id,
            "cell": cell_label,
            "pair_id": pair_id,
            "user_text": text,
            "agent_reply": reply,
            "perturbation_applied": (
                None if perturbation is None
                else {"name": perturbation.get("name"),
                      "op": perturbation.get("op"),
                      "params": perturbation.get("params")}),
        }
        with open(cell_dir / "snapshot.json", "w") as f:
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

    (cell_dir / "transcript.md").write_text("\n".join(transcript_lines))
    return snapshot


# ---------------------------------------------------------------------------
# Pair driver
# ---------------------------------------------------------------------------

def _run_pair(scenario_path: Path,
              pair: Dict[str, Any],
              run_stamp: str,
              pair_dir: Path,
              source: str) -> Optional[Dict[str, Any]]:
    """Run all four cells of a pair and write a pair-level summary.json.
    Returns None if the pair is pending (skipped)."""
    pair_id = pair.get("id", "PAIR-?")

    if pair.get("pending"):
        reason = (pair.get("pending_reason") or "").strip()
        logger.warning(
            f"{pair_id}: SKIPPED (pending=true). Reason: "
            f"{reason if reason else '(no reason given)'}")
        return None

    intent = (pair.get("intent") or "").strip()
    axes = pair.get("axes")
    option_set = pair.get("option_set") or []
    notes_for_judge = (pair.get("notes_for_judge") or "").strip()

    perturbation = pair.get("perturbation") or None
    if perturbation is None:
        raise RuntimeError(
            f"{pair_id}: missing perturbation block (required in v0.3)")

    prefix_blocks = pair.get("prefix") or []
    prefix_turns = [(b.get("text") or "") for b in prefix_blocks]

    cells = pair.get("cells") or {}
    missing = [c for c in CELL_LABELS if c not in cells]
    if missing:
        raise RuntimeError(
            f"{pair_id}: missing cell(s) {missing}; v0.3 requires all four "
            f"of {CELL_LABELS}")

    pair_dir.mkdir(parents=True, exist_ok=True)
    safe_pair = pair_id.replace("/", "-").replace(" ", "_")

    cell_text: Dict[str, str] = {}
    for label in CELL_LABELS:
        spec = cells[label] or {}
        # predict cells use 'probe', enact cells use 'stimulus'.
        text = spec.get("probe") if label.startswith("predict") else spec.get("stimulus")
        if not (text or "").strip():
            raise RuntimeError(
                f"{pair_id}: cell {label} missing "
                f"{'probe' if label.startswith('predict') else 'stimulus'}")
        cell_text[label] = text

    # cf cells get the perturbation; base cells do not.
    cell_perturbation: Dict[str, Optional[Dict[str, Any]]] = {
        "predict_base": None,
        "predict_cf": None,         # perturbation lives in the question text, not state
        "enact_base": None,
        "enact_cf": perturbation,   # perturbation operative in runtime
    }

    snapshots: Dict[str, Dict[str, Any]] = {}
    cell_worlds: Dict[str, str] = {}
    for label in CELL_LABELS:
        world = f"bench-cspred-{run_stamp}-{safe_pair}-{label}"
        cell_worlds[label] = world
        cell_dir = pair_dir / label
        print(f"\n>>> Pair {pair_id} — cell {label} world={world}", flush=True)
        snapshots[label] = _run_cell(
            scenario_path=scenario_path,
            world_name=world,
            prefix_turns=prefix_turns,
            probe_or_stimulus=cell_text[label],
            cell_dir=cell_dir,
            source=source,
            cell_label=label,
            pair_id=pair_id,
            perturbation=cell_perturbation[label],
        )

    summary = {
        "pair_id": pair_id,
        "axes": axes,
        "intent": intent,
        "option_set": option_set,
        "notes_for_judge": notes_for_judge,
        "perturbation": {
            "name": perturbation.get("name"),
            "op": perturbation.get("op"),
            "params": perturbation.get("params"),
            "describe_in_predict_cf": perturbation.get("describe_in_predict_cf", ""),
        },
        "cells": {
            label: {
                "world_name": cell_worlds[label],
                "user_text": snapshots[label].get("probe", {}).get("user_text", ""),
                "agent_reply": snapshots[label].get("probe", {}).get("agent_reply", ""),
            }
            for label in CELL_LABELS
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
    skipped: List[str] = []
    for pair in pairs:
        pair_id = pair.get("id", "PAIR-?")
        pair_dir = output_dir / pair_id.replace("/", "-").replace(" ", "_")
        summary = _run_pair(scenario_path, pair, run_stamp, pair_dir, source)
        if summary is None:
            skipped.append(pair_id)
        else:
            summaries.append(summary)

    index = {
        "run_stamp": run_stamp,
        "scenario": scenario_path.name,
        "primer": primer_path.name,
        "n_pairs": len(summaries),
        "pairs": [s["pair_id"] for s in summaries],
        "skipped_pending": skipped,
    }
    with open(output_dir / "run_index.json", "w") as f:
        json.dump(index, f, indent=2, default=str)
    print(f"\nRun complete: {output_dir}")
    print(f"  pairs ran: {[s['pair_id'] for s in summaries]}")
    if skipped:
        print(f"  pairs skipped (pending): {skipped}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Drive the counterfactual-self-prediction benchmark "
                    "(v0.3 diff-in-diff) in-process against a chat scenario.")
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
             "Default: all pairs in the primer (pending pairs are still "
             "skipped within the filter).")
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
