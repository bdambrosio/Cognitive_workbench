#!/usr/bin/env python3
"""Behavioral Consistency Benchmark — runner (v0.1).

Tier-2 of the operational-self-awareness ladder: does the agent's
enacted behavior on a class of triggers match her own pre-session
prediction of her behavior on that class?

For each probe, runs TWO independent ChatLoop / BaselineChatAgent
instances:
  - prediction:  agent receives the self_prediction_probe; reply
                 contains a low/medium/high bucket commitment.
  - behavior:    agent receives N behavioral trigger turns
                 sequentially in one session; each reply captured.

Each cell runs in its own fresh world directory. State snapshots
captured after the prediction turn (one snapshot) and after the
final behavioral turn (one snapshot containing the full turn
sequence in conversation_history).

Usage:
    python bench/behavioral_consistency/runner.py \\
        --scenario scenarios/jill-benchmark-chat.yaml

Baseline mode:
    python bench/behavioral_consistency/runner.py \\
        --scenario scenarios/jill-benchmark-chat.yaml \\
        --baseline \\
        --baseline-prompt bench/introspective_fidelity/baseline_prompts/assistant_capabilities.txt
"""

from __future__ import annotations

import argparse
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
INTROSPECTIVE_DIR = REPO_ROOT / "bench" / "introspective_fidelity"
CSPRED_DIR = REPO_ROOT / "bench" / "counterfactual_self_prediction"

sys.path.insert(0, str(SRC_DIR))
sys.path.insert(0, str(INTROSPECTIVE_DIR))
sys.path.insert(0, str(CSPRED_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402
from baseline import BaselineChatAgent  # noqa: E402
from runner import _snapshot_all, _latest_reply  # noqa: E402  (cspred runner)

logger = logging.getLogger("bench.behavioral_consistency")

CELL_LABELS = ("prediction", "behavior")


def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> Tuple[str, dict]:
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


def _instantiate_loop(scenario_path: Path, world_name: str,
                      baseline: bool, baseline_prompt: str
                      ) -> Tuple[str, "Any"]:
    char_name, char_config = _build_character_config(scenario_path, world_name)
    if baseline:
        loop = BaselineChatAgent(
            llm_config=(char_config.get("llm_config") or {}),
            system_prompt=baseline_prompt,
            character_name=char_name,
        )
    else:
        if not (char_config.get("chat") or {}).get("benchmark_mode"):
            logger.warning(
                "scenario does not set chat.benchmark_mode=true; reflection "
                "runs on the background executor and probe-time snapshots "
                "may miss in-flight writes")
        loop = ChatLoop(character_name=char_name, character_config=char_config)
    return char_name, loop


def _send_turn(loop: "Any", source: str, text: str, char_name: str,
               turn_label: str, transcript_lines: List[str]) -> str:
    text = (text or "").strip()
    print(f"\n=== {turn_label} ===", flush=True)
    print(f"User: {text}", flush=True)
    loop._process_user_turn(source=source, text=text, close=False)
    reply = _latest_reply(loop, source)
    print(f"\n{char_name}: {reply}", flush=True)
    transcript_lines.append(f"## {turn_label}")
    transcript_lines.append("")
    transcript_lines.append("**User:**")
    transcript_lines.append("")
    transcript_lines.append(text)
    transcript_lines.append("")
    transcript_lines.append(f"**{char_name}:**")
    transcript_lines.append("")
    transcript_lines.append(reply)
    transcript_lines.append("")
    return reply


def _cleanup_loop(loop: "Any") -> None:
    try:
        loop._post_turn_executor.shutdown(wait=True)
    except Exception as e:
        logger.warning(f"executor shutdown failed: {e}")
    try:
        loop._persist_to_disk()
    except Exception as e:
        logger.warning(f"final persist failed: {e}")


def _run_prediction_cell(scenario_path: Path, world_name: str,
                         self_prediction_probe: str,
                         cell_dir: Path, source: str,
                         probe_id: str, baseline: bool,
                         baseline_prompt: str) -> Dict[str, Any]:
    """Run the prediction cell: one turn (the self_prediction_probe).
    Capture the reply + snapshot. Returns the snapshot."""
    cell_dir.mkdir(parents=True, exist_ok=True)
    char_name, loop = _instantiate_loop(
        scenario_path, world_name, baseline, baseline_prompt)

    transcript_lines: List[str] = []
    transcript_lines.append(f"# Behavioral Consistency — {probe_id} / prediction")
    transcript_lines.append("")
    transcript_lines.append(f"- scenario: `{scenario_path.name}`")
    transcript_lines.append(f"- world_name: `{world_name}`")
    transcript_lines.append(f"- character: `{char_name}`")
    transcript_lines.append(f"- agent: `{'baseline' if baseline else 'jill'}`")
    transcript_lines.append("")

    snapshot: Dict[str, Any] = {}
    try:
        reply = _send_turn(loop, source, self_prediction_probe, char_name,
                           f"{probe_id}/PREDICTION", transcript_lines)
        snapshot = _snapshot_all(loop, source)
        snapshot["probe"] = {
            "id": probe_id + "_prediction",
            "cell": "prediction",
            "user_text": self_prediction_probe.strip(),
            "agent_reply": reply,
        }
        with open(cell_dir / "snapshot.json", "w") as f:
            json.dump(snapshot, f, indent=2, default=str)
    finally:
        _cleanup_loop(loop)
    (cell_dir / "transcript.md").write_text("\n".join(transcript_lines))
    return snapshot


def _run_behavior_cell(scenario_path: Path, world_name: str,
                       behavioral_turns: List[Dict[str, str]],
                       cell_dir: Path, source: str,
                       probe_id: str, baseline: bool,
                       baseline_prompt: str) -> Dict[str, Any]:
    """Run the behavior cell: send each behavioral turn sequentially in
    one session. Capture each reply. Snapshot at the end of the
    session (with full conversation_history in the snapshot).
    Returns a structured snapshot with per-turn replies."""
    cell_dir.mkdir(parents=True, exist_ok=True)
    char_name, loop = _instantiate_loop(
        scenario_path, world_name, baseline, baseline_prompt)

    transcript_lines: List[str] = []
    transcript_lines.append(f"# Behavioral Consistency — {probe_id} / behavior")
    transcript_lines.append("")
    transcript_lines.append(f"- scenario: `{scenario_path.name}`")
    transcript_lines.append(f"- world_name: `{world_name}`")
    transcript_lines.append(f"- character: `{char_name}`")
    transcript_lines.append(f"- agent: `{'baseline' if baseline else 'jill'}`")
    transcript_lines.append("")

    per_turn: List[Dict[str, str]] = []
    snapshot: Dict[str, Any] = {}
    try:
        for t in behavioral_turns:
            tid = t.get("id", "B?")
            text = (t.get("text") or "").strip()
            reply = _send_turn(loop, source, text, char_name,
                               f"{probe_id}/{tid}", transcript_lines)
            per_turn.append({"id": tid, "user_text": text, "agent_reply": reply})
        snapshot = _snapshot_all(loop, source)
        snapshot["probe"] = {
            "id": probe_id + "_behavior",
            "cell": "behavior",
            "behavioral_turns": per_turn,
        }
        with open(cell_dir / "snapshot.json", "w") as f:
            json.dump(snapshot, f, indent=2, default=str)
    finally:
        _cleanup_loop(loop)
    (cell_dir / "transcript.md").write_text("\n".join(transcript_lines))
    return snapshot


def _run_probe(scenario_path: Path, probe: Dict[str, Any], run_stamp: str,
               probe_dir: Path, source: str,
               baseline: bool, baseline_prompt: str
               ) -> Optional[Dict[str, Any]]:
    """Run both cells of a probe and write probe-level summary.json."""
    probe_id = probe.get("id", "PROBE-?")
    if probe.get("pending"):
        logger.warning(f"{probe_id}: SKIPPED (pending=true)")
        return None

    intent = (probe.get("intent") or "").strip()
    axes = probe.get("axes") or {}
    self_prediction_probe = (probe.get("self_prediction_probe") or "").strip()
    self_prediction_buckets = probe.get("self_prediction_buckets") or {}
    behavioral_turns = probe.get("behavioral_turns") or []
    behavior_buckets = probe.get("behavior_buckets") or []
    fires_buckets = probe.get("fires_buckets") or []
    notes_for_judge = (probe.get("notes_for_judge") or "").strip()

    if not self_prediction_probe:
        raise RuntimeError(f"{probe_id}: missing self_prediction_probe")
    if not behavioral_turns:
        raise RuntimeError(f"{probe_id}: missing behavioral_turns")

    probe_dir.mkdir(parents=True, exist_ok=True)
    safe_id = probe_id.replace("/", "-").replace(" ", "_")

    # Prediction cell.
    pred_world = f"bench-bcons-{run_stamp}-{safe_id}-prediction"
    pred_dir = probe_dir / "prediction"
    print(f"\n>>> Probe {probe_id} — cell prediction world={pred_world}",
          flush=True)
    pred_snap = _run_prediction_cell(
        scenario_path, pred_world, self_prediction_probe,
        pred_dir, source, probe_id, baseline, baseline_prompt)

    # Behavior cell.
    beh_world = f"bench-bcons-{run_stamp}-{safe_id}-behavior"
    beh_dir = probe_dir / "behavior"
    print(f"\n>>> Probe {probe_id} — cell behavior world={beh_world}",
          flush=True)
    beh_snap = _run_behavior_cell(
        scenario_path, beh_world, behavioral_turns,
        beh_dir, source, probe_id, baseline, baseline_prompt)

    summary = {
        "probe_id": probe_id,
        "axes": axes,
        "intent": intent,
        "self_prediction_probe": self_prediction_probe,
        "self_prediction_buckets": self_prediction_buckets,
        "behavior_buckets": behavior_buckets,
        "fires_buckets": fires_buckets,
        "notes_for_judge": notes_for_judge,
        "cells": {
            "prediction": {
                "world_name": pred_world,
                "user_text": pred_snap.get("probe", {}).get("user_text", ""),
                "agent_reply": pred_snap.get("probe", {}).get("agent_reply", ""),
            },
            "behavior": {
                "world_name": beh_world,
                "behavioral_turns": (
                    beh_snap.get("probe", {}).get("behavioral_turns") or []),
            },
        },
    }
    with open(probe_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    return summary


def run_benchmark(scenario_path: Path, primer_path: Path, output_dir: Path,
                  source: str = "User",
                  only_probes: Optional[List[str]] = None,
                  baseline: bool = False,
                  baseline_prompt: str = "") -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    primer = _load_yaml(primer_path)
    probes = primer.get("probes") or []
    if not probes:
        raise RuntimeError(f"no probes found in {primer_path}")

    if only_probes:
        wanted = set(only_probes)
        probes = [p for p in probes if p.get("id") in wanted]
        if not probes:
            raise RuntimeError(
                f"--probes filter {only_probes} matched none of the primer's probes"
            )

    run_stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    summaries: List[Dict[str, Any]] = []
    skipped: List[str] = []
    for probe in probes:
        probe_id = probe.get("id", "PROBE-?")
        probe_dir = output_dir / probe_id.replace("/", "-").replace(" ", "_")
        summary = _run_probe(scenario_path, probe, run_stamp, probe_dir, source,
                             baseline=baseline, baseline_prompt=baseline_prompt)
        if summary is None:
            skipped.append(probe_id)
        else:
            summaries.append(summary)

    index = {
        "run_stamp": run_stamp,
        "scenario": scenario_path.name,
        "primer": primer_path.name,
        "agent": "baseline" if baseline else "jill",
        "n_probes": len(summaries),
        "probes": [s["probe_id"] for s in summaries],
        "skipped": skipped,
    }
    with open(output_dir / "run_index.json", "w") as f:
        json.dump(index, f, indent=2, default=str)
    print(f"\nRun complete: {output_dir}")
    print(f"  probes ran: {[s['probe_id'] for s in summaries]}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Drive the behavioral-consistency benchmark (Tier-2) "
                    "in-process against a chat scenario.")
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--primer", type=Path, default=HERE / "primer.yaml")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--probes", nargs="+", default=None)
    parser.add_argument("--baseline", action="store_true")
    parser.add_argument("--baseline-prompt", type=Path, default=None)
    args = parser.parse_args()

    scenario = args.scenario.resolve()
    primer = args.primer.resolve()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    baseline_prompt_text = ""
    if args.baseline_prompt:
        baseline_prompt_text = Path(args.baseline_prompt).read_text().strip()
    if args.baseline:
        logger.info(
            f"baseline mode: agent=BaselineChatAgent, "
            f"system_prompt="
            f"{'<empty>' if not baseline_prompt_text else f'{args.baseline_prompt}'}"
        )
    suffix_tag = "baseline_" if args.baseline else ""

    if args.output_dir:
        out = args.output_dir.resolve()
    else:
        out = HERE / "results" / f"{stamp}_{suffix_tag}{scenario.stem}"
    run_benchmark(scenario_path=scenario, primer_path=primer,
                  output_dir=out, only_probes=args.probes,
                  baseline=args.baseline,
                  baseline_prompt=baseline_prompt_text)


if __name__ == "__main__":
    main()
