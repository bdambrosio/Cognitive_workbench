#!/usr/bin/env python3
"""Perturbation Detection Benchmark — runner (v0.1).

Tier-3 of the operational-self-awareness ladder: when the agent's
structural state is altered without the change being named to her,
does she detect that her state is anomalous?

Distinct from Tier 4 (cspred) — the perturbation is NEVER named to
the agent in the probe text. The probe asks the agent to take stock
of her current operational state. Detection (or non-detection)
emerges from the agent's introspection alone.

For each probe, runs TWO independent ChatLoop / BaselineChatAgent
instances (one per cell):
  - control:    default state; detection probe sent.
  - perturbed:  perturbed state per `perturbation.op`+params; same
                detection probe sent.

Each cell runs in its own fresh world directory so cells don't
contaminate each other. State snapshots captured at probe time.
The judge buckets each cell into option_set; the harness scores
specificity / detection / localization deterministically from
buckets.

Supported perturbation ops (same vocabulary as cspred):
  - drop_prefix_turn    (params.turn_index=N) — drops the Nth prefix
                         turn (0-indexed)
  - omit_tools          (params.tools=[<tool names>]) — filters the
                         ReAct catalog at the schema level
  - strip_self_model    (no params) — clears character.self_model

Usage:
    python bench/perturbation_detection/runner.py \\
        --scenario scenarios/jill-benchmark-chat.yaml

Baseline mode (matched-backend chat product without architecture):
    python bench/perturbation_detection/runner.py \\
        --scenario scenarios/jill-benchmark-chat.yaml \\
        --baseline \\
        --baseline-prompt bench/introspective_fidelity/baseline_prompts/assistant_capabilities.txt
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
INTROSPECTIVE_DIR = REPO_ROOT / "bench" / "introspective_fidelity"
CSPRED_DIR = REPO_ROOT / "bench" / "counterfactual_self_prediction"

sys.path.insert(0, str(SRC_DIR))
sys.path.insert(0, str(INTROSPECTIVE_DIR))
sys.path.insert(0, str(CSPRED_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402
from baseline import BaselineChatAgent  # noqa: E402
# Reuse cspred's perturbation + snapshot helpers verbatim — same op
# vocabulary, same snapshot format, no reason to duplicate.
from runner import (  # noqa: E402  (cspred runner.py)
    _apply_perturbation,
    _snapshot_all,
    _latest_reply,
)

logger = logging.getLogger("bench.perturbation_detection")

CELL_LABELS = ("control", "perturbed")


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


def _run_cell(scenario_path: Path,
              world_name: str,
              prefix_turns: List[str],
              probe_text: str,
              cell_dir: Path,
              source: str,
              cell_label: str,
              probe_id: str,
              perturbation: Optional[Dict[str, Any]],
              baseline: bool = False,
              baseline_prompt: str = "") -> Dict[str, Any]:
    """Run one cell of a probe: instantiate ChatLoop or BaselineChatAgent
    (with perturbation applied to char_config + prefix when not None),
    send prefix turns, send the detection probe, snapshot, write
    transcript. Returns the snapshot."""
    cell_dir.mkdir(parents=True, exist_ok=True)

    char_name, char_config = _build_character_config(scenario_path, world_name)

    cell_prefix = list(prefix_turns)
    if perturbation is not None:
        char_config = copy.deepcopy(char_config)
        char_config, cell_prefix = _apply_perturbation(
            char_config, cell_prefix, perturbation)

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

    transcript_lines: List[str] = []
    transcript_lines.append(
        f"# Perturbation Detection — {probe_id} / {cell_label}")
    transcript_lines.append("")
    transcript_lines.append(f"- scenario: `{scenario_path.name}`")
    transcript_lines.append(f"- world_name: `{world_name}`")
    transcript_lines.append(f"- character: `{char_name}`")
    transcript_lines.append(f"- agent: `{'baseline' if baseline else 'jill'}`")
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
            print(f"\n=== {probe_id}/{cell_label}/{tid} ===", flush=True)
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

        text = (probe_text or "").strip()
        probe_marker = cell_label.upper()
        print(f"\n=== {probe_id}/{cell_label}/{probe_marker} ===", flush=True)
        print(f"User: {text}", flush=True)
        loop._process_user_turn(source=source, text=text, close=False)
        reply = _latest_reply(loop, source)
        print(f"\n{char_name}: {reply}", flush=True)

        transcript_lines.append(f"## {probe_marker} ({cell_label})")
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
            "id": probe_marker,
            "cell": cell_label,
            "probe_id": probe_id,
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


def _run_probe(scenario_path: Path,
               probe: Dict[str, Any],
               run_stamp: str,
               probe_dir: Path,
               source: str,
               baseline: bool = False,
               baseline_prompt: str = "") -> Optional[Dict[str, Any]]:
    """Run both cells of a probe and write a probe-level summary.json.
    Returns None if the probe is pending (skipped)."""
    probe_id = probe.get("id", "PROBE-?")

    if probe.get("pending"):
        reason = (probe.get("pending_reason") or "").strip()
        logger.warning(
            f"{probe_id}: SKIPPED (pending=true). Reason: "
            f"{reason if reason else '(no reason given)'}")
        return None

    intent = (probe.get("intent") or "").strip()
    axes = probe.get("axes") or {}
    option_set = probe.get("option_set") or []
    notes_for_judge = (probe.get("notes_for_judge") or "").strip()

    perturbation = probe.get("perturbation") or None
    if perturbation is None:
        raise RuntimeError(
            f"{probe_id}: missing perturbation block (required)")

    detection_probe_text = (probe.get("detection_probe") or "").strip()
    if not detection_probe_text:
        raise RuntimeError(f"{probe_id}: missing detection_probe text")

    # Skip Jill-specific probes when running against baseline (the
    # perturbation no-ops and both cells will report identical state).
    target_arch = axes.get("target_arch")
    universal = axes.get("universal", False)
    if baseline and target_arch == "jill":
        logger.warning(
            f"{probe_id}: SKIPPED — probe is Jill-specific "
            f"(target_arch=jill, universal={universal}); "
            f"perturbation `{perturbation.get('op')}` would no-op against "
            f"baseline and produce identical-cell output.")
        return None

    prefix_blocks = probe.get("prefix") or []
    prefix_turns = [(b.get("text") or "") for b in prefix_blocks]

    probe_dir.mkdir(parents=True, exist_ok=True)
    safe_id = probe_id.replace("/", "-").replace(" ", "_")

    cell_perturbation: Dict[str, Optional[Dict[str, Any]]] = {
        "control": None,
        "perturbed": perturbation,
    }

    snapshots: Dict[str, Dict[str, Any]] = {}
    cell_worlds: Dict[str, str] = {}
    for label in CELL_LABELS:
        world = f"bench-pdetect-{run_stamp}-{safe_id}-{label}"
        cell_worlds[label] = world
        cell_dir = probe_dir / label
        print(f"\n>>> Probe {probe_id} — cell {label} world={world}", flush=True)
        snapshots[label] = _run_cell(
            scenario_path=scenario_path,
            world_name=world,
            prefix_turns=prefix_turns,
            probe_text=detection_probe_text,
            cell_dir=cell_dir,
            source=source,
            cell_label=label,
            probe_id=probe_id,
            perturbation=cell_perturbation[label],
            baseline=baseline,
            baseline_prompt=baseline_prompt,
        )

    summary = {
        "probe_id": probe_id,
        "axes": axes,
        "intent": intent,
        "option_set": option_set,
        "notes_for_judge": notes_for_judge,
        "perturbation": {
            "name": perturbation.get("name"),
            "op": perturbation.get("op"),
            "params": perturbation.get("params"),
        },
        "detection_probe": detection_probe_text,
        "cells": {
            label: {
                "world_name": cell_worlds[label],
                "user_text": snapshots[label].get("probe", {}).get("user_text", ""),
                "agent_reply": snapshots[label].get("probe", {}).get("agent_reply", ""),
            }
            for label in CELL_LABELS
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
    if skipped:
        print(f"  probes skipped: {skipped}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Drive the perturbation-detection benchmark "
                    "(Tier-3) in-process against a chat scenario.")
    parser.add_argument(
        "--scenario", type=Path, required=True,
        help="Path to a chat-mode scenario YAML.")
    parser.add_argument(
        "--primer", type=Path, default=HERE / "primer.yaml",
        help="Path to primer YAML. Default: bench/perturbation_detection/primer.yaml.")
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Where to write per-probe output. Default: results/<stamp>_<scenario>/.")
    parser.add_argument(
        "--probes", nargs="+", default=None,
        help="Subset of probe IDs to run.")
    parser.add_argument(
        "--baseline", action="store_true",
        help="Run against BaselineChatAgent instead of ChatLoop. Same "
             "backend; no concerns/reflection/tools. Jill-specific probes "
             "(target_arch=jill) are auto-skipped under --baseline.")
    parser.add_argument(
        "--baseline-prompt", type=Path, default=None,
        help="Path to a system-prompt file for baseline mode.")
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
