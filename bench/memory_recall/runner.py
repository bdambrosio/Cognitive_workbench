#!/usr/bin/env python3
"""Memory Recall Benchmark — runner (v0.1).

Drives one ChatLoop per probe-subset YAML. For each subset: instantiate
ChatLoop with a fresh world_name, run all tells as user turns, then for
each probe run `/remember` (via loop._run_remember, the same code path
the CLI slash command hits) followed by an agent ask (via
_process_user_turn). Both readouts go into raw.jsonl with the same
probe_id; the judge scores them independently.

Resets between subsets are full ChatLoop teardown + reinstantiation
under a new world_name. Within a subset, probes run in the same
ChatLoop — agent asks DO mutate memory, so probe N+1 sees probe N's
exchange. Subset YAMLs are authored to keep within-subset probes
non-contaminating.

Usage:
    cd src
    python ../bench/memory_recall/runner.py \\
        --scenario ../scenarios/jill-benchmark-chat.yaml

    # subset filter (run only specific YAMLs):
    python ../bench/memory_recall/runner.py \\
        --scenario ../scenarios/jill-benchmark-chat.yaml \\
        --subsets 01-direct-recall 02-lateral-cue
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"

sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("bench.memory_recall")

SOURCE = "User"


# ---------------------------------------------------------------------------
# YAML loading
# ---------------------------------------------------------------------------

def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> tuple[str, dict]:
    """Mirror introspective_fidelity._build_character_config: load scenario,
    force world_config.world_name, return the single chat character's
    (name, merged_config)."""
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
            f"{len(chat_chars)}")
    return chat_chars[0]


# ---------------------------------------------------------------------------
# Reply capture
# ---------------------------------------------------------------------------

def _latest_reply(loop: ChatLoop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


# ---------------------------------------------------------------------------
# Subset discovery
# ---------------------------------------------------------------------------

def _discover_subsets(probes_dir: Path,
                      filter_ids: Optional[List[str]] = None
                      ) -> List[Dict[str, Any]]:
    """Load all YAMLs under probes_dir, optionally filtered by session_id.
    Returns list of subset dicts in filename order (lexicographic — the NN-
    prefix naming convention preserves intended order)."""
    if not probes_dir.is_dir():
        raise RuntimeError(f"probes dir not found: {probes_dir}")
    out: List[Dict[str, Any]] = []
    for path in sorted(probes_dir.glob("*.yaml")):
        data = _load_yaml(path)
        sid = data.get("session_id") or path.stem
        if filter_ids and sid not in filter_ids:
            continue
        data["_path"] = str(path)
        data["session_id"] = sid
        out.append(data)
    if filter_ids:
        unmatched = set(filter_ids) - {d["session_id"] for d in out}
        if unmatched:
            logger.warning(f"subset filter unmatched: {sorted(unmatched)}")
    return out


# ---------------------------------------------------------------------------
# Per-subset session
# ---------------------------------------------------------------------------

def _run_subset(subset: Dict[str, Any], scenario_path: Path,
                world_name: str, transcripts_dir: Path
                ) -> List[Dict[str, Any]]:
    """Run one subset: tells then probes, both readouts per probe.
    Returns list of per-probe records ready for raw.jsonl. Writes
    transcripts_dir/<session_id>.md as a side-effect."""
    sid = subset["session_id"]
    tells: List[str] = list(subset.get("tells") or [])
    probes: List[Dict[str, Any]] = list(subset.get("probes") or [])
    description = subset.get("description", "")
    status = subset.get("status", "v0.1")

    char_name, char_config = _build_character_config(scenario_path, world_name)
    if not char_config.get("chat", {}).get("benchmark_mode"):
        logger.warning(
            f"[{sid}] scenario does not set chat.benchmark_mode=true; "
            "reflection runs on the background executor and probe-time reads "
            "may miss in-flight writes")

    loop = ChatLoop(character_name=char_name, character_config=char_config)

    transcript_lines: List[str] = []
    transcript_lines.append(f"# Memory Recall — session `{sid}`")
    transcript_lines.append("")
    transcript_lines.append(f"- description: {description}")
    transcript_lines.append(f"- status: `{status}`")
    transcript_lines.append(f"- scenario: `{scenario_path.name}`")
    transcript_lines.append(f"- world_name: `{world_name}`")
    transcript_lines.append(f"- character: `{char_name}`")
    backend_label = (char_config.get("llm_config") or {}).get("server", "?")
    model_label = (char_config.get("llm_config") or {}).get("model", "")
    transcript_lines.append(f"- backend: `{backend_label}` `{model_label}`")
    transcript_lines.append("")

    records: List[Dict[str, Any]] = []

    try:
        # ------ Tells ------
        if tells:
            transcript_lines.append("## Tells")
            transcript_lines.append("")
        for i, tell in enumerate(tells, start=1):
            text = (tell or "").strip()
            if not text:
                continue
            print(f"\n[{sid}] tell {i}/{len(tells)}: {text}", flush=True)
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = _latest_reply(loop, SOURCE)
            print(f"[{sid}] {char_name}: {reply}", flush=True)
            transcript_lines.append(f"### Tell {i}")
            transcript_lines.append("")
            transcript_lines.append(f"**User:** {text}")
            transcript_lines.append("")
            transcript_lines.append(f"**{char_name}:** {reply}")
            transcript_lines.append("")

        # ------ Probes ------
        if probes:
            transcript_lines.append("## Probes")
            transcript_lines.append("")
        for j, probe in enumerate(probes, start=1):
            pid = probe.get("id") or f"{sid}-p{j}"
            question = (probe.get("question") or "").strip()
            tier = probe.get("tier", "")
            rubric = probe.get("rubric") or {}
            if not question:
                logger.warning(f"[{sid}] probe {pid} has no question; skipping")
                continue

            # /remember readout — runs first per design (read-only, doesn't
            # mutate conversation memory). Verbatim probe question per the
            # v0.1 query-phrasing convention.
            slash_response = ""
            slash_error = None
            try:
                print(f"\n[{sid}] /remember {question}", flush=True)
                slash_response = str(loop._run_remember(question) or "")
                print(f"[{sid}] /remember → {slash_response[:200]}", flush=True)
            except Exception as e:
                slash_error = f"{type(e).__name__}: {e}"
                logger.exception(f"[{sid}] /remember failed for {pid}: {e}")

            # Agent ask — same path as a regular user turn. WILL mutate
            # memory; subsequent probes in this subset see this exchange.
            agent_response = ""
            agent_error = None
            try:
                print(f"\n[{sid}] agent ask: {question}", flush=True)
                loop._process_user_turn(source=SOURCE, text=question, close=False)
                agent_response = _latest_reply(loop, SOURCE)
                print(f"[{sid}] agent → {agent_response[:200]}", flush=True)
            except Exception as e:
                agent_error = f"{type(e).__name__}: {e}"
                logger.exception(f"[{sid}] agent ask failed for {pid}: {e}")

            # Record
            rec = {
                "session_id": sid,
                "session_description": description,
                "probe_id": pid,
                "probe_index_in_session": j,
                "tier": tier,
                "tells": tells,
                "question": question,
                "slash_query": question,  # v0.1: verbatim
                "rubric": rubric,
                "slash_response": slash_response,
                "slash_error": slash_error,
                "agent_response": agent_response,
                "agent_error": agent_error,
                "world_name": world_name,
                "character_name": char_name,
                "captured_at": datetime.datetime.now(
                    datetime.timezone.utc).isoformat(),
            }
            records.append(rec)

            transcript_lines.append(f"### Probe {pid} (tier {tier or '—'})")
            transcript_lines.append("")
            transcript_lines.append(f"**Question:** {question}")
            transcript_lines.append("")
            transcript_lines.append("**/remember:**")
            transcript_lines.append("")
            transcript_lines.append(slash_response or "(empty)")
            if slash_error:
                transcript_lines.append("")
                transcript_lines.append(f"_/remember error: {slash_error}_")
            transcript_lines.append("")
            transcript_lines.append(f"**{char_name} (agent ask):**")
            transcript_lines.append("")
            transcript_lines.append(agent_response or "(empty)")
            if agent_error:
                transcript_lines.append("")
                transcript_lines.append(f"_agent error: {agent_error}_")
            transcript_lines.append("")
    finally:
        # Match introspective_fidelity teardown.
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning(f"[{sid}] executor shutdown failed: {e}")
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning(f"[{sid}] final persist failed: {e}")

    transcripts_dir.mkdir(parents=True, exist_ok=True)
    (transcripts_dir / f"{sid}.md").write_text("\n".join(transcript_lines))

    return records


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def run_benchmark(scenario_path: Path, probes_dir: Path, output_dir: Path,
                  filter_ids: Optional[List[str]] = None) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    transcripts_dir = output_dir / "transcripts"
    raw_path = output_dir / "raw.jsonl"

    subsets = _discover_subsets(probes_dir, filter_ids)
    if not subsets:
        raise RuntimeError(f"no subsets to run (probes_dir={probes_dir}, "
                           f"filter={filter_ids})")

    run_stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    n_records = 0
    n_subsets_ok = 0
    n_subsets_failed = 0

    with open(raw_path, "w") as raw_f:
        for subset in subsets:
            sid = subset["session_id"]
            world_name = f"bench-memory-{run_stamp}-{sid}"
            print(f"\n========== subset {sid} (world={world_name}) ==========",
                  flush=True)
            try:
                records = _run_subset(subset, scenario_path, world_name,
                                      transcripts_dir)
            except Exception as e:
                tb = traceback.format_exc()
                logger.error(f"[{sid}] subset crashed: {e}\n{tb}")
                # Emit a single error record so the failure is visible
                # downstream rather than silently absent.
                err_rec = {
                    "session_id": sid,
                    "subset_error": f"{type(e).__name__}: {e}",
                    "subset_traceback": tb,
                    "world_name": world_name,
                    "captured_at": datetime.datetime.now(
                        datetime.timezone.utc).isoformat(),
                }
                raw_f.write(json.dumps(err_rec) + "\n")
                raw_f.flush()
                n_subsets_failed += 1
                continue
            for rec in records:
                raw_f.write(json.dumps(rec, default=str) + "\n")
                raw_f.flush()
                n_records += 1
            n_subsets_ok += 1

    summary = {
        "run_stamp": run_stamp,
        "scenario": str(scenario_path),
        "probes_dir": str(probes_dir),
        "subsets_total": len(subsets),
        "subsets_ok": n_subsets_ok,
        "subsets_failed": n_subsets_failed,
        "probes_total": n_records,
    }
    (output_dir / "run_summary.json").write_text(
        json.dumps(summary, indent=2, default=str))

    print(f"\nWrote raw records: {raw_path}")
    print(f"Transcripts:       {transcripts_dir}")
    print(f"Run summary:       {output_dir / 'run_summary.json'}")
    print(f"Subsets ok/failed: {n_subsets_ok}/{n_subsets_failed}")
    print(f"Probes recorded:   {n_records}")
    return raw_path


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Memory Recall benchmark runner. Per subset: fresh "
                    "ChatLoop with timestamped world_name, run tells, run "
                    "/remember + agent-ask per probe, emit raw.jsonl.")
    parser.add_argument(
        "--scenario", type=Path, required=True,
        help="Path to a chat-mode scenario YAML "
             "(e.g. scenarios/jill-benchmark-chat.yaml).")
    parser.add_argument(
        "--probes-dir", type=Path, default=HERE / "probes",
        help="Directory of probe-subset YAMLs. Defaults to "
             "bench/memory_recall/probes/.")
    parser.add_argument(
        "--subsets", nargs="*", default=None,
        help="Optional filter: run only these session_ids "
             "(e.g. --subsets 01-direct-recall 02-lateral-cue).")
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Where to write raw.jsonl + transcripts/. Defaults to "
             "bench/memory_recall/results/<timestamp>_<scenario_stem>.")
    args = parser.parse_args()

    scenario = args.scenario.resolve()
    probes_dir = args.probes_dir.resolve()
    if args.output_dir:
        out = args.output_dir.resolve()
    else:
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H-%M-%SZ")
        out = HERE / "results" / f"{stamp}_{scenario.stem}"

    run_benchmark(scenario_path=scenario, probes_dir=probes_dir,
                  output_dir=out, filter_ids=args.subsets)


if __name__ == "__main__":
    main()
