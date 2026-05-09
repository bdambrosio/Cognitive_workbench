#!/usr/bin/env python3
"""Remember-subagent prompt optimization — runner (v0.1).

Reuses bench/memory_recall's probes and judge but runs MULTIPLE candidate
system prompts for the /remember subagent against the same primer-populated
memory state per probe. The slash readout is read-only, so all variants
see identical state for each probe and can be compared apples-to-apples.

Per-session flow (pseudocode):
    loop = ChatLoop(...)
    for tell in tells:
        loop._process_user_turn(tell)              # populate memory
    for probe in probes:
        for variant in variants:                    # all read-only
            response[variant] = chat.remember.remember(
                probe.question, memory_dir, backend, trace_dir,
                system_prompt_builder=variant.build)
        loop._process_user_turn(probe.question)     # agent ask, mutates
        agent_response = _latest_reply(loop)        # copied across variants

Output: one raw_<variant>.jsonl per variant, in the schema bench/memory_recall/judge.py
expects. Score each with the existing judge — no judge changes needed.

Variants live in variants/v*.py. Each module must export
`build(memory_dir: Path) -> str`. The harness auto-discovers all `v*.py`
files under variants/ at startup.

Usage:
    cd src
    python ../bench/remember_prompt_optimization/runner.py \\
        --scenario ../scenarios/jill-benchmark-chat.yaml

Then score:
    python ../bench/memory_recall/judge.py \\
        --run-dir ../bench/remember_prompt_optimization/results/<stamp>/v1_minimalist
    (repeat per variant; or use score.py wrapper)
"""

from __future__ import annotations

import argparse
import datetime
import importlib.util
import json
import logging
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"

sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from chat.remember import remember as _remember  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("bench.remember_prompt_optimization")

SOURCE = "User"

# Reuse the probes from the memory_recall bench — same fixtures, different
# scoring concern (we vary the slash subagent prompt, not the agent path).
DEFAULT_PROBES_DIR = REPO_ROOT / "bench" / "memory_recall" / "probes"


# ---------------------------------------------------------------------------
# Variant discovery
# ---------------------------------------------------------------------------

def _load_variants(variants_dir: Path) -> Dict[str, Callable[[Path], str]]:
    """Import every `variants/v*.py` and return {variant_id: build_fn}.
    Each module must export `build(memory_dir: Path) -> str`."""
    if not variants_dir.is_dir():
        raise RuntimeError(f"variants dir not found: {variants_dir}")
    out: Dict[str, Callable[[Path], str]] = {}
    for path in sorted(variants_dir.glob("v*.py")):
        if path.name.startswith("_"):
            continue
        name = path.stem
        spec = importlib.util.spec_from_file_location(
            f"_cspred_variant_{name}", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"could not load variant module: {path}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        build = getattr(mod, "build", None)
        if not callable(build):
            raise RuntimeError(
                f"variant {name} missing required `build(memory_dir)` callable")
        out[name] = build
    if not out:
        raise RuntimeError(f"no variants found under {variants_dir}")
    return out


# ---------------------------------------------------------------------------
# Scenario / probes loading (mirrors bench/memory_recall/runner.py)
# ---------------------------------------------------------------------------

def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> tuple[str, dict]:
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


def _discover_subsets(probes_dir: Path,
                      filter_ids: Optional[List[str]] = None
                      ) -> List[Dict[str, Any]]:
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


def _latest_reply(loop: ChatLoop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


# ---------------------------------------------------------------------------
# Per-subset run with multi-variant slash
# ---------------------------------------------------------------------------

def _run_subset(subset: Dict[str, Any], scenario_path: Path,
                world_name: str,
                variants: Dict[str, Callable[[Path], str]]
                ) -> Dict[str, List[Dict[str, Any]]]:
    """Run one subset. For each probe, runs slash readout PER VARIANT
    against the current (read-only) memory state, then runs the agent
    ask once to advance state. Returns {variant_id: [records]} so the
    caller can concatenate into per-variant raw.jsonl files."""
    sid = subset["session_id"]
    tells: List[str] = list(subset.get("tells") or [])
    probes: List[Dict[str, Any]] = list(subset.get("probes") or [])
    description = subset.get("description", "")

    char_name, char_config = _build_character_config(scenario_path, world_name)
    if not char_config.get("chat", {}).get("benchmark_mode"):
        logger.warning(
            f"[{sid}] scenario does not set chat.benchmark_mode=true; reflection "
            "runs on the background executor and probe-time reads may miss "
            "in-flight writes")

    loop = ChatLoop(character_name=char_name, character_config=char_config)

    # Per-variant accumulators, all populated in lock-step from the same
    # primer state.
    out: Dict[str, List[Dict[str, Any]]] = {v: [] for v in variants}

    try:
        # ------ Tells ------
        for i, tell in enumerate(tells, start=1):
            text = (tell or "").strip()
            if not text:
                continue
            print(f"\n[{sid}] tell {i}/{len(tells)}: {text}", flush=True)
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = _latest_reply(loop, SOURCE)
            print(f"[{sid}] {char_name}: {reply}", flush=True)

        # ------ Probes ------
        for j, probe in enumerate(probes, start=1):
            pid = probe.get("id") or f"{sid}-p{j}"
            question = (probe.get("question") or "").strip()
            tier = probe.get("tier", "")
            rubric = probe.get("rubric") or {}
            if not question:
                logger.warning(f"[{sid}] probe {pid} has no question; skipping")
                continue

            memory_dir = loop._memory_dir()
            trace_dir = loop._subagent_traces_dir()

            # Slash readout per variant — all read-only against the
            # SAME memory state, so each variant is independent.
            slash_per_variant: Dict[str, str] = {}
            slash_err_per_variant: Dict[str, Optional[str]] = {}
            for variant_id, build_fn in variants.items():
                slash_response = ""
                slash_error: Optional[str] = None
                try:
                    print(f"\n[{sid}] /remember[{variant_id}] {question}",
                          flush=True)
                    answer = _remember(
                        query=question, memory_dir=memory_dir,
                        llm_backend=loop.backend, trace_dir=trace_dir,
                        system_prompt_builder=build_fn,
                    )
                    text = str(answer or "").strip()
                    if not text:
                        slash_response = "EMPTY: remember subagent produced no answer"
                    else:
                        slash_response = "OK: " + text
                    print(f"[{sid}] /remember[{variant_id}] → "
                          f"{slash_response[:200]}", flush=True)
                except Exception as e:
                    slash_error = f"{type(e).__name__}: {e}"
                    logger.exception(
                        f"[{sid}] /remember[{variant_id}] failed for {pid}: {e}")
                slash_per_variant[variant_id] = slash_response
                slash_err_per_variant[variant_id] = slash_error

            # Agent ask — runs once. Mutates memory; subsequent probes
            # see this exchange. Identical across variants since the
            # agent path doesn't use the subagent prompt.
            agent_response = ""
            agent_error = None
            try:
                print(f"\n[{sid}] agent ask: {question}", flush=True)
                loop._process_user_turn(source=SOURCE, text=question,
                                        close=False)
                agent_response = _latest_reply(loop, SOURCE)
                print(f"[{sid}] agent → {agent_response[:200]}", flush=True)
            except Exception as e:
                agent_error = f"{type(e).__name__}: {e}"
                logger.exception(f"[{sid}] agent ask failed for {pid}: {e}")

            # Emit one record per variant — each carries its own
            # slash_response but shares agent_response.
            captured_at = datetime.datetime.now(
                datetime.timezone.utc).isoformat()
            for variant_id in variants:
                rec = {
                    "session_id": sid,
                    "session_description": description,
                    "probe_id": pid,
                    "probe_index_in_session": j,
                    "tier": tier,
                    "tells": tells,
                    "question": question,
                    "slash_query": question,
                    "rubric": rubric,
                    "slash_response": slash_per_variant[variant_id],
                    "slash_error": slash_err_per_variant[variant_id],
                    "agent_response": agent_response,
                    "agent_error": agent_error,
                    "world_name": world_name,
                    "character_name": char_name,
                    "captured_at": captured_at,
                    "variant_id": variant_id,
                }
                out[variant_id].append(rec)
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning(f"[{sid}] executor shutdown failed: {e}")
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning(f"[{sid}] final persist failed: {e}")

    return out


# ---------------------------------------------------------------------------
# Top-level
# ---------------------------------------------------------------------------

def run(scenario_path: Path, probes_dir: Path, output_dir: Path,
        variants_dir: Path,
        only_subsets: Optional[List[str]] = None) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    variants = _load_variants(variants_dir)
    print(f"\nVariants discovered: {sorted(variants.keys())}", flush=True)

    subsets = _discover_subsets(probes_dir, filter_ids=only_subsets)
    if not subsets:
        raise RuntimeError(f"no subsets found in {probes_dir}")

    run_stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    # One per-variant output dir mirrors memory_recall's layout so judge.py
    # works unchanged.
    per_variant_dirs: Dict[str, Path] = {}
    per_variant_records: Dict[str, List[Dict[str, Any]]] = {}
    for variant_id in variants:
        d = output_dir / variant_id
        d.mkdir(parents=True, exist_ok=True)
        per_variant_dirs[variant_id] = d
        per_variant_records[variant_id] = []

    for subset in subsets:
        sid = subset["session_id"]
        world = f"bench-rememberopt-{run_stamp}-{sid}"
        print(f"\n========== subset {sid}  world={world} ==========",
              flush=True)
        per_variant = _run_subset(subset, scenario_path, world, variants)
        for variant_id, records in per_variant.items():
            per_variant_records[variant_id].extend(records)

    # Write per-variant raw.jsonl + run_summary.json. Mirrors
    # memory_recall/runner.py outputs so judge.py reads them as-is.
    for variant_id, records in per_variant_records.items():
        d = per_variant_dirs[variant_id]
        with open(d / "raw.jsonl", "w") as f:
            for rec in records:
                f.write(json.dumps(rec, default=str) + "\n")
        summary = {
            "run_stamp": run_stamp,
            "scenario": scenario_path.name,
            "variant_id": variant_id,
            "n_subsets": len(subsets),
            "n_probes": len(records),
        }
        with open(d / "run_summary.json", "w") as f:
            json.dump(summary, f, indent=2, default=str)

    # Top-level index.
    index = {
        "run_stamp": run_stamp,
        "scenario": scenario_path.name,
        "variants": sorted(variants.keys()),
        "subsets": [s["session_id"] for s in subsets],
        "n_probes_total_per_variant": len(
            per_variant_records[next(iter(variants))]),
    }
    with open(output_dir / "run_index.json", "w") as f:
        json.dump(index, f, indent=2, default=str)
    print(f"\nMulti-variant run complete: {output_dir}")
    print(f"  variants: {sorted(variants.keys())}")
    print(f"  per-variant raw.jsonl + run_summary.json under each variant subdir")
    print(f"  next: score with bench/memory_recall/judge.py --run-dir <variant_subdir>")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Multi-variant runner for the /remember subagent "
                    "prompt optimization. Drives one ChatLoop per probe "
                    "subset and tests every variant's slash readout "
                    "against identical memory state.")
    parser.add_argument(
        "--scenario", type=Path, required=True,
        help="Path to chat-mode scenario YAML (e.g. "
             "scenarios/jill-benchmark-chat.yaml).")
    parser.add_argument(
        "--probes-dir", type=Path, default=DEFAULT_PROBES_DIR,
        help="Probes directory (default: bench/memory_recall/probes/).")
    parser.add_argument(
        "--variants-dir", type=Path, default=HERE / "variants",
        help="Variants directory (default: bench/remember_prompt_optimization/variants/).")
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Output dir. Default: bench/remember_prompt_optimization/results/<stamp>_<scenario>.")
    parser.add_argument(
        "--subsets", nargs="+", default=None,
        help="Subset filter (e.g. --subsets 01-direct-recall 03-negative-fact). "
             "Default: all subsets in --probes-dir.")
    args = parser.parse_args()

    scenario = args.scenario.resolve()
    probes_dir = args.probes_dir.resolve()
    variants_dir = args.variants_dir.resolve()

    stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")
    if args.output_dir:
        out = args.output_dir.resolve()
    else:
        out = HERE / "results" / f"{stamp}_{scenario.stem}"

    run(scenario_path=scenario, probes_dir=probes_dir, output_dir=out,
        variants_dir=variants_dir, only_subsets=args.subsets)


if __name__ == "__main__":
    main()
