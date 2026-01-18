#!/usr/bin/env python3
"""
Offline ToolModel consolidation (KISS).

Reads authoritative JSONL logs:
  - scenarios/<world>/resources/planner_history/compressed_trace.jsonl
  - scenarios/<world>/resources/planner_history/reflection_frame.jsonl

Maintains a single watermark in scenarios/<world>/resources/tool_model.json:
  - consolidation_last_seq: int

For seq > watermark:
  - extracts per-tool success/failure counts from compressed_trace
  - for tools with >=1 success and >=1 failure (cumulative), runs Single_Act_Features once
  - stores latest induced predicates in tool_model.json

Supports full reset (clear ToolModel).
"""

import argparse
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from composites import Single_Act_Features
from llm_client import ZenohLLMClient
from tool_model import empty_tool_model


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _base_dir(world: str) -> Path:
    return _project_root() / "scenarios" / world / "resources"


def _read_jsonl(path: Path) -> Iterable[Dict[str, Any]]:
    if not path.exists():
        return []
    out: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception as e:
                raise RuntimeError(f"Failed to parse JSONL {path} line {line_num}: {e}")
            if isinstance(obj, dict):
                out.append(obj)
    return out


def _load_world_model_raw_tool_contracts(base_dir: Path, tool_name: str) -> str:
    wm_path = base_dir / "world_model.json"
    if not wm_path.exists():
        return ""
    try:
        content = json.loads(wm_path.read_text(encoding="utf-8"))
    except Exception:
        return ""
    world_model = content.get("world_model") if isinstance(content, dict) and "world_model" in content else content
    if not isinstance(world_model, dict):
        return ""
    raw_data = world_model.get("raw_data", {}) if world_model.get("raw_data") else {}
    contracts = raw_data.get("tool_contracts", []) if isinstance(raw_data, dict) else []
    if not isinstance(contracts, list):
        return ""
    filtered = [c for c in contracts if isinstance(c, dict) and c.get("tool") == tool_name]
    return json.dumps(filtered, indent=2) if filtered else ""


def _find_skill_md_text(world: str, tool_name: str) -> str:
    root = _project_root()
    candidates: List[Path] = []
    # Core tools
    candidates.extend([
        root / "src" / "tools" / tool_name / "Skill.md",
        root / "src" / "tools" / tool_name / "SKILL.md",
        root / "src" / "tools" / tool_name / "skill.md",
    ])
    # World tools
    candidates.extend([
        root / "src" / "world-tools" / world / tool_name / "Skill.md",
        root / "src" / "world-tools" / world / tool_name / "SKILL.md",
        root / "src" / "world-tools" / world / tool_name / "skill.md",
    ])
    for p in candidates:
        if not p.exists():
            continue
        try:
            content = p.read_text(encoding="utf-8")
        except Exception:
            continue
        # Strip YAML frontmatter if present
        m = re.search(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL)
        if m:
            content = content[m.end():].strip()
        return content
    return ""


_RESULT_RE = re.compile(r"^\[RESULT tool=([\w-]+)(?:\s+status=(OK|FAILED))?", re.MULTILINE)
_STAGE2_RE = re.compile(r"^\[STAGE2 step=(\d+)/(\d+)\]", re.MULTILINE)


def _extract_tool_counts_and_excerpts(compressed_trace: str) -> Tuple[Dict[str, Dict[str, int]], Dict[str, List[str]]]:
    """
    Parse compressed_trace text to:
    - count per tool: total/success/failure (based on [RESULT ... status=OK|FAILED])
    - collect per tool: step-block excerpts where the tool appears (CALL/RESULT lines)
    """
    counts: Dict[str, Dict[str, int]] = {}
    excerpts: Dict[str, List[str]] = {}

    for tool, status in _RESULT_RE.findall(compressed_trace or ""):
        if tool not in counts:
            counts[tool] = {"total": 0, "success": 0, "failure": 0}
        counts[tool]["total"] += 1
        if status == "OK":
            counts[tool]["success"] += 1
        elif status == "FAILED":
            counts[tool]["failure"] += 1

    # Step-block excerpts (complete blocks)
    if not compressed_trace:
        return counts, excerpts

    lines = compressed_trace.split("\n")
    current_block: List[str] = []
    current_step: Optional[int] = None
    blocks: List[Tuple[Optional[int], str]] = []

    for line in lines:
        m = _STAGE2_RE.match(line)
        if m:
            if current_block:
                blocks.append((current_step, "\n".join(current_block).strip()))
            current_step = int(m.group(1))
            current_block = [line]
        else:
            if current_block is not None:
                current_block.append(line)
    if current_block:
        blocks.append((current_step, "\n".join(current_block).strip()))

    for _, block in blocks:
        if not block:
            continue
        # Include block for each tool mentioned in CALL/RESULT lines
        for tool in counts.keys():
            if f"[CALL tool={tool}]" in block or f"[RESULT tool={tool}" in block:
                excerpts.setdefault(tool, []).append(block)

    return counts, excerpts


class _OfflineExecutor:
    """Minimal executor shim for composites.Single_Act_Features (KISS)."""

    def __init__(self, server_name: str, model_name: str):
        self.client = ZenohLLMClient(server_name=server_name, model_name=model_name)

    def llm_generate(self, messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
        resp = self.client.generate(
            messages=messages if isinstance(messages, list) else [str(messages)],
            bindings=bindings or {},
            max_tokens=max_tokens,
            temperature=temperature,
            stops=stops or ["</end>"],
            is_json=is_json,
        )
        if not getattr(resp, "success", False):
            raise RuntimeError(getattr(resp, "error", "LLM failed"))
        return getattr(resp, "text", "")


def _load_tool_model(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return empty_tool_model()
    try:
        content = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return empty_tool_model()
    tool_model = content.get("tool_model") if isinstance(content, dict) and "tool_model" in content else content
    return tool_model if isinstance(tool_model, dict) else empty_tool_model()


def _save_tool_model(path: Path, tool_model: Dict[str, Any]) -> None:
    tool_model["updated_at"] = _now_iso()
    path.write_text(json.dumps(tool_model, indent=2), encoding="utf-8")


def consolidate(world: str, server_name: str, model_name: str) -> int:
    print(f"[consolidate] Starting consolidation for world={world}, server={server_name}, model={model_name}")
    base = _base_dir(world)
    base.mkdir(parents=True, exist_ok=True)

    tool_model_path = base / "tool_model.json"
    print(f"[consolidate] Loading tool model from {tool_model_path}")
    tm = _load_tool_model(tool_model_path)
    last_seq = int(tm.get("consolidation_last_seq") or 0)
    print(f"[consolidate] Last processed seq: {last_seq}")

    hist = base / "planner_history"
    ct_path = hist / "compressed_trace.jsonl"
    rf_path = hist / "reflection_frame.jsonl"

    print(f"[consolidate] Reading compressed_trace.jsonl...")
    compressed_records = [r for r in _read_jsonl(ct_path) if int(r.get("seq") or 0) > last_seq]
    print(f"[consolidate] Found {len(compressed_records)} new compressed_trace records")

    print(f"[consolidate] Reading reflection_frame.jsonl...")
    reflection_records = [r for r in _read_jsonl(rf_path) if int(r.get("seq") or 0) > last_seq]
    print(f"[consolidate] Found {len(reflection_records)} new reflection_frame records")

    if not compressed_records and not reflection_records:
        print(f"No new records (last_seq={last_seq}).")
        return 0

    compressed_by_seq = {int(r.get("seq")): r for r in compressed_records if "seq" in r}
    reflection_by_seq = {int(r.get("seq")): r for r in reflection_records if "seq" in r}

    # Require both records for a seq; fail fast if missing
    seqs = sorted(set(compressed_by_seq.keys()) | set(reflection_by_seq.keys()))
    for s in seqs:
        if s not in compressed_by_seq or s not in reflection_by_seq:
            raise RuntimeError(f"Missing record for seq={s}: compressed={s in compressed_by_seq} reflection={s in reflection_by_seq}")

    print(f"[consolidate] Processing {len(seqs)} sequence(s): {seqs}")

    # Update cumulative outcome counts
    outcomes: Dict[str, Dict[str, int]] = tm.get("tool_outcomes", {}) if isinstance(tm.get("tool_outcomes"), dict) else {}
    touched_tools: set[str] = set()
    excerpts_by_tool: Dict[str, List[str]] = {}

    max_seq = max(seqs) if seqs else last_seq

    print(f"[consolidate] Extracting tool counts and excerpts from {len(seqs)} compressed traces...")
    for idx, s in enumerate(seqs, 1):
        print(f"[consolidate]   Processing seq {s} ({idx}/{len(seqs)})...")
        ct = compressed_by_seq[s].get("compressed_trace", "")
        trace_len = len(ct) if isinstance(ct, str) else 0
        print(f"[consolidate]     Compressed trace length: {trace_len:,} chars")
        counts, excerpts = _extract_tool_counts_and_excerpts(ct if isinstance(ct, str) else "")
        print(f"[consolidate]     Found {len(counts)} tools, {sum(len(exs) for exs in excerpts.values())} total excerpts")
        for tool_name, c in counts.items():
            touched_tools.add(tool_name)
            cur = outcomes.get(tool_name) if isinstance(outcomes.get(tool_name), dict) else {"total": 0, "success": 0, "failure": 0}
            cur["total"] = int(cur.get("total") or 0) + int(c.get("total") or 0)
            cur["success"] = int(cur.get("success") or 0) + int(c.get("success") or 0)
            cur["failure"] = int(cur.get("failure") or 0) + int(c.get("failure") or 0)
            outcomes[tool_name] = cur
        for tool_name, exs in excerpts.items():
            if not exs:
                continue
            excerpts_by_tool.setdefault(tool_name, []).extend(exs)

    tm["tool_outcomes"] = outcomes
    print(f"[consolidate] Total touched tools: {len(touched_tools)}")

    # Load raw tool_contracts once
    def tool_contracts_for(tool_name: str) -> str:
        return _load_world_model_raw_tool_contracts(base, tool_name)

    features_by_tool: Dict[str, Any] = tm.get("tool_features", {}) if isinstance(tm.get("tool_features"), dict) else {}
    print(f"[consolidate] Initializing executor stub...")
    exec_stub = _OfflineExecutor(server_name=server_name, model_name=model_name)

    # Inclusive but bounded: only run composites for tools that (cumulative) have >=1 success and >=1 failure, and were touched
    eligible = [
        t for t in sorted(touched_tools)
        if int(outcomes.get(t, {}).get("success") or 0) > 0 and int(outcomes.get(t, {}).get("failure") or 0) > 0
    ]

    print(f"[consolidate] Processing seqs {seqs[0]}..{seqs[-1]} ({len(seqs)} records). Eligible tools: {len(eligible)}")
    if eligible:
        print(f"[consolidate] Eligible tools: {', '.join(eligible)}")

    for idx, tool_name in enumerate(eligible, 1):
        print(f"[consolidate] Processing tool {idx}/{len(eligible)}: {tool_name}")
        print(f"[consolidate]   Loading skill_md...")
        skill_md = _find_skill_md_text(world, tool_name)
        print(f"[consolidate]   Skill_md length: {len(skill_md):,} chars")
        print(f"[consolidate]   Loading tool_contracts...")
        tool_contracts = tool_contracts_for(tool_name)
        print(f"[consolidate]   Tool_contracts length: {len(tool_contracts):,} chars")
        exs = excerpts_by_tool.get(tool_name, [])
        print(f"[consolidate]   Using {len(exs)} excerpts (keeping first 12)")
        # Keep excerpts bounded (KISS)
        trace_excerpts = "\n\n".join(exs[:12])
        print(f"[consolidate]   Trace_excerpts length: {len(trace_excerpts):,} chars")
        stats = outcomes.get(tool_name, {"total": 0, "success": 0, "failure": 0})
        statistics = json.dumps(stats, indent=2)
        print(f"[consolidate]   Statistics: {stats}")

        print(f"[consolidate]   Initializing Single_Act_Features...")
        saf = Single_Act_Features(
            executor=exec_stub,
            tool_name=tool_name,
            skill_md=skill_md,
            tool_contracts=tool_contracts,
            trace_excerpts=trace_excerpts,
            statistics=statistics,
        )
        print(f"[consolidate]   Generating features (this may take a while, LLM calls in progress)...")
        predicates = saf.generate_features()
        print(f"[consolidate]   Generated {len(predicates) if isinstance(predicates, list) else 'N/A'} predicates")
        features_by_tool[tool_name] = {"latent_predicates": predicates, "updated_at": _now_iso(), "last_seq": max_seq}
        print(f"[consolidate] ✓ Updated features for {tool_name}")

    print(f"[consolidate] Saving tool model...")
    tm["tool_features"] = features_by_tool
    tm["consolidation_last_seq"] = max_seq
    if "created_at" not in tm:
        tm["created_at"] = _now_iso()

    _save_tool_model(tool_model_path, tm)
    print(f"[consolidate] ✓ Saved ToolModel. consolidation_last_seq={max_seq}")
    return 0


def clear(world: str) -> int:
    base = _base_dir(world)
    base.mkdir(parents=True, exist_ok=True)
    tool_model_path = base / "tool_model.json"
    tm = empty_tool_model()
    tm["created_at"] = _now_iso()
    tm["updated_at"] = _now_iso()
    tm["consolidation_last_seq"] = 0
    tm["tool_features"] = {}
    tm["tool_outcomes"] = {}
    _save_tool_model(tool_model_path, tm)
    print(f"Cleared ToolModel at {tool_model_path}")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--world", required=True, help="World name (e.g., minecraft)")
    p.add_argument("--server_name", default="openai")
    p.add_argument("--model_name", default="gpt-4.1")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("run", help="Consolidate ToolModel from new planner_history records")
    sub.add_parser("clear", help="Clear ToolModel (reset watermark and features)")
    args = p.parse_args(argv)

    if args.cmd == "clear":
        return clear(args.world)
    return consolidate(args.world, args.server_name, args.model_name)


if __name__ == "__main__":
    raise SystemExit(main())

