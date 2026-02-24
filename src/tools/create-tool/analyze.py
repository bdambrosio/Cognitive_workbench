#!/usr/bin/env python3
"""
Offline analysis tool for create_tool_opportunities.jsonl.

Reads the log, uses Claude/LLM to infer tool specs from the compressed trace,
and stages generated tools to src/tools-staging/.

Usage:
  cd src && python tools/create-tool/analyze.py [OPTIONS]

  --log-path PATH    Path to create_tool_opportunities.jsonl (default: logs/planner_history/create_tool_opportunities.jsonl)
  --entry N          Process only entry with seq=N
  --last N           Process last N entries (default: 1)
  --list             List entries and exit
  --vllm-url URL    vLLM API URL (default: VLLM_URL env or http://localhost:5000/v1/chat/completions)
  --vllm-model M    vLLM model (default: VLLM_MODEL env or auto from server)
"""

import argparse
import json
import logging
import os
import sys
from pathlib import Path
from typing import List, Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Add src to path for imports
_SRC = Path(__file__).resolve().parent.parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

# Load create-tool module (directory has hyphen, use direct load)
import importlib.util
_tool_spec = importlib.util.spec_from_file_location("create_tool_tool", Path(__file__).parent / "tool.py")
_tool_mod = importlib.util.module_from_spec(_tool_spec)
_tool_spec.loader.exec_module(_tool_mod)
generate_and_stage_tool = _tool_mod.generate_and_stage_tool

_DEFAULT_LOG = _SRC / "logs" / "planner_history" / "create_tool_opportunities.jsonl"


def _make_offline_llm(vllm_url: str, vllm_model: str):
    """Return an llm_generate callback that calls vLLM API."""
    import requests

    class Response:
        def __init__(self, success, text="", error=None):
            self.success = success
            self.text = text
            self.error = error

    def llm_generate(messages, max_tokens=2000, temperature=0.2, stops=None, **kwargs):
        if isinstance(messages, list):
            chat = []
            for m in messages:
                if isinstance(m, str):
                    chat.append({"role": "user", "content": m})
                else:
                    chat.append(m)
        else:
            chat = [{"role": "user", "content": str(messages)}]
        payload = {
            "model": vllm_model,
            "messages": chat,
            "max_tokens": max_tokens + 256,
            "temperature": temperature,
        }
        if stops:
            payload["stop"] = stops if isinstance(stops, list) else [stops]
        try:
            r = requests.post(vllm_url, json=payload, timeout=180)
            r.raise_for_status()
            data = r.json()
            content = data.get("choices", [{}])[0].get("message", {}).get("content", "")
            return Response(success=True, text=content or "")
        except Exception as e:
            logger.exception(f"vLLM call failed: {e}")
            return Response(success=False, error=str(e))

    return llm_generate


def _resolve_model(vllm_url: str) -> str:
    """Try to get model from vLLM server /v1/models."""
    import requests
    try:
        r = requests.get(vllm_url.replace("/v1/chat/completions", "/v1/models"), timeout=5)
        if r.ok:
            data = r.json()
            models = data.get("data", [])
            if models:
                return models[0].get("id", "default")
    except Exception:
        pass
    return "default"


def _infer_tool_spec(llm_generate, entry: dict) -> Optional[dict]:
    """Use LLM to infer name, description, requirements from opportunity entry."""
    goal = entry.get("goal", "")
    failure_evidence = entry.get("failure_evidence", [])
    open_questions = entry.get("open_questions", [])
    immediate_blockers = entry.get("immediate_blockers", [])
    available_tools = entry.get("available_tools", [])
    trace = entry.get("compressed_trace", "")

    prompt = f"""You are analyzing a planner failure where failure_mode was "missing_affordance".
The planner had these tools available: {available_tools}

Goal: {goal}
Failure evidence: {failure_evidence}
Immediate blockers: {immediate_blockers}
Open questions: {json.dumps(open_questions)[:500]}

FULL EXECUTION TRACE (compressed):
{trace[:12000]}

From the trace, infer what capability the planner was trying to use but lacked.
Output ONLY a single JSON object with keys: name, description, requirements.
- name: tool-name (lowercase, hyphens, e.g. post-bluesky)
- description: one-line description
- requirements: detailed spec (APIs, libs, env vars, params, output)

No other text. Valid JSON only."""

    response = llm_generate(messages=[prompt], max_tokens=800, temperature=0.2)
    if not response.success:
        return None
    text = (response.text or "").strip()
    start = text.find("{")
    if start < 0:
        logger.warning("No JSON object in inference response")
        return None
    # Extract object (find matching brace)
    depth = 0
    end = -1
    for i, c in enumerate(text[start:], start):
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                end = i
                break
    if end < 0:
        text = text[start:] + "}"
    else:
        text = text[start : end + 1]
    try:
        return json.loads(text)
    except json.JSONDecodeError as e:
        logger.warning(f"Could not parse inference JSON: {e}")
        return None


def load_entries(log_path: Path) -> List[dict]:
    """Load JSONL and return list of records."""
    if not log_path.exists():
        return []
    entries = []
    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return entries


def main():
    ap = argparse.ArgumentParser(description="Analyze create_tool_opportunities log and generate tools")
    ap.add_argument("--log-path", type=Path, default=_DEFAULT_LOG, help="Path to JSONL log")
    ap.add_argument("--entry", type=int, help="Process only entry with seq=N")
    ap.add_argument("--last", type=int, default=1, help="Process last N entries")
    ap.add_argument("--list", action="store_true", help="List entries and exit")
    ap.add_argument("--vllm-url", default=os.environ.get("VLLM_URL", "http://localhost:5000/v1/chat/completions"))
    ap.add_argument("--vllm-model", default=os.environ.get("VLLM_MODEL"))
    args = ap.parse_args()

    entries = load_entries(args.log_path)
    if not entries:
        if args.list:
            print(f"No entries in {args.log_path}")
            return 0
        logger.warning(f"No entries in {args.log_path}")
        return 1

    if args.list:
        for i, e in enumerate(entries):
            seq = e.get("seq", "?")
            goal = (e.get("goal", "") or "")[:60]
            print(f"  {i}: seq={seq} goal={goal}...")
        return 0

    # Select entries to process
    if args.entry is not None:
        selected = [e for e in entries if e.get("seq") == args.entry]
        if not selected:
            logger.error(f"No entry with seq={args.entry}")
            return 1
    else:
        selected = entries[-args.last :]

    vllm_model = args.vllm_model or _resolve_model(args.vllm_url)
    llm_generate = _make_offline_llm(args.vllm_url, vllm_model)

    staged = []
    for entry in selected:
        seq = entry.get("seq", "?")
        goal = (entry.get("goal", "") or "")[:80]
        logger.info(f"Processing seq={seq}: {goal}...")
        spec = _infer_tool_spec(llm_generate, entry)
        if not spec:
            logger.warning(f"  Skipped: could not infer tool spec")
            continue
        name = spec.get("name", "").strip()
        description = spec.get("description", "").strip()
        requirements = spec.get("requirements", "").strip()
        if not name or not description or not requirements:
            logger.warning(f"  Skipped: incomplete spec (name={name!r})")
            continue
        success, msg, staging_dir = generate_and_stage_tool(llm_generate, name, description, requirements)
        if success:
            logger.info(f"  Staged: {staging_dir}")
            staged.append(staging_dir)
        else:
            logger.warning(f"  Failed: {msg}")

    if staged:
        print("\nStaged tools:")
        for d in staged:
            print(f"  {d}")
        print("\nTo activate: review, then mv to src/tools/ and restart.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
