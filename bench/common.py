#!/usr/bin/env python3
"""Shared substrate for benchmarks v2.

Three things every probe needs and none should reimplement: overlaying a
backend arm onto a scenario, proving which model actually answered, and
reading the per-turn trace a run leaves behind.

Import from a probe with the repo's `src/` already on sys.path:

    sys.path.insert(0, str(REPO / "src"))
    from bench.common import load_arm, build_loop_config, verify_served_model
"""

from __future__ import annotations

import json
import logging
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
BACKENDS_FILE = HERE / "backends.yaml"

logger = logging.getLogger("bench.common")


# ---------------------------------------------------------------------------
# Backend arms
# ---------------------------------------------------------------------------

def load_arm(name: str) -> Dict[str, Any]:
    """One arm from backends.yaml, by name."""
    arms = yaml.safe_load(BACKENDS_FILE.read_text(encoding="utf-8")) or {}
    if name not in arms:
        raise KeyError(f"unknown backend {name!r}; have {sorted(arms)}")
    arm = dict(arms[name])
    arm["name"] = name
    return arm


def verify_served_model(arm: Dict[str, Any], timeout: float = 10.0) -> Dict[str, Any]:
    """Ask the server what it is actually serving, and refuse to proceed if it
    is not the arm we think we are measuring.

    This exists because a comparison whose backend identity rests on
    recollection is not a comparison. Returns a dict recorded verbatim into
    run_meta.json so a result can always name its own backend.
    """
    expect = arm.get("expects_served_model")
    base = (arm.get("llm_config") or {}).get("vllm_url") or ""
    if not expect:
        return {"checked": False, "reason": "cloud arm — nothing to interrogate",
                "served": arm.get("llm_config", {}).get("model")}
    url = base.rstrip("/")
    if url.endswith("/v1"):
        url = url[:-3]
    url = f"{url}/v1/models"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as fh:
            payload = json.loads(fh.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise RuntimeError(
            f"cannot reach {url} to verify the served model ({e}). The local "
            f"backend must be up before a probe runs — a connection-refused "
            f"mid-run reads as a mechanism failure in the trace and is not one."
        ) from e
    served = [m.get("id", "") for m in (payload.get("data") or [])]
    if not any(expect.lower() in s.lower() for s in served):
        raise RuntimeError(
            f"arm {arm['name']!r} expects a model matching {expect!r} but "
            f"{url} is serving {served}. Restart the server on the right model "
            f"before running, or this row will be mislabelled."
        )
    return {"checked": True, "endpoint": url, "served": served, "expected": expect}


# ---------------------------------------------------------------------------
# Scenario + arm -> ChatLoop config
# ---------------------------------------------------------------------------

def build_loop_config(scenario_path: Path, arm: Dict[str, Any],
                      world_name: str,
                      autonomy: bool = False) -> Tuple[str, Dict[str, Any]]:
    """Return (character_name, character_config) for the single chat character
    in `scenario_path`, with the arm's llm_config overlaid.

    Mirrors bench/hle/runner.py's loader — same `parse_characters` call, so the
    bench and the runtime cannot drift apart on how a character is assembled.
    The arm REPLACES rather than merges into llm_config: a merge would let a
    stale field from the scenario (an api_key, a reasoning_effort) survive into
    an arm that never declared one, which is exactly the silent second variable
    these probes exist to avoid.
    """
    from launcher import parse_characters  # noqa: E402  (needs src/ on path)

    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8")) or {}
    arm_llm = dict(arm.get("llm_config") or {})

    scen_llm = dict(scenario.get("llm_config") or {})
    scen_llm.update(arm_llm)
    for char in (scenario.get("characters") or {}).values():
        if isinstance(char, dict) and char.get("mode") == "chat":
            char["llm_config"] = dict(arm_llm)

    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world_name

    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat_chars = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat_chars) != 1:
        raise RuntimeError(
            f"expected exactly 1 chat character in {scenario_path}, found "
            f"{len(chat_chars)}")
    name, cfg = chat_chars[0]
    # Concern CREATION happens in-turn and needs no autonomy; only FIRING is
    # gated (chat_loop.py:2211, unconditional). Probes that assert on a
    # spawned successor therefore work with this off, which is the safe
    # default for a bench — see feedback_resource_manager_safe_testing.
    cfg["autonomy_enabled"] = bool(autonomy)
    return name, cfg


# ---------------------------------------------------------------------------
# Trace reading
# ---------------------------------------------------------------------------

def read_reasoning_trace(world_name: str, character: str) -> List[Dict[str, Any]]:
    """Per-turn records for a run. `exit_reason` and `iters` live here, which
    is how the yield probe scores without a judge."""
    path = (REPO / "scenarios" / world_name / character / "memory"
            / "reasoning_trace.jsonl")
    if not path.is_file():
        logger.warning("no reasoning trace at %s", path)
        return []
    out: List[Dict[str, Any]] = []
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError as e:
            logger.warning("unparseable trace line in %s: %s", path, e)
    return out


def read_agent_concerns(world_name: str, character: str) -> List[Dict[str, Any]]:
    """Agent-concern notes present after a run.

    Reads the persisted infospace snapshot directly. Deliberately does NOT
    stand a resource manager up: the bench must never touch live
    resource-manager state (a test did, once, and corrupted jill_chat).
    Notes live under note_instances[<id>].properties, and a concern is
    properties.kind == 'agent_concern'.
    """
    path = (REPO / "scenarios" / world_name / "resources" / character
            / "resources.json")
    if not path.is_file():
        logger.warning("no resource snapshot at %s", path)
        return []
    try:
        snap = json.loads(path.read_text(errors="replace"))
    except json.JSONDecodeError as e:
        logger.warning("unparseable resource snapshot %s: %s", path, e)
        return []
    out: List[Dict[str, Any]] = []
    for nid, note in (snap.get("note_instances") or {}).items():
        props = dict((note or {}).get("properties") or {})
        if props.get("kind") == "agent_concern":
            props["note_id"] = nid
            props.setdefault("description", (note or {}).get("description"))
            out.append(props)
    return out


def turn_costs(trace: List[Dict[str, Any]]) -> Dict[str, Any]:
    """The probe-6 cost rider. Free to compute, and it is the number that keeps
    being a surprise."""
    iters = [t.get("iters") or 0 for t in trace]
    return {
        "turns": len(trace),
        "iters_total": sum(iters),
        "iters_max": max(iters) if iters else 0,
        "exit_reasons": {r: sum(1 for t in trace if t.get("exit_reason") == r)
                         for r in sorted({t.get("exit_reason") for t in trace}
                                         - {None})},
    }


# ---------------------------------------------------------------------------
# Extraction instrument
# ---------------------------------------------------------------------------

# Held CONSTANT across arms on purpose. If arm A were scored with a Gemma
# extraction and arm B with a Luna extraction, a difference in extraction
# quality would show up as a difference in the metric — precisely the confound
# the bench exists to remove. coord_search/score.py makes the same choice.
EXTRACTOR_ARM = "gemma"


def build_extractor(arm_name: str = EXTRACTOR_ARM):
    """A _ChatBackend for reading structure out of free prose.

    House rule: no keyword matching for classification. Grading whether a reply
    says a path starts primed is meaning, not string presence, so it goes
    through a model — but the model only EXTRACTS. Comparison against ground
    truth stays mechanical, so the judge has no say in the score.
    """
    from chat.backend import _ChatBackend  # noqa: E402

    arm = load_arm(arm_name)
    llm = arm.get("llm_config") or {}
    logger.info("extraction instrument: %s (arm %s)",
                llm.get("model") or "<default>", arm_name)
    return _ChatBackend(
        server=llm.get("server", "local"),
        model=llm.get("model", ""),
        base_url=llm.get("vllm_url") or "http://127.0.0.1:5000",
        is_reasoning=llm.get("is_reasoning_model"),
        api_key=llm.get("api_key"),
        reasoning_effort=llm.get("reasoning_effort"),
    )


def extract_json(backend, system: str, body: str,
                 max_tokens: int = 2000) -> Optional[Any]:
    """One constrained extraction call. Returns parsed JSON or None."""
    from utils.json_utils import repair_json_string  # noqa: E402

    raw = backend.chat([{"role": "system", "content": system},
                        {"role": "user", "content": body}],
                       max_tokens=max_tokens, temperature=0.0, is_json=True)
    parsed = repair_json_string(raw) if isinstance(raw, str) else raw
    if parsed is None:
        logger.warning("extraction returned no usable JSON")
    return parsed


# ---------------------------------------------------------------------------
# Run validity
# ---------------------------------------------------------------------------

# A run can fail in a way that makes its NUMBER describe the harness rather
# than the backend. Those must not be ranked alongside real scores — this
# session produced two such numbers (Gemma 0/16, then 12/16) that read as
# backend results and were not. `validity` is the third outcome beside
# "scored" and "crashed": measurement invalid, do not compare.
#
# Detected by scanning the launcher log over the run's own window, the way
# coord_search/score.py already scopes its metrics. The per-turn trace carries
# no finish_reason, so the log is the only record.

_LAUNCHER_LOG = REPO / "logs" / "character_launcher.log"


def _local_window(captured_at_utc: str, wall_s: float, slack_s: float = 90.0):
    """The launcher log stamps LOCAL time; run_meta records UTC. Convert, and
    pad the end — post-turn work outlives the measured wall clock."""
    import datetime as _dt
    t0 = _dt.datetime.strptime(captured_at_utc, "%Y-%m-%dT%H-%M-%SZ").replace(
        tzinfo=_dt.timezone.utc).astimezone()
    t1 = t0 + _dt.timedelta(seconds=float(wall_s or 0) + slack_s)
    return t0.strftime("%Y-%m-%d %H:%M:%S"), t1.strftime("%Y-%m-%d %H:%M:%S")


def scan_validity(captured_at_utc: str, wall_s: float) -> Dict[str, Any]:
    """Look for signals that this run's score is not about the backend.

    truncated        - any finish=length. The model was cut off mid-emission,
                       so the reply is not what it would have written.
    empty_content    - the parser-on truncation signature: with a reasoning
                       parser active a mid-thought cut returns empty content
                       rather than partial text, which is silent.
    connection_error - the backend went away mid-run; reads as a mechanism
                       failure in the trace and is not one.
    """
    out: Dict[str, Any] = {"checked": False, "valid": True, "reasons": []}
    if not _LAUNCHER_LOG.is_file():
        out["reasons"].append("launcher log missing — could not check")
        out["valid"] = None
        return out
    try:
        lo, hi = _local_window(captured_at_utc, wall_s)
    except (ValueError, TypeError) as e:
        logger.warning("cannot build log window: %s", e)
        out["reasons"].append(f"unparseable timestamps: {e}")
        out["valid"] = None
        return out

    counts = {"finish=length": 0, "empty_content": 0, "connection_error": 0}
    for line in _LAUNCHER_LOG.read_text(errors="replace").splitlines():
        ts = line[:19]
        if len(line) < 19 or ts < lo or ts > hi:
            continue
        if "finish=length" in line:
            counts["finish=length"] += 1
        if '"content": ""' in line:
            counts["empty_content"] += 1
        if "ConnectionError" in line or "connection refused" in line.lower():
            counts["connection_error"] += 1

    out["checked"] = True
    out["window_local"] = [lo, hi]
    out["counts"] = counts
    for k, v in counts.items():
        if v:
            out["reasons"].append(f"{k} x{v}")
    out["valid"] = not any(counts.values())
    return out
