"""
Homeostasis health monitor for Cognitive Workbench.

Collects two categories of health signals:
  1. System (hardware/OS) — via system-health.sh subprocess
  2. Cognitive (memory, tools, world model, planning) — via Python internals

Returns a structured report with per-signal status classifications.
"""

import json
import logging
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

_SCRIPTS_DIR = Path(__file__).resolve().parent.parent.parent / "scripts"
_HEALTH_SCRIPT = _SCRIPTS_DIR / "system-health.sh"

# Cognitive thresholds
_TOOL_SUCCESS_RATE_WARN = 0.70
_WORLD_MODEL_CONTRADICTION_WARN = 0.30
_USER_IDLE_WARN_HOURS = 24


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _fail(executor: InfospaceExecutor, reason: str,
          value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed", value=value or reason, reason=reason, extra=extra,
    )


def _success(executor: InfospaceExecutor, value: str,
             data: Any = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "success", value=value, resource_id=None, extra=extra, data=data or value,
    )


# ---------------------------------------------------------------------------
# System health (delegates to bash script)
# ---------------------------------------------------------------------------

def _collect_system_health() -> Dict[str, Any]:
    """Run system-health.sh and parse JSON output."""
    if not _HEALTH_SCRIPT.exists():
        logger.warning(f"system-health.sh not found at {_HEALTH_SCRIPT}")
        return {"error": "system-health.sh not found"}

    try:
        result = subprocess.run(
            ["bash", str(_HEALTH_SCRIPT)],
            capture_output=True, text=True, timeout=15,
            cwd=str(_SCRIPTS_DIR),
        )
        if result.returncode != 0:
            logger.warning(f"system-health.sh exited {result.returncode}: {result.stderr[:200]}")
            return {"error": f"script exited {result.returncode}"}

        return json.loads(result.stdout)
    except subprocess.TimeoutExpired:
        return {"error": "system-health.sh timed out"}
    except json.JSONDecodeError as e:
        logger.warning(f"system-health.sh produced invalid JSON: {e}")
        return {"error": "invalid JSON from script"}
    except Exception as e:
        logger.warning(f"system-health.sh failed: {e}")
        return {"error": str(e)}


# ---------------------------------------------------------------------------
# Cognitive health (reads internal state)
# ---------------------------------------------------------------------------

def _collect_cognitive_health(executor: InfospaceExecutor,
                              resource_manager, executive_node) -> Dict[str, Any]:
    """Gather cognitive metrics from internal state."""
    report = {}

    # --- Memory integrity ---
    try:
        note_count = getattr(resource_manager, 'note_counter', 0)
        collection_count = getattr(resource_manager, 'collection_counter', 0)
        relation_count = getattr(resource_manager, 'relation_counter', 0)
        report["memory"] = {
            "note_count": note_count,
            "collection_count": collection_count,
            "relation_count": relation_count,
            "status": "ok",
        }
    except Exception as e:
        report["memory"] = {"status": "warning", "error": str(e)}

    # --- Tool model health ---
    try:
        tool_model = getattr(executive_node, 'tool_model', None)
        if tool_model and hasattr(tool_model, 'tool_model'):
            records = tool_model.tool_model.get("training_records", [])
            total = len(records)
            successes = sum(1 for r in records if r.get("tool_status") == "success")
            rate = successes / total if total > 0 else 1.0
            status = "warning" if total > 5 and rate < _TOOL_SUCCESS_RATE_WARN else "ok"
            report["tool_model"] = {
                "total_records": total,
                "success_count": successes,
                "success_rate": round(rate, 3),
                "status": status,
            }
        else:
            report["tool_model"] = {"status": "ok", "note": "no tool model loaded"}
    except Exception as e:
        report["tool_model"] = {"status": "warning", "error": str(e)}

    # --- World model health ---
    try:
        world_model_mgr = getattr(executive_node, 'world_model', None)
        if world_model_mgr and hasattr(world_model_mgr, 'world_model'):
            raw_data = world_model_mgr.world_model.get("raw_data", {})
            facts = raw_data.get("facts", [])
            total_facts = len(facts)
            total_support = sum(f.get("support_count", 0) for f in facts)
            total_contradiction = sum(f.get("contradiction_count", 0) for f in facts)
            denom = total_support + total_contradiction
            contradiction_ratio = total_contradiction / denom if denom > 0 else 0.0
            status = "warning" if contradiction_ratio > _WORLD_MODEL_CONTRADICTION_WARN else "ok"
            report["world_model"] = {
                "fact_count": total_facts,
                "support_total": total_support,
                "contradiction_total": total_contradiction,
                "contradiction_ratio": round(contradiction_ratio, 3),
                "status": status,
            }
        else:
            report["world_model"] = {"status": "ok", "note": "no world model loaded"}
    except Exception as e:
        report["world_model"] = {"status": "warning", "error": str(e)}

    # --- Planning health (recent compressed traces) ---
    try:
        if resource_manager and hasattr(resource_manager, 'base_dir'):
            trace_dir = resource_manager.base_dir / "planner_history"
            trace_file = trace_dir / "compressed_trace.jsonl"
            if trace_file.exists():
                lines = trace_file.read_text().strip().split('\n')
                recent = lines[-20:] if len(lines) > 20 else lines
                completed = 0
                for line in recent:
                    try:
                        record = json.loads(line)
                        trace_text = record.get("compressed_trace", "")
                        # Completion is marked as "DONE: YES" (with varying whitespace) inside the trace text
                        if re.search(r'DONE:\s*YES', trace_text):
                            completed += 1
                    except json.JSONDecodeError:
                        continue
                total_plans = len(recent)
                rate = completed / total_plans if total_plans > 0 else 1.0
                status = "warning" if total_plans > 3 and rate < 0.5 else "ok"
                report["planning"] = {
                    "recent_plans_checked": total_plans,
                    "completed_count": completed,
                    "completion_rate": round(rate, 3),
                    "status": status,
                }
            else:
                report["planning"] = {"status": "ok", "note": "no trace history yet"}
        else:
            report["planning"] = {"status": "ok", "note": "no resource base dir"}
    except Exception as e:
        report["planning"] = {"status": "ok", "error": str(e)}

    # --- User interaction recency ---
    try:
        conv_store = getattr(executive_node, 'conversation_store', None)
        if conv_store:
            user_turns = conv_store.get_recent_turns("User", limit=1, scope="all")
            if user_turns:
                last_turn = user_turns[-1]
                ts_str = last_turn.get("timestamp", "")
                if ts_str:
                    last_ts = datetime.fromisoformat(ts_str)
                    now = datetime.now()
                    hours_ago = (now - last_ts).total_seconds() / 3600
                    status = "warning" if hours_ago > _USER_IDLE_WARN_HOURS else "ok"
                    report["user_interaction"] = {
                        "last_interaction_hours_ago": round(hours_ago, 1),
                        "status": status,
                    }
                else:
                    report["user_interaction"] = {"status": "ok", "note": "no timestamp on last turn"}
            else:
                report["user_interaction"] = {"status": "ok", "note": "no user turns recorded"}
        else:
            report["user_interaction"] = {"status": "ok", "note": "no conversation store"}
    except Exception as e:
        report["user_interaction"] = {"status": "ok", "error": str(e)}

    return report


# ---------------------------------------------------------------------------
# Report assembly
# ---------------------------------------------------------------------------

def _extract_concerns(system: Dict, cognitive: Dict) -> List[str]:
    """Walk all subsections and collect human-readable concern strings."""
    concerns = []

    # System concerns
    if "error" in system:
        concerns.append(f"System health script error: {system['error']}")
    else:
        gpu = system.get("gpu", {})
        if gpu.get("temperature_status") in ("warning", "critical"):
            concerns.append(f"GPU temperature {gpu.get('temperature_c')}C ({gpu['temperature_status']})")
        if gpu.get("vram_status") in ("warning", "critical"):
            concerns.append(f"GPU VRAM at {gpu.get('vram_used_pct')}% ({gpu['vram_status']})")

        cpu = system.get("cpu", {})
        if cpu.get("temperature_status") in ("warning", "critical"):
            concerns.append(f"CPU temperature {cpu.get('temperature_c')}C ({cpu['temperature_status']})")
        if cpu.get("load_status") == "warning":
            concerns.append(f"CPU load high: {cpu.get('load_5m')} (5m avg, {cpu.get('cores')} cores)")

        ram = system.get("ram", {})
        if ram.get("ram_status") in ("warning", "critical"):
            concerns.append(f"RAM usage at {ram.get('used_pct')}%")
        if ram.get("swap_status") in ("warning", "critical"):
            concerns.append(f"Swap usage at {ram.get('swap_used_pct')}%")

        for disk in system.get("disk", []):
            if disk.get("status") in ("warning", "critical"):
                concerns.append(f"Disk {disk.get('mount')} at {disk.get('used_pct')}% ({disk['status']})")

        net = system.get("network", {})
        if net.get("network_status") != "ok":
            parts = []
            if not net.get("default_route"):
                parts.append("no default route")
            if not net.get("dns_resolution"):
                parts.append("DNS resolution failed")
            concerns.append(f"Network issue: {', '.join(parts)}")

        procs = system.get("processes", {})
        if procs.get("process_status") != "ok":
            down = []
            if not procs.get("zenoh_router"):
                down.append("Zenoh router")
            if not procs.get("fastapi_ui"):
                down.append("FastAPI UI")
            if not procs.get("resource_browser"):
                down.append("Resource browser")
            if down:
                concerns.append(f"Process(es) down: {', '.join(down)}")

    # Cognitive concerns
    mem = cognitive.get("memory", {})
    if mem.get("status") != "ok":
        concerns.append(f"Memory subsystem issue: {mem.get('error', 'unknown')}")

    tm = cognitive.get("tool_model", {})
    if tm.get("status") == "warning":
        concerns.append(
            f"Tool success rate low: {tm.get('success_rate', 0):.0%} "
            f"({tm.get('success_count', 0)}/{tm.get('total_records', 0)} records)"
        )

    wm = cognitive.get("world_model", {})
    if wm.get("status") == "warning":
        concerns.append(
            f"World model contradiction ratio high: {wm.get('contradiction_ratio', 0):.1%} "
            f"({wm.get('contradiction_total', 0)} contradictions)"
        )

    plan = cognitive.get("planning", {})
    if plan.get("status") == "warning":
        concerns.append(
            f"Plan completion rate low: {plan.get('completion_rate', 0):.0%} "
            f"(last {plan.get('recent_plans_checked', 0)} plans)"
        )

    ui = cognitive.get("user_interaction", {})
    if ui.get("status") == "warning":
        concerns.append(
            f"No user interaction for {ui.get('last_interaction_hours_ago', 0):.0f} hours"
        )

    return concerns


def _overall_status(system: Dict, cognitive: Dict) -> str:
    """Determine worst-case status across all signals."""
    all_statuses = set()

    # System statuses
    if "error" not in system:
        for key in ("gpu", "cpu", "ram", "network", "processes"):
            section = system.get(key, {})
            for k, v in section.items():
                if k.endswith("_status") and isinstance(v, str):
                    all_statuses.add(v)
        for disk in system.get("disk", []):
            all_statuses.add(disk.get("status", "ok"))

    # Cognitive statuses
    for section in cognitive.values():
        if isinstance(section, dict):
            all_statuses.add(section.get("status", "ok"))

    if "critical" in all_statuses:
        return "critical"
    if "warning" in all_statuses:
        return "warning"
    return "ok"


# ---------------------------------------------------------------------------
# Main tool entry point
# ---------------------------------------------------------------------------

def tool(input_value=None, runtime=None, **kwargs):
    """
    Homeostasis health check tool.

    Collects system and cognitive health metrics, classifies each signal,
    and returns a structured report for the planner to act on.
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    resource_manager = kwargs.get("resource_manager")
    executive_node = kwargs.get("executive_node")

    # Collect both domains
    system_health = _collect_system_health()
    cognitive_health = _collect_cognitive_health(executor, resource_manager, executive_node)

    # Analyze
    concerns = _extract_concerns(system_health, cognitive_health)
    overall = _overall_status(system_health, cognitive_health)

    # Build human-readable summary
    if not concerns:
        summary = "All systems nominal."
    else:
        summary = f"Health check ({overall}): " + "; ".join(concerns)

    # Full structured report
    full_report = {
        "overall_status": overall,
        "concerns": concerns,
        "system": system_health,
        "cognitive": cognitive_health,
    }

    logger.info(f"check-health: overall={overall}, concerns={len(concerns)}")

    return _success(executor, value=summary, data=full_report,
                    extra={"overall_status": overall, "concerns": concerns})
