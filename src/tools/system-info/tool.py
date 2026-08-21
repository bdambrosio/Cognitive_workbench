"""system-info — what the RUNTIME can see about the machine and the backend.

The agent's only other window onto its own substrate is `exec-script`, which
runs under `bwrap --unshare-all` with a minimal /dev. That sandbox answers
some hardware questions truthfully (memory and CPU, because /proc is mounted)
and others falsely (no /dev/nvidia* nodes, so `nvidia-smi` reports the driver
unreachable on a machine whose GPUs are healthy and busy), with nothing
marking which is which. This tool runs in-process, unsandboxed, so its
answers are about the real machine.

Deliberately NOT here: environment variable VALUES. The environment holds
OPENAI_API_KEY, CLAUDE_API_KEY, X_BEARER_TOKEN and GOOGLE_API_KEY, and a tool
that returned them would put them in the working log, then in
reasoning_trace.jsonl on disk, and potentially into an agent-say to a peer.
`runtime` reports which credentials are CONFIGURED, by name, never their
contents — which answers "can I do this" without handing anyone the key.
"""

import json
import logging
import os
import subprocess
from pathlib import Path

logger = logging.getLogger(__name__)

_HEALTH_SCRIPT = Path(__file__).resolve().parent.parent.parent / "scripts" / "system-health.sh"

# Reported as present/absent only. Names, never values.
_CREDENTIALS = ("OPENAI_API_KEY", "CLAUDE_API_KEY", "GOOGLE_API_KEY",
                "GOOGLE_CX", "X_BEARER_TOKEN", "X_LIST_ID",
                "ALPHAVANTAGE_API_KEY")

_COMPONENTS = ("gpu", "system", "runtime", "all")


def _gpus():
    """Every CUDA device, not just the first.

    system-health.sh pipes nvidia-smi through `head -1`, which on a two-card
    machine reports one card and says nothing about the omission. Here that
    silently returned the idle 16GB card while the 98GB card was at 99%
    serving this agent's own inference.
    """
    try:
        r = subprocess.run(
            ["nvidia-smi",
             "--query-gpu=index,name,memory.used,memory.total,utilization.gpu,temperature.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=15)
    except FileNotFoundError:
        return {"available": False, "reason": "nvidia-smi not installed"}
    except subprocess.TimeoutExpired:
        return {"available": False, "reason": "nvidia-smi timed out"}
    if r.returncode != 0:
        return {"available": False,
                "reason": (r.stderr or "nvidia-smi failed").strip()[:200]}
    devices = []
    for line in (r.stdout or "").strip().splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 6:
            continue
        idx, name, used, total, util, temp = parts
        devices.append({
            "index": idx, "name": name,
            "vram_used_mib": used, "vram_total_mib": total,
            "utilization_pct": util, "temperature_c": temp,
        })
    return {"available": bool(devices), "count": len(devices),
            "devices": devices,
            "note": "index is nvidia-smi order (PCI); a framework using "
                    "fastest-first ordering may number these differently"}


def _system():
    """CPU / RAM / disk / network, via the existing health script."""
    if not _HEALTH_SCRIPT.exists():
        return {"error": f"system-health.sh not found at {_HEALTH_SCRIPT}"}
    try:
        r = subprocess.run(["bash", str(_HEALTH_SCRIPT)], capture_output=True,
                           text=True, timeout=30,
                           cwd=str(_HEALTH_SCRIPT.parent))
        if r.returncode != 0:
            return {"error": f"system-health.sh exited {r.returncode}"}
        data = json.loads(r.stdout)
    except subprocess.TimeoutExpired:
        return {"error": "system-health.sh timed out"}
    except json.JSONDecodeError as e:
        return {"error": f"system-health.sh produced invalid JSON: {e}"}
    except Exception as e:                       # noqa: BLE001 - reported, not swallowed
        logger.warning(f"system-info: system-health.sh failed: {e}")
        return {"error": str(e)}
    # Its `gpu` key is the truncated single-card view; _gpus() supersedes it.
    data.pop("gpu", None)
    return data


def _runtime(backend, character_name):
    """Which weights are answering, where they live, and what is configured.

    Nothing in the prompt names the active model — the self-model block says
    what the character is, not which weights are serving it — so without this
    an agent asked to inventory its own capabilities has to guess.
    """
    out = {"character": character_name or "unknown"}
    if backend is None:
        out["backend"] = "not available to this tool call"
    else:
        out["backend"] = {
            "model": getattr(backend, "model", "") or "(unset)",
            "base_url": getattr(backend, "base_url", "") or "(unset)",
            "is_cloud": bool(getattr(backend, "is_cloud", False)),
            "is_reasoning": bool(getattr(backend, "is_reasoning", False)),
            "reasoning_effort": getattr(backend, "reasoning_effort", None),
        }
    out["credentials_configured"] = {
        name: bool(os.environ.get(name)) for name in _CREDENTIALS
    }
    out["credentials_note"] = ("presence only — values are never returned by "
                               "this tool")
    return out


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    component = str(args.get("component") or "all").strip().lower()
    if component not in _COMPONENTS:
        return {"status": "error",
                "text": (f"system-info: unknown component {component!r}; "
                         f"use one of {', '.join(_COMPONENTS)}")}

    report = {}
    if component in ("gpu", "all"):
        report["gpu"] = _gpus()
    if component in ("system", "all"):
        report["system"] = _system()
    if component in ("runtime", "all"):
        report["runtime"] = _runtime(backend, character_name)

    return {"status": "ok", "text": json.dumps(report, indent=2)}
