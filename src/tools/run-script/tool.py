"""
Submit a shell script from src/scripts/ for execution.

Accepts script name (without .sh), looks for src/scripts/{name}.sh,
submits to bash. Successful submission = success. Fails if script not found.
"""

import logging
import re
import subprocess
from pathlib import Path
from typing import Any, Dict, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

_SRC_DIR = Path(__file__).resolve().parent.parent.parent
_SCRIPTS_DIR = _SRC_DIR / "scripts"


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=value or reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str, data: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=value, resource_id=None, extra=extra, data=data or value)


def tool(input_value=None, runtime=None, **kwargs):
    """
    Submit a shell script from src/scripts/. Looks up {script_name}.sh and submits to bash.
    Successful submission = success.
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    script_name = (kwargs.get("script_name") or input_value or "").strip()
    if not script_name:
        return _fail(executor, "script_name required (e.g. daily_post)")

    if not re.fullmatch(r"[a-zA-Z0-9_-]+", script_name):
        return _fail(executor, "script_name must be alphanumeric, hyphens, underscores only")

    script_path = _SCRIPTS_DIR / f"{script_name}.sh"
    if not script_path.exists():
        return _fail(executor, f"script not found: {script_path}")

    log_dir = _SRC_DIR / "logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"run-script-{script_name}.log"
    try:
        log_file = open(log_path, "a")
        subprocess.Popen(
            ["bash", str(script_path)],
            cwd=str(_SCRIPTS_DIR),
            stdout=log_file,
            stderr=log_file,
            start_new_session=True,
        )
        log_file.close()
    except Exception as e:
        logger.error(f"run-script failed to start: {e}")
        return _fail(executor, str(e), extra={"script": str(script_path)})

    msg = f"Script {script_name} started"
    logger.info(f"run-script: {script_path} started")
    return _success(executor, msg, data=msg, extra={"script": str(script_path)})
