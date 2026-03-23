"""
Execute an arbitrary bash script in the scenario filesystem.

Script text comes from target arg (string literal or Note binding).
Asks the user for permission before executing. Runs in scenarios/<world_name>/fs/.
"""

import logging
import subprocess
from pathlib import Path
from typing import Any, Dict, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None,
          extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=value or reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=value, extra=extra)


def tool(input_value=None, runtime=None, **kwargs):
    """
    Execute an arbitrary bash script in scenarios/<world_name>/fs/.

    input_value is the resolved target: either a literal script string
    or the content of a bound Note.  Collections are rejected upstream
    by the executor, but we guard against it here too.
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    world_name = kwargs.get("world_name") or getattr(executor, "world_name", None)
    if not world_name:
        return _fail(executor, "world_name not available — cannot determine working directory")

    # ── Resolve script text ──────────────────────────────────────────
    # The executor's _execute_apply already resolves target via
    # _resolve_value, handing us the content as input_value.
    # However, we need to guard against Collection bindings: if the
    # caller passed a $var that points to a Collection, the resolved
    # input_value will be the flattened concatenation — detect that by
    # checking the original action's target binding kind.
    target_arg = kwargs.get("target")
    if target_arg and isinstance(target_arg, str) and target_arg.startswith("$"):
        var_name = target_arg.lstrip("$")
        kind = executor._get_kind(var_name)
        if kind == "Collection":
            return _fail(executor,
                         f"target ${var_name} is a Collection — exec-script requires a Note or literal string")

    script_text = input_value
    if not script_text or not isinstance(script_text, str) or not script_text.strip():
        return _fail(executor, "exec-script requires non-empty script text in target")

    script_text = script_text.strip()

    # ── Ask user for permission ──────────────────────────────────────
    ask_result = executor._execute_ask({
        "type": "ask",
        "target": "User",
        "value": f"exec-script wants to run the following script in scenarios/{world_name}/fs/:\n\n```\n{script_text}\n```\n\nProceed? (yes/no)",
    })

    if ask_result.get("status") != "success":
        return _fail(executor, "Permission request failed or timed out",
                     value=ask_result.get("reason", "ask failed"))

    response = (ask_result.get("value") or "").strip().lower()
    if response not in ("yes", "y"):
        return _fail(executor, "User denied permission to execute script",
                     value="denied")

    # ── Execute the script ───────────────────────────────────────────
    fs_dir = _PROJECT_ROOT / "scenarios" / world_name / "fs"
    if not fs_dir.is_dir():
        return _fail(executor, f"Working directory does not exist: {fs_dir}")

    try:
        result = subprocess.run(
            ["bash", "-c", script_text],
            cwd=str(fs_dir),
            capture_output=True,
            text=True,
            timeout=120,
        )
        output = result.stdout
        if result.stderr:
            output = output + ("\n" if output else "") + result.stderr

        if not output:
            output = f"(no output, exit code {result.returncode})"

        logger.info(f"exec-script: exit={result.returncode}, output length={len(output)}")
        return _success(executor, output, extra={"exit_code": result.returncode})

    except subprocess.TimeoutExpired:
        return _fail(executor, "Script timed out after 120 seconds")
    except Exception as e:
        logger.error(f"exec-script execution error: {e}", exc_info=True)
        return _fail(executor, f"Script execution failed: {e}")
