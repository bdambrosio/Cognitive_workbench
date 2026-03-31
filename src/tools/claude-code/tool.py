"""
Claude Code integration tool — delegate coding and research tasks to Claude Code CLI.

Supports two scopes via target:
  - "sandbox" (default): read-write access to the agent's own fs/src/ repo
  - "self": read-only access to the Cognitive Workbench source code

Each scope maintains its own persistent session via --resume.
"""

import json
import logging
import subprocess
from pathlib import Path
from typing import Any, Dict, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

# Session note names per scope
_SESSION_NOTES = {
    "sandbox": "_claude_code_session_sandbox",
    "self": "_claude_code_session_self",
}

# Read-only tools for self-inspection
_SELF_TOOLS = "Read,Glob,Grep"
# Full tools for sandbox work
_SANDBOX_TOOLS = "Bash,Read,Write,Edit,Glob,Grep"

_DEFAULT_MODEL = "sonnet"
_DEFAULT_MAX_TURNS = 10
_DEFAULT_MAX_BUDGET = 1.0
_DEFAULT_TIMEOUT = 300  # 5 minutes


def _fail(executor: InfospaceExecutor, reason: str,
          value: Optional[str] = None):
    return executor._create_uniform_return(
        "failed", value=value or reason, reason=reason)


def _success(executor: InfospaceExecutor, value: str,
             data: Any = None):
    return executor._create_uniform_return(
        "success", value=value, resource_id=None, data=data or value)


def _get_sandbox_dir(executive_node) -> Path:
    """Derive fs/src/ path from the resource manager's base_dir."""
    # base_dir is scenarios/<world>/resources/<agent>/
    # We want scenarios/<world>/fs/src/
    base = executive_node.resource_manager.base_dir
    scenario_dir = base.parent.parent  # up from resources/<agent>/ to scenario root
    return scenario_dir / "fs" / "src"


def _get_self_dir() -> Path:
    """Get the Cognitive Workbench src/ directory."""
    return Path(__file__).parent.parent.parent  # tools/claude-code/ -> tools/ -> src/


def _ensure_git_repo(directory: Path) -> None:
    """Ensure directory exists and is a git repo root."""
    directory.mkdir(parents=True, exist_ok=True)
    git_dir = directory / ".git"
    if not git_dir.exists():
        subprocess.run(
            ["git", "init"],
            cwd=str(directory),
            capture_output=True,
            timeout=10,
        )
        # Create initial commit so Claude Code has a valid repo
        subprocess.run(
            ["git", "commit", "--allow-empty", "-m", "Initial commit"],
            cwd=str(directory),
            capture_output=True,
            timeout=10,
        )
        logger.info(f"claude-code: Initialized git repo at {directory}")


def _get_session_id(executive_node, scope: str) -> Optional[str]:
    """Read stored session ID from named note."""
    note_name = _SESSION_NOTES[scope]
    rm = executive_node.resource_manager
    if not rm:
        return None
    note_id = rm.named_notes.get(note_name)
    if not note_id:
        return None
    try:
        res = rm.get_resource(note_id)
        content = res.get("properties", {}).get("content", "")
        data = json.loads(content) if content else {}
        return data.get("session_id")
    except Exception:
        return None


def _save_session_id(executive_node, executor, scope: str, session_id: str):
    """Store session ID in a named note."""
    note_name = _SESSION_NOTES[scope]
    content = json.dumps({"session_id": session_id, "scope": scope})
    executor.execute_action({
        "type": "create-note",
        "value": content,
        "name": note_name,
        "out": f"${note_name}",
    })
    executor.execute_action({
        "type": "persist",
        "target": f"${note_name}",
        "name": note_name,
    })


def _action_ask(executive_node, executor, scope, **kwargs) -> Dict:
    prompt = (kwargs.get("prompt") or "").strip()
    if not prompt:
        return _fail(executor, "prompt is required")

    if scope not in ("sandbox", "self"):
        return _fail(executor, f"Invalid target '{scope}'. Must be 'sandbox' or 'self'.")

    # Determine working directory
    if scope == "sandbox":
        work_dir = _get_sandbox_dir(executive_node)
        _ensure_git_repo(work_dir)
        allowed_tools = kwargs.get("allowed_tools") or _SANDBOX_TOOLS
    else:
        work_dir = _get_self_dir()
        allowed_tools = _SELF_TOOLS  # Always read-only for self

    if not work_dir.exists():
        return _fail(executor, f"Working directory does not exist: {work_dir}")

    # Build command
    model = kwargs.get("model") or _DEFAULT_MODEL
    max_turns = int(kwargs.get("max_turns") or _DEFAULT_MAX_TURNS)
    max_budget = float(kwargs.get("max_budget") or _DEFAULT_MAX_BUDGET)
    timeout = int(kwargs.get("timeout") or _DEFAULT_TIMEOUT)

    cmd = [
        "claude",
        "-p", prompt,
        "--output-format", "json",
        "--model", model,
        "--max-turns", str(max_turns),
        "--max-budget-usd", str(max_budget),
        "--allowedTools", allowed_tools,
    ]

    # Resume existing session if available
    session_id = _get_session_id(executive_node, scope)
    if session_id:
        cmd.extend(["--resume", session_id])

    logger.info(f"claude-code: asking ({scope}) in {work_dir}: {prompt[:100]}...")

    try:
        result = subprocess.run(
            cmd,
            cwd=str(work_dir),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return _fail(executor, f"Claude Code timed out after {timeout}s")
    except FileNotFoundError:
        return _fail(executor, "Claude Code CLI not found. Is 'claude' in PATH?")
    except Exception as e:
        return _fail(executor, f"Failed to run Claude Code: {e}")

    if result.returncode != 0:
        stderr = (result.stderr or "").strip()[:500]
        # If resume failed (stale session), retry without resume
        if session_id and ("session" in stderr.lower() or "resume" in stderr.lower()):
            logger.warning(f"claude-code: Session {session_id} stale, starting fresh")
            cmd = [c for c in cmd if c not in ("--resume", session_id)]
            try:
                result = subprocess.run(
                    cmd,
                    cwd=str(work_dir),
                    capture_output=True,
                    text=True,
                    timeout=timeout,
                )
            except Exception as e:
                return _fail(executor, f"Retry failed: {e}")
            if result.returncode != 0:
                return _fail(executor, f"Claude Code failed: {(result.stderr or '').strip()[:500]}")
        else:
            return _fail(executor, f"Claude Code failed (exit {result.returncode}): {stderr}")

    # Parse JSON response
    try:
        response = json.loads(result.stdout)
    except json.JSONDecodeError:
        # Sometimes output has non-JSON prefix; try to extract
        stdout = result.stdout.strip()
        start = stdout.find("{")
        if start >= 0:
            try:
                response = json.loads(stdout[start:])
            except json.JSONDecodeError:
                return _fail(executor,
                             f"Could not parse Claude Code response",
                             value=stdout[:1000])
        else:
            return _fail(executor,
                         f"Could not parse Claude Code response",
                         value=stdout[:1000])

    # Save session ID for future resume
    new_session_id = response.get("session_id")
    if new_session_id:
        _save_session_id(executive_node, executor, scope, new_session_id)

    # Extract the response text
    answer = response.get("result", "")
    cost = response.get("cost", 0)
    usage = response.get("usage", {})

    logger.info(f"claude-code: response ({scope}), "
                f"cost=${cost:.4f}, "
                f"tokens={usage.get('output', '?')}")

    summary = answer if answer else "(empty response)"
    return _success(executor, value=summary, data={
        "result": answer,
        "session_id": new_session_id,
        "cost": cost,
        "scope": scope,
    })


def _action_reset(executive_node, executor, scope, **kwargs) -> Dict:
    """Reset (clear) a session so the next ask starts fresh."""
    if scope not in ("sandbox", "self"):
        return _fail(executor, f"Invalid target '{scope}'.")

    note_name = _SESSION_NOTES[scope]
    rm = executive_node.resource_manager
    if rm:
        note_id = rm.named_notes.get(note_name)
        if note_id:
            executor.execute_action({
                "type": "create-note",
                "value": json.dumps({"session_id": None, "scope": scope}),
                "name": note_name,
                "out": f"${note_name}",
            })
            executor.execute_action({
                "type": "persist",
                "target": f"${note_name}",
                "name": note_name,
            })

    return _success(executor, value=f"Claude Code session reset for '{scope}'")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

_ACTIONS = {
    "ask": _action_ask,
    "reset": _action_reset,
}


def tool(input_value=None, runtime=None, **kwargs):
    """Delegate tasks to Claude Code CLI.

    target (resolved by executor as input_value): "sandbox" or "self"
    prompt: the instruction to send to Claude Code
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    executive_node = kwargs.get("executive_node")
    if not executive_node:
        return _fail(executor, "executive_node not available")

    # target arrives as input_value, default to sandbox
    scope = (input_value or "sandbox").strip().lower()

    # Default action is "ask" — most common use
    action = (kwargs.get("action") or "ask").strip().lower()
    if action not in _ACTIONS:
        return _fail(executor,
                     f"Unknown action '{action}'. Must be one of: {', '.join(_ACTIONS)}")

    try:
        action_kwargs = {k: v for k, v in kwargs.items()
                        if k not in ("executive_node", "executor", "action")}
        return _ACTIONS[action](executive_node, executor, scope, **action_kwargs)
    except Exception as e:
        logger.error(f"claude-code {action} failed: {e}", exc_info=True)
        return _fail(executor, f"{action} failed: {e}")
