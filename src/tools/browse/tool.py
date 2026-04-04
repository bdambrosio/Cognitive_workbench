"""
Browser automation tool wrapping the agent-browser CLI.

Each call executes `agent-browser <command> [args]` as a subprocess.
The browser daemon persists between calls (session scoped to agent name).
Snapshot returns a Note with the accessibility tree; other actions return
their output as a value string.

Requires: agent-browser CLI on PATH (https://github.com/nichochar/agent-browser)
"""

import json
import logging
import os
import subprocess
from typing import Any, Dict, List, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

_TIMEOUT = 30  # seconds per command
_BATCH_TIMEOUT = 60  # seconds for batch operations


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None,
          extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed", value=value or reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str,
             resource_id: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "success", value=value, resource_id=resource_id, extra=extra)


def _run(args: List[str], env: Dict[str, str], timeout: int = _TIMEOUT) -> subprocess.CompletedProcess:
    """Run agent-browser with given args."""
    return subprocess.run(
        ["agent-browser"] + args,
        capture_output=True, text=True, timeout=timeout, env=env,
    )


def _session_env(agent_name: str) -> Dict[str, str]:
    """Build env dict with session scoped to agent."""
    env = os.environ.copy()
    env["AGENT_BROWSER_SESSION"] = agent_name or "default"
    return env


def _create_note(text: str, agent_name: str, resource_manager,
                 source_skill: str = 'browse',
                 tool_metadata: Optional[Dict] = None) -> Optional[str]:
    """Create a Note from text content."""
    if not resource_manager:
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name, text, 'text', source_skill, (text or '')[:100], '',
        {'tool_metadata': tool_metadata or {}})
    return note_id if success else None


# ── Action handlers ─────────────────────────────────────────────────

def _do_open(kwargs, env, executor):
    url = kwargs.get('url', '')
    if not url:
        return _fail(executor, "open requires 'url' parameter")
    result = _run(["open", url], env)
    if result.returncode != 0:
        return _fail(executor, f"open failed: {(result.stderr or result.stdout)[:500]}")
    return _success(executor, result.stdout.strip() or f"Opened {url}")


def _do_snapshot(kwargs, env, executor, agent_name, resource_manager):
    args = ["snapshot", "-i", "-c", "--json"]
    selector = kwargs.get('selector')
    if selector:
        args.extend(["-s", selector])
    result = _run(args, env)
    if result.returncode != 0:
        return _fail(executor, f"snapshot failed: {(result.stderr or result.stdout)[:500]}")
    content = result.stdout.strip()
    if not content:
        return _fail(executor, "snapshot returned empty content")
    note_id = _create_note(content, agent_name, resource_manager,
                           tool_metadata={"action": "snapshot"})
    if not note_id:
        return _fail(executor, "Failed to create Note from snapshot")
    preview = content[:200] + "..." if len(content) > 200 else content
    return _success(executor, preview, resource_id=note_id)


def _do_click(kwargs, env, executor):
    selector = kwargs.get('selector', '')
    if not selector:
        return _fail(executor, "click requires 'selector' parameter")
    result = _run(["click", selector], env)
    if result.returncode != 0:
        return _fail(executor, f"click failed: {(result.stderr or result.stdout)[:500]}")
    return _success(executor, result.stdout.strip() or "clicked")


def _do_type_or_fill(action, kwargs, env, executor):
    selector = kwargs.get('selector', '')
    text = kwargs.get('text', '')
    if not selector:
        return _fail(executor, f"{action} requires 'selector' parameter")
    if not text:
        return _fail(executor, f"{action} requires 'text' parameter")
    result = _run([action, selector, text], env)
    if result.returncode != 0:
        return _fail(executor, f"{action} failed: {(result.stderr or result.stdout)[:500]}")
    return _success(executor, result.stdout.strip() or f"{action} done")


def _do_press(kwargs, env, executor):
    key = kwargs.get('key', '')
    if not key:
        return _fail(executor, "press requires 'key' parameter")
    result = _run(["press", key], env)
    if result.returncode != 0:
        return _fail(executor, f"press failed: {(result.stderr or result.stdout)[:500]}")
    return _success(executor, result.stdout.strip() or f"pressed {key}")


def _do_scroll(kwargs, env, executor):
    direction = kwargs.get('text', 'down')
    args = ["scroll", direction]
    selector = kwargs.get('selector')
    if selector:
        args.extend(["--selector", selector])
    result = _run(args, env)
    if result.returncode != 0:
        return _fail(executor, f"scroll failed: {(result.stderr or result.stdout)[:500]}")
    return _success(executor, result.stdout.strip() or f"scrolled {direction}")


def _do_get(kwargs, env, executor):
    subcommand = kwargs.get('subcommand', '')
    if not subcommand:
        return _fail(executor, "get requires 'subcommand' (text|html|value|title|url)")
    args = ["get", subcommand]
    if subcommand in ('text', 'html', 'value'):
        selector = kwargs.get('selector', '')
        if not selector:
            return _fail(executor, f"get {subcommand} requires 'selector' parameter")
        args.append(selector)
    args.append("--json")
    result = _run(args, env)
    if result.returncode != 0:
        return _fail(executor, f"get failed: {(result.stderr or result.stdout)[:500]}")
    return _success(executor, result.stdout.strip())


def _do_eval(kwargs, env, executor):
    expression = kwargs.get('expression', '')
    if not expression:
        return _fail(executor, "eval requires 'expression' parameter")
    result = _run(["eval", expression], env)
    if result.returncode != 0:
        return _fail(executor, f"eval failed: {(result.stderr or result.stdout)[:500]}")
    return _success(executor, result.stdout.strip())


def _do_close(env, executor):
    result = _run(["close"], env, timeout=10)
    return _success(executor, "session closed")


def _do_batch(kwargs, env, executor, agent_name, resource_manager):
    actions = kwargs.get('actions')
    if not actions or not isinstance(actions, list):
        return _fail(executor, "batch requires 'actions' parameter (list of [command, ...args] arrays)")
    # Validate structure
    for i, cmd in enumerate(actions):
        if not isinstance(cmd, list) or not cmd:
            return _fail(executor, f"batch actions[{i}] must be a non-empty array")

    input_json = json.dumps(actions)
    try:
        result = subprocess.run(
            ["agent-browser", "batch", "--json", "--bail"],
            input=input_json, capture_output=True, text=True,
            timeout=_BATCH_TIMEOUT, env=env,
        )
    except subprocess.TimeoutExpired:
        return _fail(executor, f"batch timed out after {_BATCH_TIMEOUT}s")

    if result.returncode != 0:
        return _fail(executor, f"batch failed: {(result.stderr or result.stdout)[:500]}")

    content = result.stdout.strip()
    if not content:
        return _fail(executor, "batch returned empty output")

    # If last command was snapshot, create a Note
    last_cmd = actions[-1][0] if actions else ""
    if last_cmd == "snapshot":
        note_id = _create_note(content, agent_name, resource_manager,
                               tool_metadata={"action": "batch", "commands": len(actions)})
        if note_id:
            preview = content[:200] + "..." if len(content) > 200 else content
            return _success(executor, preview, resource_id=note_id)

    return _success(executor, content[:500])


# ── Main entry point ────────────────────────────────────────────────

def tool(input_value=None, **kwargs):
    """Browser automation via agent-browser CLI."""
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    action = kwargs.get('action') or input_value or ''
    if not action:
        return _fail(executor, "browse requires 'action' parameter")

    agent_name = kwargs.get('agent_name', 'default')
    resource_manager = kwargs.get('resource_manager')
    env = _session_env(agent_name)

    if action == 'open':
        return _do_open(kwargs, env, executor)
    elif action == 'snapshot':
        return _do_snapshot(kwargs, env, executor, agent_name, resource_manager)
    elif action == 'click':
        return _do_click(kwargs, env, executor)
    elif action in ('type', 'fill'):
        return _do_type_or_fill(action, kwargs, env, executor)
    elif action == 'press':
        return _do_press(kwargs, env, executor)
    elif action == 'scroll':
        return _do_scroll(kwargs, env, executor)
    elif action == 'get':
        return _do_get(kwargs, env, executor)
    elif action == 'eval':
        return _do_eval(kwargs, env, executor)
    elif action == 'close':
        return _do_close(env, executor)
    elif action == 'batch':
        return _do_batch(kwargs, env, executor, agent_name, resource_manager)
    else:
        return _fail(executor, f"Unknown action '{action}'. Use: open|snapshot|click|type|fill|press|scroll|get|eval|close|batch")
