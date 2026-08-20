"""
Execute an arbitrary bash script inside a bubblewrap sandbox.

Script text comes from target arg (string literal or Note binding).

The safeguard is containment, not judgement: nothing inspects or approves
the script, it simply runs somewhere it cannot do harm. The sandbox gives
it a read-only view of the repo and the system toolchain, no network, no
HOME, and an empty environment. It can write to exactly two places: a
per-character `scratch/` directory that persists between calls (so a
script can be built, run, and fixed across ReAct iterations, and so code
that *is* the product has somewhere to land), and a private /tmp that
dies with the process. If bwrap is missing the tool refuses to run rather
than falling back to a bare shell.

The invariant that makes the persistent directory safe: nothing outside
the sandbox ever executes what is inside `scratch/`. Tool discovery scans
`src/tools/` only, and self-extension stages through its own directory
behind a human promote-and-restart gate. If anything is ever pointed at
`scratch/`, this tool stops being contained.
"""

import logging
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, Optional

# infospace_executor was the planner-side runtime; the chat ReAct loop
# bypasses it. Keep the import optional so this module loads in chat-only
# builds — the InfospaceExecutor annotations degrade to Any there.
try:
    from infospace_executor import InfospaceExecutor
except ImportError:
    InfospaceExecutor = Any  # type: ignore[assignment,misc]

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent

# Environment handed to the script. Deliberately tiny: every API key this
# process holds (OPENAI_, TAVILY_, GMAIL_APP_PASSWORD, ...) arrives through
# os.environ, so --clearenv is what keeps a script from reading them out.
_SANDBOX_ENV = {"PATH": "/usr/bin:/bin", "HOME": "/tmp", "LANG": "C.UTF-8"}

# The embedder InfospaceResourceManager lazy-loads (BAAI/bge-small-en-v1.5)
# normally resolves out of ~/.cache, which the sandbox does not bind, and
# cannot be downloaded there either — no network. Pre-seeded in-repo
# instead, so note/concern/memory code paths work rather than failing with
# a silently-degraded "create failed" that looks like a real regression.
_SANDBOX_HF_HOME = _PROJECT_ROOT / "sandbox_models"

# Interpreter for `python3` inside the sandbox. A venv built against
# /usr/bin/python3 works because the real interpreter lives under the
# read-only /usr bind; only its site-packages sit in the repo. (This is
# why zenoh_venv is unusable there — its python symlinks into $HOME,
# which is deliberately not bound.) Absent, the script falls back to the
# bare system python.
_SANDBOX_VENV_BIN = _PROJECT_ROOT / "sandbox_venv" / "bin"


def _scratch_dir(agent_name):
    """Per-character persistent writable directory, created on demand.

    Character-scoped rather than shared so two agents running in the same
    world (Jill and Jack in coord_search) cannot overwrite each other's
    working files.
    """
    safe = "".join(c for c in str(agent_name or "") if c.isalnum() or c in "-_")
    path = _PROJECT_ROOT / "scratch" / (safe or "shared")
    path.mkdir(parents=True, exist_ok=True)
    return path


def _sandbox_argv(script_text: str, scratch: Path):
    """Wrap `bash -c script_text` in bubblewrap.

    --unshare-all covers the network (no exfiltration, no fetching). The
    repo is bound read-only, so a script can read source and run `git log`
    but cannot modify the tree — including the tool that contains it. The
    single read-write bind is `scratch`, layered over the read-only repo.
    """
    argv = ["bwrap", "--unshare-all", "--die-with-parent",
            "--ro-bind", "/usr", "/usr",
            "--ro-bind", "/etc", "/etc",
            "--proc", "/proc", "--dev", "/dev", "--tmpfs", "/tmp"]
    # Merged-usr hosts symlink these into /usr; non-merged ones need a real
    # bind. Either way the script sees the standard layout it expects.
    for top in ("/bin", "/sbin", "/lib", "/lib64"):
        path = Path(top)
        if path.is_symlink():
            argv += ["--symlink", os.readlink(top), top]
        elif path.is_dir():
            argv += ["--ro-bind", top, top]
    root = str(_PROJECT_ROOT)
    # Order matters: the read-write scratch bind is applied after the
    # read-only repo bind so it wins for that subtree.
    argv += ["--ro-bind", root, root,
             "--bind", str(scratch), str(scratch),
             "--chdir", root, "--clearenv"]
    env = dict(_SANDBOX_ENV)
    if _SANDBOX_VENV_BIN.is_dir():
        env["PATH"] = f"{_SANDBOX_VENV_BIN}:{env['PATH']}"
    # cwd is the repo root so reading source needs no path juggling;
    # $SCRATCH is how a script reaches the one place it can write.
    env["SCRATCH"] = str(scratch)
    if _SANDBOX_HF_HOME.is_dir():
        env["HF_HOME"] = str(_SANDBOX_HF_HOME)
    for key, value in env.items():
        argv += ["--setenv", key, value]
    return argv + ["bash", "-c", script_text]


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None,
          extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=value or reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=value, extra=extra)


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract.

    No interactive prompt: the chat loop has no channel to ask the user
    (the legacy ASK_USER step belongs to the OODA executor, and
    chat_tool_stub.StubExecutor reports benchmark_mode so it is skipped).
    What makes that acceptable is the sandbox below, which is mandatory
    on every path — not a trust grant.
    """
    from utils.chat_tool_stub import build_tool_kwargs, CapturingResourceManager, translate_result
    script = args.get("script", "")
    if not isinstance(script, str) or not script.strip():
        return {"status": "error", "text": "exec-script requires non-empty `script`"}

    mgr = CapturingResourceManager()
    result = tool(script, **build_tool_kwargs(
        character_name=character_name, backend=backend, manager=mgr,
        target=script,
    ))
    return translate_result(result, manager=mgr,
                            empty_text="script produced no output")


def tool(input_value=None, runtime=None, **kwargs):
    """
    Execute an arbitrary bash script, sandboxed, with the repo root as cwd.

    input_value is the resolved target: either a literal script string
    or the content of a bound Note.  Collections are rejected upstream
    by the executor, but we guard against it here too.
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

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

    # ── Ask the user, where there is a user to ask ───────────────────
    # Only the OODA executor has an ask channel; the chat ReAct loop runs
    # turns off a serial inbox with nobody necessarily present, and
    # StubExecutor reports benchmark_mode so this is skipped there. The
    # ask is a courtesy on the path that can afford it — the sandbox
    # below, not this prompt, is what makes the tool safe to offer.
    benchmark_mode = bool(
        getattr(getattr(executor, "executive_node", None), "benchmark_mode", False)
    )
    if benchmark_mode:
        logger.info(f"exec-script: running sandboxed script "
                    f"(length={len(script_text)}, no ask channel)")
    else:
        ask_result = executor._execute_ask({
            "type": "ask",
            "target": "User",
            "value": f"exec-script wants to run the following script (sandboxed, read-only, no network):\n\n```\n{script_text}\n```\n\nProceed? (yes/no)",
        })

        if ask_result.get("status") != "success":
            return _fail(executor, "Permission request failed or timed out",
                         value=ask_result.get("reason", "ask failed"))

        response = (ask_result.get("value") or "").strip().lower()
        if response not in ("yes", "y"):
            return _fail(executor, "User denied permission to execute script",
                         value="denied")

    # ── Execute the script, sandboxed ────────────────────────────────
    # Fail closed: an unsandboxed fallback would silently turn the
    # contained tool back into arbitrary shell on the host.
    if shutil.which("bwrap") is None:
        return _fail(executor, "bubblewrap (bwrap) is not installed — "
                               "exec-script will not run a script unsandboxed")

    try:
        result = subprocess.run(
            _sandbox_argv(script_text, _scratch_dir(kwargs.get("agent_name"))),
            cwd=str(_PROJECT_ROOT),
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
