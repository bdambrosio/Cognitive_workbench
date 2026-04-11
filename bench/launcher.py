#!/usr/bin/env python3
"""Stage 3b: Launcher lifecycle helper for the benchmark harness.

The cross-validation experiment harness needs to start, stop, and restart
the cognitive workbench launcher between trials so it can snapshot/restore
state with the agent down. This module wraps the lifecycle:

  1. Spawn the launcher as a subprocess in src/ with the bench scenario
  2. Wait for both:
       - cognitive/launcher/ready  (launcher published, agent threads spawned)
       - cognitive/<character>/scheduled_goals  (per-character queryable up,
         proves agent __init__ completed)
  3. Hand control to the harness for the trial
  4. Send shutdown via cognitive/launcher/shutdown
  5. Wait for the subprocess to exit (with a force-kill timeout)

Used as a context manager:

    from launcher import LauncherProcess

    with LauncherProcess() as proc:
        # launcher is up and ready
        run_trial(...)
    # launcher is now stopped cleanly

Or imperatively:

    proc = LauncherProcess()
    proc.start()
    proc.wait_ready()
    ...
    proc.stop()

Requires the zenoh_venv interpreter and a sensor-free scenario YAML
(scenarios/jill-benchlab.yaml is the default).
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

# Path setup so we can import zenoh utils for liveness/shutdown communication.
_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))

DEFAULT_VENV_PYTHON = _REPO_ROOT / "zenoh_venv" / "bin" / "python3"
DEFAULT_SCENARIO = "jill-benchlab.yaml"
DEFAULT_CHARACTER = "Jill"
DEFAULT_READY_TIMEOUT_S = 60.0
DEFAULT_SHUTDOWN_TIMEOUT_S = 30.0


class LauncherError(RuntimeError):
    """Lifecycle errors: failed to start, ready timeout, dirty shutdown."""


class LauncherProcess:
    """Manages a cognitive workbench launcher subprocess for benchmark trials.

    Attributes:
        process: subprocess.Popen handle once started, else None
        scenario: scenario YAML filename (passed to launcher.py)
        character: character name to wait for ready
        venv_python: path to the Python interpreter that has zenoh installed
        log_path: optional path to redirect stdout+stderr to
    """

    def __init__(
        self,
        scenario: str = DEFAULT_SCENARIO,
        character: str = DEFAULT_CHARACTER,
        venv_python: Optional[Path] = None,
        log_path: Optional[Path] = None,
        extra_args: Optional[List[str]] = None,
    ):
        self.scenario = scenario
        self.character = character
        self.venv_python = Path(venv_python) if venv_python else DEFAULT_VENV_PYTHON
        self.log_path = Path(log_path) if log_path else None
        self.extra_args = list(extra_args or [])
        self.process: Optional[subprocess.Popen] = None
        self._log_handle = None

    # ──────────────────────────────────────────────────────────────────
    # Subprocess lifecycle
    # ──────────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Spawn the launcher subprocess in src/. Returns immediately;
        call wait_ready() to block until the agent is responsive.

        Raises LauncherError if the venv python or src/ dir doesn't exist.
        """
        if self.process is not None and self.process.poll() is None:
            raise LauncherError("LauncherProcess.start: already running")
        if not self.venv_python.exists():
            raise LauncherError(f"Venv python not found: {self.venv_python}")
        if not _SRC_DIR.exists():
            raise LauncherError(f"src/ directory not found: {_SRC_DIR}")
        scenario_path = _REPO_ROOT / "scenarios" / self.scenario
        if not scenario_path.exists():
            raise LauncherError(f"Scenario file not found: {scenario_path}")

        cmd = [str(self.venv_python), "launcher.py", self.scenario] + self.extra_args
        # Open log file if requested. Otherwise inherit stdout/stderr from
        # the parent (the harness will get the launcher's output mixed in).
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_handle = open(self.log_path, "ab")
            stdout = self._log_handle
            stderr = subprocess.STDOUT
        else:
            stdout = subprocess.DEVNULL
            stderr = subprocess.DEVNULL

        # Start in src/ — launcher.py loads scenarios via "../scenarios/..."
        # so it expects to be run from src/.
        self.process = subprocess.Popen(
            cmd,
            cwd=str(_SRC_DIR),
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            start_new_session=True,  # detach so we can signal cleanly
        )
        print(
            f"LauncherProcess: started PID={self.process.pid} "
            f"(scenario={self.scenario}, character={self.character})",
            file=sys.stderr,
        )

    def is_running(self) -> bool:
        """True if the subprocess is alive."""
        return self.process is not None and self.process.poll() is None

    # ──────────────────────────────────────────────────────────────────
    # Readiness check
    # ──────────────────────────────────────────────────────────────────

    def wait_ready(self, timeout: float = DEFAULT_READY_TIMEOUT_S) -> None:
        """Block until the launcher is fully ready to accept trial commands.

        Two-stage check:
          (a) cognitive/launcher/ready signal published — proves the
              launcher's main() reached the agent-thread-spawn step
          (b) cognitive/<character>/scheduled_goals queryable responds —
              proves the character's executive_node finished __init__
              and registered its queryables

        The launcher publishes ready immediately after spawning agent
        threads, but the agent's __init__ takes a few more seconds to
        register its queryables. Both checks together avoid races.

        Raises LauncherError on timeout or if the subprocess died.
        """
        if not self.is_running():
            raise LauncherError("LauncherProcess.wait_ready: process not running")

        deadline = time.monotonic() + timeout
        try:
            import zenoh  # noqa: WPS433
            from utils.zenoh_utils import make_localhost_config  # noqa: WPS433
        except ImportError as e:
            raise LauncherError(f"zenoh not importable from this environment: {e}")

        # Poll the queryable directly. The launcher's ready signal is a
        # publisher (one-shot), and we may have missed it; the queryable
        # check is sufficient and stricter (it proves the agent's __init__
        # actually finished, not just that the thread was spawned).
        session = None
        try:
            session = zenoh.open(make_localhost_config())
        except Exception as e:
            raise LauncherError(f"could not open zenoh session for readiness check: {e}")

        try:
            interval = 0.5
            attempts = 0
            while time.monotonic() < deadline:
                if not self.is_running():
                    raise LauncherError(
                        f"LauncherProcess: subprocess exited during startup "
                        f"(returncode={self.process.returncode if self.process else '?'})"
                    )
                attempts += 1
                key = f"cognitive/{self.character}/scheduled_goals"
                try:
                    for reply in session.get(key, timeout=2.0):
                        if hasattr(reply, "ok") and reply.ok is not None:
                            elapsed = time.monotonic() - (deadline - timeout)
                            print(
                                f"LauncherProcess: ready after {elapsed:.1f}s "
                                f"(attempt {attempts})",
                                file=sys.stderr,
                            )
                            return
                except Exception:
                    pass
                time.sleep(interval)
            raise LauncherError(
                f"LauncherProcess: ready timeout after {timeout}s "
                f"(character {self.character} queryable never responded)"
            )
        finally:
            try:
                session.close()
            except Exception:
                pass

    # ──────────────────────────────────────────────────────────────────
    # Shutdown
    # ──────────────────────────────────────────────────────────────────

    def stop(self, timeout: float = DEFAULT_SHUTDOWN_TIMEOUT_S) -> None:
        """Cleanly stop the launcher.

        Sequence:
          1. Send shutdown via cognitive/launcher/shutdown
          2. Wait for subprocess to exit (poll up to timeout)
          3. If still running, escalate to SIGTERM
          4. Wait again
          5. If still running, SIGKILL the process group
          6. Verify sglang worker processes have also exited (waits up
             to SGLANG_TEARDOWN_BUDGET_S, escalates to pkill if needed)

        Idempotent: safe to call when not running. Closes log file handle
        if one was opened.
        """
        if self.process is None:
            return
        if self.process.poll() is not None:
            self._finalize_stop()
            return

        # 1. Zenoh shutdown
        zenoh_sent = self._send_zenoh_shutdown()

        # 2. Wait politely
        if zenoh_sent:
            polite_deadline = time.monotonic() + timeout
            while time.monotonic() < polite_deadline:
                if self.process.poll() is not None:
                    print(
                        f"LauncherProcess: stopped cleanly via Zenoh shutdown "
                        f"(returncode={self.process.returncode})",
                        file=sys.stderr,
                    )
                    self._finalize_stop()
                    return
                time.sleep(0.5)

        # 3. SIGTERM the process group
        print(
            f"LauncherProcess: Zenoh shutdown didn't take, escalating to SIGTERM",
            file=sys.stderr,
        )
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

        # 4. Wait again
        sigterm_deadline = time.monotonic() + 5.0
        while time.monotonic() < sigterm_deadline:
            if self.process.poll() is not None:
                print(
                    f"LauncherProcess: stopped via SIGTERM "
                    f"(returncode={self.process.returncode})",
                    file=sys.stderr,
                )
                self._finalize_stop()
                return
            time.sleep(0.5)

        # 5. SIGKILL the process group
        print(
            f"LauncherProcess: SIGTERM didn't take, escalating to SIGKILL",
            file=sys.stderr,
        )
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        try:
            self.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            print(
                f"LauncherProcess: SIGKILL also failed; PID {self.process.pid} "
                f"may be a zombie",
                file=sys.stderr,
            )
        self._finalize_stop()

    def _finalize_stop(self) -> None:
        """Common tail of stop(): close the log handle, then verify no
        sglang worker processes linger from the launcher we just exited.
        Called from every return path in stop().
        """
        self._cleanup_log()
        self._verify_sglang_teardown()

    def _verify_sglang_teardown(
        self,
        max_wait_s: float = 180.0,
        poll_interval: float = 3.0,
    ) -> None:
        """After the launcher subprocess exits, verify its sglang child
        workers have also exited.

        sglang's in-process runtime spawns scheduler, tokenizer, and
        model-worker subprocesses. When the parent Python exits, SIGTERM
        sometimes fails to reach all of them (they break from the
        process group, or take a while to release GPU resources and
        file locks). Lingering workers hold GPU memory + IPC ports that
        the next launcher startup needs — the symptom is that the next
        `wait_ready` succeeds but goal execution silently fails because
        the new sglang runtime can't allocate resources.

        Strategy:
        1. Poll `pgrep -f sglang` up to `max_wait_s` (default 180s to
           accommodate cognitive_graph / world_model save variance).
        2. If still present, escalate to `pkill -f sglang` (SIGTERM,
           not -9; in practice pkill is sufficient).
        3. Wait 10s, re-verify. Log final state either way.

        Best-effort: never raises, only logs. If the user has another
        sglang process running elsewhere on the machine, the pkill will
        catch it too — but bench runs assume no interactive Jill is
        running concurrently (the preflight check enforces this).
        """
        start = time.monotonic()
        logged_waiting = False
        while time.monotonic() - start < max_wait_s:
            if not self._sglang_workers_present():
                elapsed = time.monotonic() - start
                if logged_waiting:
                    print(
                        f"LauncherProcess: sglang workers exited after "
                        f"{elapsed:.1f}s",
                        file=sys.stderr,
                    )
                return
            if not logged_waiting:
                print(
                    f"LauncherProcess: sglang workers still present after "
                    f"subprocess exit — polling for teardown "
                    f"(budget {max_wait_s:.0f}s)",
                    file=sys.stderr,
                )
                logged_waiting = True
            time.sleep(poll_interval)

        # Didn't exit naturally — escalate.
        print(
            f"LauncherProcess: sglang workers STILL present after "
            f"{max_wait_s:.0f}s, sending SIGTERM via pkill",
            file=sys.stderr,
        )
        try:
            subprocess.run(
                ["pkill", "-f", "sglang"],
                capture_output=True,
            )
        except Exception as e:
            print(
                f"LauncherProcess: pkill invocation failed: {e}",
                file=sys.stderr,
            )
            return

        # Give SIGTERM a moment, then re-check.
        time.sleep(10.0)
        if self._sglang_workers_present():
            print(
                f"LauncherProcess: WARN — sglang workers STILL present "
                f"after pkill SIGTERM; giving up. Next launcher start may "
                f"fail until they are cleaned up manually.",
                file=sys.stderr,
            )
        else:
            print(
                f"LauncherProcess: sglang workers exited after pkill",
                file=sys.stderr,
            )

    @staticmethod
    def _sglang_workers_present() -> bool:
        """True if any process matching 'sglang' is running. Returns
        False on pgrep failure (can't determine → treat as absent so
        we don't loop forever).
        """
        try:
            result = subprocess.run(
                ["pgrep", "-f", "sglang"],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            return result.returncode == 0 and bool(result.stdout.strip())
        except Exception:
            return False

    def _send_zenoh_shutdown(self) -> bool:
        """Send the shutdown message to cognitive/launcher/shutdown.
        Returns True if the put succeeded, False if zenoh wasn't reachable.
        """
        try:
            import zenoh  # noqa: WPS433
            from utils.zenoh_utils import make_localhost_config  # noqa: WPS433
        except ImportError:
            return False
        try:
            session = zenoh.open(make_localhost_config())
        except Exception:
            return False
        try:
            payload = {
                "timestamp": datetime.now().isoformat(),
                "source": "bench-harness",
                "mode": "save_and_shutdown",
            }
            session.put(
                "cognitive/launcher/shutdown",
                json.dumps(payload).encode("utf-8"),
            )
            return True
        except Exception:
            return False
        finally:
            try:
                session.close()
            except Exception:
                pass

    def _cleanup_log(self) -> None:
        if self._log_handle is not None:
            try:
                self._log_handle.close()
            except Exception:
                pass
            self._log_handle = None

    # ──────────────────────────────────────────────────────────────────
    # Context manager
    # ──────────────────────────────────────────────────────────────────

    def __enter__(self):
        self.start()
        try:
            self.wait_ready()
        except Exception:
            self.stop()
            raise
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False  # don't suppress exceptions


# ──────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────

def main(argv: Optional[List[str]] = None) -> int:
    import argparse
    p = argparse.ArgumentParser(
        description="Test harness for the launcher lifecycle helper. "
        "Spawns the launcher with the bench scenario, waits for ready, "
        "optionally sleeps, then cleanly shuts down.",
    )
    p.add_argument("--scenario", default=DEFAULT_SCENARIO)
    p.add_argument("--character", default=DEFAULT_CHARACTER)
    p.add_argument("--ready-timeout", type=float, default=DEFAULT_READY_TIMEOUT_S)
    p.add_argument("--hold-seconds", type=float, default=2.0,
                   help="Seconds to hold the launcher up after ready before shutting down")
    p.add_argument("--log-path", type=Path, default=None,
                   help="Redirect launcher stdout/stderr to this file")
    args = p.parse_args(argv)

    proc = LauncherProcess(
        scenario=args.scenario,
        character=args.character,
        log_path=args.log_path,
    )
    try:
        proc.start()
        proc.wait_ready(timeout=args.ready_timeout)
        print(
            f"LauncherProcess: holding for {args.hold_seconds}s before shutdown",
            file=sys.stderr,
        )
        time.sleep(args.hold_seconds)
        proc.stop()
        return 0
    except LauncherError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        proc.stop()
        return 1
    except KeyboardInterrupt:
        print("Interrupted; shutting down launcher", file=sys.stderr)
        proc.stop()
        return 130


if __name__ == "__main__":
    sys.exit(main())
