"""generate-image — local text-to-image via FLUX.2 klein-4B on the RTX PRO 6000.

ReAct entry-point. Runs worker.py as a subprocess with the CUDA pin set in the
environment (so the device choice takes effect before torch imports), generates
a PNG under the canvas server's local root, and returns a ready-to-display URL.
The image bytes never travel through the ReAct text log.

Two execution modes:
  - keep-warm (DEFAULT): hold a single --serve worker for the session so the
    import + model load is paid once. Lazy — spawns on the first generation,
    then holds ~18-19 GB on the PRO 6000 until the session exits.
  - one-shot (set GENERATE_IMAGE_KEEP_WARM=0/false/no/off): spawn a worker per
    call and free its ~13 GB on exit.
"""
import atexit
import json
import logging
import os
import select
import subprocess
import threading
import urllib.parse
import uuid
from pathlib import Path

_log = logging.getLogger(__name__)

_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _DIR.parents[2]
# zenoh_venv carries diffusers+accelerate; the host process may be running a
# different interpreter, so target this one explicitly rather than sys.executable.
_VENV_PY = _REPO_ROOT / "zenoh_venv" / "bin" / "python"
_WORKER = _DIR / "worker.py"

# Write under the canvas server's allowlisted local root, following its
# documented convention (~/.cache/cognitive/<tool-name>/). The server's
# /local route then serves these files to the browser — bytes reach the
# canvas out-of-band, never through the ReAct text log.
_OUT_DIR = Path("~/.cache/cognitive/generate-image").expanduser()
# Browser reaches the server on 127.0.0.1 (CSP allowlist + default bind);
# port follows the server's CANVAS_HTTP_PORT env, same default.
_CANVAS_PORT = os.environ.get("CANVAS_HTTP_PORT", "8789")

# CUDA pin: PCI_BUS_ID makes the index match nvidia-smi (0 = 5060 Ti bus 82,
# 1 = PRO 6000 bus 84). Target the PRO 6000 (index 1) — it has ~45 GB free
# alongside the FP8 Gemma vLLM pool, and klein-4B (~13 GB) won't fit the 16 GB
# 5060 Ti. The worker also asserts on the device name as a hard backstop.
_CUDA_ENV = {"CUDA_DEVICE_ORDER": "PCI_BUS_ID", "CUDA_VISIBLE_DEVICES": "1"}


def _keep_warm_enabled():
    # Keep-warm is the DEFAULT (lazy: the persistent worker spawns on the first
    # generation, then stays resident for the session). Set
    # GENERATE_IMAGE_KEEP_WARM to a falsy value (0/false/no/off) to disable and
    # fall back to one-shot-per-call.
    return os.environ.get("GENERATE_IMAGE_KEEP_WARM", "1").strip().lower() not in (
        "0", "false", "no", "off")


def _worker_args(args):
    """Optional generation args, normalized to worker flag/field names."""
    out = {}
    for key in ("steps", "size", "seed"):
        if args.get(key) is not None:
            out[key] = args[key]
    return out


# ── keep-warm persistent worker ──────────────────────────────────────
# A single --serve subprocess, lazily spawned and reused. Guarded by a lock
# (one request in flight). Dies on parent exit (stdin EOF) or atexit.
_worker_lock = threading.Lock()
_worker_proc = None
_worker_stderr = None


def _stderr_tail(n=300):
    try:
        if _worker_stderr is not None:
            _worker_stderr.flush()
        return (_OUT_DIR / "worker.stderr.log").read_text()[-n:].strip()
    except OSError as e:
        _log.warning(f"generate-image: could not read worker stderr log: {e}")
        return ""


def _shutdown_worker():
    global _worker_proc, _worker_stderr
    if _worker_proc is not None:
        try:
            _worker_proc.terminate()
        except (ProcessLookupError, OSError) as e:
            _log.debug(f"generate-image: worker terminate: {e}")
        _worker_proc = None
    if _worker_stderr is not None:
        try:
            _worker_stderr.close()
        except OSError as e:
            _log.debug(f"generate-image: worker stderr close: {e}")
        _worker_stderr = None


atexit.register(_shutdown_worker)


def _read_line(proc, timeout):
    """Read one stdout line with a timeout; None if nothing arrives in time."""
    ready, _, _ = select.select([proc.stdout], [], [], timeout)
    if not ready:
        return None
    return proc.stdout.readline()


def _get_worker():
    """Return a live --serve worker, spawning (and loading the model) if needed.
    Raises RuntimeError if the worker can't start."""
    global _worker_proc, _worker_stderr
    if _worker_proc is not None and _worker_proc.poll() is None:
        return _worker_proc
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    _worker_stderr = open(_OUT_DIR / "worker.stderr.log", "w")
    _worker_proc = subprocess.Popen(
        [str(_VENV_PY), str(_WORKER), "--serve"],
        env={**os.environ, **_CUDA_ENV},
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=_worker_stderr,
        text=True, bufsize=1,
    )
    # Wait for {"ready": true} — covers the one-time model load (and the first
    # run's ~13 GB weight download) and surfaces a load failure (e.g. wrong GPU)
    # instead of hanging the chat turn.
    line = _read_line(_worker_proc, timeout=900)
    if line is None or '"ready"' not in line:
        _shutdown_worker()
        raise RuntimeError(
            f"keep-warm worker failed to start: {(line or 'no output').strip()} "
            f"{_stderr_tail()}")
    _log.info("generate-image: keep-warm worker ready")
    return _worker_proc


def _persistent_generate(args, out_path):
    req = {"prompt": args["prompt"], "out": str(out_path), **_worker_args(args)}
    with _worker_lock:
        proc = _get_worker()
        try:
            proc.stdin.write(json.dumps(req) + "\n")
            proc.stdin.flush()
        except (BrokenPipeError, OSError) as e:
            _log.warning(f"generate-image: keep-warm worker pipe broke: {e}")
            _shutdown_worker()
            raise RuntimeError("keep-warm worker died; retry to respawn it")
        line = _read_line(proc, timeout=180)
    if line is None:
        _log.warning("generate-image: keep-warm worker timed out (180s)")
        _shutdown_worker()
        raise RuntimeError("image generation timed out (180s)")
    return json.loads(line)


# ── default one-shot worker ──────────────────────────────────────────
def _oneshot_generate(args, out_path):
    cmd = [str(_VENV_PY), str(_WORKER), "--prompt", args["prompt"], "--out", str(out_path)]
    for flag, key in (("--steps", "steps"), ("--size", "size"), ("--seed", "seed")):
        if args.get(key) is not None:
            cmd += [flag, str(args[key])]
    try:
        # Generous timeout: the first-ever run downloads ~13 GB of weights.
        proc = subprocess.run(cmd, env={**os.environ, **_CUDA_ENV},
                              capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        _log.warning("generate-image: one-shot worker timed out (900s)")
        raise RuntimeError("image generation timed out (900s)")
    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout or "").strip().splitlines()
        _log.warning(f"generate-image: worker failed (rc={proc.returncode}): "
                     f"{(proc.stderr or '').strip()[-500:]}")
        raise RuntimeError(f"image generation failed: {tail[-1] if tail else 'unknown error'}")
    lines = proc.stdout.strip().splitlines()
    if not lines:
        raise RuntimeError("image generation produced no output")
    return json.loads(lines[-1])


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """See Skill.md for the args contract. Returns {status, text}."""
    prompt = args.get("prompt", "")
    if not isinstance(prompt, str) or not prompt.strip():
        return {"status": "error", "text": "generate-image requires a non-empty `prompt`"}
    if not _VENV_PY.exists():
        return {"status": "error",
                "text": f"diffusers venv python not found at {_VENV_PY}"}

    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = _OUT_DIR / f"{uuid.uuid4().hex}.png"

    try:
        if _keep_warm_enabled():
            result = _persistent_generate(args, out_path)
        else:
            result = _oneshot_generate(args, out_path)
    except RuntimeError as e:
        return {"status": "error", "text": str(e)}
    except json.JSONDecodeError as e:
        _log.warning(f"generate-image: unparseable worker result: {e}")
        return {"status": "error", "text": "image generation produced no parseable result"}

    if "error" in result:
        return {"status": "error", "text": f"image generation failed: {result['error']}"}

    url = (f"http://127.0.0.1:{_CANVAS_PORT}/local?path="
           + urllib.parse.quote(result["path"]))
    return {"status": "ok",
            "text": (f"image generated ({result.get('gen_s')}s). Show it with the "
                     f"display tool, format=html: <img src=\"{url}\" style=\"max-width:100%\">")}
