"""generate-image — local text-to-image via the Bonsai-Image backend server.

ReAct entry-point. POSTs the prompt directly to the long-lived Bonsai FastAPI
backend (scripts.local_backend, which strips the bearer gate on Linux), saves
the returned PNG under the canvas server's allowlisted local root, and returns a
ready-to-display URL. The image bytes never travel through the ReAct text log.

The backend is expected at http://localhost:4001 by default and exposes
/generate and /healthz (the /api/generate path belongs to the Next.js studio
frontend, which we bypass). If the backend isn't running, the tool launches it
via the Bonsai-Image-Demo serve.sh and waits for /healthz to come up. Override
the base URL with GENERATE_IMAGE_URL and the launcher with GENERATE_IMAGE_SERVE_SH.
"""
import logging
import os
import subprocess
import time
import urllib.parse
import uuid
from pathlib import Path

import requests

_log = logging.getLogger(__name__)

_OUT_DIR = Path("~/.cache/cognitive/generate-image").expanduser()
_CANVAS_PORT = os.environ.get("CANVAS_HTTP_PORT", "8789")

# Bonsai FastAPI backend base. /generate and /healthz live here.
_BASE = os.environ.get("GENERATE_IMAGE_URL", "http://localhost:4001").rstrip("/")
_GENERATE_URL = f"{_BASE}/generate"
_HEALTHZ_URL = f"{_BASE}/healthz"
_BACKEND_PORT = urllib.parse.urlsplit(_BASE).port or 4001

_BONSAI_BACKEND = os.environ.get("GENERATE_IMAGE_BACKEND",
                                 "bonsai-ternary-gemlite")

# Launcher. serve.sh starts the backend (and a studio frontend we don't use) and
# prewarms the model before /healthz answers, so a cold boot can take minutes.
_SERVE_SH = os.environ.get(
    "GENERATE_IMAGE_SERVE_SH",
    "/home/bruce/Downloads/Bonsai-Image-Demo/scripts/serve.sh")
_FRONTEND_PORT = os.environ.get("GENERATE_IMAGE_FRONTEND_PORT", "3100")
# Which GPU the image backend runs on, in PCI_BUS_ID order (so it matches
# nvidia-smi). 0 is the 5060 Ti here; 1 holds the live vLLM. See the pin
# in _spawn_serve for why the order matters.
_IMAGE_GPU = os.environ.get("GENERATE_IMAGE_GPU", "0")
_BOOT_TIMEOUT_S = int(os.environ.get("GENERATE_IMAGE_BOOT_TIMEOUT", "240"))

# Bonsai is distilled for 4 steps; the studio's fast preset is 512×512.
_DEFAULT_STEPS = 4
_DEFAULT_SIZE = 512
# Wall-clock cap. Bonsai cold-first-shape JIT can take ~30-60s; warm calls are
# ~1s. 120s matches the python_test reference and leaves slack for cold boot.
_TIMEOUT_S = 120


def _healthy(timeout=2):
    """True if the backend answers /healthz with 200."""
    try:
        return requests.get(_HEALTHZ_URL, timeout=timeout).status_code == 200
    except requests.RequestException as e:
        _log.debug(f"generate-image: healthz probe failed: {e}")
        return False


def _spawn_serve():
    """Spawn serve.sh detached with the binary-path workaround. Returns the
    Popen, or None if serve.sh is missing or the launch failed.

    Single source of truth for *how* the Bonsai server is started — used both
    by the on-demand path (_ensure_server) and the launcher's prestart hook
    (start_image_server)."""
    serve = Path(_SERVE_SH)
    if not serve.is_file():
        _log.warning(f"generate-image: serve.sh not found at {serve}")
        return None

    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    log_path = _OUT_DIR / "serve.log"
    env = dict(os.environ,
               BACKEND_PORT=str(_BACKEND_PORT),
               FRONTEND_PORT=_FRONTEND_PORT)
    # Pin the image backend to the spare card. Without this it inherits the
    # agent's environment, which sets no CUDA vars, so torch falls back to
    # its default FASTEST_FIRST device order — and that REVERSES nvidia-smi
    # on this box: torch's cuda:0 is the RTX PRO 6000 running the live vLLM
    # (97GB, ~0 free), not the 5060 Ti. The image backend would allocate on
    # the inference card and OOM, or disturb the running model.
    # CUDA_DEVICE_ORDER=PCI_BUS_ID makes the index match nvidia-smi, where
    # 0 is the 5060 Ti. Both must be set together; setting VISIBLE_DEVICES
    # alone still selects by the reversed order. Respect an explicit
    # override so a different host can point elsewhere.
    env.setdefault("CUDA_DEVICE_ORDER", "PCI_BUS_ID")
    env.setdefault("CUDA_VISIBLE_DEVICES", _IMAGE_GPU)
    # GpuPipeline.__init__ eagerly requires a path for BOTH the ternary and
    # binary models, but serve.sh only sets the binary path when the binary model
    # is installed. We only ever request the ternary model, so the binary model is
    # never loaded — point it at any installed gemlite transformer so
    # construction succeeds. (No-op if already set.)
    if "MFLUX_STUDIO_GPU_BINARY_TRANSFORMER_PATH" not in env:
        models = serve.parent.parent / "models"
        xfmr = next(models.glob("bonsai-image-4B-*-gemlite/transformer-gemlite-*"),
                    None)
        if xfmr is not None:
            env["MFLUX_STUDIO_GPU_BINARY_TRANSFORMER_PATH"] = str(xfmr)
        else:
            _log.warning("generate-image: no installed gemlite transformer "
                         f"found under {models}; backend may fail to start")
    _log.info(f"generate-image: launching Bonsai server "
              f"(BACKEND_PORT={_BACKEND_PORT}); logging to {log_path}")
    try:
        with open(log_path, "ab") as logf:
            return subprocess.Popen(
                [str(serve)],
                cwd=str(serve.parent.parent),
                env=env,
                stdout=logf, stderr=logf, stdin=subprocess.DEVNULL,
                start_new_session=True)
    except OSError as e:
        _log.warning(f"generate-image: failed to launch serve.sh: {e}")
        return None


def start_image_server():
    """Fire-and-forget prestart of the Bonsai backend (e.g. from the launcher's
    --image-server flag) so it's warm by the first generate-image call. Returns
    the Popen if a launch was started, or None (already healthy / couldn't
    start). Does NOT wait for readiness."""
    if _healthy():
        _log.info("generate-image: backend already healthy; no prestart needed")
        return None
    return _spawn_serve()


def _ensure_server():
    """Make the backend healthy, launching serve.sh if needed. Returns bool."""
    if _healthy():
        return True

    if not Path(_SERVE_SH).is_file():
        _log.warning(f"generate-image: serve.sh not found at {_SERVE_SH}")
        return False

    # Atomic launch lock so two concurrent invocations don't both cold-start
    # serve.sh (a simultaneous bind race could trip the second launcher's
    # cleanup trap and kill the first server).
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    lock = _OUT_DIR / ".serve.lock"
    launched_here = False
    try:
        os.close(os.open(str(lock), os.O_CREAT | os.O_EXCL | os.O_WRONLY))
        launched_here = True
    except FileExistsError:
        _log.info("generate-image: a serve.sh launch is already in progress; "
                  "waiting for /healthz")

    if launched_here and _spawn_serve() is None:
        lock.unlink(missing_ok=True)
        return False

    # Poll until the model finishes prewarm (cold boot can take minutes).
    deadline = time.monotonic() + _BOOT_TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            if _healthy():
                return True
            time.sleep(2)
        _log.warning(f"generate-image: backend not healthy after "
                     f"{_BOOT_TIMEOUT_S}s (still warming?)")
        return False
    finally:
        if launched_here:
            lock.unlink(missing_ok=True)


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """See Skill.md for the args contract. Returns {status, text}."""
    prompt = args.get("prompt", "")
    if not isinstance(prompt, str) or not prompt.strip():
        return {"status": "error", "text": "generate-image requires a non-empty `prompt`"}

    if not _ensure_server():
        return {"status": "error",
                "text": (f"Bonsai image backend not reachable at {_BASE} and "
                         f"could not be started. Start it manually from the "
                         f"Bonsai-Image-Demo repo: "
                         f"BACKEND_PORT={_BACKEND_PORT} FRONTEND_PORT={_FRONTEND_PORT} "
                         f"./scripts/serve.sh")}

    size = int(args.get("size") or _DEFAULT_SIZE)
    payload = {
        "prompt": prompt,
        "steps": int(args.get("steps") or _DEFAULT_STEPS),
        "height": size,
        "width": size,
        "backend": _BONSAI_BACKEND,
    }
    if args.get("seed") is not None:
        payload["seed"] = int(args["seed"])

    try:
        resp = requests.post(_GENERATE_URL, json=payload, timeout=_TIMEOUT_S)
    except requests.ConnectionError as e:
        _log.warning(f"generate-image: cannot reach Bonsai at {_GENERATE_URL}: {e}")
        return {"status": "error",
                "text": (f"Bonsai image backend not reachable at {_GENERATE_URL}. "
                         f"Start it from the Bonsai-Image-Demo repo: "
                         f"BACKEND_PORT={_BACKEND_PORT} FRONTEND_PORT={_FRONTEND_PORT} "
                         f"./scripts/serve.sh")}
    except requests.Timeout:
        _log.warning(f"generate-image: Bonsai timed out after {_TIMEOUT_S}s")
        return {"status": "error", "text": f"image generation timed out ({_TIMEOUT_S}s)"}

    if resp.status_code != 200:
        body = resp.text[:300] if resp.text else ""
        _log.warning(f"generate-image: Bonsai returned {resp.status_code}: {body}")
        return {"status": "error",
                "text": f"image generation failed (HTTP {resp.status_code}): {body}"}

    ctype = resp.headers.get("content-type", "")
    if ctype != "image/png" or resp.content[:8] != b"\x89PNG\r\n\x1a\n":
        _log.warning(f"generate-image: unexpected response content-type={ctype!r}")
        return {"status": "error",
                "text": f"Bonsai returned a non-PNG response (content-type={ctype!r})"}

    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = _OUT_DIR / f"{uuid.uuid4().hex}.png"
    out_path.write_bytes(resp.content)

    gen_s = resp.headers.get("x-wall-seconds")
    gen_info = f" ({gen_s}s)" if gen_s else ""

    url = (f"http://127.0.0.1:{_CANVAS_PORT}/local?path="
           + urllib.parse.quote(str(out_path)))
    return {"status": "ok",
            "text": (f"image generated{gen_info}. Show it with the display tool, "
                     f"format=html: <img src=\"{url}\" style=\"max-width:100%\">")}
