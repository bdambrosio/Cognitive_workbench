#!/usr/bin/env python3
"""
Cognitive Workbench Launcher

Loads a scenario YAML, creates a shared SGLang runtime (if configured),
and runs each character as a thread sharing that runtime.
Shared services (FastAPI UI, resource browser) remain subprocesses.
"""

import subprocess
import threading
import time
import signal
import sys
import json
import argparse
import logging
import yaml
import os
from pathlib import Path
from typing import Dict, List, Any, Optional


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

_debug_env = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')

console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# Anchor log path to the repo root (parent of src/), regardless of cwd.
# Without this, `python launcher.py …` and `python bench/.../runner.py …`
# write to different log files (`<repo>/src/logs/…` vs `<repo>/logs/…`)
# and chasing missing entries means checking both. This pins the file.
_LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
_LOG_DIR.mkdir(parents=True, exist_ok=True)
file_handler = logging.FileHandler(_LOG_DIR / 'character_launcher.log', mode='a')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('launcher')


# ---------------------------------------------------------------------------
# YAML parsing
# ---------------------------------------------------------------------------

def load_scenario(config_file: str) -> dict:
    """Load and return the full scenario dict from a YAML file."""
    config_path = os.path.join('../scenarios', config_file)
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def parse_characters(config_data: dict, llm_config: dict, world_config: dict, setting, alt_llm_config: dict = None) -> List[Dict[str, Any]]:
    """Return a list of (name, config) tuples from the YAML characters section."""
    characters_config = config_data.get('characters', [])
    ontology = config_data.get('Ontology', False)
    activities = config_data.get('Activities', False)
    result = []
    alt_llm = alt_llm_config if alt_llm_config else {}

    if isinstance(characters_config, dict):
        for name, config in characters_config.items():
            new_config = config.copy()
            new_config['ontology'] = ontology
            new_config['activities'] = activities
            new_config['characters'] = characters_config.copy()
            # Merge per-character llm_config OVER top-level so per-character
            # overrides (server, model, vllm_url, is_reasoning_model, etc.)
            # actually reach the agent. Same for alt_llm_config.
            char_llm = config.get('llm_config') or {}
            new_config['llm_config'] = {**(llm_config or {}), **char_llm}
            char_alt_llm = config.get('alt_llm_config') or {}
            new_config['alt_llm_config'] = {**(alt_llm or {}), **char_alt_llm}
            new_config['world_config'] = world_config
            new_config['setting'] = setting
            result.append((name.capitalize(), new_config))
    elif isinstance(characters_config, list):
        for char_config in characters_config:
            if isinstance(char_config, dict):
                name = char_config.get('name', f'character_{len(result)}')
                new_config = {k: v for k, v in char_config.items() if k != 'name'}
                new_config['ontology'] = ontology
                new_config['activities'] = activities
                new_config['characters'] = characters_config
                char_llm = char_config.get('llm_config') or {}
                new_config['llm_config'] = {**(llm_config or {}), **char_llm}
                char_alt_llm = char_config.get('alt_llm_config') or {}
                new_config['alt_llm_config'] = {**(alt_llm or {}), **char_alt_llm}
                new_config['world_config'] = world_config
                new_config['setting'] = setting
                result.append((name.capitalize(), new_config))
    return result


# ---------------------------------------------------------------------------
# World data cleanup (interactive, runs before launch)
# ---------------------------------------------------------------------------

def maybe_clean_world_data(world_name: str, character_names: List[str]):
    """Prompt user to reuse or delete existing world data."""
    world_file = Path(f"data/world/{world_name}_world.json")
    if not world_file.exists():
        return

    print(f"\nFound existing world data for '{world_name}'")
    reuse = input("Reuse existing world? (y/n): ").strip().lower()
    if reuse not in ['n', 'no']:
        print(f"Reusing existing world '{world_name}'")
        return

    confirm = input("Are you sure you want to delete existing world data and create new? (y/n): ").strip().lower()
    if confirm != 'y':
        print(f"Reusing existing world '{world_name}'")
        return

    print("Creating new world...")
    try:
        world_file.unlink()
        print(f"Removed existing world data for '{world_name}'")
    except Exception as e:
        print(f"Failed to remove existing world data: {e}")

    data_dir = Path("data")
    if not data_dir.exists():
        return

    for subdir, suffix in [("memory", "_memory.json"), ("situation", "_situation.json")]:
        target_dir = data_dir / subdir
        if target_dir.exists():
            for f in target_dir.glob(f"*{suffix}"):
                char_name = f.stem.replace(suffix.replace('.json', ''), '')
                if char_name in character_names:
                    try:
                        f.unlink()
                        print(f"Removed {f.name}")
                    except Exception:
                        pass

    rag_dir = data_dir / "rag_stores"
    if rag_dir.exists():
        import shutil
        for char_name in character_names:
            char_rag = rag_dir / char_name
            if char_rag.exists():
                try:
                    shutil.rmtree(char_rag)
                    print(f"Removed RAG store: {char_name}")
                except Exception as e:
                    print(f"Failed to remove RAG store for {char_name}: {e}")


# ---------------------------------------------------------------------------
# SGLang runtime factory
# ---------------------------------------------------------------------------

def create_sglang_runtime(llm_config: dict):
    """Create and return (runtime, tokenizer) or (None, None) if SGLang unavailable."""
    sgl_model_path = llm_config.get('sgl_model_path')
    if not sgl_model_path:
        return None, None

    try:
        import sglang as sgl
        from transformers import AutoTokenizer
    except ImportError:
        logger.warning("SGLang not available - skipping runtime creation")
        return None, None

    try:
        # Temporarily ignore SIGINT so SGLang child processes (scheduler,
        # detokenizer) inherit SIG_IGN and don't crash on Ctrl+C.
        # The launcher's own signal handler (installed later) handles shutdown.
        prev_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)

        logger.info(f"Initializing SGLang Runtime with model: {sgl_model_path}")
        tokenizer = AutoTokenizer.from_pretrained(sgl_model_path)

        # Optional custom chat template (e.g. to suppress thinking mode)
        chat_template = llm_config.get('chat_template')
        # Optional sglang reasoning parser (e.g. "qwen3" for Qwen3+ models
        # that emit <think>...</think>). See sglang ReasoningParser.DetectorMap
        # for valid values: deepseek-r1, deepseek-v3, glm45, gpt-oss, interns1,
        # kimi, kimi_k2, minimax, minimax-append-think, nano_v3, qwen3,
        # qwen3-thinking, step3, step3p5.
        reasoning_parser = llm_config.get('reasoning_parser')

        extra_kwargs = {}
        if chat_template:
            extra_kwargs['chat_template'] = chat_template
        if reasoning_parser:
            extra_kwargs['reasoning_parser'] = reasoning_parser

        if 'NVFP4' in sgl_model_path: # patch for Qwen3.5-122B-A10B-NVFP4 models as of 3/1/2026
            logger.info(f"🚀 Initializing SGLang Runtime with NVFP4 patch!")
            if chat_template:
                logger.info(f"Using custom chat template: {chat_template}")
                import os
                os.environ["SGLANG_DISABLE_DEEP_GEMM"] = "1"

                import sglang as sgl

                runtime = sgl.Runtime(
                    model_path="txn545/Qwen3.5-122B-A10B-NVFP4",
                    tp_size=1,
                    quantization="modelopt_fp4",
                    attention_backend="triton",
                    moe_runner_backend="flashinfer_cutlass",
                    fp4_gemm_runner_backend="flashinfer_cutlass",
                    mem_fraction_static=0.92,
                    disable_cuda_graph=True,
                    disable_radix_cache=True,
                    kv_cache_dtype="bf16",
                    context_length=32768,
                    port=5000,
                    **extra_kwargs,
                )

                sgl.set_default_backend(runtime)
                #runtime = sgl.Runtime(model_path=sgl_model_path,device="cuda",context_length=65536,tp_size=1,mem_fraction_static=0.9,quantization="modelopt_fp4",attention_backend="triton",**({'chat_template': chat_template} if chat_template else {}))
        elif 'FP8' in sgl_model_path:
            logger.info("Initializing SGLang Runtime for FP8")
            runtime = sgl.Runtime(model_path=sgl_model_path, tokenizer_path=sgl_model_path, device="cuda", context_length=65536,
                      dtype="auto", tp_size=1, mem_fraction_static=0.85, attention_backend="triton", fp8_gemm_runner_backend="cutlass", **extra_kwargs)
        else:
            runtime = sgl.Runtime(model_path=sgl_model_path, tokenizer_path=sgl_model_path, device="cuda", context_length=65536, dtype="auto", tp_size=1, mem_fraction_static=0.9, attention_backend="triton", **extra_kwargs)

        sgl.set_default_backend(runtime)
        extras_log = ", ".join(f"{k}={v}" for k, v in extra_kwargs.items())
        logger.info(f"SGLang Runtime initialized (model={sgl_model_path})" + (f", {extras_log}" if extras_log else ""))
        # Restore SIGINT handler now that child processes have been spawned
        signal.signal(signal.SIGINT, prev_sigint)
        return runtime, tokenizer
    except Exception as e:
        logger.error(f"Failed to initialize SGLang Runtime: {e}")
        signal.signal(signal.SIGINT, prev_sigint)
        return None, None


def start_api_server(runtime, model_path: str, port: int = 5000,
                     reasoning_parser: Optional[str] = None):
    """Start the OpenAI-compatible API server on a daemon thread. Returns server or None."""
    if runtime is None:
        return None
    try:
        from sglang_api_server import SGLangAPIServer
        server = SGLangAPIServer(runtime=runtime, model_path=model_path, port=port,
                                 reasoning_parser=reasoning_parser)
        server.start()
        logger.info(f"OpenAI-compatible API server started on port {port}"
                    + (f" (reasoning_parser={reasoning_parser})" if reasoning_parser else ""))
        return server
    except Exception as e:
        logger.warning(f"Failed to start API server: {e}")
        return None


# ---------------------------------------------------------------------------
# Shared-service subprocess helpers
# ---------------------------------------------------------------------------

def launch_ui(scenario_name: str, port: int) -> Optional[subprocess.Popen]:
    try:
        args = [sys.executable, 'fastapi_action_display.py', '--port', str(port)]
        if scenario_name:
            args.extend(['--scenario', scenario_name])
        proc = subprocess.Popen(args)
        logger.info(f"FastAPI UI launched on port {port}  http://localhost:{port}")
        return proc
    except Exception as e:
        logger.error(f"Failed to launch FastAPI UI: {e}")
        return None


def launch_resource_browser(world_label: str, character: str = "") -> Optional[subprocess.Popen]:
    try:
        cmd = [sys.executable, 'resource_browser.py', '--map', world_label, '--port', '3001', '--no-browser']
        if character:
            cmd += ['--character', character]
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        logger.info(f"Resource Browser launched on port 3001 (character={character or '*'})")
        return proc
    except Exception as e:
        logger.error(f"Failed to launch Resource Browser: {e}")
        return None


_BROWSER_CANDIDATES = (
    # google-chrome preferred over chromium because snap-confined chromium
    # silently drops some flags (--window-position, --window-size) due to
    # AppArmor. google-chrome is non-snap and respects flags reliably.
    'google-chrome', 'google-chrome-stable',
    'chromium', 'chromium-browser',
    'brave-browser', 'microsoft-edge',
)


def _resolve_browser(override: Optional[str]) -> Optional[str]:
    """Return a browser binary suitable for --app= chromium-style launches,
    or None if none found. `override` is taken verbatim if truthy."""
    import shutil
    if override:
        return override
    for c in _BROWSER_CANDIDATES:
        p = shutil.which(c)
        if p:
            return p
    return None


def _parse_size(s: str, default: str) -> str:
    """Normalize 'WxH' or 'W,H' to chromium's 'W,H' format."""
    txt = (s or default).strip().lower().replace('x', ',').replace(' ', '')
    parts = txt.split(',')
    if len(parts) != 2 or not all(p.isdigit() for p in parts):
        logger.warning(f"unparseable size {s!r}; using default {default!r}")
        txt = default.replace('x', ',').replace(' ', '')
    return txt


def _launch_widget_window(
    label: str,
    html_path: str,
    size: str,
    pos: str,
    browser: Optional[str],
) -> Optional[subprocess.Popen]:
    """Open html_path as a chromium app-mode window. Uses --user-data-dir
    pointed at a fresh tmp profile to prevent the WM from inheriting a
    previous 'maximized' state — the cause of the affect window opening
    full-screen on snap chromium."""
    b = _resolve_browser(browser)
    if not b:
        logger.warning(
            f"{label}: no chromium-family browser on PATH; widget window "
            "not opened. Set --browser <path> or install chromium/chrome.")
        return None
    profile_dir = f"/tmp/jill-{label}-{os.getpid()}"
    cmd = [
        b,
        f'--app=file://{html_path}',
        f'--window-size={size}',
        f'--window-position={pos}',
        f'--user-data-dir={profile_dir}',
        # Fresh profile per widget ⇒ Chrome's first-run welcome + default-
        # browser prompt would fire every time. Both irrelevant for an
        # app-mode display surface.
        '--no-first-run',
        '--no-default-browser-check',
    ]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        logger.info(f"{label} widget window launched (browser={b}, size={size}, pos={pos})")
        return proc
    except Exception as e:
        logger.error(f"{label}: failed to launch browser: {e}")
        return None


def launch_affect_display(size: str, pos: str, browser: Optional[str]
                          ) -> List[subprocess.Popen]:
    """Spawn the affect-display bridge + chromium widget window.
    Returns the list of subprocesses to track for cleanup."""
    procs: List[subprocess.Popen] = []
    try:
        bridge = subprocess.Popen(
            [sys.executable, '-m', 'affect.display'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(bridge)
        logger.info("affect bridge launched (ws://127.0.0.1:8787)")
    except Exception as e:
        logger.error(f"affect: failed to launch bridge: {e}")
        return procs
    # Brief pause so the WS server is listening when chromium connects.
    # The JS shim reconnects on failure, so this is just an optimization.
    time.sleep(0.4)
    html = os.path.join(_THIS_DIR := os.path.dirname(os.path.abspath(__file__)),
                        'affect', 'display', 'static', 'index.html')
    win = _launch_widget_window('affect', html, size, pos, browser)
    if win is not None:
        procs.append(win)
    return procs


def launch_canvas_display(character: str, size: str, pos: str,
                          browser: Optional[str]) -> List[subprocess.Popen]:
    """Spawn the canvas-display bridge + chromium widget window for
    the given character. Returns subprocesses to track."""
    procs: List[subprocess.Popen] = []
    env = os.environ.copy()
    env['CANVAS_CHARACTER'] = character
    try:
        bridge = subprocess.Popen(
            [sys.executable, '-m', 'canvas.display'],
            env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(bridge)
        logger.info(f"canvas bridge launched (character={character}, ws://127.0.0.1:8788)")
    except Exception as e:
        logger.error(f"canvas: failed to launch bridge: {e}")
        return procs
    time.sleep(0.4)
    html = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'canvas', 'display', 'static', 'index.html')
    win = _launch_widget_window('canvas', html, size, pos, browser)
    if win is not None:
        procs.append(win)
    return procs


def launch_telegram_bridge(character: str) -> Optional[subprocess.Popen]:
    """Spawn telegram_bridge.py as a sibling subprocess.

    The bridge is opt-in (--telegram); we fail loudly here rather than
    after launch if the env vars are missing, so the user finds out
    immediately rather than from a silently-not-replying bot.
    """
    if not os.getenv('TELEGRAM_BOT_TOKEN', '').strip():
        logger.error(
            "Telegram bridge requested but TELEGRAM_BOT_TOKEN is not set; "
            "skipping. Export the bot token from @BotFather and retry."
        )
        return None
    if not os.getenv('TELEGRAM_ALLOWED_CHAT_IDS', '').strip():
        logger.error(
            "Telegram bridge requested but TELEGRAM_ALLOWED_CHAT_IDS is "
            "empty; skipping. The allowlist is the only authorization "
            "boundary — refusing to start a publicly-DM-able bridge."
        )
        return None
    try:
        cmd = [sys.executable, 'telegram_bridge.py', '--character', character]
        proc = subprocess.Popen(cmd)
        logger.info(f"Telegram bridge launched (character={character})")
        return proc
    except Exception as e:
        logger.error(f"Failed to launch Telegram bridge: {e}")
        return None


def launch_image_server() -> Optional[subprocess.Popen]:
    """Fire-and-forget prestart of the local Bonsai text-to-image backend so
    the generate-image tool is warm by first use.

    Reuses the tool's own launch logic (serve.sh path + binary-transformer
    workaround) as the single source of truth — the tool's dir is hyphenated
    and isn't importable as a package, so we load it by file path."""
    import importlib.util
    tool_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'tools', 'generate-image', 'tool.py')
    try:
        spec = importlib.util.spec_from_file_location('generate_image_tool', tool_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    except Exception as e:
        logger.error(f"image-server: could not load generate-image tool ({e}); skipping")
        return None
    proc = mod.start_image_server()
    if proc is not None:
        logger.info("image server prestart launched (warming in background)")
    else:
        logger.info("image server already up (or could not start) — see generate-image log")
    return proc


# ---------------------------------------------------------------------------
# Agent thread
# ---------------------------------------------------------------------------

def run_agent(name: str, config: dict, runtime, tokenizer, shutdown_event: threading.Event):
    """Entry point for each character thread.

    Dispatches on config['mode']:
      - 'chat'  → lightweight ChatLoop. No OODA, no goals, no task scheduler,
                  but sensors (e.g. tick) and concerns are wired through.
      - default → full ZenohExecutiveNode OODA loop
    """
    mode = config.get('mode', 'agent')
    if mode == 'chat':
        try:
            from chat.chat_loop import ChatLoop
            logger.info(f"Starting chat thread: {name}")
            loop = ChatLoop(character_name=name, character_config=config, shutdown_event=shutdown_event)
            loop.run()
        except Exception as e:
            logger.error(f"Chat {name} crashed: {e}")
            import traceback
            traceback.print_exc()
        return

    node = None
    try:
        try:
            from executive_node import ZenohExecutiveNode
        except ImportError as e:
            logger.error(
                f"Cannot start non-chat character {name!r}: OODA mode is no "
                f"longer supported in this build (executive_node not "
                f"importable: {e}). Use a chat-mode scenario instead."
            )
            return
        logger.info(f"Starting agent thread: {name}")
        node = ZenohExecutiveNode(character_name=name, character_config=config, runtime=runtime, tokenizer=tokenizer)
        # Hand the shared shutdown event to the node so its sensor threads
        # (spawned in node.run() via _start_sensors) can wait on the same
        # event the launcher uses for SIGINT/SIGTERM coordination.
        node._shutdown_event = shutdown_event
        # Start a tiny daemon thread that propagates the shared event to this node
        def _watch_shutdown():
            shutdown_event.wait()
            if node:
                node.shutdown_requested = True
        watcher = threading.Thread(target=_watch_shutdown, daemon=True)
        watcher.start()

        # Run the OODA loop (blocks until node.shutdown_requested becomes True)
        node.run()
    except Exception as e:
        logger.error(f"Agent {name} crashed: {e}")
        import traceback
        traceback.print_exc()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Cognitive Workbench Launcher')
    parser.add_argument('config_file', help='YAML configuration file (in scenarios/)')
    parser.add_argument('--characters', nargs='+', help='Override which characters to launch')
    parser.add_argument('--list-only', action='store_true', help='List characters and exit')
    parser.add_argument('--ui', action='store_true', help='Launch FastAPI web UI')
    parser.add_argument('--ui-port', type=int, default=3000, help='Port for web UI (default: 3000)')
    parser.add_argument('--resource-browser', action='store_true', help='Launch Resource Browser (port 3001)')
    parser.add_argument('--telegram', action='store_true',
                        help='Launch Telegram bridge subprocess (requires '
                             'TELEGRAM_BOT_TOKEN and TELEGRAM_ALLOWED_CHAT_IDS '
                             'env vars; relays inbound DMs to the primary '
                             'character and forwards outbound say events back)')
    parser.add_argument('--cli', action='store_true', help='Launch interactive CLI chat interface')
    parser.add_argument('--autonomy', action='store_true',
                        help='Enable autonomous concern firing in chat mode (off by default; '
                             'when off, the tick handler is a no-op so benchmarks and chat '
                             'scenarios behave identically regardless of active concerns).')
    parser.add_argument('--affect', action='store_true',
                        help='Launch the affect (processing-state) widget window.')
    parser.add_argument('--affect-size', default='320x320',
                        help='Affect window size WxH or W,H (default: 320x320).')
    parser.add_argument('--affect-pos', default='60,60',
                        help='Affect window position X,Y (default: 60,60).')
    parser.add_argument('--canvas', action='store_true',
                        help='Launch the canvas (rich-display) window. Uses the '
                             'first launched character as its target.')
    parser.add_argument('--canvas-size', default='820x640',
                        help='Canvas window size WxH or W,H (default: 820x640).')
    parser.add_argument('--canvas-pos', default='540,60',
                        help='Canvas window position X,Y (default: 540,60).')
    parser.add_argument('--browser', default=None,
                        help='Browser binary for widget windows (default: auto-detect '
                             'chromium/chrome/brave/edge).')
    parser.add_argument('--image-server', action='store_true',
                        help='Prestart the local Bonsai text-to-image backend so the '
                             'generate-image tool is warm (otherwise the tool cold-starts '
                             'it on first use). Fire-and-forget; warms while jill boots.')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    if args.debug:
        os.environ['CWB_DEBUG'] = '1'

    # ---- Load scenario ----
    try:
        config_data = load_scenario(args.config_file)
    except Exception as e:
        print(f"Error loading config: {e}")
        return

    scenario_name = Path(args.config_file).stem
    llm_config = config_data.get('llm_config', {})
    alt_llm_config = config_data.get('alt_llm_config', {})
    world_config = config_data.get('world_config', {})
    world_name = (world_config.get('world_name') if world_config else None) or scenario_name or 'infospace'
    setting = config_data.get('setting', {})

    # Optional max_turns
    max_turns = config_data.get('max_turns')
    if max_turns is not None:
        os.environ['CWB_MAX_TURNS'] = str(int(max_turns))

    # ---- Parse characters ----
    characters = parse_characters(config_data, llm_config, world_config, setting, alt_llm_config)

    # Command-line override
    if args.characters:
        characters = [(n.capitalize(), {}) for n in args.characters]

    if not characters:
        print("No characters defined in scenario")
        return

    # Add infospace flag and map name
    for _name, cfg in characters:
        cfg['is_infospace'] = True
        cfg['map_name'] = world_name
        cfg['autonomy_enabled'] = bool(args.autonomy)

    if args.list_only:
        print("Characters:")
        for name, _ in characters:
            print(f"  - {name}")
        return

    # ---- World data cleanup ----
    maybe_clean_world_data(world_name, [n for n, _ in characters])

    # ---- Shared SGLang runtime ----
    runtime, tokenizer = create_sglang_runtime(llm_config)
    api_server = start_api_server(runtime, llm_config.get('sgl_model_path', ''),
                                  reasoning_parser=llm_config.get('reasoning_parser'))

    # ---- Shared services (subprocesses) ----
    service_procs: List[subprocess.Popen] = []
    if args.ui:
        proc = launch_ui(scenario_name, args.ui_port)
        if proc:
            service_procs.append(proc)
    if args.resource_browser:
        primary_character = characters[0][0] if characters else ""
        proc = launch_resource_browser(world_name, character=primary_character)
        if proc:
            service_procs.append(proc)
    if args.telegram:
        primary_character = characters[0][0] if characters else ""
        proc = launch_telegram_bridge(character=primary_character)
        if proc:
            service_procs.append(proc)
    if args.image_server:
        proc = launch_image_server()
        if proc:
            service_procs.append(proc)

    # Affect (processing-state) and canvas (rich-display) widget windows.
    # Each spawns a Python bridge + a chromium app-mode window. Both are
    # tracked in service_procs so they get terminated on shutdown.
    if args.affect:
        affect_size = _parse_size(args.affect_size, '320,320')
        service_procs.extend(launch_affect_display(
            size=affect_size, pos=args.affect_pos, browser=args.browser))
    if args.canvas:
        primary_character = characters[0][0] if characters else ""
        if not primary_character:
            logger.warning("--canvas requested but no characters; window not opened.")
        else:
            canvas_size = _parse_size(args.canvas_size, '820,640')
            service_procs.extend(launch_canvas_display(
                character=primary_character,
                size=canvas_size, pos=args.canvas_pos, browser=args.browser))

    # ---- URL listener (shared; sensors drain the queue) ----
    # Start if any character declares a browser-visits sensor
    _any_browser_sensor = any(
        any(s.get('name') == 'browser-visits' for s in cfg.get('sensors', []))
        for _, cfg in characters
    )
    if _any_browser_sensor:
        try:
            from url_listener import start as start_url_listener
            start_url_listener(port=5004)
        except Exception as e:
            logger.warning(f"Failed to start URL listener: {e}")

    if service_procs:
        # Wait for UI to be healthy before launching agents (announcements need a listener)
        if args.ui:
            import urllib.request
            ui_url = f"http://localhost:{args.ui_port}/api/characters"
            for attempt in range(30):
                try:
                    urllib.request.urlopen(ui_url, timeout=1)
                    logger.info(f"UI is healthy (attempt {attempt + 1})")
                    break
                except Exception:
                    time.sleep(1)
            else:
                logger.warning("UI did not become healthy in 30s, launching agents anyway")
        else:
            time.sleep(2)

    # ---- Zenoh ready signal ----
    zenoh_session = None
    try:
        import zenoh
        from utils.zenoh_utils import make_localhost_config
        zenoh_session = zenoh.open(make_localhost_config())
        ready_pub = zenoh_session.declare_publisher("cognitive/launcher/ready")
        ready_pub.put(json.dumps({'status': 'ready', 'character_count': len(characters), 'timestamp': time.time()}))
        logger.info(f"Published ready signal: {len(characters)} characters")
    except Exception as e:
        logger.warning(f"Could not publish ready signal: {e}")

    # ---- Load sensor metadata (before agent launch so configs are enriched) ----
    all_sensors = {}
    try:
        from utils.sensor_loader import load_sensors, parse_schedule as _parse_schedule
        src_dir = Path(__file__).parent
        all_sensors = load_sensors(str(src_dir / 'sensors'))
    except Exception as e:
        logger.warning(f"Sensor metadata loading failed (non-fatal): {e}")

    # Enrich each character config with resolved sensor summaries for self-model.
    # Also stash the full sensor metadata dict on each character config so that
    # ZenohExecutiveNode._start_sensors() can construct SensorRunner instances
    # at agent-init time (with a real resource_manager) instead of having the
    # launcher spawn sensors with resource_manager=None.
    if all_sensors:
        for _char_name, _config in characters:
            sensor_summaries = []
            for sensor_cfg in _config.get('sensors', []):
                s_name = sensor_cfg.get('name', '')
                if s_name not in all_sensors:
                    continue
                meta = all_sensors[s_name]
                effective_disposition = sensor_cfg.get('disposition', meta.get('disposition', 'inform'))
                schedule_secs = meta.get('schedule_seconds', 300)
                if 'schedule' in sensor_cfg:
                    parsed = _parse_schedule(sensor_cfg['schedule'])
                    if parsed is not None:
                        schedule_secs = parsed
                sensor_summaries.append({
                    'name': s_name,
                    'type': meta.get('type', 'unknown'),
                    'disposition': effective_disposition,
                    'schedule_seconds': schedule_secs,
                    'description': meta.get('description', ''),
                })
            _config['_sensor_configs'] = sensor_summaries
            _config['_sensor_metadata_by_name'] = all_sensors

    # Validate sensor trigger dispositions against scheduled goals / task
    # templates. Done at config-load time (before any agent thread starts) so
    # the user gets early feedback if a sensor references a missing goal.
    #
    # Two sources are checked:
    #   (a) YAML-declared scheduled_goals[].name — for goals defined in the
    #       scenario config
    #   (b) Persisted scheduled goal IDs from the character's resources.json
    #       — for goals created at runtime via /goal add
    # Either match passes the validation. The runtime _resolve_goal_id at
    # OODA Decide handles both forms correctly; this validation just stops
    # warning the user about runtime IDs that legitimately exist.
    def _load_persisted_goal_ids(_world_name: str, _character_name: str) -> set:
        """Read the character's persisted resources.json and return the set of
        scheduled goal IDs (e.g. {'goal_1', 'goal_15'}). Returns empty set if
        the file is missing or unreadable; caller falls back to YAML-only."""
        import re as _re
        repo_root = Path(__file__).resolve().parent.parent
        candidate = repo_root / 'scenarios' / _world_name / 'resources' / _character_name / 'resources.json'
        if not candidate.exists():
            return set()
        try:
            text = candidate.read_text(encoding='utf-8', errors='replace')
            return {f"goal_{m}" for m in _re.findall(r'"_scheduled_goal_(\w+)"', text)}
        except Exception as _e:
            logger.debug(f"_load_persisted_goal_ids: read failed for {candidate}: {_e}")
            return set()

    if all_sensors:
        for _char_name, _config in characters:
            _char_world = (_config.get('world_config') or {}).get('world_name', world_name)
            _persisted_goal_ids = _load_persisted_goal_ids(_char_world, _char_name)
            for sensor_cfg in _config.get('sensors', []):
                s_name = sensor_cfg.get('name', '')
                if s_name not in all_sensors:
                    if s_name:
                        logger.warning(f"Sensor '{s_name}' declared for {_char_name} but not found in src/sensors/")
                    continue
                sensor_meta = all_sensors[s_name]
                effective_disposition = sensor_cfg.get('disposition', sensor_meta.get('disposition', 'inform'))
                if effective_disposition.startswith('trigger-task:'):
                    tmpl_name = effective_disposition[len('trigger-task:'):]
                    task_templates = [t.get('name', '') for t in _config.get('task_templates', [])]
                    if tmpl_name not in task_templates:
                        logger.warning(f"Sensor '{s_name}' has disposition 'trigger-task:{tmpl_name}' but template '{tmpl_name}' not found in {_char_name}'s task_templates")
                elif effective_disposition.startswith('trigger:'):
                    goal_name = effective_disposition[len('trigger:'):]
                    yaml_goal_names = [g.get('name', '') for g in _config.get('scheduled_goals', [])]
                    if goal_name not in yaml_goal_names and goal_name not in _persisted_goal_ids:
                        logger.warning(
                            f"Sensor '{s_name}' has disposition 'trigger:{goal_name}' but "
                            f"goal '{goal_name}' not found in {_char_name}'s YAML scheduled_goals "
                            f"({len(yaml_goal_names)} entries) or persisted runtime goals "
                            f"({len(_persisted_goal_ids)} entries)"
                        )

    # ---- Set CLI mode env var before agent threads import their logging configs ----
    if args.cli:
        os.environ['CWB_CLI_MODE'] = '1'

    # ---- Launch agent threads ----
    shutdown_event = threading.Event()
    threads: List[threading.Thread] = []
    for name, config in characters:
        t = threading.Thread(target=run_agent, args=(name, config, runtime, tokenizer, shutdown_event), name=f"agent-{name}", daemon=True)
        t.start()
        threads.append(t)
        time.sleep(0.5)  # Small stagger

    logger.info(f"All {len(threads)} agent threads started")

    # Sensor threads are now spawned by ZenohExecutiveNode._start_sensors()
    # at the start of run() — after init has constructed the agent's
    # resource_manager. The launcher's role for sensors is just to load the
    # metadata once (above) and validate the dispositions; spawning happens
    # in the agent thread so sensors get a real resource_manager reference.

    # ---- Wait for shutdown ----
    _interrupt_count = [0]

    def _signal_handler(signum, frame):
        _interrupt_count[0] += 1
        logger.info(f"Received signal {signum}, shutting down...")
        shutdown_event.set()
        if _interrupt_count[0] >= 2:
            # Second interrupt — force exit immediately
            print("\nForce exit.")
            os._exit(1)

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT, _signal_handler)

    # Also listen for Zenoh shutdown requests
    launcher_shutdown_sub = None
    if zenoh_session:
        def _zenoh_shutdown(sample):
            logger.warning("Received shutdown request via Zenoh")
            shutdown_event.set()
        launcher_shutdown_sub = zenoh_session.declare_subscriber("cognitive/launcher/shutdown", _zenoh_shutdown)

    if args.cli:
        # Suppress ALL console logging so only CLI output reaches the terminal.
        # File handlers continue logging at INFO+.
        root = logging.getLogger()
        for h in root.handlers[:]:
            if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
                root.removeHandler(h)
        # Also suppress console handlers on any named loggers already created
        for name in list(logging.Logger.manager.loggerDict):
            lg = logging.getLogger(name)
            for h in lg.handlers[:]:
                if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
                    lg.removeHandler(h)

        from cli import run_cli
        character_names = [n for n, _ in characters]
        try:
            run_cli(zenoh_session, character_names, shutdown_event)
        except KeyboardInterrupt:
            shutdown_event.set()
    else:
        try:
            # Block until shutdown requested
            while not shutdown_event.is_set():
                # Check if all agent threads have exited on their own
                if all(not t.is_alive() for t in threads):
                    logger.info("All agent threads have exited")
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            shutdown_event.set()

    # ---- Graceful shutdown ----
    logger.info("Shutting down...")
    shutdown_event.set()

    # Also publish Zenoh shutdown so nodes exit their OODA loops
    if zenoh_session:
        try:
            zenoh_session.put("cognitive/shutdown/executive", json.dumps({"source": "launcher"}).encode('utf-8'))
        except Exception:
            pass

    # Wait for agent threads
    for t in threads:
        t.join(timeout=10)
        if t.is_alive():
            logger.warning(f"Agent thread {t.name} did not exit in time")

    # Stop API server
    if api_server:
        try:
            api_server.stop()
        except Exception:
            pass

    # Terminate service subprocesses
    for proc in service_procs:
        try:
            proc.terminate()
        except Exception:
            pass
    for proc in service_procs:
        try:
            proc.wait(timeout=5)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    # Shutdown SGLang runtime: send SIGTERM for graceful drain, wait, then SIGKILL fallback
    if runtime:
        import signal as _signal
        import psutil as _psutil
        pid = getattr(runtime, "pid", None)
        runtime_shutdown_done = threading.Event()
        def _shutdown_runtime():
            try:
                if pid is not None:
                    try:
                        os.kill(pid, _signal.SIGTERM)
                        logger.info(f"Sent SIGTERM to SGLang Runtime (pid={pid})")
                        _psutil.Process(pid).wait(timeout=20)
                        logger.info("SGLang Runtime exited gracefully")
                    except _psutil.NoSuchProcess:
                        logger.info("SGLang Runtime already exited")
                    except _psutil.TimeoutExpired:
                        logger.warning("SGLang Runtime did not exit after SIGTERM; forcing kill")
                        runtime.shutdown()
                    except ProcessLookupError:
                        logger.info("SGLang Runtime already exited")
                else:
                    runtime.shutdown()
                    logger.info("SGLang Runtime shut down")
            except Exception as e:
                logger.warning(f"Error shutting down runtime: {e}")
            finally:
                runtime_shutdown_done.set()
        threading.Thread(target=_shutdown_runtime, name="runtime-shutdown", daemon=True).start()
        if not runtime_shutdown_done.wait(timeout=30):
            logger.warning("Timed out shutting down SGLang Runtime; continuing exit")

    # Close Zenoh
    if zenoh_session:
        try:
            zenoh_session.close()
        except Exception:
            pass

    logger.info("Shutdown complete")
    os._exit(0)


if __name__ == '__main__':
    main()
