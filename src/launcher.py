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
console_handler.setLevel(logging.INFO if _debug_env else logging.WARNING)

file_handler = logging.FileHandler('logs/character_launcher.log', mode='w')
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
            new_config['llm_config'] = llm_config
            new_config['alt_llm_config'] = alt_llm
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
                new_config['llm_config'] = llm_config
                new_config['alt_llm_config'] = alt_llm
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
        logger.info(f"Initializing SGLang Runtime with model: {sgl_model_path}")
        tokenizer = AutoTokenizer.from_pretrained(sgl_model_path)

        if sgl_model_path.startswith("allenai/Olmo-3"):
            runtime = sgl.Runtime(model_path=sgl_model_path, context_length=32768, cuda_graph_max_bs=4, tp_size=1, mem_fraction_static=0.82, attention_backend="triton")
        elif 'NVFP4' in sgl_model_path: # patch for Qwen3.5-122B-A10B-NVFP4 models as of 3/1/2026
            logger.info(f"🚀 Initializing SGLang Runtime with NVFP4 patch!")
            runtime = sgl.Runtime(model_path=sgl_model_path,device="cuda",context_length=65536,tp_size=1,mem_fraction_static=0.9,quantization="modelopt_fp4",attention_backend="triton")
        elif 'FP8' in sgl_model_path:
            logger.info("Initializing SGLang Runtime with FP8 patch")
            runtime = sgl.Runtime(model_path=sgl_model_path, tokenizer_path=sgl_model_path, device="cuda", context_length=65536, dtype="auto", tp_size=1, mem_fraction_static=0.9, fp8_gemm_runner_backend="triton", attention_backend="flashinfer")
        else:
            runtime = sgl.Runtime(model_path=sgl_model_path, tokenizer_path=sgl_model_path, device="cuda", context_length=65536, dtype="auto", tp_size=1, mem_fraction_static=0.9, attention_backend="flashinfer")

        sgl.set_default_backend(runtime)
        logger.info(f"SGLang Runtime initialized (model={sgl_model_path})")
        return runtime, tokenizer
    except Exception as e:
        logger.error(f"Failed to initialize SGLang Runtime: {e}")
        return None, None


def start_api_server(runtime, model_path: str, port: int = 5000):
    """Start the OpenAI-compatible API server on a daemon thread. Returns server or None."""
    if runtime is None:
        return None
    try:
        from sglang_api_server import SGLangAPIServer
        server = SGLangAPIServer(runtime=runtime, model_path=model_path, port=port)
        server.start()
        logger.info(f"OpenAI-compatible API server started on port {port}")
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


def launch_resource_browser(world_label: str) -> Optional[subprocess.Popen]:
    try:
        proc = subprocess.Popen([sys.executable, 'resource_browser.py', '--map', world_label, '--port', '3001', '--no-browser'])
        logger.info("Resource Browser launched on port 3001")
        return proc
    except Exception as e:
        logger.error(f"Failed to launch Resource Browser: {e}")
        return None


# ---------------------------------------------------------------------------
# Agent thread
# ---------------------------------------------------------------------------

def run_agent(name: str, config: dict, runtime, tokenizer, shutdown_event: threading.Event):
    """Entry point for each character thread.
    
    The node's OODA loop checks node.shutdown_requested. We monitor the
    shared shutdown_event and propagate it to the node.
    """
    node = None
    try:
        from executive_node import ZenohExecutiveNode
        logger.info(f"Starting agent thread: {name}")
        node = ZenohExecutiveNode(character_name=name, character_config=config, runtime=runtime, tokenizer=tokenizer)
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

    if args.list_only:
        print("Characters:")
        for name, _ in characters:
            print(f"  - {name}")
        return

    # ---- World data cleanup ----
    maybe_clean_world_data(world_name, [n for n, _ in characters])

    # ---- Shared SGLang runtime ----
    runtime, tokenizer = create_sglang_runtime(llm_config)
    api_server = start_api_server(runtime, llm_config.get('sgl_model_path', ''))

    # ---- Shared services (subprocesses) ----
    service_procs: List[subprocess.Popen] = []
    if args.ui:
        proc = launch_ui(scenario_name, args.ui_port)
        if proc:
            service_procs.append(proc)
    if args.resource_browser:
        proc = launch_resource_browser(world_name)
        if proc:
            service_procs.append(proc)

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
        zenoh_session = zenoh.open(zenoh.Config())
        ready_pub = zenoh_session.declare_publisher("cognitive/launcher/ready")
        ready_pub.put(json.dumps({'status': 'ready', 'character_count': len(characters), 'timestamp': time.time()}))
        logger.info(f"Published ready signal: {len(characters)} characters")
    except Exception as e:
        logger.warning(f"Could not publish ready signal: {e}")

    # ---- Launch agent threads ----
    shutdown_event = threading.Event()
    threads: List[threading.Thread] = []
    for name, config in characters:
        t = threading.Thread(target=run_agent, args=(name, config, runtime, tokenizer, shutdown_event), name=f"agent-{name}", daemon=True)
        t.start()
        threads.append(t)
        time.sleep(0.5)  # Small stagger

    logger.info(f"All {len(threads)} agent threads started")

    # ---- Launch sensor threads ----
    sensor_threads: List[threading.Thread] = []
    try:
        from utils.sensor_loader import load_sensors
        from sensor_runner import SensorRunner

        src_dir = Path(__file__).parent
        all_sensors = load_sensors(str(src_dir / 'sensors'))

        if all_sensors:
            for char_name, config in characters:
                char_sensors = config.get('sensors', [])
                if not char_sensors:
                    continue
                for sensor_cfg in char_sensors:
                    s_name = sensor_cfg.get('name', '')
                    if s_name not in all_sensors:
                        logger.warning(f"Sensor '{s_name}' declared for {char_name} but not found in src/sensors/")
                        continue

                    sensor_meta = all_sensors[s_name]

                    # Build scenario overrides: parse schedule override if present
                    overrides = {}
                    if 'schedule' in sensor_cfg:
                        from utils.sensor_loader import parse_schedule
                        secs = parse_schedule(sensor_cfg['schedule'])
                        if secs is not None:
                            overrides['schedule_seconds'] = secs
                    if 'parameters' in sensor_cfg:
                        overrides['parameters'] = sensor_cfg['parameters']
                    if 'gate' in sensor_cfg:
                        overrides['gate'] = sensor_cfg['gate']
                    if 'disposition' in sensor_cfg:
                        overrides['disposition'] = sensor_cfg['disposition']

                    # Validate trigger dispositions against scheduled goals
                    effective_disposition = sensor_cfg.get('disposition', sensor_meta.get('disposition', 'inform'))
                    if effective_disposition.startswith('trigger:'):
                        goal_name = effective_disposition[len('trigger:'):]
                        scheduled_goals = [g.get('name', '') for g in config.get('scheduled_goals', [])]
                        if goal_name not in scheduled_goals:
                            logger.warning(f"Sensor '{s_name}' has disposition 'trigger:{goal_name}' but goal '{goal_name}' not found in {char_name}'s scheduled goals")

                    runner = SensorRunner(
                        sensor_name=s_name,
                        sensor_meta=sensor_meta,
                        character_name=char_name,
                        scenario_overrides=overrides,
                        resource_manager=None,  # Set after agent init if needed
                        zenoh_session=zenoh_session,
                        available_tools={},
                        shutdown_event=shutdown_event,
                    )

                    t = threading.Thread(target=runner.run, name=f"sensor-{char_name}-{s_name}", daemon=True)
                    t.start()
                    sensor_threads.append(t)
                    logger.info(f"Started sensor {s_name} for {char_name}")
    except Exception as e:
        logger.warning(f"Sensor loading failed (non-fatal): {e}")
        import traceback
        traceback.print_exc()

    if sensor_threads:
        logger.info(f"Started {len(sensor_threads)} sensor threads")

    # ---- Wait for shutdown ----
    def _signal_handler(signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        shutdown_event.set()

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT, _signal_handler)

    # Also listen for Zenoh shutdown requests
    launcher_shutdown_sub = None
    if zenoh_session:
        def _zenoh_shutdown(sample):
            logger.warning("Received shutdown request via Zenoh")
            shutdown_event.set()
        launcher_shutdown_sub = zenoh_session.declare_subscriber("cognitive/launcher/shutdown", _zenoh_shutdown)

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
