#!/usr/bin/env python3
"""
Scenario Manager

Lightweight FastAPI app to run on a server and allow a user to select a scenario.
Once selected, this app will:
- Dynamically allocate a UI port for the Action Display
- Launch the Action Display on that port
- Launch the launcher process to start all necessary Zenoh nodes for the scenario

Notes:
- This app avoids modifying existing code by not using the launcher's --ui flag.
  It starts the Action Display itself on the selected port, then calls the launcher
  without --ui (so no port conflicts).
- The launcher currently prompts for "Reuse existing world? (y/n)" when a world exists.
  To keep things simple and non-interactive for now, we default to reusing the world
  by sending "y" to the launcher's stdin. (Future: replace with a UI toggle.)
"""

import os
import sys
import json
import time
import socket
import random
import string
import logging
import threading
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, RedirectResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn


LOGGER = logging.getLogger("scenario_manager")
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)


# Absolute paths
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
SCENARIOS_DIR = PROJECT_ROOT / "scenarios"


class PortAllocator:
    """Simple incremental port allocator with availability check."""

    def __init__(self, base_port: int = 3000, max_port: int = 3999):
        self.base_port = base_port
        self.max_port = max_port
        self._lock = threading.Lock()
        self._in_use: set[int] = set()

    @staticmethod
    def _is_port_free(port: int) -> bool:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(("0.0.0.0", port))
                return True
            except OSError:
                return False

    def acquire(self) -> int:
        with self._lock:
            for port in range(self.base_port, self.max_port + 1):
                if port in self._in_use:
                    continue
                if self._is_port_free(port):
                    self._in_use.add(port)
                    LOGGER.info(f"Allocated UI port {port}")
                    return port
            raise RuntimeError("No free ports available in the configured range")

    def release(self, port: int) -> None:
        with self._lock:
            if port in self._in_use:
                self._in_use.remove(port)
                LOGGER.info(f"Released UI port {port}")


def generate_scenario_run_id() -> str:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = ''.join(random.choices(string.digits, k=4))
    return f"run_{timestamp}_{suffix}"


def list_scenarios() -> List[Dict[str, str]]:
    """Return available scenarios with optional map parsed from the first line."""
    if not SCENARIOS_DIR.exists():
        return []
    items: List[Dict[str, str]] = []
    for path in sorted(SCENARIOS_DIR.glob("*.yaml")):
        map_name = ""
        try:
            with path.open("r", encoding="utf-8") as f:
                first_line = f.readline().strip()
                if first_line.startswith("map:"):
                    map_name = first_line.split(":", 1)[1].strip()
        except Exception:
            pass
        items.append({
            "name": path.name,
            "map": map_name,
        })
    return items


def world_exists(map_file: str) -> bool:
    """Check if a world file exists for the provided map name."""
    if not map_file.endswith(".py"):
        return False
    world_name = map_file[:-3]
    world_file = PROJECT_ROOT / "src" / "data" / "world" / f"{world_name}_world.json"
    return world_file.exists()


class ScenarioProcess:
    """Holds process info for a running scenario."""
    def __init__(self, run_id: str, scenario_file: str, map_file: Optional[str], ui_port: int,
                 display_proc: subprocess.Popen, launcher_proc: subprocess.Popen):
        self.run_id = run_id
        self.scenario_file = scenario_file
        self.map_file = map_file
        self.ui_port = ui_port
        self.display_proc = display_proc
        self.launcher_proc = launcher_proc
        self.start_time = time.time()

    def to_dict(self) -> Dict[str, str]:
        return {
            "run_id": self.run_id,
            "scenario": self.scenario_file,
            "map": self.map_file or "",
            "ui_port": str(self.ui_port),
            "display_pid": str(self.display_proc.pid if self.display_proc and self.display_proc.poll() is None else -1),
            "launcher_pid": str(self.launcher_proc.pid if self.launcher_proc and self.launcher_proc.poll() is None else -1),
        }


class ScenarioManager:
    """Coordinates scenario discovery and launching."""

    def __init__(self, ui_base_port: int = 3000):
        self.port_allocator = PortAllocator(base_port=ui_base_port)
        self._lock = threading.Lock()
        self._runs: Dict[str, ScenarioProcess] = {}

    def start_scenario(self, scenario_file: str, reuse_world: bool = True) -> Tuple[str, int]:
        """Start the action display on a dynamic port and then run the launcher."""
        scenario_path = SCENARIOS_DIR / scenario_file
        if not scenario_path.exists():
            raise FileNotFoundError(f"Scenario file not found: {scenario_file}")

        # Parse first line for map=... or map: ... if present
        map_file: Optional[str] = None
        try:
            with scenario_path.open("r", encoding="utf-8") as f:
                first_line = f.readline().strip()
                if first_line.startswith("map="):
                    map_file = first_line.split("=", 1)[1].strip()
                elif first_line.startswith("map:"):
                    map_file = first_line.split(":", 1)[1].strip()
        except Exception:
            map_file = None

        # Allocate a port for the Action Display UI
        ui_port = self.port_allocator.acquire()

        # Start the Action Display
        display_cmd = [
            sys.executable,
            "fastapi_action_display.py",
            "--port", str(ui_port),
        ]
        LOGGER.info(f"Starting Action Display on port {ui_port}: {' '.join(display_cmd)}")
        display_proc = subprocess.Popen(
            display_cmd,
            cwd=str(SRC_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Start the launcher WITHOUT --ui (to avoid port conflict)
        launcher_cmd = [
            sys.executable,
            "launcher.py",
            scenario_file,
        ]
        if map_file:
            launcher_cmd.extend(["--map-file", map_file])

        LOGGER.info(f"Starting launcher: {' '.join(launcher_cmd)}")

        # Open stdin pipe and proactively send 'y' to reuse world if prompted.
        launcher_proc = subprocess.Popen(
            launcher_cmd,
            cwd=str(SRC_DIR),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

        def _feed_stdin_reuse_world(proc: subprocess.Popen) -> None:
            try:
                # Give the process a moment to reach any prompt
                time.sleep(1.0)
                if proc.poll() is None and proc.stdin:
                    # Send user's choice to avoid blocking
                    proc.stdin.write(("y\n") if reuse_world else ("n\n"))
                    proc.stdin.flush()
            except Exception:
                pass

        threading.Thread(target=_feed_stdin_reuse_world, args=(launcher_proc,), daemon=True).start()

        run_id = generate_scenario_run_id()
        with self._lock:
            self._runs[run_id] = ScenarioProcess(
                run_id=run_id,
                scenario_file=scenario_file,
                map_file=map_file,
                ui_port=ui_port,
                display_proc=display_proc,
                launcher_proc=launcher_proc,
            )

        return run_id, ui_port

    def list_runs(self) -> List[Dict[str, str]]:
        with self._lock:
            return [r.to_dict() for r in self._runs.values()]


app = FastAPI(title="Scenario Manager", version="0.1")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

manager = ScenarioManager(ui_base_port=3000)


@app.get("/", response_class=HTMLResponse)
def index_page():
    # Minimal HTML page using basic fetch for KISS
    html = f"""
<!DOCTYPE html>
<html>
<head>
  <meta charset=\"UTF-8\" />
  <title>Scenario Manager</title>
  <style>
    :root {{
      --bg: #0f0f0f;
      --panel: #1b1b1b;
      --panel-border: #2f2f2f;
      --text: #e6e6e6;
      --muted: #a0a0a0;
      --accent: #00d4ff;
      --accent-strong: #00b8e6;
      --input-bg: #141414;
      --input-border: #2f2f2f;
    }}
    * {{ box-sizing: border-box; }}
    body {{ font-family: Arial, sans-serif; background:var(--bg); color:var(--text); margin:0; padding:20px; }}
    .container {{ max-width: 820px; margin: 0 auto; }}
    .card {{ background:var(--panel); padding: 20px; border-radius: 8px; border: 1px solid var(--panel-border); }}
    h1 {{ color:var(--accent); margin: 0 0 8px; }}
    .row {{ margin-top:12px; display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }}
    .muted {{ color:var(--muted); font-size: 12px; }}
    select {{ padding:8px 12px; background: var(--input-bg); color: var(--text); border: 1px solid var(--input-border); border-radius: 6px; min-width: 260px; outline: none; }}
    select:focus {{ border-color: var(--accent); box-shadow: 0 0 0 2px rgba(0,212,255,0.15); }}
    button {{ padding:8px 14px; border: none; border-radius: 6px; background: var(--accent); color: #0f0f0f; font-weight: 700; cursor: pointer; }}
    button:hover {{ background: var(--accent-strong); }}
    button:active {{ transform: translateY(1px); }}
    pre {{ background: var(--bg); color: var(--text); border: 1px solid var(--panel-border); border-radius: 8px; padding: 12px; overflow: auto; }}
  </style>
  <script>
    async function loadScenarios() {{
      const res = await fetch('/api/scenarios');
      const data = await res.json();
      const sel = document.getElementById('scenario');
      sel.innerHTML = '';
      data.forEach(item => {{
        const opt = document.createElement('option');
        opt.value = item.name;
        const mapText = item.map ? ` (map: ${{item.map}})` : '';
        opt.textContent = item.name + mapText;
        sel.appendChild(opt);
      }});
    }}

    async function startScenario() {{
      const sel = document.getElementById('scenario');
      const name = sel.value;
      const reuse = document.getElementById('reuse').checked;
      if (!name) {{ alert('Select a scenario'); return; }}
      const res = await fetch(`/api/start/${{encodeURIComponent(name)}}?reuse=${{reuse}}`, {{ method: 'POST' }});
      if (!res.ok) {{
        const txt = await res.text();
        alert('Failed to start scenario: ' + txt);
        return;
      }}
      const {{ url }} = await res.json();
      // Redirect to Action Display
      window.location.href = url;
    }}

    async function refreshRuns() {{
      const res = await fetch('/api/runs');
      const data = await res.json();
      const pre = document.getElementById('runs');
      pre.textContent = JSON.stringify(data, null, 2);
    }}

    window.addEventListener('DOMContentLoaded', async () => {{
      await loadScenarios();
      await refreshRuns();
      setInterval(refreshRuns, 5000);
    }});
  </script>
  
</head>
<body>
  <div class="container">
    <div class="card">
      <h1>Scenario Manager</h1>
      <div class="row">
        <label for="scenario">Scenario:&nbsp;</label>
        <select id="scenario"></select>
        <label class="muted" style="display:flex; align-items:center; gap:6px;">
          <input type="checkbox" id="reuse" checked /> Reuse existing world
        </label>
        <button onclick="startScenario()">Start</button>
      </div>
      <div class="row muted">After starting, you will be redirected to the scenario's Action Display UI on its own port.</div>
    </div>
    <div class="card" style="margin-top:16px;">
      <h3>Running Scenarios</h3>
      <pre id="runs"></pre>
    </div>
  </div>
</body>
</html>
    """
    return HTMLResponse(content=html)


@app.get("/api/scenarios")
def api_scenarios():
    return JSONResponse(list_scenarios())


@app.get("/api/runs")
def api_runs():
    return JSONResponse(manager.list_runs())


@app.post("/api/start/{scenario_name}")
def api_start_scenario(scenario_name: str, reuse: bool = True):
    try:
        run_id, ui_port = manager.start_scenario(scenario_name, reuse_world=reuse)
        # Build redirect URL based on the host the manager is served from.
        # Client will request http(s)://host:manager_port/api/start/... and then redirect to host:ui_port
        # Use window.location.host on the client; server provides only the port here.
        url = f"http://{os.environ.get('SCENARIO_MANAGER_EXTERNAL_HOST', 'localhost')}:{ui_port}"
        return JSONResponse({"run_id": run_id, "url": url})
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        LOGGER.exception("Failed to start scenario")
        raise HTTPException(status_code=500, detail=str(e))


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Scenario Manager")
    parser.add_argument("--port", type=int, default=3100, help="Port to host the scenario manager UI (default: 3100)")
    parser.add_argument("--ui-base-port", type=int, default=3000, help="Base port for Action Display UIs (default: 3000)")
    args = parser.parse_args()

    # Update manager with desired base port
    global manager
    manager = ScenarioManager(ui_base_port=args.ui_base_port)

    # Start FastAPI
    uvicorn.run(app, host="0.0.0.0", port=args.port)


if __name__ == "__main__":
    main()


