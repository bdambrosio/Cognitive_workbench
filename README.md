# Cognitive Workbench

Code as Laboratory for LLM cognitive architecture research.

[![Status: Research Laboratory](https://img.shields.io/badge/status-research_laboratory-purple.svg)]()
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## What this is

This repo is experimental research software. It prioritizes **inspectable agent behavior** and fast iteration over stability.

Core ideas:
- **Incremental planning**: the planner interleaves reasoning and tool execution.
- **Infospace memory**: Notes + Collections are treated as working memory.
- **World integrations** (optional): scenario-specific “world tools” (e.g., Minecraft) can be loaded per character.

## Wiki (recommended entry points)

The GitHub wiki is intended to be the primary long-form documentation surface.

- [Architecture overview](https://github.com/bdambrosio/Cognitive_workbench/wiki/Architecture)
- [Incremental planner](https://github.com/bdambrosio/Cognitive_workbench/wiki/Incremental-Planner)
- [Infospace memory (Notes & Collections)](https://github.com/bdambrosio/Cognitive_workbench/wiki/Infospace)
- [Tools & primitives (uniform return, tool catalog)](https://github.com/bdambrosio/Cognitive_workbench/wiki/Tools)
- [World integrations (world_config, world_state)](https://github.com/bdambrosio/Cognitive_workbench/wiki/Worlds)
- [Minecraft guide](https://github.com/bdambrosio/Cognitive_workbench/wiki/Minecraft)
- [UI guide (FastAPI + WebSockets)](https://github.com/bdambrosio/Cognitive_workbench/wiki/UI)
- [Troubleshooting](https://github.com/bdambrosio/Cognitive_workbench/wiki/Troubleshooting)

If you rename these pages later, update the links here.

## Quick start

### Prerequisites

- Python 3.10+
- An LLM backend configured for your environment (SGLang is commonly used for planning)

### Install

```bash
python3 -m venv zenoh_venv
source zenoh_venv/bin/activate
pip install -r requirements.txt
```

### Run (UI)

From `src/`, run the launcher with UI enabled.

```bash
cd src
python3 launcher.py <your-characters-yaml> --ui --resource-browser
```

Open the UI at `http://localhost:3000`.

## How tools work (important)

### Schema + implementation

Tools are defined by:
- `Skill.md`: tool interface / contract (inputs, outputs, behavior)
- `tool.py`: implementation

Core tools live in `src/tools/`. World-specific tools live in `src/world-tools/<world_name>/`.

### Uniform return format

All tools return a uniform dictionary via `InfospaceExecutor._create_uniform_return()`:
- `result["status"]`: `"success"` or `"failed"`
- `result["data"]`: **raw value** (dict/list/etc.) on success (or reason/value on failure)
- `result["value"]`: formatted/truncated string for display/logging
- `result["reason"]`: failure reason (only on failure)

When consuming tool outputs programmatically, **use `data`**, not `value`.

### Tool catalog ordering and sources

The planner sees tools grouped by **source**:
- World tools first (e.g., `#MINECRAFT`)
- Then core infospace tools (`#INFOSPACE CORE`)

Each tool entry includes a `source` field: `"core"` or `"<world_name>"`.

## Worlds: loading, state, and UI

World integrations are enabled per character via `world_config.world_name` in your scenario config.

Key mechanics:
- World tools are loaded from `src/world-tools/<world_name>/`.
- `InfospaceExecutor` maintains a persistent `world_state` dict for the active world.
- The UI has a **State** tab that displays `world_state` (updates on `set_world_state`).

## Minecraft (current state)

Minecraft support lives under `src/world-tools/minecraft/`.

Highlights:
- Navigation history is tracked in `world_state.nav` as a **list** (most recent first, bounded).
- Yaw is normalized to **0–360**.
- `minecraft/init` is auto-run on executor initialization (if present) to normalize pose and seed nav state.
- Mapping uses a **SpatialMap** compiled representation (`mc-map-update`, `mc-map-query`, `mc-map-visualize`).
  - Legacy map-Collection queries are deprecated; SpatialMap is the source of truth.

### Recent Updates (2026-01)

**Navigation-First Observation**:
- `mc-observe-blocks` now uses **navigation surface scanning** (`nav_surface`) instead of visual occlusion-based scanning.
- Scans 2D (x,z) cells within a forward cone (yaw ±60°, pitch -60° to +90°) and probes downward to find supporting blocks.
- Non-supporting cover blocks (snow layers, grass) don't occlude navigation surface detection.
- Default radius: **7 blocks** (max 7).

**Spatial Map Improvements**:
- **Query-time attributes**: "blocked", "step_up", "step_down" are computed dynamically relative to current position (not stored statically).
- Removed agent-relative data (`delta_y_from_agent`) from map storage.
- Support detection uses `adjacent_blocks['down']` for reliable ground detection.
- Map cells store absolute `support_y`; surface = `support_y + 1`.

**Path Planning**:
- `path-frontier` now **over-approximates** frontier candidates (default `allow_unknown=True`).
- Handles climb/descend actions (y±1) correctly in simulation.
- Returns frontier positions even with partial map coverage.

**Observation Tools**:
- `mc-observe-blocks`, `mc-observe-entities`, `mc-observe-items` all use **cone-based visibility** (120° horizontal, pitch -60° to +90°).
- Default radius: **7 blocks** (max 7).
- Entities/items include `dx`, `dy`, `dz` offsets and `distance` from agent position.

## Repository structure (high level)

```
src/
  executive_node.py
  infospace_executor.py
  incremental_planner.py
  fastapi_action_display.py
  tools/
  world-tools/
    minecraft/
scenarios/
tests/
```

Note: runtime-generated scenario resources live under `scenarios/*/resources/` and are ignored by git.

## Contributing

See `src/AGENTS.md` for repository guidelines and contributor expectations.

## License

MIT License - see `LICENSE`.
