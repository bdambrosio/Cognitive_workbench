# Repository Guidelines

## Project Structure & Module Organization
Cognitive Workbench is a pure-Python Zenoh stack. `launcher.py` orchestrates `memory_node.py`, `situation_node.py`, `executive_node.py`, and `skill_executor.py` instances defined in `characters.yaml`. Spatial reasoning and visualization live in `map_node.py`, `map_agent.py`, `world_map.py`, and the `maps/` and `render/` directories. RAG stores and planner traces stay in `data/`; helpers live in `utils/` and `templates.py`. Runtime output belongs in `logs/`; keep generated assets outside Git.

## Build, Test, and Development Commands
- `python -m venv .venv && source .venv/bin/activate` — create a Python 3.10+ virtual environment.
- `pip install -r requirements.txt` — install Zenoh, FastAPI, visualization, and pytest dependencies.
- `python launcher.py characters.yaml --map-file maps/rural.py --ui --resource-browser` — run the multi-character loop with optional UI/debugging.
- `python fastapi_action_display.py --port 3000` — iterate on the FastAPI dashboard without respawning nodes.
- `python render/test_visualization.py` or `python utils/test_OpenAIClient.py` — smoke-test visualization and LLM connectivity.

## Coding Style & Naming Conventions
Use 4-space indentation, type hints, and concise docstrings (see `launcher.py`, `llm_client.py`). Keep modules and functions `snake_case`, classes `PascalCase`, and YAML keys lower-case. Prefer dataclasses for config bundles, keep functions cohesive, log via `logging`, and store reusable prompts or scenario data in `templates.py` and `maps/`.

## Testing Guidelines
Pytest ships with the requirements. Add suites beside the code (e.g., `render/test_<feature>.py`) and run them with `pytest -q path/to/test.py`, reserving `-s` when console output matters. Mock LLM or Zenoh calls via `utils.llm_api.LLM` fakes, keep fixtures in `data/tmp/`, and let GUI or map tests skip gracefully when assets are absent.

## Commit & Pull Request Guidelines
History uses short, imperative subjects (`Update README for clarity on LLM and planning`, `Added a task library...`); keep summaries <72 characters and prefix subsystems when helpful (`infospace: improve planner fallback`). PRs should describe the scenario exercised, note new CLI flags or configs, link issues, list manual steps (e.g., clear `data/vector/*`), and call out incompatible map or topic schema updates.

## Security & Configuration Tips
`utils/llm_api` expects API keys from the environment—never commit `.env` or tokens. Treat `setup.sh` as reference, keep `data/`, `logs/`, and generated RAG stores out of Git, sanitize map files before sharing, and store non-local Zenoh endpoints in deployment YAML rather than code.
