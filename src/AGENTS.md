# Repository Guidelines

## Project Structure & Module Organization
Cognitive Workbench is a pure-Python Zenoh stack. `launcher.py` orchestrates one or more `executive_node.py` instances defined in a scenario YAML under `scenarios/`. Planning is driven by `incremental_planner.py`, and tool execution is centralized in `infospace_executor.py` with persistence via `infospace_resource_manager.py`. The FastAPI dashboard lives in `fastapi_action_display.py`.

World integrations are optional and live under `src/world-tools/<world_name>/` (e.g., `minecraft/`). When a character config sets `world_config.world_name`, the executor loads the corresponding world tools and maintains persistent `world_state` (also surfaced in the UI “State” tab).

Generated/runtime assets must stay out of git. In particular, scenario-specific resources are written under `scenarios/*/resources/` (e.g., SpatialMap JSON files) and should be treated as local state.

## Build, Test, and Development Commands
- `python -m venv .venv && source .venv/bin/activate` — create a Python 3.10+ virtual environment (repo often uses `zenoh_venv/` too).
- `pip install -r requirements.txt` — install Zenoh, FastAPI, visualization, and pytest dependencies.
- `python launcher.py characters.yaml --ui --resource-browser` — run the multi-character loop with optional UI/debugging.
- `python fastapi_action_display.py --port 3000` — iterate on the FastAPI dashboard without respawning nodes.
- `python utils/test_OpenAIClient.py` — smoke-test LLM connectivity.

## Coding Style & Naming Conventions
Use 4-space indentation, type hints, and concise docstrings (see `launcher.py`, `llm_client.py`). Keep modules and functions `snake_case`, classes `PascalCase`, and YAML keys lower-case. Prefer dataclasses for config bundles, keep functions cohesive, log via `logging`, and store reusable prompts or scenario data in `templates.py` or scenario YAMLs.

### Tool conventions (Python tools)
- Tools must expose a top-level `tool(input_value=None, **kwargs)` function.
- Tools must return via `executor._create_uniform_return(...)` (do not hand-roll return dicts).
- When consuming tool results, treat `result["data"]` as the raw value; `result["value"]` is display/truncation.

## Testing Guidelines
Pytest ships with the requirements. Add suites beside the code (e.g., `utils/test_<feature>.py`) and run them with `pytest -q path/to/test.py`, reserving `-s` when console output matters. Mock LLM or Zenoh calls via `utils.llm_api.LLM` fakes, keep fixtures in `data/tmp/`, and let GUI-driven tests skip gracefully when assets are absent.

## Commit & Pull Request Guidelines
History uses short, imperative subjects (`Update README for clarity on LLM and planning`, `Added a task library...`); keep summaries <72 characters and prefix subsystems when helpful (`infospace: improve planner fallback`). PRs should describe the scenario exercised, note new CLI flags or configs, link issues, list manual steps (e.g., clear `data/vector/*`), and call out incompatible map or topic schema updates.

## Security & Configuration Tips
`utils/llm_api` expects API keys from the environment—never commit `.env` or tokens. Treat `setup.sh` as reference, keep `data/`, `logs/`, and generated RAG stores out of Git, and store non-local Zenoh endpoints in deployment YAML rather than code.
