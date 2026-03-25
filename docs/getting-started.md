# Getting Started

## Prerequisites

- **Python 3.10+**
- **Git**
- **NVIDIA GPU** (recommended for SGLang local inference; not required if using cloud LLM APIs)

## Installation

```bash
git clone https://github.com/bdambrosio/Cognitive_workbench.git
cd Cognitive_workbench
python3 -m venv zenoh_venv
source zenoh_venv/bin/activate
pip install -r requirements.txt
```

> **Note:** The full `requirements.txt` includes PyTorch (for Minecraft VoxelAffordanceModel), PyQt6, and pygame. If you only need the core agent, these are safe to skip — the system degrades gracefully when optional dependencies are missing.

## Environment Variables & Credentials

The system reads API keys and service credentials from environment variables. Set them in your shell profile (`.bashrc`, `.zshrc`) or a `.env` file in the project root.

### LLM Backend (choose one or more)

| Variable | Service | Notes |
|----------|---------|-------|
| *(none needed)* | **SGLang** (local) | Model path is set in the scenario YAML; requires NVIDIA GPU |
| `OPENROUTER_API_KEY` | **OpenRouter** (cloud) | Sign up at [openrouter.ai](https://openrouter.ai/) |
| `CLAUDE_API_KEY` | **Anthropic** (cloud) | From [console.anthropic.com](https://console.anthropic.com/) |
| `OPENAI_API_KEY` | **OpenAI** (cloud) | From [platform.openai.com](https://platform.openai.com/) |

### Tool-Specific Credentials

| Variable | Tool | Notes |
|----------|------|-------|
| `GMAIL_ADDRESS` | check-email, send-email | Your Gmail address |
| `GMAIL_APP_PASSWORD` | check-email, send-email | 16-character app password (requires 2FA enabled; generate at [myaccount.google.com/apppasswords](https://myaccount.google.com/apppasswords)) |
| `GOOGLE_API_KEY` | search-web | Google Custom Search API key |
| `GOOGLE_CX` | search-web | Google Custom Search Engine ID |
| `ALPHA_VANTAGE_API_KEY` | stock-price | From [alphavantage.co](https://www.alphavantage.co/) |
| `BLUESKY_ACCOUNT_HANDLE` | post-bluesky | Bluesky handle (e.g., `alice.bsky.social`) |
| `BLUESKY_APP_PASSWORD` | post-bluesky | App password from Bluesky Settings > App Passwords |
| `OBSIDIAN_MCP_URL` | search-obsidian | Obsidian MCP server URL (default: `http://127.0.0.1`) |
| `OBSIDIAN_MCP_API_KEY` | search-obsidian | Obsidian MCP server API key |

### Debug & System

| Variable | Purpose |
|----------|---------|
| `CWB_DEBUG` | Set to `1` to enable verbose console logging |

### Example `.bashrc` snippet

```bash
# Cognitive Workbench - LLM
export OPENROUTER_API_KEY="sk-or-v1-..."
# export CLAUDE_API_KEY="sk-ant-..."
# export OPENAI_API_KEY="sk-..."

# Cognitive Workbench - Tools
export GMAIL_ADDRESS="you@gmail.com"
export GMAIL_APP_PASSWORD="abcd efgh ijkl mnop"
export GOOGLE_API_KEY="AIzaSy..."
export GOOGLE_CX="abc123..."
```

## Choosing an LLM Backend

### SGLang (Local GPU Inference)

Best for: research use, offline work, cost optimization. Requires an NVIDIA GPU.

1. SGLang is installed with `requirements.txt`
2. Set the model path in your scenario YAML:
   ```yaml
   llm_config:
     sgl_model_path: "Qwen/Qwen3-30B-A3B-Instruct-2507"
   ```
3. The launcher starts the SGLang runtime automatically

**Tested models:**
- `Qwen/Qwen3-30B-A3B-Instruct-2507` — good balance of quality and speed
- `Qwen/Qwen3-Coder-30B-A3B-Instruct` — code-heavy tasks
- `Qwen/Qwen3-8B` — smaller, for limited GPU memory

**GPU memory:** 30B models need ~24 GB VRAM; 8B models need ~8 GB.

### OpenRouter (Cloud API)

Best for: quick setup, no GPU required.

1. Get an API key from [openrouter.ai](https://openrouter.ai/)
2. Set the environment variable: `export OPENROUTER_API_KEY="sk-or-v1-..."`
3. Use a scenario YAML that specifies an OpenRouter model:
   ```yaml
   llm_config:
     openrouter_model_path: "deepseek/deepseek-v3.2"
   ```
   Or copy an existing scenario: `cp scenarios/jill-infospace-openrouter.yaml scenarios/my-agent.yaml`

### Anthropic Claude (Cloud API)

1. Get an API key from [console.anthropic.com](https://console.anthropic.com/)
2. Set: `export CLAUDE_API_KEY="sk-ant-..."`
3. Scenario YAML:
   ```yaml
   llm_config:
     anthropic_model_path: "claude-sonnet-4-6"
   ```

## First Run

1. **Activate the virtual environment:**
   ```bash
   source zenoh_venv/bin/activate
   ```

2. **Launch the core agent with the web UI:**
   ```bash
   cd src
   python3 launcher.py ../scenarios/jill-infospace.yaml --ui --resource-browser --task-manager
   ```

   > Edit the scenario YAML first if you need to change the LLM model or backend.

3. **Open the UI** at [http://localhost:3000](http://localhost:3000) (it may auto-open).

   The default view is the **Activation Field** — a D3 force-directed graph showing the agent, its goals, concerns, and resources as interactive nodes. Click any node to inspect it, use the bottom dock bar to chat, add goals, and control execution. You can switch to the text-based **Classic UI** via the dock bar or by navigating to `/classic`.

4. **Submit a goal** using the **+ Goal** button in the dock bar, or type in the chat panel with a `goal:` prefix:
   ```
   goal: Find and summarize recent papers on transformer architectures
   ```

   The `goal:` prefix tells the system to treat this as a goal for the planner, not a chat message. You'll see goal nodes appear on the graph and the OODA pulse animate as the planner works.

5. **Browse resources** at [http://localhost:3001](http://localhost:3001) to inspect Notes and Collections created during execution.

6. **Monitor concerns and tasks** at [http://localhost:3002](http://localhost:3002) to see the concern-to-task triage pipeline.

## Launcher Options

```
python3 launcher.py <scenario.yaml> [OPTIONS]

Options:
  --ui                  Launch the web UI (port 3000)
  --ui-port PORT        Custom UI port (default: 3000)
  --resource-browser    Launch the Resource Browser (port 3001)
  --task-manager        Launch the Task & Concern Manager (port 3002)
  --debug               Enable verbose logging (same as CWB_DEBUG=1)
```

## Available Scenarios

| Scenario YAML | Description |
|---------------|-------------|
| `jill-infospace.yaml` | Core agent — infospace reasoning, web search, semantic scholar (SGLang) |
| `jill-infospace-openrouter.yaml` | Same as above but using OpenRouter API |
| `jill-infospace-anthropic.yaml` | Same as above but using Anthropic Claude |
| `jill-infospace-openai.yaml` | Same as above but using OpenAI API |
| `jill-infospace-vllm.yaml` | Same as above but using vLLM backend |
| `jill-fs.yaml` | File system world — fs-list, fs-read, fs-grep, fs-find tools |
| `jill-fs-openrouter.yaml` | File system world via OpenRouter |
| `jill-minecraft.yaml` | Minecraft integration — navigation, crafting, 3D world |
| `jill-osworld.yaml` | OS/desktop automation world |
| `jill-scienceworld.yaml` | ScienceWorld simulation for science tasks |
| `jack-and-jill.yaml` | Multi-agent scenario (two characters) |

## Optional Services

### GROBID (PDF Parsing)

GROBID extracts structured text from PDFs. Without it, the system falls back to PyMuPDF (simpler extraction).

```bash
docker run -d -p 8070:8070 grobid/grobid:latest
```

Configure in your scenario YAML:
```yaml
llm_config:
  grobid: "http://localhost:8070/"
  pdf_parser: "grobid"   # or "pymupdf" to skip GROBID
```

### Playwright (Web Scraping)

Some tools use Playwright for browser-based scraping. After `pip install`, run:

```bash
playwright install
```

## Troubleshooting

**SGLang fails to start:**
- Check GPU memory: `nvidia-smi`
- Try a smaller model (e.g., `Qwen/Qwen3-8B`)
- Some FP8 quantizations don't work in SGLang yet

**Web UI doesn't load:**
- Verify FastAPI is running: `curl http://localhost:3000/api/characters`
- Check `logs/fastapi_action_display.log`

**Tools fail with "API key not set":**
- Check your environment variables: `echo $OPENROUTER_API_KEY`
- Ensure you sourced your profile after editing it

**Planning produces no output:**
- Check `logs/incremental_planner.log` for LLM connectivity issues
- Test LLM connectivity: `python3 utils/test_OpenAIClient.py`

See also: `logs/` directory for detailed runtime logs (one file per component).

## Next

- [Architecture](architecture.md) — how the system works
- [Goals & Scheduling](goals-and-scheduling.md) — goal handling and scheduling
- [Configuration](configuration.md) — scenario YAML reference
