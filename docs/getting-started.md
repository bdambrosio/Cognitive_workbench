# Getting Started

*(Rewritten 2026-07-19 against the live ChatLoop runtime; the previous
version described the deleted OODA/goal UI.)*

## Prerequisites

- **Python 3.10+**, Linux or macOS
- **Git**
- **NVIDIA GPU** only if running a local LLM backend; not required for
  cloud APIs

## Installation

```bash
git clone https://github.com/bdambrosio/Cognitive_workbench.git
cd Cognitive_workbench
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt          # + requirements-dev.txt for tests/benches
```

> **Note:** `requirements.txt` includes some heavyweight optional
> dependencies (PyTorch, PyQt6, pygame). The system degrades gracefully
> when optional dependencies are missing.

Optional system binaries (all degrade gracefully if missing): `ripgrep`
(inspect subagent), `nmap` (security subagent), a Chromium-family browser
(`--affect` / `--canvas` windows), Playwright browsers (`playwright
install chromium`, JS-heavy page-fetch fallback).

## Environment Variables & Credentials

Set in your shell profile (`.bashrc`, `.zshrc`) or a `.env` file in the
project root.

### LLM Backend (choose one)

The backend is selected by the **per-character `llm_config`** block in the
scenario YAML (see [configuration.md](configuration.md)):

| `server:` | Credentials | Notes |
|-----------|-------------|-------|
| `local` | *(none)* | POSTs to `{vllm_url}/v1/chat/completions` — any OpenAI-compatible server (vLLM, SGLang, llama-server, LM Studio) |
| `anthropic` | env var named by the scenario's `api_key:` field (conventionally `ANTHROPIC_API_KEY`) | Native `/v1/messages` route; `api_key` is required |
| `openrouter` | `OPENROUTER_API_KEY` | Or name an env var via `api_key:` |
| `openai` | `OPENAI_API_KEY` | Or name an env var via `api_key:` |

### Tool-Specific Credentials

| Variable | Tool | Notes |
|----------|------|-------|
| `GMAIL_ADDRESS` | check-email, send-email | Your Gmail address |
| `GMAIL_APP_PASSWORD` | check-email, send-email | 16-character app password (requires 2FA; generate at [myaccount.google.com/apppasswords](https://myaccount.google.com/apppasswords)) |
| `GOOGLE_API_KEY` | search-web | Google Custom Search API key |
| `GOOGLE_CX` | search-web | Google Custom Search Engine ID |
| `TAVILY_API_KEY` | tavily | From [tavily.com](https://www.tavily.com/); every call is an `advanced` search (2 credits) because page extraction is unreliable on `basic` |
| `ALPHA_VANTAGE_API_KEY` | stock-price, get-financial-statements | From [alphavantage.co](https://www.alphavantage.co/) |
| `BLUESKY_ACCOUNT_HANDLE` | post-bluesky | Bluesky handle (e.g., `alice.bsky.social`) |
| `BLUESKY_APP_PASSWORD` | post-bluesky | App password from Bluesky Settings > App Passwords |
| `OBSIDIAN_MCP_URL` | obsidian | Obsidian MCP server URL (default: `http://127.0.0.1`) |
| `OBSIDIAN_MCP_API_KEY` | obsidian | Obsidian MCP server API key |
| `TELEGRAM_BOT_TOKEN`, `TELEGRAM_ALLOWED_CHAT_IDS` | `--telegram` bridge | |
| `FACTORIO_URL` | fac-* tools | Bridge URL, default `http://localhost:3004` (see `factorio/README.md`) |

### Debug & System

| Variable | Purpose |
|----------|---------|
| `CWB_DEBUG` | Set to `1` to enable verbose console logging |

## First Run

```bash
source .venv/bin/activate
cd src
python launcher.py jill-chat.yaml --cli
```

That starts the chat loop with the interactive terminal CLI. Type
**`/help`** at the prompt for the slash commands (`/recall`, `/concerns`,
`/status`, `/note`, `/img`, …) — the primary way to inspect agent state
outside the turn flow. See [commands.md](commands.md).

Before first run, edit the `llm_config` block in
`scenarios/jill-chat.yaml` (or start from a sibling `jill-chat-*.yaml`
variant that already targets your backend).

A fuller session:

```bash
python launcher.py jill-chat.yaml --cli --autonomy --resource-browser --affect --canvas
```

- `--autonomy` enables autonomous concern firing (off by default).
- `--resource-browser` serves the web browser for memories, concerns,
  notes, and traces at [http://localhost:3001](http://localhost:3001).
- `--affect` / `--canvas` open the processing-state and rich-display
  widget windows.

Full flag list: `python launcher.py --help` (includes `--voice`,
`--wake`, `--telegram`, `--characters`, `--list-only`, `--ui`,
`--image-server`, `--head-aliveness`, `--browser`, `--debug`).

## Optional Services

### Local LLM server

`server: local` expects an OpenAI-compatible chat endpoint at
`vllm_url` (default `http://127.0.0.1:5000`). Launch vLLM / SGLang /
llama-server yourself, or set the top-level `llm_config.sgl_model_path`
in the scenario to have the launcher bring up a shared SGLang runtime.

### GROBID (PDF parsing)

GROBID extracts structured text from PDFs. With it, `fetch-text` returns
a research paper as a section index you can read section by section;
without it the system falls back to flat PyMuPDF extraction capped at
8000 chars.

```bash
docker run -d --name grobid-server -p 127.0.0.1:8070:8070 grobid/grobid:0.8.2-full
curl -s http://127.0.0.1:8070/api/isalive    # expect: true
```

Bind to `127.0.0.1`, not `0.0.0.0` — GROBID has no authentication, and
publishing it on all interfaces exposes an unauthenticated document
parser to the LAN.

`fetch-text` resolves the endpoint from `GROBID_URL`, defaulting to
`http://localhost:8070/api/processFulltextDocument`. A bare host:port is
accepted (the `/api/processFulltextDocument` path is appended); set
`GROBID_URL=""` to disable GROBID and force the PyMuPDF path. An
unreachable server degrades to the same fallback rather than failing the
fetch.

## Troubleshooting

**Tools fail with "API key not set":**
- Check your environment variables: `echo $OPENROUTER_API_KEY`
- Ensure you sourced your profile after editing it

**Local backend not responding:**
- Verify the server: `curl http://127.0.0.1:5000/v1/models`
- Check `vllm_url` in the scenario matches where your server listens

**No autonomous fires:**
- `--autonomy` must be passed; it is off by default
- `/concerns` in the CLI shows current activations; fires require
  activation ≥ threshold *and* an `instruction` on the concern

Runtime logs land in `logs/` (`character_launcher.log`, plus per-surface
logs); autonomous-fire events append to `autonomy.jsonl` under the
scenario's character directory.

## Next

- [Configuration](configuration.md) — scenario YAML reference
- [commands.md](commands.md) — CLI slash commands
- [concerns-architecture.md](concerns-architecture.md) — the concern system
- [STATUS.md](STATUS.md) — doc index and what's live vs aspirational
