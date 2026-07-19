# Configuration

*(Rewritten 2026-07-19 against `scenarios/jill-chat.yaml` and
`src/launcher.py`; the previous version documented the deleted
agent-mode schema.)*

A scenario is a YAML file under `scenarios/`. `scenarios/jill-chat.yaml`
is the reference scenario and is heavily commented — read it alongside
this page. Sibling `jill-chat-*.yaml` files are backend variants of the
same shape.

## Top-level keys

```yaml
world_config:
  world_name: "jill_chat"      # names the persistent world directory
                               # (memories, notes, concerns live under it)

llm_config:                    # OPTIONAL shared local runtime only
  sgl_model_path: null         # set a HF model path to have the launcher
  reasoning_parser: null       #   bring up a shared SGLang runtime;
                               #   null skips SGL bring-up entirely

setting: |                     # scene-setting text included in prompts

characters:
  Jill:
    ...                        # per-character config, below
```

The **per-character** `llm_config` (not the top-level one) selects the
actual chat backend.

## Per-character keys

| Key | Purpose |
|---|---|
| `mode: chat` | Selects the live ChatLoop ReAct runtime. Required — the old agent mode is no longer importable. |
| `llm_config` | Backend selection, below |
| `character` | Persona text (who the agent is) |
| `self_model` | Self-model text (how the agent understands its own machinery) |
| `capabilities` | Capability/grounding notes injected into the prompt (e.g. tool conventions, the Factorio y-axis line) |
| `discourse.enabled` | Discourse-state tracking (on by default per project decision) |
| `orientation.enabled` | Per-turn orientation pass (`src/character_evaluator.py`) |
| `external_repo` | Initial binding for the `inspect_external` tool; the `/set-external-repo` CLI command overrides it (persisted Note wins on conflict) |
| `concerns` | Seed concern list, below |
| `sensors` | Sensor list, below |

### `llm_config` (per character)

```yaml
llm_config:
  server: local          # local | anthropic | openrouter | openai
  model: ""              # model name (required for cloud servers)
  vllm_url: http://127.0.0.1:5000   # server:local — any OpenAI-compatible
                                    # chat endpoint (vLLM, SGLang,
                                    # llama-server, LM Studio)
  api_key: ANTHROPIC_API_KEY  # NAME of the env var holding the key.
                              # Required for server:anthropic; optional
                              # for openrouter/openai (legacy path reads
                              # OPENROUTER_API_KEY / OPENAI_API_KEY)
  is_reasoning_model: false   # override name-substring reasoning detection
```

### `concerns` — seed concerns

Each entry seeds a durable `agent_concern` note (`seed=True`; seeds are
architectural baseline and are never closed). Fields:

```yaml
concerns:
  - text: "Keep iron-plate production healthy in my assigned zone."
    name: iron-plate           # stable identifier
    category: durable          # one_shot | durable | derived
    rhythm_hours: 1            # firing rhythm; snapped to buckets
                               # (1, 2, 4, 8, 12, 24, 168); default 168
    instruction: |             # REQUIRED for the concern to fire:
      ...                      #   what an autonomous fire should do
    domain: factorio           # optional tag, stamped on fire records
                               #   for stratified analysis
    user_model_reviewer: true  # optional: heat-coupling target — hot
                               #   user_concerns pull this fire forward
    wip_reviewer: true         # optional: the WIP escalate-or-retire
                               #   reviewer role
    self_extension: true       # optional: the capability-gap concern
```

Concern dynamics (activation growth, firing, triage, WIP) are documented
in [concerns-architecture.md](concerns-architecture.md). Autonomous
firing also requires the launcher `--autonomy` flag — off by default.

### `sensors`

```yaml
sensors:
  - name: tick                 # directory name under src/sensors/
    schedule: "1m"
  - name: factorio-telemetry
    schedule: "2m"
```

Sensor results arrive as user-like turns via
`cognitive/{character}/sense_data`. See [sensor_spec.md](sensor_spec.md)
(note its status banner) and `src/sensors/*/SKILL.md`.

## Tools

Tools are not configured in the scenario: every directory under
`src/tools/` with a `Skill.md` + `tool.py` is auto-discovered into the
catalog at launch (there is currently no per-scenario tool filter).
Restart to pick up new tools.

## Launcher flags

`python launcher.py <scenario.yaml> [flags]` — full list via `--help`:

| Flag | Purpose |
|---|---|
| `--cli` | interactive terminal chat |
| `--autonomy` | enable autonomous concern firing (off by default) |
| `--resource-browser` | web UI on :3001 — memories, concerns, notes, traces |
| `--affect` / `--affect-size` / `--affect-pos` | processing-state widget window |
| `--canvas` / `--canvas-size` / `--canvas-pos` | rich-display widget window |
| `--voice` / `--wake PHRASE` | ChatterBot mic→turn voice sensor |
| `--telegram` | Telegram DM bridge (`TELEGRAM_BOT_TOKEN`, `TELEGRAM_ALLOWED_CHAT_IDS`) |
| `--ui` / `--ui-port` | legacy FastAPI web UI on :3000 (OODA-era display; of limited use in chat mode) |
| `--image-server` | image HTTP server for canvas/affect assets |
| `--head-aliveness` | ChatterBot head idle-motion binding |
| `--browser CMD` | browser command for widget windows |
| `--characters NAME...` / `--list-only` | select / list characters |
| `--debug` | verbose logging (same as `CWB_DEBUG=1`) |

## Logs

`logs/character_launcher.log` plus per-surface logs under `logs/`;
autonomous-fire events append to the character's `autonomy.jsonl`.
