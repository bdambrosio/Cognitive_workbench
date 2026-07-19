# Cognitive Workbench

A research workbench for agents that maintain a persistent model of what
matters — *concerns* — and act from it: in conversation, and in the idle
cycle between conversations.

What's distinctive here:

- **Concerns as persistent salience, not a task queue.** Agent concerns
  accumulate activation on a per-concern rhythm and from semantic evidence
  in the conversation; crossing threshold doesn't trigger action directly —
  an LLM triage step judges whether acting *now* is warranted (fire / defer
  / reset), and completed work leaves a running summary the next firing
  reads. User concerns are the mirror image: recall-driven, decaying unless
  the user touches the topic, never firing — they shape responses instead.
  The asymmetry is the design.
- **Reflection is the write path.** The system prompt is reassembled every
  turn from evolving state — persona, self-model, a fair-witness companion
  model of the user, discourse agreements, recalled memories, active
  concerns — and a post-turn reflection pass is the only thing that updates
  that state. Nothing is appended to a transcript and forgotten.
- **Local-first.** Runs end-to-end against a local model (vLLM /
  llama.cpp / SGLang); cloud backends (Anthropic, OpenAI-compatible) are a
  YAML edit away.

*Research code, single-author, breaking changes without notice. The
ChatLoop chat subproject is the live surface; the status of every design
document and legacy layer is tracked in [docs/STATUS.md](docs/STATUS.md).*

## The idle-cycle problem

Most agent frameworks are sophisticated listeners: a request arrives, tools
fire, a response ships, and the agent is inert until the next event. That's
fine for assistants. It fails the moment you want an agent that notices
things on its own, tracks your evolving interests, and acts without being
asked — because a task list can repeat actions, but it can't preserve a
live evaluative pressure that later justifies *different* actions.

The concern system is this project's answer. Concerns are persistent,
revisable structures that keep domains of mattering in play:

- **Agent concerns** are pressure-driven. Activation grows with wall-clock
  time scaled by each concern's `rhythm_hours`, and jumps when conversation
  semantically touches the concern's domain. Past threshold, a triage LLM
  call decides fire / defer / reset — the verdict is cached and aged so a
  deferred concern doesn't re-cost a call every tick. Fires run through the
  same ReAct loop as user turns, and each completed fire rewrites the
  concern's work-in-progress note so consecutive fires don't start cold.
- **User concerns** model what the user cares about. They're born from
  reflection with a short stable handle plus an evolving *context* (the
  evidence, and what the user appears to want — witnessed, tracked, helped,
  researched, discussed). They decay per turn, bump on semantic match,
  age out after two unbumped weeks, and can be closed or updated by
  reflection when an exchange shows resolution or development. A weekly
  seed concern reviews them and judges — through the same triage gate —
  whether any warrants one small proactive act.

The framing is developed in
[docs/what-agents-care-about.md](docs/what-agents-care-about.md) (concerns
vs tasks, where autonomy actually lives) and the research stance in
[BACKGROUND.md](BACKGROUND.md).

## One real turn

Captured verbatim against a local vLLM (Gemma-class model). The user asks
the agent to explain its own firing logic; the agent reads its own source
through the geofenced `inspect` subagent.

> **User:** I've been wondering — when one of your concerns fires, what
> actually decides whether you act or stay quiet? Take a look at your own
> implementation and give me the short version.

```
ORIENTATION  read: asking for a technical explanation of concern activation.
             posture: refer to the access boundary; explain what is known
             from the prompt versus what is inaccessible substrate.

iter 1  thought: ...I need to inspect the codebase to see how this is
        implemented.
        tool: inspect("How is the activation and firing logic for
              agent_concerns implemented?...")
        obs:  OK: (inspect: hit max iterations without responding;
              consider narrowing the query)

iter 2  thought: The previous query was too broad; I'll search for
        'concern' to find the relevant files first.
        tool: inspect("List files containing 'concern'...")
        obs:  OK: ...Firing Logic: _check_and_fire_agent_concerns ...
              Triage: _triage_fire_candidate performs a final judgment
              (fire, defer, or reset) before dispatching...

iter 3  thought: I have the method names; now I need the actual logic...
        tool: inspect("Show the implementation of
              _check_and_fire_agent_concerns and _triage_fire_candidate")
        obs:  OK: ...implemented in chat/concerns.py ... [source follows]

iter 4  tool: respond
```

> **Jill:** It's a two-stage filter.
>
> First, there's a pure arithmetic check in
> `_check_and_fire_agent_concerns`. A concern only becomes a candidate if
> its activation level crosses a specific threshold and it actually has an
> instruction attached to it. If it doesn't hit that number, it stays quiet.
>
> If it does cross the threshold, it hits a second filter:
> `_triage_fire_candidate`. This is where a separate LLM pass looks at the
> concern, the instruction, and any existing work-in-progress to decide if
> acting *right now* is actually warranted. It can return three verdicts:
> **fire** (run the ReAct loop), **defer** (not worth it yet; keep the
> activation high but don't act), **reset** (nothing needed this cycle;
> drop the activation without acting).
>
> If the LLM fails or the JSON is garbled, it defaults to 'fire' so I don't
> accidentally ignore something critical.

Reflection then wrote, without being asked:

```
user_concern: "agent concern firing logic"
  context: User is investigating the specific mechanical triggers and
  filters that determine when Jill acts on a concern versus staying
  quiet... Wants technical transparency and a system-level audit of the
  decision-making process.
```

Note the recovery in iter 1→2 — tool failures come back as observations,
not exceptions, and the agent narrows and retries.

## Anatomy of a turn

A turn runs four phases. Within the turn the prompt is read-only; state is
updated *between* turns.

1. **Orient** — one LLM pass reads active concerns, the companion model,
   recent context, persona, and self-model; emits a posture line and
   per-concern relevance.
2. **ReAct** — the main loop. Each iteration emits one JSON action; the
   dispatcher runs the tool and binds the result to `$stepN`. Exits on
   `respond` (capped at 12 iterations; a cap-out synthesizes a fallback
   reply and, on autonomous fires, spawns a narrowed successor concern).
3. **Respond** — the reply ships.
4. **Reflect** — asynchronous post-turn updates: memories, discourse state,
   companion model, user-concern adds/updates/closes, agent-concern
   extraction. The reply does not wait.

**Push state vs pull state.** Push state is always rendered into the
system prompt: top-K recalled memories from a FAISS-indexed collection, the
companion model, discourse agreements, active concerns (user concerns with
their context lines). The agent does not have to ask; they are present.
Pull state is the `recall` tool — a read-only subagent that navigates the
per-world memory directory (list/read/grep) and synthesizes an answer,
for when the agent needs what was actually said rather than what's
currently summarized. Writes happen only via reflection; there is no
in-loop write tool.

**Tools.** Built-ins (`process_text`, `recall`, `inspect`,
`inspect_external`, `security`, `display`, `respond`) plus a
drop-in registry: every directory under `src/tools/` with a `Skill.md` and
a `tool.py` exposing `react_invoke` is discovered at startup — currently
web search, page fetch, calculator, email check, Obsidian, Semantic
Scholar, stock quotes, company financial statements, image generation,
shell scripts, and others. No core edits to add one.

**Geofenced subagents.** Persona-less ReAct loops scoped to a typed
surface; from the parent's vantage each call is one step, and per-call
traces land in sibling `*_traces/` directories.

| Subagent | Scope | Primitives |
|---|---|---|
| `recall` (`src/chat/remember.py`) | per-world per-agent `memory/` dir | list, read, grep |
| `inspect` / `inspect_external` (`src/chat/code_subagent.py`) | own `src/` or an externally-bound repo | list, read, grep (ripgrep) |
| `security` (`src/chat/security.py`) | LAN, RFC1918 ranges only | nmap discovery/-sV, ss/ip |

```
                       user input                    tick sensor
                           │                             │
                           ▼                             ▼
                      orient pass            grow activations → threshold?
                           │                             │
                           │                      triage (LLM, cached)
                           │                       fire / defer / reset
                           ▼                             │ fire
               ┌─────── ReAct loop ◄─────────────────────┘
               │  thought → tool → observation ($stepN)
               │    built-ins · discovered tools
               │    recall / inspect / security   (subagents)
               │    respond ──► reply
               └──────────────│──────────────────────────
                              ▼
                  reflection (async, post-turn)
                              │
        memories · companion · discourse · user_concerns(+context)
        agent_concerns · threads · concern WIP
```

## Quick start

Python 3.10+ on Linux or macOS.

```
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt          # + requirements-dev.txt for tests/benches
```

System binaries (all degrade gracefully if missing):

| Binary | For | Install |
|---|---|---|
| `ripgrep` | `inspect` subagent | `apt install ripgrep` |
| `nmap` (optional) | `security` subagent | `apt install nmap` |
| Chromium-family (optional) | `--affect` / `--canvas` windows | `apt install chromium-browser` |
| Playwright browsers (optional) | JS-heavy page fetch fallback | `playwright install chromium` |

**Backend:** edit the `llm_config` block in `scenarios/jill-chat.yaml`.
Simplest cloud path: `server: anthropic` with `ANTHROPIC_API_KEY` exported.
Local vLLM / llama.cpp / OpenRouter / xAI variants are shown in the sibling
`jill-chat-*.yaml` files.

```
cd src
python launcher.py jill-chat.yaml --cli
```

Type **`/help`** at the prompt for the slash commands (`/recall`,
`/concerns`, `/status`, `/note`, `/img`, `/set-external-repo`, …) — the
primary way to inspect agent state outside the turn flow.

Key launcher flags (see `python launcher.py --help` for the full set):

| Flag | Purpose |
|---|---|
| `--cli` | interactive terminal chat |
| `--autonomy` | enable autonomous concern firing (off by default) |
| `--resource-browser` | web UI on :3001 — memories, concerns, notes, traces |
| `--affect` / `--canvas` | processing-state and rich-display widget windows |
| `--ui` | FastAPI trace-inspection UI on :3000 |
| `--telegram` | Telegram DM bridge (needs `TELEGRAM_BOT_TOKEN`, `TELEGRAM_ALLOWED_CHAT_IDS`) |

A typical session:

```
python launcher.py jill-chat.yaml --cli --autonomy --resource-browser --affect --canvas
```

## Configuration

A scenario is a YAML file under `scenarios/`. Minimum shape:

```yaml
world_config:
  world_name: my_world          # per-world, per-agent state directories

characters:
  Jill:
    mode: chat
    llm_config:
      server: anthropic         # or local / openrouter / openai / xai
      model: claude-opus-4-7
      api_key: ANTHROPIC_API_KEY   # env var NAME, not the key
    character: |
      [voice and stance]
    self_model: |
      [architectural account: what the agent is and isn't]
    concerns:                   # seed agent_concerns
      - text: "Periodically review what the user has been tracking and
               consider whether any of it warrants proactive help."
        rhythm_hours: 168       # one of {1,2,4,8,12,24,168}
        instruction: |
          [what to do when this fires; omit to make it a non-firing
           source concern]
    sensors:
      - name: tick              # autonomy heartbeat
        schedule: "1m"
```

`server` selects the route: `anthropic` → native Messages API; cloud
OpenAI-compatible endpoints → unified Bearer-auth POST with the key read
from the env var named by `api_key`; local servers POST to `vllm_url`
without auth and accept grammar / chat-template kwargs.
`chat.omitted_tools: [...]` ablates tools per scenario (used by benches);
`external_repo:` pre-binds `inspect_external`.

## Memory substrate

Each character in each world gets a `memory/` directory.

- **Overwritten snapshots (authoritative):** `companion_state_<entity>.txt`,
  `discourse_state_<entity>.txt`.
- **Append-only history:** `conversation.txt`, `reasoning_trace.jsonl`
  (per-iteration ReAct records), `memories.jsonl` (write provenance),
  `autonomy.jsonl` (concern fires + triage verdicts).
- **FAISS-indexed collections:** `memories`, `reasoning_history`,
  `agent_concerns`, `user_concerns`, `agent_threads`. Concerns and threads
  are first-class notes — user concerns carry their evolving context,
  agent concerns their triage cache and work-in-progress — all inspectable
  in the resource browser.

The push-side prompt retrieves via FAISS; the `recall` subagent greps the
plain-text files. All full-file writes go through atomic
write-temp-then-rename (`src/utils/file_utils.py`).

## Project layout

```
src/
  chat/                  the live subproject
    chat_loop.py         orchestration spine: init, turn loop, tick, run()
    concerns.py          concern dynamics, triage, WIP, serialization
    reflection.py        post-turn extraction (memories / concerns / closes)
    memories.py          FAISS-backed remember/recall
    threads.py           activity-thread anchors (centroid embeddings)
    react.py             ReAct loop + built-in tool runners
    tools.py             discovered-tool registry (src/tools/*)
    prompts.py           system-prompt assembly, orientation, history render
    zenoh_io.py          Zenoh session + browser/CLI queryables
    backend.py           LLM client (anthropic / OpenAI-compat / local)
    remember.py · code_subagent.py · security.py   subagents
  tools/                 drop-in ReAct tools (Skill.md + tool.py each)
  sensors/               drop-in sensors (tick, rss-watcher, …)
  affect/  canvas/       widget publishers + WS bridges + HTML
  cli.py · resource_browser.py · telegram_bridge.py · launcher.py
scenarios/               YAML configs
bench/                   eval harnesses (each with its own README)
factorio/                Factorio game-embodiment subproject: headless
                         server + fle-bridge mod + HTTP bridge behind
                         the fac-* tools (see factorio/README.md)
docs/                    design notes — see docs/STATUS.md for what's
                         LIVE vs ASPIRATIONAL vs SUPERSEDED
```

Legacy layers from earlier architectures remain in the tree but are not
imported by `mode: chat`; [docs/STATUS.md](docs/STATUS.md) is the
authoritative map.

## Extending

**Tool:** create `src/tools/<name>/` with a `Skill.md` (frontmatter:
`name`, `description`, `args`) and a `tool.py` exposing
`react_invoke(args, *, character_name, backend, logger) → {status, text}`.
Discovered at startup; no core edits. Prefix observations `OK:` /
`EMPTY:` / `ERROR:` so the agent can route on outcome.

**Sensor:** create `src/sensors/<name>/` with a `SKILL.md` (`name`,
`type`, `schedule`, `disposition`) and a `sensor.py` exposing
`run(context)`; reference it from the scenario's `sensors:` list.

**Subagent:** use `src/chat/remember.py` as the template — a static system
prompt (stable across calls, so the backend's KV cache hits), a small set
of read-only primitives, a `respond` exit, per-call trace files.

**Scenario:** copy `scenarios/jill-chat.yaml`, change `world_name` and the
character block; per-world per-agent directories are created on first run.

**Local-LLM gotchas:** llama.cpp GBNF rule names can't contain
underscores; GBNF `{N,M}` quantifiers are approximate; Qwen-family jinja
templates auto-open `<think>` (pass `--reasoning-format none`);
reasoning-model detection is name-substring-based — override with
`is_reasoning_model:` in the scenario.

## Benchmarks

`bench/` holds the eval harnesses, each with its own README: introspective
fidelity (four-tier operational self-awareness), discourse reflection,
memory recall, recall-subagent prompt A/B, counterfactual self-prediction
(cspred), HLE. Runs land in `bench/runs/` (gitignored).

## Limitations and non-goals

- Single-author research code; breaking changes without notice.
- Memory and reflection quality is LLM-dependent: frontier models extract
  and update reliably; small local models lose recall on subtler
  write-side moves (`bench/discourse_reflect/`,
  `docs/design_note_agreements_rag.md`).
- **No autonomy bench yet** — whether autonomous fires are useful and
  well-timed is currently judged anecdotally from `autonomy.jsonl`, not
  measured. It's the next evaluation gap.
- No multi-agent coordination beyond Zenoh pub/sub primitives.
- Robot integration is **not** in this repo — the robot stack (Pi software,
  sensors, actuators) lives in a separate repository. The Jill-side binding
  to the ChatterBot companion-bot head is mostly a design note
  (`docs/jill-integration.md`); the voice path (mic→turn sensor,
  `src/chat/voice_sensor.py`, launcher `--voice`) is the first piece
  implemented, the rest remains design.

## Reading

Key entrypoints, by need:

**Running the system**

- [docs/getting-started.md](docs/getting-started.md) — install, venv,
  credentials, first run.
- [docs/configuration.md](docs/configuration.md) — scenario YAML
  reference (backends, seed concerns, sensors, launcher flags).
- [docs/commands.md](docs/commands.md) — CLI slash commands.
- [docs/ui-guide.md](docs/ui-guide.md) — the UI surfaces (CLI, resource
  browser, affect, canvas, telegram).

**The concern system**

- [docs/what-agents-care-about.md](docs/what-agents-care-about.md) — the
  motivating essay: concerns vs tasks, the idle-cycle problem.
- [docs/concerns-architecture.md](docs/concerns-architecture.md) — the
  live mechanics: two-layer strength/activation model, triage, WIP,
  yield, fire-outcome capture.
- [docs/harness-roadmap.md](docs/harness-roadmap.md) — the
  measurement-gated improvement loop (M0 done, M1 running).

**The Factorio experiment**

- [factorio/README.md](factorio/README.md) — server, bridge, tools,
  operation (v1 complete, both success criteria passed).
- [docs/game-embodiment-assessment.md](docs/game-embodiment-assessment.md)
  — goals + success criteria;
  [docs/factorio-bridge-architecture.md](docs/factorio-bridge-architecture.md)
  — design rationale.
- [docs/cohabitation-writeup.md](docs/cohabitation-writeup.md) — the
  results narrative (draft).

**Background**

- [BACKGROUND.md](BACKGROUND.md) — research stance: what LLMs already
  know, the Socratic approach, why an information space.
- [docs/STATUS.md](docs/STATUS.md) — every design doc classified against
  the live code (LIVE / ASPIRATIONAL / SUPERSEDED / REFERENCE).
- [docs/substack_sensors_vs_tools.md](docs/substack_sensors_vs_tools.md),
  [docs/self-awareness-benchmarks.md](docs/self-awareness-benchmarks.md).

## License

MIT — see [LICENSE](LICENSE).
