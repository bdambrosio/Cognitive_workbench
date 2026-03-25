# Web UI Guide

The Cognitive Workbench provides four web-facing components:

| Component | Port | Purpose |
|-----------|------|---------|
| **Activation Field** (modern UI) | 3000 | D3 force-directed graph, chat, goal entry, inspector |
| **Classic UI** | 3000/classic | Text-based action log, sidebar tabs, scheduling |
| **Resource Browser** | 3001 | Browse, edit, and delete Notes and Collections |
| **Task & Concern Manager** | 3002 | Concern activation, task lifecycle, triage status |

Plus an optional **Browser Extension** that feeds visited URLs to the agent as sensor input.

## Launching

```bash
cd src
python3 launcher.py ../scenarios/jill-infospace.yaml --ui --resource-browser --task-manager
```

- `--ui` — main dashboard on port 3000 (override with `--ui-port PORT`)
- `--resource-browser` — Resource Browser on port 3001
- `--task-manager` — Task & Concern Manager on port 3002

All three flags are included in the VS Code launch configurations.

---

## Activation Field — Modern UI (Port 3000)

The default view is a real-time D3 force-directed graph centered on the active agent.

### Graph Canvas

The main area renders an interactive node graph:

- **Agent node** (large, center) — the character (e.g., Jill)
- **Goal nodes** — active goals, sized by activation level
- **Concern nodes** — user and derived concerns
- **Note nodes** — infospace resources created during execution
- **Binding nodes** — current variable bindings (e.g., `$results`)
- **Edges** — connections between agent, goals, and resources

Interactions:
- **Click** a node to open the Inspector panel
- **Scroll** to zoom (0.2x–4x)
- **Drag** to pan the canvas
- Nodes glow brighter with higher activation
- A completion pulse animation plays when a goal finishes

### OODA Pulse Overlay

Expanding colored rings pulse from the agent node on each OODA tick:
- Blue = Observe, Yellow = Orient, Orange = Decide, Green = Act

### Inspector Panel (Right Side)

Slides in when you click a node. Shows contextual detail depending on node type:

- **Agent** — current OODA phase, recent actions
- **Goal** — status, goal text, result summary, output resources
- **Concern** — status, weight, stance, description
- **Note** — resource content
- **Binding** — current variable value

Goal nodes also show action buttons: interrupt, remove, rename, change mode.

### Bottom Dock Bar

A fixed bar along the bottom with:

| Control | Function |
|---------|----------|
| **Chat** | Toggle the chat slide panel |
| **+ Goal** | Toggle the goal entry panel |
| **Status** | Connection/sync indicator |
| **Stop** | Pause execution |
| **Continuous** | Toggle auto-advance between goals |
| **LLM** | Switch between primary and alt LLM |
| **Resources** | Open Resource Browser (port 3001) |
| **Tasks** | Open Task Manager (port 3002) |
| **Classic** | Switch to Classic UI |
| **Save** | Persist all state to disk |
| **Shutdown** | Save and stop the system |

### Chat Panel

Slides up from the bottom when **Chat** is clicked:

- Message history (user / agent / system messages)
- Text input with **Send** button
- **End Conversation** button to close the active dialog
- Unread badge appears on the dock button when the agent sends a message

### Goal Entry Panel

Slides up from the bottom when **+ Goal** is clicked:

- Text area for the goal description
- **Schedule** checkbox — auto-proceed after completion
- **Mode** dropdown — Auto / Manual / Daily
- **Submit** button

---

## Classic UI (Port 3000/classic)

A text-oriented alternative view, accessible via the dock bar or by navigating to `/classic`.

### Layout

```
┌─────────────────────────────────────────────────────────┐
│  [Text Input]                    [Step] [Auto] [Stop]   │
├───────────────────┬─────────────────────────────────────┤
│  Character Tabs   │  Action Log                         │
│  ┌─────────────┐  │  [14:22:30] [JILL] search-web ...  │
│  │ ● Jill      │  │  [14:22:45] [JILL] create-note ... │
│  └─────────────┘  │  [14:23:01] [JILL] say User: ...   │
│                   │                                     │
│  Sidebar Tabs:    ├─────────────────────────────────────┤
│  Plan             │  Controls: LLM toggle, Stop, Save,  │
│  Bindings         │  Resource Browser, Task Manager,     │
│  Goals            │  Obsidian Export, Shutdown            │
│  Plans            │                                     │
│  State            │                                     │
│  Schedule         │                                     │
│  Tasks            │                                     │
└───────────────────┴─────────────────────────────────────┘
```

### Text Input

- **Goals**: Prefix with `goal:` to submit a goal (e.g., `goal: Summarize this PDF`)
- **Chat**: Type without prefix for conversational messages
- **Commands**: `proceed <id>`, `reuse <id>`, `terminate <id>`, `clear-cache <id>` for scheduled goal management

### Sidebar Tabs

| Tab | Content |
|-----|---------|
| **Plan** | Current execution plan steps |
| **Bindings** | Variable values (e.g., `$results → Note_15`) |
| **Goals** | Available test goal YAML files |
| **Plans** | Saved reusable plan templates |
| **State** | World model snapshot |
| **Schedule** | Goal scheduler with auto-proceed toggle, mode, interval |
| **Tasks** | Task WIPs (work in progress) with approve/edit/abandon |

### Action Log

Scrollable history of all actions:

```
[14:22:30] [JILL] search-web "transformer architectures" | status: SUCCESS
[14:22:45] [JILL] create-note search_results | Note_15
[14:23:01] [JILL] say User: "I found 8 relevant papers..."
```

- Resource IDs (e.g., `Note_15`) are clickable links to the Resource Browser
- Action types are color-coded: dialog, reasoning, tool execution, errors (red)

---

## Resource Browser (Port 3001)

A standalone app for inspecting infospace memory.

### Features

- **Two-panel layout**: resource list (left) + content viewer (right)
- **Browse**: all active Notes and Collections with IDs and names
- **View**: click a resource to see full content
- **Edit**: modify note content in-place
- **Delete**: remove resources
- **Search**: filter by name or content

### Resource ID Format

- Notes: `Note_15`, `Note_42`
- Collections: `Collection_4`, `Collection_12`

---

## Task & Concern Manager (Port 3002)

A standalone app for monitoring the concern-to-task pipeline.

### Layout

```
┌──────────────────────┬──────────────────────────────────┐
│  Character Selector  │                                  │
├──────────────────────┤  Tasks Panel                     │
│  Concerns Panel      │  ┌────────────────────────────┐  │
│                      │  │ Task WIPs                   │  │
│  User Concerns       │  │  name, state, approve/edit  │  │
│  ┌────────────────┐  │  ├────────────────────────────┤  │
│  │ concern name   │  │  │ Scheduled Goals             │  │
│  │ status, weight │  │  │  status, controls           │  │
│  │ manage buttons │  │  ├────────────────────────────┤  │
│  └────────────────┘  │  │ Situation Note              │  │
│                      │  │ Triage Status               │  │
│  Derived Concerns    │  └────────────────────────────┘  │
│  (same format)       │                                  │
└──────────────────────┴──────────────────────────────────┘
```

### Concerns Panel (Left)

- **User Concerns** — concerns surfaced from user goals and interactions
- **Derived Concerns** — agent-generated concerns from orientation and reflection
- Each concern shows: name, status badge, weight, activation trend
- Expand to see full description
- **Manage buttons**: close, resolve, abandon, delete

### Tasks Panel (Right)

- **Task WIPs** — in-progress tasks with approve, edit, abandon, run-now controls
- **Scheduled Goals** — active goals with status and scheduling info
- **Situation Note** — current `_situation` resource content
- **Triage Status** — concern triage pipeline statistics

---

## Browser Extension

An optional Chrome extension that monitors page visits and feeds them to the agent.

### What It Does

- Captures URLs, page titles, and timestamps as you browse
- Sends them to a local HTTP listener on port 5004
- Buffers up to 500 visits if the listener is temporarily down
- The `browser-visits` sensor polls this data and delivers it to the agent

### Installation

1. Open Chrome → Extensions → Enable Developer Mode
2. Click "Load unpacked" → select the `browser_extension/` directory
3. The extension runs automatically in the background

The URL listener starts automatically when any character declares a `browser-visits` sensor in its scenario config.

---

## Sensors

Sensors are autonomous data collectors that run on configurable schedules and feed information to the agent.

### Available Sensors

| Sensor | Schedule | What It Does |
|--------|----------|-------------|
| **browser-visits** | 30s | Polls the URL listener for recent page visits |
| **rss-watcher** | 15m | Polls RSS feeds, filters by keywords, deduplicates |

### Configuration

Sensors are declared per-character in the scenario YAML:

```yaml
characters:
  Jill:
    sensors:
      - name: browser-visits
      - name: rss-watcher
        parameters:
          feeds:
            - "https://example.com/feed.xml"
          keywords:
            - "AI"
            - "transformers"
```

---

## Real-Time Updates (WebSocket)

Both UIs connect via WebSocket (`ws://localhost:3000/ws`) for live updates:

| Message Type | Content |
|--------------|---------|
| `action` | Tool execution, dialog, reasoning steps |
| `goal` | Goal state changes |
| `concern` | Concern activation updates |
| `binding` | Variable binding changes |
| `decided_action` | Planner's next intended action |
| `current_plan` | Full plan state |
| `world_state_update` | World state changes |
| `turn_state_update` | Turn number, execution mode |

---

## Tips

- The Classic UI sidebar is **resizable** — drag the divider (width saved to localStorage)
- **Ctrl+Enter** submits text input
- Open Resource Browser and Task Manager in **separate tabs** for side-by-side viewing
- Use browser DevTools (F12) → Network → WS tab to inspect raw WebSocket messages

## Next

- [Getting Started](getting-started.md) — launching the UI for the first time
- [Goals & Scheduling](goals-and-scheduling.md) — using the Schedule tab
- [Architecture](architecture.md) — what the action log is showing you
