# Web UI Guide

The Cognitive Workbench provides two web interfaces: the **main dashboard** (port 3000) for monitoring and controlling agents, and the **Resource Browser** (port 3001) for inspecting Notes and Collections.

## Launching the UI

```bash
cd src
python3 launcher.py ../scenarios/jill-infospace.yaml --ui --resource-browser
```

- `--ui` enables the main dashboard on port 3000 (override with `--ui-port PORT`)
- `--resource-browser` enables the Resource Browser on port 3001

## Main Dashboard (Port 3000)

### Layout

```
┌─────────────────────────────────────────────────────────┐
│  [Text Input]                    [Step] [Auto] [Stop]   │
├───────────────────┬─────────────────────────────────────┤
│  Character Tabs   │  Character: Jill                    │
│  ┌─────────────┐  │  Goal: Find recent papers on...     │
│  │ ● Jill      │  │                                     │
│  │   Bob       │  ├─────────────────────────────────────┤
│  └─────────────┘  │  Action Log                         │
│                   │  [14:22:30] [JILL] search-web ...   │
│  Tabs:            │  [14:22:45] [JILL] create-note ...  │
│  Goals            │  [14:23:01] [JILL] say User: ...    │
│  Situation        │                                     │
│  Schedule         │                                     │
│  Plans            │                                     │
│  Tools            │                                     │
│                   │                                     │
└───────────────────┴─────────────────────────────────────┘
```

### Text Input

The text input field at the top is how you interact with agents:

- **Goals**: Prefix with `goal:` to submit a goal (e.g., `goal: Summarize this PDF`)
- **Chat**: Type without prefix for conversational messages (processed via envisioning)
- **Commands**: `proceed <id>`, `reuse <id>`, `terminate <id>`, `clear-cache <id>` for scheduled goal management

### Execution Controls

| Button | Action |
|--------|--------|
| **Step** | Execute one planner iteration, then pause |
| **Autonomous** | Run continuously until interrupted |
| **Stop** | Pause execution and interrupt the current plan |
| **End** (red/grey) | Close active dialog. Red = dialog active, grey = no active dialog |

**Continuous Mode** (toggle): When enabled, automatically proceeds to the next queued goal after the current one completes.

### Character Sidebar

Click a character tab to view its details. The sidebar has sub-tabs:

#### Goals Tab
Shows the current active goal text.

#### Situation Tab
Displays the character's current world state (for world integrations like Minecraft, this shows position, inventory, nearby objects).

#### Schedule Tab
Lists all scheduled goals with controls:
- **Status badge**: ready / executing / completed / blocked / abandoned
- **Mode dropdown**: manual / auto / recurring / daily
- **Time picker**: appears when mode is "daily" — set the run time (24h format)
- **Buttons**: Proceed, Terminate, Reuse, Clear Cache
- **Cached actions count**: shows how many plan steps are cached for re-use
- **Scheduler status**: disabled / waiting / running (task ID)
- **Scheduler events**: recent scheduler activity log

#### Plans Tab
Shows saved/cached plans that can be re-executed.

#### Tools Tab
Lists saved plan templates (reusable tool definitions).

### Action Log

The main content area shows a scrollable log of all actions with timestamps:

```
[14:22:30] [JILL] search-web "transformer architectures" | status: SUCCESS
[14:22:45] [JILL] create-note search_results | Note_15
[14:23:01] [JILL] say User: "I found 8 relevant papers..."
[14:23:15] [JILL] think: "I should organize these by date..."
```

- **Timestamps** show per-action execution time
- **Resource IDs** (e.g., `Note_15`, `Collection_4`) are clickable links that open in the Resource Browser
- **Variable bindings** (e.g., `$results → Note_15`) are shown as links
- Action types are color-coded: dialog (say/ask), reasoning (think), tool execution, errors (red)

## Resource Browser (Port 3001)

The Resource Browser is a separate application for inspecting infospace memory.

### Features

- **Two-panel layout**: Resource list (left) + content viewer (right)
- **Browse**: See all active Notes and Collections with their IDs and names
- **View**: Click a resource to see its full content
- **Search**: Filter resources by name or content
- **Copy**: Copy resource content to clipboard
- **Delete**: Remove resources
- **Raw JSON**: Inspect the underlying JSON representation

### Resource ID Format

- Notes: `Note_15`, `Note_42`, etc.
- Collections: `Collection_4`, `Collection_12`, etc.

## Real-Time Updates (WebSocket)

The UI connects via WebSocket (`ws://localhost:3000/ws`) and receives live updates:

| Message Type | Content |
|--------------|---------|
| `action` | Tool execution, dialog, reasoning steps |
| `goal` | Current goal changes |
| `decided_action` | Planner's next intended action |
| `current_plan` | Full plan state (all planned steps) |
| `world_state_update` | World state changes (for world integrations) |
| `time_update` | Simulation time updates |
| `turn_state_update` | Turn number, active/completed characters, execution mode |

## API Endpoints

The dashboard exposes a REST API for programmatic access:

### Character & State

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/characters` | GET | List active characters |
| `/api/world_state/{character}` | GET | Get character's world state |
| `/api/conversation_status` | GET | Check if character is in dialog |

### Goal Management

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/scheduled_goals/{character}` | GET | List all scheduled goals |
| `/api/goal_schedule_mode/{character}` | POST | Change a goal's schedule mode |
| `/api/goal_rename/{character}` | POST | Rename a goal |
| `/api/goal_cache/{character}` | POST | Manage plan cache |
| `/api/text_input` | POST | Submit text (goal or chat) |

### Execution Control

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/turn/autonomous` | POST | Start autonomous execution |
| `/api/turn/stop` | POST | Stop and interrupt |
| `/api/interrupt` | POST | Interrupt current plan |

### Data Management

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/save` | POST | Save all state to disk |
| `/api/save_and_shutdown` | POST | Save and graceful shutdown |
| `/api/control/clear_transients` | POST | Clear temporary resources |
| `/api/control/clear_persistents` | POST | Clear persistent storage |
| `/api/control/clear_world_model` | POST | Clear world model knowledge |

### Test Goals

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/test_goals` | GET | List available goal YAML files |
| `/api/goal_details/{name}` | GET | Get goal file content |
| `/api/execute_test_goal` | POST | Execute a goal from YAML |

## Keyboard and Browser Tips

- The sidebar is **resizable** — drag the divider. Width is saved to localStorage
- **Ctrl+Enter** submits text input (browser-dependent)
- Action log **auto-scrolls** to the latest entry
- Open Resource Browser in a **separate tab** for side-by-side viewing
- Use browser DevTools (F12) → Network → WS tab to inspect raw WebSocket messages

## Next

- [Getting Started](getting-started.md) — launching the UI for the first time
- [Goals & Scheduling](goals-and-scheduling.md) — using the Schedule tab
- [Architecture](architecture.md) — what the action log is showing you
