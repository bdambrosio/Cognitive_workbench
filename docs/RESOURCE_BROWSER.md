# Resource Browser

> **STATUS (2026-07-19): LIVE, but no longer read-only.** Since this doc was
> written the browser gained resource deletion (context menu + `DELETE
> /api/resource/{id}`), inline content editing, concern management
> (`POST /api/concern/{character}/manage`), and a Graph tab. Note the Graph
> tab queries the deleted executive node and is non-functional under the
> live ChatLoop runtime.

Standalone web-based browser for viewing Notes and Collections in a
running agent, reached over Zenoh (queryables served by
`src/chat/zenoh_io.py`).

## Features

- **Read-only view** of all Notes and Collections
- **Raw content display** (text, JSON)
- **Manual refresh** for latest state
- **Dark theme** VS Code-style UI
- **Independent process** - runs alongside other nodes

## Usage

### Start the browser

```bash
cd src
python3 resource_browser.py --map infolab --port 3001
```

### Parameters

- `--map NAME` - Map name to browse (default: infolab)
- `--port PORT` - Web server port (default: 3001)

### Access

Open browser to: `http://localhost:3001`

## Interface

```
┌──────────────────────────────────────────────┐
│ 🔍 Resource Browser - infolab      [Refresh] │
├──────────────┬───────────────────────────────┤
│ NOTES (12)   │ Note_14                [Copy] │
│ □ Note_1     │ ─────────────────────────────│
│ □ Note_2     │ Properties:                   │
│ ☑ Note_14    │   persistent: false           │
│ ...          │ ─────────────────────────────│
│              │ https://papers.neurips.cc/... │
│ COLLECTIONS  │                               │
│ □ Coll_1     │                               │
│ □ Coll_2     │                               │
└──────────────┴───────────────────────────────┘
```

## How It Works

1. Connects to Zenoh network
2. Queries `cognitive/map/resources` for resource list
3. Queries `cognitive/map/resource/{id}` for specific content
4. Displays raw content in monospace font

## Troubleshooting

**No resources showing:**
- Ensure the agent is running
- Check map name matches (default: infolab)
- Click Refresh button

**Connection error:**
- Verify Zenoh router is running
- Check ports not blocked

## Development

Simple FastAPI + Vanilla JS application:
- Backend: FastAPI with Zenoh client
- Frontend: Single-page HTML with embedded CSS/JS
- No build step, no dependencies beyond existing requirements

