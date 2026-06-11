# Command Reference

Commands accepted by the chat CLI (`src/cli.py`). Type anything without a
leading `/` to chat. Prefix a command with `@<agent>` (or `/@<agent>`) to
target a specific character in multi-character scenarios.

## Concerns

| Command | Description |
|---|---|
| `/concerns` | List all concerns with activations |
| `/concerns <owner>` | Filter by owner (e.g. `User`, character name) |
| `/concerns wipe` | Bulk-delete every non-seed concern |
| `/concern close <id>` | Close a concern |
| `/concern reopen <id>` | Reopen a concern |
| `/concern resolve <id>` | Mark a concern satisfied |
| `/concern delete <id>` | Delete a concern |
| `/concern activate <id>` | Reactivate a concern |
| `/concern weight <id> <0-1>` | Set concern weight/activation |
| `/concern revisit <id> <hours>` | Set revisit (rhythm) interval |

## Notes & Recall

| Command | Description |
|---|---|
| `/note <id>` | Show note content (e.g. `/note 3940` or `/note Note_3940`) |
| `/note show <id>` | Same, explicit form |
| `/recall <query>` | Direct memory-subagent query (bypasses the ReAct loop) |

## Images

| Command | Description |
|---|---|
| `/img <path-or-url> [caption]` | Send an image to the agent as a turn |
| `/paste [caption]` | Send the clipboard image to the agent |

## External repo (inspect_external tool geofence)

| Command | Description |
|---|---|
| `/set-external-repo <abs-path>` | Bind the inspect_external tool to a repo |
| `/external-repo` | Show current binding |
| `/clear-external-repo` | Remove the binding |

## System

| Command | Description |
|---|---|
| `/status` | Is the agent ready for new input? |
| `/shutdown` | Save and shutdown |
| `/verbose` | Toggle verbose output |
| `/help` | Show help |

## Navigation

| Command | Description |
|---|---|
| `/char <name>` | Switch active character |
| `/ui` | Open web UI (port 3000) |
| `/resources` | Open resource browser (port 3001) |
