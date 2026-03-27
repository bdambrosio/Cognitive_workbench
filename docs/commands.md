# Command Reference

## Tasks

| Command | Description |
|---|---|
| `/tasks` | List all tasks |
| `/task show <name>` | Show task detail |
| `/task add <intention>` | Create a new task |
| `/task propose` | Propose task from conversation context |
| `/task approve <name> [text]` | Approve a proposed task |
| `/task edit <name> <text>` | Edit task intention |
| `/task abandon <name> [reason]` | Abandon a task |
| `/task delete <name>` | Delete task + artifacts |
| `/task interrupt <name>` | Pause active task |
| `/task run <name>` | Run now (clear cooldown) |
| `/task cooldown <name> [secs]` | Set cooldown (default 300s) |

## Goals

| Command | Description |
|---|---|
| `/goals` | List all goals |
| `/goal show <id>` | Show goal detail |
| `/goal add <text>` | Create and run new goal |
| `/goal run <id>` | Execute goal now |
| `/goal terminate <id>` | Stop a running goal |
| `/goal rename <id> <name>` | Rename goal |
| `/goal edit <id> <text>` | Update goal text |
| `/goal remove <id>` | Delete goal |
| `/goal mode <id> <mode> [time]` | Schedule: manual / auto / recurring / daily |
| `/goal exec <id> <mode>` | Execution: replan / replay |
| `/goal cache clear <id>` | Clear cached plan |

## Concerns

| Command | Description |
|---|---|
| `/concerns` | List all concerns |
| `/concerns User` | User concerns only |
| `/concerns <character>` | Derived concerns only |
| `/concern close <id>` | Close a user concern |
| `/concern reopen <id>` | Reopen a user concern |
| `/concern resolve <id>` | Satisfy a derived concern |
| `/concern delete <id>` | Delete a concern |
| `/concern activate <id>` | Reactivate a derived concern |
| `/concern weight <id> <0-1>` | Set concern weight |
| `/concern revisit <id> <hours>` | Set revisit interval |

## System

| Command | Description |
|---|---|
| `/stop` | Halt current execution |
| `/continuous` | Toggle continuous mode |
| `/llm` | Toggle LLM (primary/alt) |
| `/delay <seconds>` | Set turn delay |
| `/scheduler` | Show scheduler status |
| `/scheduler on` | Enable goal scheduler |
| `/scheduler off` | Disable goal scheduler |
| `/scheduler interval <secs>` | Set scheduler check interval |
| `/clear world-model` | Reset world model |
| `/clear map` | Delete map data |
| `/clear transients` | Clear transient resources |
| `/clear persistents` | Clear persistent resources |
| `/save` | Save all data |
| `/shutdown` | Save and shutdown |
| `/bye` | End conversation |
| `/action <json>` | Execute direct JSON action |

## Notes

| Command | Description |
|---|---|
| `/note <id>` | Show note content (e.g. `/note 3940` or `/note Note_3940`) |

## Info

| Command | Description |
|---|---|
| `/status` | System status |
| `/triage` | Triage pipeline status |
| `/ooda` | Recent OODA events |

## Navigation

| Command | Description |
|---|---|
| `/char <name>` | Switch active character |
| `/ui` | Open web UI |
| `/tasks-ui` | Open task manager |
| `/resources` | Open resource browser |
| `/verbose` | Toggle verbose output |
| `/help` | Show help |

Task names accept short form (e.g. `1` for `_task_wip_1`). Chat: type anything without `/`.
