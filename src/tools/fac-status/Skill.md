---
name: fac-status
description: Quick Factorio situational check — your character's position and walking state, server tick, connected players, your last bridge action, and the latest game chat. Use first when turning attention to the factory, and to see whether the human said anything in game chat.
args: {}
---

# fac-status

One-call read of your state in the shared Factorio world, via the game
bridge. Cheap; use it freely before acting.

## Behavior

- Reports your character position, whether it is currently walking, the
  server tick, and who is connected.
- Includes your last bridge action (from the activity log) and the last
  few game-chat lines — check these for requests or corrections from the
  human before starting new work.
- Returns `{status: "error", ...}` if the bridge process is not running.

```json
{"thought": "check the factory situation before acting", "tool": "fac-status"}
```
