---
name: fac-say
description: Say something in Factorio game chat — visible to everyone on the server. Use to report what you're doing, flag blockers, answer the human, or announce before acting near someone else's work.
args:
  message: required string — what to say
---

# fac-say

Speak in multiplayer chat via the game bridge. Appears in-game as a
blue "Jill: ..." line.

## Typical use

Coordination, not narration: announce intentions near others' work,
report completed assignments or blockers, answer questions seen in
fac-status's chat tail. Keep it brief — it's game chat.

```json
{"thought": "let Bruce know the column is done", "tool": "fac-say", "args": {"message": "smelting column is up — plates are flowing"}}
```
