---
name: factorio-telemetry
description: Watches the shared Factorio world via the game bridge — new game chat from other players, players joining or leaving, watched production flows stopping, and new alert kinds. Publishes only on events; quiet cycles publish nothing.
type: code
schedule: "2m"
parameters:
  watch_items: ["iron-plate"]
---

# factorio-telemetry

Event ingress from the Factorio game bridge (factorio/bridge/, HTTP at
FACTORIO_URL). Polls /chat and /telemetry and reports only threshold
events, so Jill's rhythm is driven by what happens in the world, not
by the poll:

- game chat from anyone but Jill herself (this is how in-game requests
  reach her — no addressing filter, she judges relevance)
- players joining or leaving the server
- a watched item's production flow hitting zero after being live
  (`watch_items` parameter)
- alert kinds not seen before (entity + issue, e.g. a furnace newly
  out of fuel)

First run initializes silently (no history replay). Bridge down is
logged and skipped, not surfaced as an event. State (chat seq, seen
alert kinds, flow/player baselines) persists in
`~/.cache/cognitive/factorio-telemetry/state.json`.
