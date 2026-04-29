---
name: tick
description: Time heartbeat for chat-mode autonomy — fires every schedule period to trigger concern cadence checks
type: code
schedule: "30m"
parameters: {}
---

Stateless time sensor. On each scheduled tick it publishes a `{kind: 'tick'}`
marker to the agent's `sense_data` channel. The chat loop dispatches on the
marker, runs `_check_and_fire_concerns` against the current wall clock, and
autonomously executes any concern instructions whose cadence has elapsed.

This is the trigger source for chat-mode Phase C autonomy. Wall clock is the
single source of truth — the sensor does not advance any internal clock; it
just removes the user-turn dependency from cadence checks.
