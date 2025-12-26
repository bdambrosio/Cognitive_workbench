# Action Semantics and Exclusivity (Minecraft Agent Model)

This document defines how actions behave over time and interact.
It constrains how the agent reasons about "current action" vs "next action".

---

## Core Rules

- Actions have **duration**, not just effects.
- At most **one action is active** at any given time.
- While an action is active:
  - New actions may be rejected, delayed, or ignored.
  - Status may report the current action in progress.

---

## Digging Semantics

- `mc-dig` initiates a digging action.
- Digging may persist briefly after completion is reported.
- If `mc-status` reports a current dig action:
  - The agent should assume the dig is ongoing or recently completed.
  - Issuing another dig may be unsafe.

---

## Stopping Actions

- `mc-stop`:
  - Immediately cancels all movement and digging
  - Is safe to use as an interrupt
  - Returns acknowledgement, not confirmation of world state

---

## Agent Guidance

- If uncertain whether an action has completed:
  - Prefer observation or status over assumption
- Use `mc-stop` only when:
  - Changing plans
  - Interrupting movement
  - Clearing ambiguous action state

---

## Common Failure Mode

Assuming:
> "The dig already finished, so I can act immediately"

This assumption is unsafe unless confirmed by status or observation.
