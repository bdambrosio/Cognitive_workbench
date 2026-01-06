---
name: mc-respawn
type: python
description: "Reset embodiment after death - lifecycle/recovery. Returns ok"
---

# Minecraft Respawn Tool

Resets embodiment after death. Lifecycle and recovery tool for post-death state restoration.

## Purpose

Post-death recovery for restoring bot state after death. Resets health, position, and other death-related state.

## Input

- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (acknowledgement message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean

## Behavior & Performance

- Resets bot state after death
- Restores health and position
- Returns immediately after respawn

## Guidelines

- Use after detecting death (via `mc-status`)
- Resets embodiment to spawn point or bed location
- Inventory may be lost depending on game mode
- Use `mc-status` after respawn to verify state

## Usage Examples

Respawn after death:
```json
{"type":"mc-respawn","out":"$respawn"}
```
