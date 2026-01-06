---
name: mc-wait
type: python
description: "Synchronous wait for 1 second using time.sleep. Useful for timing delays or pacing actions"
---

# Minecraft Wait Tool

Synchronous wait for 1 second using time.sleep. Useful for timing delays or pacing actions.

## Purpose

Timing control for pacing actions, waiting for asynchronous operations, and coordinating multi-step sequences.

## Input

- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (wait completion message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `wait_duration_seconds`: Float (currently fixed at `1.0`)

## Behavior & Performance

- Synchronous wait - blocks execution for 1 second
- Fixed duration of 1.0 seconds
- Useful for pacing rapid actions

## Guidelines

- Use to pace rapid action sequences
- Use after asynchronous operations to allow time for completion
- Fixed 1-second duration - cannot be customized
- Blocks execution during wait

## Usage Examples

Wait one second:
```json
{"type":"mc-wait","out":"$wait"}
```
