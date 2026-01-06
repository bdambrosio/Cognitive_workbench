---
name: mc-stop
type: python
description: "Stop all movement - cancel embodied motion. Critical for safety, interrupts, reflection pauses"
---

# Minecraft Stop Tool

Stops all movement and cancels embodied motion. Critical safety tool for interrupts and reflection pauses.

## Purpose

Movement cancellation for safety, emergency stops, interrupt handling, and reflection pauses. Immediately halts all ongoing movement operations.

## Input

- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (acknowledgement message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean

## Behavior & Performance

- Immediately stops all movement
- Cancels ongoing movement operations
- Critical for safety and emergency stops

## Guidelines

- Use for emergency stops and safety interrupts
- Use before reflection pauses and replanning
- Use to cancel unintended movement
- Critical safety tool - use liberally when uncertain

## Usage Examples

Stop all movement:
```json
{"type":"mc-stop","out":"$stop"}
```
