---
name: mc-say
type: python
description: "Send chat message in Minecraft - social/debugging/alignment. Returns acknowledgement only"
---

# Minecraft Say Tool

Sends chat message in Minecraft. Social, debugging, and alignment tool for communication.

## Purpose

Chat communication for social interaction, debugging output, and alignment with human observers. Sends text messages visible in Minecraft chat.

## Input

- `value`: Message text (preferred)
- `message`: Message text (alternative to value)

## Output

Returns uniform_return format with:
- `value`: Text summary (acknowledgement message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean

## Behavior & Performance

- Sends message to Minecraft chat
- Message visible to all players and observers
- Returns immediately after sending

## Guidelines

- Use for debugging and status reporting
- Use for social interaction with other players
- Use for alignment and transparency with human observers
- Messages are public and visible to all

## Usage Examples

Send chat message:
```json
{"type":"mc-say","value":"Hello!","out":"$ack"}
```

Send debug message:
```json
{"type":"mc-say","message":"Debug message","out":"$ack"}
```
