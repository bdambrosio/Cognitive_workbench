---
name: mc-status
type: python
description: "Fast heartbeat + sanity check for Minecraft bot. Returns connection status, position, orientation, health, food (hunger), current action, and vertical_state (derived from Y position changes). Use to check position, orientation, or other status values"
---

# Minecraft Status Tool

Fast heartbeat and sanity check for Minecraft bot. Returns connection status, position, orientation, health, food (hunger), current action, and vertical_state.

## Purpose

Quick status check for bot state. Returns both human-readable status text and machine-readable structured data including position, orientation, health, food, and vertical movement state.

## Input

- `value`: Ignored unless `"double_sample"`
  - `"double_sample"`: Take two samples (delay ~0.2s) and compute `vertical_state` immediately

## Output

Returns uniform_return format with:
- `value`: Multi-line status text (human-readable)
- `data`: Structured status dict (machine-readable). Key fields:
  - `position`: `{x, y, z}` (floats)
  - `yaw`: Degrees, normalized to 0–360
  - `pitch`: Degrees
  - `health`: 0–20
  - `food`: 0–20
  - `action`: Dict (may be empty)
  - `vertical_state`: `"STABLE"` | `"FALLING"` | `"UNKNOWN"`
  - `vertical_state_explanation`: String
  - `vertical_state_mode`: `"cached"` | `"double_sample"` | `"auto_double_sample"` | `"single"`
  - `success`: Boolean

## Behavior & Performance

- Fast status check with minimal latency
- Vertical state detection: Uses Y position changes to determine if bot is falling, stable, or unknown
- Double sample mode: When `value="double_sample"`, takes two samples with delay to compute vertical_state immediately

## Guidelines

- Use for periodic health checks and state monitoring
- Use double_sample mode when immediate vertical_state detection is needed
- Vertical_state helps detect falling conditions for safety

## Usage Examples

Standard status check:
```json
{"type":"mc-status","out":"$status"}
```

With double sample for immediate vertical_state:
```json
{"type":"mc-status","value":"double_sample","out":"$status"}
```