---
name: mc-place-until-supported
type: python
description: "Create reliable footing by placing blocks until observed support becomes provably solid. Absorbs placement uncertainty and never exposes 'accepted but ineffective' outcomes"
---

# Minecraft Place Until Supported Tool

Creates reliable footing by placing blocks until observed support becomes provably solid. Absorbs placement uncertainty and never exposes 'accepted but ineffective' outcomes.

## Purpose

Reliable footing creation for safe movement and construction. Uses observation-verification loop to ensure blocks are actually placed and provide support, handling asynchronous placement uncertainty.

## Input

- `value`: Item/block name (preferred, optional - auto-selects from inventory if not provided)
- `item`: Item/block name (alternative to value, optional)
- `placement_policy`: One of `{"underfoot", "forward-underfoot", "lateral"}` (default: `"underfoot"`)
- `max_attempts`: Integer maximum placement attempts (default: `3`)
- `verify_delay`: Float seconds to wait before re-observation (default: `0.0`)

## Output

Returns uniform_return format with:
- `value`: Text summary with outcome and attempt count
- `data`: Structured data dict (machine-readable). Key fields:
  - `outcome`: `"SUPPORTED"` | `"NO_PLACEABLE_TARGET"` | `"PLACEMENT_IMPOSSIBLE"` | `"INVENTORY_EXHAUSTED"` | `"SUPPORT_AMBIGUOUS"`
  - `attempts_made`: Integer
  - `max_attempts`: Integer
  - `placement_policy`: String
  - `item`: String

## Behavior & Performance

- Observation-verification loop: Places block, observes, verifies support
- Retries up to max_attempts if support not achieved
- Handles asynchronous placement uncertainty
- Auto-selects item from inventory if not specified

## Guidelines

- Use for creating reliable footing before movement
- `SUPPORTED` outcome indicates solid footing achieved
- `NO_PLACEABLE_TARGET` indicates no valid placement location
- `INVENTORY_EXHAUSTED` indicates no more blocks available
- `SUPPORT_AMBIGUOUS` indicates uncertain support state - may need retry

## Usage Examples

Place until supported with defaults:
```json
{"type":"mc-place-until-supported","value":"dirt","placement_policy":"underfoot","max_attempts":3,"out":"$place"}
```

Place forward-underfoot:
```json
{"type":"mc-place-until-supported","item":"cobblestone","placement_policy":"forward-underfoot","out":"$place"}
```
