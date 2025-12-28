---
name: mc-observe-blocks
type: python
description: "Exhaustive enumeration of ALL visible non-air blocks within radius R. Visibility = within radius R AND line-of-sight from bot's eye position. Does NOT report items (use mc-observe-items for that)."
schema_hint:
  value: "ignored"
  radius: "optional observation radius (default: 4, max: 6)"
  blocks_radius: "optional blocks observation radius (default: 4, max: 6)"
  entities_radius: "optional entities observation radius (default: 4, max: 12)"
  out: "$variable"
examples:
  - '{"type":"mc-observe-blocks","out":"$obs"}'
  - '{"type":"mc-observe-blocks","radius":4,"out":"$obs"}'
---

Tool: mc-observe-blocks

Purpose:
Summarize nearby visible blocks, geometry, and movement affordances.

Returns:
A structured SUMMARY with:
- pose
- directional distances + nearest block types
- footing and clearance
- observed block types (deduplicated)
- geometry hints (pit, stair, slope)
- affordances (step, jump, descend, sky)
- confidence level

Guarantees:
- SUMMARY always present and ≤ 1024 chars
- Raw exhaustive data stored out-of-band
- conf=low indicates incomplete observation

Planner must rely ONLY on SUMMARY fields.
