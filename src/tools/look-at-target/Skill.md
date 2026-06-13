---
name: look-at-target
description: Aim the ChatterBot head to find and center a target in view — the user, a person, an object, an animal. Runs a closed visual loop (capture, judge where the target is, nudge pan/tilt, repeat) until the target is centered, or reports it could not find the target after searching. Use when the user says point at me, look at me, turn to face someone, find the cat, center on the person. For a one-off snapshot without re-aiming use camera-capture; for a manual fixed angle use head-move.
args:
  target: required string — what to find and center on, in natural language (e.g. 'me' / 'the person on the couch' / 'a cat'). Describe it as you would to another person.
---

# look-at-target

Closed-loop gaze (Tier 2). Given a natural-language target, the tool drives the
ChatterBot head until the target is centered in the camera, then returns the
final pose and the centered frame (which you can then `display` to the user).

How it works (all internal — one step from your view):
1. Capture the current view and judge whether the target is visible.
2. If not visible, sweep a small set of safe poses, capturing at each, until the
   target appears (or give up after a bounded search).
3. Once visible, repeatedly judge the target's position relative to frame center
   and nudge pan/tilt toward it, bounded per step and clamped to the safe head
   envelope, until centered or an iteration cap is reached.

It is **deliberate and slow** (each cycle is a move, a settle, a capture, and a
visual judgement) and it centers **once** — it is not a continuous tracker. If
the target keeps moving it returns its best effort. The returned frame is
attached to your view so you can confirm what you ended up looking at.

## Examples

```json
{"thought": "the user asked me to point at them", "tool": "look-at-target", "target": "me, the person talking to you"}
```

```json
{"thought": "find and face the cat", "tool": "look-at-target", "target": "a cat"}
```
