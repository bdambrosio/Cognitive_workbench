---
name: camera-capture
description: Capture a still photo from the ChatterBot head camera. The captured frame is attached to your own visual input, so you can SEE it and answer questions about what is in view — whether the user is present, whether there is a cat, what the scene looks like. The camera rides the pan/tilt head, so it shows whatever the head is currently aimed at; aim first with head-move if needed. To also show the photo to the user on screen, follow with the display tool (the observation includes a ready <img> URL). Use when the user asks what you see, to take a picture or snapshot, or to check whether something or someone is in view.
args: {}
---

# camera-capture

Requests one `chatter/camera/capture` from the ChatterBot Pi, waits for the
matching `chatter/camera/image` reply (filtered by request_id, since the topic
is broadcast), and saves the JPEG under the canvas server's allowlisted local
root. The frame is attached to your visual input for the rest of this turn, so
you can reason about its contents directly (no separate "describe" step) — read
it like any image you were shown.

The camera is mounted on the moving head — framing depends on where the head
points. If you need a specific view, issue `head-move` first and let it settle,
then capture.

The observation also contains an `<img>` tag and a `/local` URL; pass that to
the `display` tool (`format=html`) when you also want the user to see it on
screen. Only the most recent capture is kept in view.

## Examples

```json
{"thought": "user asked what I can see right now", "tool": "camera-capture"}
```
