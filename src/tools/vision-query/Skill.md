---
name: vision-query
type: python
description: "Ask a vision-language model a question about what the Body's camera sees. Captures a fresh RGB frame from the Body over zenoh and returns the VLM's text answer. Use whenever the user asks about the visible scene or you need to inspect the environment."
---

# vision-query

Look through the Body's camera and answer a question. Use this whenever the user asks anything about what can be seen — "what do you see?", "is there a person?", "describe the scene", "is the door open?", etc. — or whenever a task requires visual inspection.

## Input

- `question`: the natural-language question to ask about the current view. Be specific; focused questions return better-grounded answers than "describe everything."
- `image_path` *(optional)*: absolute path to a JPEG on disk (typically under `/tmp/body_vision/`). When provided, analyses that exact image instead of capturing a fresh one. Use this only if the operator has referenced a specific image path in their message.

## Output

Success (`status: "success"`):
- `value`: the VLM's text response.

Failure (`status: "failed"`):
- `reason`: short error string. Common failures:
  - `capture_timeout_5.0s` — the Body didn't respond with an image in time (offline, or OAK-D busy)
  - `capture_failed: <detail>` — the Body reported the capture itself failed
  - `zenoh_init: <detail>` — couldn't reach the Body's zenoh router
  - `image_not_found: <path>` — an `image_path` was passed but the file doesn't exist
  - `vlm_error: <detail>` — the VLM call failed (server down, network, etc.)

## Behavior

- Default path: publishes a capture request to `body/oakd/config`, waits up to 5 s for the reply on `body/oakd/rgb`, extracts the JPEG, runs one VLM call with the question.
- With `image_path`: reads the JPEG from disk and runs the VLM call. No capture.
- Captured frames are cached under `/tmp/body_vision/` so they can be referenced in follow-up questions or for debugging.
- One chat-completions call per invocation; no Note is created — the answer flows back as the value string.

## Environment

- `BODY_ROUTER` — zenoh endpoint for the Body (default `tcp/127.0.0.1:7447`; falls back to `ZENOH_CONNECT`).
- `VISION_CAPTURE_TIMEOUT_S` — override the 5 s RGB reply timeout.

## Related

- `vision_service.py` is the underlying VLM client; the `body_stub` desktop tool also uses it for the operator-facing Vision dock.
