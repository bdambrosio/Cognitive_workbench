---
name: generate-image
description: Generate an original image from a text description, locally (Bonsai-Image 4B, ternary-quantized, on a long-lived studio server). Use when the user wants a picture, illustration, avatar, or face created from a description that does not already exist on the web. For existing photos of real things, prefer image search instead; for simple diagrams or line drawings, prefer authoring inline SVG.
args:
  prompt: required string — what to depict, e.g. "a friendly cartoon robot face, soft smile, flat vector style". State qualities you want directly; the model follows prompts literally.
  steps: optional int (default 4) — denoising steps; Bonsai is distilled for 4, more buys little
  size: optional int (default 512) — square side length in pixels. 512 is the fast preset; 1024 is the quality preset (slower).
  seed: optional int — fix for reproducible output; omit for variety
---

# generate-image

Produces an original image from a prompt using Bonsai-Image 4B (ternary
1.58-bit) running locally on the GPU via the prism-image-studio backend. The
backend is a long-lived server; the tool is a thin HTTP client that POSTs the
prompt and saves the returned PNG.

The tool talks to the FastAPI backend directly: `POST {base}/generate`, with a
`{base}/healthz` readiness probe. Base URL defaults to `http://localhost:4001`
(override with `GENERATE_IMAGE_URL`). If the backend isn't already running, the
tool launches it via the Bonsai-Image-Demo `serve.sh` and waits for `/healthz`
(a cold boot prewarms the model and can take a couple of minutes). To start it
by hand:

```
BACKEND_PORT=4001 FRONTEND_PORT=3100 ./scripts/serve.sh
```

`4001` is the **backend** port here. The tool bypasses the Next.js studio
frontend (`/api/generate`), so its port is irrelevant and its prompt-moderation
gate is skipped.

Best for invented/illustrative imagery — characters, faces, scenes, styled
graphics. It is a *generator*, not a search: it cannot reproduce a specific
real photograph or a named existing image. There is no separate negative
prompt; describe exactly what you want (including what to leave out) in the
`prompt`.

## Examples

```json
{"thought": "the user wants a friendly assistant face drawn", "tool": "generate-image", "prompt": "a friendly cartoon robot assistant face, large round eyes, gentle smile, flat vector illustration, pastel background, centered"}
```

```json
{"thought": "generate a calm scene with no people or text", "tool": "generate-image", "prompt": "a quiet misty lake at dawn, soft pastel sky, minimalist, no people, no text", "seed": 7}
```
