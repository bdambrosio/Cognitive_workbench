---
name: generate-image
description: Generate an original image from a text description, locally (FLUX.2 klein). Use when the user wants a picture, illustration, avatar, or face created from a description that does not already exist on the web. For existing photos of real things, prefer image search instead; for simple diagrams or line drawings, prefer authoring inline SVG.
args:
  prompt: required string — what to depict, e.g. "a friendly cartoon robot face, soft smile, flat vector style". State qualities you want directly; the model follows prompts literally.
  steps: optional int (default 4) — denoising steps; klein-4B is distilled for 4, more buys little
  size: optional int (default 1024) — square side length in pixels (FLUX.2 is tuned for 1024)
  seed: optional int — fix for reproducible output; omit for variety
---

# generate-image

Produces an original image from a prompt using FLUX.2 klein-4B running locally
on the RTX PRO 6000 (no network, no cloud). Returns the path to a saved PNG.

Best for invented/illustrative imagery — characters, faces, scenes, styled
graphics. It is a *generator*, not a search: it cannot reproduce a specific
real photograph or a named existing image. FLUX.2 follows prompts literally
and renders legible text well, so describe exactly what you want (including
what to leave out) in the `prompt` itself — there is no separate negative
prompt.

## Examples

```json
{"thought": "the user wants a friendly assistant face drawn", "tool": "generate-image", "prompt": "a friendly cartoon robot assistant face, large round eyes, gentle smile, flat vector illustration, pastel background, centered"}
```

```json
{"thought": "generate a calm scene with no people or text", "tool": "generate-image", "prompt": "a quiet misty lake at dawn, soft pastel sky, minimalist, no people, no text", "seed": 7}
```
