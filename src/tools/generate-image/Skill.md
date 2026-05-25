---
name: generate-image
description: Generate an original image from a text description, locally (SDXL-Turbo). Use when the user wants a picture, illustration, avatar, or face created from a description that does not already exist on the web. For existing photos of real things, prefer image search instead; for simple diagrams or line drawings, prefer authoring inline SVG.
args:
  prompt: required string — what to depict, e.g. "a friendly cartoon robot face, soft smile, flat vector style"
  negative: optional string — qualities to avoid, e.g. "extra limbs, text, watermark, blurry"
  steps: optional int (default 4) — denoising steps; SDXL-Turbo is tuned for 1-4, more buys little
  size: optional int (default 512) — square side length in pixels
  seed: optional int — fix for reproducible output; omit for variety
---

# generate-image

Produces an original image from a prompt using SDXL-Turbo running locally on
the RTX 5060 Ti (no network, no cloud). Returns the path to a saved PNG.

Best for invented/illustrative imagery — characters, faces, scenes, styled
graphics. It is a *generator*, not a search: it cannot reproduce a specific
real photograph or a named existing image. SDXL-Turbo favours speed
(sub-second, 1-4 steps) over fine prompt fidelity, so it can be loose on exact
object counts and small details — lean on `negative` and a clear `prompt` for
the qualities that matter.

## Examples

```json
{"thought": "the user wants a friendly assistant face drawn", "tool": "generate-image", "prompt": "a friendly cartoon robot assistant face, large round eyes, gentle smile, flat vector illustration, pastel background, centered"}
```

```json
{"thought": "generate a calm scene, avoid people and text", "tool": "generate-image", "prompt": "a quiet misty lake at dawn, soft pastel sky, minimalist", "negative": "people, text, watermark", "seed": 7}
```
