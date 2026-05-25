"""FLUX.2 klein-4B image generation worker — runs as a subprocess.

Why a subprocess: the CUDA device pin must be set in the environment
*before* torch is imported, and the host ChatLoop process has already
imported torch (sentence-transformers, etc.). A fresh subprocess lets the
caller set CUDA_DEVICE_ORDER=PCI_BUS_ID + CUDA_VISIBLE_DEVICES before this
module's `import torch` runs, so generation lands on the RTX PRO 6000 (which
has ~45 GB free alongside the FP8 Gemma vLLM pool). See reference:
dual-Blackwell index-reversal hazard.

FLUX.2 klein-4B is a guidance-distilled 4-step model (Apache-2.0, ~13 GB
bf16, Qwen3 text encoder). It does not use negative prompts (guidance is
distilled in), so there is no `negative` arg.

One-shot CLI: python worker.py --prompt "..." --out /path/to.png [--steps N]
     [--size N] [--seed N]
  Prints one JSON line on stdout: {"path": ..., "gen_s": ..., "vram_gb": ...}

Serve mode: python worker.py --serve
  Loads the model once, prints {"ready": true}, then reads one JSON request
  per line from stdin ({"prompt","out", optional "steps"/"size"/"seed"}) and
  writes one JSON response line per request. Exits on stdin EOF (parent
  death). Used for lazy keep-warm so the import + model load is paid once per
  session. All progress/warnings go to stderr; stdout carries only JSON.
"""
import argparse
import json
import sys
import time

import torch
from diffusers import Flux2KleinPipeline

MODEL_ID = "black-forest-labs/FLUX.2-klein-4B"

# Module-level cache: one pipeline per process. Harmless for the current
# subprocess-per-call mode; makes the keep-warm serve mode trivial.
_PIPE = None


def _get_pipe():
    global _PIPE
    if _PIPE is not None:
        return _PIPE
    name = torch.cuda.get_device_name(0)
    # Hard safety gate: refuse to run anywhere but the PRO 6000. The image
    # model is meant to be co-resident with the FP8 Gemma vLLM in that card's
    # free headroom; a misconfigured pin landing on the 16 GB 5060 Ti (which
    # can't hold klein-4B) should fail loudly, not OOM.
    if "PRO 6000" not in name:
        raise RuntimeError(
            f"refusing to load: visible cuda:0 is {name!r}, not the RTX PRO 6000. "
            "Set CUDA_DEVICE_ORDER=PCI_BUS_ID and CUDA_VISIBLE_DEVICES=1.")
    # ~13 GB bf16 fits fully on-card in the PRO 6000's free headroom, so load
    # straight to CUDA (faster than CPU offload, which trades VRAM for latency
    # we don't need here).
    _PIPE = Flux2KleinPipeline.from_pretrained(
        MODEL_ID, torch_dtype=torch.bfloat16,
    ).to("cuda")
    return _PIPE


def generate(prompt, out_path, *, steps=4, size=1024, seed=None):
    pipe = _get_pipe()
    gen = None
    if seed is not None:
        gen = torch.Generator(device="cuda").manual_seed(int(seed))
    torch.cuda.reset_peak_memory_stats()
    t0 = time.time()
    # klein-4B is guidance-distilled: 4 steps, guidance_scale 1.0, no negative.
    image = pipe(
        prompt=prompt,
        num_inference_steps=int(steps),
        guidance_scale=1.0,
        height=int(size), width=int(size),
        generator=gen,
    ).images[0]
    gen_s = time.time() - t0
    image.save(out_path)
    return {"path": out_path, "gen_s": round(gen_s, 2),
            "vram_gb": round(torch.cuda.max_memory_allocated() / 1e9, 1)}


def _emit(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def serve_loop():
    # Load once up front; surface any load failure (e.g. wrong GPU) as a
    # structured line rather than a silent crash, then exit non-zero.
    try:
        _get_pipe()
    except Exception as e:
        _emit({"error": f"load failed: {e}"})
        return 1
    _emit({"ready": True})
    # readline (not `for line in stdin`) to avoid the read-ahead buffering
    # that would stall this request/response protocol.
    while True:
        line = sys.stdin.readline()
        if not line:          # EOF — parent closed stdin / exited
            return 0
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            result = generate(req["prompt"], req["out"],
                              steps=req.get("steps", 4), size=req.get("size", 1024),
                              seed=req.get("seed"))
        except Exception as e:
            result = {"error": str(e)}
        _emit(result)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--serve", action="store_true", help="persistent keep-warm mode")
    ap.add_argument("--prompt")
    ap.add_argument("--out")
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--size", type=int, default=1024)
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args(argv)
    if args.serve:
        return serve_loop()
    if not args.prompt or not args.out:
        ap.error("--prompt and --out are required unless --serve")
    result = generate(args.prompt, args.out, steps=args.steps, size=args.size,
                      seed=args.seed)
    _emit(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
