"""SDXL-Turbo image generation worker — runs as a subprocess.

Why a subprocess: the CUDA device pin must be set in the environment
*before* torch is imported, and the host ChatLoop process has already
imported torch (sentence-transformers, etc.). A fresh subprocess lets the
caller set CUDA_DEVICE_ORDER=PCI_BUS_ID + CUDA_VISIBLE_DEVICES before this
module's `import torch` runs, so generation lands on the RTX 5060 Ti and
never the PRO 6000 that serves the live vLLM. See reference: dual-Blackwell
index-reversal hazard.

One-shot CLI: python worker.py --prompt "..." --out /path/to.png [--steps N]
     [--size N] [--seed N] [--negative "..."]
  Prints one JSON line on stdout: {"path": ..., "gen_s": ..., "vram_gb": ...}

Serve mode: python worker.py --serve
  Loads the model once, prints {"ready": true}, then reads one JSON request
  per line from stdin ({"prompt","out", optional "steps"/"size"/"seed"/
  "negative"}) and writes one JSON response line per request. Exits on stdin
  EOF (i.e. when the parent process dies). Used for lazy keep-warm so the
  ~20 s torch/diffusers import + model load is paid once per session, not
  per image. All progress/warnings go to stderr; stdout carries only JSON.
"""
import argparse
import json
import sys
import time

import torch
from diffusers import AutoPipelineForText2Image

MODEL_ID = "stabilityai/sdxl-turbo"

# Module-level cache: one pipeline per process. Harmless for the current
# subprocess-per-call mode; makes a future persistent-worker mode trivial.
_PIPE = None


def _get_pipe():
    global _PIPE
    if _PIPE is not None:
        return _PIPE
    name = torch.cuda.get_device_name(0)
    # Hard safety gate: refuse to run anywhere but the 5060 Ti, so a
    # misconfigured pin can never touch the live-vLLM PRO 6000.
    if "5060" not in name:
        raise RuntimeError(
            f"refusing to load: visible cuda:0 is {name!r}, not the RTX 5060 Ti. "
            "Set CUDA_DEVICE_ORDER=PCI_BUS_ID and CUDA_VISIBLE_DEVICES=0.")
    _PIPE = AutoPipelineForText2Image.from_pretrained(
        MODEL_ID, torch_dtype=torch.float16, variant="fp16",
    ).to("cuda")
    return _PIPE


def generate(prompt, out_path, *, steps=4, size=512, seed=None, negative=None):
    pipe = _get_pipe()
    gen = None
    if seed is not None:
        gen = torch.Generator(device="cuda").manual_seed(int(seed))
    torch.cuda.reset_peak_memory_stats()
    t0 = time.time()
    # SDXL-Turbo is a guidance-distilled model: guidance_scale must be 0.0.
    image = pipe(
        prompt=prompt,
        negative_prompt=negative or None,
        num_inference_steps=int(steps),
        guidance_scale=0.0,
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
                              steps=req.get("steps", 4), size=req.get("size", 512),
                              seed=req.get("seed"), negative=req.get("negative"))
        except Exception as e:
            result = {"error": str(e)}
        _emit(result)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--serve", action="store_true", help="persistent keep-warm mode")
    ap.add_argument("--prompt")
    ap.add_argument("--out")
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--size", type=int, default=512)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--negative", default=None)
    args = ap.parse_args(argv)
    if args.serve:
        return serve_loop()
    if not args.prompt or not args.out:
        ap.error("--prompt and --out are required unless --serve")
    result = generate(args.prompt, args.out, steps=args.steps, size=args.size,
                      seed=args.seed, negative=args.negative)
    _emit(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
