#!/usr/bin/env python3
"""
Create a deterministic "probe" LoRA PEFT adapter that should *obviously* change output
if the LoRA is being applied.

- Targets ONLY the last layer's self_attn.o_proj
- Rank r = 1
- Deterministic A and B:
    A picks one input hidden dim k
    B injects a large value into one output hidden dim m

Writes:
  <output_dir>/<adapter_name>/
    adapter_config.json
    adapter_model.safetensors

Usage:
  python3 create_probe_peft_lora.py \
    --model Qwen/Qwen3-4B-Instruct-2507 \
    --out ./adapters \
    --name probe_adapter \
    --alpha 50 \
    --bval 50 \
    --k 0 --m 0
"""

import argparse
import json
from pathlib import Path

import torch
from safetensors.torch import save_file
from transformers import AutoConfig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="Qwen/Qwen3-4B-Instruct-2507", help="HF model id or local path")
    ap.add_argument("--out", default="./adapters", help="Parent output dir")
    ap.add_argument("--name", default="probe_adapter", help="Adapter directory name")
    ap.add_argument("--alpha", type=float, default=50.0, help="LoRA alpha (scale)")
    ap.add_argument("--r", type=int, default=1, help="LoRA rank (keep at 1 for probe)")
    ap.add_argument("--dropout", type=float, default=0.0, help="LoRA dropout")
    ap.add_argument("--k", type=int, default=0, help="Input hidden index to select in A")
    ap.add_argument("--m", type=int, default=0, help="Output hidden index to inject in B")
    ap.add_argument("--bval", type=float, default=5.0, help="Injected magnitude in B[m]")
    ap.add_argument("--dtype", default="float32", choices=["float32", "float16", "bfloat16"])
    args = ap.parse_args()

    out_parent = Path(args.out).expanduser().resolve()
    adapter_dir = out_parent / args.name
    adapter_dir.mkdir(parents=True, exist_ok=True)

    # Load config only (fast, no model weights)
    cfg = AutoConfig.from_pretrained(args.model, trust_remote_code=True)
    num_layers = getattr(cfg, "num_hidden_layers", None) or getattr(cfg, "num_layers", None)
    hidden_size = getattr(cfg, "hidden_size", None)

    if num_layers is None or hidden_size is None:
        raise ValueError(f"Could not determine num_layers/hidden_size from config: {cfg}")

    last = num_layers - 1
    r = args.r
    if r != 1:
        print("NOTE: probe works best with r=1; continuing anyway.")

    if not (0 <= args.k < hidden_size):
        raise ValueError(f"k must be in [0, {hidden_size-1}]")
    if not (0 <= args.m < hidden_size):
        raise ValueError(f"m must be in [0, {hidden_size-1}]")

    # Choose tensor dtype
    dtype_map = {
        "float32": torch.float32,
        "float16": torch.float16,
        "bfloat16": torch.bfloat16,
    }
    tdtype = dtype_map[args.dtype]

    # --- Deterministic A and B ---
    # LoRA convention: A: (r, in_features), B: (out_features, r)
    A = torch.zeros((r, hidden_size), dtype=tdtype)
    B = torch.full((hidden_size, r), float(args.bval), dtype=tdtype)


    # Deterministic "pick one channel then inject a strong direction"
    # A[0, k] = 1 ; B[m, 0] = bval
    A[0, args.k] = 1.0
    B[:] = float(args.bval)

    # --- Key names ---
    # You showed keys like: base_model.model.model.layers.N.self_attn.o_proj.lora_A.weight
    # We'll follow that exactly.
    prefix = f"base_model.model.model.layers.{last}.self_attn.o_proj"
    tensors = {
        f"{prefix}.lora_A.weight": A.contiguous(),
        f"{prefix}.lora_B.weight": B.contiguous(),
    }

    # --- PEFT adapter_config.json ---
    # This is a standard PEFT LoRA config. Two fields here are important:
    #   target_modules: ["o_proj"]
    #   layers_to_transform: [last] + layers_pattern: "model.layers"
    #
    # That *should* restrict injection to the last layer. Even if a loader ignores
    # layers_to_transform, only last-layer weights exist, so the probe is still meaningful.
    adapter_config = {
        "base_model_name_or_path": args.model,
        "peft_type": "LORA",
        "task_type": "CAUSAL_LM",
        "inference_mode": True,

        "r": r,
        "lora_alpha": float(args.alpha),
        "lora_dropout": float(args.dropout),
        "bias": "none",
        "fan_in_fan_out": False,

        # Apply LoRA to o_proj modules
        "target_modules": ["o_proj"],

        # Attempt to restrict to last layer only
        "layers_to_transform": [last],
        "layers_pattern": "model.layers",
    }

    # Write files
    (adapter_dir / "adapter_config.json").write_text(json.dumps(adapter_config, indent=2) + "\n")
    save_file(tensors, str(adapter_dir / "adapter_model.safetensors"))

    print("\n✅ Probe adapter written:")
    print(f"  {adapter_dir}")
    print("\nDetails:")
    print(f"  model:       {args.model}")
    print(f"  last layer:  {last}")
    print(f"  hidden_size: {hidden_size}")
    print(f"  r:           {r}")
    print(f"  alpha:       {args.alpha}")
    print(f"  A selects k: {args.k}")
    print(f"  B injects m: {args.m} with bval={args.bval}")
    print("\nNext:")
    print("  Register this adapter in SGLang via lora_paths, and run a generation.")
    print("  With alpha/bval this large in the last layer, you should see obvious degeneration or stylistic shift.\n")


if __name__ == "__main__":
    main()
