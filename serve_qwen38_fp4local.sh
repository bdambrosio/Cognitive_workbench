#!/usr/bin/env bash
# Start the local model server on port 5000: NVIDIA's Qwen3.8-Flash-Next
# FP4-Local checkpoint in the local-inference-lab vLLM container, on the
# RTX PRO 6000 only. Jill, the outreach run and the measure/ local configs
# call it. It starts with Docker at boot; restart a stopped server with
# `docker start qwen38-fp4local`.
#
# The container's launcher builds the vllm command from its profile
# qwen38-flash-next. CHECKPOINT=original with MODEL set to NVIDIA's repo gives
# the flags on NVIDIA's model card (--quantization modelopt_mixed
# --load-format instanttensor). Without it, the profile downloads a different
# repack of the same weights. To see the command without starting anything:
#
#     docker run --rm --entrypoint bash <the -e and -v options below> $IMAGE \
#       -c 'cd /opt/lil/runtime; /opt/venv/bin/python -m runtime.launcher --print-config'
#
# Differences from the card's simplified command, each for this machine:
#   - The GPU is chosen by UUID. Docker's device=0 here is the 5060 Ti.
#   - PORT=5000, not 8000: the callers use 5000, and 127.0.0.1:8000 is taken.
#   - The weights are on /data (bind mounts), not in named volumes on /.
#   - restart unless-stopped (2026-10-17): the server starts with Docker at
#     boot and comes back after a crash (one GPU hang on 10-17 left it down).
#     `docker stop` keeps it stopped, across reboots too. The server this
#     replaced (qwen38-flash-next, stopped and kept, with its weights in
#     /data/qwen3.8_flash) is restart no, so the two never both start.
#   - Schema-constrained JSON may not contain free whitespace. Without this, an
#     enumeration call on 2026-10-17 wrote 23,935 spaces after `"lines":` until
#     it reached its token cap. vLLM accepts disable_any_whitespace only with
#     an explicit xgrammar or guidance backend.
set -euo pipefail

IMAGE=ghcr.io/local-inference-lab/vllm:release-10152026
PRO6000=GPU-5df5f419-48e1-de31-9e35-eff7e8e56caa
MODEL=nvidia/Qwen3.8-Flash-Next-FP4-Local

docker run -d --name qwen38-fp4local --init \
  --gpus "\"device=$PRO6000\"" \
  --restart unless-stopped --network host --ipc host --shm-size 32g --stop-timeout 30 \
  --ulimit memlock=-1 --ulimit stack=67108864:67108864 \
  -v /data/qwen3.8_fp4local:/root/.cache/huggingface \
  -v /data/qwen3.8_fp4local_runtime:/cache \
  -e HF_HUB_OFFLINE=1 \
  -e PROFILE=qwen38-flash-next -e HARDWARE=rtx-pro-6000-pcie -e PORT=5000 \
  -e CHECKPOINT=original -e MODEL="$MODEL" -e SERVED_MODEL_NAME="$MODEL" \
  "$IMAGE" \
  --structured-outputs-config '{"backend":"xgrammar","disable_any_whitespace":true}'
