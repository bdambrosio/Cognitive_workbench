---
name: mc-training-capture
type: python
description: "Enable or disable automatic training data capture (partial observation grid + optional ground truth)."
schema_hint:
  enable: "bool - Enable (true) or disable (false) capture (default: true)"
  sample_interval: "int - Capture every N position changes (default: 5)"
  include_ground_truth: "bool - Fetch ground truth from bridge (default: false, slower)"
  minecraft_url: "str - Bridge URL for ground truth (optional)"
examples:
  - '{"type":"mc-training-capture","enable":true,"sample_interval":5}'
  - '{"type":"mc-training-capture","enable":true,"include_ground_truth":true}'
  - '{"type":"mc-training-capture","enable":false}'
---

# Minecraft Training Data Capture Tool

Enable or disable automatic capture of training samples for perception model training.

## Purpose

Captures paired partial-observation inputs and ground truth derived from Minecraft world state:
- **Partial Observation Grid**: Current `local_grid` state (radius 10, 21×21×21 voxels)
- **Ground Truth Grid**: Full observable grid (radius 12, 25×25×25 voxels) - optional, requires bridge endpoint
- **Agent Context**: Pose, movement state, inventory summary

## Input

- `enable`: Boolean - Enable (`true`) or disable (`false`) capture (default: `true`)
- `sample_interval`: Integer - Capture sample every N position changes (default: `5`)
- `include_ground_truth`: Boolean - If `true`, fetch ground truth from bridge `/ground-truth` endpoint (default: `false`, slower)
- `minecraft_url`: String - Bridge URL for ground truth (optional, defaults to `MINECRAFT_URL` env var)

## Output

Returns uniform return format with:
- `status`: `"success"` or `"failed"`
- `value`: Human-readable status message
- `extra.enabled`: Boolean indicating if capture is enabled
- `extra.sample_interval`: Capture interval (if enabled)
- `extra.include_ground_truth`: Whether ground truth is included (if enabled)

## Storage

Training samples are saved to:
- **Disk**: `scenarios/minecraft/training_data/sample_{timestamp_ms}.json.zst` (Zstd compressed)
- **Memory**: Last 10 samples in `executor.world_state["training_samples"]`

## Usage

**Enable capture (partial grid only, faster):**
```json
{"type": "mc-training-capture", "enable": true, "sample_interval": 5}
```

**Enable capture with ground truth (slower, requires bridge endpoint):**
```json
{"type": "mc-training-capture", "enable": true, "include_ground_truth": true, "sample_interval": 10}
```

**Disable capture:**
```json
{"type": "mc-training-capture", "enable": false}
```

## Notes

- Capture is **disabled by default** - must be explicitly enabled
- Samples are captured automatically on position changes (when agent moves)
- Ground truth capture requires bridge `/ground-truth` endpoint (Phase 2)
- Samples are compressed with Zstd if available, otherwise stored as JSON
- Directory is created automatically if it doesn't exist
