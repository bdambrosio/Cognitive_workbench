# Integration Notes: SGLang LoRA Adapter System

## Overview

The musing adapter system has been updated to work with SGLang's file-based LoRA loading API. The new implementation converts policy G outputs (A, B matrices) into PEFT format files and loads them into SGLang Runtime.

## New Components

### `lora_file_manager.py`
- Converts weights dict (`layer{N}.{module}` → `(A, B)` tensors) to PEFT format
- Creates `adapter_config.json` and `adapter_model.safetensors` (or `.bin`)
- Maps musing keys to PEFT keys: `layer0.attn_o` → `base_model.model.model.layers.0.self_attn.o_proj.lora_A.weight`

### `sglang_lora_loader.py`
- Wrapper around `runtime.load_lora_adapter(lora_name, lora_path)`
- Validates paths and files before loading
- Tracks loaded adapters

### `sglang_adapter.py`
- Bridge class that integrates file-based approach with existing policy interface
- Handles segment boundary synchronization
- Manages adapter lifecycle (load, update, decay, zero)

## Integration Points (Changes Needed Elsewhere)

### 1. Executive Node (`src/executive_node.py`)

**Location**: Around line 433 where `sgl.Runtime()` is initialized

**Changes Needed**:
- Pass `runtime` instance to `SGLangMusingAdapter` constructor
- Store adapter instance for use by planner/policy

**Example**:
```python
# After runtime initialization
if HAS_SGLANG and sgl_model_path:
    self.runtime = sgl.Runtime(...)
    
    # Initialize musing adapter
    from musing.musing.sglang_adapter import SGLangMusingAdapter
    from musing.musing.config import AdapterConfig
    
    adapter_config = AdapterConfig(
        rank=4,  # Conservative for 4B model
        target_layers=None,  # Will use middle third
    )
    
    self.musing_adapter = SGLangMusingAdapter(
        runtime=self.runtime,
        config=adapter_config,
        lora_output_dir="/tmp/musing_adapters",  # Configurable
        adapter_name="musing_active",
    )
```

### 2. Policy Integration

**Location**: Wherever policy G is called (likely in planner or executor)

**Changes Needed**:
- Get site shapes from model (requires model access or config)
- Call policy G with observation payload
- Pass policy output to `SGLangMusingAdapter.apply_adapter_weights()`
- Ensure calls happen at segment boundaries

**Example**:
```python
# At segment boundary (after tool call, error, etc.)
adapter.end_segment()  # Mark segment end

# Get policy output
policy_output = policy_g(observation_payload)
if policy_output:
    weights = policy_output["weights"]
    alpha = policy_output["alpha"]
    
    # Apply weights (will create files and load into SGLang)
    adapter.apply_adapter_weights(weights, alpha)

adapter.start_segment()  # Mark new segment start
```

### 3. Model Dimension Discovery

**Issue**: `SGLangMusingAdapter.get_site_shapes()` needs model dimensions but doesn't have direct model access (SGLang Runtime doesn't expose this easily).

**Options**:
1. Pass dimensions via config (requires manual specification)
2. Load model separately just to inspect dimensions (one-time cost)
3. Use default dimensions for Qwen3-4B-Instruct (if known)
4. Policy G gets shapes from elsewhere (e.g., config file)

**Recommended**: Option 3 or 4 - hardcode known dimensions or pass via config.

**Qwen3-4B-Instruct dimensions** (from spec):
- Hidden size: 2048 (typical for 4B models)
- Attention heads: 32 query, 8 key/value (GQA)
- MLP intermediate: ~5504 (typical 2.7x hidden size)

### 4. Segment Boundary Synchronization

**Location**: Planner/executor generation loops

**Changes Needed**:
- Call `adapter.end_segment()` before weight updates
- Call `adapter.start_segment()` after weight updates
- Ensure KV cache is rebuilt when adapter changes (SGLang handles this)

**Current SGLang behavior**: Loading a new LoRA adapter likely invalidates KV cache, so segment boundaries should align naturally.

### 5. Adapter Usage in Generation

**SGLang API**: When using loaded adapters, specify via model name:
- Format: `model:adapter_name` (e.g., `qwen3-4b-instruct:musing_active`)
- Or use default if adapter is loaded and active

**Location**: `sglang_api_server.py` or wherever generation requests are made

**Changes Needed**:
- Pass adapter name in model parameter if needed
- Or rely on SGLang's default behavior if single adapter is active

**Note**: May need to check SGLang documentation for exact usage pattern.

### 6. Configuration

**New Config Options Needed**:
- `lora_output_dir`: Where to write LoRA files (default: `/tmp/musing_adapters`)
- `adapter_name`: Name for the adapter (default: `musing_active`)
- `use_safetensors`: Prefer safetensors format (default: True)

**Location**: Character config or environment variables

## Testing Checklist

- [ ] Verify PEFT file creation (check `adapter_config.json` and `adapter_model.*`)
- [ ] Test loading adapter into SGLang Runtime
- [ ] Verify key naming matches SGLang expectations
- [ ] Test alpha scaling (verify `lora_alpha = alpha * rank`)
- [ ] Test segment boundary synchronization
- [ ] Test adapter updates (rewrite same file)
- [ ] Test zero_adapters() (disable musing)
- [ ] Test decay_alpha() (exponential decay)
- [ ] Verify adapter is used in generation requests

## Dependencies

**Required**:
- `torch` (for tensor operations)
- `sglang` (for Runtime API)

**Optional**:
- `safetensors` (preferred format, falls back to PyTorch .bin if unavailable)

## Error Handling

- File creation failures: Hard fail with error logged
- Loading failures: Hard fail with error logged
- Invalid weights: ValueError raised
- Segment boundary violations: SegmentBoundaryError raised

## Limitations

1. **Model Dimensions**: Cannot automatically discover model dimensions from SGLang Runtime. Need to pass via config or hardcode.

2. **Single Adapter**: Current implementation uses single adapter name that gets rewritten. For multiple adapters, would need to create separate `SGLangMusingAdapter` instances with different names.

3. **No Unload**: Currently no explicit unload mechanism (SGLang may support this, but not implemented here).

4. **File Management**: Old adapter files are overwritten, not cleaned up. Manual cleanup required.

