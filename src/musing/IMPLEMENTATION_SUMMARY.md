# Implementation Summary: SGLang LoRA Adapter System

## Files Created

### 1. `musing/lora_file_manager.py`
**Purpose**: Converts musing adapter weights to PEFT format files for SGLang.

**Key Features**:
- Converts weights dict (`layer{N}.{module}` → `(A, B)` tensors) to PEFT state dict
- Maps musing keys to PEFT keys:
  - `layer0.attn_o` → `base_model.model.model.layers.0.self_attn.o_proj.lora_A.weight`
  - `layer0.mlp_up` → `base_model.model.model.layers.0.mlp.up_proj.lora_A.weight`
- Creates `adapter_config.json` with `lora_alpha`, `r`, `target_modules`
- Writes `adapter_model.safetensors` (preferred) or `adapter_model.bin`
- Handles alpha conversion: `lora_alpha = alpha * rank` (so SGLang's `α/r` = `alpha`)

**Main Class**: `LoRAFileManager`
- `create_peft_files(weights, alpha, rank, adapter_name)` → Creates PEFT directory

### 2. `musing/sglang_lora_loader.py`
**Purpose**: Wrapper around SGLang Runtime's `load_lora_adapter()` API.

**Key Features**:
- Validates adapter directory and required files exist
- Converts paths to absolute (SGLang requirement)
- Calls `runtime.load_lora_adapter(lora_name, lora_path)`
- Tracks loaded adapters
- Hard fails on errors with logging

**Main Class**: `SGLangLoRALoader`
- `load_adapter(lora_name, lora_path)` → Loads adapter into Runtime
- `is_loaded(lora_name)` → Check if adapter is loaded
- `get_loaded_adapters()` → Get set of loaded adapter names

### 3. `musing/sglang_adapter.py`
**Purpose**: Bridge class integrating file-based LoRA with existing policy interface.

**Key Features**:
- Maintains same interface as `Qwen3AdapterScaffold` (for compatibility)
- Manages segment boundary synchronization
- Converts policy outputs → PEFT files → SGLang Runtime
- Handles adapter lifecycle: load, update, decay, zero
- Rewrites same adapter file on updates (as requested)

**Main Class**: `SGLangMusingAdapter`
- `apply_adapter_weights(weights, alpha)` → Create files and load adapter
- `set_alpha(alpha)` → Update alpha (rewrites files)
- `decay_alpha(factor)` → Exponential decay
- `zero_adapters()` → Disable musing (zero weights)
- `start_segment()` / `end_segment()` → Segment boundary management

### 4. `musing/INTEGRATION_NOTES.md`
**Purpose**: Documentation of integration points and required changes elsewhere.

**Contents**:
- Overview of new components
- Integration points in `executive_node.py`
- Policy integration workflow
- Model dimension discovery (limitation)
- Segment boundary synchronization
- Configuration options
- Testing checklist

## Files Modified

### `musing/__init__.py`
- Added exports for `LoRAFileManager`, `SGLangLoRALoader`, `SGLangMusingAdapter`

## Design Decisions

1. **File Rewriting**: Same adapter file is rewritten on each update (not creating new files)
2. **Error Handling**: Hard fail with logging (no recovery)
3. **Format**: Prefers safetensors, falls back to PyTorch .bin
4. **Adapter Naming**: Single adapter name (`musing_active`) that gets rewritten
5. **Alpha Conversion**: `lora_alpha = alpha * rank` so SGLang's scaling `α/r` equals `alpha`

## Key Mapping Logic

**Musing Format** → **PEFT Format**:
- `layer0.attn_o` → `base_model.model.model.layers.0.self_attn.o_proj.lora_A.weight` / `lora_B.weight`
- `layer0.mlp_up` → `base_model.model.model.layers.0.mlp.up_proj.lora_A.weight` / `lora_B.weight`

**Module Mapping**:
- `attn_o` → `self_attn.o_proj`
- `mlp_up` → `mlp.up_proj`

## Integration Points (Not Yet Implemented)

See `INTEGRATION_NOTES.md` for details. Main points:

1. **Executive Node**: Initialize `SGLangMusingAdapter` with Runtime instance
2. **Policy Integration**: Call `apply_adapter_weights()` at segment boundaries
3. **Model Dimensions**: Need to pass dimensions or hardcode for Qwen3-4B-Instruct
4. **Segment Boundaries**: Ensure `end_segment()` / `start_segment()` calls align with weight updates

## Testing Status

- ✅ Code written and linted
- ⏳ Integration with executive node (pending)
- ⏳ Policy integration (pending)
- ⏳ End-to-end testing (pending)

## Dependencies

- `torch` (required)
- `sglang` (required)
- `safetensors` (optional, preferred)

## Limitations

1. **Model Dimensions**: Cannot auto-discover from SGLang Runtime. Need config or hardcode.
2. **Single Adapter**: Uses single adapter name (can create multiple instances for multiple adapters)
3. **No Unload**: No explicit unload mechanism (may not be needed)
4. **File Cleanup**: Old files overwritten, manual cleanup required

## Next Steps

1. Integrate `SGLangMusingAdapter` into `executive_node.py`
2. Wire up policy G calls to adapter
3. Test PEFT file creation and loading
4. Verify adapter usage in generation requests
5. Test segment boundary synchronization

