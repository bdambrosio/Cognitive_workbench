# Adapter Creation Guide

## Overview

There are two ways to create a musing adapter:

### Quick Start: CLI Tool (Recommended)

Use the CLI tool to create initial adapter files:

```bash
python src/musing/create_adapter.py --model qwen/Qwen3-4B-Instruct --output-dir ./adapters
```

This will:
1. Inspect the model to get dimensions
2. Generate initial random weights
3. Create PEFT files ready to load into SGLang

See `create_adapter.py --help` for all options.

### Programmatic Creation

For programmatic creation (e.g., during runtime):
1. Initializing the `SGLangMusingAdapter` with SGLang Runtime
2. Getting site shapes (requires model dimensions)
3. Running policy G to generate weights
4. Applying weights at a segment boundary

## Step-by-Step Workflow

### 1. Initialize SGLangMusingAdapter

```python
from musing.musing.sglang_adapter import SGLangMusingAdapter
from musing.musing.config import AdapterConfig
import sglang as sgl

# Initialize SGLang Runtime (already done in executive_node.py)
runtime = sgl.Runtime(
    model_path="path/to/qwen3-4b-instruct",
    # ... other args
)

# Create adapter configuration
adapter_config = AdapterConfig(
    rank=4,  # LoRA rank (conservative for 4B model)
    target_layers=None,  # Will use middle third by default
    magnitude=AdapterMagnitude(max_alpha=16.0)
)

# Initialize adapter manager
adapter = SGLangMusingAdapter(
    runtime=runtime,
    config=adapter_config,
    lora_output_dir="/tmp/musing_adapters",  # Where PEFT files are written
    adapter_name="musing_active",  # Name for the adapter in SGLang
)
```

### 2. Get Site Shapes

**Current Limitation**: `get_site_shapes()` requires model dimensions which aren't easily accessible from SGLang Runtime.

**Option A: Hardcode for Qwen3-4B-Instruct** (Recommended for now)

```python
# Qwen3-4B-Instruct dimensions (36 layers, hidden_size=2048)
NUM_LAYERS = 36
HIDDEN_SIZE = 2048
MLP_INTERMEDIATE = 5504  # Typical ~2.7x hidden_size

# Build site shapes manually
from musing.musing_types import AdapterSiteShape

target_layers = adapter.config.resolve_layers(NUM_LAYERS)  # e.g., [12, 13, ..., 23]
site_shapes = {}

for layer_idx in target_layers:
    # Attention output projection: hidden_size -> hidden_size
    site_shapes[f"layer{layer_idx}.attn_o"] = AdapterSiteShape(
        in_features=HIDDEN_SIZE,
        out_features=HIDDEN_SIZE,
        rank=adapter.config.rank
    )
    
    # MLP up projection: hidden_size -> mlp_intermediate
    site_shapes[f"layer{layer_idx}.mlp_up"] = AdapterSiteShape(
        in_features=HIDDEN_SIZE,
        out_features=MLP_INTERMEDIATE,
        rank=adapter.config.rank
    )
```

**Option B: Load model once to inspect** (One-time cost)

```python
from transformers import AutoModelForCausalLM

# Load model just to inspect dimensions
model = AutoModelForCausalLM.from_pretrained("path/to/qwen3-4b-instruct")
num_layers = len(model.model.layers)
hidden_size = model.config.hidden_size
mlp_intermediate = model.config.intermediate_size

# Build site shapes as above
# ... then delete model to free memory
del model
```

### 3. Run Policy G to Generate Weights

```python
from musing.musing.policy import RandomGaussianPolicy
from musing.musing.observation import build_observation

# Initialize policy
policy = RandomGaussianPolicy(
    sigma=1e-3,  # Small random weights
    alpha=0.5,  # Initial alpha
    enabled=True
)

# Build observation payload
observation = build_observation(
    goal="Your goal here",
    preplan="Your preplan here",
    step_index=0,
    error_count=0,
)

# Add site shapes to payload
observation["site_shapes"] = site_shapes

# Run policy to get weights
policy_output = policy(observation)
# Returns: {"weights": {...}, "alpha": 0.5}
```

### 4. Apply Weights at Segment Boundary

```python
# Must be at segment boundary (not mid-generation)
adapter.end_segment()  # Mark current segment end

# Apply weights (creates PEFT files and loads into SGLang)
adapter.apply_adapter_weights(
    weights=policy_output["weights"],
    alpha=policy_output["alpha"]
)

adapter.start_segment()  # Mark new segment start
```

## Complete Example

```python
import sglang as sgl
from musing.musing.sglang_adapter import SGLangMusingAdapter
from musing.musing.config import AdapterConfig, AdapterMagnitude
from musing.musing_types import AdapterSiteShape
from musing.musing.policy import RandomGaussianPolicy
from musing.musing.observation import build_observation

# 1. Initialize Runtime (assuming already done)
runtime = sgl.Runtime(...)

# 2. Create adapter
config = AdapterConfig(rank=4)
adapter = SGLangMusingAdapter(
    runtime=runtime,
    config=config,
    lora_output_dir="/tmp/musing_adapters",
    adapter_name="musing_active"
)

# 3. Build site shapes (hardcoded for Qwen3-4B-Instruct)
NUM_LAYERS = 36
HIDDEN_SIZE = 2048
MLP_INTERMEDIATE = 5504

target_layers = config.resolve_layers(NUM_LAYERS)
site_shapes = {}
for layer_idx in target_layers:
    site_shapes[f"layer{layer_idx}.attn_o"] = AdapterSiteShape(
        HIDDEN_SIZE, HIDDEN_SIZE, config.rank
    )
    site_shapes[f"layer{layer_idx}.mlp_up"] = AdapterSiteShape(
        HIDDEN_SIZE, MLP_INTERMEDIATE, config.rank
    )

# 4. Initialize policy
policy = RandomGaussianPolicy(sigma=1e-3, alpha=0.5)

# 5. Generate weights
observation = build_observation(goal="...", preplan="...")
observation["site_shapes"] = site_shapes
policy_output = policy(observation)

# 6. Apply at segment boundary
adapter.end_segment()
adapter.apply_adapter_weights(
    weights=policy_output["weights"],
    alpha=policy_output["alpha"]
)
adapter.start_segment()

# Adapter is now loaded and active in SGLang Runtime!
```

## Updating Adapter Weights

To update weights later (e.g., after policy G learns):

```python
# At segment boundary
adapter.end_segment()

# Get new weights from policy
new_policy_output = policy(new_observation)

# Apply new weights (overwrites same adapter file)
adapter.apply_adapter_weights(
    weights=new_policy_output["weights"],
    alpha=new_policy_output["alpha"]
)

adapter.start_segment()
```

## Decaying Alpha

To decay alpha over time:

```python
# At segment boundary
adapter.end_segment()

# Decay alpha (e.g., multiply by 0.9)
new_alpha = adapter.decay_alpha(factor=0.9)

adapter.start_segment()
```

## Disabling Musing

To disable musing (zero weights):

```python
adapter.end_segment()
adapter.zero_adapters()
adapter.start_segment()
```

## Key Points

1. **Segment Boundaries**: All weight updates must happen at segment boundaries (between `end_segment()` and `start_segment()`)

2. **File Management**: Adapter files are written to `lora_output_dir` and automatically loaded into SGLang Runtime

3. **Adapter Name**: The `adapter_name` is used as the identifier in SGLang Runtime. You can have multiple adapters with different names.

4. **Model Dimensions**: Currently need to hardcode or inspect model to get dimensions for site shapes. This is a known limitation.

5. **Policy Output Format**: Policy G must return `{"weights": {...}, "alpha": float}` where weights dict maps site keys to `(A_tensor, B_tensor)` tuples.

