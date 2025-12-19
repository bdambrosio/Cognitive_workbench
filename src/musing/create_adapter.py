#!/usr/bin/env python3
"""
CLI tool to create initial musing adapter PEFT files for a given model.

Usage:
    python create_adapter.py --model qwen/Qwen3-4B-Instruct --output-dir ./adapters --rank 4
"""

import argparse
import logging
import sys
from pathlib import Path
from typing import Dict

# Add src directory to path so we can import musing
# Script is in src/musing/, so we need to go up one level to get src/
_script_file = Path(__file__).resolve()
script_dir = _script_file.parent  # src/musing/
src_dir = script_dir.parent  # src/
src_dir_str = str(src_dir)

if src_dir_str not in sys.path:
    sys.path.insert(0, src_dir_str)

# Import torch first (needed by musing modules)
try:
    import torch
except ImportError:
    print("ERROR: torch is required. Install with: pip install torch")
    sys.exit(1)

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

try:
    from transformers import AutoConfig, AutoModelForCausalLM
    HAS_TRANSFORMERS = True
except ImportError:
    HAS_TRANSFORMERS = False
    logger.error("transformers library required. Install with: pip install transformers")

# Now import musing modules
from musing.config import AdapterConfig
from musing.musing_types import AdapterSiteShape
from musing.policy import RandomGaussianPolicy
from musing.lora_file_manager import LoRAFileManager


def inspect_model_dimensions(model_path: str) -> Dict[str, int]:
    """
    Load model config to inspect dimensions.
    
    Args:
        model_path: HuggingFace model path or local path
    
    Returns:
        Dict with num_layers, hidden_size, mlp_intermediate_size
    """
    logger.info(f"Inspecting model dimensions from: {model_path}")
    
    try:
        config = AutoConfig.from_pretrained(model_path, trust_remote_code=True)
    except Exception as e:
        logger.error(f"Failed to load model config: {e}")
        raise
    
    # Extract dimensions
    num_layers = getattr(config, 'num_hidden_layers', None) or getattr(config, 'num_layers', None)
    hidden_size = getattr(config, 'hidden_size', None)
    mlp_intermediate = getattr(config, 'intermediate_size', None)
    
    if num_layers is None:
        raise ValueError(f"Could not determine num_layers from config: {config}")
    if hidden_size is None:
        raise ValueError(f"Could not determine hidden_size from config: {config}")
    if mlp_intermediate is None:
        # Estimate: typically 2.7x hidden_size for Qwen/LLaMA models
        mlp_intermediate = int(hidden_size * 2.7)
        logger.warning(f"intermediate_size not found, estimating as {mlp_intermediate} (2.7x hidden_size)")
    
    dimensions = {
        'num_layers': num_layers,
        'hidden_size': hidden_size,
        'mlp_intermediate': mlp_intermediate,
    }
    
    logger.info(f"Model dimensions: {dimensions}")
    return dimensions


def build_site_shapes(
    config: AdapterConfig,
    num_layers: int,
    hidden_size: int,
    mlp_intermediate: int,
) -> Dict[str, AdapterSiteShape]:
    """
    Build site shapes for all target layers.
    
    Args:
        config: Adapter configuration
        num_layers: Total number of layers
        hidden_size: Model hidden size
        mlp_intermediate: MLP intermediate size
    
    Returns:
        Dict mapping site keys to AdapterSiteShape
    """
    target_layers = config.resolve_layers(num_layers)
    site_shapes = {}
    
    for layer_idx in target_layers:
        # Attention output projection: hidden_size -> hidden_size
        site_shapes[f"layer{layer_idx}.attn_o"] = AdapterSiteShape(
            in_features=hidden_size,
            out_features=hidden_size,
            rank=config.rank
        )
        
        # MLP up projection: hidden_size -> mlp_intermediate
        site_shapes[f"layer{layer_idx}.mlp_up"] = AdapterSiteShape(
            in_features=hidden_size,
            out_features=mlp_intermediate,
            rank=config.rank
        )
    
    logger.info(f"Built site shapes for {len(target_layers)} layers ({len(site_shapes)} sites)")
    return site_shapes


def create_adapter(
    model_path: str,
    output_dir: str,
    adapter_name: str = "musing_adapter",
    rank: int = 4,
    alpha: float = 0.5,
    sigma: float = 1e-3,
    target_layers: list = None,
    seed: int = None,
) -> Path:
    """
    Create initial musing adapter PEFT files.
    
    Args:
        model_path: HuggingFace model path or local path
        output_dir: Directory where adapter files will be written
        adapter_name: Name for the adapter
        rank: LoRA rank
        alpha: Initial alpha value
        sigma: Standard deviation for random weights
        target_layers: List of layer indices (None = middle third)
        seed: Random seed for weight generation
    
    Returns:
        Path to created adapter directory
    """
    # Create adapter config
    adapter_config = AdapterConfig(
        rank=rank,
        #target_layers = [dimensions['num_layers'] - 1],
        target_layers=target_layers,
        alpha=alpha,
    )
    adapter_config.validate()
    
    # Inspect model dimensions
    dimensions = inspect_model_dimensions(model_path)
    
    # Build site shapes
    site_shapes = build_site_shapes(
        config=adapter_config,
        num_layers=dimensions['num_layers'],
        hidden_size=dimensions['hidden_size'],
        mlp_intermediate=dimensions['mlp_intermediate'],
    )
    
    # Generate weights using policy
    logger.info("Generating initial weights using RandomGaussianPolicy")
    policy = RandomGaussianPolicy(sigma=sigma, alpha=alpha, seed=seed)
    observation = {"site_shapes": site_shapes}
    policy_output = policy(observation)
    
    if policy_output is None:
        raise RuntimeError("Policy returned None - check if policy is enabled")
    
    weights = policy_output["weights"]
    final_alpha = policy_output["alpha"]
    
    logger.info(f"Generated weights for {len(weights)} sites, alpha={final_alpha}")
    
    # Create PEFT files
    file_manager = LoRAFileManager(output_dir)
    adapter_dir = file_manager.create_peft_files(
        weights=weights,
        alpha=final_alpha,
        rank=rank,
        adapter_name=adapter_name,
    )
    
    logger.info(f"✅ Adapter created successfully: {adapter_dir}")
    return adapter_dir


def main():
    parser = argparse.ArgumentParser(
        description="Create initial musing adapter PEFT files for a model",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Create adapter for Qwen3-4B-Instruct
  python create_adapter.py --model qwen/Qwen3-4B-Instruct --output-dir ./adapters
  
  # Custom rank and alpha
  python create_adapter.py --model qwen/Qwen3-4B-Instruct --rank 8 --alpha 1.0
  
  # Specific layers only
  python create_adapter.py --model qwen/Qwen3-4B-Instruct --layers 12 13 14 15
        """
    )
    
    parser.add_argument(
        "--model",
        type=str,
        required=True,
        help="HuggingFace model path or local path (e.g., 'qwen/Qwen3-4B-Instruct')"
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default="./adapters",
        help="Directory where adapter files will be written (default: ./adapters)"
    )
    
    parser.add_argument(
        "--adapter-name",
        type=str,
        default="musing_adapter",
        help="Name for the adapter (default: musing_adapter)"
    )
    
    parser.add_argument(
        "--rank",
        type=int,
        default=1,
        help="LoRA rank (default: 4)"
    )
    
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.5,
        help="Initial alpha value (default: 0.5)"
    )
    
    parser.add_argument(
        "--sigma",
        type=float,
        default=1e-3,
        help="Standard deviation for random weight initialization (default: 1e-3)"
    )
    
    parser.add_argument(
        "--layers",
        type=int,
        nargs="+",
        default=None,
        help="Specific layer indices to target (default: middle third)"
    )
    
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for weight generation"
    )
    
    args = parser.parse_args()
    
    if not HAS_TRANSFORMERS:
        logger.error("transformers library required. Install with: pip install transformers")
        sys.exit(1)
    
    try:
        adapter_dir = create_adapter(
            model_path=args.model,
            output_dir=args.output_dir,
            adapter_name=args.adapter_name,
            rank=args.rank,
            alpha=args.alpha,
            sigma=args.sigma,
            target_layers=args.layers,
            seed=args.seed,
        )
        
        print(f"\n✅ Success! Adapter created at: {adapter_dir}")
        
    except Exception as e:
        logger.error(f"Failed to create adapter: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()

