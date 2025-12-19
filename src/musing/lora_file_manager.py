"""
LoRA File Manager - Converts musing adapter weights to PEFT format for SGLang.

Converts policy G outputs (A, B matrices) into HuggingFace PEFT format files
that SGLang can load via runtime.load_lora_adapter().
"""

import json
import logging
from pathlib import Path
from typing import Dict, Tuple, Optional

import torch

logger = logging.getLogger(__name__)

# Try to import safetensors (preferred format)
try:
    from safetensors.torch import save_file as save_safetensors
    HAS_SAFETENSORS = True
except ImportError:
    HAS_SAFETENSORS = False
    logger.warning("safetensors not available, will use PyTorch .bin format")


class LoRAFileManager:
    """
    Manages conversion of musing adapter weights to PEFT format files.
    
    Converts weights dict with keys like "layer0.attn_o" → PEFT format
    with keys like "base_model.model.model.layers.0.self_attn.o_proj.lora_A.weight"
    """

    def __init__(self, output_dir: str, use_safetensors: bool = True):
        """
        Initialize LoRA file manager.
        
        Args:
            output_dir: Directory where LoRA files will be written
            use_safetensors: Prefer safetensors format if available
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.use_safetensors = use_safetensors and HAS_SAFETENSORS
        
        logger.info(f"LoRAFileManager initialized (output_dir={output_dir}, safetensors={self.use_safetensors})")

    def _parse_site_key(self, site_key: str) -> Tuple[int, str]:
        """
        Parse site key like "layer0.attn_o" into (layer_idx, module_type).
        
        Returns:
            (layer_index, module_type) where module_type is "attn_o" or "mlp_up"
        """
        if not site_key.startswith("layer"):
            raise ValueError(f"Invalid site key format: {site_key}")
        
        parts = site_key.split(".")
        if len(parts) < 2:
            raise ValueError(f"Invalid site key format: {site_key}")
        
        layer_part = parts[0]  # "layer0"
        module_part = parts[1]  # "attn_o" or "mlp_up"
        
        try:
            layer_idx = int(layer_part[5:])  # Extract number after "layer"
        except ValueError:
            raise ValueError(f"Invalid layer index in key: {site_key}")
        
        return layer_idx, module_part

    def _map_module_name(self, module_type: str) -> str:
        """
        Map musing module type to PEFT module name.
        
        Args:
            module_type: "attn_o" or "mlp_up"
        
        Returns:
            PEFT module name: "self_attn.o_proj" or "mlp.up_proj"
        """
        mapping = {
            "attn_o": "self_attn.o_proj",
            "mlp_up": "mlp.up_proj",
        }
        
        if module_type not in mapping:
            raise ValueError(f"Unknown module type: {module_type}")
        
        return mapping[module_type]

    def _build_peft_key(self, layer_idx: int, module_name: str, matrix: str) -> str:
        """
        Build PEFT state dict key for a LoRA matrix.
        
        Args:
            layer_idx: Layer index (0-based)
            module_name: PEFT module name (e.g., "self_attn.o_proj")
            matrix: "A" or "B"
        
        Returns:
            PEFT key like "base_model.model.model.layers.0.self_attn.o_proj.lora_A.weight"
        """
        return f"base_model.model.model.layers.{layer_idx}.{module_name}.lora_{matrix}.weight"

    def create_peft_files(
        self,
        weights: Dict[str, Tuple[torch.Tensor, torch.Tensor]],
        alpha: float,
        rank: int,
        adapter_name: str = "musing_adapter",
    ) -> Path:
        """
        Create PEFT format files from musing adapter weights.
        
        Args:
            weights: Dict mapping site keys (e.g., "layer0.attn_o") to (A, B) tensors
            alpha: Musing scale factor (will be converted to lora_alpha = alpha * rank)
            rank: LoRA rank
            adapter_name: Name for the adapter (used for directory/filename)
        
        Returns:
            Path to the adapter directory containing adapter_config.json and adapter_model.*
        
        Raises:
            ValueError: If weights dict is invalid
            RuntimeError: If file writing fails
        """
        if not weights:
            raise ValueError("weights dict cannot be empty")
        
        # Create adapter directory
        adapter_dir = self.output_dir / adapter_name
        adapter_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"Creating PEFT adapter files in {adapter_dir}")
        
        # Build PEFT state dict
        peft_state_dict = {}
        target_modules_set = set()
        
        for site_key, (a_tensor, b_tensor) in weights.items():
            # Parse site key
            layer_idx, module_type = self._parse_site_key(site_key)
            
            # Map to PEFT module name
            peft_module_name = self._map_module_name(module_type)
            target_modules_set.add(peft_module_name.split(".")[-1])  # e.g., "o_proj" or "up_proj"
            
            # Build PEFT keys
            key_a = self._build_peft_key(layer_idx, peft_module_name, "A")
            key_b = self._build_peft_key(layer_idx, peft_module_name, "B")
            
            # Store tensors
            peft_state_dict[key_a] = a_tensor.clone().detach()
            peft_state_dict[key_b] = b_tensor.clone().detach()
            
            logger.debug(f"Mapped {site_key} → {key_a} and {key_b}")
        
        # Write weights file
        weights_path = adapter_dir / "adapter_model.safetensors" if self.use_safetensors else adapter_dir / "adapter_model.bin"
        
        try:
            if self.use_safetensors:
                save_safetensors(peft_state_dict, str(weights_path))
                logger.info(f"Saved weights to {weights_path} (safetensors format)")
            else:
                torch.save(peft_state_dict, weights_path)
                logger.info(f"Saved weights to {weights_path} (PyTorch format)")
        except Exception as e:
            logger.error(f"Failed to save weights file: {e}")
            raise RuntimeError(f"Failed to save weights file: {e}") from e
        
        # Create adapter_config.json
        # Convert alpha to lora_alpha: SGLang applies alpha/r, so lora_alpha = alpha * r
        lora_alpha = int(alpha * rank)
        
        config = {
            "peft_type": "LORA",
            "r": rank,
            "lora_alpha": lora_alpha,
            "target_modules": sorted(list(target_modules_set)),
            "lora_dropout": 0.0,
            "bias": "none",
            "task_type": "CAUSAL_LM",
        }
        
        config_path = adapter_dir / "adapter_config.json"
        try:
            with open(config_path, "w") as f:
                json.dump(config, f, indent=2)
            logger.info(f"Saved config to {config_path}")
        except Exception as e:
            logger.error(f"Failed to save config file: {e}")
            raise RuntimeError(f"Failed to save config file: {e}") from e
        
        logger.info(f"Created PEFT adapter: {adapter_dir} (rank={rank}, alpha={alpha}, lora_alpha={lora_alpha})")
        return adapter_dir

