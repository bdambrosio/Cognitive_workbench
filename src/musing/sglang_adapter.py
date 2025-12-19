"""
SGLang Adapter Bridge - Integrates musing policy with SGLang LoRA loading.

This module bridges the existing musing policy interface (which emits weights dicts)
with the file-based LoRA system required by SGLang. It handles:
- Converting policy outputs to PEFT files
- Loading adapters into SGLang Runtime
- Managing adapter lifecycle at segment boundaries
"""

import logging
from pathlib import Path
from typing import Dict, Tuple, Optional

import torch

from .config import AdapterConfig
from .lora_file_manager import LoRAFileManager
from .sglang_lora_loader import SGLangLoRALoader
from .segment import SegmentState, SegmentBoundaryError

logger = logging.getLogger(__name__)


class SGLangMusingAdapter:
    """
    Manages musing adapters for SGLang Runtime using file-based LoRA.
    
    This replaces the in-memory Qwen3AdapterScaffold approach with a file-based
    workflow that works with SGLang's LoRA loading API.
    
    Workflow:
    1. Policy G emits weights dict + alpha
    2. Convert to PEFT files via LoRAFileManager
    3. Load into SGLang Runtime via SGLangLoRALoader
    4. Adapter is active for subsequent generation requests
    """

    def __init__(
        self,
        runtime,
        config: Optional[AdapterConfig] = None,
        segment_state: Optional[SegmentState] = None,
        lora_output_dir: str = "/tmp/musing_adapters",
        adapter_name: str = "musing_active",
    ):
        """
        Initialize SGLang musing adapter.
        
        Args:
            runtime: SGLang Runtime instance
            config: Adapter configuration (rank, target_layers, etc.)
            segment_state: Segment state tracker (for boundary enforcement)
            lora_output_dir: Directory where LoRA files will be written
            adapter_name: Name for the adapter (file will be rewritten on each update)
        """
        self.runtime = runtime
        self.config = config or AdapterConfig()
        self.segment_state = segment_state or SegmentState()
        self.adapter_name = adapter_name
        
        # Initialize file manager and loader
        self.file_manager = LoRAFileManager(lora_output_dir)
        self.loader = SGLangLoRALoader(runtime)
        
        # Track current adapter state
        self.current_alpha = 0.0
        self.current_weights = None
        
        logger.info(f"SGLangMusingAdapter initialized (adapter_name={adapter_name}, output_dir={lora_output_dir})")

    def start_segment(self):
        """Mark start of a generation segment."""
        self.segment_state.start()

    def end_segment(self):
        """Mark end of a generation segment."""
        self.segment_state.end()

    def apply_adapter_weights(
        self,
        weights: Dict[str, Tuple[torch.Tensor, torch.Tensor]],
        alpha: float,
    ) -> None:
        """
        Apply adapter weights from policy G at a segment boundary.
        
        Converts weights to PEFT files and loads into SGLang Runtime.
        Must be called at a segment boundary.
        
        Args:
            weights: Dict mapping site keys (e.g., "layer0.attn_o") to (A, B) tensors
            alpha: Musing scale factor
        
        Raises:
            SegmentBoundaryError: If called mid-segment
            RuntimeError: If file creation or loading fails
        """
        self.segment_state.ensure_boundary("Updating adapter weights")
        
        # Clamp alpha
        clamped_alpha = max(0.0, min(float(alpha), self.config.magnitude.max_alpha))
        
        logger.info(f"Applying adapter weights (alpha={clamped_alpha}, {len(weights)} sites)")
        
        # Create PEFT files
        adapter_dir = self.file_manager.create_peft_files(
            weights=weights,
            alpha=clamped_alpha,
            rank=self.config.rank,
            adapter_name=self.adapter_name,
        )
        
        # Load into SGLang Runtime
        self.loader.load_adapter(
            lora_name=self.adapter_name,
            lora_path=adapter_dir,
        )
        
        # Update state
        self.current_alpha = clamped_alpha
        self.current_weights = weights
        
        logger.info(f"Adapter weights applied and loaded: {self.adapter_name}")

    def set_alpha(self, alpha: float) -> float:
        """
        Update alpha scale factor at a segment boundary.
        
        This requires rewriting the adapter files with new alpha value.
        If no weights are currently loaded, this is a no-op.
        
        Args:
            alpha: New alpha value
        
        Returns:
            Clamped alpha value
        """
        if self.current_weights is None:
            logger.warning("set_alpha called but no weights loaded - no-op")
            return 0.0
        
        self.segment_state.ensure_boundary("Updating alpha")
        
        # Clamp alpha
        clamped_alpha = max(0.0, min(float(alpha), self.config.magnitude.max_alpha))
        
        # Reapply weights with new alpha
        self.apply_adapter_weights(self.current_weights, clamped_alpha)
        return clamped_alpha

    def decay_alpha(self, factor: float) -> float:
        """
        Decay alpha at a segment boundary (exponential decay).
        
        Args:
            factor: Decay factor (0.0 to 1.0)
        
        Returns:
            New alpha value after decay
        """
        if factor < 0:
            raise ValueError("Decay factor must be non-negative")
        
        new_alpha = self.current_alpha * factor
        return self.set_alpha(new_alpha)

    def zero_adapters(self):
        """
        Disable musing by creating zero-weight adapter.
        
        Creates an adapter with zero weights and loads it, effectively
        disabling musing while keeping the adapter loaded.
        """
        self.segment_state.ensure_boundary("Zeroing adapters")
        
        if self.current_weights is None:
            logger.warning("zero_adapters called but no weights loaded - no-op")
            return
        
        # Create zero-weight version
        zero_weights = {}
        for site_key, (a, b) in self.current_weights.items():
            zero_a = torch.zeros_like(a)
            zero_b = torch.zeros_like(b)
            zero_weights[site_key] = (zero_a, zero_b)
        
        # Apply zero weights
        self.apply_adapter_weights(zero_weights, alpha=0.0)
        
        logger.info("Adapters zeroed (musing disabled)")

    def get_site_shapes(self, num_layers: int) -> Dict[str, Tuple[int, int, int]]:
        """
        Get site shapes for policy G initialization.
        
        Returns dict mapping site keys to (rank, in_features, out_features).
        This is used by policy G to emit correctly-sized matrices.
        
        Args:
            num_layers: Total number of layers in the model
        
        Returns:
            Dict mapping site keys to shape tuples
        """
        target_layers = self.config.resolve_layers(num_layers)
        shapes = {}
        
        # We need to know the model dimensions to compute shapes
        # For now, return placeholder - actual implementation would need model access
        # This is a limitation: we can't know exact dimensions without model access
        # Policy G will need to get this information elsewhere or we need to pass it in
        
        logger.warning("get_site_shapes() called but requires model dimensions - returning empty dict")
        logger.warning("Policy G should get site shapes from model inspection or config")
        
        return shapes

