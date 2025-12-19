"""
SGLang LoRA Loader - Interface for loading LoRA adapters into SGLang Runtime.

Provides a clean interface to runtime.load_lora_adapter() with error handling
and adapter management.
"""

import logging
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# Try to import SGLang
try:
    import sglang as sgl
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False
    logger.warning("SGLang not available - LoRA loader will be disabled")


class SGLangLoRALoader:
    """
    Manages loading LoRA adapters into SGLang Runtime.
    
    Wraps runtime.load_lora_adapter() with validation and error handling.
    """

    def __init__(self, runtime):
        """
        Initialize SGLang LoRA loader.
        
        Args:
            runtime: SGLang Runtime instance (from sgl.Runtime())
        
        Raises:
            ValueError: If runtime is None or SGLang not available
        """
        if not HAS_SGLANG:
            raise ValueError("SGLang not available - cannot initialize LoRA loader")
        
        if runtime is None:
            raise ValueError("Runtime cannot be None")
        
        self.runtime = runtime
        self.loaded_adapters = set()  # Track loaded adapter names
        
        logger.info("SGLangLoRALoader initialized")

    def load_adapter(self, lora_name: str, lora_path: Path) -> None:
        """
        Load a LoRA adapter into SGLang Runtime.
        
        Args:
            lora_name: Alias/name for the adapter (used to reference it later)
            lora_path: Path to adapter directory (containing adapter_config.json and adapter_model.*)
        
        Raises:
            ValueError: If path is invalid or files missing
            RuntimeError: If loading fails
        """
        lora_path = Path(lora_path)
        
        # Validate path exists
        if not lora_path.exists():
            error_msg = f"LoRA adapter path does not exist: {lora_path}"
            logger.error(error_msg)
            raise ValueError(error_msg)
        
        if not lora_path.is_dir():
            error_msg = f"LoRA adapter path is not a directory: {lora_path}"
            logger.error(error_msg)
            raise ValueError(error_msg)
        
        # Validate required files exist
        config_path = lora_path / "adapter_config.json"
        weights_path_safetensors = lora_path / "adapter_model.safetensors"
        weights_path_bin = lora_path / "adapter_model.bin"
        
        if not config_path.exists():
            error_msg = f"adapter_config.json not found in {lora_path}"
            logger.error(error_msg)
            raise ValueError(error_msg)
        
        if not weights_path_safetensors.exists() and not weights_path_bin.exists():
            error_msg = f"adapter_model.safetensors or adapter_model.bin not found in {lora_path}"
            logger.error(error_msg)
            raise ValueError(error_msg)
        
        # Convert to absolute path (SGLang requires absolute paths)
        abs_path = lora_path.resolve()
        
        logger.info(f"Loading LoRA adapter: name={lora_name}, path={abs_path}")
        
        # Call SGLang Runtime API
        try:
            self.runtime.load_lora_adapter(
                lora_name=lora_name,
                lora_path=str(abs_path)
            )
            self.loaded_adapters.add(lora_name)
            logger.info(f"Successfully loaded LoRA adapter: {lora_name}")
        except Exception as e:
            error_msg = f"Failed to load LoRA adapter {lora_name} from {abs_path}: {e}"
            logger.error(error_msg)
            raise RuntimeError(error_msg) from e

    def is_loaded(self, lora_name: str) -> bool:
        """
        Check if an adapter is currently loaded.
        
        Args:
            lora_name: Adapter name to check
        
        Returns:
            True if adapter is loaded, False otherwise
        """
        return lora_name in self.loaded_adapters

    def get_loaded_adapters(self) -> set:
        """
        Get set of currently loaded adapter names.
        
        Returns:
            Set of adapter names
        """
        return self.loaded_adapters.copy()

