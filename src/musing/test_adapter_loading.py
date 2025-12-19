#!/usr/bin/env python3
"""
Test program to load Qwen3-4B-Instruct model and a musing adapter.

Usage:
    python test_adapter_loading.py --model qwen/Qwen3-4B-Instruct --adapter-path ./adapters/musing_adapter
"""

import argparse
import logging
import sys
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Add src directory to path
_script_file = Path(__file__).resolve()
script_dir = _script_file.parent  # src/musing/
src_dir = script_dir.parent  # src/
src_dir_str = str(src_dir)

if src_dir_str not in sys.path:
    sys.path.insert(0, src_dir_str)

try:
    import sglang as sgl
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False
    logger.error("SGLang not available. Install with: pip install sglang")
    sys.exit(1)

try:
    from transformers import AutoTokenizer
    HAS_TRANSFORMERS = True
except ImportError:
    HAS_TRANSFORMERS = False
    logger.error("transformers not available. Install with: pip install transformers")
    sys.exit(1)


def load_model_and_adapter(model_path: str, adapter_path: str, adapter_name: str = "musing_adapter"):
    """
    Load SGLang Runtime with model and LoRA adapter.
    
    Args:
        model_path: HuggingFace model path or local path
        adapter_path: Path to adapter directory (containing adapter_config.json and adapter_model.*)
        adapter_name: Name for the adapter in SGLang Runtime
    """
    logger.info(f"Initializing SGLang Runtime with model: {model_path}")
    
    # Initialize tokenizer
    try:
        tokenizer = AutoTokenizer.from_pretrained(model_path, trust_remote_code=True)
        logger.info("Tokenizer loaded successfully")
    except Exception as e:
        logger.error(f"Failed to load tokenizer: {e}")
        raise
    
    # Initialize SGLang Runtime
    try:
        runtime = sgl.Runtime(
            model_path=model_path,
            tokenizer_path=model_path,
            device="cuda",
            context_length=32768,
            dtype="auto",
            tp_size=1,
            mem_fraction_static=0.25,
            enable_lora=True, 
            # OPTIONAL: Defaults to 8. Controls how many distinct LoRAs can run in a single batch.
            max_loras_per_batch=1,
            lora_paths = {adapter_name: str(Path(adapter_path).resolve())},

            attention_backend="flashinfer"
        )
        sgl.set_default_backend(runtime)
        logger.info(f"SGLang Runtime initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize SGLang Runtime: {e}")
        raise
    
    # Load LoRA adapter
    adapter_path_obj = Path(adapter_path)
    if not adapter_path_obj.exists():
        raise ValueError(f"Adapter path does not exist: {adapter_path}")
    
    if not adapter_path_obj.is_dir():
        raise ValueError(f"Adapter path is not a directory: {adapter_path}")
    
    # Check for required files
    config_file = adapter_path_obj / "adapter_config.json"
    weights_safetensors = adapter_path_obj / "adapter_model.safetensors"
    weights_bin = adapter_path_obj / "adapter_model.bin"
    
    if not config_file.exists():
        raise ValueError(f"adapter_config.json not found in {adapter_path}")
    
    if not weights_safetensors.exists() and not weights_bin.exists():
        raise ValueError(f"adapter_model.safetensors or adapter_model.bin not found in {adapter_path}")
    
    logger.info(f"Loading LoRA adapter: name={adapter_name}, path={adapter_path_obj.resolve()}")
    
    #try:
    #    # CHANGE THIS:
    #    # runtime.load_lora_adapter(...)

    #    # TO THIS:
    #    runtime.load_lora(
    #        lora_name="musing_adapter",
    #        lora_path=str(adapter_path_obj.resolve())
    #    )
    #    logger.info(f"✅ Adapter '{adapter_name}' loaded successfully!")
    #except Exception as e:
    #    logger.error(f"Failed to load adapter: {e}")
    #    raise
            
    return runtime, adapter_name


def test_generation(runtime, adapter_name: str, model_path: str):
    """
    Test generation with the loaded adapter.
    
    Args:
        runtime: SGLang Runtime instance
        adapter_name: Name of the loaded adapter
        model_path: Model path (for model ID)
    """
    logger.info("Testing generation with adapter...")
    
    try:
        from sglang import function, system, user, assistant, gen
        
        @function
        def test_chat(s, prompt):
            #s += sgl.set_lora("probe_adapter") # no such thing, runtime does not support this
            s += system("You are a helpful assistant.")
            s += user(prompt)
            s += assistant(gen("response", max_tokens=500, temperature=0.7))
        
        # Test prompt
        test_prompt = "What is 2+2? Explain your reasoning step by step, reflecting on alternative possibilities."
        
        logger.info(f"Generating response to: '{test_prompt}'")
        state = test_chat.run(prompt=test_prompt)
        response = state["response"].strip()
        
        logger.info(f"✅ Generation successful!")
        logger.info(f"Response: {response}")
        
        return response
        
    except Exception as e:
        logger.error(f"Generation test failed: {e}")
        raise


def main():
    parser = argparse.ArgumentParser(
        description="Test loading Qwen3-4B-Instruct model with musing adapter",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test with default adapter name
  python test_adapter_loading.py --model qwen/Qwen3-4B-Instruct --adapter-path ./adapters/musing_adapter
  
  # Test with custom adapter name
  python test_adapter_loading.py --model qwen/Qwen3-4B-Instruct --adapter-path ./adapters/musing_adapter --adapter-name my_adapter
        """
    )
    
    parser.add_argument(
        "--model",
        type=str,
        default="Qwen/Qwen3-4B-Instruct-2507",
        help="HuggingFace model path or local path (e.g., 'qwen/Qwen3-4B-Instruct')"
    )
    
    parser.add_argument(
        "--adapter-path",
        type=str,
        default="./adapters/musing_adapter",
        help="Path to adapter directory (containing adapter_config.json and adapter_model.*)"
    )
    
    parser.add_argument(
        "--adapter-name",
        type=str,
        default="musing_adapter",
        help="Name for the adapter in SGLang Runtime (default: musing_adapter)"
    )
    
    parser.add_argument(
        "--skip-generation",
        action="store_true",
        default=False,
        help="Skip generation test, just load model and adapter"
    )
    
    args = parser.parse_args()
    
    if not HAS_SGLANG:
        logger.error("SGLang not available")
        sys.exit(1)
    
    if not HAS_TRANSFORMERS:
        logger.error("transformers not available")
        sys.exit(1)
    
    try:
        # Load model and adapter
        runtime, adapter_name = load_model_and_adapter(
            model_path=args.model,
            adapter_path=args.adapter_path,
            adapter_name=args.adapter_name
        )
        
        logger.info("\n" + "="*60)
        logger.info("✅ Model and adapter loaded successfully!")
        logger.info(f"   Model: {args.model}")
        logger.info(f"   Adapter: {args.adapter_name} from {args.adapter_path}")
        logger.info("="*60 + "\n")
        
        # Test generation if requested
        if not args.skip_generation:
            test_generation(runtime, adapter_name, args.model)
        
        logger.info("\n✅ Test completed successfully!")
        
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()

