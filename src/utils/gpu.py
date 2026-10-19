"""Choose the spare GPU for in-process models (STT, embeddings).

This machine has two cards: the RTX PRO 6000 serves the local LLM (vLLM,
nearly all of its memory), and the 5060 Ti is spare. torch's default device
order is the reverse of nvidia-smi's here, so plain 'cuda' selects the PRO
6000 and puts a second process on the card vLLM uses. Pick the card by name
instead.
"""
import logging
import os

logger = logging.getLogger(__name__)

# A substring of the CUDA device name of the spare card.
SPARE_GPU_MATCH = os.environ.get("CW_SPARE_GPU", "5060")


def spare_cuda_device(match: str = SPARE_GPU_MATCH) -> tuple:
    """(device, device_index): ('cuda', i) for the CUDA device whose name
    contains `match`, else ('cpu', 0). Uses torch's enumeration, which is the
    same CUDA runtime order ctranslate2 sees."""
    try:
        import torch
        for i in range(torch.cuda.device_count()):
            if match in torch.cuda.get_device_name(i):
                return "cuda", i
        logger.warning(f"no CUDA device matching {match!r}; using CPU")
    except Exception as e:
        logger.warning(f"torch unavailable for GPU pick ({e}); using CPU")
    return "cpu", 0
