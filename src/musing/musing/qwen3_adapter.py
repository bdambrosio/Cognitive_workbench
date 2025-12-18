import logging
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F

from .config import AdapterConfig
from .segment import SegmentState

logger = logging.getLogger(__name__)


@dataclass
class AdapterSiteShape:
    """Shape metadata for a single adapter site."""

    in_features: int
    out_features: int
    rank: int


class LoRAInjectedLinear(nn.Module):
    """
    Wraps a linear projection with a fixed low-rank adapter scaffold.

    The base weight is untouched; runtime adapter weights are injected via A/B
    matrices supplied by the bias policy G. Adapter parameters remain frozen
    inside a generation segment per docs/musing_spec.md.
    """

    def __init__(
        self,
        base: nn.Linear,
        rank: int,
        scale: float,
        alpha: float,
        max_alpha: float,
        max_norm: Optional[float],
    ):
        super().__init__()
        if not isinstance(base, nn.Linear):
            raise TypeError("LoRAInjectedLinear expects an nn.Linear base module")

        self.base = base
        self.rank = rank
        self.scale = scale
        self.max_alpha = max_alpha
        self.max_norm = max_norm

        device = base.weight.device
        dtype = base.weight.dtype

        self.register_buffer("A", torch.zeros(rank, base.in_features, device=device, dtype=dtype))
        self.register_buffer("B", torch.zeros(base.out_features, rank, device=device, dtype=dtype))
        self.alpha = float(alpha)

        # Ensure the base projection stays frozen
        for param in self.base.parameters():
            param.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        base_out = self.base(x)

        if self.alpha == 0.0 or self.rank == 0:
            return base_out

        delta_weight = torch.matmul(self.B, self.A)
        delta = F.linear(x, delta_weight)
        return base_out + (self.alpha * self.scale) * delta

    def set_alpha(self, alpha: float):
        clamped = max(0.0, min(float(alpha), float(self.max_alpha)))
        self.alpha = clamped

    def set_adapter_weights(self, a: torch.Tensor, b: torch.Tensor):
        if a.shape != self.A.shape:
            raise ValueError(f"Adapter A has shape {a.shape}, expected {self.A.shape}")
        if b.shape != self.B.shape:
            raise ValueError(f"Adapter B has shape {b.shape}, expected {self.B.shape}")

        with torch.no_grad():
            a_cast = a.to(device=self.A.device, dtype=self.A.dtype)
            b_cast = b.to(device=self.B.device, dtype=self.B.dtype)
            if self.max_norm:
                delta = torch.matmul(b_cast, a_cast)
                norm = torch.linalg.vector_norm(delta)
                if norm > self.max_norm:
                    scale = float(self.max_norm / (norm + 1e-12))
                    b_cast = b_cast * scale
            self.A.copy_(a_cast)
            self.B.copy_(b_cast)

    def zero_adapter(self):
        with torch.no_grad():
            self.A.zero_()
            self.B.zero_()
        self.alpha = 0.0

    @property
    def site_shape(self) -> AdapterSiteShape:
        return AdapterSiteShape(
            in_features=self.A.shape[1],
            out_features=self.B.shape[0],
            rank=self.rank,
        )


class Qwen3AdapterScaffold:
    """
    Installs fixed LoRA-style adapter hooks for Qwen3-Coder-30B modules.

    Attachment points follow docs/musing_spec.md:
      - Attention output projection (W_o)
      - MLP up projection (W_up)
    Adapter weights and alpha can only change at segment boundaries; callers
    should use start_segment/end_segment to protect the KV cache invariants.
    """

    def __init__(
        self,
        model: nn.Module,
        config: Optional[AdapterConfig] = None,
        segment_state: Optional[SegmentState] = None,
    ):
        self.model = model
        self.config = config or AdapterConfig()
        self.segment_state = segment_state or SegmentState()
        self.adapter_modules: Dict[str, LoRAInjectedLinear] = {}
        self._install_scaffold()

    def _extract_layers(self) -> Iterable[nn.Module]:
        for path in ("model.layers", "transformer.layers", "layers"):
            target = self.model
            for part in path.split("."):
                target = getattr(target, part, None)
                if target is None:
                    break
            if target is not None:
                return target
        raise ValueError("Unable to locate decoder layers on Qwen3 model")

    def _install_scaffold(self):
        layers = list(self._extract_layers())
        self.config.validate()
        target_layers = self.config.resolve_layers(len(layers))

        for idx in target_layers:
            try:
                layer = layers[idx]
            except IndexError:
                logger.warning("Skipping layer %s; index out of range", idx)
                continue

            attn = getattr(layer, "self_attn", None) or getattr(layer, "attn", None)
            mlp = getattr(layer, "mlp", None)

            if attn and hasattr(attn, "o_proj"):
                wrapper = LoRAInjectedLinear(
                    attn.o_proj,
                    rank=self.config.rank,
                    scale=self.config.attn_scale,
                    alpha=self.config.alpha,
                    max_alpha=self.config.magnitude.max_alpha,
                    max_norm=self.config.magnitude.max_weight_norm,
                )
                attn.o_proj = wrapper
                key = f"layer{idx}.attn_o"
                self.adapter_modules[key] = wrapper
                logger.info("Installed adapter scaffold at %s", key)
            else:
                logger.warning("Layer %s missing attention o_proj; skipping adapter", idx)

            if mlp and hasattr(mlp, "up_proj"):
                wrapper = LoRAInjectedLinear(
                    mlp.up_proj,
                    rank=self.config.rank,
                    scale=self.config.mlp_scale,
                    alpha=self.config.alpha,
                    max_alpha=self.config.magnitude.max_alpha,
                    max_norm=self.config.magnitude.max_weight_norm,
                )
                mlp.up_proj = wrapper
                key = f"layer{idx}.mlp_up"
                self.adapter_modules[key] = wrapper
                logger.info("Installed adapter scaffold at %s", key)
            else:
                logger.warning("Layer %s missing mlp up_proj; skipping adapter", idx)

    def start_segment(self):
        self.segment_state.start()

    def end_segment(self):
        self.segment_state.end()

    def apply_adapter_weights(self, weights: Dict[str, Tuple[torch.Tensor, torch.Tensor]]):
        """
        Apply adapter weights emitted by G. Must be called at a segment boundary.
        """
        self.segment_state.ensure_boundary("Updating adapter weights")

        for key, (a, b) in weights.items():
            module = self.adapter_modules.get(key)
            if module is None:
                raise KeyError(f"Adapter site {key} not found on scaffold")
            module.set_adapter_weights(a, b)

    def set_alpha(self, alpha: float) -> float:
        """Set the global musing scale (clamped)."""
        self.segment_state.ensure_boundary("Updating alpha")
        clamped = max(0.0, min(float(alpha), self.config.magnitude.max_alpha))
        for module in self.adapter_modules.values():
            module.set_alpha(clamped)
        return clamped

    def decay_alpha(self, factor: float) -> float:
        """Decay alpha at a segment boundary; useful for exponential schedules."""
        if factor < 0:
            raise ValueError("Decay factor must be non-negative")
        current_alpha = next(iter(self.adapter_modules.values())).alpha if self.adapter_modules else 0.0
        return self.set_alpha(current_alpha * factor)

    def zero_adapters(self):
        """Disable musing and clear adapter weights at a segment boundary."""
        self.segment_state.ensure_boundary("Zeroing adapters")
        for module in self.adapter_modules.values():
            module.zero_adapter()

    def site_shapes(self) -> Dict[str, AdapterSiteShape]:
        """Expose site shapes so G can emit correctly-sized matrices."""
        return {key: module.site_shape for key, module in self.adapter_modules.items()}
