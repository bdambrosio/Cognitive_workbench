import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class AdapterMagnitude:
    """Magnitude controls for transient adapters."""

    max_alpha: float = 16.0
    max_weight_norm: Optional[float] = None


@dataclass
class AdapterConfig:
    """
    Configuration for the fixed adapter scaffold described in docs/musing_spec.md.

    The adapter architecture is static; adapter weights and alpha are supplied
    at runtime by the bias policy G.
    """

    rank: int = 4
    attn_scale: float = 1.0
    mlp_scale: float = 1.0
    target_layers: Optional[List[int]] = None
    alpha: float = 0.0
    magnitude: AdapterMagnitude = field(default_factory=AdapterMagnitude)

    def resolve_layers(self, num_layers: int) -> List[int]:
        """
        Choose target layers. Defaults to the middle third to avoid syntax drift
        in shallow layers while still affecting reasoning depth.
        """
        if self.target_layers is not None:
            return list(self.target_layers)

        if num_layers == 0:
            return []

        one_third = math.floor(num_layers / 3)
        start = one_third
        end = num_layers - one_third
        return list(range(start, end))

    def validate(self):
        if self.rank <= 0:
            raise ValueError("Adapter rank must be positive")
        if self.alpha < 0:
            raise ValueError("Initial alpha must be non-negative")
        if self.magnitude.max_alpha <= 0:
            raise ValueError("max_alpha must be positive")
