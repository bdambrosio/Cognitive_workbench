import logging
from typing import Dict, Optional

import torch

from .musing_types import AdapterSiteShape

logger = logging.getLogger(__name__)


class RandomGaussianPolicy:
    """
    Simple initializer policy: emits small random adapter weights and a fixed alpha.
    Useful for bootstrapping RL/bandit training where G will later be learned.
    """

    def __init__(self, sigma: float = 1e-3, alpha: float = 0.5, enabled: bool = True, seed: Optional[int] = None):
        self.sigma = float(sigma)
        self.alpha = float(alpha)
        self.enabled = enabled
        self._rng = torch.Generator()
        if seed is not None:
            self._rng.manual_seed(seed)

    def __call__(self, payload: Dict):
        if not self.enabled:
            return None
        shapes: Dict[str, AdapterSiteShape] = payload.get("site_shapes", {})
        weights = {}
        for name, shape in shapes.items():
            a = torch.randn(shape.rank, shape.in_features, generator=self._rng) * self.sigma
            b = torch.randn(shape.out_features, shape.rank, generator=self._rng) * self.sigma
            weights[name] = (a, b)
        logger.debug("RandomGaussianPolicy emitted weights for %d sites", len(weights))
        return {"weights": weights, "alpha": self.alpha}
