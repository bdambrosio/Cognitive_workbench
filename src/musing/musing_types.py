"""
Type definitions for musing adapter system.
"""

from dataclasses import dataclass


@dataclass
class AdapterSiteShape:
    """Shape metadata for a single adapter site."""

    in_features: int
    out_features: int
    rank: int

