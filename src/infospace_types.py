"""
Infospace type definitions.

Provides lightweight versions of the resource/terrain enums and the
ResourceTypeRegistry that were previously scattered across the map stack.
"""

from __future__ import annotations

from enum import Enum, auto
from typing import Dict, Optional


class ResourceType:
    """Represents a single resource type, compatible with Enum-like access."""

    def __init__(self, name: str, value: int, description: str = ""):
        self.name = name
        self.value = value
        self.description = description

    def __eq__(self, other):
        if isinstance(other, ResourceType):
            return self.value == other.value
        if hasattr(other, "value"):
            return self.value == other.value
        return False

    def __hash__(self):
        return hash(self.value)

    def __repr__(self):
        return f"ResourceType.{self.name}"

    def __str__(self):
        return self.name


class ResourceTypeRegistry:
    """
    Dynamic registry for resource types.

    Mirrors Enum access semantics so existing map-node code can continue to use
    attribute lookups (e.g., resource_types.Note).
    """

    def __init__(self, base_enum: Optional[type[Enum]] = None):
        self._members: Dict[str, ResourceType] = {}
        self._counter = 1000  # Avoid collisions with builtin enum values.
        self._descriptions: Dict[str, str] = {}

        if base_enum:
            for name, member in base_enum.__members__.items():
                self._members[name] = ResourceType(name, member.value)
                self._descriptions[name] = getattr(member, "description", "")

    def __getattr__(self, name: str) -> ResourceType:
        if name.startswith("_"):
            raise AttributeError(f"{self.__class__.__name__} has no attribute {name}")
        if name in self._members:
            return self._members[name]
        raise AttributeError(f"No resource type: {name}")

    def __hasattr__(self, name: str) -> bool:  # pragma: no cover - sugar for callers
        return name in self._members

    @property
    def __members__(self) -> Dict[str, ResourceType]:
        return dict(self._members)

    def add_type(self, name: str, description: str = "") -> ResourceType:
        if name not in self._members:
            new_type = ResourceType(name, self._counter, description)
            self._members[name] = new_type
            self._descriptions[name] = description
            self._counter += 1
            return new_type
        return self._members[name]

    def get_description(self, name: str) -> str:
        return self._descriptions.get(name, "")


class InfospaceTerrain(Enum):
    """Single logical terrain used by the infospace runtime."""

    InfoSpace = 1


class InfospaceResources(Enum):
    """Core resource kinds managed by the infospace runtime."""

    Note = auto()
    Collection = auto()
