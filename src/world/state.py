"""WorldState — who is in the world and where.

Mirrors the shape of canvas/state.py (dataclass + to_json), but this state
is shared rather than per-character: one world, several occupants. The
server owns it; the browser and the agent tools are both clients.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional  # noqa: F401

WALK_SPEED_MS = 1.7          # comfortable walking pace
ARRIVE_TOL_M = 0.8


@dataclass
class Occupant:
    """One embodied presence. `kind` decides who drives it: a human is
    driven by keyboard input from the browser, an agent by movement goals
    posted through the world-move tool."""
    name: str
    kind: str = 'agent'                  # 'human' | 'agent'
    x: float = 0.0
    y: float = 0.0                       # ground height, server-derived
    z: float = 0.0
    heading: float = 0.0                 # radians, 0 = +z
    gait: str = 'idle'                   # 'idle' | 'walk'
    color: str = '#8ab4f8'
    goal: Optional[List[float]] = None   # [x, z] while walking
    online: bool = True

    def distance_to(self, other: 'Occupant') -> float:
        return math.hypot(self.x - other.x, self.z - other.z)

    def bearing_to(self, other: 'Occupant') -> float:
        """Absolute bearing from self to other, radians, 0 = +z."""
        return math.atan2(other.x - self.x, other.z - self.z)

    def facing_offset(self, other: 'Occupant') -> float:
        """How far off `other` is from where self is looking, in radians
        [0, pi]. Small means self is looking at other."""
        d = self.bearing_to(other) - self.heading
        return abs(math.atan2(math.sin(d), math.cos(d)))


MAX_MARKERS = 40


@dataclass
class Marker:
    """A thing left in the world rather than said.

    Persists past the moment it was placed, which is what makes it usable
    across a rate gap: a marker dropped now is still there when the other
    party gets round to looking, and it points at a place without needing
    a coordinate to be read out loud.
    """
    id: int
    label: str
    by: str
    x: float
    y: float
    z: float
    ts: float = 0.0


@dataclass
class WorldState:
    occupants: Dict[str, Occupant] = field(default_factory=dict)
    markers: List[Marker] = field(default_factory=list)
    tick: int = 0
    ts: float = 0.0

    def to_json(self) -> str:
        return json.dumps(
            {'tick': self.tick, 'ts': round(self.ts, 3),
             'occupants': [asdict(o) for o in self.occupants.values()],
             'markers': [asdict(m) for m in self.markers]},
            separators=(',', ':'))
