from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence, Tuple, Dict, Set
import re

from scienceworld import ScienceWorldEnv

_LOCATION_RE = re.compile(
    r"\byou are in (?:the )?([a-zA-Z0-9 _-]+?)(?:\.|\n)",
    re.IGNORECASE,
)

def extract_locations_from_observation(obs_text: str) -> set[str]:
    locations = set()
    for match in _LOCATION_RE.finditer(obs_text):
        loc = match.group(1).strip()
        if not loc:
            continue
        if loc in {"air", "sky", "space"}:   # hard exclusions
            continue
        print(f"Found location: {loc}")
        locations.add(loc.lower())
    return locations

# -----------------------------
# Action parsing
# -----------------------------

@dataclass(frozen=True)
class Action:
    raw: str
    verb: str
    arg1: Optional[str] = None
    prep: Optional[str] = None
    arg2: Optional[str] = None

    def key(self) -> Tuple[str, Optional[str], Optional[str], Optional[str]]:
        return (self.verb, self.arg1, self.prep, self.arg2)


_PREP_SET = {"in", "on", "into", "to", "from", "with", "at", "under", "over", "inside", "on top of"}
# Prefer longer preps first (e.g. "on top of" before "on")
_PREP_REGEX = sorted(_PREP_SET, key=len, reverse=True)


def parse_action(raw: str) -> Optional[Action]:
    """
    Parse common ScienceWorld-ish actions:
      - look
      - examine X
      - go north
      - take X / pick up X
      - open X / close X
      - eat X
      - use X on Y
      - put X in Y / put X on Y / move X to Y
      - pour X in Y / dunk X in Y

    Returns None if it can't parse into a supported structure.
    """
    s = raw.strip()
    if not s:
        return None
    s = re.sub(r"\s+", " ", s)

    # Single-word verbs
    if s in {"look"}:
        return Action(raw=raw, verb=s)

    # Two+ token verbs like "pick up"
    if s.startswith("pick up "):
        rest = s[len("pick up "):].strip()
        return Action(raw=raw, verb="take", arg1=rest)

    # General: verb + remainder
    parts = s.split(" ", 1)
    verb = parts[0]
    rest = parts[1].strip() if len(parts) > 1 else ""

    if verb in {"examine", "take", "open", "close", "eat", "go"}:
        if not rest and verb != "go":
            return None
        return Action(raw=raw, verb=verb, arg1=rest or None)

    # handle patterns: "use X on Y", "put X in Y", "move X to Y", etc.
    if verb in {"use", "put", "move", "pour", "dunk"}:
        if not rest:
            return None

        # Find a preposition occurrence (first match in preference order)
        prep_found = None
        for prep in _PREP_REGEX:
            # word-boundary-ish matching
            token = f" {prep} "
            if token in f" {rest} ":
                prep_found = prep
                break

        if prep_found is None:
            # "use X" might exist, but we treat as unsupported for filtering
            return Action(raw=raw, verb=verb, arg1=rest)

        left, right = rest.split(f" {prep_found} ", 1)
        arg1 = left.strip() or None
        arg2 = right.strip() or None
        return Action(raw=raw, verb=verb, arg1=arg1, prep=prep_found, arg2=arg2)

    # Unknown verb -> still represent it, but likely filtered out.
    return Action(raw=raw, verb=verb, arg1=rest or None)


# -----------------------------
# World state interface
# -----------------------------

@dataclass
class WorldState:
    """
    Keep this light. You can fill these from ScienceWorld observations.
    """
    location: Optional[str] = None                # current room/location name
    inventory: Set[str] = field(default_factory=set)
    visible: Set[str] = field(default_factory=set)  # objects visible in current location
    containers: Set[str] = field(default_factory=set)  # known containers/doors
    surfaces: Set[str] = field(default_factory=set)    # tables/counters/etc (optional)
    liquids: Set[str] = field(default_factory=set)     # things considered liquids (optional)
    # If you can track containment (container -> set(items)), it improves "take container w/ stuff" logic:
    contains: Dict[str, Set[str]] = field(default_factory=dict)


# -----------------------------
# Heuristic classifiers
# -----------------------------

class Heuristics:
    NON_PHYSICAL = {"agent", "self", "air", "sky", "light", "sound", "time"}
    DIRECTIONS = {"north", "south", "east", "west", "up", "down"}

    # You can expand/override these per task.
    DEFAULT_TOOL_HINTS = ("knife", "shovel", "spoon", "tool", "key", "watering can", "hose", "scissors")
    DEFAULT_EDIBLE_HINTS = ("apple", "bread", "food", "fruit", "sandwich", "banana", "cookie")
    DEFAULT_CONTAINER_HINTS = ("fridge", "box", "drawer", "door", "cabinet", "jar", "bottle", "cup", "mug", "pot", "pan", "bucket")
    DEFAULT_SURFACE_HINTS = ("table", "counter", "bench", "shelf", "desk")
    DEFAULT_LIQUID_HINTS = ("water", "milk", "juice", "oil", "liquid")

    def __init__(
        self,
        locations: Optional[Set[str]] = None,
        tool_hints: Sequence[str] = DEFAULT_TOOL_HINTS,
        edible_hints: Sequence[str] = DEFAULT_EDIBLE_HINTS,
        container_hints: Sequence[str] = DEFAULT_CONTAINER_HINTS,
        surface_hints: Sequence[str] = DEFAULT_SURFACE_HINTS,
        liquid_hints: Sequence[str] = DEFAULT_LIQUID_HINTS,
    ):
        self.locations = locations or set()
        self.tool_hints = tuple(tool_hints)
        self.edible_hints = tuple(edible_hints)
        self.container_hints = tuple(container_hints)
        self.surface_hints = tuple(surface_hints)
        self.liquid_hints = tuple(liquid_hints)

    @staticmethod
    def _norm(x: Optional[str]) -> str:
        return (x or "").strip().lower()

    def is_location(self, x: Optional[str]) -> bool:
        nx = self._norm(x)
        return nx in self.locations

    def is_direction(self, x: Optional[str]) -> bool:
        return self._norm(x) in self.DIRECTIONS

    def is_non_physical(self, x: Optional[str]) -> bool:
        return self._norm(x) in self.NON_PHYSICAL

    def name_has_any(self, x: Optional[str], hints: Sequence[str]) -> bool:
        nx = self._norm(x)
        return any(h in nx for h in hints)

    def is_tool_like(self, x: Optional[str]) -> bool:
        return self.name_has_any(x, self.tool_hints)

    def is_edible(self, x: Optional[str]) -> bool:
        return self.name_has_any(x, self.edible_hints)

    def is_container_like(self, x: Optional[str], ws: Optional[WorldState] = None) -> bool:
        if ws and x and x in ws.containers:
            return True
        return self.name_has_any(x, self.container_hints)

    def is_surface_like(self, x: Optional[str], ws: Optional[WorldState] = None) -> bool:
        if ws and x and x in ws.surfaces:
            return True
        return self.name_has_any(x, self.surface_hints)

    def is_liquid_like(self, x: Optional[str], ws: Optional[WorldState] = None) -> bool:
        if ws and x and x in ws.liquids:
            return True
        return self.name_has_any(x, self.liquid_hints)

    def is_portable(self, x: Optional[str], ws: Optional[WorldState] = None) -> bool:
        """
        Extremely cheap: treat anything not a location/container-door as portable.
        Tighten if you have object metadata.
        """
        if not x:
            return False
        if self.is_location(x) or self.is_direction(x):
            return False
        if self.is_non_physical(x):
            return False
        # doors are usually non-portable even though they’re "containers"
        if self.is_container_like(x, ws) and self.name_has_any(x, ("door",)):
            return False
        return True


# -----------------------------
# Affordance filter
# -----------------------------

@dataclass
class FilterConfig:
    # Enable/disable layers
    use_inventory_checks: bool = True
    use_location_checks: bool = True
    suppress_repeats_after: int = 2  # if failed >= N times, hide it


class AffordanceFilter:
    def __init__(self, heur: Heuristics, cfg: Optional[FilterConfig] = None):
        self.heur = heur
        self.cfg = cfg or FilterConfig()
        self.fail_counts: Dict[Tuple[str, Optional[str], Optional[str], Optional[str]], int] = {}

    def note_failure(self, action_raw: str):
        a = parse_action(action_raw)
        if not a:
            return
        self.fail_counts[a.key()] = self.fail_counts.get(a.key(), 0) + 1

    def _is_suppressed(self, a: Action) -> bool:
        if self.cfg.suppress_repeats_after <= 0:
            return False
        return self.fail_counts.get(a.key(), 0) >= self.cfg.suppress_repeats_after

    def filter_actions(self, raw_actions: Iterable[str], ws: Optional[WorldState] = None) -> list[str]:
        out: list[str] = []
        ws = ws or WorldState()

        for raw in raw_actions:
            a = parse_action(raw)
            if a is None:
                continue

            if self._is_suppressed(a):
                continue

            if not self._layer1_type_exclusions(a, ws):
                continue

            if not self._layer2_verb_role_rules(a, ws):
                continue

            if self.cfg.use_inventory_checks and not self._layer3_inventory_rules(a, ws):
                continue

            if self.cfg.use_location_checks and not self._layer3_location_rules(a, ws):
                continue

            out.append(raw)

        return out

    # ---- Layer 1: hard exclusions ----

    def _layer1_type_exclusions(self, a: Action, ws: WorldState) -> bool:
        # Non-physical entity anywhere -> reject
        if self.heur.is_non_physical(a.arg1) or self.heur.is_non_physical(a.arg2):
            return False

        # Block physical manipulation verbs on locations
        physical_verbs = {"eat", "take", "open", "close", "pour", "dunk", "put", "move", "use"}
        if a.verb in physical_verbs:
            if self.heur.is_location(a.arg1) or self.heur.is_location(a.arg2):
                return False

        return True

    # ---- Layer 2: verb-role rules ----

    def _layer2_verb_role_rules(self, a: Action, ws: WorldState) -> bool:
        v = a.verb

        # Always safe
        if v in {"look", "examine"}:
            return True

        if v == "go":
            return self.heur.is_direction(a.arg1) or self.heur.is_location(a.arg1)

        if v == "take":
            if not a.arg1:
                return False
            # can't take a container that currently contains other objects (optional)
            if a.arg1 in ws.contains and ws.contains[a.arg1]:
                return False
            return self.heur.is_portable(a.arg1, ws) and not self.heur.is_container_like(a.arg1, ws)

        if v in {"open", "close"}:
            return self.heur.is_container_like(a.arg1, ws)

        if v == "eat":
            return self.heur.is_edible(a.arg1)

        if v == "use":
            # either "use X on Y" or "use X" (we keep "use X" if tool-like, but it's rarer)
            if not a.arg1:
                return False
            if a.prep is None:
                return self.heur.is_tool_like(a.arg1)
            if a.prep != "on":
                return False
            if not a.arg2:
                return False
            return self.heur.is_tool_like(a.arg1) and (self.heur.is_portable(a.arg2, ws) or self.heur.is_container_like(a.arg2, ws) or self.heur.is_surface_like(a.arg2, ws))

        if v in {"put", "move"}:
            if not (a.arg1 and a.arg2 and a.prep):
                return False
            if a.arg1 == a.arg2:
                return False
            if not self.heur.is_portable(a.arg1, ws):
                return False
            # destinations: container / surface / location
            return (
                self.heur.is_container_like(a.arg2, ws)
                or self.heur.is_surface_like(a.arg2, ws)
                or self.heur.is_location(a.arg2)
            )

        if v in {"pour", "dunk"}:
            if not (a.arg1 and a.arg2 and a.prep):
                return False
            # enforce "in/into" most of the time
            if a.prep not in {"in", "into"}:
                return False
            # source must be liquid-like OR container-with-liquid (liquid-like name or in ws.liquids)
            src_ok = self.heur.is_liquid_like(a.arg1, ws)
            # a.arg1 might be a container that holds a liquid object
            if not src_ok and a.arg1 in ws.contains:
                src_ok = any(self.heur.is_liquid_like(x, ws) for x in ws.contains[a.arg1])
            if not src_ok:
                return False
            # destination must be a container-like thing
            return self.heur.is_container_like(a.arg2, ws)

        # Unknown verbs: drop by default (you can whitelist if needed)
        return False

    # ---- Layer 3: inventory and location sanity ----

    def _layer3_inventory_rules(self, a: Action, ws: WorldState) -> bool:
        # Actions that usually require holding arg1
        requires_holding_arg1 = {"put", "move", "use", "pour", "dunk", "eat"}
        if a.verb in requires_holding_arg1 and a.arg1:
            # liquids might be "water" not in inventory but in a container; so skip if liquid-like and not strictly held
            if a.verb in {"pour", "dunk"} and self.heur.is_liquid_like(a.arg1, ws):
                return True
            return a.arg1 in ws.inventory
        return True

    def _layer3_location_rules(self, a: Action, ws: WorldState) -> bool:
        # If you have a "visible" set, require targets (arg2 mostly) to be visible for interaction.
        interaction_verbs = {"open", "close", "use", "put", "move", "pour", "dunk", "take", "eat"}
        if a.verb in interaction_verbs:
            # arg1 can be in inventory; arg2 should generally be visible (unless it's a location)
            if a.arg2 and not self.heur.is_location(a.arg2):
                if ws.visible and (a.arg2 not in ws.visible):
                    return False
            # open/close/take on arg1 usually needs visibility if not in inventory
            if a.arg1 and a.arg1 not in ws.inventory and a.verb in {"open", "close", "take", "eat"}:
                if ws.visible and (a.arg1 not in ws.visible):
                    return False
        return True


# -----------------------------
# Example usage
# -----------------------------

if __name__ == "__main__":
    env = ScienceWorldEnv()
    env.load('grow-plant', 0, '')
    obs, info = env.reset()    
    from scienceworld_interface import extract_actions_and_state

    heur = Heuristics(locations={"kitchen", "bedroom", "bathroom", "living room", "hallway", "garage", "workshop", "greenhouse", "outside"})
    filt = AffordanceFilter(heur)

    raw_actions, ws = extract_actions_and_state(env)
    new_locs = extract_locations_from_observation(obs)
    heur.locations |= new_locs
    #print(f"New locations: {new_locs}")
    filtered = filt.filter_actions(raw_actions, ws)
    print("Filtered actions:")
    for a in filtered:
        print("  ", a)

    # Suppose "use knife on apple" failed twice; suppress afterwards.
    filt.note_failure("use knife on apple")
    filt.note_failure("use knife on apple")
    filtered2 = filt.filter_actions(raw_actions, ws)
    print("\nAfter suppression:")
    for a in filtered2:
        print("  ", a)
