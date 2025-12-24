# scienceworld_interface.py

from __future__ import annotations
from typing import List, Set, Dict, Tuple, Optional
import re

from affordances import WorldState   # import your earlier class


# ---------------------------------------------------------
# Utilities
# ---------------------------------------------------------

_OBJ_SPLIT_RE = re.compile(r",\s*|\n")


def _normalize_obj(name: str) -> str:
    return name.strip().lower()


# ---------------------------------------------------------
# Action extraction
# ---------------------------------------------------------

def extract_raw_actions(env) -> List[str]:
    """
    Pull raw action strings directly from ScienceWorld.

    Preference order:
    1) get_possible_action_object_combinations() (snake_case)
    2) getPossibleActionObjectCombinations() (camelCase, deprecated)
    3) get_possible_actions() (snake_case)
    4) getPossibleActions() (camelCase, deprecated)
    """
    actions = set()

    # Try snake_case API first
    if hasattr(env, "get_possible_action_object_combinations"):
        try:
            templates, _ = env.get_possible_action_object_combinations()
            if templates:
                for template in templates:
                    if isinstance(template, dict):
                        action = template.get('action', '')
                        if action:
                            actions.add(action)
                    elif isinstance(template, str):
                        actions.add(template.strip())
        except Exception as e:
            pass  # Fall through to next method

    # Try camelCase API (deprecated)
    if not actions and hasattr(env, "getPossibleActionObjectCombinations"):
        try:
            combos = env.getPossibleActionObjectCombinations()
            if combos:
                # Handle tuple return format: (templates, _)
                if isinstance(combos, tuple) and len(combos) >= 1:
                    templates = combos[0]
                    for template in templates:
                        if isinstance(template, dict):
                            action = template.get('action', '')
                            if action:
                                actions.add(action)
                        elif isinstance(template, str):
                            actions.add(template.strip())
                # Handle list of strings
                elif isinstance(combos, list):
                    for c in combos:
                        if isinstance(c, str):
                            actions.add(c.strip())
                        elif isinstance(c, dict):
                            action = c.get('action', '')
                            if action:
                                actions.add(action)
        except Exception as e:
            pass  # Fall through to next method

    # Try state-dependent actions (snake_case)
    if not actions and hasattr(env, "get_possible_actions"):
        try:
            acts = env.get_possible_actions()
            if acts:
                for a in acts:
                    if isinstance(a, str):
                        actions.add(a.strip())
        except Exception as e:
            pass  # Fall through to next method

    # Try state-dependent actions (camelCase, deprecated)
    if not actions and hasattr(env, "getPossibleActions"):
        try:
            acts = env.getPossibleActions()
            if acts:
                for a in acts:
                    if isinstance(a, str):
                        actions.add(a.strip())
        except Exception as e:
            pass

    if actions:
        return sorted(actions)

    raise RuntimeError("ScienceWorld env exposes no possible-action API")


# ---------------------------------------------------------
# Observation parsing
# ---------------------------------------------------------

def _parse_inventory(text: str) -> Set[str]:
    """
    Parse inventory output.
    Typical format:
      "You are carrying: apple, knife"
    """
    inv = set()
    if ":" in text:
        _, rhs = text.split(":", 1)
        for item in _OBJ_SPLIT_RE.split(rhs):
            item = _normalize_obj(item)
            if item and item != "nothing":
                inv.add(item)
    return inv


def _parse_visible_objects(text: str) -> Set[str]:
    """
    Very lightweight heuristic:
    extract noun-like phrases from 'look' output.
    This does NOT need to be perfect; affordance filter is conservative.
    """
    objs = set()

    # Common ScienceWorld phrasing:
    # "You see a table. On the table is an apple."
    for line in text.splitlines():
        line = line.lower()

        # simple patterns
        for token in re.findall(r"\b[a-z][a-z0-9_ ]+\b", line):
            token = token.strip()
            if len(token) <= 2:
                continue
            if token in {"you", "see", "are", "is", "on", "in", "the"}:
                continue
            objs.add(token)

    return objs


def _parse_location(text: str) -> Optional[str]:
    """
    Extract current location name if present.
    ScienceWorld often says:
      'You are in the kitchen.'
    """
    m = re.search(r"you are in (?:the )?([a-z_ ]+)", text.lower())
    if m:
        return m.group(1).strip()
    return None


# ---------------------------------------------------------
# WorldState extraction
# ---------------------------------------------------------

def extract_world_state(env) -> WorldState:
    """
    Actively query ScienceWorld to build a WorldState snapshot.
    """

    # --- LOOK ---
    obs, _, _, _ = env.step("look")
    look_text = obs

    visible = _parse_visible_objects(look_text)
    location = _parse_location(look_text)

    # --- INVENTORY ---
    obs, _, _, _ = env.step("inventory")
    inventory = _parse_inventory(obs)

    # --- Containers / surfaces / liquids (best-effort) ---
    containers = set()
    surfaces = set()
    liquids = set()

    for obj in visible:
        if any(k in obj for k in ("fridge", "cabinet", "drawer", "box", "jar", "pot", "cup", "bottle")):
            containers.add(obj)
        if any(k in obj for k in ("table", "counter", "bench", "shelf")):
            surfaces.add(obj)
        if any(k in obj for k in ("water", "liquid", "juice", "milk")):
            liquids.add(obj)

    return WorldState(
        location=location,
        inventory=inventory,
        visible=visible,
        containers=containers,
        surfaces=surfaces,
        liquids=liquids,
        contains={},  # optional; can be filled later if you track it
    )


# ---------------------------------------------------------
# Unified entry point
# ---------------------------------------------------------

def extract_actions_and_state(env) -> Tuple[List[str], WorldState]:
    """
    Single call used by your planner loop.
    """
    raw_actions = extract_raw_actions(env)
    ws = extract_world_state(env)
    return raw_actions, ws
