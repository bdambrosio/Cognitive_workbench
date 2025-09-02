#!/usr/bin/env python3
"""
Minimal physiology state utilities.

State convention: each dimension is 0..100 where 0 is best (no need/condition), 100 is worst.
This module is intentionally lightweight and stateless.
"""
from typing import Dict, Any

# Known physiological and psychological states
KNOWN_STATES = ["hunger", "fatigue", "injury", "thirst"]

def get_known_states() -> list:
    """Get list of all known state names"""
    return KNOWN_STATES.copy()

def get_default_state_value() -> float:
    """Get default starting value for states (0 = best condition)"""
    return 0.0

def initialize_character_states() -> Dict[str, Dict[str, float]]:
    """Initialize character state with all known states set to default values"""
    return {state: {'value': get_default_state_value()} for state in KNOWN_STATES}


def clamp(value: float, lo: float = 0.0, hi: float = 100.0) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def tick_state(state: Dict[str, Dict[str, Any]], dt_minutes: float, *, hunger_decay_per_min: float = 0.2, fatigue_decay_per_min: float = 0.2, thirst_decay_per_min: float = 0.6) -> None:
    """
    Advance state values forward in time by dt_minutes.

    - hunger increases (worsens) by hunger_decay_per_min per minute, clamped to 100.
    - fatigue increases (worsens) by fatigue_decay_per_min per minute, clamped to 100.
    - other dimensions can be added later with their own decay rules.
    - state is mutated in-place.
    """
    if not state or dt_minutes <= 0:
        return

    # Initialize keys if missing
    for state_name in KNOWN_STATES:
        if state_name not in state:
            state[state_name] = {'value': 0.0}

    # Hunger worsens with time
    hunger = float(state['hunger'].get('value', 0.0))
    hunger += hunger_decay_per_min * float(dt_minutes)
    state['hunger']['value'] = clamp(hunger)
    
    # Fatigue worsens with time (slower than hunger)
    fatigue = float(state['fatigue'].get('value', 0.0))
    fatigue += fatigue_decay_per_min * float(dt_minutes)
    state['fatigue']['value'] = clamp(fatigue)

    # Thirst worsens with time (faster than hunger)
    thirst = float(state['thirst'].get('value', 0.0))
    thirst += thirst_decay_per_min * float(dt_minutes)
    state['thirst']['value'] = clamp(thirst)


def apply_restore(state: Dict[str, Dict[str, Any]], key: str, amount: float) -> None:
    """
    Reduce the severity for a state dimension by amount (i.e., move toward 0).
    Mutates state in-place.
    """
    if key not in state:
        state[key] = {'value': 0.0}
    current = float(state[key].get('value', 0.0))
    state[key]['value'] = clamp(current - float(amount))


def calculate_state_activity_alignment(activity: Dict[str, Any], character_state: Dict[str, Dict[str, float]]) -> float:
    """Calculate continuous state-activity alignment using squared scaling"""
    if not character_state:
        return 0.0
    
    alignment_score = 0.0
    
    # Hunger alignment - eating activities get bonus when hungry
    if 'hunger' in character_state and 'hunger' in activity.get('states_addressed', []):
        hunger_val = character_state['hunger']['value'] / 100.0  # Normalize to 0-1
        hunger_bonus = (hunger_val ** 2) * 0.8  # hunger_constant = 0.8
        alignment_score += hunger_bonus
    
    # Fatigue alignment - rest activities get bonus when tired
    if 'fatigue' in character_state and 'fatigue' in activity.get('states_addressed', []):
        fatigue_val = character_state['fatigue']['value'] / 100.0  # Normalize to 0-1
        fatigue_bonus = (fatigue_val ** 2) * 0.6  # fatigue_constant = 0.6
        alignment_score += fatigue_bonus
    
    # Thirst alignment - drinking activities get bonus when thirsty
    if 'thirst' in character_state and 'thirst' in activity.get('states_addressed', []):
        thirst_val = character_state['thirst']['value'] / 100.0  # Normalize to 0-1
        thirst_bonus = (thirst_val ** 2) * 0.6  # thirst_constant = 0.6
        alignment_score += thirst_bonus
    
    # Future: injury, etc.
    # if 'injury' in character_state and 'injury' in activity.get('states_addressed', []):
    #     injury_val = character_state['injury']['value'] / 100.0
    #     injury_bonus = (injury_val ** 2) * 1.0  # injury_constant = 1.0 (most urgent)
    #     alignment_score += injury_bonus
    
    return min(alignment_score, 1.0)  # Cap at 1.0


def get_available_states(character_state: Dict[str, Dict[str, float]]) -> list:
    """Get list of available state names from character state"""
    if not character_state:
        return []
    return list(character_state.keys())


