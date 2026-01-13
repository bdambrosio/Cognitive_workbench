"""
test.py — manual world-setup helpers for navigation / path testing

Loaded via MineScript using:
    \test

Then invoked interactively:
    \test_setup("flat")
    \test_setup("step_down")
    \test_setup("cliff")

This file:
- DOES mutate the world
- DOES NOT start threads
- DOES NOT block
- DOES NOT interfere with bridge.py
"""

import minescript as m  # type: ignore


# ------------------------------------------------------------------------------
# Utilities
# ------------------------------------------------------------------------------

def _player_origin():
    """Return integer (x,y,z) at player feet."""
    x, y, z = m.player_position()
    return int(x), int(y), int(z)


def _cmd(s: str):
    """Execute a raw Minecraft command."""
    m.execute(s)


# ------------------------------------------------------------------------------
# Test Scenarios
# ------------------------------------------------------------------------------

def test_setup(scenario: str = "flat"):
    """
    Build a small deterministic navigation test scenario around the player.

    Scenarios:
      - "flat"       : flat walkable plane
      - "step_down"  : 1-block step down ahead
      - "cliff"      : abrupt drop-off
    """

    ox, oy, oz = _player_origin()

    # Clear a small working volume
    _cmd(f"fill {ox-3} {oy-2} {oz-3} {ox+3} {oy+3} {oz+3} air")

    if scenario == "flat":
        _cmd(f"fill {ox-3} {oy-1} {oz-3} {ox+3} {oy-1} {oz+3} minecraft:stone")

    elif scenario == "step_down":
        _cmd(f"fill {ox-3} {oy-1} {oz-3} {ox+3} {oy-1} {oz+3} minecraft:stone")
        _cmd(f"fill {ox+1} {oy-2} {oz-1} {ox+3} {oy-2} {oz+1} minecraft:stone")

    elif scenario == "cliff":
        _cmd(f"fill {ox-3} {oy-1} {oz-3} {ox+3} {oy-1} {oz} minecraft:stone")
    # Forward (+Z) is now empty → true cliff
        # Right side intentionally left empty (void drop)

    else:
        m.chat(f"[test] unknown scenario '{scenario}'")
        return

    m.player_set_orientation(180.0, 0.0)
    m.player_set_position(ox + 0.5, oy, oz + 0.5)
    m.flush()
    # Snap player to canonical start pose
    #_cmd(f"tp @s {ox} {oy} {oz} 0 0")

    m.chat(f"[test] setup complete: {scenario}")


# ------------------------------------------------------------------------------
# Convenience aliases (shorter typing)
# ------------------------------------------------------------------------------

def flat():
    test_setup("flat")


def step():
    test_setup("step_down")


def cliff():
    test_setup("cliff")
