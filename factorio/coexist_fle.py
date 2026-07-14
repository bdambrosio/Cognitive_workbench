"""Build-order step 2: FLE init alongside a connected human player.

FLE's stock create_agent_characters destroys EVERY character entity on
the surface — including a connected human's — before spawning agent
characters (fle/env/tools/admin/create_agent_characters/server.lua).
HumanSafeInstance redefines that injected Lua function, after
setup_tools has loaded it and before initialise() calls it, to spare
any character attached to a player. It also skips the original's
always_day mutation (don't change a shared world's daylight).

Run with a human connected. Verifies before/after that the human's
character survives with the same unit_number, then performs a visible
FLE action.
"""

from pathlib import Path

_pw = (Path(__file__).parent / "data" / "config" / "rconpw").read_text().strip()
import fle.env.instance as _fle_instance
_fle_instance.RCON_PASSWORD = _pw

from factorio_rcon import RCONClient
from fle.env.instance import FactorioInstance
from fle.env.game_types import Prototype
from fle.env.entities import Position

_SAFE_CREATE_CHARACTERS = r"""
storage.actions.create_agent_characters = function(num_agents)
    for _, entity in pairs(game.surfaces[1].find_entities_filtered{type = "character"}) do
        if entity.player == nil then entity.destroy() end
    end
    if storage.agent_characters then
        for _, char in pairs(storage.agent_characters) do
            if char and char.valid then char.destroy() end
        end
    end
    storage.agent_characters = {}
    for i = 1, num_agents do
        local target = {x = 0, y = 4 + (i - 1) * 2}
        local pos = game.surfaces[1].find_non_colliding_position("character", target, 30, 0.5) or target
        local char = game.surfaces[1].create_entity{
            name = "character", position = pos, force = game.forces.player}
        char.color = {r = 0.2, g = 0.6, b = 1.0, a = 1.0}
        storage.agent_characters[i] = char
    end
    player = storage.agent_characters[1]
end
"""


class HumanSafeInstance(FactorioInstance):
    """FactorioInstance whose init never destroys player-attached characters."""

    def initialise(self, fast, all_technologies_researched, clear_entities):
        self.rcon_client.send_command(
            "/silent-command " + " ".join(_SAFE_CREATE_CHARACTERS.split("\n"))
        )
        super().initialise(fast, all_technologies_researched, clear_entities)


def humans(rcon):
    out = rcon.send_command(
        '/silent-command for _,p in pairs(game.connected_players) do '
        'if p.character then rcon.print(p.name.."="..p.character.unit_number) end end'
    )
    return dict(kv.split("=") for kv in out.split("\n")) if out else {}


rcon = RCONClient("localhost", 27015, _pw)
before = humans(rcon)
print("humans before:", before)
if not before:
    raise SystemExit("No human connected — this check needs a player in-game.")

print("\n-- constructing HumanSafeInstance (the stock one would kill the human here)")
inst = HumanSafeInstance(
    address="localhost",
    tcp_port=27015,
    fast=True,                    # teleport-move for the smoke; bridge will walk
    clear_entities=False,
    all_technologies_researched=False,
    num_agents=1,
    inventory={"stone-furnace": 1},
)
ns = inst.first_namespace

after = humans(rcon)
print("humans after: ", after)
assert before == after, f"HUMAN CHARACTER CHANGED: {before} -> {after}"
print("   human character intact (same unit_number)")

print("\n-- FLE action with human connected: place a furnace at (30, -6)")
ns.move_to(Position(x=28, y=-6))
f = ns.place_entity(Prototype.StoneFurnace, position=Position(x=30, y=-6))
print("   placed:", f.name, f.position)
rcon.send_command(
    '/silent-command game.print("[Jill bridge test] agent character placed a furnace at (30,-6) — north of the belt line")'
)

final = humans(rcon)
assert before == final, f"HUMAN CHARACTER CHANGED during action: {before} -> {final}"
print("\nCOEXIST PASS — human character untouched through FLE init and actions")
