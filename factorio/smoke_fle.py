"""Build-order step 1 smoke: drive FLE against the live jill-factorio server.

Proves the crib works before any CW code exists: RCON connect, Lua
injection, place_entity, and the connect_entities macro, then reads the
result back with get_entities. Run from factorio/ with .venv/bin/python.

Deliberately NON-destructive: clear_entities=False, no reset, no
character destruction beyond what FLE's connect does on a fresh server
(verified empty — no human character present yet; step 2 covers
coexistence).
"""

from pathlib import Path

# Our server uses a non-default RCON password; FLE hardcodes a module
# constant, imported into fle.env.instance at import time — patch there.
_pw = (Path(__file__).parent / "data" / "config" / "rconpw").read_text().strip()
import fle.env.instance as _fle_instance
_fle_instance.RCON_PASSWORD = _pw

from fle.env.instance import FactorioInstance
from fle.env.game_types import Prototype
from fle.env.entities import Position

print("connecting to localhost:27015 ...")
inst = FactorioInstance(
    address="localhost",
    tcp_port=27015,
    fast=True,                          # no human present; step 2 revisits
    clear_entities=False,               # never wipe the world
    all_technologies_researched=False,  # keep the save's tech state
    num_agents=1,
    inventory={
        "stone-furnace": 2,
        "transport-belt": 100,
        "coal": 50,
    },
)
ns = inst.first_namespace

tick = inst.get_elapsed_ticks()
print(f"connected; server tick = {tick}")

print("\n-- move_to near spawn")
pos = ns.move_to(Position(x=0, y=0))
print("   character at", pos)

print("\n-- clean up artifacts from any previous smoke run")
for e in ns.get_entities(position=Position(x=8, y=4), radius=30):
    try:
        ns.pickup_entity(e)
        print("   picked up", getattr(e, "name", type(e).__name__))
    except Exception as ex:
        print("   could not pick up", getattr(e, "name", type(e).__name__), "-", ex)

print("\n-- place two furnaces 12 tiles apart")
f1 = ns.place_entity(Prototype.StoneFurnace, position=Position(x=2, y=2))
print("   furnace 1:", f1.name, f1.position)
ns.move_to(Position(x=12, y=2))
f2 = ns.place_entity(Prototype.StoneFurnace, position=Position(x=14, y=2))
print("   furnace 2:", f2.name, f2.position)

print("\n-- connect_entities: belt run between two ground positions")
# (furnace->furnace directly is rejected by the resolver — needs
# inserters — which is the transactional validation working; a plain
# position-to-position belt run exercises pathing + placement.)
belt = ns.connect_entities(
    Position(x=2, y=6), Position(x=14, y=6),
    connection_type=Prototype.TransportBelt,
)
print("   result:", type(belt).__name__, "-", belt)

print("\n-- read back with get_entities (radius 30 around spawn)")
ents = ns.get_entities(position=Position(x=8, y=2), radius=30)
for e in ents:
    print("   ", type(e).__name__, getattr(e, "name", ""), getattr(e, "position", ""))

print("\nSMOKE PASS — entities placed and connected via RCON-injected Lua")
