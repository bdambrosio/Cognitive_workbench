"""Generate the fle-bridge Factorio mod from the installed fle package.

Why a mod (vs FLE's runtime RCON injection): Lua functions in `storage`
make the world unsaveable and crash the headless server (step-2 finding,
docs/factorio-bridge-architecture.md "Step-2 reversal"). This script
cribs FLE's per-tool server.lua bodies and shared libs into a single
control.lua where functions live in the mod's Lua state (globals
`actions` / `utils`, recreated on every load) and `storage` stays
data-only. Actions are exposed via remote.add_interface("fle_bridge").

Usage:  .venv/bin/python build_mod.py     (writes fle-bridge/)
Then restart the server; docker-compose.yml mounts fle-bridge/ into
/factorio/mods/.

Transforms applied to cribbed sources:
  storage.actions.  ->  actions.   (function registry -> mod global)
  storage.utils.    ->  utils.     (helper registry  -> mod global)
All other storage.* references are data and stay in storage.
checksum.lua is skipped entirely (functions on storage; mods don't
need re-injection checksums).
"""

from pathlib import Path

FLE = Path(__file__).parent / ".venv/lib/python3.13/site-packages/fle/env"
OUT = Path(__file__).parent / "fle-bridge"

MOD_NAME = "fle-bridge"
# Bump on any change that must re-run init_data on an existing save —
# on_configuration_changed only fires when the version changes.
MOD_VERSION = "0.4.6"

# Shared libs, in FLE's own init_scripts order (instance.initialise).
# alerts.lua provides utils.get_issues (used by place_entity's warnings)
# and registers a static on_tick alert scanner — static registration is
# the correct mod pattern, unlike the dynamic on_nth_tick issue above.
LIBS = [
    "lualib_util.lua",
    "utils.lua",
    "alerts.lua",
    "connection_points.lua",
    "recipe_fluid_connection_mappings.lua",
    "serialize.lua",
    "serialize_direction_fix.lua",
]

# Per-file fixes beyond the two global replacements. alerts.lua opens
# with a bare top-level `storage.alerts = {}` (illegal outside events in
# a mod; initialise.lua's guarded version covers it) and defines
# storage.get_alerts as a function directly on storage (save poison).
FILE_FIXES = {
    # lualib_util ends with a chunk-level `return util` (vestigial — util
    # is a global). Inside our do-wrapper that returns from the WHOLE
    # control.lua chunk, silently skipping everything after it (observed:
    # mod active but remote interface never registered).
    "lualib_util.lua": [
        ("\nreturn util", "\n-- (chunk-level `return util` stripped: would end control.lua early)"),
    ],
    "alerts.lua": [
        ("storage.alerts = {}\n", "-- storage.alerts init lives in init_data (initialise.lua)\n"),
        ("storage.get_alerts", "get_alerts"),
    ],
    # avoid_entity teleports the agent character diagonally (+i,+i up to
    # +10,+10, script teleport ignores collision) to test whether SHE is
    # what blocks a path goal — and on success returns WITHOUT restoring
    # her position, leaving her wherever the loop parked her, including
    # inside another entity's collision box. This is the mechanism that
    # embedded the agent in a stone-furnace (observed live 2026-07-17,
    # the "pathfinder island"). Always teleport back: the function's
    # value is its can_place answer; the displacement is a side effect
    # nothing depends on (request_path ignores the return entirely).
    "utils.lua": [
        (
            """        if can_place then
            return true
        end
        player.teleport({player_position.x + i, player_position.y + i})""",
            """        if can_place then
            player.teleport(player_position)
            return true
        end
        player.teleport({player_position.x + i, player_position.y + i})""",
        ),
    ],
    # serialize.lua exposes drop_position for inserters only. Mining
    # drills need it too: it is the ONE tile where the drill ejects ore —
    # exactly where a receiving belt/chest must go. Without it the agent
    # can't see the 2x2 footprint or the output tile and places belts
    # inside the drill (observed live 2026-07-15).
    "serialize.lua": [
        (
            "    -- Add input and output locations if the entity is an inserter\n",
            """    -- CW: expose where a mining drill ejects its output — the one
    -- tile a receiving belt/chest must occupy.
    if entity.type == "mining-drill" and entity.drop_position then
        serialized.drop_position = {
            x = math.round(entity.drop_position.x * 2) / 2,
            y = math.round(entity.drop_position.y * 2) / 2,
        }
    end

    -- Add input and output locations if the entity is an inserter
""",
        ),
    ],
    # move_to: the walking-queue handler was registered top-level behind
    # `if not storage.fast` — top-level storage access crashes a mod at
    # load, and conditional registration breaks save/load determinism.
    # Register statically; gate + pcall inside the handler (an error in
    # a mod event handler is a non-recoverable server crash).
    # 0.4.0: move_to ALWAYS takes the walking-queue branch — real walking
    # at 1x is a v1 requirement (visible presence, architecture note);
    # storage.fast stays true for every other action's semantics
    # (synchronous place/craft/harvest).
    "move_to/server.lua": [
        (
            """if not storage.fast then
    script.on_nth_tick(5, function(event)
        if storage.walking_queues then
            storage.actions.update_walking_queues()
        end
    end)
end""",
            """script.on_nth_tick(5, function(event)
    if not storage.walking_queues or not next(storage.walking_queues) then return end
    local ok, err = pcall(storage.actions.update_walking_queues)
    if not ok then log("fle-bridge walking-queue error: " .. tostring(err)) end
end)""",
        ),
        (
            """    -- If fast mode is disabled, set up walking queue
    if not storage.fast then""",
            """    -- CW: always real walking, regardless of storage.fast
    if true then""",
        ),
    ],
    # inspect_inventory: the non-fast branch registers a dynamic
    # on_nth_tick (the non-recoverable crash class) to auto-close a UI
    # pane no headless agent has. Dead behind storage.fast=true, but
    # stripped outright — one flag flip away from killing the server.
    # place_entity: enforce the entity's placement grid up front with a
    # teaching error. Odd-tile entities (belt, inserter) center on tile
    # centers (coordinates ending in .5); even-tile ones (drill, furnace)
    # on tile corners (integers). A misaligned request collides with
    # neighbours and yields a misleading "blocked by" list — observed
    # 2026-07-15: agent read the drop tile (-54.5, -32.5) correctly, then
    # placed at (-54, -32) and spiralled on phantom ground-item blockage.
    "place_entity/server.lua": [
        (
            "    local position = {x = x, y = y}\n",
            """    local position = {x = x, y = y}

    -- CW: placement-grid guard (see build_mod.py FILE_FIXES)
    local cw_proto = prototypes.entity[entity]
    if cw_proto and cw_proto.tile_width then
        local function cw_axis_err(axis, v, tiles)
            if tiles % 2 == 1 then
                if (v - math.floor(v)) ~= 0.5 then
                    return axis .. " must end in .5 (this entity is " .. tiles ..
                        " tile(s) wide and sits on tile centers; nearest: " ..
                        (math.floor(v) - 0.5) .. " or " .. (math.floor(v) + 0.5) .. ")"
                end
            else
                if v ~= math.floor(v) then
                    return axis .. " must be a whole number (this entity is " .. tiles ..
                        " tiles wide and sits on tile corners; nearest: " ..
                        math.floor(v) .. " or " .. math.ceil(v) .. ")"
                end
            end
            return nil
        end
        local cw_errs = {}
        local ex = cw_axis_err("x", position.x, cw_proto.tile_width)
        local ey = cw_axis_err("y", position.y, cw_proto.tile_height)
        if ex then table.insert(cw_errs, ex) end
        if ey then table.insert(cw_errs, ey) end
        if #cw_errs > 0 then
            error("\\"Cannot place " .. entity .. " at (" .. position.x .. ", " .. position.y ..
                ") - position is off the placement grid: " .. table.concat(cw_errs, "; ") ..
                ". Use the exact coordinates from observation (e.g. an 'outputs to' tile) - do not round.\\"")
        end
    end
""",
        ),
    ],
    # pickup_entity: (1) an item name ("iron-plate") is a valid
    # ground-item pickup but an unknown ENTITY name, and
    # find_entities_filtered ERRORS on unknown entity names before the
    # ground-item scan can run — observed live 2026-07-17: agent could
    # not clear plate debris, bridge said "Unknown entity name".
    # (2) the ground-item loop has a stray `return true` after the
    # first element regardless of match — a wrong-named first item in
    # range silently no-ops as success.
    "pickup_entity/server.lua": [
        (
            """    -- Find both types of entities first
    local player_entities = surface.find_entities_filtered{
        name=entity,
        position=position,
        radius=0.707,
        force="player"
    }""",
            """    -- CW: reject names that are neither entity nor item; scan placed
    -- entities only for real entity prototypes (see build_mod.py)
    if not prototypes.entity[entity] and not prototypes.item[entity] then
        error("\\"Unknown entity or item name: " .. entity .. "\\"")
    end
    local player_entities = {}
    if prototypes.entity[entity] then
        player_entities = surface.find_entities_filtered{
            name=entity,
            position=position,
            radius=0.707,
            force="player"
        }
    end""",
        ),
        (
            """            end
            return true
        end
        return false
    end""",
            """            end
        end
        return false
    end""",
        ),
    ],
    "inspect_inventory/server.lua": [
        (
            """       if not is_fast then
           player.opened = closest_entity
           script.on_nth_tick(60, function()
               if automatic_close == True then
                   if closest_entity and closest_entity.valid then
                       player.opened = nil
                   end
                   automatic_close = False
               end
           end)
       end
""",
            "       -- (non-fast UI auto-close stripped: dynamic on_nth_tick is the crash class)\n",
        ),
    ],
    # request_path: top-level storage guard moved to lazy init inside the
    # action; player-visible game.print noise demoted to the log.
    "request_path/server.lua": [
        (
            """-- Store created entities globally
if not storage.clearance_entities then
    storage.clearance_entities = {}
end""",
            "-- clearance_entities init is lazy (mods cannot touch storage at top level)",
        ),
        (
            "storage.actions.request_path = function(player_index, start_x, start_y, goal_x, goal_y, radius, allow_paths_through_own_entities, entity_size)\n",
            "storage.actions.request_path = function(player_index, start_x, start_y, goal_x, goal_y, radius, allow_paths_through_own_entities, entity_size)\n"
            "    storage.clearance_entities = storage.clearance_entities or {}\n",
        ),
        (
            'game.print("No request data found for ID: " .. event.id)',
            'log("fle-bridge: no path request data for ID: " .. event.id)',
        ),
        # The pathfinder mask must mirror the CHARACTER's own collision
        # layers EXACTLY ({is_object, player, train}) — collision is
        # symmetric mask intersection, so every extra layer creates
        # false walls. FLE's {player, train, water_tile, object} blocks
        # belts twice over: belts carry `object` (but not `is_object` —
        # which is why real characters walk over them) AND `water_tile`
        # (the can't-build-on-water flag every placeable carries).
        # Dropping water_tile loses nothing: water TILES carry `player`,
        # so the character mask already avoids water. Observed live
        # 2026-07-17, twice: agent sealed in a pocket "walled" by her
        # own belts that her body could step straight over.
        (
            """        collision_mask = {
            layers = {
                player = true,
                train = true,
                water_tile = true,
                object = true,
                transport_belt = true
            }
        },""",
            """        collision_mask = {
            -- CW: mirror the character's own collision layers exactly
            -- (see build_mod.py FILE_FIXES) — the extra layers (object,
            -- water_tile) made belts into pathfinder walls; water tiles
            -- carry `player`, so water is still avoided.
            layers = {
                is_object = true,
                player = true,
                train = true
            }
        },""",
        ),
        # A script-walked agent can stop embedded in an entity's collision
        # box (observed 2026-07-17: agent inside a stone-furnace's box).
        # The start then collides at every bounding-box size and all path
        # requests return not_found. Snap the start to the nearest
        # character-sized free spot; move_to's walking queue steps the
        # character out through it.
        (
            "    local start_position = {y = start_y, x = start_x}\n",
            """    local start_position = {y = start_y, x = start_x}
    -- CW: unstick an embedded start (see build_mod.py FILE_FIXES).
    -- Self-collision-aware: entity_prototype_collides at the agent's own
    -- position is ALWAYS true (her body carries the player layer), so
    -- test for a solid (is_object) entity other than herself instead.
    local cw_embedded = false
    for _, cw_e in pairs(surface.find_entities_filtered{
            area = {{start_position.x - 0.3, start_position.y - 0.3},
                    {start_position.x + 0.3, start_position.y + 0.3}}}) do
        if cw_e ~= player and cw_e.valid
                and cw_e.prototype.collision_mask.layers.is_object then
            cw_embedded = true
            break
        end
    end
    if cw_embedded then
        local cw_free = surface.find_non_colliding_position("character", start_position, 3, 0.25)
        if cw_free then start_position = cw_free end
    end
""",
        ),
    ],
}

# Agent actions. place_entity cross-calls get_entity; move_to needs the
# request_path/get_path admin pipeline. connect_entities' Python-side
# resolvers live in the bridge process; only its Lua ships here.
ACTIONS = [
    "get_entity",
    "get_entities",
    "nearest",
    "place_entity",
    "place_entity_next_to",
    "move_to",
    "harvest_resource",
    "insert_item",
    "extract_item",
    "craft_item",
    "pickup_entity",
    "rotate_entity",
    "connect_entities",
    "inspect_inventory",
]

# Admin actions (fle/env/tools/admin/). set_inventory stocks the agent
# character — needed because the mod's storage is isolated from the
# level state, so /sc cannot reach storage.agent_characters directly.
ADMIN_ACTIONS = ["set_inventory", "request_path", "get_path"]

# The marker in initialise.lua separating storage-data guards (must run
# inside an event: on_init/on_configuration_changed) from function
# definitions (must run at every load, i.e. top level).
INIT_SPLIT_MARKER = "-- Factorio 2.0 compatibility"

# Our human-safe character spawn — replaces FLE's admin
# create_agent_characters, which destroys every character on the
# surface including a connected human's (see coexist_fle.py).
SAFE_CREATE_CHARACTERS = """
actions.create_agent_characters = function(num_agents)
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
    return num_agents
end
"""

# Bridge-support actions (ours). agent_position backs /status and walk
# arrival polling (get_entities' name filter does not match the agent
# character); stop backs /act/stop and the hard "Jill, stop" (clearing
# the queue alone leaves walking_state stuck on); get_alerts exposes the
# alerts lib's drain-and-return global through the remote interface.
BRIDGE_SUPPORT = """
actions.agent_position = function(player_index)
    local player = utils.ensure_valid_character(player_index)
    local q = storage.walking_queues and storage.walking_queues[player_index]
    return {
        x = player.position.x,
        y = player.position.y,
        walking = (q and q.current_target) ~= nil,
    }
end

actions.stop = function(player_index)
    actions.clear_walking_queue(player_index)
    local player = utils.ensure_valid_character(player_index)
    player.walking_state = {walking = false}
    return {x = player.position.x, y = player.position.y}
end

actions.get_alerts = function(seconds)
    return get_alerts(seconds or 0)
end

-- CW debug: raw path request with selectable mask variant and NO goal
-- shifting / avoid_entity, to isolate which pipeline stage or mask layer
-- kills a path. Results readable via the normal get_path action.
actions.cw_debug_path = function(sx, sy, gx, gy, size, variant, straight, ignore_self)
    local surface = game.surfaces[1]
    local masks = {
        mirror = {is_object = true, player = true, train = true, water_tile = true},
        noplayer = {is_object = true, train = true, water_tile = true},
        objectful = {object = true, player = true, train = true, water_tile = true},
        watertile = {water_tile = true},
    }
    local half = size / 2 - 0.01
    local flags = {cache = false, no_break = true}
    if straight then flags.prefer_straight_paths = true end
    local ignore = nil
    if ignore_self then ignore = storage.agent_characters[1] end
    local request_id = surface.request_path{
        bounding_box = {{-half, -half}, {half, half}},
        collision_mask = {layers = masks[variant] or masks.mirror},
        start = {x = sx, y = sy},
        goal = {x = gx, y = gy},
        force = game.forces.player,
        radius = 1.0,
        can_open_gates = true,
        pathfind_flags = flags,
        entity_to_ignore = ignore,
    }
    storage.path_requests = storage.path_requests or {}
    storage.paths = storage.paths or {}
    storage.path_requests[request_id] = 1
    return request_id
end
"""

# Game-chat capture (ours). on_console_chat -> data-only ring buffer in
# storage; the bridge polls get_chat(since_seq) (same since-id pattern as
# check-x-feed). say() is the agent's voice: game.print does NOT fire
# on_console_chat, so it is appended to the buffer explicitly (no loop).
CHAT_CAPTURE = """
local CHAT_CAP = 200

local function chat_state()
    storage.chat = storage.chat or {seq = 0, entries = {}}
    return storage.chat
end

local function chat_append(speaker, message)
    local chat = chat_state()
    chat.seq = chat.seq + 1
    table.insert(chat.entries, {seq = chat.seq, tick = game.tick, speaker = speaker, message = message})
    while #chat.entries > CHAT_CAP do table.remove(chat.entries, 1) end
end

script.on_event(defines.events.on_console_chat, function(event)
    -- An error in a mod event handler is a non-recoverable server crash.
    local ok, err = pcall(function()
        if not event.message or event.message == "" then return end
        local speaker = "<server>"
        if event.player_index then
            local p = game.get_player(event.player_index)
            if p then speaker = p.name end
        end
        chat_append(speaker, event.message)
    end)
    if not ok then log("fle-bridge chat handler error: " .. tostring(err)) end
end)

actions.say = function(message)
    message = tostring(message)
    game.print("[color=#55aaff]Jill: " .. message .. "[/color]")
    chat_append("Jill", message)
    return true
end

actions.get_chat = function(since_seq)
    since_seq = since_seq or 0
    local out = {}
    for _, e in ipairs(chat_state().entries) do
        if e.seq > since_seq then table.insert(out, e) end
    end
    return out
end
"""

REMOTE_INTERFACE = """
remote.add_interface("fle_bridge", {
    ping = function() return "pong" end,
    version = function() return "%s" end,
    actions_list = function()
        local out = {}
        for k in pairs(actions) do table.insert(out, k) end
        table.sort(out)
        return out
    end,
    call = function(name, ...)
        local fn = actions[name]
        if not fn then error("fle_bridge: unknown action '" .. tostring(name) .. "'") end
        return fn(...)
    end,
})
""" % MOD_VERSION


def transform(lua: str) -> str:
    return lua.replace("storage.actions.", "actions.").replace("storage.utils.", "utils.")


def block(title: str, body: str) -> str:
    return f"\n-- ==== {title} ====\ndo\n{body}\nend\n"


def main():
    parts = [
        f"-- {MOD_NAME} {MOD_VERSION} — GENERATED by build_mod.py; do not edit by hand.\n"
        "-- Cribbed from fle (MIT, github.com/JackHopkins/factorio-learning-environment)\n"
        "-- with storage.actions/storage.utils moved out of storage (save safety).\n\n"
        "actions = actions or {}\n"
        "utils = utils or {}\n"
    ]

    for lib in LIBS:
        src = (FLE / "mods" / lib).read_text()
        for old, new in FILE_FIXES.get(lib, []):
            if old not in src:
                raise SystemExit(f"{lib}: expected fix target not found: {old!r}")
            src = src.replace(old, new)
        parts.append(block(f"lib: {lib}", transform(src)))

    init_src = transform((FLE / "mods" / "initialise.lua").read_text())
    if INIT_SPLIT_MARKER not in init_src:
        raise SystemExit(f"initialise.lua split marker not found: {INIT_SPLIT_MARKER!r}")
    data_part, fn_part = init_src.split(INIT_SPLIT_MARKER, 1)
    parts.append(block("initialise: function defs (every load)", INIT_SPLIT_MARKER + fn_part))
    parts.append(
        "\n-- ==== initialise: storage data (on_init / on_configuration_changed) ====\n"
        "local function init_data()\n" + data_part +
        "\n    -- CW override: always fast/synchronous actions. The non-fast\n"
        "    -- path registers on_nth_tick closures dynamically; an error in\n"
        "    -- one is a non-recoverable mod error that kills the server\n"
        "    -- (observed live). Walking gets a static queue processor when\n"
        "    -- move_to is ported.\n"
        "    storage.fast = true\n"
        "end\n"
        "script.on_init(init_data)\n"
        "script.on_configuration_changed(init_data)\n"
    )

    def load_action(kind: str, action: str) -> str:
        src = (FLE / "tools" / kind / action / "server.lua").read_text()
        for old, new in FILE_FIXES.get(f"{action}/server.lua", []):
            if old not in src:
                raise SystemExit(f"{action}/server.lua: expected fix target not found: {old!r}")
            src = src.replace(old, new)
        return src

    for action in ACTIONS:
        parts.append(block(f"action: {action}", transform(load_action("agent", action))))

    for action in ADMIN_ACTIONS:
        parts.append(block(f"admin action: {action}", transform(load_action("admin", action))))

    parts.append("\n-- ==== action: create_agent_characters (human-safe, ours) ====\n")
    parts.append(SAFE_CREATE_CHARACTERS)
    parts.append("\n-- ==== game-chat capture: say / get_chat (ours) ====\n")
    parts.append(block("chat capture", CHAT_CAPTURE))
    parts.append("\n-- ==== bridge support: agent_position / stop / get_alerts (ours) ====\n")
    parts.append(block("bridge support", BRIDGE_SUPPORT))
    parts.append("\n-- ==== remote interface ====\n")
    parts.append(REMOTE_INTERFACE)

    OUT.mkdir(exist_ok=True)
    (OUT / "info.json").write_text(
        '{\n'
        f'  "name": "{MOD_NAME}",\n'
        f'  "version": "{MOD_VERSION}",\n'
        '  "title": "FLE Bridge",\n'
        '  "author": "Cognitive Workbench",\n'
        '  "factorio_version": "2.0",\n'
        '  "dependencies": ["base >= 2.0"],\n'
        '  "description": "FLE action vocabulary as remote interfaces for the CW game-embodiment bridge."\n'
        '}\n'
    )
    control = "".join(parts)
    (OUT / "control.lua").write_text(control)
    print(f"wrote {OUT}/info.json and control.lua ({len(control.splitlines())} lines)")


if __name__ == "__main__":
    main()
