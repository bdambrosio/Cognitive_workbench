"""Human-coexistence policy layer.

ALLOWED_ACTIONS is the only mod surface the bridge will invoke;
RconLink.call refuses anything else. FLE's world-owning actions
(reset, clear_entities, regenerate_resources, game.speed) are simply
absent. create_agent_characters here is the mod's human-safe variant
(spares player-attached characters).

STOP_PHRASE: "Jill, stop" in game chat aborts the current walk from
the bridge itself, below the LLM (locked v1 decision). The literal
phrase match is a deliberate keyword-rule exception, same class as the
voice wake-word: a safety interlock must not depend on an LLM call.
"""

STOP_PHRASE = "jill, stop"

ALLOWED_ACTIONS = frozenset({
    # bridge support
    "agent_position", "stop", "say", "get_chat", "get_alerts",
    "create_agent_characters", "set_inventory",
    # epistemic
    "get_entity", "get_entities", "nearest", "inspect_inventory",
    # action
    "place_entity", "place_entity_next_to", "pickup_entity",
    "rotate_entity", "insert_item", "extract_item", "craft_item",
    "harvest_resource", "connect_entities",
    # walking pipeline
    "request_path", "get_path", "move_to", "clear_walking_queue",
})
