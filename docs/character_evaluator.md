# Character orientation evaluator

## Purpose

Evaluate an incoming event **before** the character (agent) handles it, to estimate significance relative to the agent's persistent orientation.

V2 uses an LLM call to assess events semantically, replacing V1's keyword/token-overlap heuristics. The LLM emits compact key tokens (~120-150 generated tokens); the module expands them into full structured output for downstream consumption. Falls back to minimal rule-based defaults when no LLM is available.

## Implementation

- Module: [`src/character_evaluator.py`](../src/character_evaluator.py)
- Integration: [`src/executive_node.py`](../src/executive_node.py) — `_character_eval_run`, hooks in `_handle_chat_response`, `_process_text_input`, `sense_data_callback`, and ask-reply path
- Disable via character YAML: `character_evaluator: { enabled: false }`
  Legacy key `jill_evaluator` is still read if `character_evaluator` is absent.
- Tests: [`tests/test_character_evaluator.py`](../tests/test_character_evaluator.py)

## Trigger types

1. **User / dialog** — `user_text` (chat path, agent–agent text, ask reply)
2. **Goal initiation** — `goal_initiation` (commands, scheduler, JSON actions, etc.)
3. **Sensors** — `sensor_event` (alert / trigger / inform)

## Inputs / outputs

The LLM prompt receives: event fields, **character** concerns (seeded `DEFAULT_CHARACTER_CONCERNS` with enriched descriptions), user concerns, compact goals, recent context, activity snapshot.

The LLM returns a compact JSON with: `matters`, per-concern relevance (`concerns` array with `id:level` pairs), `goal_relevance`, `urgency`, `novelty`, `persistence`, `retention`, `action`, `rationale` (one short sentence), and `epistemic` flag.

The module expands this into the full assessment dict matching the V1 downstream contract: `matters`, `retention_evaluation`, `action_evaluation`, `salience_factors`, `overall_rationale`, `notes`.

Console log tag: **`[CHARACTER_EVAL]`**. Log includes `eval_source: llm` or `eval_source: fallback`.

## Orientation-to-chat integration

The evaluator assessment is passed into chat generation via `build_orientation_summary()`, which produces a compact `## ORIENTATION` block injected into the chat prompt. This block includes:

- **Relevant agent concerns** — which character concerns the event touches, at what level
- **User concern relevance** — if any tracked user concerns match
- **Posture** — how to approach the exchange (derived from action + epistemic flags)
- **Epistemic constraint** — when the agent should hedge or verify before asserting
- **Idle orientation hint** — only when the user asks about idle/free-time behavior

This ensures the chat generator's response is consistent with the evaluator's assessment without dumping raw concern state.

## Idle orientation policy

Defined as `IDLE_ORIENTATION_POLICY` in the module. When idle with no urgent demand, the agent tends toward: reviewing unresolved concerns, checking homeostasis, integrating recent exchanges, noticing discrepancies, and considering whether concerns warrant candidate goals. This is directed idle behavior, not simulated leisure.

Surfaced conditionally in the orientation summary when the user asks about idle behavior. Also importable by other modules for future idle-tick evaluation.

## Epistemic flags

The evaluator can flag events requiring epistemic care:

- `status_unverified` — agent status inquiry without recent diagnostics
- `requires_tool` — factual assertion would need tool verification
- `speculative` — response would involve speculation

These are expanded into posture guidance in the orientation summary.

## Constraints (V2)

No action execution, no concern/memory updates, no full tool enumeration in the evaluator itself. Assessment is advisory — it informs chat generation posture but does not gate downstream behavior.
