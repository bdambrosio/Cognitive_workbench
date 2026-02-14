# Conversation-Aware Goal Wrapper

## Design Spec — Draft for Review

### Problem

When an agent receives a `say` from another agent or from the user, the message is currently wrapped in a minimal goal:

```
"Respond to {source}: {content}"
```

This strips all conversational context. The planner sees every incoming message as an equivalent task — a standalone request requiring a "response." It has no sense of:

- Where we are in a conversation (opening / exploring / closing)
- What kind of speech act is being performed (invitation to debate, factual query, emotional bid, farewell)
- What the correspondent likely wants from us
- Whether we even want to continue

The result: agents treat philosophical invitations as information-retrieval tasks, echo content instead of engaging with it, and loop indefinitely because there's no arc awareness.

### Core Proposal

use the existing envisioning pattern: let the LLM's priors decide what kind of conversational move this moment calls for. The LLM already knows what good conversations look like.

### Architecture

The conversation wrapper sits **outside** the planner, in the goal-construction path within `executive_node.py`. It has two components:

1. **ConversationTracker** — lightweight state object per correspondent
2. **Conversation Envision** — an LLM call that produces context for the goal wrapper

The output is a goal with a `###CONTEXT###` suffix that gives the planner conversational awareness without prescribing content.

---

## 1. ConversationTracker

A lightweight state object, one per active correspondent (lives alongside or within `EntityModel`).

### State

```python
class ConversationTracker:
    """Tracks dialog-level state for a single correspondent."""
    
    def __init__(self, correspondent_name: str):
        self.correspondent = correspondent_name
        self.turn_count = 0           # Turns in current dialog
        self.my_turn_count = 0        # My turns specifically
        self.their_turn_count = 0     # Their turns specifically
        self.last_speaker = None      # Who spoke last
        self.dialog_started = None    # Timestamp of first turn
        self.last_turn_time = None    # Timestamp of most recent turn
        self.phase = 'opening'        # opening | exploring | closing | closed
```

### Phase Transitions

Phase is determined by simple heuristics, NOT by LLM:

- **opening** → **exploring**: after 2+ exchanges (turn_count >= 4, meaning at least 2 from each side)
- **exploring** → **closing**: when `natural_dialog_end()` returns True (this already exists in EntityModel), OR when turn_count exceeds a soft ceiling (e.g., 20 turns)
- **closing** → **closed**: after the next turn post-closing detection
- Any phase → **opening**: on new dialog (after `close_dialog()`)

### Update

Called every time a conversation entry is added (hook into `EntityModel.add_conversation_entry`):

```python
def record_turn(self, source: str):
    self.turn_count += 1
    if source == self.correspondent:
        self.their_turn_count += 1
    else:
        self.my_turn_count += 1
    self.last_speaker = source
    self.last_turn_time = datetime.now()
    if self.dialog_started is None:
        self.dialog_started = datetime.now()
    self._update_phase()
```

### Integration Point

`ConversationTracker` lives on `EntityModel` (one tracker per entity, created lazily). It gets updated in `add_conversation_entry()`.

---

## 2. Conversation Envision

When a `say` arrives and we need to construct a goal, we first call a lightweight LLM envision to characterize the conversational moment. This replaces the generic "Respond to X" framing.

### Input

```
CONVERSATION PHASE: {tracker.phase} (turn {tracker.turn_count})

RECENT DIALOG (last 3-5 turns):
{formatted_recent_turns}

INCOMING MESSAGE from {source}:
{message_content}

DISCOURSE STATE (if available):
{entity.discourse_state}

YOUR CHARACTER (brief):
{character_name} — {one_line_drive_summary}
```

### Prompt

```
Given this conversational moment, provide two things:

1. TURN_INTENT: What kind of conversational move is the incoming message making? 
   (e.g., "opening a philosophical discussion", "asking a factual question", 
   "pushing back on my last claim", "wrapping up", "changing topic",
   "expressing emotion", "making a request")

2. MY_MOVE: What kind of conversational move would be natural and good for me 
   to make in response? Consider my character, the conversation phase, and what 
   would make this a genuine exchange rather than a task completion.
   (e.g., "engage with their specific claim about X and push back",
   "share a concrete example that extends their point",
   "ask a clarifying question before responding substantively",
   "acknowledge and begin wrapping up",
   "this is a simple query — just answer it")

Be concise. 1-2 sentences each.
```

### Output

Two short strings: `turn_intent` and `my_move`.

### Cost Control

This is a **small, fast** LLM call:
- `max_tokens=128`
- `temperature=0.3`
- Uses the same `llm_generate` wrapper everything else uses
- Total added latency: one inference call per incoming message (comparable to existing `natural_dialog_end` check)

Skip the envision for trivial cases:
- Phase is `opening` and turn_count == 0 → this is a brand new conversation, just use "Engage with {source}"
- Source is `User` and message starts with `goal:` → already handled as explicit goal, not a conversation

---

## 3. Goal Construction

Currently in `executive_node.py`, agent-to-agent messages produce:

```python
self.parse_and_set_goal("", f"Respond to {source}: {goal}")
```

Replace with:

```python
# Get or create conversation tracker
entity = self.memory.get_or_create_entity(source)
tracker = entity.get_conversation_tracker()  # Lazy init
tracker.record_turn(source)

# Envision the conversational moment
envision = self._envision_conversation_turn(
    tracker=tracker,
    entity=entity,
    source=source,
    message=clean_input
)

# Construct goal with conversational context
goal_text = (
    f"Continue dialog with {source} (turn {tracker.turn_count}, "
    f"phase: {tracker.phase})"
)

context = (
    f"###CONTEXT###\n"
    f"Their move: {envision['turn_intent']}\n"
    f"Your move: {envision['my_move']}\n"
    f"Phase: {tracker.phase} — turn {tracker.turn_count}\n"
    f"Message: {clean_input[:500]}"
)

self.parse_and_set_goal("", f"{goal_text}\n{context}")
```

### Key Change in Goal Framing

The goal verb shifts from **"Respond"** to **"Continue dialog."** This is not cosmetic — it tells the planner this is a turn in an ongoing exchange, not a task to complete. Combined with the `###CONTEXT###` carrying `my_move`, the planner's envisioning step has what it needs to generate appropriate quality criteria for a *conversational* artifact rather than an *information retrieval* artifact.

---

## 4. What This Changes Downstream

### Planner's `_generate_vision`

No changes needed. The vision prompt already takes the goal text as input. With a goal like:

> "Continue dialog with Jill (turn 4, phase: exploring)"
> CONTEXT: Their move: pushing back on my claim about formalism. My move: engage with their specific objection and offer a concrete counterexample.

...the vision generator will naturally produce criteria like:
- "Response addresses Jill's specific objection rather than restating own position"
- "Includes at least one concrete example"
- "Advances the conversation rather than summarizing"

Instead of the current generic criteria that trigger unsatisfiable search loops.

### Planner's `_preplan`

Similarly, the preplan will see "Continue dialog" + conversational context and produce a strategy like "load Jill's last message, reflect on the objection, generate a response that engages with her specific point" rather than "search-web → filter → extract → synthesize."

### Say Action

No changes to `_execute_say` itself. The say action just delivers the message. The improvement is in what gets *composed* to say.

---

## 5. Conversation Arc Enforcement

The `phase` field provides natural loop prevention:

- **Opening** (turns 0-3): Agents are establishing the conversation. Vision criteria should be loose. No deep research expected.
- **Exploring** (turns 4-12): The substantive middle. This is where depth, pushback, and genuine engagement matter.
- **Closing** (turns 12+, or when `natural_dialog_end` fires): Wrap up. Vision criteria should favor brevity and closure over depth.

If `turn_count` exceeds a hard ceiling (e.g., 20), the goal wrapper should include in context:

> "This conversation has been going for {turn_count} turns. Consider wrapping up unless there's a compelling reason to continue."

This gives the planner a soft nudge toward closure without hard-coding it.
Be careful not to encode hard limits. Some conversations can be just two turns ("are you ok?"; "yup"), let the llm decide.

---

## 6. Implementation Plan

### Phase 1: ConversationTracker (minimal)

1. Add `ConversationTracker` class (new file or in `entity_model.py`)
2. Add `conversation_tracker` attribute to `EntityModel`, lazily initialized
3. Hook `record_turn()` into `add_conversation_entry()`
4. Verify phase transitions work with existing `natural_dialog_end()`

### Phase 2: Conversation Envision

1. Add `_envision_conversation_turn()` method to `ZenohExecutiveNode`
2. Wire it into agent-to-agent message handling (the `if source and source not in ('unknown', 'console')` block)
3. Format output as `###CONTEXT###` suffix on goal

### Phase 3: Goal Framing

1. Change goal text from "Respond to X" to "Continue dialog with X"
2. Include phase and turn count in goal text
3. Add turn-ceiling soft-close nudge

### Phase 4: Validate

1. Re-run Jack/Jill AI-in-mathematics scenario
2. Verify: Jack's opener triggers `opening` phase with loose envision
3. Verify: Jill's response triggers `exploring` phase with substantive envision
4. Verify: No infinite loops (phase eventually reaches `closing`)
5. Verify: Planner vision criteria reflect conversational intent, not information-retrieval

---

## 7. What This Does NOT Do

- **Does not model speech acts explicitly.** No taxonomy of illocutionary force. The LLM priors handle this implicitly through the envision.
- **Does not require changes to the planner or vision system.** Everything flows through existing `###CONTEXT###` and goal-text mechanisms.
- **Does not track cross-conversation history.** Each dialog is independent (resets on `close_dialog`). Cross-dialog memory lives in discourse state and ToM as before.
- **Does not hard-code conversational strategies.** The envision is generative — it produces natural-language guidance, not structured commands.

---

## 8. Open Questions

1. **User conversations:** Should this also apply to User→Agent messages? Yes, use same path.

2. **Multi-party conversations:** If Jack and Jill are both talking to the User, each has its own tracker. But what about Jack→Jill→Jack three-way? Current architecture is pairwise, which seems sufficient for now. Agreed.

3. **Envision caching:** If an agent receives multiple messages in quick succession (e.g., during the loop scenario), should we skip envision for rapid-fire messages? No. For now KISS, no special casing.

4. **Integration with discourse tracker:** The existing `DiscourseTracker` (in `discourse.py`) already tracks commitments, agreements, and obligations. The conversation envision could consume `discourse_state` as input (included in spec above). phase transitions should also consult discourse state E.g., if all commitments are fulfilled and no issues are unresolved, that's a closing hint.

5. **Vision criteria for conversations vs. artifacts:** Should there be a flag that tells the vision generator "this is a conversational turn, not a document"? The goal text framing may be sufficient, but if the vision generator still produces artifact-style criteria ("minimum 3 sources cited"), we may need to add a hint. - This should be clear from the goal, as above: "The goal verb shifts from **"Respond"** to **"Continue dialog."** "