# Save/Load Analysis Report

## Executive Summary

The system has **CRITICAL GAPS** in save/load functionality, particularly around character state that you specifically requested:
- ❌ **discourse_state** is NOT saved
- ❌ **tom_model** (Theory of Mind) is NOT saved  
- ❌ **executive_node state** is NOT saved at all (character drives, plans, activities, self_state)
- ✅ **Conversations** ARE saved
- ✅ **Entity models** (partial) ARE saved
- ✅ **Inventory** IS saved

---

## 1. What IS Currently Saved

### Memory Node (`memory_node.py`)
**File:** `data/memory/{CharacterName}_memory.json`

**What's saved:**
```json
{
  "character_name": "Joe",
  "entities": {
    "Samantha": {
      "entity_name": "Samantha",
      "first_seen": "2025-10-11T15:34:54.483660",
      "last_seen": "2025-10-11T15:34:54.483657",
      "dialogs": [...],  // ✅ Conversations ARE saved
      "active": true
    }
  },
  "inventory": {...}  // ✅ Inventory IS saved
}
```

**Lines:** 643-658 in `memory_node.py`
```python
def save_memory(self):
    data = {
        'character_name': self.character_name,
        'entities': {},
        'inventory': self.inventory
    }
    for entity_name, entity in self.entity_models.items():
        data['entities'][entity_name] = entity.to_dict()
```

### Situation Node (`situation_node.py`)
**File:** `data/situation/{CharacterName}_situation.json`

**What's saved:**
- Current location
- Visible characters and resources
- Adjacent entities
- Views

**Lines:** 585-592 in `situation_node.py`

### Map Node (`map_node.py`)
**File:** `data/world/{WorldName}_world.json`

**What's saved:**
- Agent positions
- World name and map file
- Simulation time
- World dimensions

**Lines:** 2141-2173 in `map_node.py`

---

## 2. What IS NOT Saved (CRITICAL GAPS)

### EntityModel (`entity_model.py`) - MAJOR ISSUE

**Lines 254-267 show what `to_dict()` saves:**
```python
def to_dict(self) -> Dict[str, Any]:
    return {
        'entity_name': self.entity_name,
        'first_seen': self.first_seen.isoformat() if self.first_seen else None,
        'last_seen': self.last_seen.isoformat() if self.last_seen else None,
        'dialogs': self.dialogs,  # ✅ This IS saved
        'active': self.active
        # ❌ discourse_state is NOT saved
        # ❌ tom_model is NOT saved
    }
```

**Missing fields (lines 34-35):**
- `self.discourse_state = ""` - Structured discourse analysis (commitments, agreements, issues)
- `self.tom_model = ""` - Theory of Mind model (trust, competence, reliability)

These are **populated during dialog close** (line 87-88):
```python
def close_dialog(self) -> None:
    self.discourse_state = self.discourse.analyze_segment(...)
    self.tom_model = self.discourse.update_tom_from_discourse_segment(...)
```

**IMPACT:** After save/load, all discourse analysis and ToM understanding is LOST.

### Executive Node (`executive_node.py`) - NO SAVE AT ALL

**NO save/load methods exist** (grep found no "save" in entire file)

**Critical state NOT saved (lines 108-293):**
```python
# Plan execution state
self.current_activity = None
self.current_step = None  
self.current_plan = None
self.plan_state = None
self.plan_bindings = {}

# Character physiological state
self.self_state: Dict[str, Dict[str, float]] = initialize_character_states()
# Contains: hunger, thirst, fatigue (0=good, 100=worst)

# Action history
self.action_history = []  # List of ActionRecord instances

# Character configuration
self.drives = self.character_config.get('drives', [])
self.ontology = json.load(...)  # Activity ontology
self.activities = json.load(...)  # Available activities
```

**IMPACT:** After save/load:
- Character starts with fresh/default physiological state (hunger, thirst, fatigue)
- All in-progress plans are lost
- Action history is lost
- Current activities are lost

### Memory Node - Partial Gaps

**NOT saved (lines 76-78):**
```python
self.short_term_memory = []  # NOT persisted
self.chat_memory = []  # NOT persisted  
self.long_term_memory = []  # NOT persisted
```

Only `entity_models` and `inventory` are persisted. The general memory arrays are transient.

---

## 3. Risk Analysis

### HIGH RISK 🔴

1. **Conversation Continuity Broken**
   - **Risk:** tom_model and discourse_state loss means character "forgets" relationship context
   - **Example:** Character trusted someone before save → after load, trust assessment is blank
   - **Affected:** All multi-session conversations
   - **Fix Complexity:** Medium (add 2 fields to EntityModel.to_dict/load_from_dict)

2. **Character State Reset**
   - **Risk:** Physiological states (hunger/thirst/fatigue) reset to defaults
   - **Example:** Character was very hungry (90/100) → after load, back to 0
   - **Affected:** All character continuity
   - **Fix Complexity:** Medium (create executive_node save/load)

3. **Plan Continuity Lost**
   - **Risk:** In-progress plans/activities are abandoned
   - **Example:** Character mid-way through multi-step plan → after load, plan forgotten
   - **Affected:** Long-running scenarios
   - **Fix Complexity:** High (serialize complex plan state)

### MEDIUM RISK 🟡

4. **Short-term Memory Loss**
   - **Risk:** Recent events before last dialog close are lost
   - **Affected:** Contextual awareness
   - **Fix Complexity:** Low (add to memory save)

5. **Action History Loss**
   - **Risk:** Can't query what character did recently
   - **Affected:** Analytics, debugging, scenario analysis
   - **Fix Complexity:** Low (serialize action_history)

### LOW RISK 🟢

6. **Map State Partial**
   - **Risk:** Resource modifications/terrain changes not saved
   - **Note:** Map node has TODO comment acknowledging this (line 2105)
   - **Affected:** Dynamic world state
   - **Fix Complexity:** Medium-High (depends on map complexity)

---

## 4. Ambiguity Analysis

### Design Ambiguities

1. **Discourse State Snapshot Timing**
   - **Ambiguity:** When should discourse_state be updated?
   - **Current:** Only on dialog close (line 87)
   - **Issue:** If crash before close, discourse_state for active dialog is lost
   - **Options:**
     - A) Only save on dialog close (current, lossy but simple)
     - B) Save incrementally after each message (complex, accurate)
     - C) Save on explicit save command even for active dialogs (middle ground)

2. **Executive State Restore Semantics**
   - **Ambiguity:** What happens to in-progress plans on load?
   - **Options:**
     - A) Resume exactly where left off (complex, may be inconsistent with world state)
     - B) Abandon current plan, start fresh (simple, lossy)
     - C) Save plan checkpoints only at step boundaries (middle ground)

3. **Memory Hierarchy**
   - **Ambiguity:** Should short_term_memory be persisted?
   - **Current:** Transient
   - **Rationale:** Unclear if this is intentional or oversight
   - **Trade-off:** Storage size vs. continuity

4. **Inventory vs. Memory Consistency**
   - **Ambiguity:** Inventory saved in memory_node, but used by executive_node
   - **Current:** executive_node has `inventory_cache` (line 222) that's NOT persisted
   - **Risk:** Cache could be stale after load
   - **Solution:** Either persist cache or rebuild on load

### Data Format Ambiguities

5. **Timestamp Handling**
   - **Current:** ISO format strings (good)
   - **Risk:** Timezone handling unclear, datetime.now() vs simulation_time inconsistency

6. **Character Name Canonicalization**
   - **Current:** Uses `.capitalize()` everywhere (line 65 memory_node, line 110 executive_node)
   - **Risk:** Case-sensitive filename systems could cause issues
   - **Mitigation:** Currently consistent, but fragile

---

## 5. Complexity Analysis

### Current System Complexity: **MODERATE**

**Positive:**
- Clean separation of concerns (memory/situation/map nodes)
- JSON serialization (human-readable, debuggable)
- Explicit save callbacks responding to `cognitive/save_all` message
- Load happens automatically on node initialization

**Negative:**
- No transaction semantics (partial save possible if node crashes mid-save)
- No save versioning (future schema changes will break old saves)
- No validation on load (malformed JSON will crash)
- No backup/rollback mechanism

### To Add Complete Save/Load: **HIGH COMPLEXITY**

**Required changes:**

1. **EntityModel** (LOW complexity)
   - Add `discourse_state` and `tom_model` to `to_dict()`
   - Add to `load_from_dict()`
   - **Estimate:** 10 lines of code, 30 minutes

2. **Executive Node** (HIGH complexity)
   - Create save/load methods
   - Serialize: `self_state`, `current_plan`, `plan_state`, `action_history`, `current_activity`
   - Handle plan resumption logic
   - **Estimate:** 200+ lines, 4-8 hours

3. **Memory Node short_term/chat** (LOW complexity)
   - Add arrays to save_memory()
   - Add to load_memory()
   - **Estimate:** 20 lines, 1 hour

4. **Validation & Testing** (MEDIUM complexity)
   - Save/load tests for each node
   - Integration tests for full scenario save/restore
   - Migration tests (old saves → new code)
   - **Estimate:** 4-8 hours

**Total Estimate:** 2-3 days for complete, robust save/load

---

## 6. Recommended Priorities

### P0 - Critical (Do First)
1. **Add discourse_state and tom_model to EntityModel save/load**
   - Required for your stated goal
   - Low risk, low complexity
   - Immediate value

2. **Add executive_node self_state (hunger/thirst/fatigue) save/load**
   - Needed for character continuity
   - Medium complexity
   - High impact

### P1 - High Priority (Do Soon)
3. **Add executive_node action_history save/load**
   - Valuable for debugging/analysis
   - Low complexity

4. **Add short_term_memory, chat_memory save/load**
   - Improves continuity
   - Low complexity

### P2 - Medium Priority (Consider)
5. **Add plan state save/load**
   - Complex, need to decide on resumption semantics
   - Moderate value

6. **Add save versioning**
   - Future-proofs against schema changes
   - Low complexity to add now

### P3 - Low Priority (Nice to Have)
7. **Add save validation**
8. **Add backup/rollback**
9. **Complete map state save**

---

## 7. Quick Fixes (Minimal Changes)

### Fix #1: EntityModel discourse_state/tom_model (15 lines)

**File:** `src/entity_model.py`

**In `to_dict()` (line 261):**
```python
return {
    'entity_name': self.entity_name,
    'first_seen': self.first_seen.isoformat() if self.first_seen else None,
    'last_seen': self.last_seen.isoformat() if self.last_seen else None,
    'dialogs': self.dialogs,
    'active': self.active,
    'discourse_state': self.discourse_state,  # ADD
    'tom_model': self.tom_model  # ADD
}
```

**In `load_from_dict()` (line 305, before return):**
```python
# Load discourse and ToM state
entity.discourse_state = data.get('discourse_state', '')
entity.tom_model = data.get('tom_model', '')
```

### Fix #2: Executive self_state save/load (60 lines)

**File:** `src/executive_node.py`

**Add after `__init__()` (around line 305):**
```python
# State file path
self.state_file = Path(f"data/executive/{character_name}_state.json")
self.state_file.parent.mkdir(parents=True, exist_ok=True)

# Load existing state
self.load_state()

# Subscribe to save commands
self.save_subscriber = self.session.declare_subscriber(
    "cognitive/save_all",
    self.save_callback
)
```

**Add methods:**
```python
def save_state(self):
    """Save executive state to file."""
    data = {
        'character_name': self.character_name,
        'self_state': self.self_state,
        'action_counter': self.action_counter,
        'timestamp': datetime.now().isoformat()
    }
    with open(self.state_file, 'w') as f:
        json.dump(data, f, indent=2)
    logger.info(f'💾 Saved executive state to {self.state_file}')

def load_state(self):
    """Load executive state from file."""
    if self.state_file.exists():
        with open(self.state_file, 'r') as f:
            data = json.load(f)
            self.self_state = data.get('self_state', initialize_character_states())
            self.action_counter = data.get('action_counter', 0)
        logger.info(f'📂 Loaded executive state from {self.state_file}')

def save_callback(self, sample):
    """Handle save command from UI."""
    logger.info(f'💾 {self.character_name} Executive Node received save command')
    self.save_state()
```

---

## 8. Testing Recommendations

### Minimal Test (5 minutes)
1. Start scenario with 2 characters
2. Have them converse for 5+ exchanges
3. Close dialog (generates discourse_state, tom_model)
4. Trigger save
5. Stop all nodes
6. Restart with same scenario
7. Check if discourse_state and tom_model are present

### Comprehensive Test (30 minutes)
1. Load scenario
2. Have character eat (changes hunger), walk (changes fatigue)
3. Have conversation
4. Pick up items (inventory)
5. Save
6. Restart
7. Verify:
   - Hunger/fatigue values preserved
   - Conversation history present
   - Discourse state present
   - ToM model present
   - Inventory present
   - Character position preserved

---

## 9. Migration Concerns

### Backward Compatibility

**Current saves will break if you:**
- Add required fields to to_dict/load_from_dict without defaults
- Change field names or types
- Remove fields

**Safe approach:**
- Use `.get('field', default_value)` in load methods
- Add version field to save files
- Keep old load paths for migration

### Example Migration Code
```python
def load_from_dict(cls, character_name: str, data: Dict[str, Any], ...):
    entity = cls(character_name, data['entity_name'], ...)
    
    # ... existing loads ...
    
    # New fields with defaults for backward compatibility
    entity.discourse_state = data.get('discourse_state', '')
    entity.tom_model = data.get('tom_model', '')
    
    # Detect old format and migrate
    if 'conversation_history' in data and 'dialogs' not in data:
        # Migration logic...
```

---

## 10. Summary Table

| Component | What's Saved | What's Missing | Risk | Fix Complexity |
|-----------|--------------|----------------|------|----------------|
| EntityModel | ✅ dialogs<br>✅ first_seen<br>✅ last_seen<br>✅ active | ❌ discourse_state<br>❌ tom_model | 🔴 HIGH | 🟢 LOW (15 lines) |
| Memory Node | ✅ entity_models<br>✅ inventory | ❌ short_term_memory<br>❌ chat_memory | 🟡 MEDIUM | 🟢 LOW (20 lines) |
| Executive Node | ❌ Nothing | ❌ self_state<br>❌ current_plan<br>❌ action_history<br>❌ current_activity | 🔴 HIGH | 🔴 HIGH (200+ lines) |
| Situation Node | ✅ location<br>✅ visible entities | ✅ Complete | 🟢 LOW | ✅ DONE |
| Map Node | ✅ positions<br>✅ sim time | ⚠️ resource mods<br>⚠️ terrain changes | 🟢 LOW | 🟡 MEDIUM |

---

## Conclusion

**For your stated goal** of continuing conversations and remembering `tom_model`, `discourse_state`, and conversations:

- **Conversations:** ✅ Already saved
- **tom_model:** ❌ NOT saved (HIGH RISK)
- **discourse_state:** ❌ NOT saved (HIGH RISK)

**Minimum viable fix:** Apply Fix #1 (15 lines in entity_model.py)

**Recommended fix:** Apply Fix #1 + Fix #2 (75 lines total) to also preserve character physiological state

**Time estimate:** 1-2 hours for minimal fix, properly tested

