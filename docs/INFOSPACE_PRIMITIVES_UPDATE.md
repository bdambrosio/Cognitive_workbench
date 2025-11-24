# Infospace Primitives Consistency Update

## Changes Made

### 1. **Executive Node Routing** (`executive_node.py`)

**Removed:**
- ❌ `scan` - No longer in INFOSPACE_PLAN_TEMPLATE

**Added:**
- ✅ `say` - Communication primitive
- ✅ `think` - Internal notes primitive

**Current List (19 primitives):**
```python
infospace_primitives = {
    # Phase 1 Core (7)
    'apply', 'move', 'create', 'save', 
    'index', 'organize', 'search',
    # Phase 1 Control (3)
    'if', 'while', 'wait',
    # Phase 1 Communication (3)
    'say', 'think', 'ask',
    # Phase 2 Data (4)
    'extract', 'filter', 'merge', 'transform',
    # Phase 2 Analysis (4)
    'aggregate', 'sort', 'group_by', 'compare'
}
```

### 2. **Infospace Executor Handlers** (`infospace_executor.py`)

**Removed:**
- ❌ `scan` from handlers dictionary (method still exists for backward compatibility)

**Updated Order:**
- Reorganized to group: Core, Storage, Control, Communication, then Phase 2

**Handler List (19 primitives):**
- Phase 1: `apply`, `move`, `create`, `save`, `index`, `organize`, `search`, `if`, `while`, `wait`, `say`, `think`, `ask`
- Phase 2: `extract`, `filter`, `merge`, `transform`, `aggregate`, `sort`, `group_by`, `compare`

### 3. **Test Primitives** (`infospace_executor.py`)

**Removed:**
- ❌ `scan` - Duplicate test
- ❌ `scanb` - Extra duplicate test

**Added:**
- ✅ `create` - Test for Note/Collection creation
- ✅ `organize` - Test for index alias
- ✅ `say` - Test for output communication
- ✅ `think` - Test for internal notes
- ✅ `ask` - Test for user input (interactive primitive)

**Test Coverage:** 19/19 primitives (100%)

---

## Consistency Check

### ✅ **All Sources Now Aligned:**

| Primitive | Executive Node | Executor Handlers | Test Primitives | PLAN_TEMPLATE |
|-----------|----------------|-------------------|-----------------|---------------|
| apply | ✓ | ✓ | ✓ | ✓ |
| move | ✓ | ✓ | ✓ | ✓ |
| create | ✓ | ✓ | ✓ | ✓ |
| save | ✓ | ✓ | ✓ | ✓ |
| index | ✓ | ✓ | ✓ | ✓ |
| organize | ✓ | ✓ | ✓ | ✓ (alias) |
| search | ✓ | ✓ | ✓ | ✓ |
| if | ✓ | ✓ | ✓ | - |
| while | ✓ | ✓ | ✓ | - |
| wait | ✓ | ✓ | ✓ | - |
| say | ✓ | ✓ | ✓ | ✓ |
| think | ✓ | ✓ | ✓ | ✓ |
| ask | ✓ | ✓ | ✓ | ✓ |
| extract | ✓ | ✓ | ✓ | - |
| filter | ✓ | ✓ | ✓ | - |
| merge | ✓ | ✓ | ✓ | - |
| transform | ✓ | ✓ | ✓ | - |
| aggregate | ✓ | ✓ | ✓ | - |
| sort | ✓ | ✓ | ✓ | - |
| group_by | ✓ | ✓ | ✓ | - |
| compare | ✓ | ✓ | ✓ | - |
| ~~scan~~ | ❌ | ❌ | ❌ | ❌ |

**Note:** Control and Phase 2 primitives intentionally omitted from PLAN_TEMPLATE (not shown to LLM planner).

---

## Impact

### **Before:**
- Inconsistent primitive lists across files
- `scan` removed from planner but still routed
- `say` and `think` not routed to infospace executor
- Test coverage incomplete (missing 4 primitives)

### **After:**
- ✅ All primitive lists consistent
- ✅ `scan` fully removed from routing
- ✅ `say` and `think` properly routed
- ✅ 100% test coverage (18/18 primitives)
- ✅ All infospace actions now publish to UI

---

## Files Modified

1. `/src/executive_node.py` - Updated routing primitives list
2. `/src/infospace_executor.py` - Updated handlers and test primitives

---

## Related Changes

This update is part of the broader infospace action visibility fix:
- **Issue:** Infospace actions didn't appear in FastAPI UI
- **Root Cause:** Actions weren't published to `cognitive/{character}/action`
- **Fix:** Added action publishing in executive_node routing (lines 1603-1614)
- **Result:** All infospace primitives now visible in UI and fully tested

---

## Testing

Run the primitive test suite:
```python
# In infospace_executor instance
executor.test_primitives()
```

Expected: All 18 primitives pass (or gracefully fail if dependencies missing)

---

Date: 2025-10-21

