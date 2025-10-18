# Launcher and MapNode Integration Analysis

**Date:** 2025-10-18  
**Question:** How should launcher.py and map_node.py determine which map class to use (WorldMap vs InfospaceMap)?

---

## Current Flow

### 1. Launcher.py Startup Sequence

```
launcher.py main()
  ├─> Load YAML config (e.g., scenarios/jill.yaml)
  │   ├─> Extract 'map' parameter (e.g., "forest.py")
  │   ├─> Extract 'characters' list
  │   └─> Extract 'llm_config'
  │
  ├─> launch_shared_services()
  │   └─> Spawn map_node.py with args:
  │       ├─> -m forest.py       (map file)
  │       ├─> -w forest          (world name)
  │       ├─> -s "setting text"  (optional)
  │       ├─> --server vllm      (LLM server)
  │       └─> --model ...        (LLM model)
  │
  └─> launch_character() for each character
      └─> Spawn nodes: memory, situation, perception, agenda, executive
```

### 2. MapNode.py Startup Sequence

```python
# Line 140-156 in map_node.py
maps_dir = os.path.join(os.path.dirname(__file__), 'maps')
map_path = os.path.join(maps_dir, self.map_file)

# Load the module dynamically
spec = importlib.util.spec_from_file_location("map_module", map_path)
map_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(map_module)

# 🚨 HARDCODED: Always creates WorldMap
logger.info("Creating WorldMap instance...")
self.world_map = WorldMap(map_module)
logger.info(f"WorldMap created successfully: {self.world_map.width}x{self.world_map.height}")
```

**Problem:** Line 155 hardcodes `WorldMap` instantiation.

---

## Design Options

### Option 1: map_class Attribute in Map Modules ✅ RECOMMENDED

**Concept:** Each map module explicitly declares its class.

**Implementation:**

```python
# In src/maps/forest.py (WorldMap):
from map import WorldMap
# ... terrain, resources, etc ...
map_class = WorldMap  # Explicit declaration

# In src/maps/infolab.py (InfospaceMap):
from infospace_map import InfospaceMap
# ... infospace setup ...
map_class = InfospaceMap  # Explicit declaration

# In src/infospace.py:
from infospace_map import InfospaceMap
# ... module setup ...
map_class = InfospaceMap  # Explicit declaration
```

**Changes Required:**

1. **map_node.py** (lines 153-156):
```python
# OLD:
logger.info("Creating WorldMap instance...")
self.world_map = WorldMap(map_module)

# NEW:
from map import WorldMap
from infospace_map import InfospaceMap

# Determine map class from module
MapClass = getattr(map_module, 'map_class', WorldMap)  # Default to WorldMap
logger.info(f"Creating {MapClass.__name__} instance...")
self.world_map = MapClass(map_module)
logger.info(f"{MapClass.__name__} created successfully: {self.world_map.width}x{self.world_map.height}")
```

2. **Add to each physical map module** (forest.py, lab.py, rural.py, etc.):
```python
# At the end of the file:
from map import WorldMap
map_class = WorldMap
```

3. **Add to infospace.py**:
```python
# At the end of the file:
from infospace_map import InfospaceMap
map_class = InfospaceMap
```

4. **Already done in infolab.py** (uses infospace module).

**Pros:**
- ✅ No changes to launcher.py
- ✅ Explicit and clear
- ✅ Easy to debug (just check module attribute)
- ✅ Default fallback to WorldMap (backward compatible)
- ✅ Follows Python conventions
- ✅ Already discussed in MAP_NODE_API_ANALYSIS.md

**Cons:**
- ⚠️ Need to update existing map modules (one line each)
- ⚠️ Could forget to add attribute to new maps (falls back to WorldMap)

---

### Option 2: Detect from Module Contents

**Concept:** Inspect the loaded module to detect terrain types.

**Implementation:**

```python
# In map_node.py after loading map_module:
terrain_types = getattr(map_module, 'terrain_types', None)
if terrain_types:
    terrain_names = [t.name for t in terrain_types]
    if terrain_names == ['InfoSpace']:
        from infospace_map import InfospaceMap
        MapClass = InfospaceMap
    else:
        MapClass = WorldMap
else:
    MapClass = WorldMap
```

**Pros:**
- ✅ No changes to existing map modules
- ✅ No changes to launcher.py
- ✅ Automatic detection

**Cons:**
- ❌ Fragile (relies on terrain type names)
- ❌ What if a physical map has only one terrain type?
- ❌ Implicit behavior (harder to debug)
- ❌ Inspection overhead
- ❌ Not extensible to future space types

---

### Option 3: Naming Convention

**Concept:** Files starting with "info" or containing "infospace" use InfospaceMap.

**Implementation:**

```python
# In map_node.py:
if 'infospace' in self.map_file.lower() or self.map_file.startswith('info'):
    from infospace_map import InfospaceMap
    MapClass = InfospaceMap
else:
    MapClass = WorldMap
```

**Pros:**
- ✅ No changes to map modules
- ✅ No changes to launcher.py
- ✅ Simple to implement

**Cons:**
- ❌ Fragile (relies on naming)
- ❌ Constrains file names
- ❌ Not explicit
- ❌ Breaks if someone names a WorldMap file "info_town.py"
- ❌ Poor software engineering practice

---

### Option 4: Add Metadata to YAML

**Concept:** YAML config specifies map type.

**Implementation:**

```yaml
# In scenarios/jill.yaml:
map: forest.py
map_type: physical  # or "infospace"

# In scenarios/sage.yaml:
map: infolab.py
map_type: infospace
```

```python
# In launcher.py:
map_type = config_data.get('map_type', 'physical')
map_args.extend(['--map-type', map_type])

# In map_node.py:
parser.add_argument('--map-type', default='physical', 
                   choices=['physical', 'infospace'])
if args.map_type == 'infospace':
    from infospace_map import InfospaceMap
    MapClass = InfospaceMap
else:
    MapClass = WorldMap
```

**Pros:**
- ✅ Explicit in scenario config
- ✅ Easy to see which scenarios use which map type
- ✅ Could support future types easily

**Cons:**
- ❌ Adds parameter to launcher (user wants to avoid)
- ❌ Duplication (map type in both YAML and map module)
- ❌ Could get out of sync
- ❌ More changes required

---

### Option 5: Try-Except Pattern

**Concept:** Try WorldMap first, fall back to InfospaceMap if error.

**Implementation:**

```python
# In map_node.py:
try:
    from map import WorldMap
    self.world_map = WorldMap(map_module)
    logger.info(f"WorldMap created successfully")
except (AttributeError, ValueError) as e:
    if "Water" in str(e) or "terrain" in str(e).lower():
        logger.info("WorldMap failed, trying InfospaceMap...")
        from infospace_map import InfospaceMap
        self.world_map = InfospaceMap(map_module)
        logger.info(f"InfospaceMap created successfully")
    else:
        raise
```

**Pros:**
- ✅ No changes to map modules
- ✅ No changes to launcher.py
- ✅ Automatic fallback

**Cons:**
- ❌ Relies on error messages (fragile)
- ❌ Poor error handling practice
- ❌ Slow (creates errors)
- ❌ Hard to debug
- ❌ Hides real errors
- ❌ Terrible software engineering

---

## Comparison Matrix

| Criterion | map_class Attr | Detect Contents | Naming Conv | YAML Metadata | Try-Except |
|-----------|---------------|----------------|-------------|---------------|-----------|
| **Explicit** | ✅ Excellent | ❌ Implicit | ⚠️ Indirect | ✅ Excellent | ❌ Hidden |
| **Maintainable** | ✅ Easy | ⚠️ Fragile | ❌ Fragile | ✅ Easy | ❌ Nightmare |
| **Debuggable** | ✅ Clear | ⚠️ Harder | ⚠️ Confusing | ✅ Clear | ❌ Very hard |
| **Extensible** | ✅ Yes | ⚠️ Limited | ❌ No | ✅ Yes | ❌ No |
| **Changes to launcher** | ✅ None | ✅ None | ✅ None | ❌ Required | ✅ None |
| **Changes to maps** | ⚠️ 1 line each | ✅ None | ✅ None | ✅ None | ✅ None |
| **Backward compat** | ✅ Yes | ✅ Yes | ✅ Yes | ⚠️ Depends | ✅ Yes |
| **Performance** | ✅ Fast | ⚠️ Slower | ✅ Fast | ✅ Fast | ❌ Slow |
| **Best practice** | ✅ Yes | ⚠️ Okay | ❌ No | ✅ Yes | ❌ No |

**Score (out of 10):**
1. **map_class attribute**: 9/10
2. YAML metadata: 7/10
3. Detect contents: 5/10
4. Naming convention: 3/10
5. Try-except: 1/10

---

## Recommended Solution: Option 1 (map_class Attribute)

### Why This is Best

1. **Explicit is better than implicit** (Zen of Python)
2. **No changes to launcher.py** (user's preference)
3. **Minimal changes to map modules** (one line each)
4. **Follows existing pattern** (already discussed for map_node in MAP_NODE_API_ANALYSIS.md)
5. **Extensible** (easy to add GraphSpace, SocialSpace, etc.)
6. **Default fallback** (backward compatible with existing maps)

---

## Required Changes

### 1. map_node.py (Lines 25-27, 153-156)

**Add imports at top:**
```python
# Line 25-26:
from map import WorldMap, hash_direction_info, extract_direction_info
from map_agent import MapAgent  # Also update Agent → MapAgent
from infospace_map import InfospaceMap  # NEW
```

**Update map instantiation:**
```python
# Lines 153-156, OLD:
# Create WorldMap instance
logger.info("Creating WorldMap instance...")
self.world_map = WorldMap(map_module)
logger.info(f"WorldMap created successfully: {self.world_map.width}x{self.world_map.height}")

# Lines 153-159, NEW:
# Determine map class from module attribute
MapClass = getattr(map_module, 'map_class', WorldMap)
logger.info(f"Creating {MapClass.__name__} instance...")
self.world_map = MapClass(map_module)
logger.info(f"{MapClass.__name__} created successfully: {self.world_map.width}x{self.world_map.height}")
```

### 2. Add map_class to Existing Physical Maps

**forest.py, lab.py, rural.py, suburban.py, sageForest.py:**
```python
# Add at the end of each file:

# Specify map class for map_node
from map import WorldMap
map_class = WorldMap
```

### 3. Add map_class to infospace.py

**Already should be added (from previous discussion):**
```python
# At end of infospace.py:

# Specify map class for map_node
from infospace_map import InfospaceMap
map_class = InfospaceMap
```

### 4. infolab.py Already Uses infospace Module

No changes needed - it imports infospace which will have map_class.

### 5. launcher.py

**No changes required!** ✅

This was the user's preference.

---

## Implementation Checklist

### Phase 1: Core Integration (30 minutes)

- [ ] Update `map_node.py` line 26: Add `from infospace_map import InfospaceMap`
- [ ] Update `map_node.py` lines 153-159: Dynamic map class selection
- [ ] Add `map_class = WorldMap` to `forest.py`
- [ ] Add `map_class = WorldMap` to `lab.py`
- [ ] Add `map_class = WorldMap` to `rural.py`
- [ ] Add `map_class = WorldMap` to `suburban.py`
- [ ] Add `map_class = WorldMap` to `sageForest.py`
- [ ] Add `map_class = InfospaceMap` to `infospace.py`

### Phase 2: MapAgent Integration (15 minutes)

- [ ] Update `map_node.py` line 26: Change `Agent` to `MapAgent`
- [ ] Update `map_node.py` line 1244: `Agent` → `MapAgent`
- [ ] Update `map_node.py` line 1869: `Agent` → `MapAgent`
- [ ] Update `map_node.py` line 2178: `Agent` → `MapAgent`

### Phase 3: Additional Fixes (30 minutes)

- [ ] Fix spawn location (line 1241): Use `(world_map.width // 2, world_map.height // 2)`
- [ ] Fix line of sight (line 1803): Use `world_map.is_visible()` instead of hardcoded terrain
- [ ] Fix terrain reference (line 1238): Try first available terrain type

### Phase 4: Testing (1-2 hours)

- [ ] Test physical map scenario (e.g., forest.py)
- [ ] Test information space scenario (e.g., infolab.py via infospace)
- [ ] Test agent spawning in both map types
- [ ] Test agent movement in both map types
- [ ] Test visibility queries in both map types
- [ ] Test resource interaction in both map types

---

## Testing Plan

### Test 1: Physical Map (Existing Behavior)

```bash
cd src
python3 launcher.py jill.yaml --map-file forest.py --ui
```

**Expected:**
- ✅ Map Node launches with WorldMap
- ✅ Log shows "Creating WorldMap instance..."
- ✅ Characters spawn and move normally
- ✅ Existing functionality unchanged

### Test 2: Information Space (New Behavior)

First, create a test scenario YAML:

```yaml
# scenarios/infospace_test.yaml
llm_config:
  server_name: 'vllm'
  model_name: 'qwen2.5-coder:7b'

map: infolab.py  # Uses infospace module

setting: |
  You are exploring an information space containing various skills.

characters:
  Explorer:
    name: Explorer
    goal: Discover and learn new skills
```

```bash
cd src
python3 launcher.py infospace_test.yaml --ui
```

**Expected:**
- ✅ Map Node launches with InfospaceMap
- ✅ Log shows "Creating InfospaceMap instance..."
- ✅ Skills load from directory
- ✅ Agent can spawn and move
- ✅ Visibility shows nearby skills

### Test 3: Default Fallback

Create a map module without map_class attribute:

```python
# test_map.py (no map_class)
terrain_types = SomeTerrain
# ... other stuff
```

```bash
cd src
python3 map_node.py -m test_map.py -w test
```

**Expected:**
- ✅ Falls back to WorldMap
- ✅ Log shows "Creating WorldMap instance..."
- ✅ Works with legacy map files

---

## Error Handling

### Scenario 1: Invalid map_class

```python
# In bad_map.py:
map_class = "not a class"  # String instead of class
```

**Current behavior:** Will fail with TypeError when trying to instantiate.

**Improved handling:**
```python
# In map_node.py:
MapClass = getattr(map_module, 'map_class', WorldMap)
if not isinstance(MapClass, type):
    logger.error(f"map_class in {self.map_file} is not a class: {MapClass}")
    logger.warning("Falling back to WorldMap")
    MapClass = WorldMap
```

### Scenario 2: Missing imports

```python
# In bad_map.py:
from nonexistent_module import FakeMap
map_class = FakeMap
```

**Current behavior:** Will fail at import time with ImportError.

**Handling:** Let it fail early with clear error message (no special handling needed).

### Scenario 3: map_class instantiation fails

**Current behavior:** Exception raised during MapClass(map_module).

**Handling:** Existing try-except in map_node.py (lines 161-164) will catch and log.

---

## Future Extensions

With this architecture, adding new map types is trivial:

### Example: GraphSpace

```python
# In src/maps/knowledge_graph.py:
from graph_space import GraphSpace
# ... graph configuration ...
map_class = GraphSpace

# In src/graph_space.py:
class GraphSpace(SpaceMap):
    # ... implementation ...
```

**No changes needed** to launcher.py or map_node.py!

---

## Backward Compatibility

### Existing Maps Without map_class

```python
# Old forest.py (no map_class attribute)
from enum import Enum
# ... terrain types, resources ...
# (no map_class defined)
```

**Behavior:**
```python
MapClass = getattr(map_module, 'map_class', WorldMap)  # Returns WorldMap
```

✅ **Works perfectly** - defaults to WorldMap.

### Existing Scenarios

All existing YAML configurations continue working:
```yaml
map: forest.py  # No map_type needed
```

✅ **No changes required** to existing scenarios.

---

## Documentation Updates Needed

1. **Update SPACEMAP_ARCHITECTURE.md**
   - Document map_class pattern
   - Add to "Creating New Map Types" section

2. **Update MAP_NODE_API_ANALYSIS.md**
   - Mark map_class implementation as complete
   - Update integration status

3. **Create sample scenario YAML**
   - Example information space scenario
   - Show both WorldMap and InfospaceMap usage

4. **Update README**
   - Document map_class attribute requirement for new maps
   - Show examples of both map types

---

## Summary

**Recommended Solution:** Add `map_class` attribute to map module files.

**Why:**
- ✅ No changes to launcher.py (user's preference)
- ✅ Explicit and maintainable
- ✅ Extensible to future map types
- ✅ Backward compatible
- ✅ Minimal changes (1 line per map file)
- ✅ Follows Python best practices

**Implementation Effort:**
- map_node.py: ~10 lines changed
- Existing physical maps: 1 line added each (5 files)
- infospace.py: 1 line added
- **Total:** ~15 lines of code, 30 minutes work

**Risk:** Low - clear fallback behavior, well-defined pattern

**Benefit:** Clean, extensible architecture for multiple map types

---

**End of Analysis**

