# Collection Spatial Resource Implementation

## Summary

Implemented full spatial resource support for Collections alongside Notes, enabling persistent, indexed Collections in the infospace.

## Rationale

**Why Collections need spatial persistence:**
- Organized/indexed Collections represent valuable work
- May take time and resources to build
- Should persist across plans and sessions
- Enable reuse of structured data

**Example use cases:**
- Research results indexed for semantic search
- Categorized knowledge bases
- Curated datasets
- Multi-step data pipelines

---

## Implementation: Option B (Full Spatial Resource Support)

### **Architecture**

Collections and Notes are now symmetric:

| Aspect | Note | Collection |
|--------|------|------------|
| **Resource Type** | `InfospaceResources.Note` | `InfospaceResources.Collection` |
| **ID Format** | `Note_1`, `Note_2`, ... | `Collection_1`, `Collection_2`, ... |
| **Counter** | `self.note_counter` | `self.collection_counter` |
| **Queryable** | `cognitive/map/note/create` | `cognitive/map/collection/create` |
| **Handler** | `handle_create_note()` | `handle_create_collection()` |
| **Persistence** | `note_instances`, `note_counter` | `collection_instances`, `collection_counter` |
| **Content Type** | Single value (str/dict) | List or dict |
| **Format** | text, json | list, dict |

---

## Changes Made

### **1. map_node.py** (4 areas updated)

#### **A. Initialization** (lines 80-84, 274-278)
```python
# Added counter
self.collection_counter = 0

# Added queryable
self.collection_create_queryable = self.session.declare_queryable(
    "cognitive/map/collection/create",
    self.handle_create_collection
)
```

#### **B. Handler** (lines 1334-1436)
```python
def handle_create_collection(self, query):
    """
    Handle Collection resource creation.
    
    Topic: cognitive/map/collection/create
    Payload: {
        "character_name": str,
        "content": list or dict,
        "format": "list|dict",
        "source_skill": str (optional),
        "source_value": str (optional)
    }
    
    Returns: {
        "success": bool,
        "info_id": str,  # Collection_N
        "location": [x, y]
    }
    """
    # Generate Collection_N ID
    self.collection_counter += 1
    info_id = f"Collection_{self.collection_counter}"
    
    # Get Collection type from registry
    collection_type = self.world_map.resource_types.Collection
    
    # Create spatial resource
    info_data = {
        'name': info_id,
        'type': collection_type,
        'location': agent_location,
        'description': f"Collection artifact...",
        'properties': {
            'content': content,
            'format': format_type,
            'item_count': len(content),
            ...
        }
    }
    
    # Register and place in grid
    self.world_map.resource_registry[info_id] = info_data
    self.world_map.patches[x][y].resources[info_id] = info_data
```

#### **C. Load/Restore** (lines 2572-2607)
```python
# Restore Collection instances
if 'collection_instances' in world_data:
    self.collection_counter = world_data.get('collection_counter', 0)
    collection_type = self.world_map.resource_types.Collection
    
    for info_id, info_data in world_data['collection_instances'].items():
        # Reconstruct and place resource
        ...
    
    logger.info(f"Restored {len(instances)} Collection instances")
```

#### **D. Save/Persist** (lines 2646-2676)
```python
# Save Collection instances
collection_instances = {}
for resource_id, resource_data in registry.items():
    type_name = getattr(resource_data['type'], 'name', ...)
    
    if type_name == 'Collection':
        # Serialize Collection
        collection_instances[resource_id] = {
            'name': ...,
            'location': ...,
            'properties': ...
        }

world_data['collection_instances'] = collection_instances
world_data['collection_counter'] = self.collection_counter
```

---

### **2. executive_node.py** (Apply output detection)

#### **Updated: Skill Output Binding** (lines 3585-3638)

Now detects result type and creates appropriate resource:

```python
# Detect if result is a Collection (list) or Note (single value)
if isinstance(skill_result, list):
    # Collection type
    resource_kind = 'Collection'
    create_topic = "cognitive/map/collection/create"
    format_type = 'list'
elif isinstance(skill_result, dict):
    # Note with JSON
    resource_kind = 'Note'
    create_topic = "cognitive/map/note/create"
    format_type = 'json'
else:
    # Note with text
    resource_kind = 'Note'
    create_topic = "cognitive/map/note/create"
    format_type = 'text'

# Create spatial resource via appropriate topic
self.session.get(create_topic, payload=...)
```

**Result:** Tool outputs are now correctly typed as Note or Collection based on their structure.

---

### **3. infospace_executor.py** (Create primitive)

#### **Updated: _execute_create()** (lines 243-317)

Now creates spatial resources via map_node instead of local-only:

```python
def _execute_create(self, action: Dict) -> Dict:
    """Create Note or Collection as spatial resource."""
    
    # Determine topic based on kind
    if kind == 'Collection':
        create_topic = "cognitive/map/collection/create"
    else:
        create_topic = "cognitive/map/note/create"
    
    # Create spatial resource via map_node
    response = self.session.get(create_topic, payload=...)
    
    if response.success:
        info_id = response.get('info_id')
        self._bind_variable(out_var, info_id)
        return {'status': 'success', 'value': info_id}
    
    # Fallback to local if spatial creation fails
    info_id = self._create_info(content, name, kind)
    return {'status': 'success', 'value': info_id}
```

**Note:** `_create_info()` still exists for fallback and internal use, but `create` primitive now prefers spatial resources.

---

### **4. Test Files** (Documentation updates)

Updated test documentation to list both queryables:
- `tests/test_map_queryables.py` - Listed `collection/create` as destructive
- `tests/test_map_subscribers.py` - Listed `collection/create` as queryable

Updated queryable count: **22 → 24 queryables**

---

## Usage Examples

### **Creating Persistent Collections**

```json
// Explicit create - Collection spatial resource
{"type": "create", "kind": "Collection", "value": [], "out": "my_data"}

// Apply with list result - auto-detected as Collection
{"type": "apply", "target": "web-search", "value": "query", "out": "results"}
// If results = [{...}, {...}] → Creates Collection_1

// Index the Collection for search
{"type": "index", "target": "$results", "store_name": "research", "index_type": "semantic"}
```

### **World File Persistence**

```json
{
  "note_instances": {
    "Note_1": {
      "name": "Note_1",
      "location": [5, 5],
      "properties": {"content": "Hello", "format": "text"}
    }
  },
  "note_counter": 1,
  "collection_instances": {
    "Collection_1": {
      "name": "Collection_1",
      "location": [5, 5],
      "properties": {
        "content": [{"title": "Doc1"}, {"title": "Doc2"}],
        "format": "list",
        "item_count": 2
      }
    }
  },
  "collection_counter": 1
}
```

---

## Spatial Semantics

**Both Notes and Collections:**
- Placed at agent's current location when created
- Visible in agent's perception (via look/scan)
- Persist across sessions (saved/loaded with world state)
- Not consumable (`remove_on_take: false`)
- Tracked in resource_registry
- Appear in map summary

**Differences:**
- Notes store single values/structures
- Collections store lists/arrays
- Collections track `item_count` in properties

---

## Benefits

1. **Persistence:** Organized Collections survive plan completion
2. **Reusability:** Other agents can discover and use Collections
3. **Visibility:** Collections appear in map summary and agent perception
4. **Indexing:** Collections can be organized/indexed for semantic search
5. **Workflow:** Multi-step data pipelines can build up structured datasets

---

## Total Queryable Count

Map_node now has **24 queryables:**
- 22 original queryables
- +1 `cognitive/map/note/create`
- +1 `cognitive/map/collection/create`

Test coverage: 19/24 (79%)

---

## Files Modified

1. `/src/map_node.py` - Added Collection support (handler, persistence)
2. `/src/executive_node.py` - Auto-detect Note vs Collection from skill results
3. `/src/infospace_executor.py` - Create spatial resources for both types
4. `/tests/test_map_queryables.py` - Updated documentation
5. `/tests/test_map_subscribers.py` - Updated documentation

---

Date: 2025-10-21

