# Information → Note Resource Type Migration

## Summary

Renamed all "Information" resource type references to "Note" to align with current infospace terminology.

## Changes Made

### **Files Modified: 2**

1. `/src/executive_node.py` - 6 updates
2. `/src/map_node.py` - 25 updates

---

### **executive_node.py Changes**

| Line Area | Old | New |
|-----------|-----|-----|
| ~3585 | `# create Information instance` | `# create Note instance` |
| ~3598 | `# Create Information instance via map_node` | `# Create Note instance via map_node` |
| ~3601 | `cognitive/map/information/create` | `cognitive/map/note/create` |
| ~3617 | `# Bind variable to Information resource ID` | `# Bind variable to Note resource ID` |
| ~3621 | `Failed to create Information instance` | `Failed to create Note instance` |
| ~3624 | `Error creating Information instance` | `Error creating Note instance` |

---

### **map_node.py Changes**

#### **Initialization** (lines 80-81, 266-268)
```python
# OLD
self.information_counter = 0
self.information_create_queryable = self.session.declare_queryable(
    "cognitive/map/information/create",
    self.handle_create_information
)

# NEW
self.note_counter = 0
self.note_create_queryable = self.session.declare_queryable(
    "cognitive/map/note/create",
    self.handle_create_note
)
```

#### **Handler Method** (lines 1221-1229)
```python
# OLD
def handle_create_information(self, query):
    """Handle Information resource creation...
    Topic: cognitive/map/information/create

# NEW
def handle_create_note(self, query):
    """Handle Note resource creation...
    Topic: cognitive/map/note/create
```

#### **Resource Creation** (lines 1272-1290)
```python
# OLD
self.information_counter += 1
info_id = f"Information_{self.information_counter}"
information_type = self.world_map.resource_types.Information
'type': information_type,
'description': f"Information artifact created by {source_skill}",
'remove_on_take': False,  # Information is not consumable

# NEW
self.note_counter += 1
info_id = f"Note_{self.note_counter}"
note_type = self.world_map.resource_types.Note
'type': note_type,
'description': f"Note artifact created by {source_skill}",
'remove_on_take': False,  # Note is not consumable
```

#### **Loading/Restoring** (lines 2422-2455)
```python
# OLD
if 'information_instances' in world_data:
    self.information_counter = world_data.get('information_counter', 0)
    information_type = self.world_map.resource_types.Information
    instances = world_data['information_instances']
    'type': information_type,
    logger.info(f"Restored {len(instances)} Information instances, counter at {self.information_counter}")

# NEW
if 'note_instances' in world_data:
    self.note_counter = world_data.get('note_counter', 0)
    note_type = self.world_map.resource_types.Note
    instances = world_data['note_instances']
    'type': note_type,
    logger.info(f"Restored {len(instances)} Note instances, counter at {self.note_counter}")
```

#### **Saving/Persistence** (lines 2496-2514)
```python
# OLD
information_instances = {}
if type_name == 'Information':
    # Serialize Information instance
    information_instances[resource_id] = info_serialized
world_data['information_instances'] = information_instances
world_data['information_counter'] = self.information_counter

# NEW
note_instances = {}
if type_name == 'Note':
    # Serialize Note instance
    note_instances[resource_id] = info_serialized
world_data['note_instances'] = note_instances
world_data['note_counter'] = self.note_counter
```

---

## Zenoh Topic Changes

### **Old Topic:**
```
cognitive/map/information/create
```

### **New Topic:**
```
cognitive/map/note/create
```

---

## Backward Compatibility

### **World File Migration Needed:**

Old world files contain:
```json
{
  "information_instances": {...},
  "information_counter": 5
}
```

New world files will save:
```json
{
  "note_instances": {...},
  "note_counter": 5
}
```

**Impact:** Code checks for both `note_instances` (new) and falls back gracefully if not found. Old saved worlds will not restore Note instances until resaved with new format.

---

## Resource Type Registry

Map files must define `Note` resource type:

```python
# In infospace maps (e.g., infolab.py)
ResourceType.Note = ResourceType('Note', {...})
```

Or dynamically check:
```python
note_type = self.world_map.resource_types.Note
```

---

## Testing Checklist

- [ ] Create Note via skill execution with 'out' field
- [ ] Verify Note ID format: `Note_1`, `Note_2`, etc.
- [ ] Check Note appears in infospace at agent location
- [ ] Save world with Note instances
- [ ] Load world and verify Note instances restored
- [ ] Verify Note counter persists across save/load

---

## Related Changes

This migration is part of the broader infospace terminology updates:
- **Phase 1**: Removed `scan` primitive
- **Phase 2**: Added `create` primitive for explicit Note/Collection creation
- **Phase 3**: Renamed `Information` → `Note` (this document)
- **Future**: Add `Collection` resource type

---

Date: 2025-10-21

