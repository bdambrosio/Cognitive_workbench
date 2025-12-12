# Collection Semantics Analysis

## Medium Priority Item 7: Collection Semantics

### The Problem

There is a semantic ambiguity in how "Collections" are treated in the infospace primitives:

**Two conflicting models:**

1. **Collections as Objects** (implied by draft)
   - Draft lines 36-39 suggest Collections are first-class objects
   - "organize - build an embedding index for a Collection"
   - "catalog - add a Note or Collection to a Collection"
   - Implies you can pass a `$collection_variable` to organize/search

2. **Collections as Named Stores** (current implementation)
   - `index` operation takes a `source` (data) and creates a named `store_name`
   - `search` operation queries by `store_name` (string), not a Collection variable
   - Stores are persistent, keyed by string name

### Current Implementation

**What actually happens:**

```json
{"type":"index","target":"$my_data","store_name":"memory_store","index_type":"semantic"}
```

- Takes data from `$my_data` (could be Note or Collection content)
- Creates/updates an indexed store named "memory_store" 
- Store is managed by map_node, not stored as a Collection object

```json
{"type":"search","store_name":"memory_store","query":"find this","out":"results"}
```

- Queries the named store "memory_store" by string name
- Does NOT take a `$collection` variable
- Returns results as a new Note/Collection bound to `$results`

### The Conceptual Gap

**Draft implies:**
```json
{"type":"create","kind":"Collection","out":"my_coll"}
{"type":"organize","target":"$my_coll"}  // organize the Collection
{"type":"search","target":"$my_coll","query":"find"}  // search the Collection
```

**Implementation requires:**
```json
{"type":"create","kind":"Collection","value":[],"out":"my_coll"}
{"type":"index","target":"$my_coll","store_name":"my_coll_store"}
{"type":"search","store_name":"my_coll_store","query":"find","out":"results"}
```

### Recommended Resolution (KISS)

**Keep the current implementation** (named stores) because:

1. **Simpler architecture** - Map node manages stores by name
2. **Persistence** - Named stores naturally persist beyond plan lifecycle
3. **Shared access** - Multiple agents could reference same store name
4. **Already implemented** - No need to refactor map_node

**Clarify documentation** to make explicit:

- Collections are data objects (lists stored in Notes/Collections)
- Index/search operate on **named stores** (string keys)
- The store_name is separate from the variable name
- A Collection object `$my_coll` can be indexed into a store "my_store"

### Updated Semantics

**Type hierarchy:**
```
Info Objects (have info_id):
├── Note (kind='Note')     - scalar data or structured object
└── Collection (kind='Collection')  - list of items

Indexed Stores (have store_name):
└── Named index stores - managed by map_node, queried by string name
```

**Relationship:**
- You can INDEX a Collection (data) TO a named store
- You can SEARCH a named store BY string name
- Collections are data containers, stores are query infrastructure

### Example Workflow

```json
{
  "plan": [
    {"type":"create","kind":"Collection","value":[],"out":"documents"},
    {"type":"apply","target":"web-scrape","value":"https://example.com","out":"doc1"},
    {"type":"save","value":"$doc1","out":"doc1_saved"},
    // Note: catalog/remove not yet implemented, would add to collection
    
    {"type":"index","target":"$documents","store_name":"doc_index","index_type":"semantic"},
    {"type":"search","store_name":"doc_index","query":"find topic","out":"matches"}
  ]
}
```

### Implications

1. **catalog/remove primitives** - When implemented, these would manipulate Collection objects (add/remove items)
2. **organize primitive** - Should be an alias for "index" (already done)
3. **search primitive** - Already correct, uses store_name not Collection variable
4. **Documentation** - Updated to clarify the distinction

### Summary

Collections are **data objects** that can be indexed **into** named stores. Search operates **on** named stores (by string name), not on Collection objects directly. This separation is intentional and follows KISS principles.

