# Primitive Tool Test Examples

This document provides JSON action examples for each primitive tool in the infospace executor. Each example shows the minimal JSON needed to execute the primitive, and includes setup steps where required.

## Core Operations

### apply
Execute a tool (dynamic tool, not a primitive itself, but routes to tool execution).

```json
{"type": "mc-status", "out": "$status"}
```

```json
{"type": "assess", "target": "$content", "predicate": "is this about AI?", "out": "$result"}
```

---

## Storage Operations

### create-note
Create a Note with content.

**Simple text:**
```json
{"type": "create-note", "value": "Hello, world!", "out": "$note1"}
```

**JSON content:**
```json
{"type": "create-note", "value": {"title": "Test", "count": 42}, "out": "$note2"}
```

**With name:**
```json
{"type": "create-note", "value": "Important data", "name": "important-note", "out": "$note3"}
```

**With properties:**
```json
{"type": "create-note", "value": "Data", "properties": {"source": "test", "priority": "high"}, "out": "$note4"}
```

### create-collection
Create a Collection from Notes or variables.

**Empty Collection:**
```json
{"type": "create-collection", "value": [], "out": "$coll1"}
```

**From Note variables:**
```json
{"type": "create-collection", "value": ["$note1", "$note2"], "out": "$coll2"}
```

**From single Collection variable (dereferences):**
```json
{"type": "create-collection", "value": "$coll1", "out": "$coll3"}
```

**With name:**
```json
{"type": "create-collection", "value": ["$note1"], "name": "my-collection", "out": "$coll4"}
```

**Setup:** Requires `$note1`, `$note2` to exist (created via `create-note`).

### persist
Mark a Note or Collection as persistent.

```json
{"type": "persist", "target": "$note1"}
```

```json
{"type": "persist", "target": "$coll1"}
```

**Setup:** Requires `$note1` or `$coll1` to exist.

### load
Load a persistent Note or Collection by ID or name.

```json
{"type": "load", "target": "Note_123", "out": "$loaded_note"}
```

```json
{"type": "load", "target": "my-collection", "out": "$loaded_coll"}
```

**Setup:** Requires a persistent resource with ID `Note_123` or name `my-collection`.

---

## Indexing & Search Operations

### index (or organize)
Create embeddings index for a Collection.

```json
{"type": "index", "target": "$coll1", "index_type": "semantic"}
```

```json
{"type": "organize", "target": "$coll1", "index_type": "keyword"}
```

**Setup:** Requires `$coll1` to be a Collection with Notes.

### search-within-collection
Search within an indexed Collection.

```json
{"type": "search-within-collection", "target": "$coll1", "value": "AI research", "out": "$results", "limit": 5, "threshold": 0.3}
```

**Setup:** Requires `$coll1` to be an indexed Collection (via `index`).

### search-notes
Global search across all Notes.

```json
{"type": "search-notes", "value": "machine learning", "out": "$found_notes", "limit": 10, "threshold": 0.3}
```

### search-collections
Global search across all Collections.

```json
{"type": "search-collections", "value": "research papers", "out": "$found_colls", "limit": 5, "threshold": 0.3}
```

---

## Communication Operations

### say
Produce output (agent speech).

```json
{"type": "say", "value": "Hello, user!"}
```

```json
{"type": "say", "value": "$message", "target": "User"}
```

**Setup:** For variable version, requires `$message` to exist.

### display
Display formatted content to UI action log.

```json
{"type": "display", "value": "This is displayed in the UI"}
```

```json
{"type": "display", "target": "$note1"}
```

**Setup:** For variable version, requires `$note1` to exist.

### think
Internal reasoning (appended to planner context only).

```json
{"type": "think", "value": "I need to analyze this data"}
```

```json
{"type": "think", "value": "$reflection", "out": "$thought"}
```

**Setup:** For variable version, requires `$reflection` to exist.

### ask
Ask user a question and wait for response.

```json
{"type": "ask", "value": "What is your name?", "out": "$response"}
```

```json
{"type": "ask", "value": "Continue?", "target": "User", "out": "$answer"}
```

---

## Data Transformation Operations

### coerce
Convert Note content to different type/format.

**to-string:**
```json
{"type": "coerce", "target": "$number_note", "coercion": "to-string", "out": "$string_note"}
```

**to-int:**
```json
{"type": "coerce", "target": "$string_note", "coercion": "to-int", "out": "$int_note"}
```

**to-float:**
```json
{"type": "coerce", "target": "$string_note", "coercion": "to-float", "out": "$float_note"}
```

**to-bool:**
```json
{"type": "coerce", "target": "$string_note", "coercion": "to-bool", "out": "$bool_note"}
```

**to-json:**
```json
{"type": "coerce", "target": "$json_string_note", "coercion": "to-json", "out": "$parsed_note"}
```

**to-list:**
```json
{"type": "coerce", "target": "$csv_note", "coercion": "to-list", "delimiter": ",", "out": "$list_note"}
```

**Setup:** Requires `$number_note`, `$string_note`, etc. to exist (created via `create-note`).

### map
Apply operation to each item in a Collection.

**Map with primitive:**
```json
{"type": "map", "target": "$coll1", "operation": "coerce", "coercion": "to-string", "out": "$mapped_coll"}
```

**Map with tool:**
```json
{"type": "map", "target": "$coll1", "operation": "assess", "predicate": "is this positive?", "out": "$assessed_coll"}
```

**Map with tool dict:**
```json
{"type": "map", "target": "$coll1", "operation": {"tool": "refine", "instruction": "Summarize this"}, "out": "$refined_coll"}
```

**Map with filter_null:**
```json
{"type": "map", "target": "$coll1", "operation": "some-tool", "filter_null": false, "out": "$mapped_coll"}
```

**Setup:** Requires `$coll1` to be a Collection. For tool operations, the tool must be available.

### flatten
Flatten a Collection into a single Note by concatenating content.

```json
{"type": "flatten", "target": "$coll1", "out": "$flattened_note"}
```

```json
{"type": "flatten", "target": "$coll1", "separator": "\n---\n", "out": "$flattened_note"}
```

**Setup:** Requires `$coll1` to be a Collection.

### split
Split a Note into a Collection of Notes.

**Simple JSON array:**
```json
{"type": "split", "target": "$array_note", "out": "$items"}
```
*Where `$array_note` contains: `[1, 2, 3]`*

**JSON object with array field:**
```json
{"type": "split", "target": "$data_note", "field": "results", "out": "$items"}
```
*Where `$data_note` contains: `{"results": [{"a": 1}, {"a": 2}]}`*

**JSONL format:**
```json
{"type": "split", "target": "$jsonl_note", "out": "$items"}
```
*Where `$jsonl_note` contains: `{"key":"val1"}\n{"key":"val2"}`*

**Plain text by sentences (default):**
```json
{"type": "split", "target": "$text_note", "out": "$sentences"}
```

**Plain text by paragraphs:**
```json
{"type": "split", "target": "$text_note", "delimiter": "paragraph", "out": "$paragraphs"}
```

**Plain text by lines:**
```json
{"type": "split", "target": "$text_note", "delimiter": "line", "out": "$lines"}
```

**Plain text by custom delimiter:**
```json
{"type": "split", "target": "$text_note", "delimiter": "---", "out": "$sections"}
```

**Setup:** Requires `$array_note`, `$data_note`, `$jsonl_note`, or `$text_note` to exist.

---

## Collection Mutation Operations

### add
Add a Note or Collection to an existing Collection (mutates Collection).

**Add single Note:**
```json
{"type": "add", "target": "$coll1", "value": "$note1", "out": "$coll1"}
```

**Add Collection (all items):**
```json
{"type": "add", "target": "$coll1", "value": "$coll2", "out": "$coll1"}
```

**Add literal (creates Note first):**
```json
{"type": "add", "target": "$coll1", "value": "New item", "out": "$coll1"}
```

**Setup:** Requires `$coll1` to be a Collection, and `$note1`/`$coll2` to exist.

### remove
Remove a Note from a Collection (mutates Collection).

```json
{"type": "remove", "target": "$coll1", "value": "$note1", "out": "$coll1"}
```

```json
{"type": "remove", "target": "$coll1", "value": "Note_123", "out": "$coll1"}
```

**Setup:** Requires `$coll1` to be a Collection, and `$note1` or `Note_123` to exist.

### size
Get size (item count) of a Collection.

```json
{"type": "size", "target": "$coll1", "out": "$size_note"}
```

**Setup:** Requires `$coll1` to be a Collection.

---

## Set Operations

### union
Union of two Collections (A ∪ B) - all items, deduplicated.

```json
{"type": "union", "target": "$coll1", "value": "$coll2", "out": "$union_coll"}
```

**Setup:** Requires `$coll1` and `$coll2` to be Collections.

### intersection
Intersection of two Collections (A ∩ B) - items in both.

```json
{"type": "intersection", "target": "$coll1", "value": "$coll2", "out": "$intersection_coll"}
```

**Setup:** Requires `$coll1` and `$coll2` to be Collections.

### difference
Difference of two Collections (A - B) - items in A but not in B.

```json
{"type": "difference", "target": "$coll1", "value": "$coll2", "out": "$difference_coll"}
```

**Setup:** Requires `$coll1` and `$coll2` to be Collections.

---

## SQL-like Collection Operations

### project
Extract specific fields from each Note in a Collection (SQL SELECT fields).

**Single field:**
```json
{"type": "project", "target": "$coll1", "fields": ["title"], "out": "$projected_coll"}
```

**Multiple fields:**
```json
{"type": "project", "target": "$coll1", "fields": ["title", "year", "author"], "out": "$projected_coll"}
```

**Nested fields:**
```json
{"type": "project", "target": "$coll1", "fields": ["metadata.uri", "metadata.score"], "out": "$projected_coll"}
```

**Setup:** Requires `$coll1` to be a Collection with Notes containing JSON objects with the specified fields.

### pluck
Extract a single field value from each Note in a Collection.

```json
{"type": "pluck", "target": "$coll1", "field": "title", "out": "$titles_coll"}
```

```json
{"type": "pluck", "target": "$coll1", "field": "metadata.uri", "out": "$uris_coll"}
```

**Setup:** Requires `$coll1` to be a Collection with Notes containing JSON objects with the specified field.

### head
Take first N items from Collection.

```json
{"type": "head", "target": "$coll1", "count": 5, "out": "$head_coll"}
```

```json
{"type": "head", "target": "$coll1", "out": "$first_item"}
```
*Default count is 1*

**Setup:** Requires `$coll1` to be a Collection.

### filter-structured
Filter Collection based on JSON field predicates (SQL WHERE).

**Simple comparison:**
```json
{"type": "filter-structured", "target": "$coll1", "where": "year > 2020", "out": "$filtered_coll"}
```

**Multiple conditions:**
```json
{"type": "filter-structured", "target": "$coll1", "where": "year >= 2020 AND citations > 100", "out": "$filtered_coll"}
```

**OR conditions:**
```json
{"type": "filter-structured", "target": "$coll1", "where": "venue == \"NeurIPS\" OR venue == \"ICML\"", "out": "$filtered_coll"}
```

**Nested fields:**
```json
{"type": "filter-structured", "target": "$coll1", "where": "metadata.score >= 0.5", "out": "$filtered_coll"}
```

**Setup:** Requires `$coll1` to be a Collection with Notes containing JSON objects with the specified fields.

### sort
Sort Collection by field value (SQL ORDER BY).

**Ascending (default):**
```json
{"type": "sort", "target": "$coll1", "by": "year", "out": "$sorted_coll"}
```

**Descending:**
```json
{"type": "sort", "target": "$coll1", "by": "score", "order": "desc", "out": "$sorted_coll"}
```

**Nested fields:**
```json
{"type": "sort", "target": "$coll1", "by": "metadata.score", "order": "desc", "out": "$sorted_coll"}
```

**Setup:** Requires `$coll1` to be a Collection with Notes containing JSON objects with the specified field.

### join
Join two Collections on a key field (SQL JOIN).

**Inner join (default):**
```json
{"type": "join", "target": "$coll1", "value": "$coll2", "on": "id", "out": "$joined_coll"}
```

**Left join:**
```json
{"type": "join", "target": "$coll1", "value": "$coll2", "on": "id", "join_type": "left", "out": "$joined_coll"}
```

**Different key names:**
```json
{"type": "join", "target": "$coll1", "value": "$coll2", "left_key": "paper_id", "right_key": "id", "out": "$joined_coll"}
```

**Outer join:**
```json
{"type": "join", "target": "$coll1", "value": "$coll2", "on": "id", "join_type": "outer", "out": "$joined_coll"}
```

**Setup:** Requires `$coll1` and `$coll2` to be Collections with Notes containing JSON objects with the specified key fields.

---

## Complete Example Sequences

### Example 1: Create Notes, Build Collection, Search
```json
[
  {"type": "create-note", "value": "First document about AI", "out": "$note1"},
  {"type": "create-note", "value": "Second document about ML", "out": "$note2"},
  {"type": "create-collection", "value": ["$note1", "$note2"], "out": "$coll1"},
  {"type": "index", "target": "$coll1"},
  {"type": "search-within-collection", "target": "$coll1", "value": "AI", "out": "$results", "limit": 5}
]
```

### Example 2: Transform and Filter Collection
```json
[
  {"type": "create-note", "value": [{"title": "Paper 1", "year": 2021}, {"title": "Paper 2", "year": 2023}], "out": "$data"},
  {"type": "split", "target": "$data", "out": "$papers"},
  {"type": "filter-structured", "target": "$papers", "where": "year > 2022", "out": "$recent"},
  {"type": "project", "target": "$recent", "fields": ["title"], "out": "$titles"}
]
```

### Example 3: Combine Collections
```json
[
  {"type": "create-note", "value": "Item A", "out": "$note_a"},
  {"type": "create-note", "value": "Item B", "out": "$note_b"},
  {"type": "create-collection", "value": ["$note_a"], "out": "$coll1"},
  {"type": "create-collection", "value": ["$note_b"], "out": "$coll2"},
  {"type": "union", "target": "$coll1", "value": "$coll2", "out": "$combined"}
]
```

---

## Notes

- All variables must start with `$` (e.g., `$note1`, not `note1`)
- Resource IDs (like `Note_123`, `Collection_456`) are used without `$`
- Named resources (like `"my-collection"`) are used without `$`
- Literal strings, numbers, booleans are used directly without `$`
- The `out` field always requires a variable name starting with `$`
- For primitives that mutate Collections (`add`, `remove`), the `out` variable typically matches the `target` variable
- Search operations return Collections of structured Notes with metadata (source_id, uri, score, type)
- `map` operations can use primitives or tools; tools must be available in the system
- `split` handles multiple formats: JSON arrays, JSON objects with array fields, JSONL, and plain text
- `coerce` supports: `to-string`, `to-int`, `to-float`, `to-bool`, `to-json`, `to-list`
- `filter-structured` supports: `>`, `<`, `>=`, `<=`, `==`, `!=`, `AND`, `OR`
- `join` supports: `inner` (default), `left`, `right`, `outer`
