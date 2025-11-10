# Tool Argument Naming Convention Analysis

## Standard Convention

**Primitives that operate on existing data use `target`:**
- `transform`, `flatten`, `expand`, `map`, `add` → all use `target` for the main input
- `persist` → uses `target` 
- `index`, `search` → use `source` (semantically equivalent to `target`)

**Primitives that create new resources use `value`:**
- `createNote`, `createCollection` → use `value` for the initial content

**Primitives with dual purpose:**
- `say`, `display` → use `target` (recipient) and `value` (content)
- `add` → uses `target` (collection) and `value` (item to add)

## Current Tool Usage

### Tools Using `value` (Standard for Direct Tool Invocation):
✅ **All tools invoked directly as action types use `value`:**
- `transform-note` - uses `value`
- `test-note` - uses `value` (when invoked directly)
- `web-search` - uses `value` (via args.query, but value is main input)
- `summarize-content` - uses `value`
- `compare-notes` - uses `value`
- `as-json` - uses `value`
- `as-markdown` - uses `value`
- `format` - uses `value`
- `edit-text` - uses `value`
- `is-empty` - uses `value`
- `text-find` - uses `value`
- `contains-pattern` - uses `value`
- `word-count` - uses `value`
- `is-positive` - uses `value`
- `filter-collection` - uses `value` (Collection input, Collection output)
- `is-question` - uses `value`
- `extract-entities` - uses `value`

### Special Case: `tool_condition` Uses `target`:
⚠️ **`tool_condition` is different** - it uses `target`:
```json
{"type":"tool_condition","tool":"test-note","target":"$paper","args":{"predicate":"..."}}
```

## Analysis

### Why the Inconsistency?

1. **Direct tool invocation** (lines 1512-1524 in `executive_node.py`):
   - Tools invoked as `{"type":"transform-note","value":"$data",...}` 
   - Get converted to `{"type":"apply","target":"transform-note","value":"$data",...}`
   - So `value` is preserved through the conversion

2. **Tool condition** (line 1545 in `infospace_executor.py`):
   - Uses `target` because it's a condition, not a direct action
   - Conditions consistently use `target` across the board

3. **Template documentation** (line 126 in `infospace_planner.py`):
   - Explicitly states: `{"type":"tool-name","value":"input text or $data",...}`
   - But also shows primitives use `target`: `{"type":"transform","target":"$data",...}`

## Recommendation

**The inconsistency is problematic because:**

1. **Semantic clarity**: Tools like `transform-note` operate on existing Notes (similar to `transform` primitive), so they should use `target` for consistency
2. **Cognitive load**: Having two different conventions (`target` for primitives, `value` for tools) is confusing
3. **Future-proofing**: If tools are ever used in `map` operations, they'd need to match the primitive pattern

**However, there's a counter-argument:**

1. **`apply` primitive**: Tools via `apply` use `target` (for tool name) and `value` (for input), so direct invocation preserving `value` maintains consistency with `apply`
2. **Template already established**: The template explicitly documents `value` for tools
3. **Breaking change**: Changing would require updating all tool documentation and potentially breaking existing plans

## Non-Conformists Summary

**Tools that should arguably use `target` instead of `value`** (to match primitives that operate on existing data):

1. ✅ `transform-note` - operates on existing Note (like `transform` primitive)
2. ✅ `test-note` - operates on existing Note (like conditions)
3. ✅ `summarize-content` - operates on existing content
4. ✅ `compare-notes` - operates on existing Notes
5. ✅ `as-json` - operates on existing JSON Note
6. ✅ `as-markdown` - operates on existing markdown Note
7. ✅ `format` - operates on existing text
8. ✅ `edit-text` - operates on existing text
9. ✅ `is-empty` - operates on existing Note
10. ✅ `text-find` - operates on existing text
11. ✅ `contains-pattern` - operates on existing text
12. ✅ `word-count` - operates on existing text
13. ✅ `is-positive` - operates on existing value
14. ✅ `filter-collection` - operates on existing Collection, returns new Collection
15. ✅ `is-question` - operates on existing text
16. ✅ `extract-entities` - operates on existing content

**Tools that legitimately use `value`** (creating new content or searching):

1. ✅ `web-search` - `value` is the query (creates new search results)
2. ✅ `createNote`/`createCollection` - these are primitives, already correct

**Special case:**
- `tool_condition` already uses `target` correctly ✅

## Conclusion

**Yes, there is an inconsistency.** Tools that operate on existing data should use `target` to match the primitive convention (`transform`, `flatten`, `expand`, `map`, `add` all use `target`).

However, the system currently works because:
- Direct tool invocation gets converted to `apply` format internally
- `apply` uses `target` for tool name and `value` for input
- The template documents `value` as the standard for tools

**The question is**: Should we standardize on `target` for tools that operate on existing data to match primitives, or keep `value` as the tool standard?

My recommendation: **Standardize on `target`** for tools that operate on existing Notes/Collections, to match primitives. This would improve consistency and reduce cognitive load.

