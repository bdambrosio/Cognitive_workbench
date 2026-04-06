---
name: obsidian
type: python
description: "Read, search, write, and list files in the Obsidian vault via Local REST API."
schema_hint:
  action: "string (required): search|read|write|list"
  query: "string (for search: search query text)"
  path: "string (for read/write/list: vault-relative path, e.g. 'Clippings/article.md' or 'KnowledgeBases/')"
  content: "string (write): markdown or a reference — $var / Note_* / Collection_* resolved to note body (use for long text)"
  frontmatter: "object (for write, optional): YAML frontmatter fields to prepend"
  out: "$variable"
---

# obsidian

Read, search, write, and list files in the Obsidian vault via the Local REST API.

## Actions

### search — Find notes by query
```json
{"type":"obsidian","action":"search","query":"neural networks","out":"$notes"}
```
Searches by filename first, then content match. Returns a Collection of Notes.

### read — Read a specific file by path
```json
{"type":"obsidian","action":"read","path":"KnowledgeBases/Transformers/_index.md","out":"$page"}
```
Returns a single Note with the file content. Path is relative to vault root.

### write — Create or update a file
```json
{"type":"obsidian","action":"write","path":"KnowledgeBases/Transformers/attention.md","content":"# Attention\n\nContent here...","out":"$result"}
```
Creates the file if it doesn't exist, overwrites if it does. Optional `frontmatter` dict is prepended as YAML frontmatter.

**`content` and long text (planner context):** The executor resolves `content` like other tool string args. Prefer **short references** instead of pasting full articles into the action:

- **`$variable`** — bound to a Note or Collection; body is loaded from the resource manager.
- **`Note_*` / `Collection_*` / `Relation_*`** — literal resource id; body is loaded the same way.
- **Plain markdown string** — used as-is (fine for short snippets; avoid huge literals in codegen).

Optional two-step when the body only exists under a **named** resource: `load` that name into `$x`, then `write` with `"content":"$x"`.

### list — List directory contents
```json
{"type":"obsidian","action":"list","path":"KnowledgeBases/Transformers/","out":"$files"}
```
Returns a Note listing the files and subdirectories at the given path.

## Output

Success (`status: "success"`):
- `search`: `resource_id` — Collection of Notes with `text`, `metadata.uri`, `metadata.domain`, `format`, `char_count`
- `read`: `resource_id` — Note with file content
- `write`: `value` — confirmation message
- `list`: `resource_id` — Note with directory listing

## Requirements

- Obsidian Local REST API plugin installed and enabled
- `OBSIDIAN_URL` environment variable (default: `http://127.0.0.1:27123`)
- `OBSIDIAN_API_KEY` environment variable

## Examples

Read a file, then update it:
```json
{"type":"obsidian","action":"read","path":"KnowledgeBases/AI/_index.md","out":"$idx"}
{"type":"obsidian","action":"write","path":"KnowledgeBases/AI/_index.md","content":"# AI Knowledge Base\n\nUpdated index...","out":"$r"}
```

List a directory to discover structure:
```json
{"type":"obsidian","action":"list","path":"KnowledgeBases/","out":"$dirs"}
```

Search and summarize:
```json
{"type":"obsidian","action":"search","query":"attention mechanism","out":"$notes"}
{"type":"summarize","target":"$notes","focus":"attention mechanisms","out":"$summary"}
```

Write a large note already in the infospace (by binding — keeps the tool call small):
```json
{"type":"obsidian","action":"write","path":"Clippings/archived-article.md","content":"$article","out":"$obs_result"}
```

Write from a named note (two steps):
```json
{"type":"load","target":"web-clip-draft","out":"$draft"}
{"type":"obsidian","action":"write","path":"Clippings/web-clip-draft.md","content":"$draft","out":"$obs_result"}
```
