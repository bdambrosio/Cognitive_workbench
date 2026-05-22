---
name: obsidian
description: Read, search, write, and list notes in the user's Obsidian vault. Multi-action — set `action` first, then provide the args required by that action.
args:
  action: required string — one of "read", "search", "write", "list"
  path: required for read/write/list — vault-relative path, e.g. "Clippings/article.md" or "KnowledgeBases/"
  query: required for search — search query text
  content: required for write — markdown text to write (may be a `$stepN` binding)
  frontmatter: optional for write — YAML frontmatter fields to prepend (flat dict of strings)
---

# obsidian

Access the user's Obsidian vault via the Local REST API plugin.

## Actions

- **`read`** — read a single file. Needs `path`. Returns the file's markdown content.
- **`search`** — find notes by query. Needs `query`. Filename matches first, then content matches.
- **`write`** — create or overwrite a file. Needs `path` and `content`. Optional `frontmatter` prepended as YAML.
- **`list`** — list directory contents. Needs `path` (directory, e.g. `"KnowledgeBases/"`).

## Required environment

- `OBSIDIAN_URL` (default `http://127.0.0.1:27123`) — Local REST API endpoint
- `OBSIDIAN_API_KEY` — API key for the Local REST API plugin

## Examples

```json
{"thought": "look up the transformers note", "tool": "obsidian", "action": "read", "path": "KnowledgeBases/Transformers/_index.md"}
```

```json
{"thought": "search for notes about attention mechanisms", "tool": "obsidian", "action": "search", "query": "attention mechanism"}
```

```json
{"thought": "save the synthesis from the previous step", "tool": "obsidian", "action": "write", "path": "Clippings/2026-05-21-synthesis.md", "content": "$step2"}
```

```json
{"thought": "see what's in the AI knowledge base", "tool": "obsidian", "action": "list", "path": "KnowledgeBases/AI/"}
```

## Notes

- Large content is best passed via a `$stepN` binding rather than inlined into the action — keeps the tool call small.
- `write` is destructive (overwrites if file exists). Read first if you want to preserve existing content.
