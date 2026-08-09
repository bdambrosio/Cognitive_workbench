---
name: obsidian
description: Read, search, write, and list notes in the user's Obsidian vault. You can READ and SEARCH the whole vault — the user's own notes, web clippings, everything. You can only WRITE inside your own area (CW/<your name>/); any path you give for a write is placed there, so you can never overwrite the user's notes. The write observation tells you the actual path written — use that path to read it back. Multi-action — set `action` first, then provide the args required by that action.
args:
  action: required string — one of "read", "search", "write", "list"
  path: required for read/write/list — vault-relative path, e.g. "Clippings/article.md" or "KnowledgeBases/". Real note names often contain spaces; pass them exactly as they appear. For write, the path is taken relative to your own area.
  query: required for search — search query text
  content: required for write — markdown text to write (may be a `$stepN` binding)
  frontmatter: optional for write — YAML frontmatter fields to prepend (flat dict of strings)
---

# obsidian

Access the user's Obsidian vault via the Local REST API plugin.

## Actions

- **`read`** — read a single file. Needs `path`. Returns the file's markdown content. Whole vault in scope.
- **`search`** — find notes by query. Needs `query`. Filename matches first, then content matches. Whole vault in scope.
- **`write`** — create or overwrite a file **inside your own area**. Needs `path` and `content`. Optional `frontmatter` prepended as YAML.
- **`list`** — list directory contents. Needs `path` (directory, e.g. `"KnowledgeBases/"`). Whole vault in scope.

## Write scope

Writes are fenced to `CW/<agent-name>/`. The path you supply is interpreted
relative to that area: asking to write `Security/patrol.md` as Sentinel puts
the note at `CW/Sentinel/Security/patrol.md`. This is not an error and needs
no special path prefix from you — just write where you mean to, and read the
returned path to know where it landed.

You cannot write outside your own area, including into another agent's. A
path naming somewhere else in the vault nests under your area rather than
reaching it. To get something to another agent, message them directly
instead of leaving it in the vault.

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
- `write` is destructive **within your own area** (overwrites if the file exists there). Read first if you want to preserve existing content.
- Note names are preserved exactly on read; spaces in a filename are normal and need no escaping. Names of files you *create* have spaces replaced with underscores.
