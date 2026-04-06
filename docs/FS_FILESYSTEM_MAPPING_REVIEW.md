# FS Filesystem Mapping to Collections/Notes: Comprehensive Review

## Executive Summary

This document reviews how filesystem operations map to the infospace Collection/Note model, documents **current** tool behavior, and retains a postmortem of an agent failure that exposed usability issues around opaque IDs and discovery.

**Implementation update:** `fs-list` **no longer** returns a `Collection` of per-file Notes. It returns a **single `Note`** whose body is a **plain-text directory listing** (like `ls`: path header, `subdir/`, `filename  (size)`). Read it with **`get_text($binding)`**—**not** `get_items` (that API is for Collections only). Skills and planner specs were aligned with this behavior.

**Historical failure (original write-up):** The agent located the target file (`Nan_Ar.txt` under `bhagavan/`) in principle but failed to recognize it when `fs-list` exposed **only anonymous IDs** in a Collection summary. That specific failure mode for **`fs-list` is largely addressed** by the listing Note (names appear in the listing text). **Opaque IDs** and **placeholder Notes** remain a concern for **`fs-find`** (Collection of placeholders) and for any workflow that only inspects `resource_id` without loading content.

---

## Current implementation (authoritative)

### `fs-list`

- **Input:** `path` (relative under `scenarios/<world_name>/fs`), `recursive`, `include_files`, `include_dirs`, `max_entries`.
- **Output:** `resource_id` is a **`Note_*`**, not a Collection.
- **Body:** Multi-line string: first line is the listed path; **directories** end with `/`; **files** look like `name  (2.1K)` / `name  (219B)`; optional truncation line when `max_entries` is hit.
- **Agent pattern:** After `tool("fs-list", ..., out="$dir_list")`, read the body with **`get_text("$dir_list")`**. Do **not** call `get_items` on the result. Interpreting that text is up to the planner.

### `fs-find`

- **Output:** `Collection_*` of **placeholder** file Notes (body `(placeholder)` until `fs-read`).
- **Use case:** Glob-style filename search; then `fs-read` on chosen paths or Note IDs as supported by your stack.

### `fs-read`

- **Output:** `Note_*` with actual file content (text, JSON, PDF-extracted text, etc.) and metadata.

### `fs-stat`

- **Output:** `Note_*` with structured JSON metadata (path, size, mtime, etc.).

### `fs-grep`

- **Output:** `Collection_*` of Notes with matching line snippets (content search, not filename search).

### Mapping at a glance

| Goal | Tool | Primary result |
|------|------|----------------|
| See directory contents as text | `fs-list` | One **listing Note** |
| Find files by name pattern | `fs-find` | **Collection** of placeholder Notes |
| Read file bytes as Note | `fs-read` | **Note** |
| Metadata without full read | `fs-stat` | **Note** (JSON) |
| Search inside files | `fs-grep` | **Collection** |

---

## Historical context: prior `fs-list` model (Collection-based)

The following described behavior applied **before** `fs-list` was changed to a single listing Note. It is kept because it explains the original incident and still illustrates pitfalls that affect **other** tools (e.g. `fs-find`).

### Former mapping (obsolete for `fs-list`)

- **Directory → Collection:** Each listed directory was a `Collection` whose items were `Note_*` (files) and nested `Collection_*` (subdirs).
- **File Note content:** Filename often appeared as the Note’s visible text, with path/metadata in structured fields—discovery required **loading each Note** because summaries showed only IDs.

### What the agent saw (historical `fs-list`)

```
Success: 2 items [Note_9, Note_10]
resource_id: Collection_10
```

Without loading each Note or rich summaries, the agent could not map `Collection_10` to `bhagavan/` or `Note_9` to `Nan_Ar.txt`.

### Critical gap (historical)

1. Receive `Collection_10` from `fs-list` on `bhagavan/`.
2. Infer that `Collection_10` **is** that directory (not always obvious from ID-only summaries).
3. Load each `Note_*` to learn filenames.
4. Match the target filename.

The agent failed at steps 2–4 when IDs were opaque and summaries omitted paths/names.

---

## Root cause analysis (still relevant in part)

### Opaque ID-based representation

**Problem:** Many tools still surface `Note_*` / `Collection_*` without human-readable path/name in the short success string.

**Impact:** Workflows that never call `get_text` / `load` / structured display may remain confused.

**Mitigation today:** For directory **listing**, read the **`fs-list` Note body**—names are in plain text.

### Tool limitations (updated)

- **`fs-grep`:** Searches **file contents**, not paths/filenames—misuse if the goal is “find file named X” (use **`fs-find`** or the **`fs-list`** listing text).
- **`fs-find`:** Now exists (glob on basename under a path); returns a **Collection** of placeholders—agents still need a clear pattern: list IDs → `fs-read` or follow docs for binding chain.

---

## Design critique

### Listing Note (`fs-list` today)

**Strengths:**

- **Self-describing text:** Filenames and directory names appear directly in the Note body.
- **No per-entry ID hop** for “what files are here?”—one string holds the listing.
- **Aligned with planner guidance:** `get_text` on the bound variable.

**Weaknesses:**

- **Unstructured text:** Interpretation is planner-dependent (contrast with JSON); edge cases in filenames are theoretically possible.
- **Not a Collection:** Per-file infospace operations (map, filter on items) require **`fs-find`** + placeholders or **`fs-read`** paths, not `get_items` on `fs-list` output.

### Collection of placeholders (`fs-find`)

**Strengths:** Fits infospace patterns for “set of files” and chaining to `fs-read`.

**Weaknesses:** Same class of **opacity** as the old `fs-list` unless displays and docs emphasize loading/reading.

### Alternative / hybrid ideas (design space)

Ideas in the original doc (directory as JSON Note, richer Collection summaries, companion discovery Note) remain valid **enhancements** for `fs-find` and global UX, but **`fs-list` already implements** the “single document listing names” idea in text form.

---

## Tool set (updated)

### Current tools

| Tool | Purpose | Notes |
|------|---------|--------|
| `fs-list` | List directory | **One listing Note**; use `get_text` |
| `fs-find` | Find by glob | **Collection** of placeholder Notes |
| `fs-stat` | Metadata | **Note** (JSON) |
| `fs-read` | Read content | **Note** |
| `fs-head` | First lines | **Note** |
| `fs-grep` | Search contents | **Collection** of match Notes |

### Possible future improvements

- Richer **success summaries** (path + snippet) for any tool returning IDs.
- Lightweight **path-exists** or **walk** helpers if recurring gaps appear.

---

## Recommendations (revised)

### Done / addressed for `fs-list`

1. **Listing in one Note** — implemented; document and teach **`get_text`**, not **`get_items`**.

### Still valuable

2. **Display and docs:** When a tool returns a **Collection** of file Notes, make it obvious that items are placeholders until **`fs-read`** (and how bindings work).
3. **`fs-find`:** Treat as the primary **filename pattern** path; pair with **`fs-read`** for content.
4. **Agent training / planner rules:** Valid **`out=` names** (identifiers only); handle **structured snippets** (e.g. YAML frontmatter) in code when layout is fixed before defaulting to LLM **`extract`**.

---

## Conclusion

The filesystem stack is **no longer** “directory as Collection” for **`fs-list`**: a **single listing Note** reduces the original **name discovery** problem for directory listing. **Opaque IDs** and **Collections of placeholders** remain important for **`fs-find`** and similar flows—good summaries, docs, and planner hints still matter.

**Recommended path for agents:**

1. **List dir:** `fs-list` → **`get_text`** — use the text as appropriate to the goal.
2. **Find by pattern:** **`fs-find`** → iterate Collection → **`fs-read`** as needed.
3. **Avoid** assuming every filesystem tool returns a Collection; check each tool’s Skill.md / catalog entry.
