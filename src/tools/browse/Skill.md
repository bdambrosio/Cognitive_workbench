---
name: browse
type: python
description: "Browser automation via agent-browser CLI. Navigate, snapshot accessibility tree, interact by element ref."
schema_hint:
  action: "string (required): open|snapshot|click|type|fill|press|scroll|get|eval|close|batch"
  url: "string (for open)"
  selector: "string (CSS selector or @ref from snapshot, for click/type/fill/press/scroll/get)"
  text: "string (for type/fill)"
  key: "string (for press, e.g. 'Enter', 'Tab')"
  expression: "string (for eval — JavaScript expression)"
  subcommand: "string (for get: text|html|value|title|url)"
  actions: "list of [command, ...args] arrays (for batch)"
  out: "$variable"
---

# browse

Control a persistent browser session via [agent-browser](https://github.com/nichochar/agent-browser). Navigate pages, read the accessibility tree, and interact with elements by ref.

## Core Workflow

1. **Open** a URL
2. **Snapshot** to get the accessibility tree with element refs (`@e1`, `@e2`, ...)
3. **Read** the snapshot Note to find target elements
4. **Interact** using refs: click, type, fill, press
5. **Snapshot** again to see the result

## Actions

### open — Navigate to URL
```json
{"type":"browse","action":"open","url":"https://example.com","out":"$status"}
```

### snapshot — Get accessibility tree as Note
```json
{"type":"browse","action":"snapshot","out":"$page"}
```
Returns interactive elements only (buttons, links, inputs). Use `extract` on the resulting Note to find specific content. Supports optional `selector` to scope to a CSS subtree.

### click — Click an element
```json
{"type":"browse","action":"click","selector":"@e3","out":"$r"}
```

### type / fill — Enter text into a field
```json
{"type":"browse","action":"fill","selector":"@e5","text":"search query","out":"$r"}
```
`fill` clears the field first; `type` appends.

### press — Press a key
```json
{"type":"browse","action":"press","key":"Enter","out":"$r"}
```

### scroll — Scroll the page
```json
{"type":"browse","action":"scroll","text":"down","out":"$r"}
```
Directions: `up`, `down`, `left`, `right`. Optional `selector` to scroll within an element.

### get — Extract specific content
```json
{"type":"browse","action":"get","subcommand":"text","selector":"@e2","out":"$content"}
{"type":"browse","action":"get","subcommand":"title","out":"$title"}
{"type":"browse","action":"get","subcommand":"url","out":"$url"}
```
Subcommands: `text`, `html`, `value` (require selector), `title`, `url` (no selector needed).

### eval — Run JavaScript
```json
{"type":"browse","action":"eval","expression":"document.title","out":"$result"}
```

### close — End browser session
```json
{"type":"browse","action":"close","out":"$r"}
```

### batch — Run multiple commands in one step
```json
{"type":"browse","action":"batch","actions":[["open","https://news.ycombinator.com"],["wait","2000"],["snapshot","-i","-c"]],"out":"$page"}
```
Each inner array is `[command, arg1, arg2, ...]`. Commands run sequentially; stops on first error. The last command's output is returned. Use batch for deterministic sequences (open+wait+snapshot, or multiple fills) to conserve planner steps. Do NOT batch when you need to read a snapshot before deciding what to do next.

## Output

Success: `resource_id` for snapshot (Note with accessibility tree), `value` for everything else.
Failure: `reason` with error details.

## Planning Notes

- Element refs (`@e1`, `@e2`) are only valid until the next snapshot — page changes invalidate them.
- Always snapshot after navigation or clicks that change page state.
- Batch deterministic sequences (open+wait+snapshot) to save steps. Keep interactive decisions as separate steps.
- Use `get text @ref` for targeted extraction instead of re-snapshotting the whole page.
- The browser session persists across steps within a goal. Call `close` when done if the page won't be needed again.
- Requires `agent-browser` CLI installed and on PATH.
