---
name: ramana-saying
description: Return one genuine saying of Ramana Maharshi, drawn verbatim from his recorded talks, with source attribution. Use when delivering an authentic Ramana quote with attribution — not a paraphrase or a synthesized reflection. The returned text is a raw quote; add your own brief framing before presenting it.
args: {}
---

# ramana-saying

Samples a single question → answer exchange from a bundled, verbatim corpus of
*Talks with Sri Ramana Maharshi* and returns it as markdown with source
attribution (e.g. *(Talk 26)*). Takes no arguments. Where the book provides a
narrative scene-setting passage for the talk's opening exchange, it is included
as an italicized lead-in above the Q&A.

The tool does **not** add commentary — it hands back the genuine quote so you
can frame it in your own voice. A small recent-history file keeps successive
calls from repeating the same saying.

## Behavior

- Loads the bundled pool (`data/sayings.jsonl`), skips recently-shown sayings,
  and picks one at random.
- Returns `{status: "ok", text: <markdown quote + attribution>}`.
- Returns `{status: "error", text: ...}` if the pool is missing or empty.

## Typical use

When a quote is wanted, call the tool, add one short framing sentence in your
own voice, render a card to the canvas with `display`, then post a brief line
to the conversation.

```json
{"thought": "deliver a genuine attributed Ramana saying", "tool": "ramana-saying"}
```

## Refreshing the pool

`data/sayings.jsonl` is generated once from the source corpus by
`build_pool.py` (reads `/data/satsang/ramana/`). Re-run that script by hand to
rebuild; it is not invoked at runtime.
