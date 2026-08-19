---
name: tavily
description: Web search that returns the sources themselves — ranked results, each with a verbatim snippet and the page's own extracted text. Use when you will state specifics someone may act on (a number, price, date, name, credential, quote, API contract), or whenever you mean to read sources rather than accept an answer. Contrast `search-web`, which returns one model's synthesized answer *about* the sources.
args:
  query: required string — what you want to know, in natural language (a question, not a search-engine query string)
  max_results: optional int — how many pages to return (default 4, max 8)
---

# tavily

Search the web and get back what the pages actually say. Each result
carries a title, its URL, a verbatim snippet, and — where extraction
succeeded — the page's text in Markdown.

## Choosing between this and `search-web`

Both take a question and neither needs a URL, so the split is about what
you intend to do with the answer:

- **`search-web`** — you want *an answer*. Orientation, breadth,
  unfamiliar territory, reconciling sources that disagree. You get one
  model's synthesis, which is fast to read and cheap to act on when the
  stakes are low.
- **`tavily`** — you want *the sources*. Anything you will assert as a
  specific: a phone number, a price, a version, a date, a credential, a
  quoted contract, an API parameter. The page's own words are the
  evidence, so what you say next rests on a document rather than on a
  summary of one.

Using both in one turn is normal, not a fallback: `search-web` for the
map, `tavily` for the handful of facts that go into the reply.

`fetch-text` remains the tool for a URL you already have, and for reading
a page in full — this returns the head of each page, not all of it.

## What comes back

```
[1] <title>
    <url>
snippet: <verbatim extract, relevant to the query>
page: <the page's text, truncated>
```

`page:` is best-effort. Some sites defeat extraction, and a result whose
page text is missing says so — you still have the snippet, but treat that
result as a lead rather than as a read source. When a page matters and
its text is missing or cut short, `fetch-text` the URL.

## Cost

Every call is a Tavily `advanced` search: 2 credits. Extraction on
`basic` is materially less reliable — measured over three live queries,
4 of 9 results came back with no page text on `basic` and 9 of 9 had it
on `advanced` — and a result without page text defeats the point of
reaching for this tool.

## Required environment

- `TAVILY_API_KEY`

## Examples

```json
{"thought": "the user will call this office, so the number has to come off their own page", "tool": "tavily", "query": "Acorn Family Dental Care Berkeley phone number and address"}
```

```json
{"thought": "quoting the API's own docs on this parameter, not a summary of them", "tool": "tavily", "query": "Tavily search API include_raw_content parameter accepted values", "max_results": 3}
```
