---
name: web-search
type: python
trusted: true
description: Search the web using Google CSE + LLM extraction and return formatted results with relevant content and URLs
parameters: none
---

# Web Search Tool

Searches the web for current information on any topic using Google Custom Search Engine with LLM-powered content extraction.

## Features
- Concurrent fetching for speed
- LLM-based content extraction and summarization
- Query rephrasing for better results
- Domain-aware ranking

## Input
- Query string (e.g., "weather forecast Berkeley CA October 2025")

## Output
- Markdown-formatted search results with:
  - Domain name
  - Extracted relevant content (LLM-summarized)
  - Source URL

## Configuration
Requires `GOOGLE_API_KEY` and `GOOGLE_CX` environment variables.

## Examples

**Query:** "transformer architecture papers"

**Returns:**
```
# Search Results for: transformer architecture papers

## 1. Attention Is All You Need
The dominant sequence transduction models are based on complex recurrent or convolutional neural networks...
Source: https://arxiv.org/abs/1706.03762

## 2. BERT: Pre-training of Deep Bidirectional Transformers
We introduce a new language representation model called BERT...
Source: https://arxiv.org/abs/1810.04805
```

## Common Workflows

### Pattern 1: Search for user consumption
When user needs direct answer, summarize results with query as focus:
```json
{"type":"apply","target":"web-search","args":{"query":"what are transformers in AI"},"out":"results"}
{"type":"apply","target":"summarize-content","value":"$results","args":{"focus":"what are transformers"},"out":"summary"}
{"type":"say","target":"user","value":"$summary"}
```

### Pattern 2: Search for further analysis
When results need indexing/semantic search, split into individual notes per URL:
```json
{"type":"apply","target":"web-search","args":{"query":"transformer papers"},"out":"results"}
{"type":"expand","target":"$results","out":"notes_collection"}
{"type":"index","value":"$notes_collection"}
{"type":"search","value":"$notes_collection","args":{"query":"attention mechanism"},"out":"relevant"}
```

Note: Do NOT create a collection with only the web-search result Note and try to index/search it - this returns the same single item. Use Pattern 1 (summarize) or Pattern 2 (expand) instead.

