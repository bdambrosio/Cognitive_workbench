---
name: web-search
type: python
trusted: true
description: Search the web using Tavily API and return formatted results with titles, snippets, and URLs
---

# Web Search Tool

Searches the web for current information on any topic.

## Input
- Query string (e.g., "LLM cognitive agents 2025")

## Output
- Markdown-formatted search results with:
  - Result title
  - Content snippet
  - Source URL

## Configuration
Requires `TAVILY_API_KEY` environment variable.

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

