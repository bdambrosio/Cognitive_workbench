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

