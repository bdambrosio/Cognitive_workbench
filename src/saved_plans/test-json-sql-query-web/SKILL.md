---
name: test-json-sql-search-web
description: Test JSON SQL primitives with search-web output
type: plan
manual_only: true
parameters: []
---

# test-json-sql-search-web

Tests JSON SQL primitives (project, pluck, filter-structured, sort) with real search-web output.

## What it tests
- search-web returns Collection of Notes
- project extracts metadata.uri, metadata.domain, char_count
- pluck extracts first URI
- filter-structured filters by char_count > 100
- sort orders by char_count descending

