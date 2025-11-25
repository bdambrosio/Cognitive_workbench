---
name: test-json-sql-query-web
description: Test JSON SQL primitives with query-web output
type: plan
manual_only: true
parameters: []
---

# test-json-sql-query-web

Tests JSON SQL primitives (project, pluck, filter-structured, sort) with real query-web output.

## What it tests
- query-web returns Collection of Notes
- project extracts metadata.uri, metadata.domain, char_count
- pluck extracts first URI
- filter-structured filters by char_count > 100
- sort orders by char_count descending

