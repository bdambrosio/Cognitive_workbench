---
name: test-json-sql-semantic-scholar
description: Test JSON SQL primitives with semantic-scholar output
type: plan
manual_only: true
parameters: []
---

# test-json-sql-semantic-scholar

Tests JSON SQL primitives (project, pluck, filter-structured, sort) with real semantic-scholar output.

## What it tests
- semantic-scholar returns Collection of paper Notes
- project extracts metadata.title, metadata.year, metadata.citations
- pluck extracts first title
- filter-structured filters by metadata.citations > 0
- sort orders by metadata.citations descending

