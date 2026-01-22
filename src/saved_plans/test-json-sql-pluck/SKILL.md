---
name: test-json-sql-pluck
description: Tests pluck primitive (SELECT single field)
manual_only: true
---

# Test Pluck Primitive

**Self-contained:** Creates test data internally

**Input:** Creates $papers collection (4 papers with id, title, year, citations, venue)

**Operation:** Pluck title field from each paper

**Expected Output:** $titles collection with 4 items, each containing just the title string

