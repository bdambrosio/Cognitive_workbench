---
name: test-json-sql-pluck
description: Tests pluck primitive (SELECT single field)
manual_only: true
---

# Test Pluck Primitive

**Requires:** Run test-json-sql-setup first to create $papers

**Input:** $papers collection

**Operation:** Pluck title field from each paper

**Expected Output:** $titles collection with 4 items, each containing just the title string

