---
name: test-json-sql-filter
description: Tests filter-structured primitive (WHERE clause)
manual_only: true
---

# Test Filter-Structured Primitive

**Requires:** Run test-json-sql-setup first to create $papers

**Input:** $papers collection (citations: 100, 250, 50, 180)

**Operation:** Filter where citations > 100

**Expected Output:** $high_cited collection with 2 items (Transformers: 250, Scaling Laws: 180)

