---
name: test-json-sql-project
description: Tests project primitive (SELECT fields)
manual_only: true
---

# Test Project Primitive

**Self-contained:** Creates test data internally

**Input:** Creates $papers collection (4 papers with id, title, year, citations, venue)

**Operation:** Project only title and year fields

**Expected Output:** $projected collection with 4 items, each containing only {title, year}

