---
name: test-json-sql-sort
description: Tests sort primitive (ORDER BY)
manual_only: true
---

# Test Sort Primitive

**Self-contained:** Creates test data internally

**Input:** Creates $papers collection (years: 2020, 2021, 2023, 2022)

**Operation:** Sort by year descending

**Expected Output:** $sorted collection ordered: 2023, 2022, 2021, 2020

