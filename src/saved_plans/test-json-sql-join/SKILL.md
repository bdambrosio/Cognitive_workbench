---
name: test-json-sql-join
description: Tests join primitive (INNER JOIN)
manual_only: true
---

# Test Join Primitive

**Self-contained:** Creates test data internally

**Input:** 
- Creates $papers: A, B, C, D
- Creates $authors: A (Alice), B (Bob), E (Eve)

**Operation:** Inner join on id field

**Expected Output:** $joined collection with 2 items (A and B have matches)
- Paper A + Author Alice
- Paper B + Author Bob

