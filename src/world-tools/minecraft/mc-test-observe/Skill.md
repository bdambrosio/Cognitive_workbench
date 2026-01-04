---
name: mc-test-observe
type: python
description: "Tests movement prediction accuracy by comparing observation-based predictions with actual movement results for forward, left, right, and back directions."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"mc-test-observe","out":"$test_result"}'
---

Tool: mc-test-observe

Purpose:
Test the accuracy of movement predictions based on observation data by comparing predictions with actual movement attempts.

Returns:
A test report showing:
- For each direction (fwd, left, right, back): prediction, actual result, and whether they match
- Summary of prediction accuracy

