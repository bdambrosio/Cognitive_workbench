---
name: test-json-sql-setup
description: Creates test data for JSON SQL primitive tests
manual_only: true
---

# Test JSON SQL Setup

Creates two test collections for testing JSON SQL primitives:

## $papers Collection (4 items)
- Paper A: Deep Learning, 2020, 100 citations, ICML
- Paper B: Transformers, 2021, 250 citations, NeurIPS  
- Paper C: GPT-4 Analysis, 2023, 50 citations, JMLR
- Paper D: Scaling Laws, 2022, 180 citations, ICML

## $authors Collection (3 items)
- Author A: Alice, MIT
- Author B: Bob, Stanford
- Author E: Eve, Berkeley (no matching paper)

## Usage
Execute this plan first, then run other test-json-sql-* plans.
Check Bindings tab to verify $papers and $authors are created.

