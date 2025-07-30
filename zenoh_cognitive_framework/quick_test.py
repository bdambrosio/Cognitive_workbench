#!/usr/bin/env python3

from plan import parse_plan_text

# Test if-then-else parsing
plan_text = """plan:
  if:
    near(Well23)
  else:
    move(north, 1)
  endif:"""

result = parse_plan_text(plan_text)
print("If-then-else parsing result:")
print(result) 