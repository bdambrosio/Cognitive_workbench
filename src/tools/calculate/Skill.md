---
name: calculate
description: Compute the value of a math expression. Handles arithmetic, algebra, trig, calculus. Use this instead of LLM arithmetic — calculate is exact and deterministic.
args:
  expression: required string — the math expression, e.g. "2 + 3 * sqrt(7)" or "x**2 - 4 = 0"
  vars: optional string — variable bindings as "name=value, name=value", e.g. "v=20, theta=30, g=9.8"
  precision: optional int (default 10) — decimal places for the result
---

# calculate

Numerically evaluate mathematical expressions via SymPy. Returns the result as a string.

## Supported operations

- **Operators**: `+`, `-`, `*`, `/`, `**`, `%`
- **Constants**: `pi`, `E`
- **Functions**: `sin`, `cos`, `tan`, `asin`, `acos`, `atan` (radians), `sqrt`, `log`, `ln`, `exp`, `abs`, `factorial`
- **Equations**: an expression containing `=` is solved (e.g. `x**2 - 4 = 0` → `-2.0, 2.0`)

All trig functions expect radians.

## Examples

```json
{"thought": "compute the discriminant", "tool": "calculate", "expression": "b**2 - 4*a*c", "vars": "a=1, b=-5, c=6"}
```

```json
{"thought": "projectile range", "tool": "calculate", "expression": "2 * v * sin(theta * pi / 180) / g", "vars": "v=20, theta=30, g=9.8"}
```

```json
{"thought": "roots of a quadratic", "tool": "calculate", "expression": "x**2 - 4 = 0"}
```
