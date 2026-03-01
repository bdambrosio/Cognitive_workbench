---
name: calculate
type: python
description: "Numerically evaluate mathematical expressions using SymPy"
schema_hint:
  value: "string (math expression, required)"
  variables: "dict (variable substitutions, optional)"
  precision: "int (decimal places, default: 10)"
  out: "$variable (optional)"
---

# calculate

Evaluate mathematical expressions numerically using SymPy. Returns floating-point results.

## Input

- `value`: Mathematical expression as string (required)
- `variables`: Dictionary of variable substitutions (optional, numbers only)
- `precision`: Decimal places for evaluation (optional, default: 10)

## Output

Success (`status: "success"`):
- `value`: String representation of numeric result

Failure (`status: "failed"`):
- `reason`: Error description

## Behavior

- **Operators**: `+`, `-`, `*`, `/`, `**`, `%`
- **Constants**: `pi`, `E`
- **Functions**: `sin`, `cos`, `tan`, `asin`, `acos`, `atan` (radians), `sqrt`, `log`, `ln`, `exp`, `abs`, `factorial`
- **Equations**: Expressions with `=` are solved (e.g., `x**2 - 4 = 0` → `-2.0, 2.0`)
- All trig functions expect radians

## Planning Notes

- Use for reliable arithmetic (LLM arithmetic is unreliable)
- To convert result to integer, use coercion tool afterward

## Examples

```json
{"type":"calculate","value":"2 + 3 * 4","out":"$result"}
{"type":"calculate","value":"2 * v * sin(theta * pi / 180) / g","variables":{"v":20,"theta":30,"g":9.8},"out":"$time"}
{"type":"calculate","value":"x**2 - 4 = 0","out":"$roots"}
```
