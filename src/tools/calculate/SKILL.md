---
name: calculate
description: Evaluate mathematical expressions using SymPy
type: python
trusted: true
parameters:
  - value: Mathematical expression string
  - variables: Optional dict of variable substitutions
  - precision: Optional decimal places (default 10)
examples:
  - '{"type":"calculate","value":"2 + 3 * 4","out":"$result"}'
  - '{"type":"calculate","value":"2 * v * sin(theta * pi / 180) / g","variables":{"v":20,"theta":30,"g":9.8},"out":"$time"}'
---

# calculate

Evaluate mathematical expressions using SymPy. Returns exact numeric results.

## Action Format

```json
{"type": "calculate", "value": "2 * 20 * sin(30 * pi / 180) / 9.8", "out": "$result"}
```

## Parameters

| Parameter | Required | Description |
|-----------|----------|-------------|
| `value` | Yes | Mathematical expression string |
| `variables` | No | Dict of variable substitutions: `{"v": 20, "g": 9.8}` |
| `precision` | No | Decimal places (default: 10) |
| `out` | Yes | Variable to bind result |

## Expression Syntax

- **Arithmetic**: `+`, `-`, `*`, `/`, `**` (power), `%` (mod)
- **Constants**: `pi`, `E` (Euler's number)
- **Trig (radians)**: `sin(x)`, `cos(x)`, `tan(x)`, `asin(x)`, `acos(x)`, `atan(x)`
- **Degrees**: Use `sin(30 * pi / 180)` for sin(30°)
- **Other**: `sqrt(x)`, `log(x)`, `ln(x)`, `exp(x)`, `abs(x)`, `factorial(n)`

## Examples

Basic arithmetic:
```json
{"type": "calculate", "value": "2 + 3 * 4", "out": "$result"}
```
Result: `14`

Physics with inline values:
```json
{"type": "calculate", "value": "2 * 20 * sin(30 * pi / 180) / 9.8", "out": "$time"}
```
Result: `1.0204081633`

Formula with variable substitution:
```json
{"type": "calculate", "value": "2 * v * sin(theta * pi / 180) / g", "variables": {"v": 20, "theta": 30, "g": 9.8}, "out": "$time"}
```
Result: `1.0204081633`

Quadratic formula:
```json
{"type": "calculate", "value": "(-b + sqrt(b**2 - 4*a*c)) / (2*a)", "variables": {"a": 2, "b": 5, "c": 3}, "out": "$root"}
```
Result: `-1.0`

## When to Use

- Evaluating physics formulas
- Solving arithmetic in word problems
- Any calculation requiring precision (LLM arithmetic is unreliable)

## Output

Returns the numeric result as a string. Creates a Note containing the result value.

**Note**: Results are returned as floats (e.g., `14.00000000` for integer calculations). If an integer result is required, use the `coerce` primitive with `coercion: "to-int"` to convert the result.

