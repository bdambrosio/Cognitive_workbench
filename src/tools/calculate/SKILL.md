---
name: calculate
description: Numerically evaluate mathematical expressions using SymPy
type: python
schema_hint:
  value: "string (mathematical expression, required)"
  variables: "dict (optional, variable substitutions)"
  precision: "int (optional, default 10)"
  out: "$variable"
parameters:
  - value: Mathematical expression string
  - variables: Optional dict of variable substitutions
  - precision: Optional decimal places (default 10)
examples:
  - '{"type":"calculate","value":"2 + 3 * 4","out":"$result"}'
  - '{"type":"calculate","value":"2 * v * sin(theta * pi / 180) / g","variables":{"v":20,"theta":30,"g":9.8},"out":"$time"}'
---

# calculate

Evaluate mathematical expressions **numerically** using SymPy.

This tool is intended for reliable arithmetic and formula evaluation.
It returns floating-point numeric results (not symbolic expressions).

---

## Action Format

```json
{"type": "calculate", "value": "2 * 20 * sin(30 * pi / 180) / 9.8", "out": "$result"}
```

---

## Parameters

| Parameter | Required | Description |
|----------|----------|-------------|
| `value` | Yes | Mathematical expression as a string |
| `variables` | No | Dictionary of variable substitutions (numbers only) |
| `precision` | No | Decimal places for numeric evaluation (default: 10) |
| `out` | Yes | Variable name to bind the result |

---

## Expression Syntax

### Arithmetic
- `+`, `-`, `*`, `/`
- `**` (power)
- `%` (modulo)

### Constants
- `pi`
- `E` (Euler’s number)

### Trigonometry (radians only)
- `sin(x)`, `cos(x)`, `tan(x)`
- `asin(x)`, `acos(x)`, `atan(x)`

⚠ **Important**: All trigonometric functions expect **radians**.  
Degrees must be converted explicitly, e.g. `sin(30 * pi / 180)`.

### Other functions
- `sqrt(x)`
- `log(x)` (natural log)
- `ln(x)` (alias for natural log)
- `exp(x)`
- `abs(x)`
- `factorial(n)`

---

## Variable Substitution

Use `variables` to substitute numeric values into expressions.

```json
{
  "type": "calculate",
  "value": "2 * v * sin(theta * pi / 180) / g",
  "variables": {"v": 20, "theta": 30, "g": 9.8},
  "out": "$time"
}
```

All variables **must resolve to numeric values** after substitution.
If unresolved symbols remain, evaluation may fail.

---

## Equation Solving (Advanced)

This tool supports simple equation solving via SymPy.

### Using `=`
Expressions containing a single `=` are interpreted as equations.

```json
{"type":"calculate","value":"x**2 - 4 = 0","out":"$roots"}
```

### Using `solve(...)`
You may explicitly call `solve`:

```json
{"type":"calculate","value":"solve(x**2 - 4, x)","out":"$roots"}
```

### Output format for solutions
- Multiple solutions are returned as a **comma-separated string**
- Each solution is numerically evaluated to the requested precision

Example result:
```
-2.000000000, 2.000000000
```

If individual values are needed, use downstream tools (e.g. `split`).

---

## Output

- The result is returned as a **string representation of a numeric value**
- Floating-point evaluation is always used
- Integer expressions will still return floats (e.g. `14.000000000`)

To convert to an integer, use a coercion tool such as:

```json
{"type":"coerce","value":"$result","coercion":"to-int","out":"$int_result"}
```

The Skills runtime binds the returned value to the variable specified by `out`.

---

## Failure Conditions

The tool may fail if:
- The expression is syntactically invalid
- Variables are missing or non-numeric
- Symbols remain unresolved after substitution
- The expression cannot be numerically evaluated

In these cases, the tool returns a failure status rather than a value.

---

## When to Use

- Reliable arithmetic and formula evaluation
- Physics and engineering calculations
- Numeric evaluation where LLM arithmetic is unreliable

Avoid using this tool for:
- Symbolic manipulation
- Algebraic simplification
- Proofs or exact symbolic results
