---
name: calculate
type: python
description: "Numerically evaluate mathematical expressions using SymPy"
---

# Calculate Tool

Evaluate mathematical expressions numerically using SymPy. Returns floating-point numeric results (not symbolic expressions).

## Purpose

Reliable arithmetic and formula evaluation for physics, engineering, and mathematical calculations. Provides deterministic numeric results where LLM arithmetic may be unreliable.

## Input

- `value`: Mathematical expression as a string (required)
- `variables`: Dictionary of variable substitutions (optional, numbers only)
- `precision`: Decimal places for numeric evaluation (optional, default: 10)

## Output

Returns string representation of numeric value. Floating-point evaluation is always used. Integer expressions will still return floats (e.g., `14.000000000`).

## Behavior & Performance

- Expression syntax supports arithmetic (`+`, `-`, `*`, `/`, `**`, `%`), constants (`pi`, `E`), trigonometry (`sin`, `cos`, `tan`, `asin`, `acos`, `atan` - radians only), and other functions (`sqrt`, `log`, `ln`, `exp`, `abs`, `factorial`)
- Variable substitution: Use `variables` dict to substitute numeric values into expressions
- Equation solving: Expressions containing a single `=` are interpreted as equations. Multiple solutions returned as comma-separated string
- All trigonometric functions expect radians. Degrees must be converted explicitly (e.g., `sin(30 * pi / 180)`)

## Guidelines

- Use for reliable arithmetic and formula evaluation
- Avoid for symbolic manipulation, algebraic simplification, or proofs
- To convert result to integer, use coercion tool: `{"type":"coerce","target":"$result","coercion":"to-int","out":"$int_result"}`
- Tool may fail if expression is syntactically invalid, variables are missing/non-numeric, symbols remain unresolved, or expression cannot be numerically evaluated

## Usage Examples

Basic arithmetic:
```json
{"type":"calculate","value":"2 + 3 * 4","out":"$result"}
```

With variables:
```json
{"type":"calculate","value":"2 * v * sin(theta * pi / 180) / g","variables":{"v":20,"theta":30,"g":9.8},"out":"$time"}
```

Equation solving:
```json
{"type":"calculate","value":"x**2 - 4 = 0","out":"$roots"}
```
