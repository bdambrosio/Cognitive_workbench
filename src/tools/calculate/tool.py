"""
Calculate tool - numerically evaluate mathematical expressions using SymPy.
"""

from typing import Any, Dict, Optional
from sympy import (
    sympify, N, pi, E, Basic,
    sin, cos, tan, asin, acos, atan,
    sqrt, log, ln, exp, Abs, factorial,
    solve, Eq
)
from sympy.core.sympify import SympifyError
# InfospaceExecutor was removed with the OODA cleanup. The runtime
# `executor` argument is now duck-typed: callers just need a
# _create_uniform_return method. Annotations widened to Any.


def _fail(executor: Any, reason: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=reason, reason=reason, extra=extra)


def _success(executor: Any, result: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=result, extra=extra)


def _format_value(value: Any, precision: int) -> str:
    """Render one result. Exact stays exact; only genuine floats reach N().

    N() is a float evaluation, so routing everything through it answered `17!`
    with 3.556874281e+14 and would render the roots of x**2 - 4 = 0 as
    -2.000000000, 2.000000000. Non-Basic values are what multi-symbol solve
    hands back (dicts of substitutions) — they have no numeric form to take.
    """
    if not isinstance(value, Basic):
        return str(value)
    if value.is_Integer or value.is_Rational:
        return str(value)
    return str(N(value, precision))


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    from utils.chat_tool_stub import build_tool_kwargs, CapturingResourceManager, translate_result
    expression = args.get("expression", "")
    if not isinstance(expression, str) or not expression.strip():
        return {"status": "error", "text": "calculate requires non-empty `expression`"}

    # Parse "x=2, y=3.5" into a {name: float} dict.
    variables = {}
    vars_str = args.get("vars") or ""
    for pair in vars_str.split(",") if vars_str else []:
        pair = pair.strip()
        if not pair:
            continue
        if "=" not in pair:
            return {"status": "error", "text": f"invalid var spec `{pair}` — use name=number"}
        k, v = pair.split("=", 1)
        try:
            variables[k.strip()] = float(v.strip())
        except ValueError:
            return {"status": "error", "text": f"could not parse value in `{pair}` — must be numeric"}

    try:
        precision = int(args.get("precision", 10))
    except (TypeError, ValueError):
        precision = 10

    mgr = CapturingResourceManager()
    result = tool(expression, **build_tool_kwargs(
        character_name=character_name, backend=backend, manager=mgr,
        variables=variables, precision=precision,
    ))
    return translate_result(result, manager=mgr)


def tool(input_value, runtime=None, **kwargs):
    """
    Evaluate a mathematical expression numerically.

    Args:
        input_value (str): Mathematical expression
        variables (dict): Optional numeric substitutions
        precision (int): Decimal places (default: 10)

    Returns:
        str on success
        dict with status='failed' on error
    """

    # ----------------------------
    # Validate input
    # ----------------------------
    executor: Any = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    if not isinstance(input_value, str):
        return _fail(executor, "input_value must be a string expression")

    variables = kwargs.get("variables", {}) or {}

    # N() counts SIGNIFICANT DIGITS, not decimal places. precision=0 asks for
    # zero significant digits and SymPy duly answers `0.e+14` — a wrong number
    # wearing a success flag, which is the one thing a tool observation must
    # never be. Fail loudly instead.
    precision = kwargs.get("precision", 10)
    try:
        precision = int(precision)
    except (TypeError, ValueError):
        return _fail(executor, f"precision must be an integer, got {precision!r}")
    if precision < 1:
        return _fail(
            executor,
            f"precision is significant digits, not decimal places, and must "
            f"be >= 1 (got {precision}). Omit it for the default of 10 — exact "
            f"integer and rational results are returned exactly either way.")

    if not isinstance(variables, dict):
        return _fail(executor, "variables must be a dictionary")

    # ----------------------------
    # Local namespace for SymPy
    # ----------------------------
    local_dict = {
        "pi": pi,
        "E": E,
        "sin": sin,
        "cos": cos,
        "tan": tan,
        "asin": asin,
        "acos": acos,
        "atan": atan,
        "sqrt": sqrt,
        "log": log,
        "ln": ln,
        "exp": exp,
        "abs": Abs,
        "factorial": factorial,
        "solve": solve,
        "Eq": Eq,
    }

    # ----------------------------
    # Convert single "=" to Eq(...)
    # ----------------------------
    if "=" in input_value and "==" not in input_value and "!=" not in input_value:
        parts = input_value.split("=", 1)
        if len(parts) == 2:
            input_value = f"Eq({parts[0].strip()}, {parts[1].strip()})"

    # ----------------------------
    # Parse expression
    # ----------------------------
    try:
        expr = sympify(input_value, locals=local_dict)
    except SympifyError as e:
        return _fail(executor, f"Invalid mathematical expression: {e}")

    # ----------------------------
    # An explicit solve(...) is already done
    # ----------------------------
    # `solve` is in local_dict as the real function, so sympify CALLS it and
    # hands back a plain list of roots — not a node left in the tree. The old
    # `expr.func == solve` test below could therefore never match, and the
    # list fell into `.subs` instead ("'list' object has no attribute 'subs'").
    if isinstance(expr, (list, tuple)):
        if not expr:
            return _success(executor, "")
        return _success(
            executor, ", ".join(_format_value(s, precision) for s in expr))

    # ----------------------------
    # Substitute variables
    # ----------------------------
    try:
        expr = expr.subs(variables)
    except Exception as e:
        return _fail(executor, f"Variable substitution failed: {e}")

    # ----------------------------
    # Solve an equation
    # ----------------------------
    # The `=` rewrite above produces Eq(lhs, rhs), which is what actually
    # arrives here for the documented `x**2 - 4 = 0` form.
    if isinstance(expr, Eq):
        try:
            solutions = solve(expr)
        except Exception as e:
            return _fail(executor, f"Equation solving failed: {e}")
        if not solutions:
            return _success(executor, "")
        return _success(
            executor, ", ".join(_format_value(s, precision) for s in solutions))

    # ----------------------------
    # Reject unresolved symbols
    # ----------------------------
    free_symbols = getattr(expr, "free_symbols", set())
    if free_symbols:
        symbols = ', '.join(map(str, free_symbols))
        return _fail(executor, f"Unresolved symbols in expression: {symbols}")

    # ----------------------------
    # Evaluate
    # ----------------------------
    # _format_value keeps exact results exact — at the default precision N()
    # answered `17!` with 3.556874281e+14, and no value a caller could
    # reasonably pick from the arg description gave the integer.
    try:
        return _success(executor, _format_value(expr, precision))
    except Exception as e:
        return _fail(executor, f"Numeric evaluation failed: {e}")
