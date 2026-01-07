"""
Calculate tool - numerically evaluate mathematical expressions using SymPy.
"""

from typing import Any, Dict, Optional
from sympy import (
    sympify, N, pi, E,
    sin, cos, tan, asin, acos, atan,
    sqrt, log, ln, exp, Abs, factorial,
    solve, Eq
)
from sympy.core.sympify import SympifyError
from infospace_executor import InfospaceExecutor


def _fail(executor: InfospaceExecutor, reason: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, result: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=result, extra=extra)


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
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    if not isinstance(input_value, str):
        return _fail(executor, "input_value must be a string expression")

    variables = kwargs.get("variables", {}) or {}
    precision = kwargs.get("precision", 10)

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
    # Substitute variables
    # ----------------------------
    try:
        expr = expr.subs(variables)
    except Exception as e:
        return _fail(executor, f"Variable substitution failed: {e}")

    # ----------------------------
    # Handle solve(...)
    # ----------------------------
    if expr.func == solve:
        try:
            if len(expr.args) == 2:
                solutions = solve(expr.args[0], expr.args[1])
            else:
                solutions = solve(expr.args[0])

            if not solutions:
                return _success(executor, "")

            result = ", ".join(str(N(sol, precision)) for sol in solutions)
            return _success(executor, result)

        except Exception as e:
            return _fail(executor, f"Equation solving failed: {e}")

    # ----------------------------
    # Reject unresolved symbols
    # ----------------------------
    free_symbols = getattr(expr, "free_symbols", set())
    if free_symbols:
        symbols = ', '.join(map(str, free_symbols))
        return _fail(executor, f"Unresolved symbols in expression: {symbols}")

    # ----------------------------
    # Numeric evaluation
    # ----------------------------
    try:
        result = str(N(expr, precision))
        return _success(executor, result)
    except Exception as e:
        return _fail(executor, f"Numeric evaluation failed: {e}")
