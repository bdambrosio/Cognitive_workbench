"""
Calculate tool - numerically evaluate mathematical expressions using SymPy.
"""

from sympy import (
    sympify, N, pi, E,
    sin, cos, tan, asin, acos, atan,
    sqrt, log, ln, exp, Abs, factorial,
    solve, Eq
)
from sympy.core.sympify import SympifyError


def tool(value, runtime=None, **kwargs):
    """
    Evaluate a mathematical expression numerically.

    Args:
        value (str): Mathematical expression
        variables (dict): Optional numeric substitutions
        precision (int): Decimal places (default: 10)

    Returns:
        str on success
        dict with status='failed' on error
    """

    # ----------------------------
    # Validate input
    # ----------------------------
    if not isinstance(value, str):
        return {
            "status": "failed",
            "reason": "value must be a string expression"
        }

    variables = kwargs.get("variables", {}) or {}
    precision = kwargs.get("precision", 10)

    if not isinstance(variables, dict):
        return {
            "status": "failed",
            "reason": "variables must be a dictionary"
        }

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
    if "=" in value and "==" not in value and "!=" not in value:
        parts = value.split("=", 1)
        if len(parts) == 2:
            value = f"Eq({parts[0].strip()}, {parts[1].strip()})"

    # ----------------------------
    # Parse expression
    # ----------------------------
    try:
        expr = sympify(value, locals=local_dict)
    except SympifyError as e:
        return {
            "status": "failed",
            "reason": f"Invalid mathematical expression: {e}"
        }

    # ----------------------------
    # Substitute variables
    # ----------------------------
    try:
        expr = expr.subs(variables)
    except Exception as e:
        return {
            "status": "failed",
            "reason": f"Variable substitution failed: {e}"
        }

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
                return ""

            return ", ".join(str(N(sol, precision)) for sol in solutions)

        except Exception as e:
            return {
                "status": "failed",
                "reason": f"Equation solving failed: {e}"
            }

    # ----------------------------
    # Reject unresolved symbols
    # ----------------------------
    free_symbols = getattr(expr, "free_symbols", set())
    if free_symbols:
        return {
            "status": "failed",
            "reason": f"Unresolved symbols in expression: {', '.join(map(str, free_symbols))}"
        }

    # ----------------------------
    # Numeric evaluation
    # ----------------------------
    try:
        result = N(expr, precision)
        return str(result)
    except Exception as e:
        return {
            "status": "failed",
            "reason": f"Numeric evaluation failed: {e}"
        }
