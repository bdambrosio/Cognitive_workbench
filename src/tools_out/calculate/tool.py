"""
Calculate tool - evaluate mathematical expressions using SymPy.
"""

from sympy import sympify, N, pi, E, sin, cos, tan, asin, acos, atan, sqrt, log, ln, exp, Abs, factorial, solve, Eq
from sympy.core.sympify import SympifyError


def tool(value, runtime=None, **kwargs):
    """
    Evaluate mathematical expression.
    
    Args:
        value: Mathematical expression string
        variables: Optional dict of variable substitutions
        precision: Decimal places (default: 10)
    
    Returns:
        Numeric result as string
    """
    if not isinstance(value, str):
        return {
            'status': 'failed',
            'reason': 'value must be a string expression'
        }
    
    variables = kwargs.get('variables', {})
    precision = kwargs.get('precision', 10)
    
    # Convert equations (=) to Eq() format before sympify
    if '=' in value and '==' not in value and '!=' not in value:
        # Simple check: if there's a single = (not == or !=), convert to Eq()
        parts = value.split('=', 1)
        if len(parts) == 2:
            value = f"Eq({parts[0].strip()}, {parts[1].strip()})"
    
    # Build local namespace for sympify
    local_dict = {
        'pi': pi,
        'E': E,
        'sin': sin,
        'cos': cos,
        'tan': tan,
        'asin': asin,
        'acos': acos,
        'atan': atan,
        'sqrt': sqrt,
        'log': log,
        'ln': ln,
        'exp': exp,
        'abs': Abs,
        'factorial': factorial,
        'solve': solve,
        'Eq': Eq,
    }
    
    expr = sympify(value, locals=local_dict)
    
    # Substitute variables if provided
    if variables:
        expr = expr.subs(variables)
    
    # Check if this is a solve() call - solve() returns a list
    # We need to handle it specially before calling N()
    if hasattr(expr, 'func') and expr.func == solve:
        # Execute solve() directly to get the list of solutions
        # Get the arguments from the function call
        if hasattr(expr, 'args'):
            # Evaluate solve() with the arguments
            # expr.args[0] is the equation, expr.args[1] is the variable
            if len(expr.args) >= 2:
                solutions = solve(expr.args[0], expr.args[1])
            else:
                solutions = solve(expr.args[0])
            if isinstance(solutions, list):
                solutions = [str(N(sol, precision)) for sol in solutions]
                return ', '.join(solutions)
    
    # Try to evaluate numerically
    # If the expression contains solve(), N() will execute it and get a list,
    # then fail when trying to call .evalf() on the list
    try:
        result = N(expr, precision)
        return str(result)
    except AttributeError as e:
        # N() evaluated the expression and got a list (from solve())
        # The error occurs when N() tries to call .evalf() on the list
        if 'evalf' in str(e):
            # Try to evaluate using .doit() if available
            if hasattr(expr, 'doit'):
                evaluated = expr.doit()
                if isinstance(evaluated, list):
                    solutions = [str(N(sol, precision)) for sol in evaluated]
                    return ', '.join(solutions)
            # Fallback: evaluate the original expression string
            # Note: this won't have variable substitutions, but it's a fallback
            evaluated = eval(value, {"__builtins__": {}}, {**local_dict, **variables})
            if isinstance(evaluated, list):
                solutions = [str(N(sol, precision)) for sol in evaluated]
                return ', '.join(solutions)
            # If it's not a list, try N() again on the evaluated result
            return str(N(evaluated, precision))
        raise

