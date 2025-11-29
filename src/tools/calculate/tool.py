"""
Calculate tool - evaluate mathematical expressions using SymPy.
"""

from sympy import sympify, N, pi, E, sin, cos, tan, asin, acos, atan, sqrt, log, ln, exp, Abs, factorial
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
    }
    
    expr = sympify(value, locals=local_dict)
    
    # Substitute variables if provided
    if variables:
        expr = expr.subs(variables)
    
    # Evaluate numerically
    result = N(expr, precision)
    
    return str(result)

