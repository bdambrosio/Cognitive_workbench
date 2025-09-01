"""
Shared utilities for condition evaluation and variable dereferencing.
Consolidates logic used by both plan.py and executive_node.
"""

from typing import Dict, Any, Union, Tuple
import logging

logger = logging.getLogger(__name__)

def deref_plan_target(plan_bindings: Dict[str, Any], target: Any) -> Any:
    """
    Return bound value if target is a $variable, else the target unchanged.
    
    Args:
        plan_bindings: Dictionary of bound variables from plan
        target: Target to dereference (string or other type)
        
    Returns:
        Dereferenced target or original target if not a variable
    """
    try:
        if isinstance(target, str) and target.startswith('$'):
            var_name = target[1:]
            if var_name in plan_bindings:
                logger.debug(f'Dereferenced ${var_name} -> {plan_bindings[var_name]}')
                return plan_bindings[var_name]
            else:
                logger.warning(f'Variable ${var_name} not bound in plan_bindings')
                return target
    except Exception as e:
        logger.error(f'Error dereferencing target {target}: {e}')
        return target
    return target

def is_negated_condition(condition_type: str) -> bool:
    """
    Determine if a condition type is negated.
    
    Args:
        condition_type: The condition type string
        
    Returns:
        True if negated, False otherwise
    """
    negated_prefixes = ['not', 'hasnt', 'cant']
    return any(prefix in condition_type.lower() for prefix in negated_prefixes)

def normalize_condition_type(condition_type: str) -> Tuple[str, bool]:
    """
    Normalize condition type and extract negation.
    
    Args:
        condition_type: Raw condition type string
        
    Returns:
        Tuple of (normalized_type, is_negated)
    """
    negated = is_negated_condition(condition_type)
    
    # Remove negation prefixes to get base type
    if negated:
        for prefix in ['not', 'hasnt', 'cant']:
            if condition_type.lower().startswith(prefix):
                normalized = condition_type[len(prefix):].lower()
                break
        else:
            # Handle cases like 'notnear' -> 'near'
            if condition_type.lower().startswith('not'):
                normalized = condition_type[3:].lower()
            else:
                normalized = condition_type.lower()
    else:
        normalized = condition_type.lower()
    
    return normalized, negated

def validate_condition_structure(condition: Dict[str, Any]) -> bool:
    """
    Validate that a condition has the required structure.
    
    Args:
        condition: Condition dictionary to validate
        
    Returns:
        True if valid, False otherwise
    """
    if not condition:
        logger.error('No condition provided')
        return False
        
    if not isinstance(condition, dict):
        logger.error(f'Condition must be dict, got {type(condition)}')
        return False
        
    if 'type' not in condition:
        logger.error(f'Condition missing type: {condition}')
        return False
        
    # For most conditions, 'target' is required; for believes/notbelieves we now accept 'value' instead
    ctype = str(condition.get('type', '')).lower()
    if ctype in ('believes', 'notbelieves'):
        if 'value' not in condition:
            logger.error(f"Condition 'believes/notbelieves' missing value: {condition}")
            return False
    else:
        if 'target' not in condition:
            logger.error(f'Condition missing target: {condition}')
            return False
        
    return True
