"""
Infospace Planner - Generates plans using information space primitives.

Provides templates and planning logic for cognitive/information processing tasks.
"""

import json
import logging
from typing import Dict, List, Any

logger = logging.getLogger(__name__)


class InfospacePlanner:
    """
    Generates plans for information space operations.
    Uses infospace-specific templates and primitives.
    """
    
    def __init__(self, llm_client, available_tools: Dict[str, Dict] = None, logger=None):
        """
        Initialize infospace planner.
        
        Args:
            llm_client: LLM client for plan generation
            available_tools: Dict of tool_name -> metadata (from tool_loader)
            logger: Optional logger instance
        """
        self.llm_client = llm_client
        self.available_tools = available_tools or {}
        self.logger = logger or logging.getLogger(__name__)
    
    def _validate_plan(self, plan: Dict) -> str:
        """
        Validate plan structure.
        Check required fields per action type.
        
        Returns:
            String with ACTION/REPAIR format errors (empty if valid)
        """
        errors = []
        logger.info(f"Validating plan")
        if 'plan' not in plan:
            errors.append("ACTION 0:\nREPAIR: Insert missing 'plan' field key at start of plan")
            return "\n".join(errors)
        
        actions = plan['plan']
        if not isinstance(actions, list):
            errors.append("ACTION 0:\nREPAIR: 'plan' field value must be a list of actions")
            return "\n".join(errors)
        
        if len(actions) > 12:
            self.logger.warning(f'Plan has {len(actions)} steps (recommended max: 12)')
        
        # Validate each action - collect all errors
        for i, action in enumerate(actions):
            if not isinstance(action, dict):
                errors.append(f"ACTION {i}:\nREPAIR: Action must be a dict/object with 'type' field")
                continue
            
            if 'type' not in action:
                errors.append(f"ACTION {i}:\nREPAIR: Add 'type' field specifying action or tool name")
                continue
            
            # Check required fields per type
            action_type = action['type']
            validation_error = self._validate_action_fields(action_type, action, i)
            if validation_error:
                errors.append(validation_error)
        
        return "\n".join(errors)
    
    def _validate_action_fields(self, action_type: str, action: Dict, action_index: int) -> str:
        """
        Validate required fields for action type.
        
        Returns:
            String in ACTION/REPAIR format if error found, empty string if valid
        """
        # Known primitive action types with their required fields
        required_fields = {
            'apply': ['target', 'out'],
            'focus': ['target'],
            'create-note': ['value', 'out'],
            'create-collection': ['value', 'out'],
            'persist': ['target'],
            'load': ['resource_id', 'out', 'expect'],
            'index': ['source'],
            'search': ['source', 'query', 'out', 'expect'],
            'split': ['target', 'out'],
            'flatten': ['target', 'out'],
            'coerce': ['target', 'operation', 'out'],
            'add': ['target', 'value', 'out'],
            'remove': ['target', 'value', 'out'],
            'size': ['target', 'out'],
            'union': ['target', 'value', 'out'],
            'intersection': ['target', 'value', 'out'],
            'difference': ['target', 'value', 'out'],
            'map': ['target', 'operation', 'out'],
            'if': ['condition', 'then'],
            'while': ['condition', 'body'],
            'wait': ['condition'],
            'say': ['target', 'value'],
            'display': ['value'],  # Accepts target or value (both allowed)
            'think': ['value'],  # Accepts target or value (both allowed)
            'ask': ['value', 'out'],
            # Level 4: Structured data operations
            'project': ['target', 'fields', 'out'],
            'pluck': ['target', 'field', 'out'],
            'filter-structured': ['target', 'where', 'out'],
            'sort': ['target', 'by', 'out'],
            'join': ['target', 'value', 'out'],
        }
        
        required = required_fields.get(action_type, [])
        
        # If action_type is not a known primitive, treat it as a tool name
        if not required:
            # Validate tool name exists in available_tools if provided
            if self.available_tools and action_type not in self.available_tools:
                return f"ACTION {action_index}: {action_type}\nREPAIR: Unknown tool name '{action_type}'. Check available tools or use correct primitive name"
            
            # Check if tool has custom parameter source (e.g., args.query)
            tool_info = self.available_tools.get(action_type, {})
            param_source = tool_info.get('parameter_source')
            
            if param_source and param_source.startswith('args.'):
                # Tool uses args.* for input parameter
                param_name = param_source.split('.', 1)[1]
                if 'args' not in action or param_name not in action.get('args', {}):
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'args' field with '{param_name}' parameter: \"args\": {{\"{param_name}\": \"your input\"}}"
                if 'out' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'out' field with $variable syntax: \"out\": \"$results\""
                if 'expect' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'expect' field describing expected outcome: \"expect\": \"should find relevant results\""
                # Validate out field uses $variable syntax
                out_val = action.get('out', '')
                if not isinstance(out_val, str) or not out_val.startswith('$'):
                    return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
                return ""
            
            # LLM-based tools require expect
            llm_tools = ['summarize', 'relate', 'refine', 'assess', 'extract-entities', 'filter-collection']
            if action_type in llm_tools:
                if 'target' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'target' field referencing the variable to process: \"target\": \"$variable_name\""
                if 'out' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'out' field with $variable syntax: \"out\": \"$result\""
                if 'expect' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'expect' field describing expected outcome: \"expect\": \"should extract/transform...\""
                out_val = action.get('out', '')
                if not isinstance(out_val, str) or not out_val.startswith('$'):
                    return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
                return ""
            
            # Other tools need target and out
            if 'target' not in action:
                return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'target' field referencing the variable to process: \"target\": \"$variable_name\""
            if 'out' not in action:
                return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'out' field with $variable syntax: \"out\": \"$result\""
            # Validate out field uses $variable syntax
            out_val = action.get('out', '')
            if not isinstance(out_val, str) or not out_val.startswith('$'):
                return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
            return ""
        
        # Check required fields for known primitives
        for field in required:
            if field not in action:
                # Special handling: display and think accept either value or target
                if field == 'value' and action_type in ['display', 'think']:
                    if 'target' not in action:
                        return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'value' or 'target' field (either works for {action_type})"
                else:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add required field '{field}'"
        
        # Validate 'out' field format if present
        if 'out' in action:
            out_val = action.get('out', '')
            if not isinstance(out_val, str) or not out_val.startswith('$'):
                return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
        
        return ""
    
    def verify_plan(self, plan: Dict) -> str:
        """
        Validate plan structure.
        Public instance method for plan validation.
        
        Args:
            plan: Plan dict with 'plan' key
            
        Returns:
            String with ACTION/REPAIR format errors (empty if valid)
        """
        return self._validate_plan(plan)

