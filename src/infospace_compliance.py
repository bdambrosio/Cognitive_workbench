"""
Infospace Compliance Tracker - Monitors plan execution for type compatibility violations.

Used for evaluation mode to track whether planner respects:
- Note vs Collection operation compatibility
- Allowed tool restrictions
- Type system rules
"""

import logging
from typing import Dict, List, Any, Optional, Set

logger = logging.getLogger(__name__)


# Operation compatibility table
NOTE_ONLY_OPS = {"expand", "as-json", "refine"}
COLLECTION_ONLY_OPS = {"map", "flatten"}
BOTH_OPS = {"summarize", "relate"}


class ComplianceTracker:
    """
    Tracks type compatibility violations during plan execution.
    
    Monitors:
    - Type compatibility (Note-only ops on Collections, etc.)
    - Successful type checks
    """
    
    def __init__(self):
        """Initialize compliance tracker."""
        self.eval_mode = True
        
        # Tracking lists
        self.violations: List[Dict[str, Any]] = []
        self.checks: List[Dict[str, Any]] = []
        
        # Variable type tracking: $var_name -> "Note" | "Collection"
        self.variable_types: Dict[str, str] = {}
        
        logger.info("ComplianceTracker initialized")
    
    def check_action(self, action: Dict, result: Dict, plan_bindings: Dict[str, str]):
        """
        Check action for type compatibility violations.
        
        Args:
            action: Action dict being executed
            result: Execution result dict
            plan_bindings: Current variable bindings ($var -> resource_id)
        """
        action_type = action.get('type', '')
        
        # Track variable types from successful operations
        if result.get('status') == 'success':
            out_var = action.get('out', '')
            if out_var:
                var_name = out_var.lstrip('$')
                resource_id = plan_bindings.get(var_name)
                if resource_id:
                    if resource_id.startswith('Note_'):
                        self.variable_types[var_name] = 'Note'
                    elif resource_id.startswith('Collection_'):
                        self.variable_types[var_name] = 'Collection'
        
        # Check type compatibility for operations with target
        target_var = action.get('target', '')
        if target_var and target_var.startswith('$'):
            var_name = target_var.lstrip('$')
            var_type = self.variable_types.get(var_name)
            
            if var_type:
                # Check Note-only operations
                if action_type in NOTE_ONLY_OPS and var_type == 'Collection':
                    self.violations.append({
                        'type': 'note_only_on_collection',
                        'operation': action_type,
                        'variable': target_var,
                        'variable_type': var_type,
                        'action': action.copy()
                    })
                    logger.warning(f"Compliance VIOLATION: {action_type} requires Note, got Collection ({target_var})")
                
                # Check Collection-only operations
                elif action_type in COLLECTION_ONLY_OPS and var_type == 'Note':
                    self.violations.append({
                        'type': 'collection_only_on_note',
                        'operation': action_type,
                        'variable': target_var,
                        'variable_type': var_type,
                        'action': action.copy()
                    })
                    logger.warning(f"Compliance VIOLATION: {action_type} requires Collection, got Note ({target_var})")
                
                # Track successful checks
                elif action_type in (NOTE_ONLY_OPS | COLLECTION_ONLY_OPS | BOTH_OPS):
                    self.checks.append({
                        'operation': action_type,
                        'variable': target_var,
                        'variable_type': var_type,
                        'compatible': True
                    })
    
    def get_metrics(self) -> Dict[str, Any]:
        """
        Get compliance metrics summary.
        
        Returns:
            Dict with violation counts and details
        """
        return {
            'type_violations': [
                {
                    'type': v['type'],
                    'operation': v['operation'],
                    'variable': v['variable'],
                    'expected_type': 'Note' if v['type'] == 'note_only_on_collection' else 'Collection',
                    'actual_type': v['variable_type']
                }
                for v in self.violations
            ],
            'compatibility_checks': len(self.checks),
            'evaluation_mode': self.eval_mode
        }
    
    def reset(self):
        """Clear all tracking data for next test."""
        self.violations.clear()
        self.checks.clear()
        self.variable_types.clear()

