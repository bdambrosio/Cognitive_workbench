"""
Infospace Executor - Executes cognitive/information space primitives.

Handles data operations, storage, and cognitive workflows for agents
operating in information spaces (semantic/skill spaces).
"""

import json
import time
import logging
import re
from typing import Dict, List, Any, Optional

logger = logging.getLogger(__name__)


class InfospaceExecutor:
    """
    Executor for information space primitives.
    
    Phase 1 Primitives:
    - Core: scan, use, move
    - Storage: store, index, search
    - Control: if, while, wait
    """
    
    def __init__(self, agent_name: str, session, map_name: str):
        """
        Initialize infospace executor.
        
        Args:
            agent_name: Name of the agent
            session: Zenoh session for communication
            map_name: Name of the map (for Zenoh topics)
        """
        self.agent_name = agent_name
        self.session = session
        self.map_name = map_name
        
        # Plan-local state (ephemeral, cleared each plan)
        self.variables = {}      # $var_name -> value
        self.collections = {}    # collection_name -> [items]
        
        # Agent state
        self.agent_position = None
        self.visible_skills = {}
        
        logger.info(f"InfospaceExecutor initialized for {agent_name}")
    
    def clear_plan_state(self):
        """Clear ephemeral plan state (call at start of new plan)"""
        self.variables = {}
        self.collections = {}
    
    def execute_action(self, action: Dict) -> Dict:
        """
        Execute a single infospace action.
        
        Args:
            action: Action dict with 'type' field
            
        Returns:
            Result dict with 'status' field ('success', 'failed', 'retry')
        """
        action_type = action.get('type')
        
        if not action_type:
            logger.error("Action missing 'type' field")
            return {'status': 'failed', 'reason': 'Missing type field'}
        
        # Route to appropriate handler
        handlers = {
            'scan': self._execute_scan,
            'use': self._execute_use,
            'move': self._execute_move,
            'store': self._execute_store,
            'index': self._execute_index,
            'search': self._execute_search,
            'if': self._execute_if,
            'while': self._execute_while,
            'wait': self._execute_wait,
        }
        
        handler = handlers.get(action_type)
        if not handler:
            logger.error(f"Unknown action type: {action_type}")
            return {'status': 'failed', 'reason': f'Unknown action: {action_type}'}
        
        return handler(action)
    
    # ==================== Core Operations ====================
    
    def _execute_scan(self, action: Dict) -> Dict:
        """
        Scan for resource/skill by name or interface type.
        
        Required: type, target, out, prediction
        """
        target = action.get('target')
        out_var = action.get('out')
        
        if not target or not out_var:
            return {'status': 'failed', 'reason': 'scan requires target and out'}
        
        # Query map for matching resources
        query = {
            'agent_name': self.agent_name,
            'target': target,
            'scan_type': 'skill'  # In infospace, we scan for skills
        }
        
        # Publish scan request
        self.session.put(
            f"map/{self.map_name}/scan_request",
            json.dumps(query)
        )
        
        # Wait for response (with timeout)
        response = self._wait_for_response(
            f"map/{self.map_name}/scan_response/{self.agent_name}",
            timeout=5.0
        )
        
        if not response:
            return {'status': 'failed', 'reason': 'Scan timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Scan failed')}
        
        # Bind result to output variable
        result = response.get('result')
        self._bind_variable(out_var, result)
        
        logger.info(f"Scan found: {result}")
        return {'status': 'success', 'value': result}
    
    def _execute_use(self, action: Dict) -> Dict:
        """
        Apply skill to input data.
        
        Required: type, target, reason, prediction
        Optional: value (input), out (output binding)
        """
        target = self._resolve_value(action.get('target'))
        value = self._resolve_value(action.get('value', ''))
        reason = action.get('reason', '')
        out_var = action.get('out')
        
        if not target:
            return {'status': 'failed', 'reason': 'use requires target'}
        
        # Request skill execution from map
        request = {
            'agent_name': self.agent_name,
            'skill_name': target,
            'input_value': value,
            'reason': reason
        }
        
        self.session.put(
            f"map/{self.map_name}/skill_request",
            json.dumps(request)
        )
        
        # Wait for response
        response = self._wait_for_response(
            f"map/{self.map_name}/skill_response/{self.agent_name}",
            timeout=30.0  # Skills may take longer
        )
        
        if not response:
            return {'status': 'failed', 'reason': 'Skill execution timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Skill failed')}
        
        # Bind result if output specified
        result = response.get('result')
        if out_var:
            self._bind_variable(out_var, result)
        
        logger.info(f"Skill {target} executed, result bound to {out_var}")
        return {'status': 'success', 'value': result}
    
    def _execute_move(self, action: Dict) -> Dict:
        """
        Move to resource location.
        
        Required: type, target
        """
        target = self._resolve_value(action.get('target'))
        
        if not target:
            return {'status': 'failed', 'reason': 'move requires target'}
        
        # Request movement from map
        request = {
            'agent_name': self.agent_name,
            'target': target
        }
        
        self.session.put(
            f"map/{self.map_name}/move_request",
            json.dumps(request)
        )
        
        # Wait for response
        response = self._wait_for_response(
            f"map/{self.map_name}/move_response/{self.agent_name}",
            timeout=5.0
        )
        
        if not response:
            return {'status': 'failed', 'reason': 'Move timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Move failed')}
        
        logger.info(f"Moved to {target}")
        return {'status': 'success', 'value': target}
    
    # ==================== Storage Operations ====================
    
    def _execute_store(self, action: Dict) -> Dict:
        """
        Store value to variable or append to collection.
        
        Modes:
        1. store to variable: {type: store, value: X, variable: name}
        2. append to collection: {type: store, target: X, collection: name}
        """
        # Mode 1: Simple variable assignment
        if 'variable' in action:
            value = self._resolve_value(action.get('value'))
            var_name = action['variable']
            self._bind_variable(var_name, value)
            logger.info(f"Stored to ${var_name}")
            return {'status': 'success', 'value': value}
        
        # Mode 2: Append to collection
        if 'collection' in action:
            target = self._resolve_value(action.get('target'))
            collection_name = action['collection']
            
            if collection_name not in self.collections:
                self.collections[collection_name] = []
            
            self.collections[collection_name].append(target)
            logger.info(f"Appended to collection {collection_name}")
            return {'status': 'success', 'value': target}
        
        return {'status': 'failed', 'reason': 'store requires variable or collection'}
    
    def _execute_index(self, action: Dict) -> Dict:
        """
        Create searchable store with embeddings.
        
        Required: type, source, store_name, index_type, fields
        """
        source = self._resolve_value(action.get('source'))
        store_name = action.get('store_name')
        index_type = action.get('index_type', 'semantic')
        fields = action.get('fields', {})
        
        if not source or not store_name:
            return {'status': 'failed', 'reason': 'index requires source and store_name'}
        
        # Ensure source is a list
        if not isinstance(source, list):
            source = [source]
        
        # Request indexing from map_node
        request = {
            'agent_name': self.agent_name,
            'store_name': store_name,
            'source': source,
            'index_type': index_type,
            'fields': fields
        }
        
        self.session.put(
            f"map/{self.map_name}/index_request",
            json.dumps(request)
        )
        
        # Wait for response
        response = self._wait_for_response(
            f"map/{self.map_name}/index_response/{self.agent_name}",
            timeout=30.0  # Embedding generation takes time
        )
        
        if not response:
            return {'status': 'failed', 'reason': 'Index timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Index failed')}
        
        indexed_count = response.get('indexed_count', 0)
        logger.info(f"Indexed {indexed_count} items to {store_name}")
        return {'status': 'success', 'value': indexed_count}
    
    def _execute_search(self, action: Dict) -> Dict:
        """
        Query local indexed store.
        
        Required: type, store_name, query, mode, limit, out, prediction
        """
        store_name = action.get('store_name')
        query = self._resolve_value(action.get('query'))
        mode = action.get('mode', 'semantic')
        limit = action.get('limit', 5)
        threshold = action.get('threshold', 0.0)
        out_var = action.get('out')
        
        if not store_name or not query or not out_var:
            return {'status': 'failed', 'reason': 'search requires store_name, query, and out'}
        
        # Request search from map_node
        request = {
            'agent_name': self.agent_name,
            'store_name': store_name,
            'query': query,
            'mode': mode,
            'limit': limit,
            'threshold': threshold
        }
        
        self.session.put(
            f"map/{self.map_name}/search_request",
            json.dumps(request)
        )
        
        # Wait for response
        response = self._wait_for_response(
            f"map/{self.map_name}/search_response/{self.agent_name}",
            timeout=10.0
        )
        
        if not response:
            return {'status': 'failed', 'reason': 'Search timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Search failed')}
        
        # Bind results to output variable
        results = response.get('results', [])
        self._bind_variable(out_var, results)
        
        logger.info(f"Search found {len(results)} results")
        return {'status': 'success', 'value': results}
    
    # ==================== Control Flow ====================
    
    def _execute_if(self, action: Dict) -> Dict:
        """
        Conditional branching.
        
        Required: type, condition, then
        Optional: else
        """
        condition = action.get('condition')
        then_branch = action.get('then', [])
        else_branch = action.get('else', [])
        
        if not condition:
            return {'status': 'failed', 'reason': 'if requires condition'}
        
        # Evaluate condition
        condition_result = self._evaluate_condition(condition)
        
        # Execute appropriate branch
        branch = then_branch if condition_result else else_branch
        
        for step in branch:
            result = self.execute_action(step)
            if result.get('status') == 'failed':
                return result
        
        return {'status': 'success', 'value': condition_result}
    
    def _execute_while(self, action: Dict) -> Dict:
        """
        Loop while condition holds.
        
        Required: type, condition, body
        """
        condition = action.get('condition')
        body = action.get('body', [])
        max_iterations = action.get('max_iterations', 100)
        
        if not condition:
            return {'status': 'failed', 'reason': 'while requires condition'}
        
        iteration = 0
        while self._evaluate_condition(condition) and iteration < max_iterations:
            for step in body:
                result = self.execute_action(step)
                if result.get('status') == 'failed':
                    return result
            iteration += 1
        
        if iteration >= max_iterations:
            logger.warning(f"While loop hit max iterations: {max_iterations}")
        
        return {'status': 'success', 'value': iteration}
    
    def _execute_wait(self, action: Dict) -> Dict:
        """
        Block until condition satisfied.
        
        Required: type, condition
        Optional: timeout
        """
        condition = action.get('condition')
        timeout = action.get('timeout', 60.0)
        
        if not condition:
            return {'status': 'failed', 'reason': 'wait requires condition'}
        
        start_time = time.time()
        check_interval = 0.5  # Check every 500ms
        
        while time.time() - start_time < timeout:
            if self._evaluate_condition(condition):
                return {'status': 'success', 'value': True}
            time.sleep(check_interval)
        
        logger.warning(f"Wait timeout after {timeout}s")
        return {'status': 'failed', 'reason': 'Wait timeout'}
    
    # ==================== Condition Evaluation ====================
    
    def _evaluate_condition(self, condition: Dict) -> bool:
        """
        Evaluate condition dict, return True/False.
        
        Supported condition types:
        - bound/notbound: variable existence
        - has_value/empty: value truthiness
        - equals: value comparison
        - near/can_see: inherited from physical (basic support)
        """
        cond_type = condition.get('type')
        
        if not cond_type:
            logger.error("Condition missing type")
            return False
        
        handlers = {
            'bound': self._eval_bound,
            'notbound': self._eval_notbound,
            'has_value': self._eval_has_value,
            'empty': self._eval_empty,
            'equals': self._eval_equals,
            'near': self._eval_near,
            'can_see': self._eval_can_see,
        }
        
        handler = handlers.get(cond_type)
        if not handler:
            logger.error(f"Unknown condition type: {cond_type}")
            return False
        
        return handler(condition)
    
    def _eval_bound(self, condition: Dict) -> bool:
        """Check if variable is bound"""
        target = condition.get('target', '')
        if target.startswith('$'):
            target = target[1:]
        return target in self.variables
    
    def _eval_notbound(self, condition: Dict) -> bool:
        """Check if variable is not bound"""
        return not self._eval_bound(condition)
    
    def _eval_has_value(self, condition: Dict) -> bool:
        """Check if variable has truthy value"""
        target = self._resolve_value(condition.get('target'))
        return bool(target)
    
    def _eval_empty(self, condition: Dict) -> bool:
        """Check if variable is empty/falsy"""
        return not self._eval_has_value(condition)
    
    def _eval_equals(self, condition: Dict) -> bool:
        """Check value equality"""
        target = self._resolve_value(condition.get('target'))
        value = self._resolve_value(condition.get('value'))
        return target == value
    
    def _eval_near(self, condition: Dict) -> bool:
        """Check proximity (basic support for compatibility)"""
        # For infospace, "near" means at same location
        # Would need position info from map
        logger.warning("near condition not fully implemented for infospace")
        return True
    
    def _eval_can_see(self, condition: Dict) -> bool:
        """Check visibility (basic support for compatibility)"""
        # For infospace, assume visibility based on scan results
        logger.warning("can_see condition not fully implemented for infospace")
        return True
    
    # ==================== Utility Methods ====================
    
    def _resolve_value(self, value: Any) -> Any:
        """
        Resolve value - if starts with $, lookup in variables.
        Supports nested access: $var.field.subfield
        """
        if not isinstance(value, str):
            return value
        
        if not value.startswith('$'):
            return value
        
        # Simple variable: $var_name
        if '.' not in value:
            var_name = value[1:]
            if var_name not in self.variables:
                logger.warning(f"Unbound variable: {value}")
                return None
            return self.variables[var_name]
        
        # Nested access: $var.field.subfield
        parts = value[1:].split('.')
        current = self.variables.get(parts[0])
        
        if current is None:
            logger.warning(f"Unbound variable: ${parts[0]}")
            return None
        
        for part in parts[1:]:
            if isinstance(current, dict):
                current = current.get(part)
            elif isinstance(current, list) and part.isdigit():
                idx = int(part)
                if 0 <= idx < len(current):
                    current = current[idx]
                else:
                    logger.warning(f"Index out of range: {value}")
                    return None
            else:
                logger.warning(f"Cannot resolve: {value}")
                return None
        
        return current
    
    def _bind_variable(self, var_name: str, value: Any):
        """Store value in plan variables"""
        # Remove $ prefix if present
        if var_name.startswith('$'):
            var_name = var_name[1:]
        
        self.variables[var_name] = value
        logger.debug(f"Bound ${var_name} = {type(value).__name__}")
    
    def _wait_for_response(self, topic: str, timeout: float = 5.0) -> Optional[Dict]:
        """
        Wait for Zenoh response on topic.
        
        Args:
            topic: Zenoh topic to listen on
            timeout: Max wait time in seconds
            
        Returns:
            Response dict or None if timeout
        """
        response_data = [None]
        
        def handler(sample):
            response_data[0] = json.loads(sample.payload)
        
        # Subscribe and wait
        subscriber = self.session.subscribe(topic, handler)
        
        start_time = time.time()
        while response_data[0] is None and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        subscriber.close()
        
        return response_data[0]

