"""
Infospace Executor - Executes cognitive/information space primitives.

Handles data operations, storage, and cognitive workflows for agents
operating in information spaces (semantic/tool spaces).
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
    - Core: scan, apply, move
    - Storage: save, index, search
    - Control: if, while, wait
    
    Phase 2 Primitives:
    - Data: extract, filter, merge, transform
    - Analysis: aggregate, sort, group_by, compare
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
        self.plan_bindings = {}  # $var_name -> info_id
        
        # Agent state
        self.agent_position = None
        self.visible_tools = {}
        
        logger.info(f"InfospaceExecutor initialized for {agent_name}")
    
    def clear_plan_state(self):
        """Clear ephemeral plan state (call at start of new plan)"""
        self.plan_bindings = {}
    
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
            # Phase 1: Core, Storage, Control, Communication
            'apply': self._execute_apply,
            'move': self._execute_move,
            'create': self._execute_create,
            'save': self._execute_save,
            'index': self._execute_index,
            'organize': self._execute_index,  # Alias for index
            'search': self._execute_search,
            'if': self._execute_if,
            'while': self._execute_while,
            'wait': self._execute_wait,
            'say': self._execute_say,
            'think': self._execute_think,
            # Phase 2: Data & Analysis
            'extract': self._execute_extract,
            'filter': self._execute_filter,
            'merge': self._execute_merge,
            'transform': self._execute_transform,
            'aggregate': self._execute_aggregate,
            'sort': self._execute_sort,
            'group_by': self._execute_group_by,
            'compare': self._execute_compare,
        }
        
        handler = handlers.get(action_type)
        if not handler:
            logger.error(f"Unknown action type: {action_type}")
            return {'status': 'failed', 'reason': f'Unknown action: {action_type}'}
        
        return handler(action)
    
    # ==================== Core Operations ====================
    
    def _execute_scan(self, action: Dict) -> Dict:
        """
        Scan for resource/tool by name or interface type.
        
        Required: type, target, out, prediction
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target = action.get('target')
        out_var = action.get('out')
        
        # Query map for matching resources
        query = {
            'agent_name': self.agent_name,
            'target': target,
            'scan_type': 'tool'
        }
        
        # Publish scan request
        self.session.put(f"map/{self.map_name}/scan_request/{self.agent_name}", json.dumps(query))
        
        # Wait for response (with timeout)
        response = self._wait_for_response(f"map/{self.map_name}/scan_response/{self.agent_name}", timeout=5.0)
        
        if not response:
            return {'status': 'failed', 'reason': 'Scan timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Scan failed')}
        
        # Get result and create Note for it
        result = response.get('result')
        
        # Create Note object for scanned resource
        info_id = self._create_info(content=result, name=out_var)
        self._bind_variable(out_var, info_id)
        
        logger.info(f"Scan found: {result['name'] if isinstance(result, dict) else result}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_apply(self, action: Dict) -> Dict:
        """
        Apply tool to input data.
        
        Required: type, target, reason, prediction
        Optional: value (input), out (output binding)
        
        Argument types:
        - target: literal string (tool name) OR $variable (resolves to tool name)
        - value: literal string/value OR $variable (resolves to Note/Collection content)
        - out: literal string (variable name, no $ prefix)
        """
        target = self._resolve_value(action.get('target'))
        value = self._resolve_value(action.get('value', ''))
        reason = action.get('reason', '')
        out_var = action.get('out')
        
        if not target:
            return {'status': 'failed', 'reason': 'apply requires target'}
        
        # Request tool execution from map
        request = {
            'agent_name': self.agent_name,
            'tool_name': target,
            'input_value': value,
            'reason': reason
        }
        
        self.session.put(
            f"map/{self.map_name}/tool_request/{self.agent_name}",
            json.dumps(request)
        )
        
        # Wait for response
        response = self._wait_for_response(
            f"map/{self.map_name}/tool_response/{self.agent_name}",
            timeout=5.0  # Tools may take longer
        )
        
        if not response:
            return {'status': 'failed', 'reason': 'Tool execution timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Tool execution failed')}
        
        # Get result and create Note
        result = response.get('result')
        if out_var:
            info_id = self._create_info(content=result, name=out_var)
            self._bind_variable(out_var, info_id)
            logger.info(f"Tool executed, result → ${out_var}")
            return {'status': 'success', 'value': info_id}
        
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
            f"map/{self.map_name}/move_request/{self.agent_name}",
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
    
    def _execute_create(self, action: Dict) -> Dict:
        """
        Create a Note or Collection object.
        
        Required: type, out
        Optional: kind ("Note" or "Collection"), value, name
        """
        out_var = action.get('out') or action.get('name')
        kind = action.get('kind', 'Note')
        value = self._resolve_value(action.get('value', None))
        
        if not out_var:
            return {'status': 'failed', 'reason': 'create requires out'}
        
        if kind not in ['Note', 'Collection']:
            return {'status': 'failed', 'reason': f'Invalid kind: {kind}, must be Note or Collection'}
        
        # Create typed info object
        if kind == 'Collection' and value is None:
            value = []  # Empty collection by default
        
        info_id = self._create_info(content=value, name=out_var, kind=kind)
        self._bind_variable(out_var, info_id)
        
        logger.info(f"Created {kind} → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    # ==================== Storage Operations ====================
    
    def _execute_save(self, action: Dict) -> Dict:
        """
        Store value by creating Note object.
        
        Required: value, out (or variable)
        
        Argument types:
        - value: literal value OR $variable (resolves to content)
        - out: literal string (variable name, no $ prefix)
        
        Creates a Note object containing the value.
        """
        value = self._resolve_value(action.get('value'))
        out_var = action.get('out') or action.get('variable')
        
        if not out_var:
            return {'status': 'failed', 'reason': 'save requires out'}
        
        # Create Note object
        info_id = self._create_info(content=value, name=out_var)
        
        # Bind variable to Note
        self._bind_variable(out_var, info_id)
        
        logger.info(f"Stored to ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_index(self, action: Dict) -> Dict:
        """
        Create searchable store with embeddings (also callable as 'organize').
        
        Required: type, source, store_name, index_type, fields
        
        Argument types:
        - source: $variable (resolves to Collection/list content) OR literal list
        - store_name: literal string (name of the index store)
        - index_type: literal string ('semantic' or 'keyword')
        - fields: dict specifying which fields to embed
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
            f"map/{self.map_name}/index_request/{self.agent_name}",
            json.dumps(request)
        )
        
        # Wait for response
        response = self._wait_for_response(
            f"map/{self.map_name}/index_response/{self.agent_name}",
            timeout=5.0  # Embedding generation takes time
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
        
        Argument types:
        - store_name: literal string (name of indexed store)
        - query: literal string OR $variable (resolves to query text)
        - mode: literal string ('semantic' or 'keyword')
        - limit: int (max results to return)
        - out: literal string (variable name to store results)
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
            f"map/{self.map_name}/search_request/{self.agent_name}",
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
    
    def _execute_say(self, action: Dict) -> Dict:
        """
        Produce output (agent speech/communication).
        
        Required: type, value
        Optional: target (default: "user")
        """
        value = self._resolve_value(action.get('value'))
        target = action.get('target', 'user')
        
        if value is None:
            return {'status': 'failed', 'reason': 'say requires value'}
        
        # Publish say message to map
        message = {
            'agent_name': self.agent_name,
            'target': target,
            'content': str(value)
        }
        
        self.session.put(
            f"map/{self.map_name}/say/{self.agent_name}",
            json.dumps(message)
        )
        
        logger.info(f"Say [{target}]: {value}")
        return {'status': 'success', 'value': value}
    
    def _execute_think(self, action: Dict) -> Dict:
        """
        Internal thought/note (logged but not communicated externally).
        
        Required: type, value
        """
        value = self._resolve_value(action.get('value'))
        
        if value is None:
            return {'status': 'failed', 'reason': 'think requires value'}
        
        logger.info(f"Think: {value}")
        return {'status': 'success', 'value': value}
    
    # ==================== Phase 2: Data Operations ====================
    
    def _execute_extract(self, action: Dict) -> Dict:
        """
        Extract specific fields or elements from structured data.
        
        Required: type, target, field, out
        Optional: default
        """
        target = self._resolve_value(action.get('target'))
        field = action.get('field')
        out_var = action.get('out')
        default = action.get('default')
        
        if not target or not field or not out_var:
            return {'status': 'failed', 'reason': 'extract requires target, field, and out'}
        
        # Extract field value
        result = None
        
        # Handle nested field paths (e.g., "metadata.author")
        if '.' in field:
            parts = field.split('.')
            current = target
            for part in parts:
                if isinstance(current, dict):
                    current = current.get(part)
                elif isinstance(current, list) and part.isdigit():
                    idx = int(part)
                    current = current[idx] if 0 <= idx < len(current) else None
                else:
                    current = None
                    break
            result = current
        else:
            # Simple field extraction
            if isinstance(target, dict):
                result = target.get(field, default)
            elif isinstance(target, list):
                # Extract field from all items in list
                result = [item.get(field, default) if isinstance(item, dict) else None for item in target]
            else:
                result = default
        
        # Use default if extraction failed
        if result is None and default is not None:
            result = default
        
        info_id = self._create_info(content=result, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Extracted field '{field}' → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_filter(self, action: Dict) -> Dict:
        """
        Reduce collection by predicate.
        
        Required: type, target, condition, out
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'condition', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target = self._resolve_value(action.get('target'))
        condition = action.get('condition')
        out_var = action.get('out')
        
        # Validate type
        error = self._validate_type(target, (list,), 'target')
        if error:
            return {'status': 'failed', 'reason': error}
        
        # Extract condition parameters
        field = condition.get('field')
        operator = condition.get('operator')
        value = self._resolve_value(condition.get('value'))
        
        if not field or not operator:
            return {'status': 'failed', 'reason': 'filter condition requires field and operator'}
        
        # Filter based on operator
        filtered = []
        for item in target:
            item_value = item.get(field) if isinstance(item, dict) else item
            
            if self._apply_operator(item_value, operator, value):
                filtered.append(item)
        
        info_id = self._create_info(content=filtered, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Filtered {len(target)} → {len(filtered)} items → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_merge(self, action: Dict) -> Dict:
        """
        Combine multiple collections or objects.
        
        Required: type, targets, out
        Optional: strategy, deduplicate_by
        """
        targets = action.get('targets', [])
        out_var = action.get('out')
        strategy = action.get('strategy', 'append')
        deduplicate_by = action.get('deduplicate_by')
        
        if not targets or not out_var:
            return {'status': 'failed', 'reason': 'merge requires targets and out'}
        
        # Resolve all targets
        resolved_targets = []
        for target in targets:
            resolved = self._resolve_value(target)
            if resolved is not None:
                resolved_targets.append(resolved)
        
        if not resolved_targets:
            return {'status': 'failed', 'reason': 'No valid targets to merge'}
        
        # Merge based on strategy
        result = []
        
        if strategy == 'append':
            # Simple concatenation
            for target in resolved_targets:
                if isinstance(target, list):
                    result.extend(target)
                else:
                    result.append(target)
        
        elif strategy == 'deduplicate':
            # Remove duplicates
            seen = set()
            for target in resolved_targets:
                items = target if isinstance(target, list) else [target]
                for item in items:
                    # Deduplicate by field or by value
                    if deduplicate_by and isinstance(item, dict):
                        key = item.get(deduplicate_by)
                        if key not in seen:
                            seen.add(key)
                            result.append(item)
                    else:
                        # Try to use item as hashable key
                        item_str = json.dumps(item, sort_keys=True) if isinstance(item, (dict, list)) else str(item)
                        if item_str not in seen:
                            seen.add(item_str)
                            result.append(item)
        
        elif strategy in ['union', 'intersection']:
            # Set operations (treating lists as sets)
            sets = [set(json.dumps(item, sort_keys=True) if isinstance(item, (dict, list)) else str(item) for item in (target if isinstance(target, list) else [target])) for target in resolved_targets]
            
            if strategy == 'union':
                result_set = set.union(*sets) if sets else set()
            else:  # intersection
                result_set = set.intersection(*sets) if sets else set()
            
            # Convert back to list (json strings -> objects)
            result = [json.loads(item) if item.startswith(('{', '[')) else item for item in result_set]
        
        else:
            return {'status': 'failed', 'reason': f'Unknown merge strategy: {strategy}'}
        
        info_id = self._create_info(content=result, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Merged {len(resolved_targets)} → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_transform(self, action: Dict) -> Dict:
        """
        Convert data format or structure.
        
        Required: type, target, operation, out
        """
        target = self._resolve_value(action.get('target'))
        operation = action.get('operation')
        out_var = action.get('out')
        
        if not target or not operation or not out_var:
            return {'status': 'failed', 'reason': 'transform requires target, operation, and out'}
        
        result = None
        
        if operation == 'flatten':
            # Flatten nested lists
            if isinstance(target, list):
                result = []
                for item in target:
                    if isinstance(item, list):
                        result.extend(item)
                    else:
                        result.append(item)
            else:
                result = target
        
        elif operation == 'normalize':
            # Normalize to consistent format (dict list)
            if isinstance(target, dict):
                result = [target]
            elif isinstance(target, list):
                result = target
            else:
                result = [target]
        
        elif operation == 'pivot':
            # Simple pivot: list of dicts -> dict of lists by key
            if isinstance(target, list):
                result = {}
                for item in target:
                    if isinstance(item, dict):
                        for key, value in item.items():
                            if key not in result:
                                result[key] = []
                            result[key].append(value)
            else:
                result = target
        
        elif operation == 'reshape':
            # Reshape to single dict (merge all dict keys)
            if isinstance(target, list):
                result = {}
                for item in target:
                    if isinstance(item, dict):
                        result.update(item)
            else:
                result = target
        
        else:
            return {'status': 'failed', 'reason': f'Unknown transform operation: {operation}'}
        
        info_id = self._create_info(content=result, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Transformed ({operation}) → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    # ==================== Phase 2: Analysis Operations ====================
    
    def _execute_aggregate(self, action: Dict) -> Dict:
        """
        Reduce collection to single value.
        
        Required: type, target, operation, out
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'operation', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target = self._resolve_value(action.get('target'))
        operation = action.get('operation')
        out_var = action.get('out')
        
        # Validate type
        error = self._validate_type(target, (list,), 'target')
        if error:
            return {'status': 'failed', 'reason': error}
        
        result = None
        
        if operation == 'sum':
            result = sum(float(item) if isinstance(item, (int, float, str)) and str(item).replace('.', '').isdigit() else 0 for item in target)
        
        elif operation == 'average':
            numeric = [float(item) for item in target if isinstance(item, (int, float)) or (isinstance(item, str) and str(item).replace('.', '').isdigit())]
            result = sum(numeric) / len(numeric) if numeric else 0
        
        elif operation == 'count':
            result = len(target)
        
        elif operation == 'max':
            numeric = [float(item) for item in target if isinstance(item, (int, float))]
            result = max(numeric) if numeric else None
        
        elif operation == 'min':
            numeric = [float(item) for item in target if isinstance(item, (int, float))]
            result = min(numeric) if numeric else None
        
        elif operation == 'concat':
            result = ''.join(str(item) for item in target)
        
        elif operation == 'join':
            separator = action.get('separator', ', ')
            result = separator.join(str(item) for item in target)
        
        else:
            return {'status': 'failed', 'reason': f'Unknown aggregate operation: {operation}'}
        
        info_id = self._create_info(content=result, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Aggregated ({operation}) → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_sort(self, action: Dict) -> Dict:
        """
        Order items by criteria.
        
        Required: type, target, by, out
        Optional: order, limit
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'by', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target = self._resolve_value(action.get('target'))
        by = action.get('by')
        out_var = action.get('out')
        order = action.get('order', 'asc')
        limit = action.get('limit')
        
        # Validate type
        error = self._validate_type(target, (list,), 'target')
        if error:
            return {'status': 'failed', 'reason': error}
        
        # Sort by field or comparator
        reverse = (order == 'desc')
        
        if isinstance(by, str) and by.startswith('$'):
            # Sort using comparator variable (function)
            logger.warning("Comparator variables not fully supported in Phase 2")
            sorted_items = sorted(target, reverse=reverse)
        else:
            # Sort by field name
            def get_sort_key(item):
                if isinstance(item, dict):
                    return item.get(by, '')
                else:
                    return item
            
            sorted_items = sorted(target, key=get_sort_key, reverse=reverse)
        
        # Apply limit if specified
        if limit:
            sorted_items = sorted_items[:limit]
        
        info_id = self._create_info(content=sorted_items, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Sorted by {by} → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_group_by(self, action: Dict) -> Dict:
        """
        Partition collection by key or category.
        
        Required: type, target, by, out
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'by', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target = self._resolve_value(action.get('target'))
        by = action.get('by')
        out_var = action.get('out')
        
        # Validate type
        error = self._validate_type(target, (list,), 'target')
        if error:
            return {'status': 'failed', 'reason': error}
        
        # Group by field name
        grouped = {}
        
        for item in target:
            if isinstance(item, dict):
                key = item.get(by, 'unknown')
            else:
                key = 'items'
            
            key_str = str(key)
            if key_str not in grouped:
                grouped[key_str] = []
            
            grouped[key_str].append(item)
        
        info_id = self._create_info(content=grouped, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Grouped by {by} → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    def _execute_compare(self, action: Dict) -> Dict:
        """
        Side-by-side comparison between items.
        
        Required: type, targets, out
        Optional: dimensions
        """
        targets = action.get('targets', [])
        out_var = action.get('out')
        dimensions = action.get('dimensions', [])
        
        if not targets or not out_var:
            return {'status': 'failed', 'reason': 'compare requires targets and out'}
        
        # Resolve all targets
        resolved_targets = [self._resolve_value(t) for t in targets]
        
        # Build comparison structure
        comparison = {
            'items': resolved_targets,
            'count': len(resolved_targets)
        }
        
        # If dimensions specified, compare on those fields
        if dimensions:
            comparison['dimensions'] = {}
            for dim in dimensions:
                comparison['dimensions'][dim] = [
                    item.get(dim) if isinstance(item, dict) else None
                    for item in resolved_targets
                ]
        
        info_id = self._create_info(content=comparison, name=out_var)
        self._bind_variable(out_var, info_id)
        logger.info(f"Compared {len(resolved_targets)} items → ${out_var}")
        return {'status': 'success', 'value': info_id}
    
    # ==================== Helper Methods ====================
    
    def _apply_operator(self, item_value: Any, operator: str, compare_value: Any) -> bool:
        """Apply comparison operator to item value"""
        if operator == 'equals':
            return item_value == compare_value
        elif operator == 'not_equals':
            return item_value != compare_value
        elif operator == 'contains':
            return compare_value in str(item_value) if item_value else False
        elif operator == 'not_contains':
            return compare_value not in str(item_value) if item_value else True
        elif operator == 'matches':
            # Regex match
            return bool(re.match(str(compare_value), str(item_value))) if item_value else False
        elif operator in ['gt', 'greater_than']:
            return float(item_value) > float(compare_value) if isinstance(item_value, (int, float)) else False
        elif operator in ['lt', 'less_than']:
            return float(item_value) < float(compare_value) if isinstance(item_value, (int, float)) else False
        elif operator in ['gte', 'greater_than_equals']:
            return float(item_value) >= float(compare_value) if isinstance(item_value, (int, float)) else False
        elif operator in ['lte', 'less_than_equals']:
            return float(item_value) <= float(compare_value) if isinstance(item_value, (int, float)) else False
        elif operator == 'after':
            # Date comparison
            return str(item_value) > str(compare_value)
        elif operator == 'before':
            return str(item_value) < str(compare_value)
        else:
            logger.warning(f"Unknown operator: {operator}")
            return False
    
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
            # Phase 1 conditions
            'bound': self._eval_bound,
            'notbound': self._eval_notbound,
            'has_value': self._eval_has_value,
            'empty': self._eval_empty,
            'equals': self._eval_equals,
            'near': self._eval_near,
            'can_see': self._eval_can_see,
            # Phase 2 extended conditions
            'field_exists': self._eval_field_exists,
            'field_missing': self._eval_field_missing,
            'not_equals': self._eval_not_equals,
            'greater_than': self._eval_greater_than,
            'less_than': self._eval_less_than,
            'gte': self._eval_gte,
            'lte': self._eval_lte,
            'contains': self._eval_contains,
            'not_contains': self._eval_not_contains,
            'matches_pattern': self._eval_matches_pattern,
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
        return target in self.plan_bindings
    
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
    
    # Phase 2 extended condition evaluators
    
    def _eval_field_exists(self, condition: Dict) -> bool:
        """Check if field exists in structured data"""
        target = self._resolve_value(condition.get('target'))
        field = condition.get('field')
        
        if isinstance(target, dict):
            return field in target
        return False
    
    def _eval_field_missing(self, condition: Dict) -> bool:
        """Check if field is missing"""
        return not self._eval_field_exists(condition)
    
    def _eval_not_equals(self, condition: Dict) -> bool:
        """Check value inequality"""
        return not self._eval_equals(condition)
    
    def _eval_greater_than(self, condition: Dict) -> bool:
        """Numeric greater than"""
        target = self._resolve_value(condition.get('target'))
        value = self._resolve_value(condition.get('value'))
        
        if isinstance(target, (int, float)) and isinstance(value, (int, float)):
            return target > value
        return False
    
    def _eval_less_than(self, condition: Dict) -> bool:
        """Numeric less than"""
        target = self._resolve_value(condition.get('target'))
        value = self._resolve_value(condition.get('value'))
        
        if isinstance(target, (int, float)) and isinstance(value, (int, float)):
            return target < value
        return False
    
    def _eval_gte(self, condition: Dict) -> bool:
        """Numeric greater than or equal"""
        target = self._resolve_value(condition.get('target'))
        value = self._resolve_value(condition.get('value'))
        
        if isinstance(target, (int, float)) and isinstance(value, (int, float)):
            return target >= value
        return False
    
    def _eval_lte(self, condition: Dict) -> bool:
        """Numeric less than or equal"""
        target = self._resolve_value(condition.get('target'))
        value = self._resolve_value(condition.get('value'))
        
        if isinstance(target, (int, float)) and isinstance(value, (int, float)):
            return target <= value
        return False
    
    def _eval_contains(self, condition: Dict) -> bool:
        """Check if value contains substring or element"""
        target = self._resolve_value(condition.get('target'))
        value = self._resolve_value(condition.get('value'))
        
        if isinstance(target, str):
            return str(value) in target
        elif isinstance(target, list):
            return value in target
        elif isinstance(target, dict):
            return value in target.values()
        return False
    
    def _eval_not_contains(self, condition: Dict) -> bool:
        """Check if value does not contain substring or element"""
        return not self._eval_contains(condition)
    
    def _eval_matches_pattern(self, condition: Dict) -> bool:
        """Check if value matches regex pattern"""
        target = self._resolve_value(condition.get('target'))
        pattern = condition.get('pattern')
        
        if not pattern:
            return False
        
        return bool(re.match(pattern, str(target))) if target else False
    
    # ==================== Utility Methods ====================
    
    def _create_info(self, content: Any, name: str = None, kind: str = 'Note') -> str:
        """
        Create Note or Collection object, return info_id.
        
        Args:
            content: The data to store
            name: Optional variable name for logging
            kind: 'Note' or 'Collection' (default: 'Note')
        
        Returns:
            info_id string
        """
        import uuid
        info_id = f"info_{uuid.uuid4().hex[:8]}"
        
        # Store content and type
        self.plan_bindings[f"_content_{info_id}"] = content
        self.plan_bindings[f"_kind_{info_id}"] = kind
        
        logger.info(f"Created {kind} {info_id}" + (f" '{name}'" if name else ""))
        return info_id
    
    def _resolve_value(self, value: Any) -> Any:
        """
        Resolve $variable to Note/Collection content.
        Returns the content stored in the Note or Collection.
        
        Args:
            value: Can be a literal value, or "$variable" string
            
        Returns:
            Resolved content or the original value if not a variable
        """
        if not isinstance(value, str):
            return value
        
        if not value.startswith('$'):
            return value
        
        var_name = value[1:]
        if var_name not in self.plan_bindings:
            logger.warning(f"Unbound variable: {value}")
            return None
        
        info_id = self.plan_bindings[var_name]
        
        # Get content from Note/Collection
        content_key = f"_content_{info_id}"
        if content_key in self.plan_bindings:
            return self.plan_bindings[content_key]
        
        logger.warning(f"Note/Collection content not found for {info_id}")
        return None
    
    def _get_kind(self, var_name: str) -> Optional[str]:
        """
        Get the kind (Note or Collection) of a variable.
        
        Args:
            var_name: Variable name (with or without $ prefix)
            
        Returns:
            'Note', 'Collection', or None if not found
        """
        if var_name.startswith('$'):
            var_name = var_name[1:]
        
        if var_name not in self.plan_bindings:
            return None
        
        info_id = self.plan_bindings[var_name]
        kind_key = f"_kind_{info_id}"
        
        return self.plan_bindings.get(kind_key, 'Note')
    
    def _bind_variable(self, var_name: str, info_id: str):
        """Bind variable name to Note/Collection ID"""
        if var_name.startswith('$'):
            var_name = var_name[1:]
        
        self.plan_bindings[var_name] = info_id
        logger.debug(f"Bound ${var_name} → {info_id}")
    
    def _validate_required_fields(self, action: Dict, *fields) -> Optional[str]:
        """
        Validate that required fields are present in action.
        
        Args:
            action: The action dict
            fields: Field names to check
            
        Returns:
            Error message if validation fails, None if all fields present
        """
        missing = [f for f in fields if not action.get(f)]
        if missing:
            return f"Missing required field(s): {', '.join(missing)}"
        return None
    
    def _validate_type(self, value: Any, expected_types: tuple, field_name: str = "value") -> Optional[str]:
        """
        Validate that a value is of expected type(s).
        
        Args:
            value: Value to check
            expected_types: Tuple of expected types (e.g., (list, dict))
            field_name: Name for error message
            
        Returns:
            Error message if validation fails, None if valid
        """
        if value is None:
            return f"{field_name} is None"
        
        if not isinstance(value, expected_types):
            type_names = ', '.join(t.__name__ for t in expected_types)
            actual_type = type(value).__name__
            return f"{field_name} must be {type_names}, got {actual_type}"
        
        return None
    
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
            response_data[0] = json.loads(sample.payload.to_bytes().decode('utf-8'))
        
        # Subscribe and wait
        subscriber = self.session.declare_subscriber(topic, handler)
        
        start_time = time.time()
        while response_data[0] is None and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        # Undeclare subscriber
        subscriber.undeclare()
        
        return response_data[0]
    
    def test_primitives(self):
        """Test all infospace primitives with minimal synthetic actions"""
        logger.info("🧪 ========== INFOSPACE PRIMITIVES TEST ==========")
        
        # Save current state
        saved_bindings = self.plan_bindings.copy()
        
        results = {}
        
        # Define minimal test actions for each primitive
        test_actions = {
            'apply': {
                'type': 'apply',
                'target': 'question-decomposer',
                'value': 'how much wood can a wood chuck chuck?',
                'reason': 'test apply',
                'out': 'test_output',
                'prediction': 'test output'
            },
            'move': {
                'type': 'move',
                'target': 'question-decomposer',
                'prediction': 'test move'
            },
            'create': {
                'type': 'create',
                'kind': 'Note',
                'value': 'test note content',
                'out': 'test_note',
                'prediction': 'test create'
            },
            'save': {
                'type': 'save',
                'out': 'test_var',
                'value': 'test_data'
            },
            'index': {
                'type': 'index',
                'source': 'test_data',
                'store_name': 'test_store',
                'index_type': 'keyword'
            },
            'organize': {
                'type': 'organize',
                'source': 'test_data',
                'store_name': 'test_store_org',
                'index_type': 'keyword'
            },
            'search': {
                'type': 'search',
                'store_name': 'test_store',
                'query': 'test query',
                'mode': 'keyword',
                'out': 'test_results',
                'prediction': 'test search results'
            },
            'if': {
                'type': 'if',
                'condition': {'type': 'equals', 'target': 'test', 'value': 'test'},
                'then': [{'type': 'save', 'variable': 'test_result', 'value': 'success'}],
                'else': []
            },
            'while': {
                'type': 'while',
                'condition': {'type': 'empty', 'target': '$test_var'},
                'body': [{'type': 'save', 'variable': 'test_result', 'value': 'looped'}],
                'max_iterations': 1
            },
            'wait': {
                'type': 'wait',
                'condition': {'type': 'equals', 'target': 'ready', 'value': 'ready'},
                'timeout': 0.1
            },
            'say': {
                'type': 'say',
                'value': 'test output message',
                'target': 'user',
                'prediction': 'test say'
            },
            'think': {
                'type': 'think',
                'value': 'test internal thought',
                'prediction': 'test think'
            },
            'extract': {
                'type': 'extract',
                'target': {'test': 'value'},
                'field': 'test',
                'out': 'test_extracted',
                'prediction': 'test extract'
            },
            'filter': {
                'type': 'filter',
                'target': [{'val': 1}, {'val': 2}, {'val': 3}],
                'condition': {'field': 'val', 'operator': 'gt', 'value': 0},
                'out': 'test_filtered',
                'prediction': 'test filter'
            },
            'merge': {
                'type': 'merge',
                'targets': [{'a': 1}, {'b': 2}],
                'out': 'test_merged',
                'prediction': 'test merge'
            },
            'transform': {
                'type': 'transform',
                'target': [[1, 2], [3, 4]],
                'operation': 'flatten',
                'out': 'test_transformed',
                'prediction': 'test transform'
            },
            'aggregate': {
                'type': 'aggregate',
                'target': [1, 2, 3],
                'operation': 'count',
                'out': 'test_aggregated',
                'prediction': 'test aggregate'
            },
            'sort': {
                'type': 'sort',
                'target': [{'val': 3}, {'val': 1}, {'val': 2}],
                'by': 'val',
                'order': 'asc',
                'out': 'test_sorted',
                'prediction': 'test sort'
            },
            'group_by': {
                'type': 'group_by',
                'target': [{'type': 'a'}, {'type': 'b'}],
                'by': 'type',
                'out': 'test_grouped',
                'prediction': 'test group'
            },
            'compare': {
                'type': 'compare',
                'targets': ['test', 'test'],
                'out': 'test_compared',
                'prediction': 'test compare'
            }
        }
        
        # Execute each test
        start_time = time.time()
        for primitive_name, action in test_actions.items():
            test_start = time.time()
            result = self.execute_action(action)
            elapsed = time.time() - test_start
            
            status = result.get('status', 'unknown')
            results[primitive_name] = status
            
            if status == 'success':
                logger.info(f"🧪 TEST {primitive_name:12s}: ✅ success ({elapsed:.3f}s)")
            elif status == 'failed':
                reason = result.get('reason', 'unknown')
                logger.info(f"🧪 TEST {primitive_name:12s}: ❌ failed  ({elapsed:.3f}s) - {reason}")
            else:
                logger.info(f"🧪 TEST {primitive_name:12s}: ⚠️  {status:7s} ({elapsed:.3f}s)")
        
        # Summary
        total_time = time.time() - start_time
        passed = sum(1 for r in results.values() if r == 'success')
        failed = sum(1 for r in results.values() if r == 'failed')
        total = len(results)
        
        logger.info("🧪 ================================================")
        logger.info(f"🧪 TEST SUMMARY: {passed}/{total} passed, {failed} failed ({total_time:.2f}s total)")
        
        if failed > 0:
            failed_list = [name for name, status in results.items() if status == 'failed']
            logger.info(f"🧪 FAILED: {', '.join(failed_list)}")
        
        # Restore state
        self.plan_bindings = saved_bindings
        
        logger.info("🧪 ================================================")
        
        return results

