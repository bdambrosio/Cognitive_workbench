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
            'load': self._execute_load,
            'index': self._execute_index,
            'organize': self._execute_index,  # Alias for index
            'search': self._execute_search,
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
            'map': self._execute_map,
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
        Optional: value (input), args (additional arguments), out (output binding)
        
        Argument types:
        - target: literal string (tool name) OR $variable (resolves to tool name)
        - value: literal string/value OR $variable (resolves to Note/Collection content)
        - args: dict of additional arguments (resolved if they're variables)
        - out: literal string (variable name, no $ prefix)
        """
        target = self._resolve_value(action.get('target'))
        value = self._resolve_value(action.get('value', ''))
        reason = action.get('reason', '')
        additional_args = action.get('args', {})
        out_var = action.get('out')
        
        if not target:
            return {'status': 'failed', 'reason': 'apply requires target'}
        
        # Apply the operation using shared helper
        result = self._apply_operation_to_value(target, value, reason, additional_args)
        
        if result.get('status') != 'success':
            return result
        
        # Get result value
        result_value = result.get('value')
        if out_var:
            info_id = self._create_info(content=result_value, name=out_var)
            self._bind_variable(out_var, info_id)
            logger.info(f"Tool executed, result → ${out_var}")
            return {'status': 'success', 'value': info_id}
        
        return {'status': 'success', 'value': result_value}
    
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
        Create a Note or Collection object as a spatial resource.
        
        Required: type, out
        Optional: kind ("Note" or "Collection"), value, name
        
        Creates both:
        1. Local plan binding (ephemeral, for plan execution)
        2. Spatial resource via map_node (persistent for Notes, session-local for Collections)
        
        Collections store resource IDs (references to Notes/Collections).
        """
        out_var = action.get('out') or action.get('name')
        kind = action.get('kind', 'Note')
        value_arg = action.get('value', None)
        collection_name = action.get('name')  # Optional stable name for Collections
        
        if not out_var:
            return {'status': 'failed', 'reason': 'create requires out'}
        
        if kind not in ['Note', 'Collection']:
            return {'status': 'failed', 'reason': f'Invalid kind: {kind}, must be Note or Collection'}
        
        # Handle Collections specially - convert variables to resource IDs
        if kind == 'Collection':
            if value_arg is None:
                resource_ids = []  # Empty collection
            elif isinstance(value_arg, list):
                # Convert list of $variables to resource IDs
                resource_ids = []
                for item in value_arg:
                    if isinstance(item, str) and item.startswith('$'):
                        # Resolve variable to resource ID
                        var_name = item[1:]
                        if var_name in self.plan_bindings:
                            resource_id = self.plan_bindings[var_name]
                            resource_ids.append(resource_id)
                        else:
                            return {'status': 'failed', 'reason': f'Unbound variable: {item}'}
                    else:
                        # Must be literal - create Note to wrap it
                        note_id = self._create_info(content=item, name=f"item_{len(resource_ids)}", kind='Note')
                        resource_ids.append(note_id)
                        logger.info(f"Auto-wrapped literal in Note: {note_id}")
            elif isinstance(value_arg, str) and value_arg.startswith('$'):
                # Single variable - resolve it
                resolved = self._resolve_value(value_arg)
                if isinstance(resolved, list):
                    # Recursively process list
                    return self._execute_create({
                        **action,
                        'value': resolved
                    })
                else:
                    # Single item - wrap in list
                    var_name = value_arg[1:]
                    if var_name in self.plan_bindings:
                        resource_ids = [self.plan_bindings[var_name]]
                    else:
                        return {'status': 'failed', 'reason': f'Unbound variable: {value_arg}'}
            else:
                return {'status': 'failed', 'reason': 'Collection value must be list or $variable'}
            
            # Validate all items are valid resource IDs
            validation_error = self._validate_collection_content(resource_ids)
            if validation_error:
                return {'status': 'failed', 'reason': validation_error}
            
            spatial_content = resource_ids
            format_type = 'list'
            create_topic = f"cognitive/map/collection/create"
            
        else:  # Note
            value = self._resolve_value(value_arg) if value_arg is not None else ''
            format_type = 'json' if isinstance(value, (dict, list)) else 'text'
            create_topic = f"cognitive/map/note/create"
            spatial_content = value  # Send raw content - map_node handles metadata
        
        # Create spatial resource via map_node
        try:
            from zenoh import QueryTarget, ConsolidationMode
            for reply in self.session.get(
                create_topic,
                target=QueryTarget.BEST_MATCHING,
                consolidation=ConsolidationMode.NONE,
                timeout=5.0,
                payload=json.dumps({
                    'character_name': self.agent_name,
                    'content': spatial_content,
                    'format': format_type,
                    'source_skill': 'create_primitive',
                    'source_value': str(spatial_content)[:100],
                    'collection_name': collection_name  # Pass name for Collections
                }).encode('utf-8')
            ):
                if reply.ok:
                    response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if response.get('success'):
                        info_id = response.get('info_id')
                        # Bind variable to resource ID
                        self._bind_variable(out_var, info_id)
                        logger.info(f"Created {kind} spatial resource → ${out_var} = {info_id}")
                        return {'status': 'success', 'value': info_id}
                    else:
                        logger.error(f'Failed to create {kind} resource: {response.get("error")}')
                        return {'status': 'failed', 'reason': response.get('error', 'Unknown error')}
                break
        except Exception as e:
            logger.error(f'Error creating {kind} spatial resource: {e}')
            return {'status': 'failed', 'reason': str(e)}
        
        # Fallback: create local only if spatial creation failed
        logger.warning(f"Spatial {kind} creation failed, creating local only")
        info_id = self._create_info(content=spatial_content, name=out_var, kind=kind)
        self._bind_variable(out_var, info_id)
        return {'status': 'success', 'value': info_id}
    
    # ==================== Storage Operations ====================
    
    def _execute_save(self, action: Dict) -> Dict:
        """
        Store value by creating Note object as a spatial resource.
        
        Required: value, out (or variable)
        
        Argument types:
        - value: literal value OR $variable (resolves to content)
        - out: literal string (variable name, no $ prefix)
        
        Creates a persistent Note object containing the value.
        Map_node handles all metadata (created_at, created_by, etc.).
        
        Special case: null values bind to the distinguished Note_null singleton.
        """
        value = self._resolve_value(action.get('value'))
        out_var = action.get('out') or action.get('variable')
        
        if not out_var:
            return {'status': 'failed', 'reason': 'save requires out'}
        
        # Bind null values to distinguished Note_null singleton
        if value is None:
            self._bind_variable(out_var, "Note_null")
            logger.info(f"Saved null value → ${out_var} = Note_null")
            return {'status': 'success', 'value': "Note_null"}
        
        # Determine format type
        format_type = 'json' if isinstance(value, (dict, list)) else 'text'
        
        # Create spatial Note resource via map_node
        try:
            from zenoh import QueryTarget, ConsolidationMode
            for reply in self.session.get(
                f"cognitive/map/note/create",
                target=QueryTarget.BEST_MATCHING,
                consolidation=ConsolidationMode.NONE,
                timeout=5.0,
                payload=json.dumps({
                    'character_name': self.agent_name,
                    'content': value,
                    'format': format_type,
                    'source_skill': 'save_primitive',
                    'source_value': str(value)[:100]
                }).encode('utf-8')
            ):
                if reply.ok:
                    response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if response.get('success'):
                        info_id = response.get('info_id')
                        # Bind variable to resource ID
                        self._bind_variable(out_var, info_id)
                        logger.info(f"Saved Note spatial resource → ${out_var} = {info_id}")
                        return {'status': 'success', 'value': info_id}
                    else:
                        logger.error(f'Failed to create Note resource: {response.get("error")}')
                        return {'status': 'failed', 'reason': response.get('error', 'Unknown error')}
                break
        except Exception as e:
            logger.error(f'Error creating Note spatial resource: {e}')
            return {'status': 'failed', 'reason': str(e)}
        
        # Fallback: create local only if spatial creation failed
        logger.warning(f"Spatial Note creation failed, creating local only")
        info_id = self._create_info(content=value, name=out_var, kind='Note')
        self._bind_variable(out_var, info_id)
        return {'status': 'success', 'value': info_id}
    
    def _execute_load(self, action: Dict) -> Dict:
        """
        Load a persistent Note or Collection by resource ID.
        
        Required: resource_id, out
        
        Retrieves an existing spatial resource from the map and binds it to a variable.
        """
        resource_id = action.get('resource_id')
        out_var = action.get('out')
        
        if not resource_id:
            return {'status': 'failed', 'reason': 'load requires resource_id'}
        
        if not out_var:
            return {'status': 'failed', 'reason': 'load requires out'}
        
        # Determine resource type from ID
        if resource_id.startswith('note_') or resource_id.startswith('Note_'):
            resource_type = 'note'
        elif resource_id.startswith('collection_') or resource_id.startswith('Collection_'):
            resource_type = 'collection'
        else:
            return {'status': 'failed', 'reason': f'Invalid resource_id format: {resource_id}'}
        
        # Query map_node for the resource using resource by name queryable
        try:
            from zenoh import QueryTarget, ConsolidationMode
            for reply in self.session.get(
                f"cognitive/map/resource/{resource_id}",
                target=QueryTarget.BEST_MATCHING,
                consolidation=ConsolidationMode.NONE,
                timeout=5.0
            ):
                if reply.ok:
                    response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if response.get('success'):
                        resource_data = response.get('resource')
                        # Extract content from properties field
                        content = resource_data.get('properties', {}).get('content')
                        kind = 'Note' if resource_type == 'note' else 'Collection'
                        
                        # Store in plan_bindings
                        self.plan_bindings[f"_content_{resource_id}"] = content
                        self.plan_bindings[f"_kind_{resource_id}"] = kind
                        self._bind_variable(out_var, resource_id)
                        
                        logger.info(f"Loaded {kind} {resource_id} → ${out_var}")
                        return {'status': 'success', 'value': resource_id}
                    else:
                        error_msg = response.get('error', 'Unknown error')
                        logger.error(f'Failed to load {resource_type}: {error_msg}')
                        return {'status': 'failed', 'reason': error_msg}
                break
        except Exception as e:
            logger.error(f'Error loading {resource_type} {resource_id}: {e}')
            return {'status': 'failed', 'reason': str(e)}
        
        return {'status': 'failed', 'reason': f'{resource_type.capitalize()} not found: {resource_id}'}
    
    def _execute_index(self, action: Dict) -> Dict:
        """
        Create searchable store with embeddings (also callable as 'organize').
        
        Required: type, source, store_name, index_type, fields
        
        Argument types:
        - source: $variable (Collection of Notes to index)
        - store_name: literal string (name of the index store)
        - index_type: literal string ('semantic' or 'keyword')
        - fields: dict specifying which fields to embed
        
        Dereferences Collection to get actual Note contents for indexing.
        """
        source_arg = action.get('source')
        store_name = action.get('store_name')
        index_type = action.get('index_type', 'semantic')
        fields = action.get('fields', {})
        
        if not source_arg or not store_name:
            return {'status': 'failed', 'reason': 'index requires source and store_name'}
        
        # Source should be a Collection variable
        if not isinstance(source_arg, str) or not source_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'index source must be $variable referencing a Collection'}
        
        collection_var = source_arg[1:]
        
        # Dereference the Collection to get actual Note contents
        dereferenced_notes = self._dereference_collection(collection_var)
        
        if not dereferenced_notes:
            logger.warning(f"Collection {collection_var} is empty or failed to dereference")
            return {'status': 'success', 'value': 0}  # Empty collection, nothing to index
        
        # Extract content from Note envelopes
        index_data = []
        for note in dereferenced_notes:
            if isinstance(note, dict) and 'content' in note:
                # Note envelope - extract content
                index_data.append(note['content'])
            else:
                # Raw data (shouldn't happen with reference model, but handle gracefully)
                index_data.append(note)
        
        # Request indexing from map_node
        request = {
            'agent_name': self.agent_name,
            'store_name': store_name,
            'source': index_data,
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
        logger.info(f"Indexed {indexed_count} items from Collection to {store_name}")
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
        
        # Publish think message to map (for memory system)
        message = {
            'agent_name': self.agent_name,
            'content': str(value)
        }
        
        self.session.put(
            f"map/{self.map_name}/think/{self.agent_name}",
            json.dumps(message)
        )
        
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
        Reduce Collection by predicate.
        
        Required: type, target, condition, out
        
        Filters a Collection of Notes, returns new Collection with matching Notes.
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'condition', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        condition = action.get('condition')
        out_var = action.get('out')
        
        # Target should be a Collection variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'filter target must be $variable referencing a Collection'}
        
        collection_var = target_arg[1:]
        
        # Get resource IDs from Collection
        if collection_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Unbound variable: {target_arg}'}
        
        collection_id = self.plan_bindings[collection_var]
        resource_ids = self.plan_bindings.get(f"_content_{collection_id}", [])
        
        # Dereference to get actual content for filtering
        dereferenced_notes = self._dereference_collection(collection_var)
        
        # Extract condition parameters
        field = condition.get('field')
        operator = condition.get('operator')
        value = self._resolve_value(condition.get('value'))
        
        if not field or not operator:
            return {'status': 'failed', 'reason': 'filter condition requires field and operator'}
        
        # Filter and track which resource IDs pass
        filtered_ids = []
        for i, note in enumerate(dereferenced_notes):
            # Extract content from Note envelope
            content = note.get('content') if isinstance(note, dict) else note
            item_value = content.get(field) if isinstance(content, dict) else content
            
            if self._apply_operator(item_value, operator, value):
                filtered_ids.append(resource_ids[i])
        
        # Create new Collection with filtered resource IDs
        info_id = self._create_info(content=filtered_ids, name=out_var, kind='Collection')
        self._bind_variable(out_var, info_id)
        logger.info(f"Filtered {len(resource_ids)} → {len(filtered_ids)} Notes → ${out_var}")
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
        Order Collection items by criteria.
        
        Required: type, target, by, out
        Optional: order, limit
        
        Sorts a Collection of Notes, returns new Collection with sorted Notes.
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'by', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        by = action.get('by')
        out_var = action.get('out')
        order = action.get('order', 'asc')
        limit = action.get('limit')
        
        # Target should be a Collection variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'sort target must be $variable referencing a Collection'}
        
        collection_var = target_arg[1:]
        
        # Get resource IDs from Collection
        if collection_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Unbound variable: {target_arg}'}
        
        collection_id = self.plan_bindings[collection_var]
        resource_ids = self.plan_bindings.get(f"_content_{collection_id}", [])
        
        # Dereference to get actual content for sorting
        dereferenced_notes = self._dereference_collection(collection_var)
        
        # Create list of (resource_id, note_content) pairs
        id_content_pairs = list(zip(resource_ids, dereferenced_notes))
        
        # Sort by field
        reverse = (order == 'desc')
        
        def get_sort_key(pair):
            resource_id, note = pair
            # Extract content from Note envelope
            content = note.get('content') if isinstance(note, dict) else note
            if isinstance(content, dict):
                return content.get(by, '')
            else:
                return content
        
        sorted_pairs = sorted(id_content_pairs, key=get_sort_key, reverse=reverse)
        
        # Extract sorted resource IDs
        sorted_ids = [pair[0] for pair in sorted_pairs]
        
        # Apply limit if specified
        if limit:
            sorted_ids = sorted_ids[:limit]
        
        # Create new Collection with sorted resource IDs
        info_id = self._create_info(content=sorted_ids, name=out_var, kind='Collection')
        self._bind_variable(out_var, info_id)
        logger.info(f"Sorted {len(resource_ids)} Notes by {by} → ${out_var}")
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
    
    def _execute_map(self, action: Dict) -> Dict:
        """
        Apply operation to each item in a Collection.
        
        Required: type, target, operation, out
        Optional: filter_null (bool), args (dict)
        
        operation can be:
        - String: tool name to apply to each item
        - Dict with "type": inline action (e.g., {"type": "extract", "field": "score"})
        - Dict with "tool": tool name with additional args
        
        Argument types:
        - target: $variable (Collection to map over)
        - operation: string or dict
        - args: dict of additional arguments (for tool operations)
        - filter_null: bool (exclude null/None results)
        - out: variable name for result Collection
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'operation', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        operation = action.get('operation')
        out_var = action.get('out')
        filter_null = action.get('filter_null', False)
        additional_args = action.get('args', {})
        
        # Target must be a Collection variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'map target must be $variable referencing a Collection'}
        
        collection_var = target_arg[1:]
        
        # Get resource IDs from Collection
        if collection_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Unbound variable: {target_arg}'}
        
        collection_id = self.plan_bindings[collection_var]
        resource_ids = self.plan_bindings.get(f"_content_{collection_id}", [])
        
        if not isinstance(resource_ids, list):
            return {'status': 'failed', 'reason': 'map target must be a Collection'}
        
        # Dereference to get actual content
        dereferenced_notes = self._dereference_collection(collection_var)
        
        # Apply operation to each item
        result_ids = []
        for i, note in enumerate(dereferenced_notes):
            # Extract content from Note envelope
            content = note.get('content') if isinstance(note, dict) else note
            
            # Apply operation based on type
            if isinstance(operation, str):
                # Tool name - apply to content
                result = self._apply_operation_to_value(operation, content, 
                                                       f"map item {i}", additional_args)
            elif isinstance(operation, dict):
                if 'tool' in operation:
                    # Dict with tool name and args
                    tool_name = operation['tool']
                    tool_args = operation.get('args', {})
                    tool_args.update(additional_args)  # Merge with action-level args
                    result = self._apply_operation_to_value(tool_name, content,
                                                           f"map item {i}", tool_args)
                elif 'type' in operation:
                    # Inline action - execute it with content as implicit target
                    inline_action = operation.copy()
                    # For extract, set target to content
                    if inline_action['type'] == 'extract':
                        inline_action['target'] = content
                        inline_action['out'] = f"_map_temp_{i}"
                        result = self._execute_extract(inline_action)
                        # Get the created info
                        if result.get('status') == 'success':
                            temp_var = f"_map_temp_{i}"
                            temp_id = self.plan_bindings.get(temp_var)
                            temp_content = self.plan_bindings.get(f"_content_{temp_id}")
                            result = {'status': 'success', 'value': temp_content}
                    else:
                        return {'status': 'failed', 
                               'reason': f"Inline action type '{inline_action['type']}' not supported in map"}
                else:
                    return {'status': 'failed', 'reason': 'operation dict must have "tool" or "type" field'}
            else:
                return {'status': 'failed', 'reason': 'operation must be string (tool name) or dict'}
            
            # Handle result
            if result.get('status') == 'success':
                result_value = result.get('value')
                # Create Note for result
                note_id = self._create_info(content=result_value, name=f"{out_var}_item_{i}", kind='Note')
                # Include unless filtering nulls and value is None
                if not (filter_null and result_value is None):
                    result_ids.append(note_id)
            else:
                # Map failed on this item
                logger.warning(f"Map failed on item {i}: {result.get('reason')}")
                if not filter_null:
                    # Create note_null reference (or skip if filtering)
                    result_ids.append("Note_null")
        
        # Create new Collection with result IDs
        collection_id = self._create_info(content=result_ids, name=out_var, kind='Collection')
        self._bind_variable(out_var, collection_id)
        logger.info(f"Mapped {len(dereferenced_notes)} → {len(result_ids)} items → ${out_var}")
        return {'status': 'success', 'value': collection_id}
    
    # ==================== Operation Application Helpers ====================
    
    def _apply_operation_to_value(self, tool_name: str, value: Any, reason: str = '', 
                                   additional_args: Dict = None) -> Dict:
        """
        Helper to apply a tool to a single value with optional additional arguments.
        Shared by apply and map primitives.
        
        Args:
            tool_name: Name of the tool/skill to invoke
            value: Input value to the tool
            reason: Reason for invoking (for logging)
            additional_args: Optional dict of additional arguments for the tool
            
        Returns:
            Result dict with 'status' and 'value' fields
        """
        # Resolve any variables in additional args
        resolved_args = {}
        if additional_args:
            for key, val in additional_args.items():
                resolved_args[key] = self._resolve_value(val)
        
        # Build tool request
        request = {
            'agent_name': self.agent_name,
            'tool_name': tool_name,
            'input_value': value,
            'reason': reason,
            'additional_args': resolved_args  # Pass extra args to tool executor
        }
        
        self.session.put(
            f"map/{self.map_name}/tool_request/{self.agent_name}",
            json.dumps(request)
        )
        
        # Wait for response
        response = self._wait_for_response(
            f"map/{self.map_name}/tool_response/{self.agent_name}",
            timeout=5.0
        )
        
        if not response:
            return {'status': 'failed', 'reason': 'Tool execution timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Tool execution failed')}
        
        return {'status': 'success', 'value': response.get('result')}
    
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
    
    def _validate_collection_content(self, items: List[str]) -> Optional[str]:
        """
        Validate that all items in a Collection are valid resource IDs.
        
        Args:
            items: List of strings that should be resource IDs
            
        Returns:
            Error message if invalid, None if valid
        """
        if not isinstance(items, list):
            return "Collection content must be a list"
        
        for item in items:
            if not isinstance(item, str):
                return f"Collection item must be string resource ID, got {type(item).__name__}"
            
            # Validate resource ID format: note_xxx, Note_xxx, collection_xxx, Collection_xxx
            if not re.match(r'^(note_|Note_|collection_|Collection_)[a-zA-Z0-9]+$', item):
                return f"Invalid resource ID format: {item}"
        
        return None
    
    def _dereference_collection(self, collection_var: str) -> List[Dict]:
        """
        Dereference a Collection to load all referenced Notes/Collections.
        
        Args:
            collection_var: Variable name (without $) of the Collection
            
        Returns:
            List of dereferenced Note/Collection contents
        """
        if collection_var not in self.plan_bindings:
            logger.warning(f"Collection variable not bound: {collection_var}")
            return []
        
        collection_id = self.plan_bindings[collection_var]
        resource_ids = self.plan_bindings.get(f"_content_{collection_id}", [])
        
        if not isinstance(resource_ids, list):
            logger.warning(f"Collection content is not a list: {collection_id}")
            return []
        
        # Dereference each resource ID
        dereferenced = []
        for resource_id in resource_ids:
            # Check if already in plan_bindings
            content = self.plan_bindings.get(f"_content_{resource_id}")
            if content is not None:
                dereferenced.append(content)
            else:
                # Need to load from map_node
                loaded = self._load_resource_by_id(resource_id)
                if loaded is not None:
                    dereferenced.append(loaded)
                else:
                    logger.warning(f"Failed to dereference resource: {resource_id}")
        
        return dereferenced
    
    def _load_resource_by_id(self, resource_id: str) -> Optional[Any]:
        """
        Load a resource by ID from map_node.
        
        Args:
            resource_id: Resource ID (note_xxx or collection_xxx)
            
        Returns:
            Resource content or None if not found
        """
        # Determine resource type
        if resource_id.startswith('note_') or resource_id.startswith('Note_'):
            resource_type = 'note'
        elif resource_id.startswith('collection_') or resource_id.startswith('Collection_'):
            resource_type = 'collection'
        else:
            logger.error(f"Invalid resource ID format: {resource_id}")
            return None
        
        # Query map_node using resource by name queryable
        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(
            f"cognitive/map/resource/{resource_id}",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=5.0
        ):
            if reply.ok:
                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if response.get('success'):
                    resource_data = response.get('resource')
                    # Extract content from properties field
                    content = resource_data.get('properties', {}).get('content')
                    # Cache in plan_bindings
                    self.plan_bindings[f"_content_{resource_id}"] = content
                    kind = 'Note' if resource_type == 'note' else 'Collection'
                    self.plan_bindings[f"_kind_{resource_id}"] = kind
                    return content
            break
        
        return None
    
    def _wrap_note_content(self, content: Any, name: str = None) -> Dict:
        """
        Wrap content in consistent Note envelope schema.
        
        Args:
            content: The actual data to store
            name: Variable name for the Note
            
        Returns:
            Dict with consistent Note schema
        """
        from datetime import datetime
        
        # Determine content type
        if isinstance(content, dict):
            content_type = 'json'
        elif isinstance(content, list):
            content_type = 'list'
        elif isinstance(content, (int, float)):
            content_type = 'number'
        elif isinstance(content, bool):
            content_type = 'boolean'
        else:
            content_type = 'text'
        
        return {
            'name': name or 'unnamed',
            'created': datetime.now().isoformat(),
            'creator': self.agent_name,
            'content': content,
            'content_type': content_type
        }
    
    def _create_info(self, content: Any, name: str = None, kind: str = 'Note') -> str:
        """
        Create Note or Collection object locally (fallback), return info_id.
        
        Args:
            content: The data to store (raw, not wrapped)
            name: Optional variable name for logging
            kind: 'Note' or 'Collection' (default: 'Note')
        
        Returns:
            info_id string
        """
        import uuid
        prefix = 'note_' if kind == 'Note' else 'collection_'
        info_id = f"{prefix}{uuid.uuid4().hex[:8]}"
        
        # Store raw content and type
        self.plan_bindings[f"_content_{info_id}"] = content
        self.plan_bindings[f"_kind_{info_id}"] = kind
        
        logger.info(f"Created {kind} {info_id}" + (f" '{name}'" if name else ""))
        return info_id
    
    def _resolve_value(self, value: Any) -> Any:
        """
        Resolve $variable to Note/Collection content.
        
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
            # Map_node stores raw content - return as-is
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
            'map': {
                'type': 'map',
                'target': '$test_filtered',
                'operation': {'type': 'extract', 'field': 'val'},
                'out': 'test_mapped',
                'prediction': 'test map'
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

