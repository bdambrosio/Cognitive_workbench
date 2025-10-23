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
    
    Core Primitives (whole-value operations only):
    - Computation: apply, map, transform
    - Storage: save, load, create
    - Indexing: index, search
    - Communication: say, think
    - Spatial: move
    
    Design principle: Content is opaque. Field-based operations delegated to tools.
    """
    
    def __init__(self, agent_name: str, session, map_name: str, llm_client, available_tools: Dict[str, Dict]):
        """
        Initialize infospace executor.
        
        Args:
            agent_name: Name of the agent
            session: Zenoh session for communication
            map_name: Name of the map (for Zenoh topics)
            llm_client: LLM client for tool execution
            available_tools: Dict of tool_name -> metadata (from tool_loader)
        """
        self.agent_name = agent_name
        self.session = session
        self.map_name = map_name
        self.llm_client = llm_client
        self.available_tools = available_tools
        
        # Plan-local state (ephemeral, cleared each plan)
        self.plan_bindings = {}  # $var_name -> info_id
        
        # Agent state
        self.agent_position = None
        
        logger.info(f"InfospaceExecutor initialized for {agent_name} with {len(available_tools)} tools")
    
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
            # Phase 2: Data Operations (whole-value only)
            'transform': self._execute_transform,
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
        
        # Bind result to variable
        self._bind_variable(out_var, result)
        
        logger.info(f"Scan found: {result['name'] if isinstance(result, dict) else result}")
        return {'status': 'success', 'value': result}
    
    def _execute_apply(self, action: Dict) -> Dict:
        """
        Apply tool to input data.
        
        Required: type, target, reason
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
        
        # Check if this is a prompt-augmentation tool (handled locally)
        tool_info = self._get_tool_info(target)
        
        if tool_info and tool_info.get('type') == 'prompt_augmentation':
            # Execute locally using LLM
            result = self._execute_prompt_tool(target, value, tool_info, additional_args)
        else:
            # Delegate to map node (external tool or code execution)
            result = self._apply_operation_to_value(target, value, reason, additional_args)
        
        if result.get('status') != 'success':
            return result
        
        # Bind result if output variable specified
        result_value = result.get('value')
        if out_var:
            self._bind_variable(out_var, result_value)
            logger.info(f"Tool '{target}' executed, result → ${out_var}")
        
        return {'status': 'success', 'value': result_value}

    def _get_tool_info(self, tool_name: str) -> Optional[Dict]:
        """
        Get tool metadata from available tools.
        
        Returns tool info dict with 'type', 'description', 'tool_md_content', etc.
        Returns None if tool not found.
        """
        return self.available_tools.get(tool_name)

    def _execute_prompt_tool(self, tool_name: str, input_value: Any, 
                            tool_info: Dict, additional_args: Dict) -> Dict:
        """
        Execute a prompt-augmentation tool using local LLM.
        
        Args:
            tool_name: Name of the tool
            input_value: Input data to process
            tool_info: Tool metadata including SKILL.md content
            additional_args: Optional parameters (e.g., length='brief')
            
        Returns:
            Result dict with 'status' and 'value'
        """
        try:
            # Extract SKILL.md content
            skill_content = tool_info.get('tool_md_content', '')
            if not skill_content:
                return {'status': 'failed', 'reason': f'No SKILL.md content for {tool_name}'}
            
            # Build prompt with tool instructions + input
            prompt_parts = [
                "You are executing a cognitive tool. Follow the instructions carefully.\n",
                "# TOOL INSTRUCTIONS\n",
                skill_content,
                "\n# INPUT\n",
                str(input_value),
            ]
            
            # Add additional args as context if provided
            if additional_args:
                prompt_parts.append("\n# PARAMETERS\n")
                for key, val in additional_args.items():
                    prompt_parts.append(f"{key}: {val}\n")
            
            prompt_parts.append("\n# OUTPUT\nProvide the result directly, following the tool's output format:")
            
            full_prompt = "".join(prompt_parts)
            
            # Call LLM directly
            llm_response = self.llm_client.generate(
                messages=[full_prompt],
                bindings={},
                max_tokens=1000,
                temperature=0.5
            )
            
            if not llm_response or not llm_response.text:
                return {'status': 'failed', 'reason': 'LLM returned empty response'}
            
            result_text = llm_response.text
            logger.info(f"Prompt tool '{tool_name}' completed ({len(result_text)} chars)")
            
            return {'status': 'success', 'value': result_text}
            
        except Exception as e:
            logger.error(f"Prompt tool execution failed: {e}")
            return {'status': 'failed', 'reason': str(e)}

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
        
        # Resolve values for Collection or Note
        if kind == 'Collection':
            if value_arg is None:
                collection_values = []  # Empty collection
            elif isinstance(value_arg, list):
                # Resolve each item (variable or literal)
                collection_values = [self._resolve_value(item) for item in value_arg]
            elif isinstance(value_arg, str) and value_arg.startswith('$'):
                # Single variable - resolve it
                resolved = self._resolve_value(value_arg)
                if isinstance(resolved, list):
                    collection_values = resolved
                else:
                    collection_values = [resolved]
            else:
                return {'status': 'failed', 'reason': 'Collection value must be list or $variable'}
            
            # Bind Collection as list of values
            self._bind_variable(out_var, collection_values)
            logger.info(f"Created Collection → ${out_var} ({len(collection_values)} items)")
            return {'status': 'success', 'value': collection_values}
            
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
                        # Bind variable to actual value (Note persistence is handled by map_node)
                        self._bind_variable(out_var, value)
                        logger.info(f"Created {kind} spatial resource → ${out_var}, persisted as {info_id}")
                        return {'status': 'success', 'value': value}
                    else:
                        logger.error(f'Failed to create {kind} resource: {response.get("error")}')
                        return {'status': 'failed', 'reason': response.get('error', 'Unknown error')}
                break
        except Exception as e:
            logger.error(f'Error creating {kind} spatial resource: {e}')
            return {'status': 'failed', 'reason': str(e)}
        
        # Fallback: bind value even if spatial creation failed
        logger.warning(f"Spatial {kind} creation failed, binding locally only")
        self._bind_variable(out_var, value)
        return {'status': 'success', 'value': value}
    
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
                        # Bind variable to actual value (Note persistence handled by map_node)
                        self._bind_variable(out_var, value)
                        logger.info(f"Saved Note → ${out_var}, persisted as {info_id}")
                        return {'status': 'success', 'value': value}
                    else:
                        logger.error(f'Failed to create Note resource: {response.get("error")}')
                        return {'status': 'failed', 'reason': response.get('error', 'Unknown error')}
                break
        except Exception as e:
            logger.error(f'Error creating Note spatial resource: {e}')
            return {'status': 'failed', 'reason': str(e)}
        
        # Fallback: bind value even if spatial creation failed
        logger.warning(f"Spatial Note creation failed, binding locally only")
        self._bind_variable(out_var, value)
        return {'status': 'success', 'value': value}
    
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
                        
                        # Bind actual content to variable
                        self._bind_variable(out_var, content)
                        
                        logger.info(f"Loaded {kind} {resource_id} → ${out_var}")
                        return {'status': 'success', 'value': content}
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
        
        self._bind_variable(out_var, result)
        logger.info(f"Transformed ({operation}) → ${out_var}")
        return {'status': 'success', 'value': result}
    
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
        
        # Get Collection values
        collection_values = self._dereference_collection(collection_var)
        
        if not isinstance(collection_values, list):
            return {'status': 'failed', 'reason': 'map target must be a Collection'}
        
        # Apply operation to each item
        result_values = []
        for i, item in enumerate(collection_values):
            # Apply operation based on type
            if isinstance(operation, str):
                # Tool name - apply to item
                result = self._apply_operation_to_value(operation, item, 
                                                       f"map item {i}", additional_args)
            elif isinstance(operation, dict):
                if 'tool' in operation:
                    # Dict with tool name and args
                    tool_name = operation['tool']
                    tool_args = operation.get('args', {})
                    tool_args.update(additional_args)  # Merge with action-level args
                    result = self._apply_operation_to_value(tool_name, item,
                                                           f"map item {i}", tool_args)
                else:
                    return {'status': 'failed', 'reason': 'operation dict must have "tool" field'}
            else:
                return {'status': 'failed', 'reason': 'operation must be string (tool name) or dict'}
            
            # Handle result
            if result.get('status') == 'success':
                result_value = result.get('value')
                # Include unless filtering nulls and value is None
                if not (filter_null and result_value is None):
                    result_values.append(result_value)
            else:
                # Map failed on this item
                logger.warning(f"Map failed on item {i}: {result.get('reason')}")
                if not filter_null:
                    result_values.append(None)
        
        # Bind new Collection (list of result values)
        self._bind_variable(out_var, result_values)
        logger.info(f"Mapped {len(collection_values)} → {len(result_values)} items → ${out_var}")
        return {'status': 'success', 'value': result_values}
    
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
            # Basic conditions (whole-value)
            'bound': self._eval_bound,
            'notbound': self._eval_notbound,
            'has_value': self._eval_has_value,
            'empty': self._eval_empty,
            'equals': self._eval_equals,
            'not_equals': self._eval_not_equals,
            'greater_than': self._eval_greater_than,
            'less_than': self._eval_less_than,
            'gte': self._eval_gte,
            'lte': self._eval_lte,
            'contains': self._eval_contains,
            'not_contains': self._eval_not_contains,
            'matches_pattern': self._eval_matches_pattern,
            # Spatial conditions (compatibility)
            'near': self._eval_near,
            'can_see': self._eval_can_see,
            # Tool-based conditions (delegate to external tools)
            'tool_condition': self._eval_tool_condition,
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
    
    def _eval_tool_condition(self, condition: Dict) -> bool:
        """
        Evaluate condition by invoking an external tool.
        
        Tool must return boolean or truthy value.
        
        Condition format:
        {
            "type": "tool_condition",
            "tool": "tool_name",
            "target": "$variable",
            "args": {...}  # Optional additional arguments
        }
        """
        tool_name = condition.get('tool')
        if not tool_name:
            logger.error("tool_condition requires 'tool' field")
            return False
        
        target = self._resolve_value(condition.get('target'))
        args = condition.get('args', {})
        
        # Invoke tool with target value
        result = self._apply_operation_to_value(
            tool_name=tool_name,
            value=target,
            reason='condition evaluation',
            additional_args=args
        )
        
        if result.get('status') != 'success':
            logger.warning(f"Tool condition '{tool_name}' failed: {result.get('reason')}")
            return False
        
        # Interpret result as boolean
        result_value = result.get('value')
        
        # Explicit boolean
        if isinstance(result_value, bool):
            return result_value
        
        # Truthy/falsy coercion
        return bool(result_value)
    
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
    
    def _dereference_collection(self, collection_var: str) -> List[Any]:
        """
        Get Collection contents (already a list of values).
        
        Args:
            collection_var: Variable name (without $) of the Collection
            
        Returns:
            List of values in the Collection
        """
        if collection_var not in self.plan_bindings:
            logger.warning(f"Collection variable not bound: {collection_var}")
            return []
        
        collection_values = self.plan_bindings[collection_var]
        
        if not isinstance(collection_values, list):
            logger.warning(f"Collection variable is not a list: {collection_var}")
            return []
        
        return collection_values
    
    
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
    
    def _resolve_value(self, value: Any) -> Any:
        """
        Resolve $variable to its bound value.
        
        Args:
            value: Can be a literal value, or "$variable" string
            
        Returns:
            Resolved value or the original value if not a variable
        """
        if not isinstance(value, str):
            return value
        
        if not value.startswith('$'):
            return value
        
        var_name = value[1:]
        if var_name not in self.plan_bindings:
            logger.warning(f"Unbound variable: {value}")
            return None
        
        return self.plan_bindings[var_name]
    
    def _get_kind(self, var_name: str) -> Optional[str]:
        """
        Get the kind (Note or Collection) of a variable by inspecting its type.
        
        Args:
            var_name: Variable name (with or without $ prefix)
            
        Returns:
            'Note', 'Collection', or None if not found
        """
        if var_name.startswith('$'):
            var_name = var_name[1:]
        
        if var_name not in self.plan_bindings:
            return None
        
        value = self.plan_bindings[var_name]
        # Collection is a list, Note is anything else
        return 'Collection' if isinstance(value, list) else 'Note'
    
    def _bind_variable(self, var_name: str, value: Any):
        """Bind variable name to actual value"""
        if var_name.startswith('$'):
            var_name = var_name[1:]
        
        self.plan_bindings[var_name] = value
        logger.debug(f"Bound ${var_name} → {type(value).__name__}")
    
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

