"""
Infospace Executor - Executes cognitive/information space primitives.

Handles data operations, storage, and cognitive workflows for agents
operating in information spaces (semantic/tool spaces).
"""

import json
import time
import logging
import re
import importlib.util
from pathlib import Path
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
        self.plan_bindings = {}  # $var_name -> resource_id (Note_N or Collection_N)
        
        # Agent state (persistent across plans)
        self.agent_position = None
        
        logger.info(f"InfospaceExecutor initialized for {agent_name} with {len(available_tools)} tools")
    
    def clear_plan_state(self):
        """Clear ephemeral plan state (call at start of new plan)"""
        self.plan_bindings = {}
    
    def _create_collection(self, note_ids: list, source_context: str) -> Optional[str]:
        """
        Create a Collection resource in map_node.
        
        Args:
            note_ids: List of Note IDs (e.g., ["Note_2", "Note_3"])
            source_context: Description for logging (e.g., 'map_operation', 'create_collection')
            
        Returns:
            Collection ID if successful, None if failed
        """
        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(
            f"cognitive/map/collection/create",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=5.0,
            payload=json.dumps({
                'character_name': self.agent_name,
                'content': note_ids,  # List of Note IDs
                'format': 'list',
                'source_skill': source_context
            }).encode('utf-8')
        ):
            if reply.ok:
                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if response.get('success'):
                    return response.get('info_id')
                else:
                    logger.error(f'Failed to create Collection: {response.get("error")}')
            break
        return None
    
    def _get_content(self, resource_id: str) -> Any:
        """
        Fetch content from map_node for a given resource ID.
        
        Args:
            resource_id: Note_N or Collection_N ID
            
        Returns:
            Content value, or None if not found
        """
        if resource_id == "Note_null":
            return None
        
        # Query map_node for the resource
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
                    content = resource_data.get('properties', {}).get('content')
                    return content
                else:
                    logger.warning(f"Failed to fetch {resource_id}: {response.get('error')}")
                    return None
            break
        
        logger.warning(f"No response for {resource_id}")
        return None
    
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
            'flatten': self._execute_flatten,
            'add': self._execute_add,
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
        
        # Get tool info and route by type
        tool_info = self._get_tool_info(target)
        
        if not tool_info:
            return {'status': 'failed', 'reason': f'Tool not found: {target}'}
        
        tool_type = tool_info.get('type')
        
        if tool_type == 'prompt_augmentation':
            # Execute locally using LLM
            result = self._execute_prompt_tool(target, value, tool_info, additional_args)
        elif tool_type == 'python':
            # Execute Python tool
            result = self._execute_python_tool(target, value, tool_info, additional_args)
        else:
            return {'status': 'failed', 'reason': f'Unknown tool type: {tool_type}'}
        
        if result.get('status') != 'success':
            return result
        
        # Bind result if output variable specified
        result_value = result.get('value')
        if out_var:
            # Persist result as Note
            info_id = self._persist_note(result_value, f'apply_{target}')
            if info_id:
                self._bind_variable(out_var, info_id)
                logger.info(f"Tool '{target}' executed, Note {info_id} → ${out_var}")
                return {'status': 'success', 'value': info_id}
            else:
                logger.error(f"Failed to persist result from '{target}'")
                return {'status': 'failed', 'reason': 'Failed to persist result'}
        
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
            
            # Handle Collection input (list of Note IDs) - dereference and format
            if isinstance(input_value, list):
                formatted_notes = []
                for i, item in enumerate(input_value, 1):
                    if isinstance(item, str) and item.startswith('Note_'):
                        # It's a Note ID - fetch content
                        content = self._get_content(item)
                        formatted_notes.append(f"## Note {i}\n{content}\n")
                    else:
                        # It's already content
                        formatted_notes.append(f"## Item {i}\n{item}\n")
                input_str = '\n'.join(formatted_notes)
            else:
                input_str = str(input_value)
            
            # Build prompt with tool instructions + input
            prompt_parts = [
                "You are executing a cognitive tool. Follow the instructions carefully.\n",
                "# TOOL INSTRUCTIONS\n",
                skill_content,
                "\n# INPUT\n",
                input_str,
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

    def _execute_python_tool(self, tool_name: str, input_value: Any,
                            tool_info: Dict, additional_args: Dict) -> Dict:
        """
        Execute a Python-based tool by importing and calling tool() function.
        
        Args:
            tool_name: Name of the tool
            input_value: Input data to process
            tool_info: Tool metadata including python_file path
            additional_args: Optional parameters passed as **kwargs
            
        Returns:
            Result dict with 'status' and 'value'
        """
        # Check trust flag
        if not tool_info.get('trusted', False):
            return {'status': 'failed', 
                   'reason': f'Untrusted Python tool: {tool_name}. Set trusted: true in SKILL.md'}
        
        # Get Python file path
        python_file = tool_info.get('python_file')
        if not python_file:
            return {'status': 'failed', 
                   'reason': f'No Python file found for tool: {tool_name}'}
        
        python_path = Path(python_file)
        if not python_path.exists():
            return {'status': 'failed', 
                   'reason': f'Python file not found: {python_path}'}
        
        # Import module dynamically
        spec = importlib.util.spec_from_file_location(
            f"tool_{tool_name.replace('-', '_')}", 
            python_path
        )
        tool_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tool_module)
        
        # Get tool function
        if not hasattr(tool_module, 'tool'):
            return {'status': 'failed', 
                   'reason': f'No tool() function in {python_path.name}'}
        
        tool_func = tool_module.tool
        
        # Resolve any variables in additional args
        resolved_args = {}
        if additional_args:
            for key, val in additional_args.items():
                resolved_args[key] = self._resolve_value(val)
        
        # Execute tool
        result = tool_func(input_value, **resolved_args)
        
        # Handle result format
        if isinstance(result, dict) and 'status' in result:
            # Tool returned structured response
            logger.info(f"Python tool '{tool_name}' completed with status: {result.get('status')}")
            return result
        else:
            # Tool returned raw value
            logger.info(f"Python tool '{tool_name}' completed")
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
                note_ids = []  # Empty collection
            elif isinstance(value_arg, list):
                # Resolve each item - if it's a $var, get Note ID from bindings; if literal, create Note
                note_ids = []
                for i, item in enumerate(value_arg):
                    if isinstance(item, str) and item.startswith('$'):
                        # Variable - get Note ID from bindings
                        var_name = item[1:]
                        if var_name not in self.plan_bindings:
                            logger.warning(f"Variable {item} not bound, skipping")
                            continue
                        note_id = self.plan_bindings[var_name]
                        if isinstance(note_id, str) and note_id.startswith('Note_'):
                            note_ids.append(note_id)
                        else:
                            logger.warning(f"Variable {item} is not a Note, skipping")
                    else:
                        # Literal - create Note for it
                        note_id = self._persist_note(item, f'create_collection_item_{i}')
                        if note_id:
                            note_ids.append(note_id)
                        else:
                            logger.warning(f"Failed to persist collection item {i}, skipping")
            elif isinstance(value_arg, str) and value_arg.startswith('$'):
                # Single variable - get Note ID or list of Note IDs
                var_name = value_arg[1:]
                if var_name not in self.plan_bindings:
                    return {'status': 'failed', 'reason': f'Variable {value_arg} not bound'}
                bound_value = self.plan_bindings[var_name]
                if isinstance(bound_value, str) and bound_value.startswith('Note_'):
                    note_ids = [bound_value]
                elif isinstance(bound_value, list):
                    # Assume it's a list of Note IDs
                    note_ids = bound_value
                else:
                    return {'status': 'failed', 'reason': f'Variable {value_arg} is not a Note or list'}
            else:
                return {'status': 'failed', 'reason': 'Collection value must be list or $variable'}
            
            # Create Collection in map_node
            collection_id = self._create_collection(note_ids, 'create_collection')
            if collection_id:
                self._bind_variable(out_var, collection_id)
                logger.info(f"Created {collection_id} → ${out_var} ({len(note_ids)} Note IDs)")
                return {'status': 'success', 'value': collection_id}
            else:
                logger.error(f"Failed to create Collection")
                return {'status': 'failed', 'reason': 'Failed to create Collection'}
            
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
    
    def _persist_note(self, value: Any, source_context: str) -> Optional[str]:
        """
        Helper to persist a Note to map_node as a spatial resource.
        
        Args:
            value: Content to persist
            source_context: Description for logging (e.g., 'save_primitive', 'apply_result')
            
        Returns:
            Note ID if successful, None if failed
        """
        if value is None:
            return "Note_null"
        
        format_type = 'json' if isinstance(value, (dict, list)) else 'text'
        
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
                'source_skill': source_context,
                'source_value': str(value)[:100]
            }).encode('utf-8')
        ):
            if reply.ok:
                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if response.get('success'):
                    return response.get('info_id')
                else:
                    logger.error(f'Failed to create Note: {response.get("error")}')
            break
        return None

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
        
        # Persist to map_node
        info_id = self._persist_note(value, 'save_primitive')
        if info_id:
            self._bind_variable(out_var, info_id)
            logger.info(f"Saved Note {info_id} → ${out_var}")
            return {'status': 'success', 'value': info_id}
        
        # Fallback: failed to create Note
        logger.error(f"Spatial Note creation failed for ${out_var}")
        return {'status': 'failed', 'reason': 'Failed to create Note'}
    
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
            # Check if Collection is in local store (new-style Collection_N IDs)
            if resource_id in self.collections:
                collection_id = resource_id
                self._bind_variable(out_var, collection_id)
                logger.info(f"Loaded {collection_id} → ${out_var} ({len(self.collections[collection_id])} items)")
                return {'status': 'success', 'value': collection_id}
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
        
        Store is created using Collection ID as the key. The Collection becomes indexed.
        
        Required: source, index_type (optional)
        
        Argument types:
        - source: $variable (Collection of Notes to index)
        - index_type: literal string ('semantic' or 'keyword', default 'semantic')
        
        The Collection ID becomes the store name automatically.
        """
        source_arg = action.get('source')
        index_type = action.get('index_type', 'semantic')
        fields = action.get('fields', {})
        
        if not source_arg:
            return {'status': 'failed', 'reason': 'index requires source'}
        
        # Source should be a Collection variable
        if not isinstance(source_arg, str) or not source_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'index source must be $variable referencing a Collection'}
        
        collection_var = source_arg[1:]
        
        # Get Collection ID from bindings
        if collection_var not in self.plan_bindings:
            logger.warning(f"Collection variable not bound: {collection_var}")
            return {'status': 'failed', 'reason': f'Collection variable not bound: {collection_var}'}
        
        collection_id = self.plan_bindings[collection_var]
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return {'status': 'failed', 'reason': f'Variable {collection_var} is not a Collection'}
        
        # Request indexing from map_node - use Collection ID as store name
        request = {
            'agent_name': self.agent_name,
            'collection_id': collection_id,  # Collection ID is also the store name
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
        logger.info(f"Indexed {indexed_count} items from {collection_id}")
        return {'status': 'success', 'value': indexed_count}
    
    def _execute_search(self, action: Dict) -> Dict:
        """
        Query indexed Collection.
        
        Required: source, query, out
        Optional: mode, limit, threshold
        
        Argument types:
        - source: $variable (indexed Collection to search)
        - query: literal string OR $variable (resolves to query text)
        - mode: literal string ('semantic' or 'keyword', default 'semantic')
        - limit: int (max results to return, default 5)
        - threshold: float (minimum similarity score, default 0.0)
        - out: literal string (variable name to store results)
        """
        source_arg = action.get('source')
        query = self._resolve_value(action.get('query'))
        mode = action.get('mode', 'semantic')
        limit = action.get('limit', 5)
        threshold = action.get('threshold', 0.0)
        out_var = action.get('out')
        
        if not source_arg or not query or not out_var:
            return {'status': 'failed', 'reason': 'search requires source, query, and out'}
        
        # Source should be a Collection variable
        if not isinstance(source_arg, str) or not source_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'search source must be $variable referencing an indexed Collection'}
        
        collection_var = source_arg[1:]
        
        # Get Collection ID from bindings
        if collection_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Collection variable not bound: {collection_var}'}
        
        collection_id = self.plan_bindings[collection_var]
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return {'status': 'failed', 'reason': f'Variable {collection_var} is not a Collection'}
        
        # Request search from map_node - use Collection ID as store name
        request = {
            'agent_name': self.agent_name,
            'collection_id': collection_id,  # Collection ID is the store name
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
        
        # Create Notes for each search result
        note_ids = []
        for i, result in enumerate(results):
            note_id = self._persist_note(result, f'search_result_{i}')
            if note_id:
                note_ids.append(note_id)
        
        # Create Collection in map_node
        result_collection_id = self._create_collection(note_ids, 'search_results')
        if result_collection_id:
            self._bind_variable(out_var, result_collection_id)
            logger.info(f"Search found {len(results)} results, created {result_collection_id} with {len(note_ids)} Notes → ${out_var}")
            return {'status': 'success', 'value': result_collection_id}
        else:
            logger.error(f"Failed to create Collection for search results")
            return {'status': 'failed', 'reason': 'Failed to create result Collection'}
    
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
        
        # Persist transformed result
        info_id = self._persist_note(result, f'transform_{operation}')
        if info_id:
            self._bind_variable(out_var, info_id)
            logger.info(f"Transformed ({operation}) → Note {info_id} → ${out_var}")
            return {'status': 'success', 'value': info_id}
        else:
            logger.error(f"Failed to persist transform result")
            return {'status': 'failed', 'reason': 'Failed to persist result'}
    
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
        
        # Get Collection Note IDs
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'map target must be a Collection'}
        
        # Apply operation to each Note
        result_note_ids = []
        for i, note_id in enumerate(note_ids):
            # Fetch content for this Note
            content = self._get_content(note_id)
            if content is None and note_id != "Note_null":
                logger.warning(f"Failed to fetch content for {note_id}, skipping")
                continue
            
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
                else:
                    return {'status': 'failed', 'reason': 'operation dict must have "tool" field'}
            else:
                return {'status': 'failed', 'reason': 'operation must be string (tool name) or dict'}
            
            # Handle result
            if result.get('status') == 'success':
                result_value = result.get('value')
                # Create Note for result
                if result_value is None:
                    if not filter_null:
                        result_note_ids.append("Note_null")
                else:
                    result_note_id = self._persist_note(result_value, f'map_result_{i}')
                    if result_note_id and not (filter_null and result_value is None):
                        result_note_ids.append(result_note_id)
            else:
                # Map failed on this item
                logger.warning(f"Map failed on item {i}: {result.get('reason')}")
                if not filter_null:
                    result_note_ids.append("Note_null")
        
        # Create Collection in map_node
        operation_str = operation if isinstance(operation, str) else 'operation'
        collection_id = self._create_collection(result_note_ids, f'map_{operation_str}')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            logger.info(f"Mapped {len(note_ids)} → {len(result_note_ids)} Notes, created {collection_id} → ${out_var}")
            return {'status': 'success', 'value': collection_id}
        else:
            logger.error(f"Failed to create Collection for map results")
            return {'status': 'failed', 'reason': 'Failed to create result Collection'}
    
    def _execute_flatten(self, action: Dict) -> Dict:
        """
        Flatten a Collection into a single Note by concatenating content.
        
        Required: target, out
        Optional: separator (default: "\\n\\n")
        
        Argument types:
        - target: $variable (Collection to flatten)
        - separator: string literal (joins items)
        - out: variable name for result Note
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        out_var = action.get('out')
        separator = action.get('separator', '\n\n')
        
        # Target must be Collection variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'flatten target must be $variable'}
        
        collection_var = target_arg[1:]
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'flatten target must be Collection'}
        
        # Fetch content for each Note and convert to strings
        str_items = []
        for note_id in note_ids:
            content = self._get_content(note_id)
            if content is not None:
                str_items.append(str(content))
        
        flattened = separator.join(str_items)
        
        # Persist flattened result as Note
        info_id = self._persist_note(flattened, 'flatten')
        if info_id:
            self._bind_variable(out_var, info_id)
            logger.info(f"Flattened {len(note_ids)} Notes → Note {info_id} → ${out_var}")
            return {'status': 'success', 'value': info_id}
        else:
            logger.error(f"Failed to persist flatten result")
            return {'status': 'failed', 'reason': 'Failed to persist result'}
    
    def _execute_add(self, action: Dict) -> Dict:
        """
        Add a Note to an existing Collection (mutates Collection).
        
        Required: target, value, out
        
        Argument types:
        - target: $variable (Collection to add to)
        - value: $variable or literal (Note to add)
        - out: variable name (should be same as target to maintain reference)
        
        This is a controlled mutation for practical use cases like dialog histories.
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        # Target must be Collection variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'add target must be $variable'}
        
        collection_var = target_arg[1:]
        
        # Get Collection ID from bindings
        if collection_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Collection variable not bound: {collection_var}'}
        
        collection_id = self.plan_bindings[collection_var]
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return {'status': 'failed', 'reason': f'Variable {collection_var} is not a Collection'}
        
        # Resolve value to Note ID
        if isinstance(value_arg, str) and value_arg.startswith('$'):
            # Variable - get Note ID from bindings
            var_name = value_arg[1:]
            if var_name not in self.plan_bindings:
                return {'status': 'failed', 'reason': f'Note variable not bound: {var_name}'}
            note_id = self.plan_bindings[var_name]
            if not isinstance(note_id, str) or not note_id.startswith('Note_'):
                return {'status': 'failed', 'reason': f'Variable {var_name} is not a Note'}
        else:
            # Literal - create Note for it
            note_id = self._persist_note(value_arg, 'add_item')
            if not note_id:
                return {'status': 'failed', 'reason': 'Failed to persist value as Note'}
        
        # Request add from map_node
        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(
            f"cognitive/map/collection/add",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=5.0,
            payload=json.dumps({
                'collection_id': collection_id,
                'note_id': note_id,
                'agent_name': self.agent_name
            }).encode('utf-8')
        ):
            if reply.ok:
                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if response.get('success'):
                    # Bind to out variable (usually same as target)
                    self._bind_variable(out_var, collection_id)
                    logger.info(f"Added {note_id} to {collection_id} → ${out_var}")
                    return {'status': 'success', 'value': collection_id}
                else:
                    logger.error(f'Failed to add to Collection: {response.get("error")}')
                    return {'status': 'failed', 'reason': response.get('error', 'Add failed')}
            break
        
        return {'status': 'failed', 'reason': 'No response from map_node'}
    
    # ==================== Operation Application Helpers ====================
    
    def _apply_operation_to_value(self, tool_name: str, value: Any, reason: str = '', 
                                   additional_args: Dict = None) -> Dict:
        """
        Helper to apply a tool to a single value with optional additional arguments.
        Shared by map and transform primitives.
        
        Args:
            tool_name: Name of the tool to invoke
            value: Input value to the tool
            reason: Reason for invoking (for logging)
            additional_args: Optional dict of additional arguments for the tool
            
        Returns:
            Result dict with 'status' and 'value' fields
        """
        # Get tool info and route by type
        tool_info = self._get_tool_info(tool_name)
        
        if not tool_info:
            return {'status': 'failed', 'reason': f'Tool not found: {tool_name}'}
        
        tool_type = tool_info.get('type')
        
        if tool_type == 'prompt_augmentation':
            # Execute locally using LLM
            return self._execute_prompt_tool(tool_name, value, tool_info, additional_args or {})
        elif tool_type == 'python':
            # Execute Python tool
            return self._execute_python_tool(tool_name, value, tool_info, additional_args or {})
        else:
            return {'status': 'failed', 'reason': f'Unknown tool type: {tool_type}'}
    

    
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
        
        Tool must return boolean (Python tools) or "true"/"false" string (prompt tools).
        
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
        
        # Explicit boolean (Python tools)
        if isinstance(result_value, bool):
            return result_value
        
        # String parsing (prompt-augmentation tools)
        if isinstance(result_value, str):
            cleaned = result_value.strip().lower()
            if cleaned in ('true', '1', 'yes'):
                return True
            elif cleaned in ('false', '0', 'no'):
                return False
            else:
                logger.warning(f"Ambiguous tool_condition result from '{tool_name}': '{result_value}', defaulting to False")
                return False
        
        # Fallback: truthy/falsy coercion
        logger.warning(f"Tool_condition '{tool_name}' returned non-boolean/non-string: {type(result_value).__name__}, using truthy coercion")
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
    
    def _dereference_collection(self, collection_var: str) -> List[str]:
        """
        Get Collection Note IDs by querying map_node.
        
        Args:
            collection_var: Variable name (without $) of the Collection
            
        Returns:
            List of Note IDs in the Collection
        """
        if collection_var not in self.plan_bindings:
            logger.warning(f"Collection variable not bound: {collection_var}")
            return []
        
        collection_id = self.plan_bindings[collection_var]
        
        # Check if it's a Collection_N ID
        if isinstance(collection_id, str) and collection_id.startswith('Collection_'):
            # Fetch Collection from map_node
            content = self._get_content(collection_id)
            if isinstance(content, list):
                return content
            else:
                logger.warning(f"Collection {collection_id} content is not a list: {type(content)}")
                return []
        
        logger.warning(f"Collection variable is not a Collection: {collection_var}")
        return []
    
    
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
        Resolve $variable to its content value.
        
        Args:
            value: Can be a literal value, or "$variable" string
            
        Returns:
            Resolved content value (fetches from map_node if resource ID)
        """
        if not isinstance(value, str):
            return value
        
        if not value.startswith('$'):
            return value
        
        var_name = value[1:]
        if var_name not in self.plan_bindings:
            logger.warning(f"Unbound variable: {value}")
            return None
        
        resource_id = self.plan_bindings[var_name]
        
        # If it's a resource ID, fetch content from map_node
        if isinstance(resource_id, str) and (resource_id.startswith('Note_') or resource_id.startswith('Collection_')):
            return self._get_content(resource_id)
        
        # Otherwise return as-is (shouldn't happen in new model, but handle gracefully)
        return resource_id
    
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
        # Check if value is a Collection_N ID
        if isinstance(value, str) and value.startswith('Collection_'):
            return 'Collection'
        # Legacy: list values (pre-Collection ID implementation)
        return 'Collection' if isinstance(value, list) else 'Note'
    
    def _bind_variable(self, var_name: str, resource_id: str):
        """Bind variable name to resource ID"""
        if var_name.startswith('$'):
            var_name = var_name[1:]
        
        self.plan_bindings[var_name] = resource_id
        logger.debug(f"Bound ${var_name} → {resource_id}")
    
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

