"""
Infospace Executor - Executes cognitive/information space primitives.

Handles data operations, storage, and cognitive workflows for agents
operating in information spaces (semantic/tool spaces).
"""

import json
import time
import logging
import re
import traceback
import importlib.util
from pathlib import Path
from typing import Dict, List, Any, Optional
from datetime import datetime

logger = logging.getLogger(__name__)


class InfospaceExecutor:
    """
    Executor for information space primitives.
    
    Core Primitives (whole-value operations only):
    - Computation: apply, map, coerce
    - Storage: save, load, create
    - Indexing: index, search
    - Communication: say, think
    - Spatial: focus
    
    Design principle: Content is opaque. Field-based operations delegated to tools.
    """
    
    def __init__(self, agent_name: str, session, map_name: str, llm_client, available_tools: Dict[str, Dict], executive_node=None):
        """
        Initialize infospace executor.
        
        Args:
            agent_name: Name of the agent
            session: Zenoh session for communication
            map_name: Name of the map (for Zenoh topics)
            llm_client: LLM client for tool execution
            available_tools: Dict of tool_name -> metadata (from tool_loader)
            executive_node: Reference to ZenohExecutiveNode (for ask action)
        """
        self.agent_name = agent_name
        self.session = session
        self.map_name = map_name
        self.llm_client = llm_client
        self.available_tools = available_tools
        self.executive_node = executive_node
        
        # === ZENOH PUBLICATION ===
        # NAME: turn_heartbeat
        # TOPIC: cognitive/map/turn/heartbeat/{character}
        # DESCRIPTION: Character is alive and processing (prevents turn timeout)
        # PAYLOAD: {"timestamp": str}
        # TRIGGERS: (internal turn management - resets timeout)
        # ========================
        self.heartbeat_publisher = self.session.declare_publisher(
            f"cognitive/map/turn/heartbeat/{agent_name}"
        )
        
        # Plan-local state (ephemeral, cleared each plan)
        self.plan_bindings = {}  # $var_name -> resource_id (Note_N or Collection_N)
        
        # Agent state (persistent across plans)
        self.agent_position = None
        
        # Suspension state for sync execution
        self._sync_suspension_state = None
        
        logger.info(f"InfospaceExecutor initialized for {agent_name} with {len(available_tools)} tools")
    
    def clear_plan_state(self):
        """Clear ephemeral plan state (call at start of new plan)"""
        self.plan_bindings = {}
    
    def _create_collection(self, note_ids: list, source_context: str, collection_name: str = '', properties: Optional[Dict] = None) -> Optional[str]:
        """
        Create a Collection resource in map_node.
        
        Args:
            note_ids: List of Note IDs (e.g., ["Note_2", "Note_3"])
            source_context: Description for logging (e.g., 'map_operation', 'create_collection')
            collection_name: Optional stable name for the Collection
            properties: Optional dict of extra properties to attach
            
        Returns:
            Collection ID if successful, None if failed
        """
        from zenoh import QueryTarget, ConsolidationMode
        payload_dict = {
            'character_name': self.agent_name,
            'content': note_ids,  # List of Note IDs
            'format': 'list',
            'source_skill': source_context,
            'collection_name': collection_name
        }
        if properties:
            payload_dict['properties'] = properties
        for reply in self.session.get(
            f"cognitive/map/collection/create",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=5.0,
            payload=json.dumps(payload_dict).encode('utf-8')
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
                    # Handle both response formats:
                    # 1. handle_resource_by_name: {'success': True, 'resource': {...}}
                    # 2. handle_resource_viewer: {'success': True, 'type': '...', 'content': ..., 'metadata': {...}}
                    if 'resource' in response:
                        # Format 1: extract content from resource.properties
                        resource_data = response.get('resource')
                        if not resource_data:
                            logger.error(f"Resource {resource_id} returned no data")
                            return None
                        content = resource_data.get('properties', {}).get('content')
                    else:
                        # Format 2: content is directly in response
                        content = response.get('content')
                    
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
            'focus': self._execute_focus,
            'create-note': self._execute_create_note,
            'create-collection': self._execute_create_collection,
            'persist': self._execute_persist,
            'load': self._execute_load,
            'index': self._execute_index,
            'organize': self._execute_index,  # Alias for index
            'search': self._execute_search,
            'say': self._execute_say,
            'display': self._execute_display,
            'think': self._execute_think,
            'ask': self._execute_ask,
            # Phase 2: Data Operations (whole-value only)
            'coerce': self._execute_coerce,
            'map': self._execute_map,
            'flatten': self._execute_flatten,
            'add': self._execute_add,
            'expand': self._execute_expand,
            # Set operations
            'size': self._execute_size,
            'union': self._execute_union,
            'intersection': self._execute_intersection,
            'difference': self._execute_difference,
            'remove': self._execute_remove,
        }
        
        handler = handlers.get(action_type)
        if not handler:
            logger.error(f"Unknown action type: {action_type}")
            return {'status': 'failed', 'reason': f'Unknown action: {action_type}'}
        
        try:
            return handler(action)
        except Exception as e:
            logger.error(f"Error executing action {action_type}: {e}")
            logger.error(traceback.format_exc())
            return {'status': 'failed', 'reason': f'Execution error: {str(e)}'}
    
    # ==================== Core Operations ====================
    
    def _execute_scan(self, action: Dict) -> Dict:
        """
        Scan for resource/tool by name or interface type.
        
        Required: type, target, out, expect
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
        
        Required: type, target, out
        Optional: value (input), reason (documentation), args (additional arguments)
        
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
        
        # Handle tools with custom parameter sources (e.g., args.query)
        if tool_info:
            param_source = tool_info.get('parameter_source')
            if param_source and param_source.startswith('args.'):
                # Extract parameter from args dict (e.g., 'args.query' -> additional_args['query'])
                param_name = param_source.split('.', 1)[1]
                if param_name in additional_args:
                    value = self._resolve_value(additional_args.get(param_name))
                    # Remove from args since it's now the main value
                    additional_args = {k: v for k, v in additional_args.items() if k != param_name}
        
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
            # Special handling for Level 4 tools: return Collection ID directly
            level4_tools = ['filter-collection', 'query-web', 'semantic-scholar']
            if target in level4_tools and isinstance(result_value, str) and result_value.startswith('Collection_'):
                # Result is already a Collection ID - bind directly
                self._bind_variable(out_var, result_value)
                logger.info(f"Tool '{target}' executed, Collection {result_value} → ${out_var}")
                return {'status': 'success', 'value': result_value}
            
            # Default: Persist result as Note
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
            
            # Special handling for extract-struct: truncate input since metadata is always at start
            if tool_name == 'extract-struct':
                # Check if input is JSON from fetch-text (or extract-paper-text for backward compatibility) and extract text field
                if isinstance(input_str, str):
                    try:
                        parsed = json.loads(input_str)
                        if isinstance(parsed, dict) and 'text' in parsed:
                            input_str = parsed['text']
                            logger.info(f"extract-struct: extracted 'text' field from JSON input")
                    except (json.JSONDecodeError, TypeError):
                        pass
                
                # Truncate to first 15000 chars (metadata is always at start)
                if len(input_str) > 15000:
                    logger.info(f"extract-struct: truncating input from {len(input_str)} to 15000 chars (metadata is at start)")
                    input_str = input_str[:15000]
            
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
            
            prompt_parts.append("""\n# OUTPUT\nProvide the result directly, following the tool's output format. 
Do not include any introductory, reasoning, or explanatory text in your response. 
Only provide the result, followed by the </end> tag.""")
            
            full_prompt = "".join(prompt_parts)
            
            # Call LLM directly
            llm_response = self.llm_client.generate(
                messages=[full_prompt],
                bindings={},
                max_tokens=1000,
                temperature=0.3,
                stops=['</end>']
            )
            
            # Send heartbeat after LLM call
            self.heartbeat_publisher.put(json.dumps({
                'character': self.agent_name,
                'timestamp': time.time()
            }))
            
            if not llm_response or not llm_response.text:
                return {'status': 'failed', 'reason': 'LLM returned empty response'}
            
            result_text = llm_response.text.strip()
            logger.info(f"Prompt tool '{tool_name}' completed ({len(result_text)} chars)")
            
            # Check for null indicator - return Note_null resource ID instead of string
            if result_text.lower() == 'note-null' or result_text.lower() == 'null':
                logger.info(f"Prompt tool '{tool_name}' returned null indicator, using Note_null resource")
                return {'status': 'success', 'value': 'Note_null'}
            
            return {'status': 'success', 'value': result_text}
            
        except Exception as e:
            logger.error(f"Prompt tool execution failed: {e}")
            return {'status': 'failed', 'reason': str(e)}

    def _execute_python_tool(self, tool_name: str, input_value: Any,
                            tool_info: Dict, additional_args: Dict) -> Dict:
        """
        Execute a Python-based tool or plan-based tool.
        
        Args:
            tool_name: Name of the tool
            input_value: Input data to process
            tool_info: Tool metadata including python_file or plan_data
            additional_args: Optional parameters passed as **kwargs
            
        Returns:
            Result dict with 'status' and 'value'
        """
        tool_type = tool_info.get('type')
        
        # Handle plan-based tools
        if tool_type == 'plan':
            plan_data = tool_info.get('plan_data')
            if not plan_data:
                return {'status': 'failed', 'reason': f'Plan tool {tool_name} missing plan_data'}
            
            # Build initial bindings for plan tool
            initial_bindings = {}
            
            # Bind main input to $input variable
            if input_value is not None:
                input_note_id = self._persist_note(input_value, f'{tool_name}_input')
                if input_note_id:
                    initial_bindings['input'] = input_note_id
            
            # Bind args dict values to named variables
            for key, val in additional_args.items():
                resolved_val = self._resolve_value(val)
                arg_note_id = self._persist_note(resolved_val, f'{tool_name}_{key}')
                if arg_note_id:
                    initial_bindings[key] = arg_note_id
            
            # Execute plan synchronously with initial bindings
            result = self.execute_plan_sync(plan_data, initial_bindings=initial_bindings)
            
            if result.get('status') == 'suspended':
                # Plan suspended (ask/wait) - return suspension
                return result
            
            if result.get('status') != 'success':
                return {'status': 'failed', 'reason': f'Plan tool execution failed: {result.get("reason")}'}
            
            # Extract output from plan's 'out' variable (from plan_data)
            out_var = plan_data.get('out', 'result')
            if out_var.startswith('$'):
                out_var = out_var[1:]
            
            # Get output from isolated executor's bindings (returned in result)
            bindings = result.get('bindings', {})
            if out_var in bindings:
                output_note_id = bindings[out_var]
                output_content = self._get_content(output_note_id)
                return {'status': 'success', 'value': output_content}
            else:
                return {'status': 'failed', 'reason': f'Plan tool output variable ${out_var} not bound'}
        
        # Handle Python-based tools
        # Check trust flag
        if not tool_info.get('trusted', False):
            tool_path = tool_info.get('path', 'unknown')
            skill_md_path = f"{tool_path}/SKILL.md" if tool_path != 'unknown' else f"tools/{tool_name}/SKILL.md"
            return {'status': 'failed', 
                   'reason': f'Untrusted Python tool: {tool_name}. Add "trusted: true" to frontmatter in {skill_md_path}'}
        
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
        
        # Add heartbeat callback for tools (all tools receive it, LLM-using tools can call it)
        def heartbeat():
            """Send heartbeat to reset turn timeout"""
            self.heartbeat_publisher.put(json.dumps({
                'character': self.agent_name,
                'timestamp': time.time()
            }))
        
        resolved_args['heartbeat'] = heartbeat
        
        # Add map_name and llm_client to kwargs for tools that need them
        resolved_args['map_name'] = self.map_name
        resolved_args['llm_client'] = self.llm_client
        resolved_args['agent_name'] = self.agent_name
        
        # Execute tool
        try:
            logger.debug(f"Calling tool '{tool_name}' with input_value type: {type(input_value)}, length: {len(str(input_value)) if isinstance(input_value, str) else 'N/A'}")
            result = tool_func(input_value, **resolved_args)
        except Exception as e:
            logger.error(f"Python tool '{tool_name}' execution error: {e}", exc_info=True)
            return {'status': 'failed', 'reason': f'Tool execution error: {str(e)}'}
        
        # Handle result format
        if isinstance(result, dict) and 'status' in result:
            # Tool returned structured response
            logger.info(f"Python tool '{tool_name}' completed with status: {result.get('status')}")
            return result
        elif isinstance(result, str) and result.startswith('Error:'):
            # Tool returned error string
            error_msg = result
            logger.error(f"Python tool '{tool_name}' failed: {error_msg}")
            return {'status': 'failed', 'reason': error_msg}
        else:
            # Tool returned raw value
            logger.info(f"Python tool '{tool_name}' completed")
            return {'status': 'success', 'value': result}

    def _execute_focus(self, action: Dict) -> Dict:
        """
        Focus on resource location.
        
        Required: type, target
        """
        target = self._resolve_value(action.get('target'))
        
        if not target:
            return {'status': 'failed', 'reason': 'focus requires target'}
        
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
            return {'status': 'failed', 'reason': 'Focus timeout'}
        
        if response.get('status') != 'success':
            return {'status': 'failed', 'reason': response.get('reason', 'Focus failed')}
        
        logger.info(f"Focused on {target}")
        return {'status': 'success', 'value': target}
    
    def _execute_create_note(self, action: Dict) -> Dict:
        """
        Create a Note object as persistent spatial resource.
        
        Required: value, out
        Optional: properties (dict of extra metadata)
        """
        value_arg = action.get('value')
        out_var = action.get('out')
        extra_props = action.get('properties')
        
        if not out_var:
            return {'status': 'failed', 'reason': 'create-note requires out'}
        
        value = self._resolve_value(value_arg) if value_arg is not None else ''
        
        if value is None:
            self._bind_variable(out_var, "Note_null")
            logger.info(f"Created null Note → ${out_var} = Note_null")
            return {'status': 'success', 'value': "Note_null"}
        
        info_id = self._persist_note(value, 'create-note-primitive', extra_props)
        if info_id:
            self._bind_variable(out_var, info_id)
            logger.info(f"Created Note {info_id} → ${out_var}")
            return {'status': 'success', 'value': info_id}
        
        logger.error(f"Note creation failed for ${out_var}")
        return {'status': 'failed', 'reason': 'Failed to create Note'}
    
    def _execute_create_collection(self, action: Dict) -> Dict:
        """
        Create a Collection object as session-local resource.
        
        Required: value, out
        Optional: name (stable name for Collection), properties (dict of extra metadata)
        
        Collections store resource IDs (references to Notes/Collections).
        """
        out_var = action.get('out')
        value_arg = action.get('value')
        collection_name = action.get('name')
        extra_props = action.get('properties')
        
        if not out_var:
            return {'status': 'failed', 'reason': 'create-collection requires out'}
        
        if value_arg is None:
            note_ids = []
        elif isinstance(value_arg, list):
            note_ids = []
            for i, item in enumerate(value_arg):
                if isinstance(item, str) and item.startswith('$'):
                    var_name = item[1:]
                    if var_name not in self.plan_bindings:
                        logger.warning(f"Variable {item} not bound, skipping")
                        continue
                    note_id = self.plan_bindings[var_name]
                    if isinstance(note_id, str) and (note_id.startswith('Note_') or note_id.startswith('Collection_')):
                        note_ids.append(note_id)
                    else:
                        logger.warning(f"Variable {item} is not a Note/Collection, skipping")
                else:
                    note_id = self._persist_note(item, f'createCollection_item_{i}')
                    if note_id:
                        note_ids.append(note_id)
                    else:
                        logger.warning(f"Failed to persist collection item {i}, skipping")
        elif isinstance(value_arg, str) and value_arg.startswith('$'):
            var_name = value_arg[1:]
            if var_name not in self.plan_bindings:
                return {'status': 'failed', 'reason': f'Variable {value_arg} not bound'}
            bound_value = self.plan_bindings[var_name]
            if isinstance(bound_value, str) and bound_value.startswith('Collection_'):
                # Dereference Collection to get its Note IDs
                note_ids = self._dereference_collection(var_name)
                if not isinstance(note_ids, list):
                    return {'status': 'failed', 'reason': f'Failed to dereference Collection {value_arg}'}
            elif isinstance(bound_value, str) and bound_value.startswith('Note_'):
                # Single Note - wrap in list
                note_ids = [bound_value]
            elif isinstance(bound_value, list):
                note_ids = bound_value
            else:
                return {'status': 'failed', 'reason': f'Variable {value_arg} is not a Note/Collection or list'}
        else:
            return {'status': 'failed', 'reason': 'Collection value must be list or $variable'}
        
        collection_id = self._create_collection(note_ids, 'create-collection-primitive', collection_name, extra_props)
        if collection_id:
            self._bind_variable(out_var, collection_id)
            name_display = f" '{collection_name}'" if collection_name else ""
            logger.info(f"Created {collection_id}{name_display} → ${out_var} ({len(note_ids)} items)")
            return {'status': 'success', 'value': collection_id}
        
        logger.error(f"Collection creation failed for ${out_var}")
        return {'status': 'failed', 'reason': 'Failed to create Collection'}
    
    # ==================== Storage Operations ====================
    
    def _persist_note(self, value: Any, source_context: str, properties: Optional[Dict] = None) -> Optional[str]:
        """
        Helper to persist a Note to map_node as a spatial resource.
        
        Args:
            value: Content to persist
            source_context: Description for logging (e.g., 'save_primitive', 'apply_result')
            properties: Optional dict of extra properties to attach
            
        Returns:
            Note ID if successful, None if failed
        """
        if value is None:
            return "Note_null"
        
        # If value is already Note_null resource ID, return it directly
        if isinstance(value, str) and value == "Note_null":
            return "Note_null"
        
        format_type = 'json' if isinstance(value, (dict, list)) else 'text'
        
        from zenoh import QueryTarget, ConsolidationMode
        payload_dict = {
            'character_name': self.agent_name,
            'content': value,
            'format': format_type,
            'source_skill': source_context,
            'source_value': str(value)[:100]
        }
        if properties:
            payload_dict['properties'] = properties
        for reply in self.session.get(
            f"cognitive/map/note/create",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=5.0,
            payload=json.dumps(payload_dict).encode('utf-8')
        ):
            if reply.ok:
                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if response.get('success'):
                    return response.get('info_id')
                else:
                    logger.error(f'Failed to create Note: {response.get("error")}')
            break
        return None

    def _execute_persist(self, action: Dict) -> Dict:
        """
        Mark a Note or Collection as persistent (saved to filesystem).
        
        Required: target
        
        Once marked persistent, Note or Collection is saved to filesystem on next persist cycle.
        
        Target can be:
        - $variable referencing a Note or Collection
        - Literal Note_ID (e.g., "Note_123") or Collection_ID (e.g., "Collection_456")
        """
        target_arg = action.get('target')
        
        if not target_arg:
            return {'status': 'failed', 'reason': 'persist requires target'}
        
        if not isinstance(target_arg, str):
            return {'status': 'failed', 'reason': 'persist target must be string'}
        
        # Resolve to resource ID (handles both $var and literal IDs)
        resource_id = self._resolve_id(target_arg)
        
        if not isinstance(resource_id, str) or not (resource_id.startswith('Collection_') or resource_id.startswith('Note_')):
            return {'status': 'failed', 'reason': f'Target must be a Note or Collection ID, got: {resource_id}'}
        
        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(
            f"cognitive/map/collection/persist",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=5.0,
            payload=json.dumps({
                'resource_id': resource_id,
                'character_name': self.agent_name
            }).encode('utf-8')
        ):
            if reply.ok:
                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if response.get('success'):
                    logger.info(f"Marked {resource_id} as persistent")
                    return {'status': 'success', 'value': resource_id}
                else:
                    logger.error(f'Failed to persist resource: {response.get("error")}')
                    return {'status': 'failed', 'reason': response.get('error', 'Unknown error')}
            break
        
        logger.error(f"Persist request failed for {resource_id}")
        return {'status': 'failed', 'reason': 'Failed to mark resource as persistent'}
    
    def _execute_load(self, action: Dict) -> Dict:
        """
        Load a persistent Note or Collection by resource ID or name.
        
        Required: resource_id, out
        
        Retrieves an existing spatial resource from the map and binds it to a variable.
        resource_id can be: Note_X, Collection_X, or a collection name.
        """
        resource_id = action.get('resource_id')
        out_var = action.get('out')
        
        if not resource_id:
            return {'status': 'failed', 'reason': 'load requires resource_id'}
        
        if not out_var:
            return {'status': 'failed', 'reason': 'load requires out'}
        
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
                    # Get the actual resource ID from the response
                    if 'resource' in response:
                        resource_data = response.get('resource')
                        actual_id = resource_data.get('name')  # This is the real ID (Note_X or Collection_X)
                        if not actual_id:
                            return {'status': 'failed', 'reason': f'Resource {resource_id} has no ID'}
                    else:
                        # Fallback: assume resource_id is already an ID
                        actual_id = resource_id
                    
                    # Bind the actual resource ID to variable
                    self._bind_variable(out_var, actual_id)
                    logger.info(f"Loaded {resource_id} → ${out_var}")
                    return {'status': 'success', 'value': actual_id}
                else:
                    return {'status': 'failed', 'reason': f'Resource not found: {resource_id}'}
            break
        
        return {'status': 'failed', 'reason': f'No response for {resource_id}'}
    
    def _execute_index(self, action: Dict) -> Dict:
        """
        Create embeddings index for a Collection (also callable as 'organize').
        
        Index is created using Collection ID as the key. The Collection becomes searchable.
        
        Required: source, index_type (optional)
        
        Argument types:
        - source: $variable (Collection of Notes to index)
        - index_type: literal string ('semantic' or 'keyword', default 'semantic')
        
        The Collection ID is used as the index identifier.
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
        
        # Request indexing from map_node using Collection ID
        request = {
            'agent_name': self.agent_name,
            'collection_id': collection_id,
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
            timeout=15.0  # Embedding generation takes time, especially on first model load
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
        Optional: mode, limit, threshold, return_mode
        
        Argument types:
        - source: $variable (indexed Collection to search)
        - query: literal string OR $variable (resolves to query text)
        - mode: literal string ('semantic' or 'keyword', default 'semantic')
        - limit: int (max results to return, default 5)
        - threshold: float (minimum similarity score, default 0.0)
        - return_mode: literal string ('chunks' or 'notes', default 'chunks')
        - out: literal string (variable name to store results)
        """
        source_arg = action.get('source')
        query = self._resolve_value(action.get('query'))
        mode = action.get('mode', 'semantic')
        limit = action.get('limit', 5)
        threshold = action.get('threshold', 0.0)
        return_mode = action.get('return_mode', 'chunks')
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
        
        # Request search from map_node using Collection ID
        request = {
            'agent_name': self.agent_name,
            'collection_id': collection_id,
            'query': query,
            'mode': mode,
            'limit': limit,
            'threshold': threshold,
            'return_mode': return_mode
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
            if return_mode == 'chunks':
                # For chunks mode: store only chunk text, preserve metadata in properties
                chunk_content = result.get('document', '')
                metadata = result.get('metadata', {})
                
                # Extract relevant metadata for properties
                properties = {
                    'source_note_id': metadata.get('source_note_id'),
                    'chunk_index': metadata.get('chunk_index'),
                    'chunk_total': metadata.get('chunk_total'),
                    'is_complete_note': metadata.get('is_complete_note', False),
                    'score': result.get('score'),
                    'return_mode': 'chunks'
                }
                # Remove None values
                properties = {k: v for k, v in properties.items() if v is not None}
                
                note_id = self._persist_note(chunk_content, f'search_result_{i}', properties=properties)
            else:
                # For notes mode: store original note content (deduplicated, full document)
                note_content = result.get('document', '')
                metadata = result.get('metadata', {})
                
                # Store source reference in properties (standardized format matching chunks mode)
                properties = {
                    'source_note_id': metadata.get('source_note_id'),
                    'chunk_index': None,  # Not applicable for full notes
                    'chunk_total': 1,  # Single complete note
                    'is_complete_note': True,  # Always true for notes mode
                    'score': result.get('score'),
                    'return_mode': 'notes'
                }
                # Remove None values (chunk_index will be omitted)
                properties = {k: v for k, v in properties.items() if v is not None}
                
                note_id = self._persist_note(note_content, f'search_result_{i}', properties=properties)
            
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
        
        # Capitalize 'user' to 'User' for character name
        if target.lower() == 'user':
            target = 'User'
        
        # Send message to target's sense_data (mirrors physical space send_text_input)
        sense_data = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps({
                'source': self.agent_name,
                'text': str(value)
            })
        }
        
        self.session.put(
            f"cognitive/{target}/sense_data",
            json.dumps(sense_data)
        )
        
        logger.info(f"Say [{target}]: {value}")
        return {'status': 'success', 'value': value}
    
    def _execute_display(self, action: Dict) -> Dict:
        """
        Display formatted content in popup (similar to say but with popup UI).
        
        Required: type, value or target (accepts both for compatibility)
        """
        # Accept both value and target (target preferred for consistency with other primitives)
        target_arg = action.get('value') or action.get('target')
        
        # Resolve variable if it's a $variable
        value = self._resolve_value(target_arg)
        
        # If resolved value is a resource ID (literal string like "Note_20"), dereference it
        if isinstance(value, str) and (value.startswith('Note_') or value.startswith('Collection_')):
            content = self._get_content(value)
            if content is not None:
                value = content
        
        if value is None:
            return {'status': 'failed', 'reason': 'display requires value or target'}
        
        # Always display to User
        target = 'User'
        
        # Send message to target's sense_data (mirrors physical space send_text_input)
        sense_data = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps({
                'source': self.agent_name,
                'text': str(value)
            })
        }
        
        self.session.put(
            f"cognitive/{target}/sense_data",
            json.dumps(sense_data)
        )
        
        logger.info(f"Display [{target}]: {value}")
        return {'status': 'success', 'value': value}
    
    def _execute_think(self, action: Dict) -> Dict:
        """
        Internal thought/note (logged but not communicated externally).
        
        Required: type, value or target (accepts both for compatibility)
        """
        # Accept both value and target (target preferred for consistency with other primitives)
        value = self._resolve_value(action.get('value') or action.get('target'))
        
        if value is None:
            return {'status': 'failed', 'reason': 'think requires value or target'}
        
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
    
    def _execute_ask(self, action: Dict) -> Dict:
        """
        Ask user a question and suspend plan execution until response received.
        Suspension logic handled in executive_node._execute_next_step().
        
        Required: type, value, out
        Optional: target (defaults to 'User')
        """
        out_var = action.get('out')
        if not out_var:
            return {'status': 'failed', 'reason': 'ask requires out field'}
        
        question_text = self._resolve_value(action.get('value'))
        if question_text is None:
            return {'status': 'failed', 'reason': 'ask requires value'}
        
        target = action.get('target', 'User')
        
        # Publish question via action_data (same format as say, appears in main UI)
        action_data = {
            'type': 'ask',
            'action_id': f'action_{self.executive_node.action_counter}',
            'timestamp': datetime.now().isoformat(),
            'text': str(question_text),
            'source': self.agent_name,
            'target': target
        }
        self.executive_node.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        
        # Enter step mode so user can respond and click Step/Run
        self.session.put("cognitive/map/turn/step", b"")
        
        # Set suspension state in plan_state for _execute_next_step to handle
        self.executive_node.plan_state['awaiting_ask'] = {
            'out_var': out_var
        }
        
        logger.info(f"❓ Ask: '{question_text}' → awaiting response for ${out_var} (step mode enabled)")
        return {'status': 'success', 'value': question_text}
    
    # ==================== Phase 2: Data Operations ====================
    
    def _execute_coerce(self, action: Dict) -> Dict:
        """
        Coerce data format or structure.
        
        Required: type, target, operation, out
        """
        target = self._resolve_value(action.get('target'))
        operation = action.get('operation')
        out_var = action.get('out')
        
        if not target or not operation or not out_var:
            return {'status': 'failed', 'reason': 'coerce requires target, operation, and out'}
        
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
        else:
            return {'status': 'failed', 'reason': f'Unknown coerce operation: {operation}'}
        
        # Persist coerced result
        info_id = self._persist_note(result, f'coerce_{operation}')
        if info_id:
            self._bind_variable(out_var, info_id)
            logger.info(f"Coerced ({operation}) → Note {info_id} → ${out_var}")
            return {'status': 'success', 'value': info_id}
        else:
            logger.error(f"Failed to persist coerce result")
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
        filter_null = action.get('filter_null', True)
        additional_args = action.get('args', {})
        
        # Target must be a Collection variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'map target must be $variable referencing a Collection'}
        
        collection_var = target_arg[1:]
        
        # Get Collection Note IDs
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'map target must be a Collection'}
        
        # Primitives that can be used in map operations
        primitive_handlers = {
            'add': self._execute_add,
            'remove': self._execute_remove,
        }
        
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
                # Check if it's a primitive
                if operation in primitive_handlers:
                    # Construct action dict for primitive
                    primitive_action = {'type': operation}
                    primitive_action.update(additional_args)
                    
                    # For add/remove: map item becomes 'value', target comes from args
                    if operation in ['add', 'remove']:
                        primitive_action['value'] = note_id
                        if 'out' not in primitive_action:
                            primitive_action['out'] = additional_args.get('target', out_var)
                    
                    # Execute primitive
                    result = primitive_handlers[operation](primitive_action)
                else:
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
                return {'status': 'failed', 'reason': 'operation must be string (tool/primitive name) or dict'}
            
            # Handle result
            if result.get('status') == 'success':
                result_value = result.get('value')
                # For mutation primitives like add/remove, result is the mutated Collection ID
                if isinstance(operation, str) and operation in ['add', 'remove']:
                    # Mutation primitive - result is the mutated collection
                    # Store it to use as final output (will be same for all iterations)
                    if isinstance(result_value, str) and result_value.startswith('Collection_'):
                        # Keep reference to mutated collection (will overwrite each iteration, but that's fine)
                        result_note_ids.append(result_value)
                elif result_value is None:
                    if not filter_null:
                        result_note_ids.append("Note_null")
                else:
                    # Create Note for result
                    result_note_id = self._persist_note(result_value, f'map_result_{i}')
                    if result_note_id and not (filter_null and result_value is None):
                        result_note_ids.append(result_note_id)
            else:
                # Map failed on this item
                logger.warning(f"Map failed on item {i}: {result.get('reason')}")
                if not filter_null:
                    result_note_ids.append("Note_null")
        
        # Handle result based on operation type
        if isinstance(operation, str) and operation in ['add', 'remove']:
            # Mutation primitive - bind to the mutated collection (from args.target)
            mutation_target = additional_args.get('target')
            if mutation_target:
                target_var = mutation_target[1:] if mutation_target.startswith('$') else mutation_target
                if target_var in self.plan_bindings:
                    mutated_collection_id = self.plan_bindings[target_var]
                    self._bind_variable(out_var, mutated_collection_id)
                    logger.info(f"Mapped {len(note_ids)} items via {operation} → ${out_var} = {mutated_collection_id}")
                    return {'status': 'success', 'value': mutated_collection_id}
                else:
                    return {'status': 'failed', 'reason': f'Mutation target {mutation_target} not bound'}
            else:
                return {'status': 'failed', 'reason': f'{operation} requires target in args'}
        else:
            # Tool operation - create result Collection
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
        elif isinstance(value_arg, str) and value_arg.startswith('Note_'):
            # Direct Note ID (e.g., from map operation)
            note_id = value_arg
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
    
    def _execute_expand(self, action: Dict) -> Dict:
        """
        Expand a Note into a Collection of Notes.
        
        Handles two cases:
        1. JSON with array field: Extracts array from specified field (default 'results')
        2. Plain text: Splits on newlines and filters empty lines
        
        Required: target, out
        Optional: field (default: 'results') - only used for JSON case
        
        Argument types:
        - target: $variable (Note containing JSON with array or plain text)
        - field: literal string (name of array field, default 'results') - ignored for plain text
        - out: variable name for resulting Collection
        
        Examples:
        - Expand query-web results: {"type":"expand","target":"$results","out":"$items"}
        - Expand text lines: {"type":"expand","target":"$text_note","out":"$lines"}
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        field_name = action.get('field', 'results')
        out_var = action.get('out')
        
        # Target must be Note variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'expand target must be $variable'}
        
        note_var = target_arg[1:]
        
        # Get Note ID from bindings
        if note_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Note variable not bound: {note_var}'}
        
        note_id = self.plan_bindings[note_var]
        
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            return {'status': 'failed', 'reason': f'Variable {note_var} is not a Note'}
        
        # Check for null Note
        if note_id == 'Note_null':
            return {'status': 'failed', 'reason': 'Cannot expand null Note'}
        
        # Get Note content
        content = self._get_content(note_id)
        if content is None:
            return {'status': 'failed', 'reason': f'Note {note_id} has no content'}
        
        # Check for null content indicator
        if isinstance(content, str) and content.strip().lower() in ['note-null', 'null']:
            return {'status': 'failed', 'reason': 'Cannot expand null content'}
        
        array_data = None
        is_json = False
        
        # Try JSON extraction first (prioritize structured data)
        # Use robust extraction that handles code fences, preambles, trailing text
        if isinstance(content, str):
            # Robust extraction (strips code fences, preambles, trailing text)
            content_obj = self._extract_json_from_text(content)
            is_json = (content_obj is not None)
        elif isinstance(content, dict):
            content_obj = content
            is_json = True
        else:
            content_obj = None
            is_json = False
        
        # If content is valid JSON, extract array field (do NOT fall back to text)
        if is_json and content_obj:
            if field_name in content_obj:
                array_data = content_obj[field_name]
                if not isinstance(array_data, list):
                    return {'status': 'failed', 'reason': f'Field "{field_name}" exists but is not an array'}
            else:
                return {'status': 'failed', 'reason': f'JSON content missing "{field_name}" array field'}
        
        # Only if NOT JSON (parse failed), fall back to plain text line splitting
        elif not is_json:
            if isinstance(content, str):
                # Split on newlines and filter empty lines
                lines = [line.strip() for line in content.split('\n')]
                array_data = [line for line in lines if line]  # Filter empty lines
            else:
                return {'status': 'failed', 'reason': f'Note content must be JSON with "{field_name}" array field or plain text'}
        
        if not array_data:
            return {'status': 'failed', 'reason': 'No items to expand (empty array or no non-empty lines)'}
        
        # Create Note for each array element/line
        note_ids = []
        for i, item in enumerate(array_data):
            item_note_id = self._persist_note(item, f'expand_item_{i}')
            if item_note_id:
                note_ids.append(item_note_id)
        
        # Create Collection from Notes
        collection_id = self._create_collection(note_ids, 'expanded_collection')
        if not collection_id:
            return {'status': 'failed', 'reason': 'Failed to create Collection'}
        
        # Bind to out variable
        self._bind_variable(out_var, collection_id)
        source_desc = f"{note_id}.{field_name}" if content_obj and field_name in content_obj else f"{note_id} (lines)"
        logger.info(f"Expanded {source_desc} ({len(note_ids)} items) → ${out_var}")
        
        return {'status': 'success', 'value': collection_id}
    
    # ==================== Set Operations ====================
    
    def _execute_size(self, action: Dict) -> Dict:
        """
        Get size (item count) of a Collection.
        
        Required: target, out
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'size target must be $variable'}
        
        collection_var = target_arg[1:]
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'size target must be a Collection'}
        
        size = len(note_ids)
        info_id = self._persist_note(size, 'size_result')
        if info_id:
            self._bind_variable(out_var, info_id)
            logger.info(f"Size of {collection_var}: {size} → ${out_var}")
            return {'status': 'success', 'value': info_id}
        
        return {'status': 'failed', 'reason': 'Failed to persist size result'}
    
    def _execute_union(self, action: Dict) -> Dict:
        """
        Union of two Collections (A ∪ B) - all items from both, deduplicated.
        
        Required: target, value, out
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'union target must be $variable'}
        if not isinstance(value_arg, str) or not value_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'union value must be $variable'}
        
        target_var = target_arg[1:]
        value_var = value_arg[1:]
        
        target_ids = self._dereference_collection(target_var)
        value_ids = self._dereference_collection(value_var)
        
        if not isinstance(target_ids, list) or not isinstance(value_ids, list):
            return {'status': 'failed', 'reason': 'union requires both arguments to be Collections'}
        
        # Union: combine and deduplicate
        union_ids = list(dict.fromkeys(target_ids + value_ids))
        
        collection_id = self._create_collection(union_ids, 'union_result')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            logger.info(f"Union {len(target_ids)} + {len(value_ids)} → {len(union_ids)} → ${out_var}")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create union Collection'}
    
    def _execute_intersection(self, action: Dict) -> Dict:
        """
        Intersection of two Collections (A ∩ B) - items in both.
        
        Required: target, value, out
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'intersection target must be $variable'}
        if not isinstance(value_arg, str) or not value_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'intersection value must be $variable'}
        
        target_var = target_arg[1:]
        value_var = value_arg[1:]
        
        target_ids = self._dereference_collection(target_var)
        value_ids = self._dereference_collection(value_var)
        
        if not isinstance(target_ids, list) or not isinstance(value_ids, list):
            return {'status': 'failed', 'reason': 'intersection requires both arguments to be Collections'}
        
        # Intersection: items in both
        intersection_ids = [item for item in target_ids if item in value_ids]
        
        collection_id = self._create_collection(intersection_ids, 'intersection_result')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            logger.info(f"Intersection {len(target_ids)} ∩ {len(value_ids)} → {len(intersection_ids)} → ${out_var}")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create intersection Collection'}
    
    def _execute_difference(self, action: Dict) -> Dict:
        """
        Difference of two Collections (A - B) - items in A but not in B.
        
        Required: target, value, out
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'difference target must be $variable'}
        if not isinstance(value_arg, str) or not value_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'difference value must be $variable'}
        
        target_var = target_arg[1:]
        value_var = value_arg[1:]
        
        target_ids = self._dereference_collection(target_var)
        value_ids = self._dereference_collection(value_var)
        
        if not isinstance(target_ids, list) or not isinstance(value_ids, list):
            return {'status': 'failed', 'reason': 'difference requires both arguments to be Collections'}
        
        # Difference: items in target but not in value
        value_set = set(value_ids)
        difference_ids = [item for item in target_ids if item not in value_set]
        
        collection_id = self._create_collection(difference_ids, 'difference_result')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            logger.info(f"Difference {len(target_ids)} - {len(value_ids)} → {len(difference_ids)} → ${out_var}")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create difference Collection'}
    
    def _execute_remove(self, action: Dict) -> Dict:
        """
        Remove a Note from a Collection (mutates Collection).
        
        Required: target, value, out
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'remove target must be $variable'}
        
        collection_var = target_arg[1:]
        
        if collection_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Collection variable not bound: {collection_var}'}
        
        collection_id = self.plan_bindings[collection_var]
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return {'status': 'failed', 'reason': f'Variable {collection_var} is not a Collection'}
        
        # Resolve value to Note ID (handles both $var and literal Note IDs)
        note_id = self._resolve_id(value_arg)
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            return {'status': 'failed', 'reason': f'Value must be a Note ID, got: {note_id}'}
        
        # Get current Collection content
        note_ids = self._dereference_collection(collection_var)
        if note_id not in note_ids:
            logger.warning(f"Note {note_id} not in Collection {collection_id}, nothing to remove")
            self._bind_variable(out_var, collection_id)
            return {'status': 'success', 'value': collection_id}
        
        # Remove from list
        note_ids.remove(note_id)
        
        # Update Collection via map_node (similar to add but with modified content)
        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(
            f"cognitive/map/collection/add",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=5.0,
            payload=json.dumps({
                'collection_id': collection_id,
                'content': note_ids,  # Send full updated content
                'agent_name': self.agent_name,
                'operation': 'update'  # Signal this is an update, not add
            }).encode('utf-8')
        ):
            if reply.ok:
                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if response.get('success'):
                    self._bind_variable(out_var, collection_id)
                    logger.info(f"Removed {note_id} from {collection_id} (now {len(note_ids)} items) → ${out_var}")
                    return {'status': 'success', 'value': collection_id}
                else:
                    return {'status': 'failed', 'reason': response.get('error', 'Remove failed')}
            break
        
        return {'status': 'failed', 'reason': 'No response from map_node'}
    
    # ==================== Operation Application Helpers ====================
    
    def _apply_operation_to_value(self, tool_name: str, value: Any, reason: str = '', 
                                   additional_args: Dict = None) -> Dict:
        """
        Helper to apply a tool to a single value with optional additional arguments.
        Shared by map and coerce primitives.
        
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
        """Check if variable is empty/falsy or Collection has 0 items"""
        target_arg = condition.get('target')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            target = self._resolve_value(target_arg)
            return not bool(target)
        
        # Check if target is Collection
        target_var = target_arg[1:]
        if target_var in self.plan_bindings:
            target_id = self.plan_bindings[target_var]
            if isinstance(target_id, str) and target_id.startswith('Collection_'):
                note_ids = self._dereference_collection(target_var)
                return len(note_ids) == 0
        
        # Not a Collection - use existing logic
        target = self._resolve_value(target_arg)
        return not bool(target)
    
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
        """Check if value contains substring/element (Note) or Note ID membership (Collection)"""
        target_arg = condition.get('target')
        value_arg = condition.get('value')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            # Not a variable - use existing content check
            target = self._resolve_value(target_arg)
            value = self._resolve_value(value_arg)
            return self._check_content_contains(target, value)
        
        # Check if target is Collection
        target_var = target_arg[1:]
        if target_var in self.plan_bindings:
            target_id = self.plan_bindings[target_var]
            if isinstance(target_id, str) and target_id.startswith('Collection_'):
                # Collection membership check - resolve value to Note ID
                note_ids = self._dereference_collection(target_var)
                if not note_ids:
                    return False
                
                # Resolve value to Note ID
                if isinstance(value_arg, str) and value_arg.startswith('$'):
                    value_var = value_arg[1:]
                    if value_var in self.plan_bindings:
                        value_id = self.plan_bindings[value_var]
                        return value_id in note_ids
                    return False
                else:
                    # Literal value - check if string matches Note ID
                    return value_arg in note_ids
        
        # Not a Collection - use existing Note content check
        target = self._resolve_value(target_arg)
        value = self._resolve_value(value_arg)
        return self._check_content_contains(target, value)
    
    def _check_content_contains(self, target: Any, value: Any) -> bool:
        """Check if value appears in target content (existing logic)"""
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
    
    def _extract_json_from_text(self, text: str) -> Optional[Any]:
        """
        Extract JSON from text with code fences, preambles, and trailing text.
        Based on repair_json logic from llm_api.py.
        
        Returns parsed dict/list or None if no valid JSON found.
        """
        if not isinstance(text, str):
            return None
        
        # Remove code fences first (common in LLM responses)
        cleaned = text.replace("```json", "").replace("```", "").strip()
        
        # Find JSON boundaries (first { to last })
        if not cleaned.startswith('{') and '{' in cleaned:
            start = cleaned.find('{')
            end = cleaned.rfind('}')
            if start >= 0 and end > start:
                cleaned = cleaned[start:end+1]
        
        # Remove newlines outside of string values
        in_string = False
        result = []
        i = 0
        while i < len(cleaned):
            if cleaned[i] == '"' and (i == 0 or cleaned[i-1] != '\\'):
                in_string = not in_string
            if not in_string and cleaned[i] == '\n':
                i += 1
                continue
            result.append(cleaned[i])
            i += 1
        cleaned = ''.join(result)
        
        # Find first complete JSON object by brace counting
        brace_count = 0
        json_end = 0
        for i, char in enumerate(cleaned):
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    json_end = i + 1
                    break
        if json_end > 0:
            cleaned = cleaned[:json_end]
        
        # Parse extracted JSON
        try:
            return json.loads(cleaned)
        except (json.JSONDecodeError, ValueError):
            return None
    
    def _resolve_id(self, value: Any) -> Any:
        """
        Resolve $variable or literal to resource ID (Note_X or Collection_X).
        Does NOT fetch content - returns the ID string for ID-expecting operations.
        
        Args:
            value: Can be "$variable" string, literal Note/Collection ID, or resource name
            
        Returns:
            Resource ID string (Note_X or Collection_X format), or original value if not resolvable
        """
        if not isinstance(value, str):
            return value
        
        # If $variable, lookup in bindings and return ID (no content fetch)
        if value.startswith('$'):
            var_name = value[1:]
            if var_name not in self.plan_bindings:
                error_msg = f"Unbound variable: {value}"
                logger.error(error_msg)
                raise ValueError(error_msg)
            
            resource_id = self.plan_bindings[var_name]
            # Return ID directly (no _get_content call)
            return resource_id
        
        # If starts with Note_ or Collection_, validate and return as-is (literal ID)
        if value.startswith('Note_') or value.startswith('Collection_'):
            # Could optionally validate existence via map_node query here
            return value
        
        # Otherwise return as-is (could be a name for load operation)
        return value
    
    def _resolve_value(self, value: Any) -> Any:
        """
        Resolve $variable to its content value.
        Supports both entire variable references and template strings with embedded variables.
        
        Args:
            value: Can be a literal value, "$variable" string, or template string with embedded "$variable" patterns
            
        Returns:
            Resolved content value (fetches from map_node if resource ID)
        """
        if not isinstance(value, str):
            return value
        
        # Find all $variable patterns in the string
        pattern = r'\$(\w+)'
        matches = re.findall(pattern, value)
        
        # If no variable patterns found, check if it's a literal Note/Collection ID
        if not matches:
            # If literal ID format, fetch its content
            if value.startswith('Note_') or value.startswith('Collection_'):
                content = self._get_content(value)
                if content is not None:
                    return content
                logger.warning(f"Could not fetch content for literal ID {value}, returning as-is")
            return value
        
        # If entire string is a single variable reference (starts with $)
        if value.startswith('$') and len(matches) == 1 and value == f'${matches[0]}':
            var_name = matches[0]
            if var_name not in self.plan_bindings:
                error_msg = f"Unbound variable: {value}"
                logger.error(error_msg)
                raise ValueError(error_msg)
            
            resource_id = self.plan_bindings[var_name]
            
            # If it's a resource ID, fetch content from map_node
            if isinstance(resource_id, str) and (resource_id.startswith('Note_') or resource_id.startswith('Collection_')):
                return self._get_content(resource_id)
            
            # Otherwise return as-is
            return resource_id
        
        # Template string with embedded variables - substitute each
        result = value
        for var_name in set(matches):  # Use set to process each unique variable once
            var_ref = f'${var_name}'
            if var_name in self.plan_bindings:
                resource_id = self.plan_bindings[var_name]
                logger.debug(f"Resolving template variable {var_ref} → {resource_id}")
                
                # If it's a resource ID, fetch content from map_node
                if isinstance(resource_id, str) and (resource_id.startswith('Note_') or resource_id.startswith('Collection_')):
                    resolved_content = self._get_content(resource_id)
                    if resolved_content is None:
                        logger.warning(f"Could not resolve content for {var_ref} (resource_id: {resource_id}), leaving as-is")
                        continue
                    logger.debug(f"Fetched content for {var_ref}: {str(resolved_content)[:50]}...")
                else:
                    resolved_content = resource_id
                
                result = result.replace(var_ref, str(resolved_content))
            else:
                logger.debug(f"Variable {var_ref} not in plan_bindings, leaving as-is")
        
        return result
    
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
    
    def execute_plan_sync(self, plan: Dict, max_steps: int = 1000, max_depth: int = 10, initial_bindings: Dict = None) -> Dict:
        """
        Execute a plan synchronously without turn taking.
        
        Creates an isolated executor instance to prevent state leakage and enable recursion.
        
        Args:
            plan: Plan dict with 'plan' key containing list of actions
            max_steps: Maximum number of steps to execute (prevents infinite loops)
            max_depth: Maximum recursion depth for nested plans (prevents stack overflow)
            initial_bindings: Optional dict of variable bindings to initialize in isolated executor
            
        Returns:
            Dict with:
            - 'status': 'success', 'failed', or 'suspended'
            - 'reason': Error message or suspension reason
            - 'executed_steps': Number of steps executed
            - 'continue': Callback function if suspended (for ask/wait)
        """
        # Create isolated executor instance
        isolated_executor = InfospaceExecutor(
            agent_name=self.agent_name,
            session=self.session,
            map_name=self.map_name,
            llm_client=self.llm_client,
            available_tools=self.available_tools,
            executive_node=self.executive_node
        )
        
        # Copy initial bindings if provided
        if initial_bindings:
            isolated_executor.plan_bindings.update(initial_bindings)
        
        return isolated_executor._execute_plan_sync_internal(plan, max_steps, max_depth)
    
    def _execute_plan_sync_internal(self, plan: Dict, max_steps: int, max_depth: int) -> Dict:
        """
        Internal synchronous plan execution implementation.
        
        Uses frame-based stack for control flow (if/while) and supports suspension (ask/wait).
        """
        # Extract plan steps
        if isinstance(plan, dict) and 'plan' in plan:
            plan_steps = plan['plan']
        else:
            plan_steps = plan if isinstance(plan, list) else []
        
        if not plan_steps:
            return {'status': 'failed', 'reason': 'Plan has no steps', 'executed_steps': 0}
        
        # Initialize step stack (simple list-based)
        step_stack = []
        main_frame = {
            'plan': plan_steps,
            'idx': 0,
            'type': 'main'
        }
        step_stack.append(main_frame)
        
        executed_steps = 0
        
        # Main execution loop
        while step_stack and executed_steps < max_steps:
            if not step_stack:
                break
            
            current = step_stack[-1]
            plan_steps = current['plan']
            idx = current['idx']
            
            # Check if frame is complete
            if idx >= len(plan_steps):
                # Frame complete - handle while loop re-evaluation
                frame_type = current.get('type')
                step_stack.pop()
                
                if frame_type == 'while':
                    # While loop body complete - re-evaluate condition
                    parent = step_stack[-1] if step_stack else None
                    if parent:
                        cond = current.get('condition')
                        outcome = self._evaluate_condition(cond)
                        if outcome:
                            # Re-enter loop
                            iteration_count = current.get('iteration_count', 0) + 1
                            if iteration_count < current.get('max_iterations', 15):
                                step_stack.append({
                                    'plan': current['plan'],
                                    'idx': 0,
                                    'type': 'while',
                                    'condition': cond,
                                    'return_to': current['return_to'],
                                    'iteration_count': iteration_count,
                                    'max_iterations': 15
                                })
                            else:
                                parent['idx'] = current['return_to']
                        else:
                            # Exit loop
                            parent['idx'] = current['return_to']
                elif step_stack:
                    # Regular frame - continue with parent
                    parent = step_stack[-1]
                    parent['idx'] = current.get('return_to', parent['idx'])
                continue
            
            # Get current step
            step = plan_steps[idx]
            stype = step.get('type')
            
            # Handle control flow
            if stype == 'if':
                cond = step.get('condition')
                then_body = step.get('then', [])
                else_body = step.get('else', [])
                
                outcome = self._evaluate_condition(cond)
                branch = 'then' if outcome else ('else' if else_body else None)
                
                if branch:
                    step_stack.append({
                        'plan': step[branch],
                        'idx': 0,
                        'type': f'if_{branch}',
                        'return_to': idx + 1
                    })
                else:
                    current['idx'] = idx + 1
                continue
            
            elif stype == 'while':
                body = step.get('body', [])
                if not isinstance(body, list):
                    logger.error('While body must be a list; skipping')
                    current['idx'] = idx + 1
                    continue
                
                cond = step.get('condition')
                outcome = self._evaluate_condition(cond)
                
                if outcome:
                    # Enter loop
                    iteration_count = current.get('iteration_count', 0)
                    if iteration_count >= current.get('max_iterations', 15):
                        logger.warning('While loop exceeded max iterations')
                        current['idx'] = idx + 1
                        continue
                    
                    step_stack.append({
                        'plan': body,
                        'idx': 0,
                        'type': 'while',
                        'condition': cond,
                        'return_to': idx + 1,
                        'iteration_count': 0,
                        'max_iterations': 15
                    })
                else:
                    # Skip loop
                    current['idx'] = idx + 1
                continue
            
            elif stype == 'wait':
                cond = step.get('condition')
                outcome = self._evaluate_condition(cond)
                
                if outcome:
                    # Condition met - continue
                    current['idx'] = idx + 1
                    continue
                else:
                    # Condition not met - suspend
                    # Store state for continuation
                    self._sync_suspension_state = {
                        'step_stack': step_stack,
                        'executed_steps': executed_steps,
                        'max_steps': max_steps,
                        'max_depth': max_depth,
                        'plan': plan
                    }
                    
                    def continue_wait():
                        """Continue execution after wait condition becomes true"""
                        if not self._sync_suspension_state:
                            return {'status': 'failed', 'reason': 'No suspension state to continue'}
                        state = self._sync_suspension_state
                        self._sync_suspension_state = None
                        return self._resume_sync_execution(state)
                    
                    return {
                        'status': 'suspended',
                        'reason': 'wait',
                        'action': step,
                        'condition': cond,
                        'executed_steps': executed_steps,
                        'continue': continue_wait
                    }
            
            # Execute action
            try:
                result = self.execute_action(step)
                executed_steps += 1
                
                # Publish action result for UI display (if executive_node available)
                if self.executive_node:
                    self.executive_node._publish_action_result(step, result, stype, datetime.now())
                
                # Check for suspension (ask action)
                if stype == 'ask' and result.get('status') == 'success':
                    # Ask action sets up suspension state - return continuation
                    out_var = step.get('out')
                    
                    # Store state for continuation
                    self._sync_suspension_state = {
                        'step_stack': step_stack,
                        'executed_steps': executed_steps,
                        'max_steps': max_steps,
                        'max_depth': max_depth,
                        'plan': plan,
                        'out_var': out_var
                    }
                    
                    def continue_ask(response_value):
                        """Continue execution after ask response received"""
                        if not self._sync_suspension_state:
                            return {'status': 'failed', 'reason': 'No suspension state to continue'}
                        state = self._sync_suspension_state
                        self._sync_suspension_state = None
                        
                        # Bind response to variable
                        if response_value:
                            response_note_id = self._persist_note(response_value, 'ask-response')
                            if response_note_id:
                                self._bind_variable(state['out_var'], response_note_id)
                        else:
                            self._bind_variable(state['out_var'], "Note_null")
                        
                        # Continue execution
                        return self._resume_sync_execution(state)
                    
                    return {
                        'status': 'suspended',
                        'reason': 'ask',
                        'action': step,
                        'out_var': out_var,
                        'executed_steps': executed_steps,
                        'continue': continue_ask
                    }
                
                # Handle while loop completion (checked in frame completion logic above)
                
                # Advance step index
                if result.get('status') == 'failed':
                    # Action failed - log but continue
                    logger.warning(f"Step {executed_steps} failed: {result.get('reason')}")
                
                current['idx'] = idx + 1
                
            except Exception as e:
                logger.error(f"Error executing step {executed_steps}: {e}")
                logger.error(traceback.format_exc())
                return {
                    'status': 'failed',
                    'reason': f'Execution error at step {executed_steps}: {str(e)}',
                    'executed_steps': executed_steps
                }
        
        # Execution complete
        if executed_steps >= max_steps:
            return {
                'status': 'failed',
                'reason': f'Max steps ({max_steps}) exceeded',
                'executed_steps': executed_steps
            }
        
        return {
            'status': 'success',
            'executed_steps': executed_steps,
            'bindings': self.plan_bindings.copy()  # Return final bindings for output extraction
        }
    
    def _resume_sync_execution(self, state: Dict) -> Dict:
        """Resume execution from suspension state"""
        step_stack = state['step_stack']
        executed_steps = state['executed_steps']
        max_steps = state['max_steps']
        max_depth = state['max_depth']
        plan = state['plan']
        
        # Advance past the suspended step
        if step_stack:
            current = step_stack[-1]
            current['idx'] += 1
        
        # Continue execution loop
        while step_stack and executed_steps < max_steps:
            if not step_stack:
                break
            
            current = step_stack[-1]
            plan_steps = current['plan']
            idx = current['idx']
            
            # Check if frame is complete
            if idx >= len(plan_steps):
                # Frame complete - handle while loop re-evaluation
                frame_type = current.get('type')
                step_stack.pop()
                
                if frame_type == 'while':
                    # While loop body complete - re-evaluate condition
                    parent = step_stack[-1] if step_stack else None
                    if parent:
                        cond = current.get('condition')
                        outcome = self._evaluate_condition(cond)
                        if outcome:
                            # Re-enter loop
                            iteration_count = current.get('iteration_count', 0) + 1
                            if iteration_count < current.get('max_iterations', 15):
                                step_stack.append({
                                    'plan': current['plan'],
                                    'idx': 0,
                                    'type': 'while',
                                    'condition': cond,
                                    'return_to': current['return_to'],
                                    'iteration_count': iteration_count,
                                    'max_iterations': 15
                                })
                            else:
                                parent['idx'] = current['return_to']
                        else:
                            # Exit loop
                            parent['idx'] = current['return_to']
                elif step_stack:
                    # Regular frame - continue with parent
                    parent = step_stack[-1]
                    parent['idx'] = current.get('return_to', parent['idx'])
                continue
            
            # Get current step
            step = plan_steps[idx]
            stype = step.get('type')
            
            # Handle control flow (same as main execution)
            if stype == 'if':
                cond = step.get('condition')
                then_body = step.get('then', [])
                else_body = step.get('else', [])
                
                outcome = self._evaluate_condition(cond)
                branch = 'then' if outcome else ('else' if else_body else None)
                
                if branch:
                    step_stack.append({
                        'plan': step[branch],
                        'idx': 0,
                        'type': f'if_{branch}',
                        'return_to': idx + 1
                    })
                else:
                    current['idx'] = idx + 1
                continue
            
            elif stype == 'while':
                body = step.get('body', [])
                if not isinstance(body, list):
                    logger.error('While body must be a list; skipping')
                    current['idx'] = idx + 1
                    continue
                
                cond = step.get('condition')
                outcome = self._evaluate_condition(cond)
                
                if outcome:
                    iteration_count = current.get('iteration_count', 0)
                    if iteration_count >= current.get('max_iterations', 15):
                        logger.warning('While loop exceeded max iterations')
                        current['idx'] = idx + 1
                        continue
                    
                    step_stack.append({
                        'plan': body,
                        'idx': 0,
                        'type': 'while',
                        'condition': cond,
                        'return_to': idx + 1,
                        'iteration_count': 0,
                        'max_iterations': 15
                    })
                else:
                    current['idx'] = idx + 1
                continue
            
            elif stype == 'wait':
                cond = step.get('condition')
                outcome = self._evaluate_condition(cond)
                
                if outcome:
                    current['idx'] = idx + 1
                    continue
                else:
                    # Suspend again
                    self._sync_suspension_state = {
                        'step_stack': step_stack,
                        'executed_steps': executed_steps,
                        'max_steps': max_steps,
                        'max_depth': max_depth,
                        'plan': plan
                    }
                    
                    def continue_wait():
                        if not self._sync_suspension_state:
                            return {'status': 'failed', 'reason': 'No suspension state to continue'}
                        state = self._sync_suspension_state
                        self._sync_suspension_state = None
                        return self._resume_sync_execution(state)
                    
                    return {
                        'status': 'suspended',
                        'reason': 'wait',
                        'action': step,
                        'condition': cond,
                        'executed_steps': executed_steps,
                        'continue': continue_wait
                    }
            
            # Execute action
            try:
                result = self.execute_action(step)
                executed_steps += 1
                
                # Publish action result for UI display (if executive_node available)
                if self.executive_node:
                    self.executive_node._publish_action_result(step, result, stype, datetime.now())
                
                # Check for ask suspension
                if stype == 'ask' and result.get('status') == 'success':
                    out_var = step.get('out')
                    self._sync_suspension_state = {
                        'step_stack': step_stack,
                        'executed_steps': executed_steps,
                        'max_steps': max_steps,
                        'max_depth': max_depth,
                        'plan': plan,
                        'out_var': out_var
                    }
                    
                    def continue_ask(response_value):
                        if not self._sync_suspension_state:
                            return {'status': 'failed', 'reason': 'No suspension state to continue'}
                        state = self._sync_suspension_state
                        self._sync_suspension_state = None
                        
                        if response_value:
                            response_note_id = self._persist_note(response_value, 'ask-response')
                            if response_note_id:
                                self._bind_variable(state['out_var'], response_note_id)
                        else:
                            self._bind_variable(state['out_var'], "Note_null")
                        
                        return self._resume_sync_execution(state)
                    
                    return {
                        'status': 'suspended',
                        'reason': 'ask',
                        'action': step,
                        'out_var': out_var,
                        'executed_steps': executed_steps,
                        'continue': continue_ask
                    }
                
                # While loop completion handled in frame completion logic above
                
                if result.get('status') == 'failed':
                    logger.warning(f"Step {executed_steps} failed: {result.get('reason')}")
                
                current['idx'] = idx + 1
                
            except Exception as e:
                logger.error(f"Error executing step {executed_steps}: {e}")
                logger.error(traceback.format_exc())
                return {
                    'status': 'failed',
                    'reason': f'Execution error at step {executed_steps}: {str(e)}',
                    'executed_steps': executed_steps
                }
        
        if executed_steps >= max_steps:
            return {
                'status': 'failed',
                'reason': f'Max steps ({max_steps}) exceeded',
                'executed_steps': executed_steps
            }
        
        return {
            'status': 'success',
            'executed_steps': executed_steps,
            'bindings': self.plan_bindings.copy()  # Return final bindings for output extraction
        }
    
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
                'expect': 'test output'
            },
            'focus': {
                'type': 'focus',
                'target': 'question-decomposer',
                'expect': 'test focus'
            },
            'create': {
                'type': 'create',
                'kind': 'Note',
                'value': 'test note content',
                'out': 'test_note',
                'expect': 'test create'
            },
            'index': {
                'type': 'index',
                'source': '$test_data',
                'index_type': 'keyword'
            },
            'organize': {
                'type': 'organize',
                'source': '$test_data',
                'index_type': 'keyword'
            },
            'search': {
                'type': 'search',
                'source': '$test_data',
                'query': 'test query',
                'mode': 'keyword',
                'out': 'test_results',
                'expect': 'test search results'
            },
            'say': {
                'type': 'say',
                'value': 'test output message',
                'target': 'user',
                'expect': 'test say'
            },
            'think': {
                'type': 'think',
                'value': 'test internal thought',
                'expect': 'test think'
            },
            'extract': {
                'type': 'extract',
                'target': {'test': 'value'},
                'field': 'test',
                'out': 'test_extracted',
                'expect': 'test extract'
            },
            'filter': {
                'type': 'filter',
                'target': [{'val': 1}, {'val': 2}, {'val': 3}],
                'condition': {'field': 'val', 'operator': 'gt', 'value': 0},
                'out': 'test_filtered',
                'expect': 'test filter'
            },
            'map': {
                'type': 'map',
                'target': '$test_filtered',
                'operation': {'type': 'extract', 'field': 'val'},
                'out': 'test_mapped',
                'expect': 'test map'
            },
            'merge': {
                'type': 'merge',
                'targets': [{'a': 1}, {'b': 2}],
                'out': 'test_merged',
                'expect': 'test merge'
            },
            'coerce': {
                'type': 'coerce',
                'target': [[1, 2], [3, 4]],
                'operation': 'flatten',
                'out': 'test_coerced',
                'expect': 'test coerce'
            },
            'aggregate': {
                'type': 'aggregate',
                'target': [1, 2, 3],
                'operation': 'count',
                'out': 'test_aggregated',
                'expect': 'test aggregate'
            },
            'sort': {
                'type': 'sort',
                'target': [{'val': 3}, {'val': 1}, {'val': 2}],
                'by': 'val',
                'order': 'asc',
                'out': 'test_sorted',
                'expect': 'test sort'
            },
            'group_by': {
                'type': 'group_by',
                'target': [{'type': 'a'}, {'type': 'b'}],
                'by': 'type',
                'out': 'test_grouped',
                'expect': 'test group'
            },
            'compare': {
                'type': 'compare',
                'targets': ['test', 'test'],
                'out': 'test_compared',
                'expect': 'test compare'
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

