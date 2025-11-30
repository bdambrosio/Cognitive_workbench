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
from typing import Dict, List, Any, Optional, Tuple
from datetime import datetime

# Try to import SGLang for runtime support
try:
    import sglang as sgl
    from sglang import function, user, assistant, gen
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False

logger = logging.getLogger(__name__)


class InfospaceExecutor:
    """
    Executor for information space primitives.
    
    Core Primitives (whole-value operations only):
    - Computation: apply, map, coerce
    - Storage: save, load, create
    - Indexing: index, search
    - Communication: say, think
    
    Design principle: Content is opaque. Field-based operations delegated to tools.
    """
    
    def __init__(self, agent_name: str, session, map_name: str, llm_client, available_tools: Dict[str, Dict], executive_node=None, resource_manager=None):
        """
        Initialize infospace executor.
        
        Args:
            agent_name: Name of the agent
            session: Zenoh session for communication
            map_name: Name of the map (for Zenoh topics)
            llm_client: LLM client for tool execution
            available_tools: Dict of tool_name -> metadata (from tool_loader)
            executive_node: Reference to ZenohExecutiveNode (for ask action)
            resource_manager: InfospaceResourceManager instance for direct resource access
        """
        self.agent_name = agent_name
        self.session = session
        self.map_name = map_name
        self.llm_client = llm_client
        self.available_tools = available_tools
        self.executive_node = executive_node
        self.resource_manager = resource_manager
        
        # Plan-local state (ephemeral, cleared each plan)
        self.plan_bindings = {}  # $var_name -> resource_id (Note_N or Collection_N)
        
        # Agent state (persistent across plans)
        self.agent_position = None
        
        # Suspension state for sync execution
        self._sync_suspension_state = None
        
        # SGLang runtime (set by IncrementalPlanner if available)
        self.runtime = None
        
        logger.info(f"InfospaceExecutor initialized for {agent_name} with {len(available_tools)} tools")
    
    def clear_plan_state(self):
        """Clear ephemeral plan state (call at start of new plan)"""
        self.plan_bindings = {}
        if self.executive_node:
            self.executive_node.last_say_text = ''
            self.executive_node.last_out_resource_id = None
    
    def _create_collection(self, note_ids: list, source_context: str, collection_name: str = '', properties: Optional[Dict] = None) -> Optional[str]:
        """
        Create a Collection resource.
        
        Args:
            note_ids: List of Note IDs (e.g., ["Note_2", "Note_3"])
            source_context: Description for logging (e.g., 'map_operation', 'create_collection')
            collection_name: Optional stable name for the Collection
            properties: Optional dict of extra properties to attach
            
        Returns:
            Collection ID if successful, None if failed
        """
        if not self.resource_manager:
            logger.error("Resource manager not available")
            return None
        
        success, collection_id, error_msg, location = self.resource_manager.create_collection(
            self.agent_name, note_ids, 'list', source_context, f'{len(note_ids)} items', collection_name, properties or {}
        )
        
        if success:
            return collection_id
        else:
            logger.error(f'Failed to create Collection: {error_msg}')
            return None
    
    def _get_content(self, resource_id: str) -> Any:
        """
        Fetch content for a given resource ID.
        
        Args:
            resource_id: Note_N or Collection_N ID
            
        Returns:
            Content value, or None if not found
        """
        if resource_id == "Note_null":
            return None
        
        if not self.resource_manager:
            logger.error("Resource manager not available")
            return None
        
        resource = self.resource_manager.get_resource(resource_id)
        if not resource:
            logger.warning(f"Resource {resource_id} not found")
            return None
        
        return resource.get('properties', {}).get('content')
    
    def get_resource_metadata(self, resource_id: str) -> Optional[Dict]:
        """
        Fetch metadata for a given resource ID.
        
        Args:
            resource_id: Note_N or Collection_N ID
            
        Returns:
            Dict with metadata (item_count, source_skill, etc.), or None if not found
        """
        if resource_id == "Note_null":
            return None
        
        if not self.resource_manager:
            return None
        
        resource = self.resource_manager.get_resource(resource_id)
        if not resource:
            return None
        
        return resource.get('properties', {})
    
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
            'create-note': self._execute_create_note,
            'create-collection': self._execute_create_collection,
            'persist': self._execute_persist,
            'load': self._execute_load,
            'index': self._execute_index,
            'organize': self._execute_index,  # Alias for index
            'search-within-collection': self._execute_search_within_collection,
            'search-notes': self._execute_search_notes,
            'search-collections': self._execute_search_collections,
            'say': self._execute_say,
            'display': self._execute_display,
            'think': self._execute_think,
            'ask': self._execute_ask,
            # Phase 2: Data Operations (whole-value only)
            'coerce': self._execute_coerce,
            'map': self._execute_map,
            'flatten': self._execute_flatten,
            'add': self._execute_add,
            'split': self._execute_split,
            # Set operations
            'size': self._execute_size,
            'union': self._execute_union,
            'intersection': self._execute_intersection,
            'difference': self._execute_difference,
            'remove': self._execute_remove,
            # Level 4: Structured data operations
            'project': self._execute_project,
            'pluck': self._execute_pluck,
            'head': self._execute_head,
            'filter-structured': self._execute_filter_structured,
            'sort': self._execute_sort,
            'join': self._execute_join,
        }
        
        # Validate 'out' field has $ prefix if present
        if 'out' in action:
            out_val = action['out']
            if isinstance(out_val, str) and out_val and not out_val.startswith('$'):
                error_msg = f"Invalid 'out' field: '{out_val}' must start with $ (use '${out_val}')"
                logger.error(error_msg)
                return {'status': 'failed', 'reason': error_msg}
        
        logger.info(f"Executing action: {json.dumps(action)}")
        handler = handlers.get(action_type)
        if not handler:
            # Check if it's a dynamic tool (not a primitive)
            if action_type in self.available_tools:
                # Route to apply logic for tool execution
                return self._execute_apply(action)
            else:
                logger.error(f"Unknown action type: {action_type}")
                return {'status': 'failed', 'reason': f'Unknown action: {action_type}'}
        
        try:
            result = handler(action)
            if True: # bypass expectation comparison for now
                return result
            # Process expectation comparison if action has 'expect' field
            if action.get('expect') and result.get('status') == 'success':
                try:
                    from utils.action_post_processing import process_action_expectation
                    
                    # Create llm_generate wrapper that matches the interface (SGLang only)
                    def llm_generate_wrapper(messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
                        """Wrapper for action_post_processing using SGLang runtime."""
                        if not self.runtime:
                            raise RuntimeError("SGLang runtime not available")
                        # Apply bindings to messages if provided
                        if bindings and isinstance(messages, list):
                            processed_messages = []
                            for msg in messages:
                                if isinstance(msg, str):
                                    # Apply template bindings
                                    processed_msg = msg
                                    for key, value in bindings.items():
                                        processed_msg = processed_msg.replace(f"{{{{${key}}}}}", str(value))
                                    processed_messages.append(processed_msg)
                                else:
                                    processed_messages.append(msg)
                            messages = processed_messages
                        return self._sglang_generate(messages, max_tokens, temperature, stops, is_json=is_json)
                    
                    comparison = process_action_expectation(action, result, llm_generate_wrapper, self.resource_manager, debug=False)
                    if comparison and not comparison.get('matched'):
                        logger.warning(f"Expectation mismatch for {action_type}: {comparison.get('response', '')}")
                except Exception as e:
                    logger.debug(f"Expectation processing failed (non-critical): {e}")
            
            return result
        except Exception as e:
            logger.error(f"Error executing action {action_type}: {e}")
            logger.error(traceback.format_exc())
            return {'status': 'failed', 'reason': f'Execution error: {str(e)}'}
    
    # ==================== Core Operations ====================
        
    def _execute_apply(self, action: Dict) -> Dict:
        """
        Apply tool to input data.
        
        Supports two formats:
        1. Standard: {"type": "apply", "target": "tool-name", "value": ..., "out": "$var"}
        2. Direct:   {"type": "tool-name", "args": {...}, "out": "$var"}
        
        Required: type, out
        Optional: target (input data), any tool-specific parameters at top level
        
        All tool parameters are at top level (flat format):
        - target: $variable (input data) - resolves to Note/Collection content
        - out: literal string (variable name)
        - Any other fields: tool-specific parameters (prompt, query, instruction, etc.)
        """
        action_type = action.get('type')
        
        # Reserved fields that are NOT tool parameters
        reserved_fields = {'type', 'target', 'value', 'out', 'expect', 'reason'}
        
        # Direct format: type is the tool name, target or value is input data
        if action_type in self.available_tools:
            target = action_type  # Tool name
            # Input data: prefer 'target', fall back to 'value'
            value = self._resolve_value(action.get('target') or action.get('value', ''))
        else:
            # Standard apply format: target is the tool name
            target = self._resolve_value(action.get('target'))
            value = ''
        
        out_var = action.get('out')
        
        # Extract tool parameters from top level (flat format)
        additional_args = {}
        for key, val in action.items():
            if key not in reserved_fields:
                additional_args[key] = val
        
        if not target:
            return {'status': 'failed', 'reason': 'apply requires target or tool name in type'}
        
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
            # Special handling for Level 4 tools: return Collection ID directly
            level4_tools = ['filter-collection', 'query-web', 'semantic-scholar']
            if target in level4_tools and isinstance(result_value, str) and result_value.startswith('Collection_'):
                # Result is already a Collection ID - bind directly
                self._bind_variable(out_var, result_value)
                # out_var might already have $ prefix, normalize for display
                display_var = out_var if out_var.startswith('$') else f"${out_var}"
                logger.info(f"Tool '{target}' executed, Collection {result_value} → {display_var}")
                return {'status': 'success', 'value': result_value}
            
            # Default: Persist result as Note
            info_id = self._persist_note(result_value, f'apply_{target}')
            if info_id:
                self._bind_variable(out_var, info_id)
                # out_var might already have $ prefix, normalize for display
                display_var = out_var if out_var.startswith('$') else f"${out_var}"
                logger.info(f"Tool '{target}' executed, Note {info_id} → {display_var}")
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
            
            # Create unified LLM generation wrapper (SGLang only)
            def llm_generate(messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
                """Unified LLM generation interface using SGLang runtime."""
                if not self.runtime:
                    raise RuntimeError("SGLang runtime not available")
                # Apply bindings to messages if provided
                if bindings and isinstance(messages, list):
                    processed_messages = []
                    for msg in messages:
                        if isinstance(msg, str):
                            # Apply template bindings
                            processed_msg = msg
                            for key, value in bindings.items():
                                processed_msg = processed_msg.replace(f"{{{{${key}}}}}", str(value))
                            processed_messages.append(processed_msg)
                        else:
                            processed_messages.append(msg)
                    messages = processed_messages
                return self._sglang_generate(messages, max_tokens, temperature, stops, is_json=is_json)
            
            # Call LLM using unified wrapper
            llm_response = llm_generate(
                messages=[full_prompt],
                bindings={},
                max_tokens=1000,
                temperature=0.3,
                stops=['</end>']
            )
            
            # Handle response format (SGLang returns dict with 'text', llm_client returns object with .text)
            if isinstance(llm_response, dict):
                result_text = llm_response.get('text', '').strip()
            elif hasattr(llm_response, 'text'):
                result_text = llm_response.text.strip()
            elif isinstance(llm_response, str):
                result_text = llm_response.strip()
            else:
                return {'status': 'failed', 'reason': 'LLM returned invalid response format'}
            
            if not result_text:
                return {'status': 'failed', 'reason': 'LLM returned empty response'}
            logger.info(f"Prompt tool '{tool_name}' completed ({len(result_text)} chars)")
            
            # Check for null indicator - return Note_null resource ID instead of string
            if result_text.lower() == 'note-null' or result_text.lower() == 'null':
                logger.info(f"Prompt tool '{tool_name}' returned null indicator, using Note_null resource")
                return {'status': 'success', 'value': 'Note_null'}
            
            return {'status': 'success', 'value': result_text}
            
        except Exception as e:
            logger.error(f"Prompt tool execution failed: {e}")
            return {'status': 'failed', 'reason': str(e)}

    def _sglang_generate(self, messages, max_tokens=2000, temperature=0.7, stops=None, is_json=False):
        """
        Generate text using SGLang Runtime.
        Wraps SGLang's function decorator pattern to match llm_client.generate() interface.
        
        Args:
            messages: List of message strings (or single string)
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            stops: List of stop sequences
            is_json: If True, parse response as JSON and return dict
            
        Returns:
            Object with .success, .text, and .error attributes (matching llm_client response)
            If is_json=True, .text will be a dict (parsed JSON) instead of string
        """
        if not HAS_SGLANG or not self.runtime:
            return type('Response', (), {'success': False, 'error': 'SGLang not available', 'text': ''})()

        class Response:
            def __init__(self, success, text='', error=None):
                self.success = success
                self.text = text
                self.error = error

        try:
            # Convert messages to single prompt string
            if isinstance(messages, list):
                prompt = '\n'.join(str(m) for m in messages)
            else:
                prompt = str(messages)
            
            # Create a simple generation function
            @function
            def generate_text(s, prompt_text):
                s += user(prompt_text)
                # Convert stops list to string (SGLang uses stop as string, take first if list)
                stop_str = None
                if stops:
                    if isinstance(stops, list) and len(stops) > 0:
                        stop_str = stops[0]  # SGLang gen() uses single stop string
                    elif isinstance(stops, str):
                        stop_str = stops
                gen_kwargs = {"max_tokens": max_tokens, "temperature": temperature}
                if stop_str:
                    gen_kwargs["stop"] = stop_str
                s += assistant(gen("output", **gen_kwargs))
            
            # Run the function
            state = generate_text.run(prompt_text=prompt)
            result_text = state["output"].strip()
            
            # Post-process JSON if requested
            if is_json:
                result_text = self._parse_json_response(result_text, prompt)
            
            return Response(success=True, text=result_text)
        except Exception as e:
            logger.error(f"SGLang generation error: {e}")
            traceback.print_exc()
            return Response(success=False, error=str(e))
    
    def _parse_json_response(self, response_text, original_prompt=None):
        """
        Parse JSON from response text, with repair mechanism.
        Adapted from llm_api.py repair_json logic.
        
        Args:
            response_text: Raw text response that should contain JSON
            original_prompt: Original prompt (for error logging)
            
        Returns:
            Parsed dict if successful, None if parsing fails after repair attempts
        """
        if isinstance(response_text, dict):
            return response_text
        
        if not isinstance(response_text, str):
            response_text = str(response_text)
        
        response = response_text.strip()
        
        # Remove markdown code fences if present
        response = response.replace("```json", "").replace("```", "").strip()
        
        # Try direct parse first
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            pass
        
        # Repair attempt 1: Extract JSON if not at start
        if not response.startswith('{') and '{' in response:
            start = response.find('{')
            end = response.rfind('}')
            if start >= 0 and end >= start:
                response = response[start:end+1]
        
        # Repair attempt 2: Remove newlines outside string values
        in_string = False
        result = []
        i = 0
        while i < len(response):
            if response[i] == '"' and (i == 0 or response[i-1] != '\\'):
                in_string = not in_string
            if not in_string and response[i] == '\n':
                i += 1
                continue
            result.append(response[i])
            i += 1
        response = ''.join(result)
        
        # Repair attempt 3: Find first complete JSON object
        brace_count = 0
        json_end = 0
        for i, char in enumerate(response):
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    json_end = i + 1
                    break
        if json_end > 0:
            response = response[:json_end]
        
        # Try parsing after repairs
        try:
            return json.loads(response)
        except json.JSONDecodeError as e:
            logger.error(f'Simple JSON repair failed: {e}')
            logger.debug(f'Failed to parse JSON from: {response_text[:200]}...')
            
            # Fallback: Use LLM (sglang runtime) to repair JSON
            if self.runtime and original_prompt:
                logger.info('Attempting LLM-based JSON repair using sglang runtime')
                repair_prompt = f"""You are a JSON repair tool.
An LLM received the following prompt and returned invalid JSON. Your task is to repair the JSON.

The prompt was:
{original_prompt}

The returned JSON was:
{response_text}

The reported error was:
{str(e)}

If it seems the JSON was truncated, it may have exceeded the max_tokens limit. In that case, try shortening some string values and completing the JSON according to the prompt.
Respond only with the repaired JSON string. Do not output any reasoning.
Make sure the string is in a format that can be parsed by the json.loads function. No commentary, no code fences.
"""
                try:
                    repair_response = self._sglang_generate([repair_prompt], max_tokens=3500, temperature=0.2, stops=None, is_json=False)
                    if repair_response.success:
                        repaired_text = repair_response.text.strip()
                        # Remove code fences if present
                        repaired_text = repaired_text.replace("```json", "").replace("```", "").strip()
                        try:
                            return json.loads(repaired_text)
                        except json.JSONDecodeError as e2:
                            logger.error(f'LLM repair response also failed to parse: {e2}')
                            logger.debug(f'Repair response: {repaired_text[:200]}...')
                except Exception as repair_error:
                    logger.error(f'LLM-based JSON repair failed: {repair_error}')
            
            return None

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
            
            # Bind main input to $input variable
            if input_value is not None:
                input_note_id = self._persist_note(input_value, f'{tool_name}_input')
                if input_note_id:
                    self.plan_bindings['input'] = input_note_id
            
            # Bind args dict values to named variables
            for key, val in additional_args.items():
                resolved_val = self._resolve_value(val)
                arg_note_id = self._persist_note(resolved_val, f'{tool_name}_{key}')
                if arg_note_id:
                    self.plan_bindings[key] = arg_note_id
            
            # Execute plan synchronously
            result = self.execute_plan_sync(plan_data)
            
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
        
        # Add map_name and other kwargs for tools
        resolved_args['map_name'] = self.map_name
        resolved_args['agent_name'] = self.agent_name
        resolved_args['resource_manager'] = self.resource_manager
        
        # Add unified LLM generation callback (SGLang only)
        def llm_generate(messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
            """Unified LLM generation interface using SGLang runtime."""
            if not self.runtime:
                raise RuntimeError("SGLang runtime not available - ensure executive_node is started with sgl_model_path")
            # Apply bindings to messages if provided
            if bindings and isinstance(messages, list):
                processed_messages = []
                for msg in messages:
                    if isinstance(msg, str):
                        processed_msg = msg
                        for key, value in bindings.items():
                            processed_msg = processed_msg.replace(f"{{{{${key}}}}}", str(value))
                        processed_messages.append(processed_msg)
                    else:
                        processed_messages.append(msg)
                messages = processed_messages
            return self._sglang_generate(messages, max_tokens, temperature, stops, is_json=is_json)
        
        resolved_args['llm_generate'] = llm_generate
        
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

    def _execute_create_note(self, action: Dict) -> Dict:
        """
        Create a Note object as persistent spatial resource.
        
        Required: value, out
        Optional: name (stable name for Note), properties (dict of extra metadata)
        
        Named Notes can be loaded by name (e.g., "my-note") or by ID (e.g., "Note_123").
        """
        value_arg = action.get('value')
        out_var = action.get('out')
        note_name = action.get('name', '')
        extra_props = action.get('properties')
        
        if not out_var:
            return {'status': 'failed', 'reason': 'create-note requires out'}
        
        value = self._resolve_value(value_arg) if value_arg is not None else ''
        
        if value is None:
            self._bind_variable(out_var, "Note_null")
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Created null Note → {display_var} = Note_null")
            return {'status': 'success', 'value': "Note_null"}
        
        info_id = self._persist_note(value, 'create-note-primitive', extra_props, note_name)
        if info_id:
            self._bind_variable(out_var, info_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Created Note {info_id} → {display_var}")
            return {'status': 'success', 'value': info_id}
        
        display_var = self._normalize_var_for_log(out_var)
        logger.error(f"Note creation failed for {display_var}")
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
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Created {collection_id}{name_display} → {display_var} ({len(note_ids)} items)")
            return {'status': 'success', 'value': collection_id}
        
        display_var = self._normalize_var_for_log(out_var)
        logger.error(f"Collection creation failed for {display_var}")
        return {'status': 'failed', 'reason': 'Failed to create Collection'}
    
    # ==================== Storage Operations ====================
    
    def _extract_text_preview(self, content: Any, max_chars: int = 200) -> Tuple[str, str]:
        """
        Extract first paragraph from content and truncate to max_chars.
        
        Args:
            content: Content to extract preview from (string, dict, list, etc.)
            max_chars: Maximum characters (default 200)
            
        Returns:
            Tuple of (text_preview, format_type)
        """
        # Convert content to string
        if isinstance(content, (dict, list)):
            content_str = json.dumps(content)
            format_type = 'json'
        else:
            content_str = str(content) if content is not None else ''
            format_type = 'text'
        
        if not content_str:
            return ('', format_type)
        
        # Extract first paragraph (split on double newlines or single newline)
        paragraphs = content_str.split('\n\n')
        if len(paragraphs) == 1:
            # No double newlines, try single newline
            paragraphs = content_str.split('\n')
        
        first_para = paragraphs[0].strip() if paragraphs else ''
        
        # Truncate to max_chars with '...'
        if len(first_para) > max_chars:
            preview = first_para[:max_chars - 3] + '...'
        else:
            preview = first_para
        
        return (preview, format_type)
    
    def _build_structured_search_result(self, resource_id: str, resource_result: Dict, 
                                       resource_metadata: Optional[Dict] = None) -> Dict:
        """
        Build structured search result Note matching query-web/semantic-scholar format.
        
        Args:
            resource_id: Note or Collection ID
            resource_result: Search result dict with 'score', 'type', etc.
            resource_metadata: Optional metadata from get_resource_metadata()
            
        Returns:
            Structured dict with text, format, metadata, char_count
        """
        # Get original content
        original_content = self._get_content(resource_id)
        
        # Extract full text content (not truncated)
        if isinstance(original_content, dict):
            if 'text' in original_content:
                full_text = str(original_content['text'])
            else:
                full_text = json.dumps(original_content)
            format_type = 'json'
        elif isinstance(original_content, list):
            full_text = json.dumps(original_content)
            format_type = 'json'
        else:
            full_text = str(original_content) if original_content else ''
            format_type = 'text'
        
        # Build metadata
        metadata = {
            'source_id': resource_id,
            'uri': resource_id,  # URI field for consistency (Note/Collection ID can be used as URI)
            'score': resource_result.get('score', 0.0),
            'type': resource_result.get('type', 'Note')
        }
        
        # If original Note has a URI in metadata, use that instead
        if isinstance(original_content, dict) and 'metadata' in original_content:
            original_uri = original_content['metadata'].get('uri') or original_content['metadata'].get('url')
            if original_uri:
                metadata['uri'] = original_uri
        
        # Add optional metadata from original resource
        if resource_metadata:
            if 'source_skill' in resource_metadata:
                metadata['source_skill'] = resource_metadata['source_skill']
            if 'created_at' in resource_metadata:
                metadata['created_at'] = resource_metadata['created_at']
            if 'note_name' in resource_metadata:
                metadata['note_name'] = resource_metadata['note_name']
        
        return {
            'text': full_text,
            'format': format_type,
            'metadata': metadata,
            'char_count': len(full_text)
        }
    
    def _persist_note(self, value: Any, source_context: str, properties: Optional[Dict] = None, note_name: str = '') -> Optional[str]:
        """
        Helper to persist a Note as a resource.
        
        Args:
            value: Content to persist
            source_context: Description for logging (e.g., 'save_primitive', 'apply_result')
            properties: Optional dict of extra properties to attach
            note_name: Optional stable name for referencing
            
        Returns:
            Note ID if successful, None if failed
        """
        if value is None:
            return "Note_null"

        # If value is already Note_null resource ID, return it directly
        if isinstance(value, str) and value == "Note_null":
            return "Note_null"

        if not self.resource_manager:
            logger.error("Resource manager not available")
            return None
        
        format_type = 'json' if isinstance(value, (dict, list)) else 'text'
        
        success, note_id, error_msg, location = self.resource_manager.create_note(
            self.agent_name, value, format_type, source_context, str(value)[:100], note_name, properties or {}
        )
        
        if success:
            return note_id
        else:
            logger.error(f'Failed to create Note: {error_msg}')
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
        
        # Mark as persistent using resource manager
        if not self.resource_manager:
            return {'status': 'failed', 'reason': 'Resource manager not available'}
        
        success, error_msg = self.resource_manager.mark_persistent(resource_id, self.agent_name)
        
        if success:
            logger.info(f"Marked {resource_id} as persistent")
            return {'status': 'success', 'value': resource_id}
        else:
            logger.error(f'Failed to persist resource: {error_msg}')
            return {'status': 'failed', 'reason': error_msg or 'Unknown error'}
    
    def search_resources(self, queries: List[str], k_notes: int = 3, k_collections: int = 2, threshold: float = 0.3) -> Dict:
        """
        Search for relevant Notes and Collections using embedding-based retrieval.
        Used by Stage 0 of incremental planner.
        
        Args:
            queries: List of search query strings
            k_notes: Number of Notes to return
            k_collections: Number of Collections to return
            threshold: Minimum similarity score
            
        Returns:
            Dict with 'notes' and 'collections' lists, or error dict
        """
        if not self.resource_manager:
            return {
                'status': 'failed',
                'reason': 'Resource manager not available',
                'notes': [],
                'collections': []
            }
        
        try:
            # Search both indexes for each query, merge results
            all_notes = {}
            all_collections = {}
            
            for query_text in queries:
                # Search Notes
                note_results = self.resource_manager.resource_indexer.search_notes(
                    query_text, k=k_notes, threshold=threshold
                )
                for result in note_results:
                    resource_id = result['resource_id']
                    # Keep highest score if duplicate
                    if resource_id not in all_notes or result['score'] > all_notes[resource_id]['score']:
                        all_notes[resource_id] = result
                
                # Search Collections
                collection_results = self.resource_manager.resource_indexer.search_collections(
                    query_text, k=k_collections, threshold=threshold
                )
                for result in collection_results:
                    resource_id = result['resource_id']
                    # Keep highest score if duplicate
                    if resource_id not in all_collections or result['score'] > all_collections[resource_id]['score']:
                        all_collections[resource_id] = result
            
            # Sort by score and take top k
            notes_list = sorted(all_notes.values(), key=lambda x: x['score'], reverse=True)[:k_notes]
            collections_list = sorted(all_collections.values(), key=lambda x: x['score'], reverse=True)[:k_collections]
            
            return {
                'status': 'success',
                'notes': notes_list,
                'collections': collections_list
            }
        except Exception as e:
            logger.warning(f"Exception during resource search: {e}")
            import traceback
            traceback.print_exc()
            return {
                'status': 'failed',
                'reason': f'Exception: {str(e)}'
            }
        
        return {'status': 'failed', 'reason': 'Unknown error'}
    
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
            
        # Resolve variable if resource_id starts with $
        if isinstance(resource_id, str) and resource_id.startswith('$'):
            var_name = resource_id[1:]
            if var_name in self.plan_bindings:
                resource_id = self.plan_bindings[var_name]
                logger.debug(f"Resolved load resource_id: ${var_name} -> {resource_id}")
            else:
                return {'status': 'failed', 'reason': f'Variable not bound: {var_name}'}
        
        # Direct call to resource manager
        resource = self.resource_manager.get_resource(resource_id)
        
        if not resource:
            return {'status': 'failed', 'reason': f'Resource not found: {resource_id}'}
        
        # Get the actual resource ID
        actual_id = resource.get('name') or resource_id
        
        # Bind the actual resource ID to variable
        self._bind_variable(out_var, actual_id)
        
        # Determine resource type for logging
        resource_type = "Collection" if actual_id.startswith('Collection_') else "Note" if actual_id.startswith('Note_') else "Resource"
        display_var = self._normalize_var_for_log(out_var)
        logger.info(f"Loaded {resource_id} → {display_var} ({resource_type})")
        
        return {'status': 'success', 'value': actual_id}
    
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
        if not self.resource_manager:
            return {'status': 'failed', 'reason': 'Resource manager not available'}
        
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
        
        # Call resource_manager directly
        success, indexed_count, error_msg = self.resource_manager.index_collection(
            agent_name=self.agent_name,
            collection_id=collection_id,
            index_type=index_type,
            fields=fields
        )
        
        if not success:
            return {'status': 'failed', 'reason': error_msg or 'Index failed'}
        
        logger.info(f"Indexed {indexed_count} chunks from {collection_id}")
        return {'status': 'success', 'value': indexed_count}
    
    def _execute_search_within_collection(self, action: Dict) -> Dict:
        """
        Search within a specific indexed Collection.
        
        Required: target, value, out
        Optional: mode, limit, threshold, return_mode
        
        Argument types:
        - target: $variable (indexed Collection to search within)
        - value: literal string OR $variable (resolves to query text)
        - mode: literal string ('semantic' or 'keyword', default 'semantic')
        - limit: int (max results to return, default 5)
        - threshold: float (minimum similarity score, default 0.0)
        - return_mode: literal string ('chunks' or 'notes', default 'chunks')
        - out: literal string (variable name to store results)
        """
        target_arg = action.get('target')
        query = self._resolve_value(action.get('value'))
        mode = action.get('mode', 'semantic')
        limit = action.get('limit', 5)
        threshold = action.get('threshold', 0.0)
        return_mode = action.get('return_mode', 'chunks')
        out_var = action.get('out')
        
        if not target_arg or not query or not out_var:
            return {'status': 'failed', 'reason': 'search-within-collection requires target, value, and out'}
        
        # Target should be a Collection variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'search-within-collection target must be $variable referencing an indexed Collection'}
        
        collection_var = target_arg[1:]
        
        # Get Collection ID from bindings
        if collection_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Collection variable not bound: {collection_var}'}
        
        collection_id = self.plan_bindings[collection_var]
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return {'status': 'failed', 'reason': f'Variable {collection_var} is not a Collection'}
        
        if not self.resource_manager:
            return {'status': 'failed', 'reason': 'Resource manager not available'}
        
        # Call resource_manager directly
        success, results, error_msg = self.resource_manager.search_collection(
            agent_name=self.agent_name,
            collection_id=collection_id,
            query=query,
            mode=mode,
            limit=limit,
            threshold=threshold,
            return_mode=return_mode
        )
        
        if not success:
            return {'status': 'failed', 'reason': error_msg or 'Search failed'}
        
        # Results is already a list of dicts with 'document', 'score', 'metadata' fields
        
        # Create structured Notes for each search result (matching query-web/semantic-scholar format)
        note_ids = []
        for i, result in enumerate(results):
            document_content = result.get('document', '')
            metadata = result.get('metadata', {})
            score = result.get('score', 0.0)
            source_note_id = metadata.get('source_note_id')
            
            # Extract text preview (first paragraph, 200 chars)
            text_preview, format_type = self._extract_text_preview(document_content, max_chars=200)
            
            # Build structured result with metadata in content.metadata (not properties)
            structured_result = {
                'text': text_preview,
                'format': format_type,
                'metadata': {
                    'source_id': source_note_id if source_note_id else f'chunk_{i}',
                    'uri': source_note_id if source_note_id else f'chunk_{i}',  # URI field for consistency
                    'score': score,
                    'type': 'Note',
                    'return_mode': return_mode
                },
                'char_count': len(text_preview)
            }
            
            # Add chunk-specific metadata if in chunks mode
            if return_mode == 'chunks':
                structured_result['metadata']['chunk_index'] = metadata.get('chunk_index')
                structured_result['metadata']['chunk_total'] = metadata.get('chunk_total')
                structured_result['metadata']['is_complete_note'] = metadata.get('is_complete_note', False)
            else:
                structured_result['metadata']['is_complete_note'] = True
            
            # Remove None values from metadata
            structured_result['metadata'] = {k: v for k, v in structured_result['metadata'].items() if v is not None}
            
            # Create new Note with structured content
            note_id = self._persist_note(structured_result, f'search-within-collection', note_name=f'search_result_{i}')
            
            if note_id:
                note_ids.append(note_id)
        
        # Create Collection for results
        result_collection_id = self._create_collection(note_ids, 'search_results')
        if result_collection_id:
            self._bind_variable(out_var, result_collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Search-within-collection found {len(results)} results, created {result_collection_id} with {len(note_ids)} Notes → {display_var}")
            return {'status': 'success', 'value': result_collection_id}
        else:
            logger.error(f"Failed to create Collection for search results")
            return {'status': 'failed', 'reason': 'Failed to create result Collection'}
    
    def _execute_search_notes(self, action: Dict) -> Dict:
        """
        Global search across all Notes using embedding-based retrieval.
        
        Required: value, out
        Optional: limit, threshold
        
        Argument types:
        - value: literal string OR $variable (resolves to query text)
        - limit: int (max Notes to return, default 5)
        - threshold: float (minimum similarity score, default 0.3)
        - out: literal string (variable name to store results Collection)
        """
        query = self._resolve_value(action.get('value'))
        limit = action.get('limit', 5)
        threshold = action.get('threshold', 0.3)
        out_var = action.get('out')
        
        if not query or not out_var:
            return {'status': 'failed', 'reason': 'search-notes requires value and out'}
        
        # Search for Notes globally
        search_result = self.search_resources([query], k_notes=limit, k_collections=0, threshold=threshold)
        
        if search_result.get('status') != 'success':
            return {'status': 'failed', 'reason': search_result.get('reason', 'Search failed')}
        
        notes = search_result.get('notes', [])
        
        if not notes:
            # Return empty Collection for no results
            empty_coll_id = self._create_collection([], 'search_notes_empty')
            if empty_coll_id:
                self._bind_variable(out_var, empty_coll_id)
                return {'status': 'success', 'value': empty_coll_id}
            return {'status': 'failed', 'reason': 'No Notes found and failed to create empty Collection'}
        
        # Create structured Notes for each search result (matching query-web/semantic-scholar format)
        note_ids = []
        for note_result in notes:
            note_id = note_result.get('resource_id')
            if note_id and note_id.startswith('Note_'):
                # Get metadata for optional fields
                resource_metadata = self.get_resource_metadata(note_id)
                
                # Build structured search result
                structured_result = self._build_structured_search_result(note_id, note_result, resource_metadata)
                
                # Create new Note with structured content
                result_note_id = self._persist_note(
                    structured_result,
                    'search-notes',
                    note_name=f'search_result_{note_id}'
                )
                if result_note_id:
                    note_ids.append(result_note_id)
        
        if not note_ids:
            return {'status': 'failed', 'reason': 'No valid Note IDs found in search results'}
        
        # Create Collection containing structured result Notes
        collection_id = self._create_collection(note_ids, 'search_notes_results')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Search-notes found {len(notes)} Notes, created {collection_id} with {len(note_ids)} structured results → {display_var}")
            return {'status': 'success', 'value': collection_id}
        else:
            return {'status': 'failed', 'reason': 'Failed to create result Collection'}
    
    def _execute_search_collections(self, action: Dict) -> Dict:
        """
        Global search across all Collections using embedding-based retrieval.
        
        Required: value, out
        Optional: limit, threshold
        
        Argument types:
        - value: literal string OR $variable (resolves to query text)
        - limit: int (max Collections to return, default 3)
        - threshold: float (minimum similarity score, default 0.3)
        - out: literal string (variable name to store results Collection)
        """
        query = self._resolve_value(action.get('value'))
        limit = action.get('limit', 3)
        threshold = action.get('threshold', 0.3)
        out_var = action.get('out')
        
        if not query or not out_var:
            return {'status': 'failed', 'reason': 'search-collections requires value and out'}
        
        # Search for Collections globally
        search_result = self.search_resources([query], k_notes=0, k_collections=limit, threshold=threshold)
        
        if search_result.get('status') != 'success':
            return {'status': 'failed', 'reason': search_result.get('reason', 'Search failed')}
        
        collections = search_result.get('collections', [])
        
        if not collections:
            # Return empty Collection for no results
            empty_coll_id = self._create_collection([], 'search_collections_empty')
            if empty_coll_id:
                self._bind_variable(out_var, empty_coll_id)
                return {'status': 'success', 'value': empty_coll_id}
            return {'status': 'failed', 'reason': 'No Collections found and failed to create empty Collection'}
        
        # Create structured Notes for each search result (matching query-web/semantic-scholar format)
        note_ids = []
        for coll_result in collections:
            coll_id = coll_result.get('resource_id')
            if coll_id and coll_id.startswith('Collection_'):
                # Get metadata for optional fields
                resource_metadata = self.get_resource_metadata(coll_id)
                
                # Build structured search result
                structured_result = self._build_structured_search_result(coll_id, coll_result, resource_metadata)
                
                # Create new Note with structured content
                result_note_id = self._persist_note(
                    structured_result,
                    'search-collections',
                    note_name=f'search_result_{coll_id}'
                )
                if result_note_id:
                    note_ids.append(result_note_id)
        
        if not note_ids:
            return {'status': 'failed', 'reason': 'No valid Collection IDs found in search results'}
        
        # Create Collection containing structured result Notes
        result_collection_id = self._create_collection(note_ids, 'search_collections_results')
        if result_collection_id:
            self._bind_variable(out_var, result_collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Search-collections found {len(collections)} Collections, created {result_collection_id} with {len(note_ids)} structured results → {display_var}")
            return {'status': 'success', 'value': result_collection_id}
        else:
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
        
        # Also publish to action channel for UI display
        action_data = {
            'type': 'say',
            'action_type': 'say',
            'action_id': f'action_{self.executive_node.action_counter}',
            'timestamp': datetime.now().isoformat(),
            'text': str(value),
            'source': self.agent_name,
            'target': target,
            'is_text_only': True
        }
        self.executive_node.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        if hasattr(self.executive_node, 'last_say_text'):
            self.executive_node.last_say_text = str(value)
        
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
                logger.warning(f"display: dereferenced resource to content (len={len(str(value))})")
        
        if value is None:
            return {'status': 'failed', 'reason': 'display requires value or target'}
        
        # Always display to User
        target = 'User'
        
        # Publish display action to trigger FastAPI modal (not sense_data!)
        display_action = {
            'type': 'display',
            'character': self.agent_name,
            'target': target,
            'text': str(value),
            'value': str(value),
            'timestamp': datetime.now().isoformat()
        }
        
        # Publish to action channel so FastAPI can show modal
        self.session.put(
            f"cognitive/{self.agent_name}/action",
            json.dumps(display_action)
        )
        
        logger.warning(f"Display [{target}]: published {len(str(value))} chars to action channel for modal")
        return {'status': 'success', 'value': value}
    
    def _execute_think(self, action: Dict) -> Dict:
        """
        Internal reasoning - appended to planner context only, no Note created.
        
        Required: type, value or target
        """
        # Accept both value and target
        value = self._resolve_value(action.get('value') or action.get('target'))
        
        if value is None:
            return {'status': 'failed', 'reason': 'think requires value or target'}
        
        # Log error if out is specified - think does not create Notes
        out_var = action.get('out')
        if out_var:
            logger.error(f"Think: 'out' parameter ignored - think does not create Notes. Use generate-note or create-note instead.")
        
        logger.info(f"Think: {str(value)[:100]}")
        
        return {'status': 'success', 'value': str(value)}
    
    def _execute_ask(self, action: Dict) -> Dict:
        """
        Ask user a question and wait for response via polling.
        
        Publishes question to UI, then polls for user response.
        Zenoh callbacks run in separate threads, so while we sleep,
        _process_text_input can receive and store the response.
        
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
        
        # Publish question to UI
        action_data = {
            'type': 'ask',
            'action_type': 'ask',
            'action_id': f'action_{self.executive_node.action_counter}',
            'timestamp': datetime.now().isoformat(),
            'text': str(question_text),
            'source': self.agent_name,
            'target': target,
            'is_text_only': True
        }
        self.executive_node.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        
        display_var = self._normalize_var_for_log(out_var)
        logger.info(f"❓ Ask: '{question_text}' → awaiting response for {display_var}")
        
        # Mark that we're awaiting ask response
        self.executive_node.awaiting_ask_response = True
        
        # Poll for response (with timeout)
        import time
        timeout = 300  # 5 minutes
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            time.sleep(0.1)  # Release GIL, allow Zenoh callbacks to add to queue
            
            # Check if text input arrived in queue
            if self.executive_node.text_input_queue:
                # Process the text input directly
                sense_data = self.executive_node.text_input_queue.pop(0)
                content = sense_data['content']
                try:
                    content_data = json.loads(content)
                    response_text = content_data.get('text', '')
                except (json.JSONDecodeError, TypeError):
                    response_text = str(content)
                
                response_text = response_text.strip().strip('"').strip("'")
                
                if response_text:
                    self.executive_node.awaiting_ask_response = False
                    
                    # Don't publish response action - the User's say is already in the log
                    # Publishing both creates redundant entries
                    
                    # Create Note with response
                    response_note_id = self._persist_note(response_text, 'ask-response')
                    if response_note_id:
                        self._bind_variable(out_var, response_note_id)
                        logger.info(f"✅ Ask response received: '{response_text[:50]}...' → {out_var} = {response_note_id}")
                        return {'status': 'success', 'value': response_note_id}
                    else:
                        self._bind_variable(out_var, "Note_null")
                        return {'status': 'success', 'value': "Note_null"}
        
        # Timeout - no response received
        self.executive_node.awaiting_ask_response = False
        logger.warning(f"⏱️ Ask timeout after {timeout}s: no user response")
        return {'status': 'failed', 'reason': f'User response timeout after {timeout}s'}
    
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
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Coerced ({operation}) → Note {info_id} → {display_var}")
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
        - String: tool name or primitive name (e.g., "tool-name", "add", "remove")
        - Dict: {"tool": "name", "args": {...}} for tool with additional arguments
        
        Argument types:
        - target: $variable (Collection to map over)
        - operation: string or dict (string for tool/primitive, dict for tool with args)
        - args: dict of additional arguments (merged with operation args if dict)
        - filter_null: bool (exclude null/None results, default: true)
        - out: $variable name for result Collection
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'operation', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        operation = action.get('operation')
        out_var = action.get('out')
        filter_null = action.get('filter_null', True)
        # Extract additional args from top-level (excluding reserved fields)
        reserved_fields = {'type', 'target', 'operation', 'out', 'filter_null', 'expect', 'reason'}
        additional_args = {k: v for k, v in action.items() if k not in reserved_fields}
        
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
            'display': self._execute_display,
            'say': self._execute_say,
            'think': self._execute_think,
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
                    # For side-effect primitives (display, say, think): map item becomes 'value' or 'target'
                    elif operation in ['display', 'say', 'think']:
                        # Use 'value' field (display/say/think accept both value and target)
                        primitive_action['value'] = note_id
                    
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
                # For side-effect primitives (display, say, think): no result Note needed
                elif isinstance(operation, str) and operation in ['display', 'say', 'think']:
                    # Side-effect only - don't create result Note, but don't treat as failure
                    pass
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
                    display_var = self._normalize_var_for_log(out_var)
                    logger.info(f"Mapped {len(note_ids)} items via {operation} → {display_var} = {mutated_collection_id}")
                    return {'status': 'success', 'value': mutated_collection_id}
                else:
                    return {'status': 'failed', 'reason': f'Mutation target {mutation_target} not bound'}
            else:
                return {'status': 'failed', 'reason': f'{operation} requires target in args'}
        elif isinstance(operation, str) and operation in ['display', 'say', 'think']:
            # Side-effect primitives - create empty Collection (side effects executed, no result Notes)
            operation_str = operation if isinstance(operation, str) else 'operation'
            collection_id = self._create_collection([], f'map_{operation_str}')
            if collection_id:
                self._bind_variable(out_var, collection_id)
                display_var = self._normalize_var_for_log(out_var)
                logger.info(f"Mapped {len(note_ids)} items via {operation} (side effects executed), created {collection_id} → {display_var}")
                return {'status': 'success', 'value': collection_id}
            else:
                logger.error(f"Failed to create Collection for map results")
                return {'status': 'failed', 'reason': 'Failed to create result Collection'}
        else:
            # Tool operation - create result Collection
            operation_str = operation if isinstance(operation, str) else 'operation'
            collection_id = self._create_collection(result_note_ids, f'map_{operation_str}')
            if collection_id:
                self._bind_variable(out_var, collection_id)
                display_var = self._normalize_var_for_log(out_var)
                logger.info(f"Mapped {len(note_ids)} → {len(result_note_ids)} Notes, created {collection_id} → {display_var}")
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
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Flattened {len(note_ids)} Notes → Note {info_id} → {display_var}")
            return {'status': 'success', 'value': info_id}
        else:
            logger.error(f"Failed to persist flatten result")
            return {'status': 'failed', 'reason': 'Failed to persist result'}
    
    def _execute_add(self, action: Dict) -> Dict:
        """
        Add a Note or Collection to an existing Collection (mutates Collection).
        
        Required: target, value, out
        
        Argument types:
        - target: $variable (Collection to add to)
        - value: $variable or literal (Note or Collection to add)
        - out: variable name (should be same as target to maintain reference)
        
        If value is a Collection, all its items are added with ID-based deduplication.
        This is a controlled mutation for practical use cases like accumulating research papers.
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
        
        # Resolve value - could be Note, Collection, or literal
        note_ids_to_add = []
        
        if isinstance(value_arg, str) and value_arg.startswith('$'):
            # Variable - check if Note or Collection
            var_name = value_arg[1:]
            if var_name not in self.plan_bindings:
                return {'status': 'failed', 'reason': f'Variable not bound: {var_name}'}
            
            value_id = self.plan_bindings[var_name]
            
            if isinstance(value_id, str) and value_id.startswith('Collection_'):
                # Collection - get all its Note IDs
                note_ids = self._dereference_collection(var_name)
                if not isinstance(note_ids, list):
                    return {'status': 'failed', 'reason': f'Failed to dereference Collection {value_arg}'}
                note_ids_to_add = note_ids
            elif isinstance(value_id, str) and value_id.startswith('Note_'):
                # Single Note
                note_ids_to_add = [value_id]
            else:
                return {'status': 'failed', 'reason': f'Variable {var_name} is not a Note or Collection'}
        elif isinstance(value_arg, str) and value_arg.startswith('Note_'):
            # Direct Note ID
            note_ids_to_add = [value_arg]
        elif isinstance(value_arg, str) and value_arg.startswith('Collection_'):
            # Direct Collection ID (edge case)
            return {'status': 'failed', 'reason': 'Use $variable syntax for Collection references'}
        else:
            # Literal - create Note for it
            note_id = self._persist_note(value_arg, 'add_item')
            if not note_id:
                return {'status': 'failed', 'reason': 'Failed to persist value as Note'}
            note_ids_to_add = [note_id]
        
        if not self.resource_manager:
            return {'status': 'failed', 'reason': 'Resource manager not available'}
        
        # Get current Collection content for deduplication
        target_collection = self.resource_manager.get_resource(collection_id)
        if not target_collection:
            return {'status': 'failed', 'reason': f'Collection {collection_id} not found'}
        
        existing_ids = set(target_collection['properties'].get('content', []))
        
        # Add each Note with ID-based deduplication
        added_count = 0
        skipped_count = 0
        
        for note_id in note_ids_to_add:
            if note_id in existing_ids:
                skipped_count += 1
                continue
            
            success, item_count, error_msg = self.resource_manager.add_to_collection(
                collection_id, note_id, self.agent_name, 'add', None
            )
            
            if success:
                existing_ids.add(note_id)
                added_count += 1
            else:
                logger.warning(f'Failed to add {note_id} to {collection_id}: {error_msg}')
        
        # Bind to out variable (usually same as target)
        self._bind_variable(out_var, collection_id)
        display_var = self._normalize_var_for_log(out_var)
        
        if len(note_ids_to_add) == 1:
            # Single item - use original log format
            if added_count == 1:
                logger.info(f"Added {note_ids_to_add[0]} to {collection_id} → {display_var}")
            else:
                logger.info(f"Skipped duplicate {note_ids_to_add[0]} in {collection_id} → {display_var}")
        else:
            # Multiple items - show summary
            logger.info(f"Added {added_count} items to {collection_id} (skipped {skipped_count} duplicates) → {display_var}")
        
        return {'status': 'success', 'value': collection_id}
    
    def _execute_split(self, action: Dict) -> Dict:
        """
        Split a Note into a Collection of Notes.
        
        Handles four cases:
        1. Simple JSON array: Directly uses the array (e.g., [1, 2, 3])
        2. JSON object with array field: Extracts array from specified field (default 'results')
        3. JSONL format: Multiple JSON objects, one per line (newlines required between objects)
        4. Plain text: Splits on newlines and filters empty lines
        
        Required: target, out
        Optional: field (default: 'results') - only used for JSON object case
        
        Argument types:
        - target: $variable (Note containing JSON array, JSON object with array field, JSONL, or plain text)
        - field: literal string (name of array field, default 'results') - only used for JSON object case
        - out: variable name for resulting Collection
        
        Examples:
        - Split simple JSON array: {"type":"split","target":"$json_note","out":"$items"} (where note contains [1,2,3])
        - Split structured data: {"type":"split","target":"$data_note","out":"$items"} (where note contains {"results": [...]})
        - Split JSONL: {"type":"split","target":"$jsonl_note","out":"$items"} (where note contains {"key":"val1"}\n{"key":"val2"})
        - Split text lines: {"type":"split","target":"$text_note","out":"$lines"}
        
        NOTE: query-web and semantic-scholar return Collections directly - NO split needed.
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        field_name = action.get('field', 'results')
        out_var = action.get('out')
        
        # Target must be Note variable
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'split target must be $variable'}
        
        note_var = target_arg[1:]
        
        # Get Note ID from bindings
        if note_var not in self.plan_bindings:
            return {'status': 'failed', 'reason': f'Note variable not bound: {note_var}'}
        
        note_id = self.plan_bindings[note_var]
        
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            return {'status': 'failed', 'reason': f'Variable {note_var} is not a Note'}
        
        # Check for null Note
        if note_id == 'Note_null':
            return {'status': 'failed', 'reason': 'Cannot split null Note'}
        
        # Get Note content
        content = self._get_content(note_id)
        if content is None:
            return {'status': 'failed', 'reason': f'Note {note_id} has no content'}
        
        # Check for null content indicator
        if isinstance(content, str) and content.strip().lower() in ['note-null', 'null']:
            return {'status': 'failed', 'reason': 'Cannot split null content'}
        
        array_data = None
        is_json = False
        
        # If content is already a Python list, use it directly
        if isinstance(content, list):
            array_data = content
        # Try JSON extraction (prioritize structured data)
        # Use robust extraction that handles code fences, preambles, trailing text
        elif isinstance(content, str):
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
            # First check if content_obj is itself a simple JSON array
            if isinstance(content_obj, list):
                array_data = content_obj
            # Otherwise, check for the specified field in the object
            elif isinstance(content_obj, dict):
                if field_name in content_obj:
                    array_data = content_obj[field_name]
                    if not isinstance(array_data, list):
                        return {'status': 'failed', 'reason': f'Field "{field_name}" exists but is not an array'}
                else:
                    return {'status': 'failed', 'reason': f'JSON content missing "{field_name}" array field'}
            else:
                return {'status': 'failed', 'reason': f'JSON content must be an array or object with "{field_name}" field'}
        
        # If NOT single JSON and array_data not yet set, try JSONL format (multiple JSON objects separated by newlines)
        elif not is_json and array_data is None:
            if isinstance(content, str):
                # Try to detect and parse JSONL format
                jsonl_array = self._detect_and_parse_jsonl(content)
                if jsonl_array is not None:
                    array_data = jsonl_array
                else:
                    # Fall back to plain text line splitting
                    lines = [line.strip() for line in content.split('\n')]
                    array_data = [line for line in lines if line]  # Filter empty lines
            else:
                return {'status': 'failed', 'reason': f'Note content must be a JSON array (e.g., ["item1", "item2"]), JSON object with "{field_name}" array field, JSONL, or plain text (got {type(content).__name__})'}
        
        if not array_data:
            return {'status': 'failed', 'reason': 'No items to split (empty array or no non-empty lines)'}
        
        # Create Note for each array element/line
        note_ids = []
        for i, item in enumerate(array_data):
            item_note_id = self._persist_note(item, f'split_item_{i}')
            if item_note_id:
                note_ids.append(item_note_id)
        
        # Create Collection from Notes
        collection_id = self._create_collection(note_ids, 'split_collection')
        if not collection_id:
            return {'status': 'failed', 'reason': 'Failed to create Collection'}
        
        # Bind to out variable
        self._bind_variable(out_var, collection_id)
        # Build source description
        if is_json and content_obj:
            if isinstance(content_obj, list):
                source_desc = f"{note_id} (JSON array)"
            elif isinstance(content_obj, dict) and field_name in content_obj:
                source_desc = f"{note_id}.{field_name}"
            else:
                source_desc = f"{note_id} (JSON)"
        elif isinstance(content, str) and self._detect_and_parse_jsonl(content) is not None:
            source_desc = f"{note_id} (JSONL)"
        else:
            source_desc = f"{note_id} (lines)"
        display_var = self._normalize_var_for_log(out_var)
        logger.info(f"Expanded {source_desc} ({len(note_ids)} items) → {display_var}")
        
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
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Size of {collection_var}: {size} → {display_var}")
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
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Union {len(target_ids)} + {len(value_ids)} → {len(union_ids)} → {display_var}")
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
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Intersection {len(target_ids)} ∩ {len(value_ids)} → {len(intersection_ids)} → {display_var}")
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
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Difference {len(target_ids)} - {len(value_ids)} → {len(difference_ids)} → {display_var}")
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
        
        # Update Collection using resource manager
        if not self.resource_manager:
            return {'status': 'failed', 'reason': 'Resource manager not available'}
        
        success, item_count, error_msg = self.resource_manager.add_to_collection(
            collection_id, None, self.agent_name, 'update', note_ids
        )
        
        if success:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Removed {note_id} from {collection_id} (now {len(note_ids)} items) → {display_var}")
            return {'status': 'success', 'value': collection_id}
        else:
            return {'status': 'failed', 'reason': error_msg or 'Remove failed'}
    
    def _execute_project(self, action: Dict) -> Dict:
        """
        Extract specific fields from each Note in a Collection (SQL SELECT fields).
        
        Required: target, out
        Optional: fields (list of field paths like "title" or "metadata.year")
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        out_var = action.get('out')
        fields = action.get('fields', action.get('args', {}).get('fields'))
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'project target must be $variable'}
        
        if not fields:
            return {'status': 'failed', 'reason': 'project requires fields list'}
        
        if not isinstance(fields, list):
            return {'status': 'failed', 'reason': 'fields must be a list'}
        
        collection_var = target_arg[1:]
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'project target must be a Collection'}
        
        # Project each Note
        projected_ids = []
        for note_id in note_ids:
            content = self._get_content(note_id)
            if content is None:
                continue
            
            # Extract fields from JSON content
            if not isinstance(content, dict):
                logger.warning(f"Cannot project from non-dict Note {note_id}")
                continue
            
            projected = {}
            all_fields_found = True
            for field in fields:
                # Handle nested fields like "metadata.year"
                value = content
                parts = field.split('.')
                for part in parts:
                    if isinstance(value, dict) and part in value:
                        value = value[part]
                    else:
                        value = None
                        break
                
                # Check if field is missing or null
                if value is None:
                    all_fields_found = False
                    break
                
                # Preserve nested structure in projected Note
                if len(parts) == 1:
                    projected[parts[0]] = value
                else:
                    # Rebuild nested structure
                    current = projected
                    for i, part in enumerate(parts[:-1]):
                        if part not in current:
                            current[part] = {}
                        current = current[part]
                    current[parts[-1]] = value
            
            # Only create Note if all fields were found and not null
            if all_fields_found and projected:
                projected_id = self._persist_note(projected, 'project_result')
                if projected_id:
                    projected_ids.append(projected_id)
        
        # Create result Collection
        collection_id = self._create_collection(projected_ids, f'project_{collection_var}')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Projected {len(projected_ids)} items with fields {fields} → {display_var}")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create projected Collection'}
    
    def _execute_head(self, action: Dict) -> Dict:
        """
        Take first N items from Collection.
        
        Required: target, out
        Optional: count (default 1)
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        count = action.get('count', action.get('args', {}).get('count', 1))
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'head target must be $variable'}
        
        if not isinstance(count, int) or count < 1:
            return {'status': 'failed', 'reason': 'head count must be positive integer'}
        
        collection_var = target_arg[1:]
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'head target must be a Collection'}
        
        # Take first N items
        head_ids = note_ids[:count]
        
        # Create new Collection
        collection_id = self._create_collection(head_ids, f'head_{count}')
        if not collection_id:
            return {'status': 'failed', 'reason': 'Failed to create Collection'}
        
        self._bind_variable(out_var, collection_id)
        display_var = self._normalize_var_for_log(out_var)
        logger.info(f"Took first {len(head_ids)}/{len(note_ids)} items → {display_var}")
        
        return {
            'status': 'success',
            'out': out_var,
            'result': collection_id,
            'text': f'head: took first {len(head_ids)} items from {len(note_ids)}'
        }

    def _execute_pluck(self, action: Dict) -> Dict:
        """
        Extract a single field value from each Note in a Collection.
        Returns Collection of Notes, each containing the extracted scalar value.
        
        Required: target, field, out
        Example: Extract URLs from structured Notes for use with tools
        """
        error = self._validate_required_fields(action, 'target', 'field', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        field = action.get('field', action.get('args', {}).get('field'))
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'pluck target must be $variable'}
        
        if not field:
            return {'status': 'failed', 'reason': 'pluck requires field parameter'}
        
        collection_var = target_arg[1:]
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'pluck target must be a Collection'}
        
        # Extract field value from each Note
        plucked_ids = []
        for note_id in note_ids:
            content = self._get_content(note_id)
            if not isinstance(content, dict):
                logger.warning(f"Cannot pluck from non-dict Note {note_id}")
                continue
            
            # Navigate nested fields
            value = content
            for part in field.split('.'):
                if isinstance(value, dict) and part in value:
                    value = value[part]
                else:
                    value = None
                    break
            
            if value is not None:
                # Create Note with scalar value
                plucked_id = self._persist_note(value, 'pluck_result')
                if plucked_id:
                    plucked_ids.append(plucked_id)
        
        # Create result Collection
        collection_id = self._create_collection(plucked_ids, f'pluck_{collection_var}')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Plucked '{field}' from {len(plucked_ids)} items → {display_var}")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create plucked Collection'}
    
    def _execute_filter_structured(self, action: Dict) -> Dict:
        """
        Filter Collection based on JSON field predicates (SQL WHERE - deterministic).
        
        Required: target, where, out
        Example where: "year > 2020" or "citations >= 100 AND year < 2025"
        """
        error = self._validate_required_fields(action, 'target', 'where', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        where_clause = action.get('where', action.get('args', {}).get('where'))
        out_var = action.get('out')
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'filter-structured target must be $variable'}
        
        if not where_clause:
            return {'status': 'failed', 'reason': 'filter-structured requires where clause'}
        
        collection_var = target_arg[1:]
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'filter-structured target must be a Collection'}
        
        # Parse where clause (simple expression evaluator)
        def eval_predicate(content: dict, predicate: str) -> bool:
            """Safely evaluate predicate on JSON content."""
            # Replace field references with dict access
            # Support: field > value, field < value, field == value, field >= value, field <= value
            # Support AND, OR operators
            
            tokens = predicate.replace('>=', ' >= ').replace('<=', ' <= ').replace('==', ' == ').replace('>', ' > ').replace('<', ' < ').replace('!=', ' != ').split()
            
            # Simple recursive evaluation
            def evaluate_simple(field, op, value_str):
                # Navigate nested fields
                val = content
                for part in field.split('.'):
                    if isinstance(val, dict) and part in val:
                        val = val[part]
                    else:
                        return False
                
                # Parse value
                try:
                    if value_str.startswith('"') and value_str.endswith('"'):
                        value = value_str[1:-1]
                    elif value_str in ['true', 'True']:
                        value = True
                    elif value_str in ['false', 'False']:
                        value = False
                    else:
                        value = float(value_str) if '.' in value_str else int(value_str)
                except:
                    value = value_str
                
                # Try to convert val to match value's type for comparison
                if isinstance(value, (int, float)) and isinstance(val, str):
                    try:
                        val = float(val) if '.' in val else int(val)
                    except:
                        pass
                
                # Evaluate
                if op == '>': return val > value
                elif op == '<': return val < value
                elif op == '>=': return val >= value
                elif op == '<=': return val <= value
                elif op == '==': return val == value
                elif op == '!=': return val != value
                return False
            
            # Handle AND/OR (simple left-to-right evaluation)
            if ' AND ' in predicate:
                parts = predicate.split(' AND ')
                return all(eval_predicate(content, p.strip()) for p in parts)
            elif ' OR ' in predicate:
                parts = predicate.split(' OR ')
                return any(eval_predicate(content, p.strip()) for p in parts)
            
            # Simple comparison
            if len(tokens) >= 3:
                return evaluate_simple(tokens[0], tokens[1], tokens[2])
            return False
        
        # Filter each Note
        filtered_ids = []
        for note_id in note_ids:
            content = self._get_content(note_id)
            if not isinstance(content, dict):
                continue
            
            try:
                if eval_predicate(content, where_clause):
                    filtered_ids.append(note_id)
            except Exception as e:
                logger.warning(f"Predicate evaluation failed for {note_id}: {e}")
                continue
        
        # Create result Collection
        collection_id = self._create_collection(filtered_ids, f'filtered_{collection_var}')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Filtered {len(filtered_ids)}/{len(note_ids)} items with '{where_clause}' → {display_var}")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create filtered Collection'}
    
    def _execute_sort(self, action: Dict) -> Dict:
        """
        Sort Collection by field value (SQL ORDER BY).
        
        Required: target, by, out
        Optional: order ("asc" or "desc", default "asc")
        """
        error = self._validate_required_fields(action, 'target', 'by', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        target_arg = action.get('target')
        sort_field = action.get('by', action.get('args', {}).get('by'))
        out_var = action.get('out')
        order = action.get('order', action.get('args', {}).get('order', 'asc'))
        
        if not isinstance(target_arg, str) or not target_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'sort target must be $variable'}
        
        if not sort_field:
            return {'status': 'failed', 'reason': 'sort requires by field'}
        
        if order not in ['asc', 'desc']:
            return {'status': 'failed', 'reason': 'sort order must be "asc" or "desc"'}
        
        collection_var = target_arg[1:]
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return {'status': 'failed', 'reason': 'sort target must be a Collection'}
        
        # Extract sort keys
        items_with_keys = []
        for note_id in note_ids:
            content = self._get_content(note_id)
            if not isinstance(content, dict):
                continue
            
            # Navigate to sort field
            value = content
            for part in sort_field.split('.'):
                if isinstance(value, dict) and part in value:
                    value = value[part]
                else:
                    value = None
                    break
            
            if value is not None:
                items_with_keys.append((note_id, value))
        
        # Sort
        try:
            sorted_items = sorted(items_with_keys, key=lambda x: x[1], reverse=(order == 'desc'))
            sorted_ids = [note_id for note_id, _ in sorted_items]
        except Exception as e:
            return {'status': 'failed', 'reason': f'Sort failed: {e}'}
        
        # Create result Collection
        collection_id = self._create_collection(sorted_ids, f'sorted_{collection_var}')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Sorted {len(sorted_ids)} items by {sort_field} ({order}) → {display_var}")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create sorted Collection'}
    
    def _execute_join(self, action: Dict) -> Dict:
        """
        Join two Collections on a key field (SQL JOIN).
        
        Required: target (left), value (right), out
        Args: on (join key), type (inner/left/right/outer), left_key, right_key
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return {'status': 'failed', 'reason': error}
        
        left_arg = action.get('target')
        right_arg = action.get('value')
        out_var = action.get('out')
        
        join_key = action.get('on')
        join_type = action.get('join_type', 'inner')
        left_key = action.get('left_key', join_key)
        right_key = action.get('right_key', join_key)
        
        if not isinstance(left_arg, str) or not left_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'join target must be $variable'}
        
        if not isinstance(right_arg, str) or not right_arg.startswith('$'):
            return {'status': 'failed', 'reason': 'join value must be $variable'}
        
        if not join_key and not (left_key and right_key):
            return {'status': 'failed', 'reason': 'join requires on key or left_key/right_key'}
        
        if join_type not in ['inner', 'left', 'right', 'outer']:
            return {'status': 'failed', 'reason': 'join type must be inner/left/right/outer'}
        
        left_var = left_arg[1:]
        right_var = right_arg[1:]
        
        left_ids = self._dereference_collection(left_var)
        right_ids = self._dereference_collection(right_var)
        
        if not isinstance(left_ids, list) or not isinstance(right_ids, list):
            return {'status': 'failed', 'reason': 'join requires two Collections'}
        
        # Build right index
        right_index = {}
        for note_id in right_ids:
            content = self._get_content(note_id)
            if not isinstance(content, dict):
                continue
            
            # Navigate to right key
            key_value = content
            for part in right_key.split('.'):
                if isinstance(key_value, dict) and part in key_value:
                    key_value = key_value[part]
                else:
                    key_value = None
                    break
            
            if key_value is not None:
                if key_value not in right_index:
                    right_index[key_value] = []
                right_index[key_value].append(content)
        
        # Join
        joined_ids = []
        matched_left = set()
        
        for left_note_id in left_ids:
            left_content = self._get_content(left_note_id)
            if not isinstance(left_content, dict):
                continue
            
            # Navigate to left key
            key_value = left_content
            for part in left_key.split('.'):
                if isinstance(key_value, dict) and part in key_value:
                    key_value = key_value[part]
                else:
                    key_value = None
                    break
            
            if key_value is None:
                if join_type in ['left', 'outer']:
                    joined_id = self._persist_note(left_content, 'join_result')
                    if joined_id:
                        joined_ids.append(joined_id)
                continue
            
            if key_value in right_index:
                matched_left.add(key_value)
                for right_content in right_index[key_value]:
                    merged = {**left_content, **right_content}
                    joined_id = self._persist_note(merged, 'join_result')
                    if joined_id:
                        joined_ids.append(joined_id)
            else:
                if join_type in ['left', 'outer']:
                    joined_id = self._persist_note(left_content, 'join_result')
                    if joined_id:
                        joined_ids.append(joined_id)
        
        # Handle right/outer join - add unmatched right items
        if join_type in ['right', 'outer']:
            for key_value, right_contents in right_index.items():
                if key_value not in matched_left:
                    for right_content in right_contents:
                        joined_id = self._persist_note(right_content, 'join_result')
                        if joined_id:
                            joined_ids.append(joined_id)
        
        # Create result Collection
        collection_id = self._create_collection(joined_ids, f'join_{left_var}_{right_var}')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Joined {len(left_ids)} + {len(right_ids)} items on {left_key}/{right_key} ({join_type}) → {display_var} ({len(joined_ids)} results)")
            return {'status': 'success', 'value': collection_id}
        
        return {'status': 'failed', 'reason': 'Failed to create joined Collection'}
    
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
        Get Collection Note IDs by querying resource manager.
        
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
            # Fetch Collection from resource manager
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
        Handles both JSON objects ({...}) and JSON arrays ([...]).
        Based on repair_json logic from llm_api.py.
        
        Returns parsed dict/list or None if no valid JSON found.
        """
        if not isinstance(text, str):
            return None
        
        # Remove code fences first (common in LLM responses)
        cleaned = text.replace("```json", "").replace("```", "").strip()
        
        # Determine if we're looking for an object or array
        is_array = cleaned.startswith('[') or ('[' in cleaned and cleaned.find('[') < cleaned.find('{'))
        is_object = cleaned.startswith('{') or ('{' in cleaned and cleaned.find('{') < cleaned.find('['))
        
        # Find JSON boundaries
        if is_array:
            # Find first [ to last ]
            if not cleaned.startswith('[') and '[' in cleaned:
                start = cleaned.find('[')
                end = cleaned.rfind(']')
                if start >= 0 and end > start:
                    cleaned = cleaned[start:end+1]
        elif is_object:
            # Find first { to last }
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
        
        # Find first complete JSON structure by bracket/brace counting
        if is_array:
            # Count brackets for arrays
            bracket_count = 0
            json_end = 0
            for i, char in enumerate(cleaned):
                if char == '[':
                    bracket_count += 1
                elif char == ']':
                    bracket_count -= 1
                    if bracket_count == 0:
                        json_end = i + 1
                        break
            if json_end > 0:
                cleaned = cleaned[:json_end]
        elif is_object:
            # Count braces for objects
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
    
    def _detect_and_parse_jsonl(self, text: str) -> Optional[List[Dict]]:
        """
        Detect and parse JSONL format (multiple JSON objects separated by newlines).
        
        Handles surrounding text (code fences, preambles, trailing text) similar to
        _extract_json_from_text. Requires newlines between JSON objects (standard JSONL).
        
        Returns:
            List of parsed JSON objects if all non-empty lines are valid JSON objects, None otherwise
        """
        if not isinstance(text, str):
            return None
        
        # Remove code fences first (common in LLM responses)
        cleaned = text.replace("```json", "").replace("```", "").strip()
        
        # Split into lines
        lines = cleaned.split('\n')
        
        # Try to parse each non-empty line as JSON
        json_objects = []
        for line in lines:
            stripped = line.strip()
            if not stripped:  # Skip empty lines
                continue
            
            # Try to parse as JSON
            try:
                parsed = json.loads(stripped)
                # Only accept JSON objects (not arrays or primitives)
                if isinstance(parsed, dict):
                    json_objects.append(parsed)
                else:
                    # Found a non-object JSON value - not JSONL format
                    return None
            except (json.JSONDecodeError, ValueError):
                # Line doesn't parse as JSON - not JSONL format
                return None
        
        # Only return array if we found at least one JSON object
        if json_objects:
            return json_objects
        else:
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
            # Could optionally validate existence via resource query here
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
            Resolved content value (fetches from resource manager if resource ID)
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
            
            # If it's a resource ID, fetch content from resource manager
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
                
                # If it's a resource ID, fetch content from resource manager
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
        
        # Track last out resource for plan_result
        if self.executive_node:
            self.executive_node.last_out_resource_id = resource_id
    
    def _normalize_var_for_log(self, var_name: str) -> str:
        """
        Normalize variable name for logging (ensure single $ prefix).
        
        Args:
            var_name: Variable name that may or may not have $ prefix
            
        Returns:
            Variable name with exactly one $ prefix
        """
        if not var_name:
            return var_name
        if var_name.startswith('$'):
            return var_name
        return f'${var_name}'
    
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
    
    def execute_plan_sync(self, plan: Dict, max_steps: int = 1000, max_depth: int = 10) -> Dict:
        """
        Execute a plan synchronously. Uses self.plan_bindings directly.
        
        Args:
            plan: Plan dict with 'plan' key containing list of actions
            max_steps: Maximum steps to execute (prevents infinite loops)
            max_depth: Maximum recursion depth for nested plans
            
        Returns:
            Dict with 'status', 'reason', 'executed_steps', 'bindings'
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
                # Skip say/ask - they self-publish with proper text field
                if self.executive_node and stype not in ('say', 'ask'):
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
                # Skip say/ask - they self-publish with proper text field
                if self.executive_node and stype not in ('say', 'ask'):
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
            'search-within-collection': {
                'type': 'search-within-collection',
                'target': '$test_data',
                'value': 'test query',
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

