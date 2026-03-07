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
from infospace_resource_manager import InfospaceResourceManager

# Try to import SGLang for runtime support
try:
    import sglang as sgl
    from sglang import function, user, assistant, gen
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False

logger = logging.getLogger(__name__)

# Primitives executed by InfospaceExecutor (used by WorldModel/ToolModel to treat as available)
INFOSPACE_PRIMITIVES = frozenset({
    'apply', 'create-note', 'create-collection', 'persist', 'load', 'index', 'organize',
    'search-within-collection', 'discover-notes', 'discover-collections', 'say',
    'think', 'ask', 'bind', 'coerce', 'map', 'flatten', 'add', 'split', 'size', 'union',
    'intersection', 'difference', 'remove', 'project', 'pluck', 'head',
    'filter-structured', 'sort', 'join', 'create-relation', 'find-relations', 'related',
    'get-metadata', 'set-metadata',
})

ALT_LLM_TOOLS = frozenset({'synthesize', 'extract', 'extract-struct', 'extract-references', 'refine'})


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
    
    def __init__(self, agent_name: str, session, world_name: str, llm_client, available_tools: Dict[str, Dict], executive_node=None, resource_manager=None):
        """
        Initialize infospace executor.
        
        Args:
            agent_name: Name of the agent
            session: Zenoh session for communication
            world_name: Name of the world (for tool loading and resource management)
            llm_client: LLM client for tool execution
            available_tools: Dict of tool_name -> metadata (from tool_loader)
            executive_node: Reference to ZenohExecutiveNode (for ask action)
            resource_manager: InfospaceResourceManager instance for direct resource access
        """
        self.agent_name = agent_name
        self.session = session
        self.world_name = world_name
        self.llm_client = llm_client
        self.available_tools = available_tools
        self.executive_node = executive_node
        self.resource_manager: InfospaceResourceManager = resource_manager
        self.character_context: str = ""
        
        # Plan-local state (ephemeral, cleared each plan)
        # Binding stack: list of dicts, each dict is a scope level
        # Top of stack (last element) is current scope, bottom (first) is outer scope
        self.plan_bindings = [{}]  # Stack of scopes: [$var_name -> resource_id]
        
        # Agent state (persistent across plans)
        self.agent_position = None
        
        # World state (initialized from world_config.state JSON string)
        self.world_state = {}
        self._initialize_world_state()
        
        # Suspension state for sync execution
        self._sync_suspension_state = None
        
        # SGLang runtime (set by IncrementalPlanner if available)
        self.runtime = None
        
        # vLLM config (set by executive_node if vLLM is used)
        self.vllm_model = None
        self.vllm_url = None
        self.reasoning_config = None  # e.g. {"effort": "low"}, set from llm_config
        
        # OpenAI API config (set by executive_node if OpenAI is used)
        self.openai_model = None
        self.openai_api_key = None

        # OpenRouter config (set by executive_node if OpenRouter is used)
        self.openrouter_model = None
        self.openrouter_api_key = None
        self.openrouter_provider = None

        # Anthropic API config (set by executive_node if Anthropic is used)
        self.anthropic_model = None
        self.anthropic_api_key = None

        # Alt LLM config (for synthesize, extract, extract-struct, extract-references, refine)
        self.alt_openai_model = None
        self.alt_openai_api_key = None
        self.alt_openrouter_model = None
        self.alt_openrouter_api_key = None
        self.alt_openrouter_provider = None
        self.alt_vllm_model = None
        self.alt_vllm_url = None
        self.alt_anthropic_model = None
        self.alt_anthropic_api_key = None

        # Global interrupt flag (can be set by UI via executive_node control channel)
        self.interrupt_requested: bool = False
        
        # Caches for Python tool execution (performance optimization)
        self._tool_path_cache: Dict[str, Path] = {}  # python_file -> Path object
        self._tool_spec_cache: Dict[str, Any] = {}  # (tool_name, python_file) -> importlib spec
        self._tool_base_args_cache: Optional[Dict[str, Any]] = None  # Base resolved_args template
        self._tool_base_args_cache_key: Optional[tuple] = None  # Cache key for invalidation
        
        logger.info(f"InfospaceExecutor initialized for {agent_name} with {len(available_tools)} tools")
        
        # Load world-specific documentation if configured
        self._load_world_documentation()
        
        # Run init tool if available (after full tool catalog is loaded)
        self._run_init_tool()
    
    def _initialize_world_state(self):
        """
        Initialize world_state from world_config.state JSON string.
        
        Parses world_config.state (if present) and stores as self.world_state dict.
        If state is missing or invalid JSON, initializes to empty dict.
        """
        if not self.executive_node:
            return
        
        world_config = self.executive_node.character_config.get('world_config', {})
        state_json = world_config.get('state')
        
        if not state_json:
            logger.debug("No world_config.state found, initializing empty world_state")
            self.world_state = {}
            return
        
        if isinstance(state_json, str):
            try:
                self.world_state = json.loads(state_json)
                logger.info(f"Initialized world_state from world_config.state: {list(self.world_state.keys())}")
            except json.JSONDecodeError as e:
                logger.warning(f"Failed to parse world_config.state JSON: {e}, initializing empty world_state")
                self.world_state = {}
        elif isinstance(state_json, dict):
            # Already a dict (some configs might provide it parsed)
            self.world_state = state_json.copy()
            logger.info(f"Initialized world_state from world_config.state dict: {list(self.world_state.keys())}")
        else:
            logger.warning(f"world_config.state has unexpected type {type(state_json).__name__}, initializing empty world_state")
            self.world_state = {}
    
    def get_world_state(self, field_name: str) -> Any:
        """
        Get a top-level field from world_state.
        
        Args:
            field_name: Name of the top-level field to retrieve (e.g., 'nav')
            
        Returns:
            Value of the field, or None if field doesn't exist
        """
        return self.world_state.get(field_name)
    
    def set_world_state(self, field_name: str, value: Any) -> None:
        """
        Set a top-level field in world_state.
        
        Args:
            field_name: Name of the top-level field to set (e.g., 'nav')
            value: Value to set (can be any JSON-serializable type)
        """
        self.world_state[field_name] = value
        logger.debug(f"Set world_state.{field_name} = {value}")
        
        # Publish world_state update to Zenoh for UI updates
        if self.session:
            try:
                world_state_data = {
                    'world_state': self.world_state,
                    'updated_field': field_name,
                    'timestamp': datetime.now().isoformat()
                }
                self.session.put(
                    f"cognitive/{self.agent_name}/world_state",
                    json.dumps(world_state_data)
                )
            except Exception as e:
                logger.warning(f"Failed to publish world_state update: {e}")
    
    def _load_world_documentation(self):
        """
        Load world-specific documentation from scenarios/<world_name>/ directory.
        
        If world_config.world_name is set and scenarios/<world_name>/ exists,
        creates a Collection named <world_name> containing Notes for each .md file.
        Recreates the collection from scratch on each startup (not persistent).
        Always creates the collection (even if empty) and indexes it for search.
        """
        if not self.executive_node:
            return
        
        world_config = self.executive_node.character_config.get('world_config', {})
        world_name = world_config.get('world_name')
        
        if not world_name:
            return
        
        # Resolve absolute path to scenarios/<world_name>/
        current_dir = Path(__file__).parent
        project_root = current_dir.parent  # src/ -> project root
        world_dir = project_root / "scenarios" / world_name
        
        if not world_dir.exists() or not world_dir.is_dir():
            logger.warning(f"World directory not found: {world_dir}")
            # Still create empty collection
            note_ids = []
        else:
            # Delete existing collection if it exists (recreate from scratch)
            if self.resource_manager:
                existing_id = self.resource_manager._resolve_resource_id(world_name)
                if existing_id and existing_id.startswith('Collection_'):
                    success, error_msg = self.delete_resource_and_unbind(existing_id)
                    if success:
                        logger.info(f"Deleted existing world documentation Collection '{world_name}' for recreation")
                    else:
                        logger.warning(f"Failed to delete existing Collection '{world_name}': {error_msg}")
            
            # Find all .md files
            md_files = sorted(world_dir.glob('*.md'))
            
            # Create Notes for each .md file
            note_ids = []
            for md_file in md_files:
                try:
                    content = md_file.read_text(encoding='utf-8')
                    # Use filename (without .md) as note name
                    note_name = md_file.stem
                    note_id = self._persist_note(content, 'world-documentation', note_name=note_name)
                    if note_id:
                        note_ids.append(note_id)
                        logger.info(f"📄 Loaded world documentation: {md_file.name} → {note_id}")
                    else:
                        logger.warning(f"Failed to create Note for {md_file.name}")
                except Exception as e:
                    logger.warning(f"Failed to load world doc {md_file.name}: {e}")
        
        # Always create Collection named <world_name> (even if empty)
        collection_id = self._create_collection(
            note_ids, 
            'world-documentation-load',
            collection_name=world_name,
            properties={'world_name': world_name, 'source': 'scenarios'}
        )
        
        if collection_id:
            # Bind to variable for easy reference
            self._bind_variable(world_name, collection_id)
            if note_ids:
                logger.info(f"📚 Created world documentation Collection '{world_name}' with {len(note_ids)} Notes")
            else:
                logger.info(f"📚 Created empty world documentation Collection '{world_name}' (no .md files found)")
            
            # Index the Collection for semantic search (required for search-within-collection)
            if self.resource_manager:
                success, indexed_count, error_msg = self.resource_manager.index_collection(
                    agent_name=self.agent_name,
                    collection_id=collection_id,
                    index_type='semantic',
                    fields={}
                )
                if success:
                    logger.info(f"🔍 Indexed world documentation Collection '{world_name}' ({indexed_count} chunks)")
                else:
                    logger.warning(f"Failed to index world documentation Collection '{world_name}': {error_msg}")
        else:
            logger.error(f"Failed to create world documentation Collection '{world_name}'")
    
    def _run_init_tool(self):
        """
        Run init tool if available from world-tools/<world_name>/init.
        
        Checks for a tool named 'init' or '<world_name>-init' in available_tools
        and executes it using execute_action_with_log.
        """
        if not self.available_tools:
            return
        
        # Try to find init tool - check both 'init' and '<world_name>-init'
        init_tool_name = None
        
        # First try exact 'init' name
        if 'init' in self.available_tools:
            init_tool_name = 'init'
        # Then try world-specific name (e.g., 'minecraft-init')
        elif f'{self.world_name}-init' in self.available_tools:
            init_tool_name = f'{self.world_name}-init'
        
        if not init_tool_name:
            logger.debug(f"No init tool found (checked 'init' and '{self.world_name}-init')")
            return
        
        logger.info(f"Running init tool: {init_tool_name}")
        
        try:
            # Execute init tool with no input value
            result = self.execute_action_with_log(
                {"type": init_tool_name},
                "init-tool"
            )
            
            if result.get("status") == "success":
                logger.info(f"Init tool '{init_tool_name}' completed successfully")
            else:
                reason = result.get("reason", "unknown error")
                logger.warning(f"Init tool '{init_tool_name}' failed: {reason}")
        except Exception as e:
            logger.error(f"Error running init tool '{init_tool_name}': {e}", exc_info=True)
    
    def get_world_prompt_context(self) -> Dict[str, str]:
        """
        Get world-specific prompt context sections.
        
        Returns dict mapping section_name -> context_text.
        Worlds can populate this via world_state['_prompt_sections'] (typically set by init tool).
        
        Returns:
            Dict[str, str]: Mapping of section names to context text, empty dict if none available
        """
        prompt_sections = self.get_world_state("_prompt_sections")
        if isinstance(prompt_sections, dict):
            return prompt_sections
        return {}
    
    def clear_plan_state(self):
        """Clear ephemeral plan state (call at start of new plan)"""
        self.plan_bindings = [{}]  # Reset to single empty scope
        if self.executive_node:
            self.executive_node.last_say_text = ''
            self.executive_node.last_out_resource_id = None
    
    def call_subplanner(self, goal: str, context: Optional[str] = None, max_steps: int = 8, trace: Optional[str] = None) -> str:
        """
        Run a nested IncrementalPlanner using binding scope isolation (same executor, isolated scope).
        
        Uses the same isolation model as run_method_protocol:
        - Same executor instance (shared resource manager, tools, runtime)
        - Push binding scope with copy_outer=True (read access to outer bindings)
        - Pop binding scope when done (cleanup)
        
        Args:
            goal: Delegated goal for the sub-planner
            context: Optional context string (blackboard snapshot, etc.)
            max_steps: Maximum planning steps for the nested planner
            trace: Optional trace string (SGLang trace file)
        
        Returns:
            String summary/report from the nested planner (or error message).
        """
        try:
            from incremental_planner import IncrementalPlanner
        except ImportError as e:
            logger.error(f"Subplanner unavailable: {e}")
            return {"success": False, "error": f"Subplanner unavailable: {e}"}
        
        # Push new binding scope for subplanner (copy outer scope for read access)
        logger.info(f"Pushing binding scope for subplanner, goal: {goal}")
        self.push_binding_scope(copy_outer=True)
        
        try:
            # Create planner using same executor instance (shared resource manager, tools, runtime)
            planner = IncrementalPlanner(
                executor=self,
                available_tools=self.available_tools,
                logger_instance=logger,
                vllm_model_path=self.vllm_model if self.vllm_model else None,
                vllm_url=self.vllm_url,
                vllm_model=self.vllm_model,
                openrouter_model_path=self.openrouter_model if self.openrouter_model else None,
                anthropic_model_path=self.anthropic_model if self.anthropic_model else None,
            )
                   
            try:
                result = planner.generate_subplan(template='', goal=goal, context=None, max_steps=max_steps)
            except Exception as exc:
                logger.error(f"Subplanner execution error: {exc}")
                return {"success": False, "error": f"Subplanner execution error: {exc}"}
            
            if result.get('success'):
                # Fix: use 'response' not 'reasoning' (generate_subplan returns 'response')
                reasoning = result.get('response', 'Subplanner completed successfully')
                
                # Substitute bindings in reasoning (use current scope bindings)
                current_bindings = self.plan_bindings[-1] if self.plan_bindings else {}
                for var_name, res_id in current_bindings.items():
                    if res_id:
                        # Replace variable ($note) with resource ID (Note_123)
                        # Handle both with and without $ prefix in case of different usage
                        reasoning = reasoning.replace(f"${var_name}", res_id)
                        reasoning = reasoning.replace(var_name, res_id)
                
                result['response'] = reasoning
                # Retrieve executed plan actions
                plan_actions = getattr(self, '_plan_actions', [])
                
                # Format and substitute bindings in plan
                formatted_plan = []
                for action in plan_actions:
                    action_str = json.dumps(action)
                    for var_name, res_id in current_bindings.items():
                        if res_id:
                            action_str = action_str.replace(f"${var_name}", res_id)
                            action_str = action_str.replace(var_name, res_id)
                    formatted_plan.append(action_str)
                
                plan_text = "\n".join([f"- {a}" for a in formatted_plan])
                logger.info(f"Executed sub-plan: \n{reasoning}\n\nEXECUTED SUB-PLAN:\n{plan_text}")
                return result
            
            result["success"] = False 
            result["error"] = result.get('error', 'unknown error')
            return result
        finally:
            # Always pop the subplanner scope when done
            self.pop_binding_scope()
    
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
        
        content = resource.get('properties', {}).get('content')
        logger.debug(f"_get_content({resource_id}): content type={type(content)}, is_none={content is None}, is_empty_str={content == ''}")
        return content
    
    def _parse_note_content_to_dict(self, content: Any) -> Optional[Dict]:
        """Parse Note content to dict from JSON text only."""
        if isinstance(content, str) and content.strip():
            try:
                parsed = json.loads(content.strip())
                return parsed if isinstance(parsed, dict) else None
            except (json.JSONDecodeError, TypeError):
                return None
        return None
    
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
    
    def _truncate_text_fields(self, obj: Any, max_text_length: int = 128) -> Any:
        """
        Recursively truncate text fields in dicts/lists to max_text_length.
        
        Traverses dicts and lists, truncating string values (but not dict keys) to max_text_length
        with "..." suffix if truncated. Preserves structure and non-string values.
        
        Args:
            obj: Object to process (dict, list, or other)
            max_text_length: Maximum length for text fields (default: 128)
            
        Returns:
            Object with text fields truncated (same structure as input)
        """
        if isinstance(obj, dict):
            truncated_dict = {}
            for key, value in obj.items():
                if isinstance(value, str):
                    if len(value) > max_text_length:
                        truncated_dict[key] = value[:max_text_length] + "..."
                    else:
                        truncated_dict[key] = value
                elif isinstance(value, (dict, list)):
                    truncated_dict[key] = self._truncate_text_fields(value, max_text_length)
                else:
                    truncated_dict[key] = value
            return truncated_dict
        elif isinstance(obj, list):
            return [self._truncate_text_fields(item, max_text_length) for item in obj]
        else:
            return obj
    
    def _format_return_value(self, content: Any, extra: Optional[Dict] = None, max_chars: int = 960) -> str:
        """
        Format content for return value, truncating to max_chars with word boundary awareness.
        For dict/list content, truncates individual text fields to 128 chars first,
        then truncates the overall JSON string if needed.
        Appends extra metadata as suffix (max 64 chars) if provided.
        Args:
            content: Content to format (string, dict, list, etc.)
            extra: Optional metadata dict to append as suffix
            max_chars: Maximum characters for content (default: 960, leaving 64 for extra)
            
        Returns:
            Formatted string, truncated if necessary, with optional extra suffix
        """
        if content is None:
            content = ''
        
        # Convert to string
        if isinstance(content, (dict, list)):
            # Truncate text fields in dict/list before JSON serialization
            truncated_content = self._truncate_text_fields(content, max_text_length=960)
            content_str = json.dumps(truncated_content)
        else:
            content_str = str(content)
        
        if not content_str:
            content_str = ''
        
        # Replace newlines with space-pipe-space for readability
        content_str = content_str.replace('\n', ' | ')
        
        # Truncate if too long, breaking at word boundary
        if len(content_str) > max_chars:
            truncated = content_str[:max_chars]
            # Find last space to avoid mid-word cut
            last_space = truncated.rfind(' ')
            if last_space > max_chars - 20:  # Only if reasonably close to end
                truncated = truncated[:last_space]
            content_str = truncated + '...'
        
        # Append extra metadata as suffix if provided
        if extra:
            # Format extra as key:value pairs
            extra_parts = []
            for key, val in extra.items():
                extra_parts.append(f"{key}: {val}")
            extra_str = ', '.join(extra_parts)
            
            # Truncate extra to 256 chars if needed (increased from 64 to preserve list data in trace)
            if len(extra_str) > 256:
                extra_str = extra_str[:253] + '...'
            
            # Append with space separator
            return f"{content_str} [{extra_str}]"
        
        return content_str
    
    def _get_note_display_name(self, note_id: str, max_length: int = 40) -> str:
        """
        Get display name for a Note (filename, title, uri, or truncated content).
        
        Args:
            note_id: Note ID
            max_length: Maximum length for display name
            
        Returns:
            Display name string, or note_id if unavailable
        """
        if not self.resource_manager:
            return note_id
        
        resource = self.resource_manager.get_resource(note_id)
        if not resource:
            return note_id
        
        props = resource.get('properties', {})
        content = props.get('content', '')
        kind = props.get('kind', '')
        
        # Use tool_metadata (internal) for display; content is always string
        tool_meta = props.get('tool_metadata', {})
        if kind == 'fs-file':
            name = tool_meta.get('name', '') or props.get('source_value', '')
            if name:
                return name[:max_length]
            return note_id
        
        title = tool_meta.get('title', '')
        if title:
            return title[:max_length]
        uri = tool_meta.get('uri', '') or tool_meta.get('url', '')
        if uri:
            return uri[:max_length] if len(uri) <= max_length else uri[:max_length-3] + '...'
        
        # Text content: show truncated preview
        if isinstance(content, str) and content:
            text = content.replace('\n', ' ').replace('\r', ' ')
            if len(text) > max_length:
                return text[:max_length-3] + '...'
            return text
        
        return note_id
    
    def _format_collection_value(self, collection_id: str) -> str:
        """
        Format collection value with name/path prefix and Note display names.
        
        Args:
            collection_id: Collection ID
            
        Returns:
            Formatted string like "bhagavan: 2 items [Note_9: Nan_Ar.txt, Note_10: other.txt]"
            or "papers: 5 items [Note_1: Attention Mechanisms..., ...]"
        """
        if not collection_id or not collection_id.startswith('Collection_'):
            return ''
        
        # Get collection resource
        resource = None
        note_ids = None
        if self.resource_manager:
            resource = self.resource_manager.get_resource(collection_id)
            if resource:
                note_ids = resource.get('properties', {}).get('content', [])
        
        if not isinstance(note_ids, list):
            return f"Collection {collection_id}"
        
        item_count = len(note_ids)
        
        # Get Collection name/path prefix
        collection_prefix = ""
        if resource:
            props = resource.get('properties', {})
            kind = props.get('kind', '')
            
            # fs-dir: show path from doc_meta or collection_name
            if kind == 'fs-dir':
                doc_meta = props.get('doc_meta', {})
                if isinstance(doc_meta, dict) and 'path' in doc_meta:
                    path = doc_meta['path']
                    collection_prefix = path if path else "/"
                else:
                    collection_prefix = props.get('collection_name', '')
            # Other Collections: show collection_name or source_skill
            else:
                collection_name = props.get('collection_name', '')
                if collection_name:
                    collection_prefix = collection_name
                else:
                    source_skill = props.get('source_skill', '')
                    if source_skill:
                        collection_prefix = f"{source_skill} results"
        
        # Format items with display names
        if item_count == 0:
            prefix_str = f"{collection_prefix}: " if collection_prefix else ""
            return f"{prefix_str}0 items []"
        
        # Show up to 5 items with display names
        display_items = []
        for note_id in note_ids[:5]:
            display_name = self._get_note_display_name(note_id)
            if display_name == note_id:
                display_items.append(note_id)
            else:
                display_items.append(f"{note_id}: {display_name}")
        
        note_list_str = ', '.join(display_items)
        if item_count > 5:
            note_list_str += ', ...'
        
        prefix_str = f"{collection_prefix}: " if collection_prefix else ""
        return f"{prefix_str}{item_count} items [{note_list_str}]"
    
    def _create_uniform_return(self, status: str, value: Any = None, resource_id: Optional[str] = None, reason: Optional[str] = None, extra: Optional[Dict] = None, data: Any = None) -> Dict:
        """
        Create uniform return format for all actions.
        
        Args:
            status: 'success' or 'failed'
            value: Content value for display (will be truncated if string/too long)
            resource_id: Resource ID if resource was created/referenced
            reason: Error reason if failed
            extra: Optional metadata dict (passed to Note/Collection properties)
            data: Structured content for code blocks. If None, auto-populated:
                  - Notes: string content from resource properties
                  - Collections: list of dicts with 'text' and 'metadata' per item
                  - Scalars: the raw value
                  - Failed: None
            
        Returns:
            Uniform return dict with type='uniform_return', status, value (display), data (structured), extra, resource_id, reason
        """
        result = {
            'type': 'uniform_return',
            'status': status
        }

        if status == 'failed':
            # Promote extra["reason"] if no explicit reason was passed (common codegen mistake)
            if not reason and extra and isinstance(extra, dict) and extra.get('reason'):
                reason = str(extra['reason'])
            result['reason'] = reason or 'Unknown error'
            result['data'] = None
            result['value'] = self._format_return_value(value or reason, extra=extra, max_chars=960)
            result['resource_id'] = None
        else:
            # data: use explicit data arg if provided, otherwise fall back to value
            result['data'] = data if data is not None else value
            result['value'] = self._format_return_value(value, extra=extra, max_chars=960) if value is not None else None
            result['resource_id'] = resource_id
        
        # Preserve extra as separate field
        if extra:
            result['extra'] = extra
        
        # Auto-enrich for resource-backed results
        if status == 'success' and resource_id and isinstance(resource_id, str) and self.resource_manager:
            resource = self.resource_manager.get_resource(resource_id)
            if resource:
                props = resource.get('properties', {})
                # Auto-inject item_count for Collections
                if resource_id.startswith('Collection_'):
                    if 'extra' not in result or result['extra'] is None:
                        result['extra'] = {}
                    result['extra'].setdefault('item_count', props.get('item_count', 0))
                # Auto-populate data from resource content if not explicitly set
                if result['data'] is None or (data is None and isinstance(result['data'], str) and result['data'] == value):
                    content = props.get('content')
                    if resource_id.startswith('Note_'):
                        # Text-only model: r["data"] always string for Notes (defensive normalize)
                        result['data'] = content if isinstance(content, str) else json.dumps(content, ensure_ascii=False)
                    elif resource_id.startswith('Collection_') and isinstance(content, list):
                        # Build structured items: text + metadata per Note
                        items = []
                        for item_id in content:
                            item_resource = self.resource_manager.get_resource(item_id)
                            if item_resource:
                                item_content = item_resource.get('properties', {}).get('content', '')
                                items.append({'text': item_content if isinstance(item_content, str) else str(item_content)})
                        result['data'] = items
        
        return result
    
    def execute_action_tracked(self, action: Dict, method_name: str = None) -> Dict:
        """
        Execute an action with full side-effect tracking: plan_actions, ActionRecord,
        UI publish (with optional method prefix), and compliance.
        
        This is the single consolidated execution path used by the planner loop,
        Python tool files, and codegen blocks.
        
        Args:
            action: Action dict with 'type' field
            method_name: Optional method/caller name for UI log prefix (e.g., 'mc-seek-boundary', 'codegen')
            
        Returns:
            Result dict in uniform_return format
        """
        # Track action in plan_actions
        if hasattr(self, '_plan_actions'):
            self._plan_actions.append(action.copy())

        def _bind_cached_result(cached_result: Dict):
            out_var = action.get('out')
            resource_id = cached_result.get('resource_id') if isinstance(cached_result, dict) else None
            if out_var and isinstance(out_var, str) and out_var.startswith('$') and resource_id:
                self._bind_variable(out_var.lstrip('$'), resource_id)
            return cached_result

        # Dedup guard for side-effectful tools within a single planner session.
        # If an identical send-email or post-bluesky was already executed successfully
        # in this session, return the cached result instead of re-executing.
        _SIDE_EFFECT_TOOLS = {'send-email', 'post-bluesky'}
        action_type = action.get('type', '')
        if action_type in _SIDE_EFFECT_TOOLS:
            if not hasattr(self, '_successful_side_effect_results'):
                self._successful_side_effect_results = {}
            if getattr(self, '_done_gate_retry_active', False) and not action.get('allow_repeat_side_effect'):
                prior_success = self._successful_side_effect_results.get(action_type)
                if prior_success:
                    logger.warning(
                        f"Dedup: blocking repeated {action_type} during done-gate retry "
                        f"(returning prior successful result)"
                    )
                    return _bind_cached_result(prior_success)
            if not hasattr(self, '_side_effect_cache'):
                self._side_effect_cache = {}
            # Build dedup key from action content (resolved target text + key params)
            _dedup_parts = [action_type]
            # Resolve target/value to actual text for content-based dedup
            for field in ('target', 'value', 'to', 'subject'):
                val = action.get(field, '')
                if isinstance(val, str) and val.startswith('$'):
                    # Resolve $var → resource ID → Note content
                    rid = self.plan_bindings_flat.get(val.lstrip('$'))
                    if rid and self.resource_manager and isinstance(rid, str) and rid.startswith('Note_'):
                        try:
                            note = self.resource_manager.get_resource(rid)
                            if note and note.get('content'):
                                val = str(note['content'])[:500]
                        except Exception:
                            pass
                if val:
                    _dedup_parts.append(str(val))
            import hashlib as _hl
            _dedup_key = _hl.sha256('|'.join(_dedup_parts).encode('utf-8')).hexdigest()[:24]
            cached = self._side_effect_cache.get(_dedup_key)
            if cached:
                logger.warning(f"Dedup: skipping duplicate {action_type} within planner session (returning cached result)")
                return _bind_cached_result(cached)

        # Execute the action
        result = self.execute_action(action)
        
        # Ensure result is in uniform_return format
        if result.get('type') != 'uniform_return':
            result = self._create_uniform_return(
                result.get('status', 'unknown'),
                value=result.get('value'),
                reason=result.get('reason'),
                extra=result.get('extra')
            )

        # Cache successful side-effect results for dedup
        if action_type in _SIDE_EFFECT_TOOLS and result.get('status') == 'success' and hasattr(self, '_side_effect_cache'):
            self._side_effect_cache[_dedup_key] = result
            if hasattr(self, '_successful_side_effect_results'):
                self._successful_side_effect_results[action_type] = result

        # Create ActionRecord + publish to UI
        if self.executive_node:
            action_type = action.get('type', 'unknown')
            display_type = f"{method_name}:{action_type}" if method_name else action_type
            timestamp = datetime.now()
            
            try:
                # Import ActionRecord (avoid circular import)
                from executive_node import ActionRecord
                
                self.executive_node.step_counter += 1
                result_str = result.get('value', '') if result.get('status') == 'success' else result.get('reason', 'failed')
                
                action_record = ActionRecord(
                    action=action,
                    result=result_str,
                    result_dict=result.copy(),
                    timestamp=timestamp,
                    step_id=self.executive_node.step_counter,
                    plan_id=getattr(self.executive_node, 'current_plan_id', None),
                    requested_target=action.get('target', ''),
                    started_at=timestamp,
                    ended_at=datetime.now(),
                    outcome_status=result.get('status', 'unknown')
                )
                if hasattr(self.executive_node, '_snapshot_physiology'):
                    self.executive_node._snapshot_physiology(action_record)
                self.executive_node.action_history.append(action_record)
            except Exception as e:
                logger.debug(f"ActionRecord creation failed (non-fatal): {e}")
            
            try:
                self.executive_node._publish_action_result(action, result, display_type, timestamp)
                logger.debug(f"Published action result: {display_type} -> status={result.get('status')}")
            except Exception as e:
                logger.error(f"Failed to publish action result for {display_type}: {e}", exc_info=True)
        
        # Track compliance
        if hasattr(self, '_compliance_tracker') and self._compliance_tracker:
            self._compliance_tracker.check_action(action, result, self.plan_bindings_flat)
        
        # Increment error counter on failure
        if result.get('status') != 'success':
            if hasattr(self, '_plan_error_count'):
                self._plan_error_count += 1
        
        return result
    
    def execute_action_with_log(self, action: Dict, method_name: str) -> Dict:
        """
        Execute an action and publish it to the action log with method name prefix.
        
        Convenience wrapper that delegates to execute_action_tracked().
        Backward-compatible: same signature, same return type.
        
        Args:
            action: Action dict with 'type' field
            method_name: Name of the calling method (e.g., 'mc-seek-boundary')
            
        Returns:
            Result dict with 'status' field ('success', 'failed', 'retry')
        """
        return self.execute_action_tracked(action, method_name)
    
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
            return self._create_uniform_return('failed', reason='Missing type field')
        
        # Route to appropriate handler
        handlers = {
            # Phase 1: Core, Storage, Control, Communication
            'apply': self._execute_apply,
            'create-note': self._execute_create_note,
            'create-collection': self._execute_create_collection,
            'create-relation': self._execute_create_relation,
            'find-relations': self._execute_find_relations,
            'related': self._execute_related,
            'get-metadata': self._execute_get_metadata,
            'set-metadata': self._execute_set_metadata,
            'persist': self._execute_persist,
            'load': self._execute_load,
            'index': self._execute_index,
            'organize': self._execute_index,  # Alias for index
            'search-within-collection': self._execute_search_within_collection,
            'discover-notes': self._execute_discover_notes,
            'discover-collections': self._execute_discover_collections,
            'say': self._execute_say,
            'think': self._execute_think,
            'ask': self._execute_ask,
            'bind': self._execute_bind,
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
        
        # Validate 'out' field is assignment-style variable (must start with $)
        if 'out' in action:
            out_val = action['out']
            if not isinstance(out_val, str) or not out_val or not out_val.startswith('$'):
                error_msg = f"Invalid 'out' field: '{out_val}' must be a variable name starting with $"
                logger.error(error_msg)
                return self._create_uniform_return('failed', reason=error_msg)
        
        # Truncate action log for readability (keep full data for execution)
        action_str = json.dumps(action)
        if len(action_str) > 128:
            action_str = action_str[:125] + "..."
        logger.info(f"Executing action: {action_str}")
        handler = handlers.get(action_type)
        if not handler:
            # Check if it's a dynamic tool (not a primitive)
            if action_type in self.available_tools:
                # Route to apply logic for tool execution
                return self._execute_apply(action)
            else:
                logger.error(f"Unknown action type: {action_type}")
                return self._create_uniform_return('failed', reason=f'Unknown action: {action_type}')
        
        try:
            result = handler(action)
            return result
        except Exception as e:
            logger.error(f"Error executing action {action_type}: {e}")
            logger.error(traceback.format_exc())
            return self._create_uniform_return('failed', reason=f'Execution error: {str(e)}')
    
    # ==================== Core Operations ====================
        
    def _execute_apply(self, action: Dict) -> Dict:
        """
        Apply tool to input data.
        
        Supports direct format: {"type": "tool-name", "target": ..., "out": "$var", ...}
        
        Required: type, out
        Optional: target (input data), any tool-specific parameters at top level
        
        All tool parameters are at top level (flat format, no nested 'args' field):
        - target: $variable (input data) - resolves to Note/Collection content
        - out: literal string (variable name)
        - Any other fields: tool-specific parameters (prompt, query, instruction, etc.)
        """
        action_type = action.get('type')
        
        # Reserved fields that are NOT tool parameters
        reserved_fields = {'type', 'target', 'value', 'out', 'expect', 'reason'}
        
        # Direct format: type is the tool name, target is input data
        if action_type in self.available_tools:
            target = action_type  # Tool name
            # Input data: use 'target' if present, otherwise fall back to 'value'
            target_field = action.get('target')
            value_field = action.get('value')
            
            if target_field:
                try:
                    value = self._resolve_value(target_field)
                    logger.debug(f"_execute_apply: resolved target '{target_field}' -> value type={type(value)}, is_none={value is None}, is_empty={value == ''}, truthy={bool(value)}")
                    # Check if target resolved to None or empty (unbound variable, Note not found, or empty content)
                    if value is None:
                        return self._create_uniform_return('failed', reason=f'target "{target_field}" resolved to None (variable unbound or resource not found)')
                    # Also check for empty string - some tools require non-empty input
                    if value == '':
                        logger.warning(f"target '{target_field}' resolved to empty string - tool may fail")
                except ValueError as e:
                    # Unbound variable - for map tools, let them handle default
                    tool_name = action_type
                    if tool_name in ['mc-map-visualize', 'mc-map-query', 'mc-map-update', 'mc-waypoint']:
                        # Map tools handle target defaults internally - pass through as None
                        logger.debug(f"Unbound variable {target_field} for map tool {tool_name}, tool will use default")
                        value = None
                    else:
                        # Other tools - fail
                        logger.error(f"Unbound variable: {target_field}")
                        return self._create_uniform_return('failed', reason=str(e))
            elif value_field is not None:
                # Use 'value' field as input (for tools like calculate that don't use target)
                value = self._resolve_value(value_field)
                logger.debug(f"_execute_apply: resolved value '{value_field}' -> value type={type(value)}, is_none={value is None}, is_empty={value == ''}, truthy={bool(value)}")
                if value is None:
                    return self._create_uniform_return('failed', reason=f'value "{value_field}" resolved to None (variable unbound or resource not found)')
            else:
                value = ''
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
            return self._create_uniform_return('failed', reason='apply requires target or tool name in type')
        
        # Get tool info and route by type
        tool_info = self._get_tool_info(target)
        
        if not tool_info:
            return self._create_uniform_return('failed', reason=f'Tool not found: {target}')
        
        tool_type = tool_info.get('type')
        
        if tool_type == 'instruction':
            # Instruction tools: return body text directly
            body_text = tool_info.get('body_text')
            if not body_text:
                return self._create_uniform_return('failed', reason=f'Instruction tool {target} has no body text')
            # Return body text as result in uniform_return format
            result = self._create_uniform_return('success', value=body_text, resource_id=None)
        elif tool_type == 'prompt_augmentation':
            # Execute locally using LLM
            result = self._execute_prompt_tool(target, value, tool_info, additional_args)
        elif tool_type == 'python':
            # Execute Python tool
            result = self._execute_python_tool(target, value, tool_info, additional_args)
        else:
            return self._create_uniform_return('failed', reason=f'Unknown tool type: {tool_type}')
        
        # Validate that tool returned uniform_return format
        if result.get('type') != 'uniform_return':
            logger.error(f"Tool '{target}' returned non-uniform_return format: {result.get('type', 'missing type')}. Expected uniform_return.")
            # Convert to uniform_return for safety
            result = self._create_uniform_return(
                'failed' if result.get('status') != 'success' else 'success',
                value=result.get('value'),
                resource_id=result.get('resource_id'),
                reason=result.get('reason')
            )
        
        if result.get('status') != 'success':
            return result
        
        # Extract result components from uniform_return format
        # Always use 'data' field (raw value) - uniform_return is guaranteed
        data_field = result.get('data', value)
        result_resource_id = result.get('resource_id')
        extra_metadata = result.get('extra')  # Extract extra metadata for Note properties
        
        note_content = data_field
        display_value = result.get('value') or str(data_field)
        
        # Bind result if output variable specified
        if out_var:
            # Check if tool already returned a resource_id (e.g., Collection-creating tools)
            if result_resource_id and (result_resource_id.startswith('Note_') or result_resource_id.startswith('Collection_')):
                # Tool already created a resource - bind it
                self._bind_variable(out_var, result_resource_id)
                display_var = out_var if out_var.startswith('$') else f"${out_var}"
                logger.info(f"Tool '{target}' executed, resource {result_resource_id} → {display_var}")
                # Return uniform format with truncated value for API/planner
                return self._create_uniform_return('success', value=display_value, resource_id=result_resource_id)
            
            # Tool returned content (not a resource ID) - persist as Note with data content
            # Pass extra metadata to _persist_note (resource_manager will filter allowed fields)
            info_id = self._persist_note(note_content, f'apply_{target}', properties=extra_metadata)
            if info_id:
                self._bind_variable(out_var, info_id)
                display_var = out_var if out_var.startswith('$') else f"${out_var}"
                logger.info(f"Tool '{target}' executed, Note {info_id} → {display_var}")
                # Return uniform format with truncated value for API/planner
                return self._create_uniform_return('success', value=display_value, resource_id=info_id)
            else:
                logger.error(f"Failed to persist result from '{target}'")
                return self._create_uniform_return('failed', reason='Failed to persist result')
        
        # No out_var - result is already uniform_return, return as-is
        return result

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
                return self._create_uniform_return('failed', reason=f'No SKILL.md content for {tool_name}', value=None, resource_id=None)
            
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
            
            # Call LLM using unified wrapper (use alt for selected tools when configured, else main)
            generate_fn = self._alt_llm_generate if tool_name in ALT_LLM_TOOLS else self.llm_generate
            llm_response = generate_fn(
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
                return self._create_uniform_return('failed', reason='LLM returned invalid response format', value=None, resource_id=None)
            
            if not result_text:
                return self._create_uniform_return('failed', reason='LLM returned empty response', value=None, resource_id=None)
            logger.info(f"Prompt tool '{tool_name}' completed ({len(result_text)} chars)")
            
            # Check for null indicator - return Note_null resource ID instead of string
            if result_text.lower() == 'note-null' or result_text.lower() == 'null':
                logger.info(f"Prompt tool '{tool_name}' returned null indicator, using Note_null resource")
                return self._create_uniform_return('success', value='Note_null', resource_id=None)
            
            return self._create_uniform_return('success', value=result_text, resource_id=None)
            
        except Exception as e:
            logger.error(f"Prompt tool execution failed: {e}")
            return self._create_uniform_return('failed', reason=str(e), value=None, resource_id=None)

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
            
            # Log and traceback if content is too large
            prompt_length = len(prompt)
            if prompt_length > 100000:
                logger.warning(f"⚠️  Attempting to send {prompt_length} chars to sglang")
            
            # Create a simple generation function
            @function
            def generate_text(s, prompt_text, is_json=False):
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
            logger.exception(f"SGLang generation error: {e}")
            return Response(success=False, error=str(e))
    
    def _vllm_generate(self, messages, max_tokens=2000, temperature=0.7, stops=None, is_json=False, use_alt=False):
        """
        Generate text using vLLM API via HTTP.
        Uses /v1/chat/completions endpoint with messages array format for consistency.
        
        Args:
            messages: List of message strings (or single string)
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            stops: List of stop sequences
            is_json: If True, parse response as JSON and return dict
            use_alt: If True, use alt_vllm_* config (for alt_llm_config)
            
        Returns:
            Object with .success, .text, and .error attributes (matching llm_client response)
            If is_json=True, .text will be a dict (parsed JSON) instead of string
        """
        import requests
        
        model = self.alt_vllm_model if use_alt else self.vllm_model
        url = self.alt_vllm_url if use_alt else self.vllm_url
        
        class Response:
            def __init__(self, success, text='', error=None):
                self.success = success
                self.text = text
                self.error = error

        try:
            # Convert messages to chat format
            if isinstance(messages, list):
                # If list of strings, convert to messages array
                chat_messages = [{"role": "system", "content": "Do not include reasoning, analysis, justification, or explanation. Respond only with the final answer."}]
                for msg in messages:
                    if isinstance(msg, str):
                        # Simple heuristic: if starts with system/user/assistant markers, parse them
                        if msg.strip().startswith('<|system|>'):
                            content = msg.replace('<|system|>', '').strip()
                            chat_messages.append({"role": "system", "content": content})
                        elif msg.strip().startswith('<|user|>'):
                            content = msg.replace('<|user|>', '').strip()
                            chat_messages.append({"role": "user", "content": content})
                        elif msg.strip().startswith('<|assistant|>'):
                            content = msg.replace('<|assistant|>', '').strip()
                            chat_messages.append({"role": "assistant", "content": content})
                        else:
                            # Default to user message
                            chat_messages.append({"role": "user", "content": msg})
                    else:
                        # Already a dict with role/content
                        chat_messages.append(msg)
            else:
                # Single string - treat as user message
                chat_messages = [{"role": "user", "content": str(messages)}]
            
            # Prepare payload
            payload = {
                "model": model,
                "messages": chat_messages,
                "max_tokens": max_tokens + (256 if self.reasoning_config else 0),
                "temperature": temperature,
            }
            if self.reasoning_config:
                payload["reasoning"] = self.reasoning_config
            if stops:
                if isinstance(stops, list):
                    payload["stop"] = stops
                else:
                    payload["stop"] = [stops]

            
            
            # Call vLLM API
            logger.debug(f"Calling vLLM API: {url} with model {model}")
            response = requests.post(url, json=payload, timeout=120)
            response.raise_for_status()  # Fail fast
            
            result = response.json()
            logger.debug(f"vLLM API response structure: {list(result.keys())}")
            
            # Extract content from response
            choices = result.get("choices", [])
            if not choices:
                logger.error(f"vLLM API returned no choices. Full response: {result}")
                return Response(success=False, error="vLLM API returned no choices")
            
            message = choices[0].get("message", {})
            result_text = message.get("content")
            
            # Handle None or empty content
            if result_text is None:
                logger.warning(f"vLLM API returned None content. Message: {message}, Full response: {result}")
                return Response(success=False, error="vLLM API returned None content")
            
            # Post-process JSON if requested
            if is_json:
                result_text = self._parse_json_response(result_text, str(messages))
            
            return Response(success=True, text=result_text)
        except requests.exceptions.RequestException as e:
            logger.exception(f"vLLM API error: {e}")
            return Response(success=False, error=str(e))
        except (KeyError, ValueError) as e:
            logger.exception(f"vLLM response parsing error: {e}")
            return Response(success=False, error=str(e))
        except Exception as e:
            logger.exception(f"vLLM generation error: {e}")
            return Response(success=False, error=str(e))
    
    def _anthropic_generate(self, messages, max_tokens=2000, temperature=0.7, stops=None, is_json=False, use_alt=False):
        """
        Generate text using Anthropic API directly.
        Uses Messages API with anthropic SDK.
        
        Args:
            messages: List of message strings or dicts with role/content
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            stops: List of stop sequences
            is_json: If True, parse response as JSON and return dict
            
        Returns:
            Object with .success, .text, and .error attributes
        """
        model = self.alt_anthropic_model if use_alt else self.anthropic_model
        api_key = self.alt_anthropic_api_key if use_alt else self.anthropic_api_key
        
        class Response:
            def __init__(self, success, text='', error=None):
                self.success = success
                self.text = text
                self.error = error

        try:
            from anthropic import Anthropic
            client = Anthropic(api_key=api_key)

            # Convert messages to Anthropic format (system + messages)
            # Normalize all content: Anthropic rejects assistant content ending with trailing whitespace
            def _norm(s):
                return s.rstrip() if isinstance(s, str) else s

            system_parts = []
            api_messages = []
            if isinstance(messages, list):
                for msg in messages:
                    if isinstance(msg, str):
                        if msg.strip().startswith('<|system|>'):
                            system_parts.append(_norm(msg.replace('<|system|>', '').strip()))
                        elif msg.strip().startswith('<|user|>'):
                            api_messages.append({"role": "user", "content": _norm(msg.replace('<|user|>', '').strip())})
                        elif msg.strip().startswith('<|assistant|>'):
                            api_messages.append({"role": "assistant", "content": _norm(msg.replace('<|assistant|>', '').strip())})
                        else:
                            api_messages.append({"role": "user", "content": _norm(msg)})
                    elif isinstance(msg, dict):
                        role = msg.get("role", "user")
                        content = _norm(msg.get("content", ""))
                        if role == "system":
                            system_parts.append(content)
                        else:
                            api_messages.append({"role": role, "content": content})
            else:
                api_messages = [{"role": "user", "content": _norm(str(messages))}]

            system_text = "\n\n".join(system_parts) if system_parts else None
            # Anthropic rejects empty/whitespace-only content blocks - filter them out
            api_messages = [m for m in api_messages if m.get("content") and str(m["content"]).strip()]
            if not api_messages:
                return Response(success=False, error="No user messages to send")
            # Some Anthropic models reject assistant prefill: final turn must be user.
            if api_messages[-1].get("role") != "user":
                api_messages[-1]["role"] = "user"

            kwargs = {
                "model": model,
                "max_tokens": max_tokens,
                "temperature": temperature,
                "messages": api_messages,
            }
            if system_text:
                kwargs["system"] = system_text
            if stops:
                stop_list = stops if isinstance(stops, list) else [stops]
                # Anthropic rejects stop sequences that are empty or whitespace-only
                stop_list = [s for s in stop_list if s is not None and str(s).strip()]
                if stop_list:
                    kwargs["stop_sequences"] = stop_list

            response = client.messages.create(**kwargs)
            result_text = response.content[0].text if response.content else None

            if result_text is None:
                return Response(success=False, error="Anthropic API returned no content")

            if is_json:
                result_text = self._parse_json_response(result_text, str(messages))

            return Response(success=True, text=result_text)
        except Exception as e:
            logger.exception(f"Anthropic API error: {e}")
            return Response(success=False, error=str(e))

    def _openai_generate(self, messages, max_tokens=2000, temperature=0.7, stops=None, is_json=False, use_alt=False, reasoning_effort=None):
        """
        Generate text using the OpenAI Responses API.

        Args:
            messages: List of message strings or dicts with role/content
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            stops: Optional stop sequences (applied client-side to returned text)
            is_json: If True, request JSON mode and parse the result
            use_alt: If True, use alt_openai_* config

        Returns:
            Object with .success, .text, and .error attributes
        """
        model = self.alt_openai_model if use_alt else self.openai_model
        api_key = self.alt_openai_api_key if use_alt else self.openai_api_key

        class Response:
            def __init__(self, success, text='', error=None):
                self.success = success
                self.text = text
                self.error = error

        try:
            from openai import OpenAI

            client = OpenAI(api_key=api_key, timeout=180.0, max_retries=2)

            def _norm_text(value):
                return value.rstrip() if isinstance(value, str) else value

            instructions_parts = []
            input_items = []

            if isinstance(messages, list):
                for msg in messages:
                    if isinstance(msg, str):
                        raw = msg.strip()
                        if raw.startswith('<|system|>'):
                            instructions_parts.append(_norm_text(raw.replace('<|system|>', '', 1).strip()))
                            continue
                        if raw.startswith('<|user|>'):
                            role = "user"
                            content = _norm_text(raw.replace('<|user|>', '', 1).strip())
                        elif raw.startswith('<|assistant|>'):
                            role = "assistant"
                            content = _norm_text(raw.replace('<|assistant|>', '', 1).strip())
                        else:
                            role = "user"
                            content = _norm_text(msg)
                    elif isinstance(msg, dict):
                        role = msg.get("role", "user")
                        content = _norm_text(msg.get("content", ""))
                        if role == "system":
                            instructions_parts.append(content)
                            continue
                    else:
                        role = "user"
                        content = _norm_text(str(msg))

                    if content and str(content).strip():
                        input_items.append({
                            "role": role,
                            "content": str(content),
                        })
            else:
                content = _norm_text(str(messages))
                if content and str(content).strip():
                    input_items.append({
                        "role": "user",
                        "content": str(content),
                    })

            if not input_items:
                return Response(success=False, error="No user messages to send")

            effort = reasoning_effort or "low"
            payload = {
                "model": model,
                "input": input_items,
                "max_output_tokens": max_tokens,
                "reasoning": {"effort": effort},
            }
            if instructions_parts:
                payload["instructions"] = "\n\n".join(part for part in instructions_parts if part and str(part).strip())
            if is_json:
                payload["text"] = {"format": {"type": "json_object"}}

            response = client.responses.create(**payload)
            result_text = getattr(response, "output_text", None)

            if result_text is None:
                return Response(success=False, error="OpenAI Responses API returned no text")

            if stops and isinstance(result_text, str):
                stop_list = stops if isinstance(stops, list) else [stops]
                cutoffs = [result_text.find(stop) for stop in stop_list if stop]
                cutoffs = [idx for idx in cutoffs if idx >= 0]
                if cutoffs:
                    result_text = result_text[:min(cutoffs)]

            if is_json:
                result_text = self._parse_json_response(result_text, str(messages))

            return Response(success=True, text=result_text)
        except Exception as e:
            logger.exception(f"OpenAI API error: {e}")
            return Response(success=False, error=str(e))

    def _openrouter_generate(self, messages, max_tokens=2000, temperature=0.7, stops=None, is_json=False, use_alt=False):
        """
        Generate text using OpenRouter API via HTTP.
        Uses /v1/chat/completions endpoint with messages array format (OpenAI-compatible).
        
        Args:
            messages: List of message strings (or single string)
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            stops: List of stop sequences
            is_json: If True, parse response as JSON and return dict
            use_alt: If True, use alt_openrouter_* config (for alt_llm_config)
            
        Returns:
            Object with .success, .text, and .error attributes (matching llm_client response)
            If is_json=True, .text will be a dict (parsed JSON) instead of string
        """
        import requests
        
        model = self.alt_openrouter_model if use_alt else self.openrouter_model
        api_key = self.alt_openrouter_api_key if use_alt else self.openrouter_api_key
        provider = self.alt_openrouter_provider if use_alt else self.openrouter_provider
        
        class Response:
            def __init__(self, success, text='', error=None):
                self.success = success
                self.text = text
                self.error = error

        try:
            # Convert messages to chat format (same as vLLM)
            if isinstance(messages, list):
                chat_messages = []
                for msg in messages:
                    if isinstance(msg, str):
                        # Simple heuristic: if starts with system/user/assistant markers, parse them
                        if msg.strip().startswith('<|system|>'):
                            content = msg.replace('<|system|>', '').strip()
                            chat_messages.append({"role": "system", "content": content})
                        elif msg.strip().startswith('<|user|>'):
                            content = msg.replace('<|user|>', '').strip()
                            chat_messages.append({"role": "user", "content": content})
                        elif msg.strip().startswith('<|assistant|>'):
                            content = msg.replace('<|assistant|>', '').strip()
                            chat_messages.append({"role": "assistant", "content": content})
                        else:
                            # Default to user message
                            chat_messages.append({"role": "user", "content": msg})
                    else:
                        # Already a dict with role/content
                        chat_messages.append(msg)
            else:
                # Single string - treat as user message
                chat_messages = [{"role": "user", "content": str(messages)}]
            
            # Prepare payload (OpenRouter uses OpenAI-compatible format)
            payload = {
                "model": model,
                "messages": chat_messages,
                "max_tokens": max_tokens,
                "temperature": temperature,
                "reasoning": {"effort": "low"},
                "top_p": 1.0,
                "stream": False,
            }
            if stops:
                if isinstance(stops, list):
                    payload["stop"] = stops
                else:
                    payload["stop"] = [stops]
            if provider:
                payload["provider"] = {"order": [provider], "allow_fallbacks": True}
            
            # Call OpenRouter API
            headers = {
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
                "HTTP-Referer": "https://github.com/cognitive-workbench",  # Optional but recommended
                "X-Title": "Cognitive Workbench"  # Optional but recommended
            }
            
            logger.debug(f"Calling OpenRouter API with model {model}")
            response = requests.post(
                "https://openrouter.ai/api/v1/chat/completions",
                json=payload,
                headers=headers,
                timeout=120
            )
            response.raise_for_status()  # Fail fast
            
            result = response.json()
            logger.debug(f"OpenRouter API response structure: {list(result.keys())}")
            
            # Extract content from response (same format as OpenAI/vLLM)
            choices = result.get("choices", [])
            if not choices:
                logger.error(f"OpenRouter API returned no choices. Full response: {result}")
                return Response(success=False, error="OpenRouter API returned no choices")
            
            message = choices[0].get("message", {})
            result_text = message.get("content")
            
            # Handle None or empty content (models may return empty during warm-up or for unsupported params)
            if result_text is None:
                logger.warning(f"OpenRouter API returned None content. Message: {message}, Full response: {result}")
                return Response(success=False, error="OpenRouter API returned None content")
            if isinstance(result_text, str) and not result_text.strip():
                logger.warning(f"OpenRouter API returned empty content. Message: {message}")
                return Response(success=False, error="OpenRouter API returned empty content")
            
            # Post-process JSON if requested
            if is_json:
                result_text = self._parse_json_response(result_text, str(messages))
            
            return Response(success=True, text=result_text)
        except requests.exceptions.RequestException as e:
            logger.exception(f"OpenRouter API error: {e}")
            return Response(success=False, error=str(e))
        except (KeyError, ValueError) as e:
            logger.exception(f"OpenRouter response parsing error: {e}")
            return Response(success=False, error=str(e))
        except Exception as e:
            logger.exception(f"OpenRouter generation error: {e}")
            return Response(success=False, error=str(e))
    
    def llm_generate(self, messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None, reasoning_effort=None):
        """Unified LLM generation interface using SGLang runtime or configured remote backends."""
        # Apply bindings to messages if provided
        if bindings and isinstance(messages, list):
            processed_messages = []
            for msg in messages:
                if isinstance(msg, str):
                    processed_msg = msg
                    for key, value in bindings.items():
                        processed_msg = processed_msg.replace(f"{{${key}}}", str(value))
                    processed_messages.append(processed_msg)
                else:
                    processed_messages.append(msg)
            messages = processed_messages
        
        # Use Anthropic if configured, then OpenAI, then OpenRouter, then vLLM, then SGLang
        if self.anthropic_model and self.anthropic_api_key:
            return self._anthropic_generate(messages, max_tokens, temperature, stops, is_json=is_json)
        elif self.openai_model and self.openai_api_key:
            return self._openai_generate(messages, max_tokens, temperature, stops, is_json=is_json, reasoning_effort=reasoning_effort)
        elif self.openrouter_model and self.openrouter_api_key:
            return self._openrouter_generate(messages, max_tokens, temperature, stops, is_json=is_json)
        elif self.vllm_model and self.vllm_url:
            return self._vllm_generate(messages, max_tokens, temperature, stops, is_json=is_json)
        elif self.runtime:
            return self._sglang_generate(messages, max_tokens, temperature, stops, is_json=is_json)
        else:
            raise RuntimeError("No LLM backend available - ensure executive_node is started with sgl_model_path, vllm_model_path, anthropic_model_path, openai_model_path, or openrouter_model_path")
    
    def _alt_llm_generate(self, messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
        """Unified LLM generation using alt backend when configured; falls back to main llm_generate otherwise."""
        if bindings and isinstance(messages, list):
            processed_messages = []
            for msg in messages:
                if isinstance(msg, str):
                    processed_msg = msg
                    for key, value in bindings.items():
                        processed_msg = processed_msg.replace(f"{{${key}}}", str(value))
                    processed_messages.append(processed_msg)
                else:
                    processed_messages.append(msg)
            messages = processed_messages
        has_alt = (
            (self.alt_anthropic_model and self.alt_anthropic_api_key) or
            (self.alt_openai_model and self.alt_openai_api_key) or
            (self.alt_openrouter_model and self.alt_openrouter_api_key) or
            (self.alt_vllm_model and self.alt_vllm_url)
        )
        if has_alt:
            if self.alt_anthropic_model and self.alt_anthropic_api_key:
                return self._anthropic_generate(messages, max_tokens, temperature, stops, is_json=is_json, use_alt=True)
            elif self.alt_openai_model and self.alt_openai_api_key:
                return self._openai_generate(messages, max_tokens, temperature, stops, is_json=is_json, use_alt=True)
            elif self.alt_openrouter_model and self.alt_openrouter_api_key:
                return self._openrouter_generate(messages, max_tokens, temperature, stops, is_json=is_json, use_alt=True)
            elif self.alt_vllm_model and self.alt_vllm_url:
                return self._vllm_generate(messages, max_tokens, temperature, stops, is_json=is_json, use_alt=True)
        return self.llm_generate(messages, bindings=None, max_tokens=max_tokens, temperature=temperature, is_json=is_json, stops=stops)
    
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

        # Strip <think>...</think> blocks (Qwen3 thinking mode) before parsing
        response = re.sub(r'<think>.*?</think>\s*', '', response, flags=re.DOTALL)

        # Remove markdown code fences if present
        response = response.replace("```json", "").replace("```", "").strip()
        
        # Try direct parse first
        try:
            return json.loads(response)
        except json.JSONDecodeError as e:
            logger.error(f'Simple JSON repair failed: {e}')
            logger.debug(f'Failed to parse JSON from: {response_text[:200]}...')
        
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
                return self._create_uniform_return('failed', reason=f'Plan tool {tool_name} missing plan_data', value=None, resource_id=None)
            
            # Bind main input to $input variable
            if input_value is not None:
                input_note_id = self._persist_note(input_value, f'{tool_name}_input')
                if input_note_id:
                    self._bind_variable('input', input_note_id)
            
            # Bind args dict values to named variables
            for key, val in additional_args.items():
                resolved_val = self._resolve_value(val)
                arg_note_id = self._persist_note(resolved_val, f'{tool_name}_{key}')
                if arg_note_id:
                    self._bind_variable(key, arg_note_id)
            
            # Execute plan synchronously
            result = self.execute_plan_sync(plan_data)
            
            if result.get('status') == 'suspended':
                # Plan suspended (ask/wait) - return suspension (already in uniform format)
                return result
            
            if result.get('status') != 'success':
                return self._create_uniform_return('failed', reason=f'Plan tool execution failed: {result.get("reason")}', value=None, resource_id=None)
            
            # Extract output from plan's 'out' variable (from plan_data)
            out_var = plan_data.get('out', 'result')
            if out_var.startswith('$'):
                out_var = out_var[1:]
            
            # Get output from isolated executor's bindings (returned in result)
            bindings = result.get('bindings', {})
            if out_var in bindings:
                output_note_id = bindings[out_var]
                output_content = self._get_content(output_note_id)
                return self._create_uniform_return('success', value=output_content, resource_id=output_note_id)
            else:
                return self._create_uniform_return('failed', reason=f'Plan tool output variable ${out_var} not bound', value=None, resource_id=None)
        
        # Handle Python-based tools
        # Get Python file path
        python_file = tool_info.get('python_file')
        if not python_file:
            return self._create_uniform_return('failed', reason=f'No Python file found for tool: {tool_name}', value=None, resource_id=None)
        
        # Cache Path object (avoid repeated Path() construction)
        if python_file not in self._tool_path_cache:
            self._tool_path_cache[python_file] = Path(python_file)
        python_path = self._tool_path_cache[python_file]
        
        if not python_path.exists():
            return self._create_uniform_return('failed', reason=f'Python file not found: {python_path}', value=None, resource_id=None)
        
        # Cache spec object (avoid repeated spec creation)
        spec_cache_key = (tool_name, python_file)
        if spec_cache_key not in self._tool_spec_cache:
            self._tool_spec_cache[spec_cache_key] = importlib.util.spec_from_file_location(
                f"tool_{tool_name.replace('-', '_')}", 
                python_path
            )
        spec = self._tool_spec_cache[spec_cache_key]
        
        # Create fresh module from spec and execute (MUST be done each call)
        tool_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tool_module)
        
        # Get tool function
        if not hasattr(tool_module, 'tool'):
            return self._create_uniform_return('failed', reason=f'No tool() function in {python_path.name}', value=None, resource_id=None)
        
        tool_func = tool_module.tool
        
        # Resolve any variables in additional args (per-call, cannot be cached)
        resolved_args = {}
        if additional_args:
            for key, val in additional_args.items():
                try:
                    resolved_args[key] = self._resolve_value(val)
                except ValueError as e:
                    # If resolution fails (unbound variable), for 'target' parameter of certain tools,
                    # treat as literal string (strip $ prefix if present)
                    if key == 'target' and isinstance(val, str) and val.startswith('$'):
                        # For tools like mc-map-visualize that accept literal map names,
                        # strip $ and pass as literal string
                        resolved_args[key] = val.lstrip('$')
                        logger.warning(f"Could not resolve {val} for {key}, treating as literal string: {resolved_args[key]}")
                    else:
                        # For other parameters, re-raise the error
                        raise
        
        # Cache base resolved_args template (lines 1168-1201) - only rebuild if config changes
        cache_key = (self.agent_name, self.world_name, id(self.executive_node) if self.executive_node else None)
        if self._tool_base_args_cache is None or self._tool_base_args_cache_key != cache_key:
            # Build base template
            base_args = {}
            base_args['world_name'] = self.world_name
            base_args['agent_name'] = self.agent_name
            base_args['resource_manager'] = self.resource_manager
            base_args['executive_node'] = self.executive_node
            base_args['executor'] = self
            base_args['llm_generate'] = self.llm_generate
            
            # Extract grobid URL and pdf_parser from config (for PDF processing tools)
            if self.executive_node:
                llm_config = self.executive_node.character_config.get('llm_config', {})
                grobid_url = llm_config.get('grobid')
                if grobid_url:
                    base_args['grobid_url'] = grobid_url
                pdf_parser = llm_config.get('pdf_parser')
                if pdf_parser:
                    base_args['pdf_parser'] = pdf_parser
                
                # Extract world_config if available (generalized for any external world API)
                world_config = self.executive_node.character_config.get('world_config', {})
                if world_config:
                    world_name = world_config.get('world_name')
                    port = world_config.get('port')
                    
                    # Override world_name in base_args if world_config provides it
                    if world_name:
                        base_args['world_name'] = world_name
                    
                    # Construct URL from world_name and port if url not explicitly provided
                    if world_config.get('url'):
                        base_args['world_url'] = world_config.get('url')
                    elif world_name and port:
                        base_args['world_url'] = f"http://localhost:{port}"
                    
                    # Pass all other config fields as-is (arbitrary args for world-specific tools)
                    for key, value in world_config.items():
                        if key not in ('world_name', 'port', 'url'):  # Skip URL construction fields
                            base_args[key] = value
            
            # Cache the template
            self._tool_base_args_cache = base_args
            self._tool_base_args_cache_key = cache_key
        
        # Merge cached base args with per-call resolved args (per-call args override base)
        # Don't override if tool already set it in resolved_args
        for key, value in self._tool_base_args_cache.items():
            if key not in resolved_args:
                resolved_args[key] = value
        
        # Use alt_llm_generate for selected tools (uses alt backend when configured, else main)
        if tool_name in ALT_LLM_TOOLS:
            resolved_args['llm_generate'] = self._alt_llm_generate
        
        # Execute tool
        try:
            logger.debug(f"Calling tool '{tool_name}' with input_value type: {type(input_value)}, length: {len(str(input_value)) if isinstance(input_value, str) else 'N/A'}")
            result = tool_func(input_value, **resolved_args)
        except Exception as e:
            logger.error(f"Python tool '{tool_name}' execution error: {e}", exc_info=True)
            return self._create_uniform_return('failed', reason=f'Tool execution error: {str(e)}', value=None, resource_id=None)
        
        # Handle result format - tools should return uniform_return format
        # Check if tool already returned a uniform return dict (has type='uniform_return')
        if isinstance(result, dict) and result.get('type') == 'uniform_return':
            # Tool already used _create_uniform_return - pass through as-is
            return result
        
        # Legacy format handling - convert to uniform_return
        if isinstance(result, dict) and 'status' in result:
            # Tool returned structured response (legacy format)
            status = result.get('status')
            if status == 'failed':
                return self._create_uniform_return('failed', reason=result.get('reason', 'Unknown error'), value=None, resource_id=None)
            else:
                # Check if value is a resource ID
                value = result.get('value')
                resource_id = result.get('resource_id')
                
                # Detect resource ID if not explicitly set
                if resource_id is None and isinstance(value, str):
                    if value.startswith('Note_') or value.startswith('Collection_') or value.startswith('Relation_'):
                        resource_id = value
                
                return self._create_uniform_return('success', value=value, resource_id=resource_id)
        elif isinstance(result, str) and result.startswith('Error:'):
            # Tool returned error string
            error_msg = result
            logger.error(f"Python tool '{tool_name}' failed: {error_msg}")
            return self._create_uniform_return('failed', reason=error_msg, value=None, resource_id=None)
        else:
            # Tool returned raw value
            # Check for null indicator
            if result is None:
                logger.info(f"Python tool '{tool_name}' returned None")
                return self._create_uniform_return('success', value=None, resource_id=None)
            elif isinstance(result, str):
                stripped = result.strip()
                if stripped.lower() == 'note-null' or stripped.lower() == 'null':
                    logger.info(f"Python tool '{tool_name}' returned null indicator")
                    return self._create_uniform_return('success', value=None, resource_id=None)
                # Check if result is a resource ID
                elif stripped.startswith('Note_') or stripped.startswith('Collection_') or stripped.startswith('Relation_'):
                    logger.info(f"Python tool '{tool_name}' returned resource ID: {stripped}")
                    return self._create_uniform_return('success', value=stripped, resource_id=stripped)
            
            logger.info(f"Python tool '{tool_name}' completed")
            return self._create_uniform_return('success', value=result, resource_id=None)

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
            return self._create_uniform_return('failed', reason='create-note requires out')
        
        value = self._resolve_value(value_arg) if value_arg is not None else ''
        
        if value is None:
            self._bind_variable(out_var, "Note_null")
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Created null Note → {display_var} = Note_null")
            return self._create_uniform_return('success', value=None, resource_id="Note_null")
        
        info_id = self._persist_note(value, 'create-note-primitive', extra_props, note_name)
        if info_id:
            self._bind_variable(out_var, info_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Created Note {info_id} → {display_var}")
            # Return truncated content, resource_id is Note ID
            return self._create_uniform_return('success', value=value, resource_id=info_id)
        
        display_var = self._normalize_var_for_log(out_var)
        logger.error(f"Note creation failed for {display_var}")
        return self._create_uniform_return('failed', reason='Failed to create Note')
    
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
            return self._create_uniform_return('failed', reason='create-collection requires out')
        
        if value_arg is None:
            note_ids = []
        elif isinstance(value_arg, list):
            note_ids = []
            for i, item in enumerate(value_arg):
                if isinstance(item, str) and item.startswith('$'):
                    var_name = item[1:]
                    note_id = self._get_binding(var_name)
                    if note_id is None:
                        logger.warning(f"Variable {item} not bound, skipping")
                        continue
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
            bound_value = self._get_binding(var_name)
            if bound_value is None:
                return self._create_uniform_return('failed', reason=f'Variable {value_arg} not bound')
            if isinstance(bound_value, str) and bound_value.startswith('Collection_'):
                # Dereference Collection to get its Note IDs
                note_ids = self._dereference_collection(var_name)
                if not isinstance(note_ids, list):
                    return self._create_uniform_return('failed', reason=f'Failed to dereference Collection {value_arg}')
            elif isinstance(bound_value, str) and bound_value.startswith('Note_'):
                # Single Note - wrap in list
                note_ids = [bound_value]
            elif isinstance(bound_value, list):
                note_ids = bound_value
            else:
                return self._create_uniform_return('failed', reason=f'Variable {value_arg} is not a Note/Collection or list')
        else:
            return self._create_uniform_return('failed', reason='Collection value must be list or $variable')
        
        collection_id = self._create_collection(note_ids, 'create-collection-primitive', collection_name, extra_props)
        if collection_id:
            self._bind_variable(out_var, collection_id)
            name_display = f" '{collection_name}'" if collection_name else ""
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Created {collection_id}{name_display} → {display_var} ({len(note_ids)} items)")
            # Format as "X items [Note_1, ...]"
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        display_var = self._normalize_var_for_log(out_var)
        logger.error(f"Collection creation failed for {display_var}")
        return self._create_uniform_return('failed', reason='Failed to create Collection')

    def _execute_create_relation(self, action: Dict) -> Dict:
        """
        Create a typed directed relation between two resources.

        Required: source/target/relation/out
        Shorthand supported: target as source and value as target.
        """
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')

        out_var = action.get('out')
        source_arg = action.get('source', action.get('target'))
        target_arg = action.get('target_id', action.get('to', action.get('value')))
        relation_type = action.get('relation', action.get('relation_type'))
        relation_name = action.get('name', '')
        extra_props = action.get('properties') if isinstance(action.get('properties'), dict) else None

        if not out_var:
            return self._create_uniform_return('failed', reason='create-relation requires out')
        if not source_arg or not target_arg or not relation_type:
            return self._create_uniform_return('failed', reason='create-relation requires source, target, and relation')

        try:
            source_id = self._resolve_id(source_arg)
            target_id = self._resolve_id(target_arg)
        except ValueError as e:
            return self._create_uniform_return('failed', reason=str(e))

        if not isinstance(source_id, str) or not (source_id.startswith('Note_') or source_id.startswith('Collection_')):
            return self._create_uniform_return('failed', reason=f'Invalid source for relation: {source_id}')
        if not isinstance(target_id, str) or not (target_id.startswith('Note_') or target_id.startswith('Collection_')):
            return self._create_uniform_return('failed', reason=f'Invalid target for relation: {target_id}')

        success, relation_id, error_msg, _ = self.resource_manager.create_relation(
            self.agent_name, source_id, target_id, str(relation_type), relation_name=relation_name, extra_props=extra_props
        )
        if not success or not relation_id:
            return self._create_uniform_return('failed', reason=error_msg or 'Failed to create relation')

        self._bind_variable(out_var, relation_id)
        return self._create_uniform_return('success', value=f"{source_id} -[{relation_type}]-> {target_id}", resource_id=relation_id)

    def _execute_find_relations(self, action: Dict) -> Dict:
        """Find relations by optional source/target/type filters."""
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')

        out_var = action.get('out')
        if not out_var:
            return self._create_uniform_return('failed', reason='find-relations requires out')

        source_arg = action.get('source')
        target_arg = action.get('target')
        relation_type = action.get('relation', action.get('relation_type'))

        try:
            source_id = self._resolve_id(source_arg) if source_arg is not None else None
            target_id = self._resolve_id(target_arg) if target_arg is not None else None
        except ValueError as e:
            return self._create_uniform_return('failed', reason=str(e))

        matches = self.resource_manager.find_relations(source_id=source_id, target_id=target_id, relation_type=relation_type)
        relation_ids = [m.get('id') for m in matches if isinstance(m.get('id'), str)]
        collection_id = self._create_collection(relation_ids, 'find-relations')
        if not collection_id:
            return self._create_uniform_return('failed', reason='Failed to create relation result collection')

        self._bind_variable(out_var, collection_id)
        return self._create_uniform_return('success', value=self._format_collection_value(collection_id), resource_id=collection_id)

    def _execute_related(self, action: Dict) -> Dict:
        """
        Get resources related to a source resource.

        Required: target, out
        Optional: relation (type filter), direction ('out'|'in'|'both', default 'both')
        """
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')

        source_arg = action.get('target')
        out_var = action.get('out')
        relation_type = action.get('relation', action.get('relation_type'))
        direction = str(action.get('direction', 'both')).lower()

        if not source_arg or not out_var:
            return self._create_uniform_return('failed', reason='related requires target and out')

        try:
            source_id = self._resolve_id(source_arg)
        except ValueError as e:
            return self._create_uniform_return('failed', reason=str(e))

        if not isinstance(source_id, str) or not (
            source_id.startswith('Note_') or source_id.startswith('Collection_') or source_id.startswith('Relation_')
        ):
            return self._create_uniform_return('failed', reason=f'Invalid related target: {source_id}')

        related_ids: List[str] = []
        seen = set()

        if direction in ('out', 'both'):
            for rel in self.resource_manager.find_relations(source_id=source_id, relation_type=relation_type):
                target_id = rel.get('properties', {}).get('target')
                if isinstance(target_id, str) and target_id not in seen:
                    seen.add(target_id)
                    related_ids.append(target_id)

        if direction in ('in', 'both'):
            for rel in self.resource_manager.find_relations(target_id=source_id, relation_type=relation_type):
                origin_id = rel.get('properties', {}).get('source')
                if isinstance(origin_id, str) and origin_id not in seen:
                    seen.add(origin_id)
                    related_ids.append(origin_id)

        collection_id = self._create_collection(related_ids, 'related-resources')
        if not collection_id:
            return self._create_uniform_return('failed', reason='Failed to create related result collection')

        self._bind_variable(out_var, collection_id)
        return self._create_uniform_return('success', value=self._format_collection_value(collection_id), resource_id=collection_id)

    def _execute_get_metadata(self, action: Dict) -> Dict:
        """Retrieve metadata attached to a Note or Collection via 'meta' relation.

        Required: target (Note/Collection ID, name, or $variable)
        Optional: out ($variable — bound to the metadata Note ID if present)
        Returns the metadata text as value, or empty string if no metadata exists.
        """
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')
        target_arg = action.get('target')
        if not target_arg:
            return self._create_uniform_return('failed', reason='get-metadata requires target')
        try:
            resource_id = self._resolve_id(target_arg)
        except ValueError as e:
            return self._create_uniform_return('failed', reason=str(e))
        if not isinstance(resource_id, str) or not (
            resource_id.startswith('Note_') or resource_id.startswith('Collection_')
        ):
            return self._create_uniform_return('failed', reason=f'get-metadata target must be a Note or Collection: {resource_id}')
        meta_note_id = self.resource_manager.get_metadata_note_id(resource_id)
        if not meta_note_id:
            out_var = action.get('out')
            if out_var:
                self._bind_variable(out_var, '')
            return self._create_uniform_return('success', value='', resource_id=None)
        meta_resource = self.resource_manager.get_resource(meta_note_id)
        meta_text = (meta_resource or {}).get('properties', {}).get('content', '') if meta_resource else ''
        out_var = action.get('out')
        if out_var:
            self._bind_variable(out_var, meta_note_id)
        return self._create_uniform_return('success', value=meta_text, resource_id=meta_note_id)

    def _execute_set_metadata(self, action: Dict) -> Dict:
        """Attach or update metadata on a Note or Collection via 'meta' relation.

        Required: target (Note/Collection ID, name, or $variable), value (metadata text string)
        Optional: out ($variable — bound to the metadata Note ID)
        The operation is idempotent: updates existing metadata if already present.
        """
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')
        target_arg = action.get('target')
        value_arg = action.get('value')
        if not target_arg:
            return self._create_uniform_return('failed', reason='set-metadata requires target')
        if value_arg is None:
            return self._create_uniform_return('failed', reason='set-metadata requires value')
        try:
            resource_id = self._resolve_id(target_arg)
        except ValueError as e:
            return self._create_uniform_return('failed', reason=str(e))
        if not isinstance(resource_id, str) or not (
            resource_id.startswith('Note_') or resource_id.startswith('Collection_')
        ):
            return self._create_uniform_return('failed', reason=f'set-metadata target must be a Note or Collection: {resource_id}')
        # Resolve value: if it's a variable or resource ref, load its content
        if isinstance(value_arg, str) and value_arg.startswith('$'):
            resolved_val = self._resolve_value(value_arg)
            meta_text = str(resolved_val) if resolved_val is not None else ''
        else:
            meta_text = str(value_arg)
        meta_note_id = self.resource_manager.upsert_metadata(resource_id, meta_text, self.agent_name)
        if not meta_note_id:
            return self._create_uniform_return('failed', reason=f'Failed to set metadata on {resource_id}')
        out_var = action.get('out')
        if out_var:
            self._bind_variable(out_var, meta_note_id)
        return self._create_uniform_return('success', value=meta_text, resource_id=meta_note_id)

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
        Build structured search result Note matching search-web/semantic-scholar format.
        
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
        
        # Note: Engine metadata (source_skill, created_at, note_name) is NOT included in content.
        # Access engine metadata via get_resource_metadata() if needed, not via content['metadata'].
        
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
            value: Content to persist (string, or dict with 'text' for search results)
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
        
        # Search result format: dict with text + metadata -> text-only content, metadata in tool_metadata
        props = dict(properties or {})
        if isinstance(value, dict) and 'text' in value:
            content = value.get('text', '') or ''
            tool_meta = value.get('metadata', {})
            if tool_meta:
                props['tool_metadata'] = {**(props.get('tool_metadata') or {}), **tool_meta}
            format_type = 'text'
        elif isinstance(value, (dict, list)):
            content = json.dumps(value, sort_keys=True)
            format_type = 'text'
        else:
            content = str(value) if value is not None else ""
            format_type = 'text'
        
        success, note_id, error_msg, location = self.resource_manager.create_note(
            self.agent_name, content, format_type, source_context, (content or '')[:100], note_name, props
        )
        
        if success:
            return note_id
        else:
            logger.error(f'Failed to create Note: {error_msg}')
            return None

    def _execute_persist(self, action: Dict) -> Dict:
        """
        Mark a Note or Collection as persistent (saved to filesystem).
        Optional name assigns a stable name for later loading by name.
        
        Required: target
        Optional: name (stable name for load by name, e.g., "berkeley_weather_report")
        
        Target can be:
        - $variable referencing a Note or Collection
        - Literal Note_ID (e.g., "Note_123") or Collection_ID (e.g., "Collection_456")
        - Named resource name (e.g., "minecraft_map", "Jill-minecraft_map")
        """
        target_arg = action.get('target')
        name_arg = action.get('name')
        
        if not target_arg:
            return self._create_uniform_return('failed', reason='persist requires target')
        
        if not isinstance(target_arg, str):
            return self._create_uniform_return('failed', reason='persist target must be string')
        
        # Resolve to resource ID (handles $var, literal IDs, and named resources)
        try:
            resource_id = self._resolve_id(target_arg)
        except ValueError:
            # Variable unbound - try as named resource
            resource_id = None
        
        # If not resolved as variable or literal ID, try as named resource
        if not isinstance(resource_id, str) or not (resource_id.startswith('Collection_') or resource_id.startswith('Note_') or resource_id.startswith('Relation_')):
            if self.resource_manager:
                resolved_id = self.resource_manager._resolve_resource_id(target_arg)
                if resolved_id:
                    resource_id = resolved_id
                else:
                    return self._create_uniform_return('failed', reason=f'Target "{target_arg}" is not a bound variable, resource ID, or named resource')
            else:
                return self._create_uniform_return('failed', reason=f'Target "{target_arg}" is not a bound variable or resource ID, and resource manager not available')
        
        if not isinstance(resource_id, str) or not (resource_id.startswith('Collection_') or resource_id.startswith('Note_') or resource_id.startswith('Relation_')):
            return self._create_uniform_return('failed', reason=f'Target must be a Note, Collection, or Relation ID, got: {resource_id}')
        
        # Mark as persistent using resource manager
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')
        
        success, error_msg = self.resource_manager.mark_persistent(resource_id, self.agent_name)
        
        if not success:
            logger.error(f'Failed to persist resource: {error_msg}')
            return self._create_uniform_return('failed', reason=error_msg or 'Unknown error')
        
        # Optionally assign name for load-by-name
        if name_arg and isinstance(name_arg, str) and name_arg.strip():
            name_success, name_err = self.resource_manager.set_resource_name(resource_id, name_arg.strip())
            if not name_success:
                logger.warning(f"Persist succeeded but name assignment failed: {name_err}")
        
        logger.info(f"Marked {resource_id} as persistent")
        return self._create_uniform_return('success', value=None, resource_id=resource_id)
    
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
    
    # Ceiling constants for load slice
    LOAD_MAX_NOTE_CHARS = 4096
    LOAD_MAX_COLLECTION_ITEMS = 16

    @staticmethod
    def _parse_slice(slice_str: str, length: int) -> tuple:
        """
        Parse Python-style slice string into a slice object.
        Returns (slice, None) on success, (None, error_msg) on validation failure.
        Supports standard Python semantics including negative indices (e.g. "-500:" for last 500).
        Rejects only when both start and stop are non-negative and stop < start.
        """
        s = slice_str.strip()
        if ':' in s:
            parts = s.split(':', 1)
            start = int(parts[0]) if parts[0].strip() else None
            stop = int(parts[1]) if parts[1].strip() else None
            if start is not None and stop is not None and start >= 0 and stop >= 0 and stop < start:
                return (None, f"load slice: stop must be >= start, got {start}:{stop}")
            return (slice(start, stop), None)
        # Single index (supports negative)
        idx = int(s)
        if idx < 0:
            idx = max(0, length + idx)
        return (slice(idx, idx + 1), None)

    def _execute_load(self, action: Dict) -> Dict:
        """
        Load a persistent Note or Collection by resource ID or name.
        
        Required: target
        Optional: out (bind loaded resource to variable; omit to load content into context only),
                  slice (Python-style slice string, e.g. "0:4096", "-1000:", ":")
        
        For Notes: slice units are characters. Default "0:4096".
        For Collections: slice units are items. Default "0:5". With out: returns new Collection; without out: returns content preview.
        """
        target_arg = action.get('target')
        out_var = action.get('out')
        slice_arg = action.get('slice')
        
        if not target_arg:
            return self._create_uniform_return('failed', reason='load requires target')
        
        # Resolve target (handles $var, resource names, and IDs)
        var_name, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'load: {error}')
        
        resource_id = self._get_binding(var_name)
        if resource_id is None:
            return self._create_uniform_return('failed', reason=f'Variable ${var_name} not bound')
        
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')
        
        resource = self.resource_manager.get_resource(resource_id)
        if not resource:
            return self._create_uniform_return('failed', reason=f'Resource not found: {resource_id}')
        
        display_var = self._normalize_var_for_log(out_var) if out_var else "(context)"
        
        # --- Collection path ---
        if resource_id.startswith('Collection_'):
            note_ids = self._get_content(resource_id)
            if not isinstance(note_ids, list):
                note_ids = []
            total = len(note_ids)
            
            # Parse slice (items), default "0:5", ceiling only when no slice
            if slice_arg:
                sl, err = self._parse_slice(str(slice_arg), total)
                if err:
                    return self._create_uniform_return('failed', reason=err)
                sliced_ids = note_ids[sl]
                # No ceiling when slice is explicit (including ":" for full)
            else:
                sl = slice(0, 5)
                sliced_ids = note_ids[sl][:self.LOAD_MAX_COLLECTION_ITEMS]
            
            # Single-item slice: return the Note directly instead of wrapping in a Collection
            if len(sliced_ids) == 1 and slice_arg and ':' not in str(slice_arg).strip():
                note_id = sliced_ids[0]
                content = self._get_content(note_id)
                content_str = str(content) if content is not None else ""
                sliced_content = content_str  # Full content for single-item unwrap
                if out_var:
                    self._bind_variable(out_var, note_id)
                prefixed_content = f"Note Content: {sliced_content}"
                logger.info(f"Loaded {target_arg}[{slice_arg}] → {display_var} = {note_id} (Note, unwrapped from Collection)")
                return self._create_uniform_return('success', value=prefixed_content, resource_id=note_id)
            
            # Build content preview: each item as "Note_ID: first 200 chars..."
            preview_lines = [f"Collection Content ({len(sliced_ids)}/{total} items):"]
            for nid in sliced_ids:
                content = self._get_content(nid)
                preview = str(content)[:200] if content else "(empty)"
                preview_lines.append(f"- {nid}: {preview}")
            prefixed_content = "\n".join(preview_lines)
            
            if out_var:
                collection_id = self._create_collection(sliced_ids, f'load_slice')
                if not collection_id:
                    return self._create_uniform_return('failed', reason='Failed to create Collection from slice')
                self._bind_variable(out_var, collection_id)
                logger.info(f"Loaded {target_arg} → {display_var} = {collection_id} (Collection, {len(sliced_ids)}/{total} items)")
                return self._create_uniform_return('success', value=prefixed_content, resource_id=collection_id,
                                                   extra={"item_count": len(sliced_ids), "total_items": total})
            logger.info(f"Loaded {target_arg} → {display_var} (Collection preview, {len(sliced_ids)}/{total} items)")
            return self._create_uniform_return('success', value=prefixed_content, resource_id=resource_id,
                                               extra={"item_count": len(sliced_ids), "total_items": total})
        
        # --- Note path ---
        content = self._get_content(resource_id)
        content_str = str(content) if content is not None else ""
        total_chars = len(content_str)
        
        # Parse slice (chars), default "0:4096", ceiling LOAD_MAX_NOTE_CHARS only when no slice
        if slice_arg:
            sl, err = self._parse_slice(str(slice_arg), total_chars)
            if err:
                return self._create_uniform_return('failed', reason=err)
            sliced_content = content_str[sl]
            # No ceiling when slice is explicit (including ":" for full content)
        else:
            sl = slice(0, self.LOAD_MAX_NOTE_CHARS)
            sliced_content = content_str[sl][:self.LOAD_MAX_NOTE_CHARS]
        
        if out_var:
            self._bind_variable(out_var, resource_id)
        prefixed_content = f"Note Content: {sliced_content}"
        
        logger.info(f"Loaded {target_arg} → {display_var} = {resource_id} (Note, {len(sliced_content)}/{total_chars} chars)")
        return self._create_uniform_return('success', value=prefixed_content, resource_id=resource_id)
    
    def _execute_index(self, action: Dict) -> Dict:
        """
        Create embeddings index for a Collection (also callable as 'organize').
        
        Index is created using Collection ID as the key. The Collection becomes searchable.
        
        Required: target, index_type (optional)
        
        Argument types:
        - target: $variable (Collection of Notes to index)
        - index_type: literal string ('semantic' or 'keyword', default 'semantic')
        
        The Collection ID is used as the index identifier.
        """
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')
        
        target_arg = action.get('target')
        index_type = action.get('index_type', 'semantic')
        fields = action.get('fields', {})
        
        if not target_arg:
            return self._create_uniform_return('failed', reason='index requires target')
        
        # Target should be a Collection variable or named resource
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'index: {error}')
        
        # Get Collection ID from bindings
        collection_id = self._get_binding(collection_var)
        if collection_id is None:
            logger.warning(f"Collection variable not bound: {collection_var}")
            return self._create_uniform_return('failed', reason=f'Collection variable not bound: {collection_var}')
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return self._create_uniform_return('failed', reason=f'Variable {collection_var} is not a Collection')
        
        # Call resource_manager directly
        success, indexed_count, error_msg = self.resource_manager.index_collection(
            agent_name=self.agent_name,
            collection_id=collection_id,
            index_type=index_type,
            fields=fields
        )
        
        if not success:
            return self._create_uniform_return('failed', reason=error_msg or 'Index failed')
        
        logger.info(f"Indexed {indexed_count} chunks from {collection_id}")
        # Return indexed count as value, Collection ID as resource_id
        return self._create_uniform_return('success', value=str(indexed_count), resource_id=collection_id)
    
    def _execute_search_within_collection(self, action: Dict) -> Dict:
        """
        Search within a specific indexed Collection.
        
        Required: target, query, out
        Optional: mode, limit, threshold
        
        Argument types:
        - target: $variable (indexed Collection to search within)
        - query: literal string OR $variable (resolves to query text)
        - mode: literal string ('semantic' or 'keyword', default 'semantic')
        - limit: int (max results to return, default 5)
        - threshold: float (minimum similarity score, default 0.0)
        - out: literal string (variable name to store results)
        """
        target_arg = action.get('target')
        query = self._resolve_value(action.get('query'))
        mode = action.get('mode', 'semantic')
        limit = action.get('limit', 5)
        threshold = action.get('threshold', 0.0)
        out_var = action.get('out')
        
        if not target_arg or not query or not out_var:
            return self._create_uniform_return('failed', reason='search-within-collection requires target, query, and out')
        
        # Target should be a Collection variable or named resource
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'search-within-collection: {error}')
        
        # Get Collection ID from bindings
        collection_id = self._get_binding(collection_var)
        if collection_id is None:
            return self._create_uniform_return('failed', reason=f'Collection variable not bound: {collection_var}')
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return self._create_uniform_return('failed', reason=f'Variable {collection_var} is not a Collection')
        
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')
        
        # Call resource_manager directly - always return chunks
        success, results, error_msg = self.resource_manager.search_collection(
            agent_name=self.agent_name,
            collection_id=collection_id,
            query=query,
            mode=mode,
            limit=limit,
            threshold=threshold,
            return_mode='chunks'
        )
        
        if not success:
            return self._create_uniform_return('failed', reason=error_msg or 'Search failed')
        
        # Results is already a list of dicts with 'document', 'score', 'metadata' fields
        
        # Create structured Notes for each search result (matching search-web/semantic-scholar format)
        note_ids = []
        for i, result in enumerate(results):
            document_content = result.get('document', '')
            metadata = result.get('metadata', {})
            score = result.get('score', 0.0)
            source_note_id = metadata.get('source_note_id')
            
            # Extract full chunk text (not truncated)
            if isinstance(document_content, (dict, list)):
                chunk_text = json.dumps(document_content)
                format_type = 'json'
            else:
                chunk_text = str(document_content) if document_content else ''
                format_type = 'text'
            
            # Build structured result with metadata in content.metadata (not properties)
            structured_result = {
                'text': chunk_text,
                'format': format_type,
                'metadata': {
                    'source_id': source_note_id if source_note_id else f'chunk_{i}',
                    'uri': source_note_id if source_note_id else f'chunk_{i}',  # URI field for consistency
                    'score': score,
                    'type': 'Note',
                    'chunk_index': metadata.get('chunk_index'),
                    'chunk_total': metadata.get('chunk_total'),
                    'is_complete_note': metadata.get('is_complete_note', False)
                },
                'char_count': len(chunk_text)
            }
            
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
            # Format as "X items [Note_1, ...]"
            collection_value = self._format_collection_value(result_collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=result_collection_id)
        else:
            logger.error(f"Failed to create Collection for search results")
            return self._create_uniform_return('failed', reason='Failed to create result Collection')
    
    def _execute_discover_notes(self, action: Dict) -> Dict:
        """
        Global discovery across all Notes using embedding-based retrieval.
        
        Required: query, out
        Optional: limit, threshold
        
        Argument types:
        - query: literal string OR $variable (resolves to query text)
        - limit: int (max Notes to return, default 5)
        - threshold: float (minimum similarity score, default 0.3)
        - out: literal string (variable name to store results Collection)
        """
        query = self._resolve_value(action.get('query'))
        limit = action.get('limit', 5)
        threshold = action.get('threshold', 0.3)
        out_var = action.get('out')
        
        if not query or not out_var:
            return self._create_uniform_return('failed', reason='discover-notes requires query and out')
        
        # Search for Notes globally
        search_result = self.search_resources([query], k_notes=limit, k_collections=0, threshold=threshold)
        
        if search_result.get('status') != 'success':
            return self._create_uniform_return('failed', reason=search_result.get('reason', 'Search failed'))
        
        notes = search_result.get('notes', [])
        
        if not notes:
            # Return empty Collection for no results
            empty_coll_id = self._create_collection([], 'search_notes_empty')
            if empty_coll_id:
                self._bind_variable(out_var, empty_coll_id)
                collection_value = self._format_collection_value(empty_coll_id)
                return self._create_uniform_return('success', value=collection_value, resource_id=empty_coll_id)
            return self._create_uniform_return('failed', reason='No Notes found and failed to create empty Collection')
        
        # Create structured Notes for each search result (matching search-web/semantic-scholar format)
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
                    'discover-notes',
                    note_name=f'search_result_{note_id}'
                )
                if result_note_id:
                    note_ids.append(result_note_id)
        
        if not note_ids:
            return self._create_uniform_return('failed', reason='No valid Note IDs found in search results')
        
        # Create Collection containing structured result Notes
        collection_id = self._create_collection(note_ids, 'search_notes_results')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Discover-notes found {len(notes)} Notes, created {collection_id} with {len(note_ids)} structured results → {display_var}")
            # Format as "X items [Note_1, ...]"
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        else:
            return self._create_uniform_return('failed', reason='Failed to create result Collection')
    
    def _execute_discover_collections(self, action: Dict) -> Dict:
        """
        Global discovery across all Collections using embedding-based retrieval.
        
        Required: query, out
        Optional: limit, threshold
        
        Argument types:
        - query: literal string OR $variable (resolves to query text)
        - limit: int (max Collections to return, default 3)
        - threshold: float (minimum similarity score, default 0.3)
        - out: literal string (variable name to store results Collection)
        """
        query = self._resolve_value(action.get('query'))
        limit = action.get('limit', 3)
        threshold = action.get('threshold', 0.3)
        out_var = action.get('out')
        
        if not query or not out_var:
            return self._create_uniform_return('failed', reason='discover-collections requires query and out')
        
        # Search for Collections globally
        search_result = self.search_resources([query], k_notes=0, k_collections=limit, threshold=threshold)
        
        if search_result.get('status') != 'success':
            return self._create_uniform_return('failed', reason=search_result.get('reason', 'Search failed'))
        
        collections = search_result.get('collections', [])
        
        if not collections:
            # Return empty Collection for no results
            empty_coll_id = self._create_collection([], 'search_collections_empty')
            if empty_coll_id:
                self._bind_variable(out_var, empty_coll_id)
                collection_value = self._format_collection_value(empty_coll_id)
                return self._create_uniform_return('success', value=collection_value, resource_id=empty_coll_id)
            return self._create_uniform_return('failed', reason='No Collections found and failed to create empty Collection')
        
        # Create structured Notes for each search result (matching search-web/semantic-scholar format)
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
                    'discover-collections',
                    note_name=f'search_result_{coll_id}'
                )
                if result_note_id:
                    note_ids.append(result_note_id)
        
        if not note_ids:
            return self._create_uniform_return('failed', reason='No valid Collection IDs found in search results')
        
        # Create Collection containing structured result Notes
        result_collection_id = self._create_collection(note_ids, 'search_collections_results')
        if result_collection_id:
            self._bind_variable(out_var, result_collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Discover-collections found {len(collections)} Collections, created {result_collection_id} with {len(note_ids)} structured results → {display_var}")
            # Format as "X items [Note_1, ...]"
            collection_value = self._format_collection_value(result_collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=result_collection_id)
        else:
            return self._create_uniform_return('failed', reason='Failed to create result Collection')
    
    def _execute_say(self, action: Dict) -> Dict:
        """
        Produce output (agent speech/communication).
        
        Required: type, value
        Optional: target (default: "user")
        """
        value = self._resolve_value(action.get('value'))
        target = action.get('target', 'user')
        
        if value is None:
            return self._create_uniform_return('failed', reason='say requires value')
        
        # Check for empty or whitespace-only value to prevent blank say actions
        value_str = str(value).strip()
        if not value_str:
            logger.warning(f'Say action skipped - value is empty or whitespace only')
            return self._create_uniform_return('failed', reason='say requires non-empty value')
        
        # Capitalize 'user' to 'User' for character name
        if target.lower() == 'user':
            target = 'User'
        
        # Send message to target's sense_data (mirrors physical space send_text_input)
        content_payload = {'source': self.agent_name, 'text': value_str}
        if action.get('close'):
            content_payload['close'] = True
        sense_data = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps(content_payload)
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
            'text': value_str,
            'source': self.agent_name,
            'target': target,
            'is_text_only': True
        }
        self.executive_node.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        if hasattr(self.executive_node, 'last_say_text'):
            self.executive_node.last_say_text = value_str
        if hasattr(self.executive_node, 'conversation_store') and self.executive_node.conversation_store:
            self.executive_node.conversation_store.record_outgoing(
                target=target,
                text=value_str,
                act_type='say',
                close=bool(action.get('close'))
            )
        
        logger.info(f"Say [{target}]: {value_str}")
        
        # If close flag set, close our own dialog with the target and set cooldown
        if action.get('close') and target.lower() != 'user':
            try:
                if hasattr(self.executive_node, 'conversation_store') and self.executive_node.conversation_store:
                    self.executive_node.conversation_store.close_dialog(target)
                self.executive_node._dialog_cooldowns[target] = time.time()
                self.executive_node._dialog_purposes.pop(target, None)
                logger.info(f"Say [{target}]: dialog closed (close flag)")
            except Exception as e:
                logger.warning(f"Say close_dialog failed for {target}: {e}")
        
        # Return truncated message text, no resource_id
        return self._create_uniform_return('success', value=value_str, resource_id=None)
    
    def _execute_think(self, action: Dict) -> Dict:
        """
        Internal reasoning - appended to planner context only, no Note created.
        
        Required: type, value or target
        """
        # Accept both value and target
        value_arg = action.get('value') or action.get('target')
        if not value_arg:
            return self._create_uniform_return('failed', reason='think requires value or target')
        
        value = self._resolve_value(value_arg)
        
        # Optionally bind to variable if out is specified (for planner visibility)
        out_var = action.get('out')
        resource_id = None
        if out_var:
            # Create a transient Note so the value can be referenced
            thought_note_id = self._persist_note(str(value), 'think-reflection')
            if thought_note_id:
                self._bind_variable(out_var, thought_note_id)
                resource_id = thought_note_id
        
        logger.info(f"Think: {str(value)[:100]}")
        
        # Return truncated thought text, resource_id if Note was created
        return self._create_uniform_return('success', value=str(value), resource_id=resource_id)
    
    def _execute_ask(self, action: Dict) -> Dict:
        """
        Ask target a question and terminate current turn immediately.

        Required: value
        Optional: target (defaults to 'User'), out
        """
        out_var = action.get('out')

        question_text = self._resolve_value(action.get('value'))
        if question_text is None:
            return self._create_uniform_return('failed', reason='ask requires value')
        
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
        if hasattr(self.executive_node, 'conversation_store') and self.executive_node.conversation_store:
            self.executive_node.conversation_store.record_outgoing(
                target=target,
                text=str(question_text),
                act_type='ask',
                close=False
            )
        
        # For non-User targets, also send question to target's sense_data so they actually hear it
        if target != 'User':
            sense_data = {
                'timestamp': datetime.now().isoformat(),
                'sequence_id': 0,
                'mode': 'text',
                'content': json.dumps({'source': self.agent_name, 'text': str(question_text)})
            }
            self.session.put(f"cognitive/{target}/sense_data", json.dumps(sense_data))
        
        if out_var:
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"❓ Ask: '{question_text}' → terminating turn ({display_var}=Note_null)")
            self._bind_variable(out_var, "Note_null")
        else:
            logger.info(f"❓ Ask: '{question_text}' → terminating turn")
        self.executive_node.awaiting_ask_response = False
        # Terminal dialog action model: end this turn after ask.
        self.interrupt_requested = True
        if self.executive_node:
            self.executive_node.interrupt_requested = True
            self.executive_node.execution_mode = 'step'
            self.executive_node.execution_paused = True
            self.executive_node.awaiting_user_input = False
            self.executive_node._publish_execution_state()
        return self._create_uniform_return('success', value=str(question_text), resource_id="Note_null")

    def _execute_bind(self, action: Dict) -> Dict:
        """
        Bind out variable to an existing Note/Collection/Relation resource.

        Required: target, out
        """
        target_arg = action.get('target')
        out_var = action.get('out')
        if not target_arg or not out_var:
            return self._create_uniform_return('failed', reason='bind requires target and out')

        # Resolve to resource ID ($var, literal ID, or named resource)
        try:
            resource_id = self._resolve_id(target_arg)
        except ValueError:
            resource_id = None

        if not isinstance(resource_id, str) or not (resource_id.startswith('Note_') or resource_id.startswith('Collection_') or resource_id.startswith('Relation_')):
            if self.resource_manager:
                resolved_id = self.resource_manager._resolve_resource_id(str(target_arg))
                if resolved_id:
                    resource_id = resolved_id
                else:
                    return self._create_uniform_return('failed', reason=f'bind target "{target_arg}" is not a bound variable, resource ID, or named resource')
            else:
                return self._create_uniform_return('failed', reason='Resource manager not available for name resolution')

        if self.resource_manager and not self.resource_manager.get_resource(resource_id):
            return self._create_uniform_return('failed', reason=f'bind target resource not found: {resource_id}')

        self._bind_variable(out_var, resource_id)
        display_var = self._normalize_var_for_log(out_var)
        logger.info(f"🔗 Bound {display_var} → {resource_id}")
        return self._create_uniform_return('success', value=resource_id, resource_id=resource_id)
    
    # ==================== Phase 2: Data Operations ====================
    
    def _execute_coerce(self, action: Dict) -> Dict:
        """
        Coerce data format or structure.
        
        Required: type, target, coercion (or operation), out
        Optional: delimiter (for to-list)
        """
        target_arg = action.get('target')
        # Accept both 'coercion' (preferred, matches planner docs) and 'operation' (backward compatibility)
        coercion = action.get('coercion') or action.get('operation')
        out_var = action.get('out')
        delimiter = action.get('delimiter', ',')  # Default delimiter for to-list
        
        # Validate required fields (check for None/empty, but allow 0, False, empty string as valid values)
        if target_arg is None or not coercion or not out_var:
            return self._create_uniform_return('failed', reason='coerce requires target, coercion (or operation), and out')
        
        # Resolve target value (may raise ValueError if variable unbound)
        try:
            target = self._resolve_value(target_arg)
        except ValueError as e:
            return self._create_uniform_return('failed', reason=f'coerce: {str(e)}')
        
        # Check if target resolved to None (but allow 0, False, empty string as valid coercion inputs)
        if target is None:
            return self._create_uniform_return('failed', reason='coerce target resolved to None')
        
        result = None
        
        if coercion == 'to-string':
            # Convert any value to string
            result = str(target)
        elif coercion == 'to-int':
            # Convert string/number to integer
            if isinstance(target, (int, float)):
                result = int(target)
            elif isinstance(target, str):
                try:
                    stripped = target.strip()
                    result = int(float(stripped))  # Handle "3.14" -> 3, strip whitespace
                except ValueError:
                    return self._create_uniform_return('failed', reason=f'Cannot convert to int: {target}')
            else:
                return self._create_uniform_return('failed', reason=f'Cannot convert {type(target).__name__} to int')
        elif coercion == 'to-float':
            # Convert string/number to float
            if isinstance(target, (int, float)):
                result = float(target)
            elif isinstance(target, str):
                try:
                    stripped = target.strip()
                    result = float(stripped)  # Strip whitespace before conversion
                except ValueError:
                    return self._create_uniform_return('failed', reason=f'Cannot convert to float: {target}')
            else:
                return self._create_uniform_return('failed', reason=f'Cannot convert {type(target).__name__} to float')
        elif coercion == 'to-bool':
            # Convert string/number to boolean
            if isinstance(target, bool):
                result = target
            elif isinstance(target, str):
                lower = target.lower().strip()
                if lower in ('true', '1', 'yes', 'on'):
                    result = True
                elif lower in ('false', '0', 'no', 'off', ''):
                    result = False
                else:
                    return self._create_uniform_return('failed', reason=f'Cannot convert to bool: {target}')
            elif isinstance(target, (int, float)):
                result = bool(target)
            else:
                result = bool(target)
        elif coercion == 'to-json':
            # Parse JSON string to object
            if isinstance(target, str):
                import json
                try:
                    result = json.loads(target)
                except json.JSONDecodeError as e:
                    return self._create_uniform_return('failed', reason=f'Invalid JSON: {str(e)}')
            elif isinstance(target, (dict, list)):
                result = target  # Already JSON-compatible
            else:
                return self._create_uniform_return('failed', reason=f'Cannot parse JSON from {type(target).__name__}')
        elif coercion == 'to-list':
            # Split string by delimiter, or wrap value in list
            if isinstance(target, str):
                result = [item.strip() for item in target.split(delimiter) if item.strip()]
            elif isinstance(target, list):
                result = target  # Already a list
            else:
                result = [target]  # Wrap single value in list
        elif coercion == 'flatten':
            # Flatten nested lists (backward compatibility)
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
            return self._create_uniform_return('failed', reason=f'Unknown coerce operation: {coercion}')
        
        # Persist coerced result
        info_id = self._persist_note(result, f'coerce_{coercion}')
        if info_id:
            self._bind_variable(out_var, info_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Coerced ({coercion}) → Note {info_id} → {display_var}")
            # Return truncated coerced result, resource_id is Note ID
            return self._create_uniform_return('success', value=result, resource_id=info_id)
        else:
            logger.error(f"Failed to persist coerce result")
            return self._create_uniform_return('failed', reason='Failed to persist result')
    
    def _execute_map(self, action: Dict) -> Dict:
        """
        Apply operation to each item in a Collection.
        
        Required: type, target, operation, out
        Optional: filter_null (bool), tool-specific parameters at top level
        
        operation can be:
        - String: tool name or primitive name (e.g., "tool-name", "add", "remove")
        - Dict: {"tool": "name", ...} where all fields except "tool" are parameters (flat format)
        
        Argument types:
        - target: $variable (Collection to map over)
        - operation: string or dict (string for tool/primitive, dict with "tool" field)
        - Additional fields at top level: tool-specific parameters (instruction, pattern, etc.)
        - filter_null: bool (exclude null/None results, default: true)
        - out: $variable name for result Collection
        """
        # Validate required fields
        error = self._validate_required_fields(action, 'target', 'operation', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        operation = action.get('operation')
        out_var = action.get('out')
        filter_null = action.get('filter_null', True)
        # Extract additional args from top-level (excluding reserved fields)
        reserved_fields = {'type', 'target', 'operation', 'out', 'filter_null', 'expect', 'reason'}
        additional_args = {k: v for k, v in action.items() if k not in reserved_fields}
        
        # Target must be a Collection variable or named resource
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'map: {error}')
        
        # Get Collection Note IDs
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return self._create_uniform_return('failed', reason='map target must be a Collection')
        
        # Blacklist: Primitives that need special handling or don't make sense in map context
        # These are handled explicitly below, not via execute_action()
        MAP_BLACKLIST = {
            # Mutation primitives (mutate target Collection, need special result handling)
            'add', 'remove',
            # Side-effect primitives (don't create result Notes, need special handling)
            'say', 'think',
            # Optimized cases (have inline implementations for performance)
            'project', 'pluck',
            # Collection-only primitives (operate on Collections, not individual Notes)
            'size', 'union', 'intersection', 'difference', 'join', 'filter-structured', 
            'sort', 'head', 'flatten', 'split', 'map',
            # Control flow (don't make sense in map context)
            'if', 'while', 'wait',
            # Search primitives (operate on global/indexed state, not individual Notes)
            'discover-notes', 'discover-collections', 'search-within-collection',
            # Persistence/resource operations (operate on resources, not content)
            'persist', 'load', 'index', 'organize',
            # Create operations (create new resources, not transform existing ones)
            'create-note', 'create-collection',
            # User interaction (doesn't make sense in map)
            'ask', 'bind',
        }
        
        # Apply operation to each Note
        result_note_ids = []
        for i, note_id in enumerate(note_ids):
            # Fetch content for this Note
            content = self._get_content(note_id)
            if content is None and note_id != "Note_null":
                logger.warning(f"Failed to fetch content for {note_id}, skipping")
                continue
            structured_content = content if isinstance(content, dict) else self._parse_note_content_to_dict(content)
            
            # Apply operation based on type
            if isinstance(operation, str):
                # Check if operation is blacklisted (needs special handling)
                if operation in MAP_BLACKLIST:
                    # Special handling for blacklisted primitives
                    if operation in ['add', 'remove']:
                        # Mutation primitives: map item becomes 'value', target comes from args
                        primitive_action = {'type': operation}
                        primitive_action.update(additional_args)
                        primitive_action['value'] = note_id
                        if 'out' not in primitive_action:
                            primitive_action['out'] = additional_args.get('target', out_var)
                        result = self.execute_action(primitive_action)
                    elif operation in ['say', 'think']:
                        # Side-effect primitives: map item becomes 'value'
                        primitive_action = {'type': operation, 'value': note_id}
                        primitive_action.update(additional_args)
                        result = self.execute_action(primitive_action)
                    elif operation == 'project':
                        # Optimized inline implementation
                        fields = additional_args.get('fields', additional_args.get('value', {}).get('fields'))
                        if not fields or not isinstance(fields, list):
                            result = {'status': 'failed', 'reason': 'project requires fields list'}
                        elif not isinstance(structured_content, dict):
                            result = {'status': 'failed', 'reason': f'Cannot project from non-dict Note {note_id}'}
                        else:
                            projected = {}
                            all_fields_found = True
                            for field in fields:
                                value = structured_content
                                parts = field.split('.')
                                for part in parts:
                                    if isinstance(value, dict) and part in value:
                                        value = value[part]
                                    else:
                                        value = None
                                        break
                                if value is None:
                                    all_fields_found = False
                                    break
                                if len(parts) == 1:
                                    projected[parts[0]] = value
                                else:
                                    current = projected
                                    for j, part in enumerate(parts[:-1]):
                                        if part not in current:
                                            current[part] = {}
                                        current = current[part]
                                    current[parts[-1]] = value
                            
                            if all_fields_found and projected:
                                projected_id = self._persist_note(projected, f'project_map_{i}')
                                result = {'status': 'success', 'value': projected_id}
                            else:
                                result = {'status': 'failed', 'reason': 'Fields not found or null'}
                    elif operation == 'pluck':
                        # Optimized inline implementation
                        field = additional_args.get('field', additional_args.get('value', {}).get('field'))
                        if not field:
                            result = {'status': 'failed', 'reason': 'pluck requires field parameter'}
                        elif not isinstance(structured_content, dict):
                            result = {'status': 'failed', 'reason': f'Cannot pluck from non-dict Note {note_id}'}
                        else:
                            value = structured_content
                            for part in field.split('.'):
                                if isinstance(value, dict) and part in value:
                                    value = value[part]
                                else:
                                    value = None
                                    break
                            
                            if value is not None:
                                plucked_id = self._persist_note(value, f'pluck_map_{i}')
                                result = {'status': 'success', 'value': plucked_id}
                            else:
                                result = {'status': 'failed', 'reason': f'Field "{field}" not found'}
                    else:
                        # Other blacklisted primitives (Collection ops, control flow, etc.) - not applicable in map
                        result = {'status': 'failed', 'reason': f'Primitive "{operation}" not applicable in map context'}
                else:
                    # Not blacklisted - try as primitive first
                    # Most primitives expect 'target', but some might expect 'value'
                    # Try with 'target' first (more common for Note operations)
                    primitive_action = {'type': operation, 'target': note_id, 'out': f'$map_temp_{i}'}
                    primitive_action.update(additional_args)
                    # Try to execute as primitive
                    result = self.execute_action(primitive_action)
                    
                    # If primitive execution failed with "unknown type" or similar, try as tool
                    if result.get('status') == 'failed':
                        reason = result.get('reason', '')
                        if 'unknown' in reason.lower() or 'not found' in reason.lower() or 'missing type' in reason.lower() or 'missing' in reason.lower():
                            # Fall back to tool handling
                            result = self._apply_operation_to_value(operation, content, 
                                                                   f"map item {i}", additional_args)
            elif isinstance(operation, dict):
                if 'tool' in operation:
                    # Dict with tool name - check if it's actually a primitive first
                    tool_name = operation['tool']
                    tool_args = {k: v for k, v in operation.items() if k != 'tool'}
                    tool_args.update(additional_args)  # Merge with action-level args
                    
                    # Check if this is a blacklisted primitive (needs special handling)
                    if tool_name in MAP_BLACKLIST:
                        # Handle as blacklisted primitive (same logic as string case)
                        if tool_name in ['add', 'remove']:
                            primitive_action = {'type': tool_name}
                            primitive_action.update(tool_args)
                            primitive_action['value'] = note_id
                            if 'out' not in primitive_action:
                                primitive_action['out'] = tool_args.get('target', out_var)
                            result = self.execute_action(primitive_action)
                        elif tool_name in ['say', 'think']:
                            primitive_action = {'type': tool_name, 'value': note_id}
                            primitive_action.update(tool_args)
                            result = self.execute_action(primitive_action)
                        elif tool_name == 'project':
                            fields = tool_args.get('fields', tool_args.get('value', {}).get('fields'))
                            if not fields or not isinstance(fields, list):
                                result = {'status': 'failed', 'reason': 'project requires fields list'}
                            elif not isinstance(structured_content, dict):
                                result = {'status': 'failed', 'reason': f'Cannot project from non-dict Note {note_id}'}
                            else:
                                projected = {}
                                all_fields_found = True
                                for field in fields:
                                    value = structured_content
                                    parts = field.split('.')
                                    for part in parts:
                                        if isinstance(value, dict) and part in value:
                                            value = value[part]
                                        else:
                                            value = None
                                            break
                                    if value is None:
                                        all_fields_found = False
                                        break
                                    if len(parts) == 1:
                                        projected[parts[0]] = value
                                    else:
                                        current = projected
                                        for j, part in enumerate(parts[:-1]):
                                            if part not in current:
                                                current[part] = {}
                                            current = current[part]
                                        current[parts[-1]] = value
                                
                                if all_fields_found and projected:
                                    projected_id = self._persist_note(projected, f'project_map_{i}')
                                    result = {'status': 'success', 'value': projected_id}
                                else:
                                    result = {'status': 'failed', 'reason': 'Fields not found or null'}
                        elif tool_name == 'pluck':
                            field = tool_args.get('field', tool_args.get('value', {}).get('field'))
                            if not field:
                                result = {'status': 'failed', 'reason': 'pluck requires field parameter'}
                            elif not isinstance(structured_content, dict):
                                result = {'status': 'failed', 'reason': f'Cannot pluck from non-dict Note {note_id}'}
                            else:
                                value = structured_content
                                for part in field.split('.'):
                                    if isinstance(value, dict) and part in value:
                                        value = value[part]
                                    else:
                                        value = None
                                        break
                                
                                if value is not None:
                                    plucked_id = self._persist_note(value, f'pluck_map_{i}')
                                    result = {'status': 'success', 'value': plucked_id}
                                else:
                                    result = {'status': 'failed', 'reason': f'Field "{field}" not found'}
                        else:
                            result = {'status': 'failed', 'reason': f'Primitive "{tool_name}" not applicable in map context'}
                    else:
                        # Not blacklisted - try as primitive first
                        primitive_action = {'type': tool_name, 'target': note_id, 'out': f'$map_temp_{i}'}
                        primitive_action.update(tool_args)
                        result = self.execute_action(primitive_action)
                        
                        # If primitive execution failed with "unknown type" or similar, try as tool
                        if result.get('status') == 'failed':
                            reason = result.get('reason', '')
                            if 'unknown' in reason.lower() or 'not found' in reason.lower() or 'missing type' in reason.lower() or 'missing' in reason.lower():
                                # Fall back to tool handling
                                result = self._apply_operation_to_value(tool_name, content,
                                                                       f"map item {i}", tool_args)
                else:
                    return self._create_uniform_return('failed', reason='operation dict must have "tool" field')
            else:
                return self._create_uniform_return('failed', reason='operation must be string (tool/primitive name) or dict')
            
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
                # For side-effect primitives (say, think): no result Note needed
                elif isinstance(operation, str) and operation in ['say', 'think']:
                    # Side-effect only - don't create result Note, but don't treat as failure
                    pass
                elif result_value is None:
                    if not filter_null:
                        result_note_ids.append("Note_null")
                elif isinstance(result_value, str) and result_value == "Note_null":
                    # Explicit null indicator - exclude if filter_null is True
                    if not filter_null:
                        result_note_ids.append("Note_null")
                elif isinstance(result_value, str) and (result_value.startswith('Note_') or result_value.startswith('Collection_')):
                    # Result is already a Note/Collection ID - use it directly
                    result_note_ids.append(result_value)
                else:
                    # Create Note for result content
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
                mutated_collection_id = self._get_binding(target_var)
                if mutated_collection_id is not None:
                    self._bind_variable(out_var, mutated_collection_id)
                    display_var = self._normalize_var_for_log(out_var)
                    logger.info(f"Mapped {len(note_ids)} items via {operation} → {display_var} = {mutated_collection_id}")
                    collection_value = self._format_collection_value(mutated_collection_id)
                    return self._create_uniform_return('success', value=collection_value, resource_id=mutated_collection_id)
                else:
                    return self._create_uniform_return('failed', reason=f'Mutation target {mutation_target} not bound')
            else:
                return self._create_uniform_return('failed', reason=f'{operation} requires target in args')
        elif isinstance(operation, str) and operation in ['say', 'think']:
            # Side-effect primitives - create empty Collection (side effects executed, no result Notes)
            operation_str = operation if isinstance(operation, str) else 'operation'
            collection_id = self._create_collection([], f'map_{operation_str}')
            if collection_id:
                self._bind_variable(out_var, collection_id)
                display_var = self._normalize_var_for_log(out_var)
                logger.info(f"Mapped {len(note_ids)} items via {operation} (side effects executed), created {collection_id} → {display_var}")
                collection_value = self._format_collection_value(collection_id)
                return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
            else:
                logger.error(f"Failed to create Collection for map results")
                return self._create_uniform_return('failed', reason='Failed to create result Collection')
        else:
            # Tool operation - create result Collection
            operation_str = operation if isinstance(operation, str) else 'operation'
            collection_id = self._create_collection(result_note_ids, f'map_{operation_str}')
            if collection_id:
                self._bind_variable(out_var, collection_id)
                display_var = self._normalize_var_for_log(out_var)
                logger.info(f"Mapped {len(note_ids)} → {len(result_note_ids)} Notes, created {collection_id} → {display_var}")
                collection_value = self._format_collection_value(collection_id)
                return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
            else:
                logger.error(f"Failed to create Collection for map results")
                return self._create_uniform_return('failed', reason='Failed to create result Collection')
    
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
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        out_var = action.get('out')
        separator = action.get('separator', '\n\n')
        
        # Target must be Collection variable or named resource
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'flatten: {error}')
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return self._create_uniform_return('failed', reason='flatten target must be Collection')
        
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
            # Return truncated flattened content, resource_id is Note ID
            return self._create_uniform_return('success', value=flattened, resource_id=info_id)
        else:
            logger.error(f"Failed to persist flatten result")
            return self._create_uniform_return('failed', reason='Failed to persist result')
    
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
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        # Target must be Collection variable or named resource
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'add: {error}')
        
        # Get Collection ID from bindings
        collection_id = self._get_binding(collection_var)
        if collection_id is None:
            return self._create_uniform_return('failed', reason=f'Collection variable not bound: {collection_var}')
        
        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return self._create_uniform_return('failed', reason=f'Variable {collection_var} is not a Collection')
        
        # Resolve value - could be Note, Collection, or literal
        note_ids_to_add = []
        
        if isinstance(value_arg, str) and value_arg.startswith('$'):
            # Variable - check if Note or Collection
            var_name = value_arg[1:]
            value_id = self._get_binding(var_name)
            if value_id is None:
                return self._create_uniform_return('failed', reason=f'Variable not bound: {var_name}')
            
            if isinstance(value_id, str) and value_id.startswith('Collection_'):
                # Collection - get all its Note IDs
                note_ids = self._dereference_collection(var_name)
                if not isinstance(note_ids, list):
                    return self._create_uniform_return('failed', reason=f'Failed to dereference Collection {value_arg}')
                note_ids_to_add = note_ids
            elif isinstance(value_id, str) and value_id.startswith('Note_'):
                # Single Note
                note_ids_to_add = [value_id]
            else:
                return self._create_uniform_return('failed', reason=f'Variable {var_name} is not a Note or Collection')
        elif isinstance(value_arg, str) and value_arg.startswith('Note_'):
            # Direct Note ID
            note_ids_to_add = [value_arg]
        elif isinstance(value_arg, str) and value_arg.startswith('Collection_'):
            # Direct Collection ID (edge case)
            return self._create_uniform_return('failed', reason='Use $variable syntax for Collection references')
        else:
            # Literal - create Note for it
            note_id = self._persist_note(value_arg, 'add_item')
            if not note_id:
                return self._create_uniform_return('failed', reason='Failed to persist value as Note')
            note_ids_to_add = [note_id]
        
        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')
        
        # Get current Collection content for deduplication
        target_collection = self.resource_manager.get_resource(collection_id)
        if not target_collection:
            return self._create_uniform_return('failed', reason=f'Collection {collection_id} not found')
        
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
        
        # Return summary message, resource_id is mutated Collection
        value_msg = f"Added {added_count} items" if added_count > 0 else f"Skipped {skipped_count} duplicates"
        collection_value = self._format_collection_value(collection_id)
        return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
    
    def _execute_split(self, action: Dict) -> Dict:
        """
        Split a Note into a Collection of Notes.
        
        Handles four cases:
        1. Simple JSON array: Directly uses the array (e.g., [1, 2, 3])
        2. JSON object with array field: Extracts array from specified field (default 'results')
        3. JSONL format: Multiple JSON objects, one per line (newlines required between objects)
        4. Plain text: Splits by delimiter (default: 'sentence' for semantic text processing)
        
        Required: target, out
        Optional: 
        - field (default: 'results') - only used for JSON object case
        - delimiter (default: 'sentence') - for plain text: 'sentence', 'paragraph', 'line', or custom string
        
        Argument types:
        - target: $variable (Note containing JSON array, JSON object with array field, JSONL, or plain text)
        - field: literal string (name of array field, default 'results') - only used for JSON object case
        - delimiter: literal string - 'sentence' (default, splits on . ! ? followed by space/newline), 
                     'paragraph' (splits on double newlines), 'line' (splits on single newlines), 
                     or custom delimiter string
        - out: variable name for resulting Collection
        
        Examples:
        - Split simple JSON array: {"type":"split","target":"$json_note","out":"$items"} (where note contains [1,2,3])
        - Split structured data: {"type":"split","target":"$data_note","out":"$items"} (where note contains {"results": [...]})
        - Split JSONL: {"type":"split","target":"$jsonl_note","out":"$items"} (where note contains {"key":"val1"}\n{"key":"val2"})
        - Split text by sentences (default): {"type":"split","target":"$text_note","out":"$sentences"}
        - Split text by paragraphs: {"type":"split","target":"$text_note","delimiter":"paragraph","out":"$paragraphs"}
        - Split text by lines: {"type":"split","target":"$text_note","delimiter":"line","out":"$lines"}
        - Split text by custom delimiter: {"type":"split","target":"$text_note","delimiter":"---","out":"$sections"}
        
        NOTE: 
        - search-web and semantic-scholar return Collections directly - NO split needed.
        - For plain text, default delimiter is 'sentence' which splits on sentence boundaries (. ! ? followed by space/newline)
        - Internal newlines are removed and whitespace is normalized within each segment
        - Empty segments are filtered out
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        field_name = action.get('field', 'results')
        out_var = action.get('out')
        
        # Target must be Note variable or named resource
        note_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'split: {error}')
        
        # Get Note ID from bindings
        note_id = self._get_binding(note_var)
        if note_id is None:
            return self._create_uniform_return('failed', reason=f'Note variable not bound: {note_var}')
        
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            return self._create_uniform_return('failed', reason=f'Variable {note_var} is not a Note')
        
        # Check for null Note
        if note_id == 'Note_null':
            return self._create_uniform_return('failed', reason='Cannot split null Note')
        
        # Get Note content
        content = self._get_content(note_id)
        if content is None:
            return self._create_uniform_return('failed', reason=f'Note {note_id} has no content')
        
        # Check for null content indicator
        if isinstance(content, str) and content.strip().lower() in ['note-null', 'null']:
            return self._create_uniform_return('failed', reason='Cannot split null content')
        
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
                        return self._create_uniform_return('failed', reason=f'Field "{field_name}" exists but is not an array')
                else:
                    # Field not found - check for "text" field as fallback
                    text_fields = self._find_text_fields(content_obj)
                    if text_fields:
                        if len(text_fields) == 1:
                            # Single text field - extract and split
                            text_content = str(text_fields[0])
                            # Default to paragraph splitting for long texts or fetch-text-like output
                            # (JSON objects with "format" or "metadata" fields indicate structured documents)
                            default_delimiter = 'paragraph' if (
                                len(text_content) > 10000 or 
                                isinstance(content_obj, dict) and ('format' in content_obj or 'metadata' in content_obj)
                            ) else 'sentence'
                            delimiter = action.get('delimiter', default_delimiter)
                            array_data = self._split_text_by_delimiter(text_content, delimiter)
                        else:
                            # Multiple text fields - concatenate with separator and split
                            separator = '\n\n---\n\n'
                            combined_text = separator.join(str(t) for t in text_fields)
                            # Default to paragraph splitting for long texts
                            default_delimiter = 'paragraph' if len(combined_text) > 10000 else 'sentence'
                            delimiter = action.get('delimiter', default_delimiter)
                            array_data = self._split_text_by_delimiter(combined_text, delimiter)
                    else:
                        return self._create_uniform_return('failed', reason=f'JSON content missing "{field_name}" array field and no "text" field found')
            else:
                return self._create_uniform_return('failed', reason=f'JSON content must be an array or object with "{field_name}" field')
        
        # If NOT single JSON and array_data not yet set, try JSONL format (multiple JSON objects separated by newlines)
        elif not is_json and array_data is None:
            if isinstance(content, str):
                # Try to detect and parse JSONL format
                jsonl_array = self._detect_and_parse_jsonl(content)
                if jsonl_array is not None:
                    array_data = jsonl_array
                else:
                    # Plain text splitting - use delimiter parameter or default to sentence splitting
                    delimiter = action.get('delimiter', 'sentence')
                    array_data = self._split_text_by_delimiter(content, delimiter)
            else:
                return self._create_uniform_return('failed', reason=f'Note content must be a JSON array (e.g., ["item1", "item2"]), JSON object with "{field_name}" array field, JSONL, or plain text (got {type(content).__name__})')
        
        if not array_data:
            return self._create_uniform_return('failed', reason='No items to split (empty array or no non-empty lines)')
        
        # Create Note for each array element/line
        note_ids = []
        for i, item in enumerate(array_data):
            item_note_id = self._persist_note(item, f'split_item_{i}')
            if item_note_id:
                note_ids.append(item_note_id)
        
        # Create Collection from Notes
        collection_id = self._create_collection(note_ids, 'split_collection')
        if not collection_id:
            return self._create_uniform_return('failed', reason='Failed to create Collection')
        
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
        
        # Format as "X items [Note_1, ...]"
        collection_value = self._format_collection_value(collection_id)
        return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
    
    def _find_text_fields(self, obj: Any, found: List[str] = None) -> List[str]:
        """
        Recursively find all "text" field values in a JSON object.
        
        Args:
            obj: JSON object (dict, list, or primitive)
            found: Accumulator list (internal use)
            
        Returns:
            List of text field values (as strings)
        """
        if found is None:
            found = []
        
        if isinstance(obj, dict):
            # Check top-level "text" field
            if 'text' in obj:
                text_value = obj['text']
                # Convert to string (handles embedded JSON as string)
                found.append(str(text_value))
            # Recursively search nested structures
            for value in obj.values():
                self._find_text_fields(value, found)
        elif isinstance(obj, list):
            # Recursively search list items
            for item in obj:
                self._find_text_fields(item, found)
        
        return found
    
    def _split_text_by_delimiter(self, text: str, delimiter: str) -> List[str]:
        """
        Split text by specified delimiter.
        
        Args:
            text: Text content to split
            delimiter: Split mode - 'sentence' (default), 'paragraph', 'line', or custom string
            
        Returns:
            List of text segments (non-empty, normalized)
        """
        import re
        
        if delimiter == 'sentence':
            # Split on sentence boundaries: . ! ? followed by space or newline
            # Pattern: period/exclamation/question mark followed by space, newline, or end of string
            # Use positive lookahead to split but keep the delimiter
            pattern = r'(?<=[.!?])(?:\s+|\n+|$)'
            parts = re.split(pattern, text)
            segments = []
            for part in parts:
                part = part.strip()
                if not part:
                    continue
                # Normalize internal whitespace (remove internal newlines, collapse multiple spaces)
                part = re.sub(r'\s+', ' ', part)
                if part:
                    segments.append(part)
            return segments
            
        elif delimiter == 'paragraph':
            # Split on double newlines (paragraph breaks)
            paragraphs = re.split(r'\n\n+', text)
            segments = []
            for para in paragraphs:
                para = para.strip()
                # Normalize internal whitespace (remove internal newlines, collapse multiple spaces)
                para = re.sub(r'\s+', ' ', para)
                if para:
                    segments.append(para)
            return segments
            
        elif delimiter == 'line':
            # Split on single newlines (original behavior)
            lines = text.split('\n')
            segments = []
            for line in lines:
                line = line.strip()
                # Normalize internal whitespace (collapse multiple spaces)
                line = re.sub(r'\s+', ' ', line)
                if line:
                    segments.append(line)
            return segments
            
        else:
            # Custom delimiter string
            parts = text.split(delimiter)
            segments = []
            for part in parts:
                part = part.strip()
                # Normalize internal whitespace
                part = re.sub(r'\s+', ' ', part)
                if part:
                    segments.append(part)
            return segments
    
    # ==================== Set Operations ====================
    
    def _execute_size(self, action: Dict) -> Dict:
        """
        Get size (item count) of a Collection.
        
        Required: target, out
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        out_var = action.get('out')
        
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'size: {error}')
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return self._create_uniform_return('failed', reason='size target must be a Collection')
        
        size = len(note_ids)
        info_id = self._persist_note(size, 'size_result')
        if info_id:
            self._bind_variable(out_var, info_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Size of {collection_var}: {size} → {display_var}")
            # Return size as string, resource_id is Note ID containing size
            return self._create_uniform_return('success', value=str(size), resource_id=info_id)
        
        return self._create_uniform_return('failed', reason='Failed to persist size result')
    
    def _execute_union(self, action: Dict) -> Dict:
        """
        Union of two Collections (A ∪ B) - all items from both, deduplicated.
        
        Required: target, value, out
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        target_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'union target: {error}')
        value_var, error = self._resolve_target_var(value_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'union value: {error}')
        
        target_ids = self._dereference_collection(target_var)
        value_ids = self._dereference_collection(value_var)
        
        if not isinstance(target_ids, list) or not isinstance(value_ids, list):
            return self._create_uniform_return('failed', reason='union requires both arguments to be Collections')
        
        # Union: combine and deduplicate
        union_ids = list(dict.fromkeys(target_ids + value_ids))
        
        collection_id = self._create_collection(union_ids, 'union_result')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Union {len(target_ids)} + {len(value_ids)} → {len(union_ids)} → {display_var}")
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create union Collection')
    
    def _execute_intersection(self, action: Dict) -> Dict:
        """
        Intersection of two Collections (A ∩ B) - items in both.
        
        Required: target, value, out
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        target_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'intersection target: {error}')
        value_var, error = self._resolve_target_var(value_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'intersection value: {error}')
        
        target_ids = self._dereference_collection(target_var)
        value_ids = self._dereference_collection(value_var)
        
        if not isinstance(target_ids, list) or not isinstance(value_ids, list):
            return self._create_uniform_return('failed', reason='intersection requires both arguments to be Collections')
        
        # Intersection: items in both
        intersection_ids = [item for item in target_ids if item in value_ids]
        
        collection_id = self._create_collection(intersection_ids, 'intersection_result')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Intersection {len(target_ids)} ∩ {len(value_ids)} → {len(intersection_ids)} → {display_var}")
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create intersection Collection')
    
    def _execute_difference(self, action: Dict) -> Dict:
        """
        Difference of two Collections (A - B) - items in A but not in B.
        
        Required: target, value, out
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')
        
        target_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'difference target: {error}')
        value_var, error = self._resolve_target_var(value_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'difference value: {error}')
        
        target_ids = self._dereference_collection(target_var)
        value_ids = self._dereference_collection(value_var)
        
        if not isinstance(target_ids, list) or not isinstance(value_ids, list):
            return self._create_uniform_return('failed', reason='difference requires both arguments to be Collections')
        
        # Difference: items in target but not in value
        value_set = set(value_ids)
        difference_ids = [item for item in target_ids if item not in value_set]
        
        collection_id = self._create_collection(difference_ids, 'difference_result')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Difference {len(target_ids)} - {len(value_ids)} → {len(difference_ids)} → {display_var}")
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create difference Collection')
    
    def _execute_remove(self, action: Dict) -> Dict:
        """
        Remove a resource or a Note from a Collection.

        Delete a resource (Note or Collection):
            {"type": "remove", "target": "Note_123"}
            {"type": "remove", "target": "Collection_5"}

        Remove a Note from a Collection (out is optional):
            {"type": "remove", "target": "$my_collection", "value": "$my_note"}
        """
        error = self._validate_required_fields(action, 'target')
        if error:
            return self._create_uniform_return('failed', reason=error)

        target_arg = action.get('target')
        value_arg = action.get('value')
        out_var = action.get('out')

        # --- Delete-resource path: target is a bare resource ID, no value ---
        if not value_arg:
            resource_id = self._resolve_id(target_arg)
            if not resource_id:
                resource_id = target_arg
            if not isinstance(resource_id, str) or (
                not resource_id.startswith('Note_') and not resource_id.startswith('Collection_')
            ):
                return self._create_uniform_return('failed', reason=f'remove: target must be a Note or Collection ID, got: {resource_id}')
            if not self.resource_manager:
                return self._create_uniform_return('failed', reason='Resource manager not available')
            success, error_msg = self.delete_resource_and_unbind(resource_id)
            if success:
                logger.info(f"Deleted {resource_id}")
                return self._create_uniform_return('success', value=f'deleted {resource_id}')
            else:
                return self._create_uniform_return('failed', reason=error_msg or f'Failed to delete {resource_id}')

        # --- Remove-from-collection path: target is a Collection, value is the Note ---
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'remove: {error}')

        collection_id = self._get_binding(collection_var)
        if collection_id is None:
            return self._create_uniform_return('failed', reason=f'Collection variable not bound: {collection_var}')

        if not isinstance(collection_id, str) or not collection_id.startswith('Collection_'):
            return self._create_uniform_return('failed', reason=f'Variable {collection_var} is not a Collection')

        note_id = self._resolve_id(value_arg)
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            return self._create_uniform_return('failed', reason=f'Value must be a Note ID, got: {note_id}')

        note_ids = self._dereference_collection(collection_var)
        if note_id not in note_ids:
            logger.warning(f"Note {note_id} not in Collection {collection_id}, nothing to remove")
            if out_var:
                self._bind_variable(out_var, collection_id)
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)

        note_ids.remove(note_id)

        if not self.resource_manager:
            return self._create_uniform_return('failed', reason='Resource manager not available')

        success, item_count, error_msg = self.resource_manager.add_to_collection(
            collection_id, None, self.agent_name, 'update', note_ids
        )

        if success:
            if out_var:
                self._bind_variable(out_var, collection_id)
                display_var = self._normalize_var_for_log(out_var)
                logger.info(f"Removed {note_id} from {collection_id} (now {len(note_ids)} items) → {display_var}")
            else:
                logger.info(f"Removed {note_id} from {collection_id} (now {len(note_ids)} items)")
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        else:
            return self._create_uniform_return('failed', reason=error_msg or 'Remove failed')
    
    def _execute_project(self, action: Dict) -> Dict:
        """
        Extract specific fields from each Note in a Collection (SQL SELECT fields).
        
        Required: target, out
        Optional: fields (list of field paths like "title" or "metadata.year")
        """
        error = self._validate_required_fields(action, 'target', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        out_var = action.get('out')
        fields = action.get('fields')
        
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'project: {error}')
        
        if not fields:
            return self._create_uniform_return('failed', reason='project requires fields list')
        
        if not isinstance(fields, list):
            return self._create_uniform_return('failed', reason='fields must be a list')
        
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return self._create_uniform_return('failed', reason='project target must be a Collection')
        
        # Project each Note
        projected_ids = []
        for note_id in note_ids:
            content = self._parse_note_content_to_dict(self._get_content(note_id))
            if content is None:
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
            # Format as "X items [Note_1, ...]"
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create projected Collection')
    
    def _execute_head(self, action: Dict) -> Dict:
        """Deprecated: routes to load with slice. Use load(target, slice="0:N") instead."""
        count = action.get('count', 1)
        load_action = {"type": "load", "target": action.get('target'), "slice": f"0:{count}", "out": action.get('out')}
        return self._execute_load(load_action)

    def _execute_pluck(self, action: Dict) -> Dict:
        """
        Extract a single field value from each Note in a Collection.
        Returns Collection of Notes, each containing the extracted scalar value.
        
        Required: target, field, out
        Example: Extract URLs from structured Notes for use with tools
        """
        error = self._validate_required_fields(action, 'target', 'field', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        field = action.get('field')
        out_var = action.get('out')
        
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'pluck: {error}')
        
        if not field:
            return self._create_uniform_return('failed', reason='pluck requires field parameter')
        
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return self._create_uniform_return('failed', reason='pluck target must be a Collection')
        
        # Extract field value from each Note
        plucked_ids = []
        for note_id in note_ids:
            content = self._parse_note_content_to_dict(self._get_content(note_id))
            if content is None:
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
            # Format as "X items [Note_1, ...]"
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create plucked Collection')
    
    def _execute_filter_structured(self, action: Dict) -> Dict:
        """
        Filter Collection based on JSON field predicates (SQL WHERE - deterministic).
        
        Required: target, where, out
        Example where: "year > 2020" or "citations >= 100 AND year < 2025"
        """
        error = self._validate_required_fields(action, 'target', 'where', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        where_clause = action.get('where')
        out_var = action.get('out')
        
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'filter-structured: {error}')
        
        if not where_clause:
            return self._create_uniform_return('failed', reason='filter-structured requires where clause')
        
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return self._create_uniform_return('failed', reason='filter-structured target must be a Collection')
        
        # Parse where clause (simple expression evaluator)
        def eval_predicate(content: dict, predicate: str) -> bool:
            """Safely evaluate predicate on JSON content."""
            # Replace field references with dict access
            # Support: field > value, field < value, field == value, field >= value, field <= value
            # Support AND, OR operators
            
            # Replace operators - use placeholder approach to avoid breaking multi-char operators
            # First replace multi-char operators with placeholders, then single-char, then restore
            temp = predicate.replace('>=', '__GE__').replace('<=', '__LE__').replace('==', '__EQ__').replace('!=', '__NE__')
            temp = temp.replace('>', ' > ').replace('<', ' < ')
            temp = temp.replace('__GE__', ' >= ').replace('__LE__', ' <= ').replace('__EQ__', ' == ').replace('__NE__', ' != ')
            tokens = temp.split()
            
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
            content = self._parse_note_content_to_dict(self._get_content(note_id))
            if content is None:
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
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create filtered Collection')
    
    def _execute_sort(self, action: Dict) -> Dict:
        """
        Sort Collection by field value (SQL ORDER BY).
        
        Required: target, by, out
        Optional: order ("asc" or "desc", default "asc")
        """
        error = self._validate_required_fields(action, 'target', 'by', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        target_arg = action.get('target')
        sort_field = action.get('by')
        out_var = action.get('out')
        order = action.get('order', 'asc')
        
        collection_var, error = self._resolve_target_var(target_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'sort: {error}')
        
        if not sort_field:
            return self._create_uniform_return('failed', reason='sort requires by field')
        
        if order not in ['asc', 'desc']:
            return self._create_uniform_return('failed', reason='sort order must be "asc" or "desc"')
        
        note_ids = self._dereference_collection(collection_var)
        
        if not isinstance(note_ids, list):
            return self._create_uniform_return('failed', reason='sort target must be a Collection')
        
        # Extract sort keys
        items_with_keys = []
        for note_id in note_ids:
            content = self._parse_note_content_to_dict(self._get_content(note_id))
            if content is None:
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
            return self._create_uniform_return('failed', reason=f'Sort failed: {e}')
        
        # Create result Collection
        collection_id = self._create_collection(sorted_ids, f'sorted_{collection_var}')
        if collection_id:
            self._bind_variable(out_var, collection_id)
            display_var = self._normalize_var_for_log(out_var)
            logger.info(f"Sorted {len(sorted_ids)} items by {sort_field} ({order}) → {display_var}")
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create sorted Collection')
    
    def _execute_join(self, action: Dict) -> Dict:
        """
        Join two Collections on a key field (SQL JOIN).
        
        Required: target (left), value (right), out
        Args: on (join key), type (inner/left/right/outer), left_key, right_key
        """
        error = self._validate_required_fields(action, 'target', 'value', 'out')
        if error:
            return self._create_uniform_return('failed', reason=error)
        
        left_arg = action.get('target')
        right_arg = action.get('value')
        out_var = action.get('out')
        
        join_key = action.get('on')
        join_type = action.get('join_type', 'inner')
        left_key = action.get('left_key', join_key)
        right_key = action.get('right_key', join_key)
        
        left_var, error = self._resolve_target_var(left_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'join target: {error}')
        
        right_var, error = self._resolve_target_var(right_arg)
        if error:
            return self._create_uniform_return('failed', reason=f'join value: {error}')
        
        if not join_key and not (left_key and right_key):
            return self._create_uniform_return('failed', reason='join requires on key or left_key/right_key')
        
        if join_type not in ['inner', 'left', 'right', 'outer']:
            return self._create_uniform_return('failed', reason='join type must be inner/left/right/outer')
        
        left_ids = self._dereference_collection(left_var)
        right_ids = self._dereference_collection(right_var)
        
        if not isinstance(left_ids, list) or not isinstance(right_ids, list):
            return self._create_uniform_return('failed', reason='join requires two Collections')
        
        # Build right index
        right_index = {}
        for note_id in right_ids:
            content = self._parse_note_content_to_dict(self._get_content(note_id))
            if content is None:
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
            left_content = self._parse_note_content_to_dict(self._get_content(left_note_id))
            if left_content is None:
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
            collection_value = self._format_collection_value(collection_id)
            return self._create_uniform_return('success', value=collection_value, resource_id=collection_id)
        
        return self._create_uniform_return('failed', reason='Failed to create joined Collection')
    
    # ==================== Operation Application Helpers ====================
    
    def _apply_operation_to_value(self, tool_name: str, value: Any, reason: str = '', 
                                   additional_args: Dict = None) -> Dict:
        """
        Helper to apply a tool to a single value with optional additional arguments.
        Shared by map and coerce primitives.
        
        Args:
            tool_name: Name of the tool to invoke
            value: Input value to the tool (Note content, passed as positional argument)
            reason: Reason for invoking (for logging)
            additional_args: Optional dict of additional arguments for the tool
            
        Returns:
            Result dict with 'status' and 'value' fields
        """
        # Get tool info and route by type
        tool_info = self._get_tool_info(tool_name)
        
        if not tool_info:
            return {'status': 'failed', 'reason': f'Tool not found: {tool_name}'}
        
        # Mapping of tool names to their expected parameter names for the 'value' field in map actions.
        # When map has 'value' field, it maps to these tool-specific parameters.
        # If the tool-specific parameter is already present, it takes precedence (override).
        TOOL_VALUE_PARAM_MAP = {
            'extract': 'instruction',    # extract expects 'instruction' (required)
            'synthesize': 'focus',       # synthesize expects 'focus' (optional)
            'generate-note': 'prompt',   # generate-note expects 'prompt' (required)
            'text-find': 'pattern',      # text-find expects 'pattern' (required)
            'matches': 'pattern',         # matches expects 'pattern' (required)
            'filter-semantic': 'predicate',  # filter-semantic expects 'predicate' (required)
            'assess': 'predicate',       # assess expects 'predicate' (required)
        }
        
        # Clean up additional_args: remove 'value' if present (conflicts with positional value argument)
        # Map 'value' -> tool-specific parameter if tool has a mapping and parameter not already present
        cleaned_args = {}
        if additional_args:
            for key, val in additional_args.items():
                if key == 'value':
                    # Check if this tool has a parameter mapping
                    mapped_param = TOOL_VALUE_PARAM_MAP.get(tool_name)
                    if mapped_param:
                        # Only map 'value' -> mapped_param if mapped_param is not already present
                        # This allows planner to explicitly provide the parameter, which takes precedence
                        if mapped_param not in additional_args:
                            cleaned_args[mapped_param] = val
                        # 'value' is skipped (already passed as positional argument)
                    # For tools without mapping, skip 'value' (already passed as positional argument)
                    # Note: tools should not expect 'value' as kwarg since it's positional
                else:
                    cleaned_args[key] = val
        
        tool_type = tool_info.get('type')
        
        if tool_type == 'prompt_augmentation':
            # Execute locally using LLM
            result = self._execute_prompt_tool(tool_name, value, tool_info, cleaned_args)
        elif tool_type == 'python':
            # Execute Python tool (returns original_value, not truncated)
            result = self._execute_python_tool(tool_name, value, tool_info, cleaned_args)
        else:
            return {'status': 'failed', 'reason': f'Unknown tool type: {tool_type}'}
        
        # Convert to uniform format with truncated value for map/coerce (no Note creation here)
        if result.get('status') != 'success':
            return self._create_uniform_return('failed', reason=result.get('reason', 'Unknown error'))
        
        original_value = result.get('original_value')
        resource_id = result.get('resource_id')
        return self._create_uniform_return('success', value=original_value, resource_id=resource_id)
    

    
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
        target_id = self._get_binding(target_var)
        if target_id is not None:
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
        target_id = self._get_binding(target_var)
        if target_id is not None:
            if isinstance(target_id, str) and target_id.startswith('Collection_'):
                # Collection membership check - resolve value to Note ID
                note_ids = self._dereference_collection(target_var)
                if not note_ids:
                    return False
                
                # Resolve value to Note ID
                if isinstance(value_arg, str) and value_arg.startswith('$'):
                    value_var = value_arg[1:]
                    value_id = self._get_binding(value_var)
                    if value_id is not None:
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
            ...  # Additional tool parameters at top level (flat format)
        }
        """
        tool_name = condition.get('tool')
        if not tool_name:
            logger.error("tool_condition requires 'tool' field")
            return False
        
        target = self._resolve_value(condition.get('target'))
        # All fields except 'tool' and 'target' are parameters (flat format)
        tool_args = {k: v for k, v in condition.items() if k not in ('tool', 'target', 'type')}
        
        # Invoke tool with target value
        result = self._apply_operation_to_value(
            tool_name=tool_name,
            value=target,
            reason='condition evaluation',
            additional_args=tool_args
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
        collection_id = self._get_binding(collection_var)
        if collection_id is None:
            logger.warning(f"Collection variable not bound: {collection_var}")
            return []
        
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
    
    def get_last_plan_action(self, action_type: Optional[str] = None) -> Optional[Dict]:
        """
        Get the last executed action from the current plan's action list.
        
        This is more reliable than parsing JSON from subplanner text responses,
        as it returns the actual action dict that was executed.
        
        Args:
            action_type: Optional filter to return only actions of this type.
                        If None, returns the last action regardless of type.
        
        Returns:
            Last action dict from _plan_actions (optionally filtered by type),
            or None if no matching actions executed.
        """
        plan_actions = getattr(self, '_plan_actions', [])
        if not plan_actions:
            return None
        
        if action_type:
            # Return last action of specified type
            for action in reversed(plan_actions):
                if isinstance(action, dict) and action.get('type') == action_type:
                    return action
            return None
        else:
            # Return last action regardless of type
            return plan_actions[-1]
    
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
        
        # If $variable, lookup in bindings (searching stack) and return ID (no content fetch)
        if value.startswith('$'):
            var_name = value[1:]
            resource_id = self._get_binding(var_name)
            if resource_id is None:
                error_msg = f"Unbound variable: {value}"
                logger.error(error_msg)
                raise ValueError(error_msg)
            
            # Return ID directly (no _get_content call)
            return resource_id
        
        # If starts with Note_ or Collection_, validate and return as-is (literal ID)
        if value.startswith('Note_') or value.startswith('Collection_') or value.startswith('Relation_'):
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
        
        # If no variable patterns found, check if it's a literal Note/Collection ID or named resource
        if not matches:
            # If literal ID format, fetch its content
            if value.startswith('Note_') or value.startswith('Collection_') or value.startswith('Relation_'):
                content = self._get_content(value)
                if content is not None:
                    return content
                logger.warning(f"Could not fetch content for literal ID {value}, returning as-is")
            return value
        
        # If entire string is a single variable reference (starts with $)
        if value.startswith('$') and len(matches) == 1 and value == f'${matches[0]}':
            var_name = matches[0]
            resource_id = self._get_binding(var_name)
            if resource_id is None:
                # Fallthrough: treat as literal string, strip $ prefix
                logger.warning(f"Unbound variable {value}, treating as literal (stripping $)")
                return var_name
            
            # If it's a resource ID, fetch content from resource manager
            if isinstance(resource_id, str) and (resource_id.startswith('Note_') or resource_id.startswith('Collection_') or resource_id.startswith('Relation_')):
                return self._get_content(resource_id)
            
            # Otherwise return as-is
            return resource_id
        
        # Template string with embedded variables - substitute each
        result = value
        for var_name in set(matches):  # Use set to process each unique variable once
            var_ref = f'${var_name}'
            resource_id = self._get_binding(var_name)
            if resource_id is not None:
                logger.debug(f"Resolving template variable {var_ref} → {resource_id}")
                
                # If it's a resource ID, fetch content from resource manager
                if isinstance(resource_id, str) and (resource_id.startswith('Note_') or resource_id.startswith('Collection_') or resource_id.startswith('Relation_')):
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
        
        value = self._get_binding(var_name)
        if value is None:
            return 'Note'  # Default if not bound
        # Check if value is a Collection_N ID
        if isinstance(value, str) and value.startswith('Collection_'):
            return 'Collection'
        return 'Note'
    
    def _bind_variable(self, var_name: str, resource_id: str):
        """Bind variable name to resource ID in current scope (top of stack)"""
        if var_name.startswith('$'):
            var_name = var_name[1:]
        
        # Bind to top scope (most recent)
        if not self.plan_bindings:
            self.plan_bindings = [{}]
        self.plan_bindings[-1][var_name] = resource_id
        logger.debug(f"Bound ${var_name} → {resource_id} (scope depth: {len(self.plan_bindings)})")
        
        # Track last out resource for plan_result
        if self.executive_node:
            self.executive_node.last_out_resource_id = resource_id

    def _remove_bindings_for_resource(self, resource_id: str) -> int:
        """Remove all binding variables currently pointing to resource_id."""
        if not resource_id:
            return 0
        removed = 0
        for scope in self.plan_bindings:
            to_remove = [k for k, v in scope.items() if v == resource_id]
            for key in to_remove:
                del scope[key]
                removed += 1
        if removed:
            logger.debug(f"Removed {removed} bindings for deleted resource {resource_id}")
        return removed

    def delete_resource_and_unbind(self, resource_id: str) -> Tuple[bool, Optional[str]]:
        """Delete resource and clear any bound variables that reference it."""
        success, error_msg = self.resource_manager.delete_resource(resource_id)
        if success:
            self._remove_bindings_for_resource(resource_id)
        return success, error_msg
    
    def push_binding_scope(self, copy_outer: bool = True):
        """
        Push a new binding scope onto the stack.
        
        Args:
            copy_outer: If True, copy outer scope bindings for read access.
                       If False, start with empty scope.
        """
        if copy_outer and self.plan_bindings:
            # Copy outer scope for read access
            new_scope = self.plan_bindings[-1].copy()
        else:
            new_scope = {}
        self.plan_bindings.append(new_scope)
        logger.debug(f"Pushed binding scope (copy_outer={copy_outer}), stack depth: {len(self.plan_bindings)}")
    
    def pop_binding_scope(self) -> Dict[str, str]:
        """
        Pop the current binding scope from the stack.
        
        Returns:
            The popped scope dict (for inspection/debugging)
        """
        if len(self.plan_bindings) <= 1:
            logger.warning("Attempted to pop binding scope when only one scope exists")
            return {}
        
        popped = self.plan_bindings.pop()
        logger.debug(f"Popped binding scope, stack depth: {len(self.plan_bindings)}")
        return popped
    
    def _get_binding(self, var_name: str) -> Optional[str]:
        """
        Get binding for variable name, searching from top scope to bottom.
        
        Args:
            var_name: Variable name without $ prefix
            
        Returns:
            Resource ID if found, None otherwise
        """
        # Search from top (most recent) to bottom (outermost)
        for scope in reversed(self.plan_bindings):
            if var_name in scope:
                return scope[var_name]
        return None
    
    def _has_binding(self, var_name: str) -> bool:
        """Check if variable is bound in any scope."""
        return self._get_binding(var_name) is not None
    
    @property
    def plan_bindings_flat(self) -> Dict[str, str]:
        """
        Backward compatibility: return merged view of all scopes (top scope wins).
        Use for read-only access like .get() calls.
        """
        merged = {}
        # Merge from bottom to top (top wins on conflicts)
        for scope in self.plan_bindings:
            merged.update(scope)
        return merged
    
    def _resolve_target_var(self, target_arg: str) -> tuple:
        """
        Resolve target argument to variable name, binding named resources if needed.
        
        Args:
            target_arg: Either "$variable", resource ID (e.g., "Note_21"), or named resource name
            
        Returns:
            (var_name, error) - var_name is the variable name (without $), error is None on success
        """
        if not isinstance(target_arg, str):
            return None, 'target must be string'
        
        # Extract variable name if $ prefix present
        if target_arg.startswith('$'):
            var_name = target_arg[1:]
            # Check if variable is already bound
            if self._has_binding(var_name):
                return var_name, None
            # Not bound - try resolving as resource name (fallback)
            target_arg = var_name
        
        # Try to resolve as named resource (handles both resource IDs and names)
        if self.resource_manager:
            # Use _resolve_resource_id to get the actual resource ID
            resolved_id = self.resource_manager._resolve_resource_id(target_arg)
            if resolved_id:
                # Verify resource exists
                resource = self.resource_manager.get_resource(resolved_id)
                if resource:
                    self._bind_variable(target_arg, resolved_id)
                    return target_arg, None
        
        return None, f'"{target_arg}" is not a bound variable or named resource'
    
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
                        'continue': continue_wait,
                        'last_action_result': last_action_result if 'last_action_result' in locals() else None
                    }
            
            # Execute action
            try:
                result = self.execute_action(step)
                executed_steps += 1
                
                # Track last action result in uniform format
                last_action_result = result.copy()
                
                # Publish action result for UI display (if executive_node available)
                # Skip say/ask - they self-publish
                if self.executive_node and stype not in ('say', 'ask'):
                    self.executive_node._publish_action_result(step, result, stype, datetime.now())
                
                # Handle while loop completion (checked in frame completion logic above)
                
                # Advance step index
                if result.get('status') == 'failed':
                    # Action failed - log but continue
                    logger.warning(f"Step {executed_steps} failed: {result.get('reason')}")

                current['idx'] = idx + 1

                # Propagate interrupt (e.g. ask fired inside a codeblock)
                if self.interrupt_requested:
                    return {
                        'status': 'suspended',
                        'reason': 'interrupted',
                        'executed_steps': executed_steps,
                        'last_action_result': last_action_result
                    }
                
            except Exception as e:
                logger.error(f"Error executing step {executed_steps}: {e}")
                logger.error(traceback.format_exc())
                return {
                    'status': 'failed',
                    'reason': f'Execution error at step {executed_steps}: {str(e)}',
                    'executed_steps': executed_steps,
                    'last_action_result': last_action_result if 'last_action_result' in locals() else None
                }
        
        # Execution complete
        if executed_steps >= max_steps:
            return {
                'status': 'failed',
                'reason': f'Max steps ({max_steps}) exceeded',
                'executed_steps': executed_steps,
                'last_action_result': last_action_result if 'last_action_result' in locals() else None
            }
        
        return {
            'status': 'success',
            'executed_steps': executed_steps,
            'bindings': self.plan_bindings.copy(),  # Return final bindings for output extraction
            'last_action_result': last_action_result if 'last_action_result' in locals() else None  # Last action result in uniform format
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
                        'continue': continue_wait,
                        'last_action_result': last_action_result if 'last_action_result' in locals() else None
                    }
            
            # Execute action
            try:
                result = self.execute_action(step)
                executed_steps += 1
                
                # Track last action result in uniform format
                last_action_result = result.copy()
                
                # Publish action result for UI display (if executive_node available)
                # Skip say/ask - they self-publish
                if self.executive_node and stype not in ('say', 'ask'):
                    self.executive_node._publish_action_result(step, result, stype, datetime.now())
                
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
                    'executed_steps': executed_steps,
                    'last_action_result': last_action_result if 'last_action_result' in locals() else None
                }
        
        if executed_steps >= max_steps:
            return {
                'status': 'failed',
                'reason': f'Max steps ({max_steps}) exceeded',
                'executed_steps': executed_steps,
                'last_action_result': last_action_result if 'last_action_result' in locals() else None
            }
        
        return {
            'status': 'success',
            'executed_steps': executed_steps,
            'bindings': self.plan_bindings.copy(),  # Return final bindings for output extraction
            'last_action_result': last_action_result if 'last_action_result' in locals() else None
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
                'out': '$test_output',
                'expect': 'test output'
            },
            'create': {
                'type': 'create',
                'kind': 'Note',
                'value': 'test note content',
                'out': '$test_note',
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
                'out': '$test_results',
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
                'out': '$test_extracted',
                'expect': 'test extract'
            },
            'filter': {
                'type': 'filter',
                'target': [{'val': 1}, {'val': 2}, {'val': 3}],
                'condition': {'field': 'val', 'operator': 'gt', 'value': 0},
                'out': '$test_filtered',
                'expect': 'test filter'
            },
            'map': {
                'type': 'map',
                'target': '$test_filtered',
                'operation': {'type': 'extract', 'field': 'val'},
                'out': '$test_mapped',
                'expect': 'test map'
            },
            'merge': {
                'type': 'merge',
                'targets': [{'a': 1}, {'b': 2}],
                'out': '$test_merged',
                'expect': 'test merge'
            },
            'coerce': {
                'type': 'coerce',
                'target': [[1, 2], [3, 4]],
                'operation': 'flatten',
                'out': '$test_coerced',
                'expect': 'test coerce'
            },
            'aggregate': {
                'type': 'aggregate',
                'target': [1, 2, 3],
                'operation': 'count',
                'out': '$test_aggregated',
                'expect': 'test aggregate'
            },
            'sort': {
                'type': 'sort',
                'target': [{'val': 3}, {'val': 1}, {'val': 2}],
                'by': 'val',
                'order': 'asc',
                'out': '$test_sorted',
                'expect': 'test sort'
            },
            'group_by': {
                'type': 'group_by',
                'target': [{'type': 'a'}, {'type': 'b'}],
                'by': 'type',
                'out': '$test_grouped',
                'expect': 'test group'
            },
            'compare': {
                'type': 'compare',
                'targets': ['test', 'test'],
                'out': '$test_compared',
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
        self.plan_bindings = saved_bindings if isinstance(saved_bindings, list) else [{}]
        
        logger.info("🧪 ================================================")
        
        return results

