"""
Incremental Planner - SGLang-based iterative planning for infospace goals.

Uses SGLang's function decorator for multi-stage planning with tool execution feedback.

Note: SGLang backend must be configured before use:
    import sglang as sgl
    sgl.set_default_backend(sgl.Runtime(model_path="...", ...))
"""
import json
import logging
import traceback
import time
import re
import os
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional

# Configure logging with file handler
# Add file handler directly to this module's logger (doesn't interfere with root logger config)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Only add handlers if they don't already exist (to avoid duplicates on re-import)
if not logger.handlers:
    # File handler - ensure logs directory exists
    try:
        _log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        os.makedirs(_log_dir, exist_ok=True)
        _log_path = os.path.join(_log_dir, 'incremental_planner.log')
        file_handler = logging.FileHandler(_log_path, mode='w')
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            '%Y-%m-%d %H:%M:%S'
        ))
        logger.addHandler(file_handler)
    except Exception:
        # Fall back to console-only if file handler setup fails
        pass

# Setup OpenTelemetry tracing
try:
    from opentelemetry import trace
    from opentelemetry.sdk.trace import TracerProvider
    from opentelemetry.sdk.trace.export import ConsoleSpanExporter, SimpleSpanProcessor
    HAS_OTEL = True
except ImportError:
    HAS_OTEL = False
    logger.info("opentelemetry not available, tracing disabled")

def setup_tracing(service_name: str):
    """Setup OpenTelemetry tracing with console exporter."""
    if not HAS_OTEL:
        return None
    provider = TracerProvider()
    processor = SimpleSpanProcessor(ConsoleSpanExporter())
    provider.add_span_processor(processor)
    trace.set_tracer_provider(provider)
    return trace.get_tracer(service_name)

def tokenize_len(tokenizer, text: str) -> int:
    """Count tokens in text using tokenizer."""
    if tokenizer and text:
        return len(tokenizer.encode(text))
    return 0

class GenTracer:
    """Context manager for tracing gen() calls with input/output metrics."""
    def __init__(self, tracer, tokenizer=None):
        self.tracer = tracer
        self.tokenizer = tokenizer
    
    def span_gen(self, name: str, stage: str, step: int, input_delta: str):
        return _GenSpan(self.tracer, self.tokenizer, name, stage, step, input_delta)

class _GenSpan:
    def __init__(self, tracer, tokenizer, name: str, stage: str, step: int, input_delta: str):
        self.tracer = tracer
        self.tokenizer = tokenizer
        self.name = name
        self.stage = stage
        self.step = step
        self.input_delta = input_delta
        self._span = None
        self._t0 = None
        # slots to fill by caller before exit
        self.state = None
        self.slot = None

    def __enter__(self):
        self._span = self.tracer.start_span(
            f"gen:{self.name}",
            attributes={
                "stage": self.stage,
                "step": self.step,
                "input.delta.len": len(self.input_delta),
                "input.delta.preview": self.input_delta[:2000],
            },
        )
        self._t0 = time.time()
        if self.tokenizer:
            self._span.set_attribute("input.delta.tok", tokenize_len(self.tokenizer, self.input_delta))
        return self

    def __exit__(self, exc_type, exc, tb):
        dur_ms = (time.time() - self._t0) * 1000.0
        self._span.set_attribute("duration.ms", dur_ms)

        if self.state is not None and self.slot:
            try:
                out = self.state[self.slot]
            except (KeyError, TypeError):
                out = ""
            self._span.set_attribute("output.len", len(out))
            self._span.add_event("output.text", {"text": out[:4000]})
            if self.tokenizer:
                self._span.set_attribute("output.tok", tokenize_len(self.tokenizer, out))

        if exc:
            self._span.record_exception(exc)
        self._span.end()
        # don't suppress exceptions
        return False
tracer = setup_tracing("sgl-planner")
# If you can get a tokenizer from the backend, pass it in; otherwise omit.
gen_tracer = GenTracer(tracer, tokenizer=None)  # or tokenizer=your_tokenizer

# Try to import SGLang
_SGL_BACKEND_INITIALIZED = False
try:
    import sglang as sgl
    from sglang import function, system, user, assistant, gen
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False
    logger.warning("SGLang not available - incremental planner disabled")
    # Mock function decorator to avoid ImportErrors on definition
    def function(f): return f


INCREMENTAL_PLAN_SPECIFICATIONS = """
# INFOSPACE TYPE SYSTEM & RULES

Types:
- Note: Single value/document (persists across restarts)
  - Can be named (e.g., "my-note") for stable referencing via load
  - Named Notes can be loaded by name or by ID (e.g., "Note_123")
- Collection: List of Note/Collection IDs (session-local only)
  - Can be named (e.g., "my-collection") for stable referencing via load
  - Named Collections can be loaded by name or by ID (e.g., "Collection_456")
- Variables: Plan-local names referencing Notes/Collections

Variable Syntax:
- ALWAYS use "$variable" for references (target, value, source, out fields)
- Correct: {"value": "$my_variable"}
- Wrong: {"value": "my_variable"}
- Literal strings: Use directly without $ (e.g., "hello")

Operation Compatibility:
┌─────────────────────────┬──────┬────────────┐
│ Operation               │ Note │ Collection │
├─────────────────────────┼──────┼────────────┤
│ expand, as-json, refine │  ✓   │     ❌     │
│ summarize, relate       │  ✓   │     ✓      │
│ map, flatten            │  ❌  │     ✓      │
│ search-notes            │  N/A │     N/A    │ (global discovery)
│ search-collections      │  N/A │     N/A    │ (global discovery)
│ search-within-collection│  ❌  │     ✓      │ (requires indexed Collection)
└─────────────────────────┴──────┴────────────┘

Search Primitives:
- search-notes: Global discovery across all Notes (no target needed). Returns Collection of structured Notes with text preview, metadata.source_id, metadata.uri, metadata.score, metadata.type.
- search-collections: Global discovery across all Collections (no target needed). Returns Collection of structured Notes with text preview, metadata.source_id, metadata.uri, metadata.score, metadata.type.
- search-within-collection: Search within a specific indexed Collection (requires target Collection, must be indexed first). Returns Collection of structured Notes with text preview, metadata.source_id, metadata.uri, metadata.score, metadata.type.

All search primitives return structured Notes matching query-web/semantic-scholar format:
- text: First paragraph preview (200 chars max)
- format: "text" or "json"
- metadata.source_id: Original Note/Collection ID
- metadata.uri: URI field (Note/Collection ID or extracted URI from source)
- metadata.score: Search relevance score (0.0-1.0)
- metadata.type: "Note" or "Collection"
- char_count: Length of text preview

Use project with metadata.uri or metadata.source_id for consistent access across all search results.

Efficiency Rules:
- Use tools directly on Notes for single items
- Create Collections only for 2+ Notes together
- expand, refine, as-json work on Notes ONLY, not Collections
- Use map to apply Note operations to each Collection item

Tool Selection:
- Academic papers: semantic-scholar (provides abstracts, citations, PDFs)
- General web: query-web (broad coverage, recent content)
- Single URL fetch: fetch-text (NOT for query-web/semantic-scholar results)
- Generate new content: generate-note (creates text/code from scratch, no target needed)
- Transform existing content: refine (transforms existing Note content)
- as-markdown: EXTRACT existing markdown from mixed text (NOT for converting TO markdown)
- as-json: EXTRACT existing JSON from mixed text (NOT for converting TO JSON)

Boolean Tools (return true/false):
- is-empty: Check if text is null/empty/whitespace
- is-positive: Check if number is > 0
- is-question: Check if text contains a question
Result shows as Note with boolean content (True/False) - inspect actual value when needed
"""


def build_tool_catalog(available_tools: Dict[str, Dict], primitives_reference: str) -> Dict[str, Dict]:
    """
    Build tool catalog from available tools + infospace primitives.
    
    Returns dict mapping tool_name -> {fn, description, schema_hint}
    """
    tools = {}
    
    # Add infospace primitives (from reference doc)
    # Key primitives that need to be in catalog
    # Expanded primitive documentation from INFOSPACE_PRIMITIVES_REFERENCE
    PRIMITIVE_DOCS = {
        "create-collection": {
            "description": "Create a Collection object and bind to variable. IMPORTANT: The 'name' parameter (optional) creates a named Collection that can be loaded by name later. Variable names in 'out' (e.g., '$my_collection') are just bindings, NOT collection names.",
            "full_description": "Create a session-local Collection object and bind to variable. Collections store references to Notes. The optional 'name' parameter registers the Collection with a stable name for later loading (e.g., load by 'research' name). Variable names like '$my_collection' are temporary bindings during plan execution, not persistent names.",
            "parameters": {
                "value": "required: array of $variables or empty array",
                "name": "optional: string name for named Collection (separate from variable name)",
                "out": "required: $variable name for resulting Collection"
            },
            "examples": [
                '{"type":"create-collection","value":["$note1","$note2"],"out":"$my_collection"}',
                '{"type":"create-collection","name":"research","value":[],"out":"$papers"}',
                '{"type":"create-collection","value":"$note","out":"$single_item"}'
            ],
            "schema_hint": {"value": "array of $variables", "name": "string (optional)", "out": "$variable"}
        },
        "create-note": {
            "description": "Create a persistent Note object and bind to variable. IMPORTANT: The 'name' parameter (optional) creates a named Note that can be loaded by name later. Variable names in 'out' (e.g., '$my_note') are just bindings, NOT note names.",
            "full_description": "Create a persistent Note object and bind to variable. Notes store any content (text, JSON, etc.). The optional 'name' parameter registers the Note with a stable name for later loading (e.g., load by 'important-note' name). Variable names like '$my_note' are temporary bindings during plan execution, not persistent names. IMPORTANT: For JSON arrays/objects, pass the actual JSON structure (e.g., [\"apple\", \"banana\"] or {\"key\": \"value\"}), NOT a JSON string. The value field accepts Python types directly: strings, numbers, booleans, arrays, objects.",
            "parameters": {
                "value": "required: literal value (string, number, boolean, array, object) or $variable referencing content. For JSON arrays/objects, use actual JSON structure, not JSON string.",
                "name": "optional: string name for named Note (separate from variable name)",
                "out": "required: $variable name for resulting Note"
            },
            "examples": [
                '{"type":"create-note","value":"some data","out":"$my_note"}',
                '{"type":"create-note","value":["apple","banana","cherry"],"out":"$array_note"}',
                '{"type":"create-note","value":{"key":"value"},"out":"$object_note"}',
                '{"type":"create-note","value":"$variable","out":"$new_note"}',
                '{"type":"create-note","value":"important data","name":"important-note","out":"$my_note"}'
            ],
            "schema_hint": {"value": "any content (string, number, boolean, array, object)", "name": "string (optional)", "out": "$variable"}
        },
    }
    
    primitive_tools = {
        "create-note": {
            "description": PRIMITIVE_DOCS["create-note"]["description"],
            "full_description": PRIMITIVE_DOCS["create-note"].get("full_description"),
            "examples": PRIMITIVE_DOCS["create-note"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["create-note"]["schema_hint"]
        },
        "create-collection": {
            "description": PRIMITIVE_DOCS["create-collection"]["description"],
            "full_description": PRIMITIVE_DOCS["create-collection"].get("full_description"),
            "examples": PRIMITIVE_DOCS["create-collection"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["create-collection"]["schema_hint"]
        },
        "load": {
            "description": "Load persistent Note or Collection by ID or name. Can load named Notes/Collections by name (e.g., 'my-note') or by ID (e.g., 'Note_123').",
            "schema_hint": {"resource_id": "string (ID or name)", "out": "$variable", "expect": "string"}
        },
        "persist": {
            "description": "Mark Note/Collection as persistent",
            "schema_hint": {"target": "$variable"}
        },
        "search-notes": {
            "description": "Global search across all Notes using embedding-based retrieval. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches query-web/semantic-scholar for consistent project operations.",
            "schema_hint": {"value": "string (query)", "out": "$variable", "limit": "int (optional, default 5)", "threshold": "float (optional, default 0.3)"}
        },
        "search-collections": {
            "description": "Global search across all Collections using embedding-based retrieval. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches query-web/semantic-scholar for consistent project operations.",
            "schema_hint": {"value": "string (query)", "out": "$variable", "limit": "int (optional, default 3)", "threshold": "float (optional, default 0.3)"}
        },
        "search-within-collection": {
            "description": "Search within a specific indexed Collection. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches query-web/semantic-scholar for consistent project operations. Requires Collection to be indexed first.",
            "schema_hint": {"target": "$variable (indexed Collection)", "value": "string (query)", "out": "$variable", "limit": "int (optional, default 5)", "threshold": "float (optional, default 0.0)", "return_mode": "string (optional, 'chunks' or 'notes', default 'chunks')"}
        },
        "index": {
            "description": "Build embedding index for Collection",
            "schema_hint": {"source": "$variable"}
        },
        "map": {
            "description": "Apply operation to each item in Collection",
            "schema_hint": {"target": "$variable", "operation": "string", "out": "$variable"}
        },
        "expand": {
            "description": "Expand Note into Collection (JSON array or text lines)",
            "schema_hint": {"target": "$variable", "out": "$variable"}
        },
        "flatten": {
            "description": "Flatten Collection to single Note",
            "schema_hint": {"target": "$variable", "out": "$variable"}
        },
        "display": {
            "description": "Display Note content to user",
            "schema_hint": {"value": "$variable"}
        },
        "say": {
            "description": "Output text to user",
            "schema_hint": {"target": "user", "value": "string"}
        }
    }
    
    for name, meta in primitive_tools.items():
        tools[name] = {
            "fn": None,  # Placeholder - execution via infospace_executor
            "description": meta["description"],
            "schema_hint": meta["schema_hint"]
        }
    
    # Enhanced descriptions to prevent common confusions
    TOOL_DISAMBIGUATION = {
        "query-web": "Search web and return Collection of structured Notes. Each Note has text (full content), format, metadata.uri (URL), metadata.domain, char_count. Use project with metadata.uri to extract URLs. NO need for fetch-text after this.",
        "fetch-text": "Fetch text from a SINGLE specific URL, Note ID, or Collection ID. For Note IDs, retrieves Note content directly. For Collection IDs, uses first Note. Use ONLY when you have one URL/ID to fetch directly.",
        "semantic-scholar": "Search academic papers and return Collection of structured Notes. Each Note has text (abstract), format, metadata.uri (PDF URL), metadata.title, metadata.authors, metadata.year, metadata.citations, metadata.venue. Use project with metadata.uri to extract URLs. NO need for fetch-text after this.",
        "search-notes": "Global search across all Notes. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches query-web/semantic-scholar.",
        "search-collections": "Global search across all Collections. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches query-web/semantic-scholar.",
        "search-within-collection": "Search within indexed Collection. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches query-web/semantic-scholar.",
    }
    
    # Add available tools from map
    for tool_name, tool_meta in available_tools.items():
        param_source = tool_meta.get('parameter_source')
        schema_hint = {}
        
        if param_source and param_source.startswith('args.'):
            param_name = param_source.split('.', 1)[1]
            schema_hint[param_name] = "string"
        
        # Use enhanced description if available, otherwise use original
        description = TOOL_DISAMBIGUATION.get(tool_name, tool_meta.get('description', 'No description'))
        
        tools[tool_name] = {
            "fn": None,  # Placeholder
            "description": description,
            "schema_hint": schema_hint
        }
    
    return tools


def tool_catalog_text(tools: Dict[str, Dict]) -> str:
    """Format tool catalog for LLM prompt."""
    lines = []
    for name, meta in sorted(tools.items()):
        # Use expanded description if available (from PRIMITIVE_DOCS)
        description = meta.get('full_description') or meta.get('description', 'No description')
        lines.append(f"- {name}: {description}")
        schema = json.dumps(meta['schema_hint'])
        lines.append(f"  expected_args_schema: {schema}")
        
        # Add examples for primitives with expanded docs
        if 'examples' in meta and meta['examples']:
            lines.append(f"  examples:")
            for ex in meta['examples']:
                lines.append(f"    {ex}")
    
    # Add critical workflows to prevent common mistakes
    lines.append("\n# CRITICAL WORKFLOWS:")
    lines.append("- query-web → summarize (NOT query-web → fetch-text)")
    lines.append("- query-web already returns full text in 'text' field of each Note")
    lines.append("- semantic-scholar → summarize (NOT semantic-scholar → fetch-text)")
    lines.append("- fetch-text is for SINGLE URLs only, NOT for Collections from query-web/semantic-scholar")
    lines.append("- Level 4 tools (query-web, semantic-scholar) return Collections with complete data")
    
    return "\n".join(lines)


def load_skill_docs(tool_names: List[str], available_tools: Dict[str, Dict]) -> str:
    """
    Load full SKILL.md documentation for selected tools.
    
    Args:
        tool_names: List of tool names to load docs for
        available_tools: Dict mapping tool_name -> metadata (includes 'tool_path')
        
    Returns:
        Formatted string with full tool documentation
    """
    lines = []
    lines.append("# DETAILED TOOL DOCUMENTATION")
    lines.append("Full documentation for selected tools with examples, patterns, and output schemas:\n")
    
    for tool_name in tool_names:
        logger.debug(f"Stage 1.5: Processing {tool_name}")
        
        # Skip primitives (no SKILL.md files)
        if tool_name not in available_tools:
            logger.debug(f"Stage 1.5: {tool_name} not in available_tools (primitive, skipping)")
            continue
            
        tool_meta = available_tools[tool_name]
        # Use 'path' field which is the tool directory (not 'python_file' which is tool.py)
        tool_dir_path = tool_meta.get('path')
        logger.debug(f"Stage 1.5: {tool_name} path = {tool_dir_path}")
        
        if not tool_dir_path:
            logger.debug(f"Stage 1.5: {tool_name} has no path field, skipping")
            continue
        
        # Resolve to absolute path
        tool_dir = Path(tool_dir_path).resolve()
        logger.debug(f"Stage 1.5: {tool_name} tool_dir = {tool_dir}")
        
        # Look for SKILL.md or Skill.md in tool directory
        skill_file = None
        for variant in ['SKILL.md', 'Skill.md', 'skill.md']:
            candidate = tool_dir / variant
            if candidate.exists():
                skill_file = candidate
                logger.debug(f"Stage 1.5: {tool_name} found {variant} at {skill_file}")
                break
        
        if not skill_file:
            logger.debug(f"Stage 1.5: No SKILL.md found for {tool_name} in {tool_dir}")
            continue
        
        try:
            with open(skill_file, 'r', encoding='utf-8') as f:
                content = f.read()
            logger.debug(f"Stage 1.5: {tool_name} loaded {len(content)} chars from {skill_file}")
            
            # Robust frontmatter stripping using regex
            # Match first complete frontmatter block: --- ... ---
            original_len = len(content)
            frontmatter_match = re.search(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
            if frontmatter_match:
                # Extract content after the closing ---
                content = content[frontmatter_match.end():].strip()
                logger.debug(f"Stage 1.5: {tool_name} stripped frontmatter, {original_len} -> {len(content)} chars")
            else:
                logger.debug(f"Stage 1.5: {tool_name} no frontmatter found, using full content")
            
            if not content:
                logger.debug(f"Stage 1.5: {tool_name} content empty after stripping, skipping")
                continue
            
            lines.append(f"\n## {tool_name.upper()}")
            lines.append(content)
            lines.append("\n" + "="*80 + "\n")
            logger.info(f"Stage 1.5: Loaded docs for {tool_name} ({len(content)} chars)")
            
        except Exception as e:
            logger.warning(f"Stage 1.5: Failed to load SKILL.md for {tool_name}: {e}")
            traceback.print_exc()
            continue
    
    if len(lines) <= 2:  # Only header, no docs loaded
        logger.debug("Stage 1.5: No docs loaded for any tools")
        return ""
    
    total_docs = "\n".join(lines)
    logger.info(f"Stage 1.5: Returning {len(total_docs)} total chars of docs")
    return total_docs


def sgl_to_infospace_action(tool_name: str, args_json: str, step: int, available_tools: Dict[str, Dict]) -> Dict:
    """
    Convert SGLang Stage 2 output to infospace action format.
    
    Args:
        tool_name: Tool/primitive name
        args_json: JSON string with arguments
        step: Step number for variable naming
        available_tools: Dict of available tools (to distinguish tools from primitives)
        
    Returns:
        Infospace action dict
    """
    try:
        args = json.loads(args_json) if args_json else {}
    except json.JSONDecodeError:
        logger.warning(f"Failed to parse args_json: {args_json}")
        traceback.print_exc()
        args = {}
    
    # Fix double-nested args: if LLM generated {"args": {...}}, unwrap it
    # This happens when LLM copies the final action format instead of just the arguments
    if isinstance(args, dict) and 'args' in args and isinstance(args['args'], dict):
        # Check if 'args' is the only non-top-level field (or if other fields are top-level)
        # Core fields that should be at top level
        top_level_fields = ['out', 'expect', 'resource_id', 'source', 'operation', 'target', 'value']
        other_fields = {k: v for k, v in args.items() if k != 'args'}
        # If 'args' contains the actual parameters and other fields are just top-level, unwrap
        if not other_fields or all(k in top_level_fields for k in other_fields):
            nested_args = args.pop('args')
            # Merge nested args into args dict
            args.update(nested_args)
            logger.debug(f"Unwrapped double-nested args for {tool_name}")
    
    # Normalize variable references: ensure $ prefix for common variable fields
    # These fields typically reference variables (not literal values)
    # Exception: 'target' in say/display is a character name (literal), not a variable
    # Exception: 'value' in create-note/add/say/display/think/ask accepts literals, so don't normalize
    # These primitives accept both literals and variables - trust LLM to add $ when needed
    variable_fields = ['out', 'source']
    # Primitives that accept literal values in 'value' field (don't normalize)
    literal_value_primitives = ['create-note', 'add', 'say', 'display', 'think', 'ask']
    if tool_name not in literal_value_primitives:
        variable_fields.extend(['target', 'value'])
    # For literal_value_primitives, 'value' accepts literals, so don't normalize
    
    for field in variable_fields:
        if field in args:
            val = args[field]
            # Only normalize if it looks like a variable name (alphanumeric + underscore)
            # Skip if it's already prefixed with $, or if it looks like a resource ID, or contains special chars
            if isinstance(val, str) and val and not val.startswith('$'):
                # Check if it looks like a variable (not a URL, path, or resource ID)
                if re.match(r'^[a-zA-Z_]\w*$', val) and not val.startswith(('Note_', 'Collection_', 'http://', 'https://')):
                    args[field] = f"${val}"
                    logger.debug(f"Normalized '{field}' field: '{val}' -> '${val}'")
    
    # Build action
    action = {"type": tool_name}
    
    # Check if tool has parameter_source
    tool_has_param_source = False
    param_field_name = None
    if tool_name in available_tools:
        tool_meta = available_tools[tool_name]
        param_source = tool_meta.get('parameter_source')
        if param_source and param_source.startswith('args.'):
            tool_has_param_source = True
            param_field_name = param_source.split('.', 1)[1]
    
    # Core fields that are ALWAYS at top level (never inside args)
    always_top_level = ['out', 'expect', 'resource_id', 'source', 'operation']
    
    if tool_name not in available_tools:
        # Primitive: ALL fields at top level
        action.update(args)
    elif tool_has_param_source:
        # Tool with parameter_source (e.g., query-web with args.query)
        # Keep the parameter_source field in args, move others to top level
        for field in always_top_level:
            if field in args:
                action[field] = args.pop(field)
        # Keep param_field_name and any other args nested
        if args:
            action["args"] = args
    else:
        # Standard tool without parameter_source
        # target, value, out, expect at top level; optional params (query, focus, mode) in args
        standard_top_level = always_top_level + ['target', 'value']
        for field in standard_top_level:
            if field in args:
                action[field] = args.pop(field)
        # Remaining optional params (query, focus, mode, etc.) in nested args
        if args:
            action["args"] = args
    
    # Ensure 'out' field if tool produces output
    output_producing = ["create-note", "create-collection", "load", "search-notes", "search-collections", "search-within-collection", "map", 
                       "expand", "flatten", "query-web", "semantic-scholar", "summarize",
                       "refine", "generate-note", "assess", "relate", "extract-entities", "filter-collection",
                       "fetch-text", "as-json", "as-markdown"]
    if tool_name in output_producing and "out" not in action:
        action["out"] = f"$step_{step}_result"
    
    # Add expect if needed
    uncertain_tools = ["query-web", "semantic-scholar", "search-notes", "search-collections", "search-within-collection", "load"]
    if tool_name in uncertain_tools and "expect" not in action:
        action["expect"] = f"should get result from {tool_name}"
    
    return action


def execute_infospace_action(action: Dict, executor, agent_name: str) -> str:
    """
    Execute single action via infospace_executor, return result text.
    
    Args:
        action: Infospace action dict
        executor: InfospaceExecutor instance
        agent_name: Agent name for logging
        
    Returns:
        Result text for Stage 3 reflection
    """
    try:
        # Track action in executor's plan_actions if available
        if hasattr(executor, '_plan_actions'):
            executor._plan_actions.append(action.copy())
        
        result = executor.execute_action(action)
        
        # Track action in executive_node.action_history if available (for plan completion summary)
        if hasattr(executor, 'executive_node') and executor.executive_node:
            from datetime import datetime
            executive_node = executor.executive_node
            now_ts = datetime.now()
            executive_node.step_counter += 1
            
            # Import ActionRecord from executive_node (avoid circular import by importing here)
            from executive_node import ActionRecord
            
            action_record = ActionRecord(
                action=action,
                result=result.get('value', '') if result.get('status') == 'success' else result.get('reason', 'failed'),
                timestamp=now_ts,
                step_id=executive_node.step_counter,
                plan_id=getattr(executive_node, 'current_plan_id', None),
                requested_target=action.get('target', ''),
                started_at=now_ts,
                ended_at=datetime.now(),
                outcome_status=result.get('status', 'unknown')
            )
            # Snapshot physiology if method available
            if hasattr(executive_node, '_snapshot_physiology'):
                executive_node._snapshot_physiology(action_record)
            executive_node.action_history.append(action_record)
            
            # Publish action result for UI display
            if hasattr(executive_node, '_publish_action_result'):
                executive_node._publish_action_result(action, result, action.get('type', ''), now_ts)
        
        # Track compliance if evaluator is active
        if hasattr(executor, '_compliance_tracker') and executor._compliance_tracker:
            executor._compliance_tracker.check_action(action, result, executor.plan_bindings)
        
        if result.get('status') == 'success':
            bound_var = action.get('out', '')
            if bound_var:
                # Get bound value if available
                resource_id = executor.plan_bindings.get(bound_var.lstrip('$'))
                if resource_id:
                    # Enhanced feedback for Collections
                    if resource_id.startswith('Collection_'):
                        metadata = executor.get_resource_metadata(resource_id)
                        if metadata:
                            item_count = metadata.get('item_count', 0)
                            source_skill = metadata.get('source_skill', '')
                            # Build informative message
                            info_parts = [f"{item_count} items"]
                            if source_skill:
                                info_parts.append(f"from {source_skill}")
                            return f"[SUCCESS] Bound {bound_var} to {resource_id} ({', '.join(info_parts)})"
                    
                    # For Notes, try to show simple content (booleans, short strings)
                    elif resource_id.startswith('Note_'):
                        # Fetch content for simple types to aid reflection
                        content = executor._get_content(resource_id)
                        if isinstance(content, bool):
                            return f"[SUCCESS] Bound {bound_var} to {resource_id} (value: {content})"
                        elif isinstance(content, (int, float)):
                            return f"[SUCCESS] Bound {bound_var} to {resource_id} (value: {content})"
                        elif isinstance(content, str) and len(content) <= 50:
                            return f"[SUCCESS] Bound {bound_var} to {resource_id} (value: '{content}')"
                    
                    return f"[SUCCESS] Bound {bound_var} to {resource_id}"
            return f"[SUCCESS] {action['type']} completed"
        else:
            error_reason = result.get('reason', 'Unknown error')
            return f"[ERROR] {action['type']} failed: {error_reason}"
    except Exception as e:
        logger.error(f"Execution error: {e}")
        traceback.print_exc()
        return f"[ERROR] Exception: {str(e)}"


if HAS_SGLANG:
    @function
    def stage0_resource_retrieval(s, goal: str, executor):
        """
        Stage 0: Generate search queries from goal and retrieve relevant resources.
        
        Args:
            s: SGLang state
            goal: Goal text
            executor: InfospaceExecutor instance
            
        Returns:
            Formatted string with available resources to inject into Stage 1 prompt
        """
        s += system(
            "You are helping extract search queries from a goal statement. "
            "Focus on WHAT the goal is about (topics, concepts, data) rather than HOW to achieve it (actions, steps). "
            "Generate 1-2 concise search queries that would help find relevant Notes or Collections."
        )
        s += user(f"Goal: {goal}\n\nGenerate 1-2 concise search queries (one per line, no numbering):")
        s += assistant(gen("queries", max_tokens=128, stop="\n\n"))
        
        try:
            queries_text = s["queries"].strip()
            # Parse queries (one per line)
            queries = [q.strip() for q in queries_text.split('\n') if q.strip()][:2]
            
            if not queries:
                return ""
            
            # Search for resources
            search_result = executor.search_resources(queries, k_notes=3, k_collections=2, threshold=0.3)
            
            if search_result.get('status') != 'success':
                reason = search_result.get('reason', 'Unknown error')
                # Don't warn if queryable isn't ready yet or if indexes are empty (normal on startup)
                if 'queryable may not be registered' in reason.lower() or 'no response' in reason.lower():
                    logger.debug(f"Stage 0: Resource search unavailable (normal on startup): {reason}")
                else:
                    logger.warning(f"Stage 0: Resource search failed: {reason}")
                return ""
            
            notes = search_result.get('notes', [])
            collections = search_result.get('collections', [])
            
            if not notes and not collections:
                # Empty results are normal when no resources exist yet
                logger.debug("Stage 0: No relevant resources found (indexes may be empty)")
                return ""
            
            # Format results for prompt injection with descriptions
            lines = ["# Available Notes / Collections (may be relevant)\n"]
            
            if notes:
                lines.append("## Notes:")
                for note in notes:
                    name = note.get('name', note.get('resource_id', ''))
                    resource_id = note.get('resource_id', '')
                    props = note.get('properties', {})  # Properties extracted separately, no ResourceType
                    
                    # Build description from metadata
                    desc_parts = []
                    source_skill = props.get('source_skill', '')
                    source_value = props.get('source_value', '')
                    if source_skill:
                        if source_value:
                            desc_parts.append(f"Created via {source_skill} on {source_value}")
                        else:
                            desc_parts.append(f"Created via {source_skill}")
                    
                    # Add commentary if available
                    commentary = props.get('embedding_text', '').split('\n')
                    # Commentary is usually after source_skill line, extract relevant part
                    if len(commentary) > 2:
                        commentary_text = '\n'.join(commentary[2:]).strip()[:150]  # Skip name and source lines
                        if commentary_text:
                            desc_parts.append(commentary_text)
                    
                    description = ". ".join(desc_parts) if desc_parts else "No description available"
                    lines.append(f"- {resource_id} (\"{name}\"): {description}")
            
            if collections:
                lines.append("\n## Collections:")
                for coll in collections:
                    name = coll.get('name', coll.get('resource_id', ''))
                    resource_id = coll.get('resource_id', '')
                    item_count = coll.get('item_count', 0)
                    props = coll.get('properties', {})  # Properties extracted separately, no ResourceType
                    
                    # Build description from metadata
                    desc_parts = [f"{item_count} items"]
                    source_skill = props.get('source_skill', '')
                    source_value = props.get('source_value', '')
                    if source_skill:
                        if source_value:
                            desc_parts.append(f"created via {source_skill} on {source_value}")
                        else:
                            desc_parts.append(f"created via {source_skill}")
                    
                    # Add commentary if available
                    commentary = props.get('embedding_text', '').split('\n')
                    if len(commentary) > 3:  # Skip name, source, item_count lines
                        commentary_text = '\n'.join(commentary[3:]).strip()[:150]
                        if commentary_text:
                            desc_parts.append(commentary_text)
                    
                    description = ". ".join(desc_parts)
                    lines.append(f"- {resource_id} (\"{name}\"): {description}")
            
            lines.append("\nTo use these resources, reference by name (e.g., \"my-note\") or ID (e.g., \"Note_42\") in `load` actions.")
            
            result_text = "\n".join(lines)
            logger.info(f"Stage 0: Found {len(notes)} Notes and {len(collections)} Collections")
            return result_text
            
        except Exception as e:
            logger.warning(f"Stage 0: Error in resource retrieval: {e}")
            import traceback
            traceback.print_exc()
            return ""
    
    @function
    def tool_planner_infospace(s, goal: str, character_context: str, recent_context: str, 
                              tools_catalog_text: str, executor, max_steps: int = 16):
        """
        SGLang incremental planner for infospace goals.
        
        Args:
            s: SGLang state
            goal: Goal text
            character_context: Character description + drives
            recent_context: Recent thoughts/memories + last action
            tools_catalog_text: Formatted tool catalog
            executor: InfospaceExecutor instance (with _plan_actions attribute)
            max_steps: Maximum planning steps
        """
        # Stage 0: Resource retrieval (if executor available)
        available_resources_text = ""
        if executor:
            try:
                available_resources_text = stage0_resource_retrieval.run(goal=goal, executor=executor)
                logger.info(f"Stage 0: Available resources: {available_resources_text}")
            except Exception as e:
                logger.warning(f"Stage 0: Failed to retrieve resources: {e}")
        
        # Stage 1: Analysis + tool selection
        system_parts = [
            "You are a planning-and-acting assistant for information space operations.",
            "You can choose tools/primitives, call them via JSON arguments,",
            "and iteratively refine your plan until the goal is satisfied.\n"
        ]
        
        # Add character context if available
        if character_context:
            system_parts.append(f"\n# CHARACTER CONTEXT\n{character_context}\n")
        
        # Add recent context if available
        if recent_context:
            system_parts.append(f"{recent_context}\n")
        
        # Add available resources from Stage 0
        if available_resources_text:
            system_parts.append(f"\n{available_resources_text}\n")
        
        system_parts.append(f"\n{INCREMENTAL_PLAN_SPECIFICATIONS}\n")
        system_parts.append(
            "You will work in repeated cycles:\n"
            "Stage 1 (once): Analyze goal, select relevant tools, decompose into FIRST_TASK.\n"
            "Stage 2 (loop): Pick a single tool and JSON args for CURRENT_TASK.\n"
            "Stage 3 (loop): Reflect on result, decide if goal done, set NEXT_TASK.\n\n"
            "ALWAYS follow formatting instructions exactly."
        )
        
        s += system("".join(system_parts))
        
        s += user(
            f"Goal: {goal}\n\n"
            f"Tool catalog:\n{tools_catalog_text}\n\n"
            "Stage 1: Analyze goal and identify relevant tools. Be concise in your analysis.\n"
            "Include tools you might need AND related/supporting tools.\n"
            "Err on the side of including additional tools in SELECTED_TOOLS_JSON for better coverage.\n"
            "Then, decompose the goal into a FIRST high-level task/subgoal to focus on.\n"
            "In doing so, consider the tools you have selected, the goal you are trying to achieve, and the downstream tasks that will be required to achieve the goal.\n"
            "Respond with the following fields:\n"
            "ANALYSIS: <text>\n"
            "SELECTED_TOOLS_JSON: <json list of tool names>\n"
            "FIRST_TASK: <high-level subgoal to tackle first>\n"
        )
        
        s += assistant(
            "ANALYSIS: "
            + gen("stage1_analysis", max_tokens=256, stop="\n")
            + "\nSELECTED_TOOLS_JSON: "
            + gen("selected_tools_json", max_tokens=256, stop="\n")
            + "\nFIRST_TASK: "
            + gen("first_task", max_tokens=128, stop="\n")
            + "\n"
        )
        
        try:
            logger.info(f"Stage 1: Analysis + tool selection\n{s['stage1_analysis']}")
            logger.info(f"SELECTED_TOOLS_JSON: {s['selected_tools_json']}")
            logger.info(f"FIRST_TASK: {s['first_task']}")
        except KeyError as e:
            logger.warning(f"Stage 1 values not available: {e}")
        
        # Stage 1.5: Load and inject detailed docs for selected tools
        try:
            selected_tools_json = s['selected_tools_json']
            selected_tools = json.loads(selected_tools_json)
            if isinstance(selected_tools, list) and selected_tools:
                expanded_docs = load_skill_docs(selected_tools, executor.available_tools)
                if expanded_docs:
                    s += user(expanded_docs)
                    s += assistant("I have reviewed the detailed documentation for the selected tools.\n")
                    logger.info(f"Stage 1.5: Injected detailed docs for {len(selected_tools)} tools")
        except (json.JSONDecodeError, TypeError) as e:
            logger.warning(f"Failed to parse selected tools for doc expansion: {e}")
        
        # Stage 2/3 format instructions
        s += user(
            "Stage 2 FORMAT:\n"
            "  TOOL_NAME: <name>\n"
            "  TOOL_ARGS_JSON: <json object>\n\n"
            "Stage 3 FORMAT:\n"
            "  THOUGHTS: <text>\n"
            "  DONE: <YES or NO - is the entire GOAL satisfied?>\n"
            "  NEXT_TASK: <next high-level subgoal, or blank if DONE=YES>\n"
            "  REQUEST_TOOLS: <json list of tool names>\n\n"
            "  If you realize you need a tool not initially selected, add it to REQUEST_TOOLS.\n"
            "  You'll receive its full documentation before the next step.\n\n"
            "Follow these formats exactly."
        )
        s += assistant("Understood.\n")

        # Main loop
        current_task = s["first_task"].strip()
        for step in range(max_steps):
            # Stage 2: Choose tool + args
            s += user(
                f"STAGE 2 (step {step + 1}/{max_steps}):\n"
                f"GOAL: {goal}\n"
                f"CURRENT_TASK: {current_task}\n"
                "Choose tool and JSON args using Stage 2 FORMAT.\n"
            )
            
            s += assistant(
                "TOOL_NAME: "
                + gen(f"tool_name_{step}", max_tokens=32, stop="\n")
                + "\nTOOL_ARGS_JSON: "
                + gen(f"tool_args_{step}", max_tokens=512, stop="\n")
                + "\n"
            )
            
            # Execute tool
            tool_name = s[f"tool_name_{step}"].strip()
            tool_args_json = s[f"tool_args_{step}"].strip()
            action = sgl_to_infospace_action(tool_name, tool_args_json, step, executor.available_tools)
            
            # Track resource bindings before execution
            out_var = action.get('out', '')
            resource_id_before = None
            if out_var:
                resource_id_before = executor.plan_bindings.get(out_var.lstrip('$'))
            
            tool_result = execute_infospace_action(action, executor, executor.agent_name)
            logger.info(f"Stage 2: Choose tool + args\n{tool_name} -> {tool_args_json}")
            logger.info(f"Step {step}: {tool_name} -> {tool_result[:100]}")
            
            # Stage 3: Reflect
            s += user(
                f"STAGE 3 (step {step + 1}/{max_steps}):\n"
                f"Tool `{tool_name}` with args:\n{tool_args_json}\n\n"
                f"Result:\n{tool_result[:512]}\n\n"
                "Respond using Stage 3 FORMAT. Be concise.\n"
                "Ensure the REQUEST_TOOLS list is a valid JSON list of tool names.\n"
                "DONE: YES only when you have EXECUTED all required actions (not just planned them). "
                "Thinking about an action ≠ executing it. You must actually call display/persist/etc.\n"
            )
            
            s += assistant(
                "THOUGHTS (Be concise): "
                + gen(f"thoughts_{step}", max_tokens=128, stop="\n")
                + "\nDONE: "
                + gen(f"done_{step}", max_tokens=8, stop="\n")
                + "\nNEXT_TASK: "
                + gen(f"next_task_{step}", max_tokens=128, stop="\n")
                + "\nREQUEST_TOOLS <optional: json list of tool names>(tools you may need but didn't initially select): \n"
                + gen(f"request_tools_{step}", max_tokens=64, stop="\n")
                + "\n"
            )
            logger.info(f"THOUGHTS: {s[f'thoughts_{step}']}")
            logger.info(f"DONE: {s[f'done_{step}']}")
            logger.info(f"NEXT_TASK: {s[f'next_task_{step}']}")
            logger.info(f"REQUEST_TOOLS: {s[f'request_tools_{step}']}")
            
            # Stage 3.1: Update resource indexes with commentary
            # Track ANY resource created in this step (not just explicit create-note/create-collection)
            thoughts_text = s[f'thoughts_{step}'].strip()
            if thoughts_text:
                # Check if a new resource was bound in this step
                out_var = action.get('out', '')
                resource_id = None
                if out_var:
                    resource_id_after = executor.plan_bindings.get(out_var.lstrip('$'))
                    # If resource_id changed (new resource created) or didn't exist before
                    if resource_id_after and resource_id_after != resource_id_before:
                        if resource_id_after.startswith('Note_') or resource_id_after.startswith('Collection_'):
                            resource_id = resource_id_after
                
                # Update index with commentary if resource was created
                if resource_id:
                    try:
                        if executor.resource_manager:
                            executor.resource_manager.update_resource_commentary(resource_id, thoughts_text)
                            logger.debug(f"Stage 3.1: Updated commentary for {resource_id} (created via {action.get('type')})")
                        else:
                            logger.debug(f"Stage 3.1: Resource manager not available for {resource_id}")
                    except Exception as e:
                        logger.debug(f"Stage 3.1: Failed to update commentary for {resource_id}: {e}")
            
            # Stage 3.5: Dynamic tool loading (if requested)
            requested_tools_raw = s[f"request_tools_{step}"].strip()
            requested_tools = parse_request_tools(requested_tools_raw)
            if requested_tools:
                logger.info(f"Step {step}: LLM requested additional tools: {requested_tools}")
                expanded_docs = load_skill_docs(requested_tools, executor.available_tools)
                if expanded_docs:
                    s += user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                    s += assistant("I have reviewed the additional tool documentation.\n")
                    logger.info(f"Stage 3.5: Loaded docs for {len(requested_tools)} additional tools: {requested_tools}")
            elif requested_tools_raw and requested_tools_raw.lower() not in ["", "[]", "none", "null"]:
                # Log warning only if there was actual content that failed to parse
                logger.warning(f"Step {step}: Failed to parse REQUEST_TOOLS: {requested_tools_raw[:100]}")
            
            # Check if done
            done_raw = s[f"done_{step}"].strip().upper()
            if done_raw.startswith("YES"):
                s["final_answer"] = s[f"thoughts_{step}"]
                #print (f"full trace:\n{s}")
                return
            
            # Update current task for next iteration
            next_task_raw = s[f"next_task_{step}"].strip()
            if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                current_task = next_task_raw
                logger.info(f"Step {step}: Next task: {current_task}")
            else:
                logger.warning(f"Step {step}: No NEXT_TASK provided, keeping current task")
        
        # Max steps reached
        s["final_answer"] = (
            f"Max steps reached. Last task: {current_task}\n"
            f"Last thoughts: {s[f'thoughts_{max_steps-1}']}"
        )
        logger.info(f"full trace:\n{s}")
        return s


def parse_request_tools(raw_text: str) -> Optional[List[str]]:
    """
    Robustly parse REQUEST_TOOLS field from LLM output.
    
    Handles:
    - "REQUEST_TOOLS: " prefix
    - Markdown code fences (```json ... ```)
    - Empty/partial responses
    - Raw JSON arrays
    
    Args:
        raw_text: Raw text from LLM generation
        
    Returns:
        List of tool names if successfully parsed, None otherwise
    """
    if not raw_text:
        return None
    
    text = raw_text.strip()
    
    # Remove "REQUEST_TOOLS: " prefix if present
    if text.startswith("REQUEST_TOOLS:"):
        text = text[len("REQUEST_TOOLS:"):].strip()
    
    # Handle empty/placeholder values
    if not text or text.lower() in ["", "[]", "none", "null", "n/a"]:
        return None
    
    # Extract JSON from markdown code fences
    # Pattern: ```json ... ``` or ``` ... ```
    code_fence_match = re.search(r'```(?:json)?\s*\n?(.*?)\n?```', text, re.DOTALL)
    if code_fence_match:
        text = code_fence_match.group(1).strip()
    
    # Try to parse as JSON
    try:
        parsed = json.loads(text)
        if isinstance(parsed, list):
            # Filter out non-string items and empty strings
            tools = [t for t in parsed if isinstance(t, str) and t.strip()]
            return tools if tools else None
        return None
    except (json.JSONDecodeError, TypeError, ValueError):
        # If parsing fails, try to extract list-like content manually
        # Look for patterns like ["tool1", "tool2"] or [tool1, tool2]
        list_match = re.search(r'\[(.*?)\]', text, re.DOTALL)
        if list_match:
            content = list_match.group(1)
            # Extract quoted strings
            tools = re.findall(r'"([^"]+)"', content)
            if tools:
                return tools
        return None


class IncrementalPlanner:
    """
    Incremental planner using SGLang for iterative goal achievement.
    """
    
    def __init__(self, executor, available_tools: Dict[str, Dict], 
                 primitives_reference: str, logger_instance=None, sgl_model_path: str = None):
        """
        Initialize incremental planner.
        
        Args:
            executor: InfospaceExecutor instance
            available_tools: Dict of tool_name -> metadata
            primitives_reference: Primitives reference text
            logger_instance: Optional logger
            sgl_model_path: Path to local model for SGLang (required if not already initialized)
        """
        if not HAS_SGLANG:
            raise ImportError("SGLang not available")
        
        self.executor = executor
        self.available_tools = available_tools
        self.primitives_reference = primitives_reference
        self.logger = logger_instance or logger
        
        # Initialize SGLang backend if needed
        global _SGL_BACKEND_INITIALIZED
        if not _SGL_BACKEND_INITIALIZED and sgl_model_path:
            try:
                self.logger.info(f"Initializing SGLang backend with model: {sgl_model_path}")
                runtime = sgl.Runtime(
                    model_path=sgl_model_path,
                    tokenizer_path=sgl_model_path,
                    device="cuda",
                    context_length=32768,
                    cuda_graph_max_bs=4,
                    dtype="auto",
                    tp_size=1,
                    mem_fraction_static=0.82,
                    tool_call_parser="qwen"
                )
                sgl.set_default_backend(runtime)
                executor.runtime = runtime
                _SGL_BACKEND_INITIALIZED = True
            except Exception as e:
                self.logger.error(f"Failed to initialize SGLang backend: {e}")
                raise
        elif not _SGL_BACKEND_INITIALIZED:
            self.logger.warning("SGLang backend not initialized and no model path provided!")
        
        # Build tool catalog
        self.tools = build_tool_catalog(available_tools, primitives_reference)
        self.tools_catalog_text = tool_catalog_text(self.tools)
        self.logger.info(f"IncrementalPlanner initialized with {len(self.tools)} tools")
    
    def generate_plan(self, goal: str, context: Dict = None, max_steps: int = 16) -> Dict:
        """
        Generate plan incrementally using SGLang.
        
        Args:
            goal: Goal text
            context: Optional dict with character_context, recent_context
            max_steps: Maximum planning steps
            
        Returns:
            Plan dict with 'plan' key containing actions
        """
        if not HAS_SGLANG:
            return {'error': 'SGLang not available'}
        
        try:
            # Clear executor state for new plan
            self.executor.clear_plan_state()
            
            # Attach plan_actions list to executor for tracking
            self.executor._plan_actions = []
            
            # Extract context components
            character_context = ""
            recent_context = ""
            if context:
                character_context = context.get('character_context', '')
                recent_context = context.get('recent_context', '')
            
            # Run SGLang planner
            state = tool_planner_infospace.run(
                goal=goal,
                character_context=character_context,
                recent_context=recent_context,
                tools_catalog_text=self.tools_catalog_text,
                executor=self.executor,
                max_steps=max_steps
            )
            
            # Extract plan actions from executor
            plan_actions = getattr(self.executor, '_plan_actions', [])
            
            # Extract final_answer from ProgramState (use bracket notation)
            try:
                final_answer = state['final_answer']
            except (KeyError, TypeError):
                final_answer = 'Planning completed'
            
            return {
                'plan': plan_actions,
                'reasoning': final_answer,
                'success': True,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Incremental planning failed: {e}")
            traceback.print_exc()
            return {'error': str(e)}

