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
import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional
from infospace_executor import InfospaceExecutor
from plan_guidance import PlanGuidance
from tool_model import ToolModel
from world_model import WORLD_MODEL_SCHEMA, empty_world_model

# Global temperature setting for all gen() calls
GEN_TEMPERATURE = 0.5

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
    # Disable SGLang progress bars
    import os
    os.environ['TQDM_DISABLE'] = '1'
except ImportError:
    HAS_SGLANG = False
    logger.warning("SGLang not available - incremental planner disabled")
    # Mock function decorator to avoid ImportErrors on definition
    def function(f): return f



REFLECTION_FRAME_SCHEMA = {
  "ReflectionFrame": {
    "version": "1.0",

    "_size_limits": {
      "task_state.active_hypotheses": 6,
      "task_state.proven_safe_paths": 6,
      "task_state.exhausted_search.locations": 6,
      "task_state.exhausted_search.objects": 6,
      "task_state.exhausted_search.actions": 6,
      "world_model_updates": 10,
      "tool_insights": 6,
      "retired_beliefs": 4,
      "context_forget": 8,
      "open_questions": 4
    },

    "failure_mode": "incorrect_task_interpretation | incorrect_world_assumption | missing_affordance | tool_limitation_or_misbehavior | exhausted_or_misdirected_search",

    "failure_evidence": ["string"],

    "task_state": {
      "summary": "string",
      "immediate_blockers": ["string"],
      "active_hypotheses": ["string"],
      "proven_safe_paths": ["string"],
      "exhausted_search": {
        "locations": ["string"],
        "objects": ["string"],
        "actions": ["string"]
      }
    },

    "world_model_updates": [
      {
        "fact": "string",
        "confidence": "low | medium | high",
        "source": "observation | tool_guarantee",
        "stability": "invariant | regularity | anecdote"
      }
    ],

    "tool_insights": [
      {
        "tool": "string",
        "insight": "string",
        "status": "reliable | unreliable | constrained",
        "evidence": "string"
      }
    ],

    "retired_beliefs": [
      {
        "belief": "string",
        "reason": "contradicted | invalid_generalization",
        "evidence": "string"
      }
    ],

    "context_forget": [
      {
        "item": "string",
        "reason": "obsolete | goal_specific | location_specific | superseded"
      }
    ],

    "open_questions": [
      {
        "question": "string",
        "priority": "low | medium | high",
        "requires_environment_interaction": True
      }
    ]
  }
}

INCREMENTAL_PLAN_SPECIFICATIONS = """
# INFOSPACE TYPE SYSTEM & RULES

Types:
- Note: Single value/document 
  - Can be named (e.g., "my-note") for stable referencing via load
  - Named Notes can be loaded by name or by ID (e.g., "Note_123")
- Collection: List of Note/Collection IDs 
  - Can be named (e.g., "my-collection") for stable referencing via load
  - Named Collections can be loaded by name or by ID (e.g., "Collection_456")
- Variable: a session-local name referencing Notes/Collections. Variables *always* start with "$".

Action Syntax:
- An action is a JSON object specifying the application of a primitive or tool. 
  - e.g. {"type": "primitive or tool name", "target": "$variable_a", "value": "literal_value", "out": "$variable_b"} 
- ALWAYS use "$variable" for references (e.g. target, value, source, or out fields)
- Correct: {"target": "$my_variable"}
- Wrong: {"target": "my_variable"}
- Note reference by ID or name: Use directly without $ (e.g., "target": "Note_123" or "target": "attention-note")
- Collection reference by ID or name: Use directly without $ (e.g., "target": "Collection_456" or "target": "research-papers")
- Literal strings: Use directly without $ (e.g., "value": "hello")
- Literal numbers: Use directly without $ (e.g., "value": 123)
- Literal booleans: Use directly without $ (e.g., "value": true)

InfospacePrimitive Action / Type Compatibility:
This table applies only to native infospace primitives. World/skill tools are governed by their own contracts.
Operation_name: applicable to: <Note | Collection | Note, Collection>;   Purpose
 - split: Note;  Note structure → Collection
 - flatten: Collection;  Collection → single Note
 - as-json, refine, coerce: Note;  Transform Note content
 - summarize, relate: Note, Collection;  Generate new content
 - map: Collection;  Apply op to each Collection item
 - project, pluck, sort, filter: Collection;  SQL-like Collection ops
 - head: Collection;  Take first N items
 - join: Collection;  Merge 2 Collections (SQL JOIN)
 - add, remove, size: Collection;  Collection mutation operations
 - union, intersection, difference: Collection;  Set operations on Collections
 - load: Note, Collection;  Load persistent resource
 - persist: Note, Collection;  Mark resource as persistent
 - display: Note, Collection;  Show content to user (UI only)
 - search-notes, search-collections: N/A;  Global discovery (return Coll.)
 - search-within-collection: Collection;  Search indexed Collection

Key distinctions:
- split (Note→Coll): Transforms internal structure (array/lines) into separate items
- flatten (Coll→Note): Opposite of split, merges Collection into single Note
- load: Use to GET content *into planner context* (returns Note content or Collection Note IDs)
- display: Use to SHOW content to user (UI popup, does NOT return content for planning)
- persist: Mark resource as persistent (saved to filesystem)

Search Primitives:
- search-notes: Global discovery across all Notes (no target needed). Returns Collection of structured Notes with full text content, metadata.source_id, metadata.uri, metadata.score, metadata.type.
- search-collections: Global discovery across all Collections (no target needed). Returns Collection of structured Notes with full text content, metadata.source_id, metadata.uri, metadata.score, metadata.type.
- search-within-collection: Search within a specific indexed Collection (requires target Collection, must be indexed first). Returns Collection of structured Notes with full matched chunk text, metadata.source_id, metadata.uri, metadata.score, metadata.type, metadata.chunk_index, metadata.chunk_total.

All search primitives return structured Notes matching search-web/semantic-scholar format:
- text: Full text content (for search-within-collection: the matched chunk text; for search-notes/search-collections: full Note content)
- format: "text" or "json"
- metadata.source_id: Original Note/Collection ID (use project to extract: project with fields=["metadata.source_id"])
- metadata.uri: URI field (Note/Collection ID or extracted URI from source)
- metadata.score: Search relevance score (0.0-1.0)
- metadata.type: "Note" or "Collection"
- metadata.chunk_index, metadata.chunk_total: (search-within-collection only) Chunk position info
- char_count: Length of text

Use project to extract metadata fields (uri, source_id, score, etc.). For extracting information FROM text content, use refine (LLM-based).

Persistence Operations:
- load: Retrieve persistent Note or Collection by ID or name. Returns prefixed content ("Note Content: <text>" or "Collection Content: <ids>") truncated to 1024 chars. The prefix clarifies that the returned text is Note/Collection content, not domain-specific output. Use to get content into planner context.
- persist: Mark Note or Collection as persistent (saved to filesystem). Use after creating resources you want to keep.

Collection Mutation Operations (require Collection):
- add: Add Note or Collection to existing Collection (mutates in place)
- remove: Remove Note from Collection (mutates in place)
- size: Get item count of Collection (returns Note with integer)
- union: Combine two Collections (A ∪ B) - all items, deduplicated
- intersection: Items in both Collections (A ∩ B)
- difference: Items in A but not B (A - B)

Structured-data Collection Operations (require Notes of type dict/JSON):
- project: Extract metadata/structured fields (SELECT columns) → new Collection with subset of fields
- pluck: Extract single field as simple values → Collection of values  
- filter-structured: Filter by field conditions (WHERE clauses) → filtered Collection
- sort: Sort by field value (ORDER BY) → sorted Collection
- head: Take first N items (LIMIT) → smaller Collection preserving original Notes
- join: Combine two Collections on matching field (INNER JOIN) → merged Collection

IMPORTANT: project/pluck extract NAMED FIELDS only (metadata.uri, metadata.title, etc.). They do NOT parse unstructured text.
To extract information FROM text content, use refine (LLM-based extraction).

Use cases:
- project: Extract metadata.uri from search results, get metadata.title+metadata.year from papers
- pluck: Get just metadata.title as simple list, extract metadata.score for analysis
- filter-structured: Papers after 2020, results with score>0.5, venue contains "NeurIPS"
- sort: Rank by score (descending), chronological by year, alphabetical by title
- head: Get top 5 after sorting, take first result from search
- join: Merge papers with citation data, combine user info with profiles

Efficiency Heuristics:
- Use tools directly on Notes for single items
- Create Collections only for 2+ Notes together
- split, refine, as-json work on Notes ONLY, not Collections
- Use map to apply Note operations to each Collection item

Common Patterns:
- Add multiple Notes to Collection via map: {"type":"map","target":"$notes","operation":"add","collection":"$collection","out":"$collection"}
- Add all items from one Collection to another: {"type":"union","target":"$target_collection","value":"$source_collection","out":"$target_collection"}
- Combine two Collections into new one: {"type":"union","target":"$coll1","value":"$coll2","out":"$combined"}

#TOOLS
Tool System
  - Tools include primitive tools, discussed above, and external tools, loaded at runtime from the tools directory.
"""


def build_tool_catalog(available_tools: Dict[str, Dict]) -> Dict[str, Dict]:
    """
    Build tool catalog from available tools + infospace primitives.
    
    Returns dict mapping tool_name -> {fn, description, schema_hint}
    """
    def _infer_tool_source(tool_meta: Dict[str, Dict]) -> str:
        """
        Infer tool source as 'core' or <world_name>.
        
        - src/tools/<tool>/...            -> core
        - src/world-tools/<world>/<tool>/ -> <world>
        """
        path = tool_meta.get('path') or tool_meta.get('python_file') or ''
        if not isinstance(path, str) or not path:
            return 'core'
        
        # Normalize separators for robust matching
        p = path.replace('\\', '/')
        
        marker = '/src/world-tools/'
        if marker in p:
            after = p.split(marker, 1)[1]
            world_name = after.split('/', 1)[0].strip()
            return world_name or 'core'
        
        return 'core'
    
    tools = {}
    core_tools = {}
    world_tools = {}
    
    # Add infospace primitives (from reference doc)
    # Key primitives that need to be in catalog
    # Expanded primitive documentation from INFOSPACE_PRIMITIVES_REFERENCE
    PRIMITIVE_DOCS = {
        "create-collection": {
            "description": "Create a Collection object and bind to variable.",
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
        "think": {
            "description": "generate an internal reflection on the state of the plan. Use for reasoning about the goal or the state of the plan, or to surface internal knowledge.",
            "full_description": "generate an internal thought that enriches the planning context for subsequent tool selections. The thought is appended to the internal SGLang conversation state, allowing the LLM to reference it in later reasoning. Thoughts are NOT persisted as Notes, NOT published to the UI, and NOT communicated to the user. Use this for tracking reasoning steps, making observations, or noting intermediate conclusions that inform later decisions.",
            "parameters": {
                "value": "required: literal string or $variable referencing thought prompt"
            },
            "examples": [
                '{"type":"think","value":"The user wants a summary, but how long should it be?"}',
                '{"type":"think","value":"Do I already have the information I need to answer the question?"}',
                '{"type":"think","value":"$observation"}'
            ],
            "schema_hint": {"value": "string or $variable"}
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
            "description": "Load persistent Note or Collection by ID or name. Can load named Notes/Collections by name (e.g., 'my-note') or by ID (e.g., 'Note_123'). Returns prefixed content ('Note Content: <text>' or 'Collection Content: <ids>') to clarify that the result is Note/Collection content, not domain-specific output.",
            "schema_hint": {"target": "string (ID or name) or $variable", "out": "$variable", "expect": "string"}
        },
        "persist": {
            "description": "Mark Note/Collection as persistent. Use this to save the Note/Collection to the filesystem.",
            "schema_hint": {"target": "$variable"}
        },
        "search-notes": {
            "description": "Global search all Notes in the infospace using embedding-based retrieval. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches search-web/semantic-scholar for consistent project operations.",
            "schema_hint": {"value": "string (query)", "out": "$variable", "limit": "int (optional, default 5)", "threshold": "float (optional, default 0.3)"}
        },
        "search-collections": {
            "description": "Global search across all Collections using embedding-based retrieval. Returns Collection of structured Notes with text preview (200 chars), metadata.source_id, metadata.uri, metadata.score, metadata.type. Format matches search-web/semantic-scholar for consistent project operations.",
            "schema_hint": {"value": "string (query)", "out": "$variable", "limit": "int (optional, default 3)", "threshold": "float (optional, default 0.3)"}
        },
        "search-within-collection": {
            "description": "Search within a specific indexed Collection. Returns Collection of structured Notes each containing the full matched chunk text, metadata.source_id, metadata.uri, metadata.score, metadata.type, metadata.chunk_index, metadata.chunk_total. Format matches search-web/semantic-scholar for consistent project operations. Requires Collection to be indexed first.",
            "schema_hint": {"target": "$variable (indexed Collection)", "value": "string (query)", "out": "$variable", "limit": "int (optional, default 5)", "threshold": "float (optional, default 0.0)"}
        },
        "index": {
            "description": "Build embedding index for Collection. Use this when you want to search the Collection later.",
            "schema_hint": {"target": "$variable"}
        },
        "map": {
            "description": "Apply operation (primitive or tool)to each item in Collection. Use this to apply an operation to each item in a Collection.",
            "schema_hint": {"target": "$variable", "operation": "string", "out": "$variable"}
        },
        "split": {
            "description": "Transform Note structure into Collection: JSON array → Collection of Notes (one per element), or plain text → Collection of Notes (default: by sentence). NOT for inspecting Collection contents. Use display or flatten to view Collection data.",
            "full_description": "Split transforms a single Note's internal structure into a Collection. Input must be a Note (not Collection) containing either: (1) JSON array - each element becomes a Note in output Collection, (2) JSON object with array field - extracts array from specified field (default 'results'), (3) JSONL format - multiple JSON objects separated by newlines, or (4) plain text - splits by delimiter (default 'sentence' for semantic processing). For plain text, default delimiter is 'sentence' which splits on sentence boundaries (. ! ? followed by space/newline), normalizes whitespace (removes internal newlines), and filters empty segments. Optional delimiter parameter: 'sentence' (default), 'paragraph' (double newlines), 'line' (single newlines), or custom string. This is a STRUCTURE TRANSFORMATION, not content inspection. To view Collection contents, use display (show to user) or flatten (merge into single Note). Common mistake: trying to split a Collection to 'see inside it' - Collections are already split, use display instead.",
            "examples": [
                '{"type":"split","target":"$json_array_note","out":"$items"}  # [1,2,3] → Collection of 3 Notes',
                '{"type":"split","target":"$text_note","out":"$sentences"}  # "First. Second!" → Collection of 2 Notes (default: sentence splitting)',
                '{"type":"split","target":"$text_note","delimiter":"paragraph","out":"$paragraphs"}  # Split by paragraphs',
                '{"type":"split","target":"$text_note","delimiter":"line","out":"$lines"}  # Split by lines',
                '{"type":"split","target":"$nested_json","out":"$objects"}  # [{"x":1},{"x":2}] → Collection'
            ],
            "schema_hint": {"target": "$variable (Note with array/text)", "delimiter": "string (optional: 'sentence', 'paragraph', 'line', or custom)", "out": "$variable (Collection)"}
        },
        "flatten": {
            "description": "Flatten Collection to single Note.",
            "schema_hint": {"target": "$variable", "out": "$variable"}
        },
        "display": {
            "description": "Display Note content to user",
            "schema_hint": {"value": "$variable"}
        },
        "think": {
            "description": PRIMITIVE_DOCS["think"]["description"],
            "full_description": PRIMITIVE_DOCS["think"].get("full_description"),
            "examples": PRIMITIVE_DOCS["think"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["think"]["schema_hint"]
        },
        "say": {
            "description": "Output text to user. Use this to report progress or results to the user.",
            "schema_hint": {"target": "user", "value": "string"}
        },
        "ask": {
            "description": "Ask user a question and wait for response (suspends plan execution). NOTE: Only works with sync plan execution (manual JSON plans), not with IncrementalPlanner.",
            "schema_hint": {"target": "User (optional)", "value": "string (question text)", "out": "$variable"}
        },
        "add": {
            "description": "Add a Note to an existing Collection (mutates Collection in place). When used with map to add multiple Notes, use collection parameter: {\"type\":\"map\",\"target\":\"$notes\",\"operation\":\"add\",\"collection\":\"$collection\",\"out\":\"$collection\"}",
            "schema_hint": {"target": "$variable (Collection)", "value": "$variable or literal", "out": "$variable"}
        },
        "remove": {
            "description": "Remove a Note from a Collection (mutates Collection)",
            "schema_hint": {"target": "$variable (Collection)", "value": "$variable or Note ID", "out": "$variable"}
        },
        "union": {
            "description": "Union of two Collections (A ∪ B) - all items from both, deduplicated",
            "schema_hint": {"target": "$variable (Collection A)", "value": "$variable (Collection B)", "out": "$variable"}
        },
        "intersection": {
            "description": "Intersection of two Collections (A ∩ B) - items in both",
            "schema_hint": {"target": "$variable (Collection A)", "value": "$variable (Collection B)", "out": "$variable"}
        },
        "difference": {
            "description": "Difference of two Collections (A - B) - items in A but not in B",
            "schema_hint": {"target": "$variable (Collection A)", "value": "$variable (Collection B)", "out": "$variable"}
        },
        "size": {
            "description": "Get item count of a Collection",
            "schema_hint": {"target": "$variable (Collection)", "out": "$variable"}
        },
        "project": {
            "description": "Extract metadata/structured fields from each Note in Collection (SQL SELECT). Extracts NAMED FIELDS only (metadata.uri, metadata.title, etc.), does NOT parse unstructured text. For extracting info FROM text content, use refine instead.",
            "full_description": "Project operation extracts specified fields from each Note in a Collection, similar to SQL SELECT. Input Collection must contain JSON/dict Notes. Output is a new Collection of projected Notes containing only the requested fields. Notes missing any requested field are excluded. Nested fields use dot notation (e.g., 'metadata.uri'). IMPORTANT: project is a field accessor for structured data - it extracts metadata fields (metadata.uri, metadata.title, metadata.year) or top-level JSON fields. It does NOT interpret or parse unstructured text content. To extract information FROM text (e.g., 'extract the nationality'), use refine (LLM-based).",
            "examples": [
                '{"type":"project","target":"$search_results","fields":["metadata.uri"],"out":"$urls"}',
                '{"type":"project","target":"$papers","fields":["metadata.title","metadata.year"],"out":"$paper_info"}',
                '{"type":"project","target":"$results","fields":["metadata.source_id","metadata.score"],"out":"$filtered"}'
            ],
            "schema_hint": {"target": "$variable (Collection of dict Notes)", "fields": "array of field paths (strings)", "out": "$variable"}
        },
        "pluck": {
            "description": "Extract single field value from each Note in Collection, returning Collection of simple values. Similar to project but returns raw values instead of dict Notes. Use when you need just one field as simple values (strings, numbers).",
            "full_description": "Pluck extracts a single field from each Note in a Collection and returns a Collection of Notes containing just that field's value. Unlike project (which returns dict Notes), pluck returns simpler Notes. Notes missing the field are excluded. Use pluck for extracting a single attribute as values, use project for multiple fields as structured Notes.",
            "examples": [
                '{"type":"pluck","target":"$papers","field":"title","out":"$titles"}',
                '{"type":"pluck","target":"$results","field":"metadata.score","out":"$scores"}'
            ],
            "schema_hint": {"target": "$variable (Collection of dict Notes)", "field": "string (field path)", "out": "$variable"}
        },
        "head": {
            "description": "Take first N items from Collection (default 1). Preserves original Notes. Use after sort to get top-ranked items.",
            "full_description": "Head returns a new Collection containing the first N items from the input Collection. Items are taken in their current order, so combine with sort to get top/bottom ranked items. Default count is 1. All original Note content is preserved.",
            "examples": [
                '{"type":"head","target":"$sorted_papers","count":5,"out":"$top_5"}',
                '{"type":"head","target":"$results","out":"$first_result"}',
                '{"type":"head","target":"$ranked_items","count":10,"out":"$top_10"}'
            ],
            "schema_hint": {"target": "$variable (Collection)", "count": "int (optional, default 1)", "out": "$variable"}
        },
        "sort": {
            "description": "Sort Collection by a field value (SQL ORDER BY). Notes must be JSON/dict with sortable field. Default ascending order, use order:'desc' for descending.",
            "full_description": "Sort a Collection by comparing a field in each Note. Input Collection must contain JSON/dict Notes with the specified field. Field must contain sortable values (numbers, strings, dates). Default is ascending order (A-Z, 0-9), set order:'desc' for descending (Z-A, 9-0). Notes missing the sort field are placed at end.",
            "examples": [
                '{"type":"sort","target":"$papers","by":"metadata.year","out":"$sorted_papers"}',
                '{"type":"sort","target":"$results","by":"metadata.score","order":"desc","out":"$ranked"}',
                '{"type":"sort","target":"$items","by":"title","out":"$alphabetical"}'
            ],
            "schema_hint": {"target": "$variable (Collection of dict Notes)", "by": "string (field path)", "order": "string (optional, 'asc' or 'desc', default 'asc')", "out": "$variable"}
        },
        "filter-structured": {
            "description": "Filter Collection by field conditions (SQL WHERE clause). Use SQL-like syntax: field > value, field >= value, field == value, field != value, field < value, field <= value. Supports AND/OR for multiple conditions.",
            "full_description": "Filter a Collection using SQL-like WHERE clause syntax. Input Collection must contain JSON/dict Notes with the specified fields. WHERE clause syntax: 'field > 100', 'year >= 2020', 'score == 1.0', 'title != null'. Multiple conditions: 'year > 2020 AND citations >= 100'. Field paths use dot notation: 'metadata.year > 2020'. Notes missing fields are excluded from results.",
            "examples": [
                '{"type":"filter-structured","target":"$papers","where":"metadata.year >= 2020","out":"$recent"}',
                '{"type":"filter-structured","target":"$results","where":"metadata.score > 0.5","out":"$high_quality"}',
                '{"type":"filter-structured","target":"$papers","where":"metadata.citations >= 100 AND metadata.year < 2025","out":"$highly_cited_recent"}'
            ],
            "schema_hint": {"target": "$variable (Collection of dict Notes)", "where": "string (SQL WHERE clause)", "out": "$variable"}
        },
        "join": {
            "description": "Join two Collections on a common field (SQL JOIN). Creates new Collection of merged Notes where field values match. Inner join: only matching pairs included.",
            "full_description": "Join two Collections by matching a field in Notes from both Collections. Input Collections must contain JSON/dict Notes with the specified join field. For each Note in target (left) Collection, finds matching Notes in value (right) Collection where field values are equal. Creates new Notes by merging matched pairs (right fields overwrite left on conflict). Only matched pairs are included (inner join). Use for combining related data from different sources.",
            "examples": [
                '{"type":"join","target":"$papers","value":"$citations","on":"paper_id","out":"$papers_with_citations"}',
                '{"type":"join","target":"$users","value":"$profiles","on":"user_id","out":"$user_profiles"}'
            ],
            "schema_hint": {"target": "$variable (left Collection)", "value": "$variable (right Collection)", "on": "string (field name to join on)", "out": "$variable"}
        },
        "coerce": {
            "description": "Convert Note content to different type/format. Supports: to-string, to-int, to-float, to-bool, to-json (parse JSON string), to-list (split string or wrap value). Use for type conversion before operations requiring specific types.",
            "full_description": "Coerce converts a Note's content to a different type. Input is a Note (not Collection). Coercion types: 'to-string' (any -> string), 'to-int' (string/number -> integer), 'to-float' (string/number -> float), 'to-bool' (string/number -> boolean), 'to-json' (JSON string -> parsed object), 'to-list' (string -> split by delimiter, or value -> [value]). Use when operations require specific types (e.g., arithmetic needs numbers, sort needs comparable types).",
            "examples": [
                '{"type":"coerce","target":"$count_string","coercion":"to-int","out":"$count_number"}',
                '{"type":"coerce","target":"$json_string","coercion":"to-json","out":"$parsed_object"}',
                '{"type":"coerce","target":"$csv_line","coercion":"to-list","delimiter":",","out":"$fields"}'
            ],
            "schema_hint": {"target": "$variable (Note)", "coercion": "string (to-string|to-int|to-float|to-bool|to-json|to-list)", "delimiter": "string (optional, for to-list)", "out": "$variable"}
        }
    }
    
    for name, meta in primitive_tools.items():
        core_tools[name] = {
            "fn": None,  # Placeholder - execution via infospace_executor
            "description": meta["description"],
            "schema_hint": meta["schema_hint"],
            "source": "core"
        }
        # Include full_description and examples if available
        if "full_description" in meta:
            core_tools[name]["full_description"] = meta["full_description"]
        if "examples" in meta:
            core_tools[name]["examples"] = meta["examples"]
    
    # Enhanced descriptions to prevent common confusions
    TOOL_DISAMBIGUATION = {
        "search-web": "Search web and return Collection of structured Notes. Each Note has text (full content), format, metadata.uri (URL), metadata.domain, char_count. Use project with metadata.uri to extract URLs.",
        "semantic-scholar": "Search academic papers and return Collection of structured Notes. Each Note has text (abstract), format, metadata.uri (PDF URL), metadata.title, metadata.authors, metadata.year, metadata.citations, metadata.venue. Use project with metadata.uri to extract URLs.",
        "fetch-text": "Fetch text from a SINGLE specific URL, Do NOT use on search-web or semantic-scholar results. Use ONLY when you have one URL/ID to fetch directly and do not already have the text.",
        "search-notes": "Global search across all Notes. Returns Collection of structured Notes with full text content, metadata.source_id, metadata.uri, metadata.score, metadata.type. Use project for metadata fields, refine for extracting info from text.",
        "search-collections": "Global search across all Collections. Returns Collection of structured Notes with full text content, metadata.source_id, metadata.uri, metadata.score, metadata.type. Use project for metadata fields, refine for extracting info from text.",
        "search-within-collection": "Search within indexed Collection. Returns Collection of structured Notes with full matched chunk text, metadata.source_id, metadata.uri, metadata.score, metadata.type, metadata.chunk_index, metadata.chunk_total. Format matches search-web/semantic-scholar.",
    }
    TOOL_SCHEMA_HINT_OVERRIDE = {}
    
    # Add available tools from map
    for tool_name, tool_meta in available_tools.items():
        param_source = tool_meta.get('parameter_source')
        schema_hint = {}
        
        # First, check if schema_hint is explicitly defined in SKILL.md frontmatter
        if 'schema_hint' in tool_meta:
            schema_hint = tool_meta['schema_hint'].copy()
        # Otherwise, fall back to parameter_source (for backward compatibility)
        elif param_source and param_source.startswith('args.'):
            param_name = param_source.split('.', 1)[1]
            schema_hint[param_name] = "string"
        
        # Override takes precedence (for tools that need special handling)
        override_hint = TOOL_SCHEMA_HINT_OVERRIDE.get(tool_name)
        if override_hint:
            schema_hint = override_hint.copy()
        
        # Use enhanced description if available, otherwise use original
        description = TOOL_DISAMBIGUATION.get(tool_name, tool_meta.get('description', 'No description'))
        
        # Extract type from tool metadata (important for method tools)
        tool_type = tool_meta.get('type', 'code_execution')
        
        entry = {
            "fn": None,  # Placeholder
            "description": description,
            "schema_hint": schema_hint,
            "type": tool_type,
            "source": _infer_tool_source(tool_meta)
        }
        
        if entry["source"] == "core":
            core_tools[tool_name] = entry
        else:
            world_tools[tool_name] = entry
    
    # Merge: core first, then world-specific
    tools.update(core_tools)
    tools.update(world_tools)
    return tools


def tool_catalog_text(tools: Dict[str, Dict]) -> str:
    """Format tool catalog for LLM prompt, grouped by source."""
    lines = []
    
    # Group tools by source
    world_tools = {}  # world_name -> [(tool_name, meta), ...]
    core_tools = []   # [(tool_name, meta), ...]
    
    for name, meta in tools.items():
        source = meta.get('source', 'core')
        if source == 'core':
            core_tools.append((name, meta))
        else:
            if source not in world_tools:
                world_tools[source] = []
            world_tools[source].append((name, meta))
    
    # Helper function to format a tool entry
    def format_tool(name: str, meta: Dict) -> List[str]:
        tool_lines = []
        description = meta.get('full_description') or meta.get('description', 'No description')
        tool_lines.append(f"- {name}: {description}")
        
        # Add examples for primitives with expanded docs
        if 'examples' in meta and meta['examples']:
            tool_lines.append(f"  examples:")
            for ex in meta['examples']:
                # Format example as single-line JSON if it's a dict or string
                if isinstance(ex, dict):
                    ex_str = json.dumps(ex, separators=(',', ':'))
                elif isinstance(ex, str):
                    # Try to parse and re-serialize as single-line if it's JSON
                    try:
                        parsed = json.loads(ex)
                        ex_str = json.dumps(parsed, separators=(',', ':'))
                    except (json.JSONDecodeError, TypeError):
                        ex_str = ex
                else:
                    ex_str = str(ex)
                tool_lines.append(f"    {ex_str}")
        return tool_lines
    
    # List world tools first, grouped by world name
    for world_name in sorted(world_tools.keys()):
        lines.append(f"#{world_name.upper()}")
        for name, meta in sorted(world_tools[world_name]):
            lines.extend(format_tool(name, meta))
        lines.append("")  # Blank line between sections
    
    # List core tools under INFOSPACE CORE
    if core_tools:
        lines.append("#INFOSPACE CORE")
        for name, meta in sorted(core_tools):
            lines.extend(format_tool(name, meta))
        lines.append("")  # Blank line before workflows
    
    lines.append("Preferred tool order: World tools first when they apply, then Infospace Core")
    # Add critical workflows to prevent common mistakes
    lines.append("# CRITICAL WORKFLOWS:")
    lines.append("- search-web → summarize / refine / filter-collection")
    lines.append("- search-web already returns full text in 'text' field of each Note")
    lines.append("- semantic-scholar → summarize / refine / filter-collection")
    lines.append("- fetch-text is for SINGLE URLs only, NOT for Collections from search-web/semantic-scholar")
    lines.append("- Level 4 tools (search-web, semantic-scholar) return Collections with text content in the 'text' field of each Note")
    
    return "\n".join(lines)


def load_skill_docs(tool_names: List[str], available_tools: Dict[str, Dict]) -> str:
    """
    Load full SKILL.md documentation for selected tools and primitives.
    
    Args:
        tool_names: List of tool names to load docs for
        available_tools: Dict mapping tool_name -> metadata (includes 'tool_path')
        
    Returns:
        Formatted string with full tool documentation
    """
    lines = []
    lines.append("# DETAILED TOOL DOCUMENTATION")
    lines.append("Full documentation for selected tools with examples, patterns, and output schemas:\n")
    
    # Get primitives directory path
    primitives_dir = Path(__file__).parent / 'primitives'
    
    for tool_name in tool_names:
        logger.debug(f"Stage 1.5: Processing {tool_name}")
        
        skill_file = None
        
        # Check if it's a regular tool
        if tool_name in available_tools:
            tool_meta = available_tools[tool_name]
            # Use 'path' field which is the tool directory (not 'python_file' which is tool.py)
            tool_dir_path = tool_meta.get('path')
            logger.debug(f"Stage 1.5: {tool_name} path = {tool_dir_path}")
            
            if tool_dir_path:
                # Resolve to absolute path
                tool_dir = Path(tool_dir_path).resolve()
                logger.debug(f"Stage 1.5: {tool_name} tool_dir = {tool_dir}")
                
                # Look for SKILL.md or Skill.md in tool directory
                for variant in ['SKILL.md', 'Skill.md', 'skill.md']:
                    candidate = tool_dir / variant
                    if candidate.exists():
                        skill_file = candidate
                        logger.debug(f"Stage 1.5: {tool_name} found {variant} at {skill_file}")
                        break
        else:
            # Check if it's a primitive with Skill.md
            logger.debug(f"Stage 1.5: {tool_name} not in available_tools, checking primitives")
            primitive_dir = primitives_dir / tool_name
            if primitive_dir.exists():
                for variant in ['SKILL.md', 'Skill.md', 'skill.md']:
                    candidate = primitive_dir / variant
                    if candidate.exists():
                        skill_file = candidate
                        logger.debug(f"Stage 1.5: {tool_name} found primitive {variant} at {skill_file}")
                        break
        
        if not skill_file:
            logger.debug(f"Stage 1.5: No SKILL.md found for {tool_name}")
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
            logger.debug(f"Stage 1.5: Loaded docs for {tool_name} ({len(content)} chars)")
            
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


def repair_json_string(json_str: str) -> Optional[Dict]:
    """
    Attempt to repair malformed JSON from LLM output.
    
    Handles common LLM JSON errors:
    - Trailing extra braces
    - Missing closing braces
    - Code fences
    - Newlines in wrong places
    - Arithmetic expressions in numeric fields (e.g., "1.0236+3.14159")
    
    Args:
        json_str: Potentially malformed JSON string
        
    Returns:
        Parsed dict if successful, None if repair fails
    """
    if not json_str:
        return None
    
    response = json_str.strip()
    
    # Remove markdown code fences if present
    response = response.replace("```json", "").replace("```", "").strip()
    
    # Pre-process: Evaluate arithmetic expressions in numeric fields before parsing
    # This handles cases like {"yaw": 1.0236+3.14159} which are invalid JSON
    def eval_arithmetic_in_json(text):
        """Find and evaluate arithmetic expressions in JSON-like strings."""
        import re
        # Pattern: "key": value where value might be an arithmetic expression
        # Match unquoted numeric expressions after colons
        # Pattern matches: "key": 1.23+4.56 or "key": "1.23+4.56"
        pattern = r'"([^"]+)":\s*("?)([0-9.+\-*/().\s]+)("?)'
        
        def replace(match):
            key = match.group(1)
            open_quote = match.group(2)
            expr = match.group(3).strip()
            close_quote = match.group(4)
            
            # Only process if it contains arithmetic operators and isn't a variable reference
            if any(op in expr for op in ['+', '-', '*', '/']) and not expr.startswith('$'):
                try:
                    import ast
                    result = ast.literal_eval(expr)
                    if isinstance(result, (int, float)):
                        return f'"{key}": {result}'
                except (ValueError, SyntaxError, TypeError):
                    pass
            
            # Return original if evaluation fails
            return match.group(0)
        
        return re.sub(pattern, replace, text)
    
    # Try evaluating arithmetic expressions before parsing
    response = eval_arithmetic_in_json(response)
    
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
    
    # Repair attempt 3: Find first complete JSON object by brace counting
    brace_count = 0
    json_end = 0
    in_string = False
    for i, char in enumerate(response):
        if char == '"' and (i == 0 or response[i-1] != '\\'):
            in_string = not in_string
        if not in_string:
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    json_end = i + 1
                    break
    
    if json_end > 0:
        response = response[:json_end]
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            pass
    
    # Repair attempt 4: Add missing closing braces
    if brace_count > 0:
        response = response + ('}' * brace_count)
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            pass
    
    # Repair attempt 5: Strip trailing extra braces
    while response.endswith('}}'):
        trimmed = response[:-1]
        try:
            return json.loads(trimmed)
        except json.JSONDecodeError:
            response = trimmed
    
    return None


def safe_eval_numeric(value):
    """
    Safely evaluate simple arithmetic expressions to numeric values.
    
    Only evaluates strings that contain arithmetic operators and produces numeric results.
    Uses ast.literal_eval() for safety (no function calls, no imports).
    
    Args:
        value: Value to evaluate (int, float, or string)
        
    Returns:
        Numeric value if evaluation succeeds, original value otherwise
    """
    if isinstance(value, (int, float)):
        return value
    
    if isinstance(value, str):
        # Only evaluate if it looks like arithmetic (contains operators)
        # Skip if it's a variable reference, URL, or other non-numeric string
        if any(op in value for op in ['+', '-', '*', '/', '(', ')']) and not value.startswith('$'):
            try:
                import ast
                # ast.literal_eval only evaluates literals and simple expressions
                # It's safe - no function calls, no imports, no side effects
                result = ast.literal_eval(value)
                if isinstance(result, (int, float)):
                    logger.debug(f"Evaluated numeric expression '{value}' -> {result}")
                    return result
            except (ValueError, SyntaxError, TypeError) as e:
                logger.debug(f"Could not evaluate '{value}' as numeric expression: {e}")
                # Return original value if evaluation fails
                pass
    
    return value


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
        parsed = json.loads(args_json) if args_json else {}
        # Ensure args is always a dict (handle case where JSON parses to int/str/etc)
        # Tool arguments must be JSON objects, not primitives
        if isinstance(parsed, dict):
            args = parsed
        else:
            logger.warning(f"Tool {tool_name} args JSON parsed to non-dict type {type(parsed).__name__}: {parsed} (raw: {args_json[:100]}), using empty dict")
            args = {}
    except json.JSONDecodeError as e:
        # Try robust JSON repair
        repaired = repair_json_string(args_json)
        if repaired is not None:
            args = repaired if isinstance(repaired, dict) else {}
            if not isinstance(repaired, dict):
                logger.warning(f"Repaired JSON is non-dict type {type(repaired).__name__}: {repaired}, using empty dict")
            else:
                logger.info(f"Repaired malformed JSON for {tool_name}")
        else:
            logger.warning(f"Failed to parse/repair args_json: {args_json}")
            logger.debug(f"JSON error: {e}")
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
    # Primitives/tools that accept literal values in 'value' field (don't normalize)
    literal_value_primitives = [
        'create-note', 'create-collection', 'add', 'remove', 'say', 'display', 'think', 'ask',
        'search-web', 'semantic-scholar', 'search-notes', 'search-collections',
    ]
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
    
    # Resolve numeric expressions in known numeric fields
    # Common numeric fields across tools
    numeric_fields = [
        # Minecraft tools
        'yaw', 'pitch', 'duration', 'forward', 'back', 'right', 'left', 'up', 'down',
        'x', 'y', 'z', 'rel_x', 'rel_y', 'rel_z', 'radius', 'blocks_radius', 'entities_radius',
        'count', 'limit', 'threshold', 'compression_ratio',
        # General numeric fields
        'precision', 'step', 'offset'
    ]
    
    for field in numeric_fields:
        if field in args:
            original_value = args[field]
            evaluated_value = safe_eval_numeric(original_value)
            if evaluated_value != original_value:
                args[field] = evaluated_value
                logger.info(f"Resolved numeric expression in {tool_name}.{field}: '{original_value}' -> {evaluated_value}")
    
    # Build action - ALL fields at top level (flat format)
    action = {"type": tool_name}
    action.update(args)
    
    # Ensure 'out' field if tool produces output
    output_producing = ["create-note", "create-collection", "load", "search-notes", "search-collections", "search-within-collection", "map", 
                       "split", "flatten", "search-web", "semantic-scholar", "summarize",
                       "refine", "generate-note", "assess", "relate", "extract-entities", "filter-collection",
                       "fetch-text", "as-json", "as-markdown"]
    if tool_name in output_producing and "out" not in action:
        action["out"] = f"$step_{step}_result"
    
    # Add expect if needed
    uncertain_tools = ["search-web", "semantic-scholar", "search-notes", "search-collections", "search-within-collection", "load"]
    if tool_name in uncertain_tools and "expect" not in action:
        action["expect"] = f"should get result from {tool_name}"
    
    return action


def execute_infospace_action(action: Dict, executor: InfospaceExecutor, agent_name: str) -> str:
    """
    Execute single action via infospace_executor, return result text.
    
    Args:
        action: Infospace action dict
        executor: InfospaceExecutor instance
        agent_name: Agent name for logging
        
    Returns:
        Result text for Stage 3 reflection (format: SUCCESS | <result> | <action> | Bound: <var>)
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
            
            # Format result string for backward compatibility
            result_str = result.get('value', '') if result.get('status') == 'success' else result.get('reason', 'failed')
            
            action_record = ActionRecord(
                action=action,
                result=result_str,
                result_dict=result.copy(),  # Store full uniform format
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
            executor._compliance_tracker.check_action(action, result, executor.plan_bindings_flat)
        
        if result.get('status') == 'success':
            # Extract value and resource_id from uniform return format
            value = result.get('value', '')
            resource_id = result.get('resource_id')
            
            # Get bound variable
            bound_var = action.get('out', '')
            action_type = action['type']
            
            # Format value for display (already truncated to 384 chars in executor)
            if value:
                # Replace newlines with space-pipe-space for readability
                value_str = str(value).replace('\n', ' | ')
            else:
                value_str = ''
            
            # Build result message
            if value_str:
                if bound_var and resource_id:
                    return f"SUCCESS | {value_str} | {action_type} completed | Bound: {bound_var} to {resource_id}"
                elif bound_var:
                    return f"SUCCESS | {value_str} | {action_type} completed | Bound: {bound_var}"
                else:
                    return f"SUCCESS | {value_str} | {action_type} completed"
            else:
                # No value to show, use resource_id if available
                if bound_var and resource_id:
                    return f"SUCCESS | {action_type} completed | Bound: {bound_var} to {resource_id}"
                elif bound_var:
                    return f"SUCCESS | {action_type} completed | Bound: {bound_var}"
                else:
                    return f"SUCCESS | {action_type} completed"
        else:
            # Increment error counter
            if hasattr(executor, '_plan_error_count'):
                executor._plan_error_count += 1
            error_reason = result.get('reason', 'Unknown error')
            return f"ERROR | {action['type']} failed: {error_reason}"
    except Exception as e:
        # Increment error counter for exceptions
        if hasattr(executor, '_plan_error_count'):
            executor._plan_error_count += 1
        logger.error(f"Execution error: {e}")
        traceback.print_exc()
        return f"ERROR | Exception: {str(e)}"


if HAS_SGLANG:
    #@function
    def stage0_resource_retrieval(goal: str, executor):
        """
        Stage 0: Generate search queries from goal and retrieve relevant resources.
        
        Args:
            goal: Goal text
            executor: InfospaceExecutor instance
            
        Returns:
            Formatted string with available resources to inject into Stage 1 prompt
        """
        
        try:
            queries = [goal]
            
            # Search for resources
            search_result = executor.search_resources(queries, k_notes=3, k_collections=2, threshold=0.3)
            
            if search_result.get('status') != 'success':
                reason = search_result.get('reason', 'Unknown error')
                # Don't warn if queryable isn't ready yet or if indexes are empty (normal on startup)
                if 'queryable may not be registered' in reason.lower() or 'no response' in reason.lower():
                    logger.warning(f"Stage 0: Resource search unavailable (normal on startup): {reason}")
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
            lines = ["#Available Notes / Collections that may be relevant)\n"]
            
            if notes:
                lines.append("## Notes:")
                for note in notes:
                    name = note.get('name', note.get('resource_id', ''))
                    resource_id = note.get('resource_id', '')
                    props = note.get('properties', {})  # Properties extracted separately, no ResourceType
                    
                    # Build description from metadata
                    desc_parts = []
                    
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
            traceback.print_exc()
            return ""
    
    def _extract_goal_for_step(goal: str) -> str:
        """
        Extract goal text up to ## CONTEXT ## delimiter for use in STAGE 2 prompts.
        Keeps goal visible but prevents context explosion from large TRACE sections.
        
        Args:
            goal: Full goal text (may contain ## CONTEXT ## delimiter)
            
        Returns:
            Goal text up to ## CONTEXT ##, or full goal if delimiter not found
        """
        if '## CONTEXT ##' in goal:
            return goal.split('## CONTEXT ##')[0].strip()
        return goal
    
    @function
    def tool_planner_infospace(s, template, goal: str, world_model: Dict, character_context: str, recent_context: str, 
                              tools_catalog_text: str, executor, trace_file=None, max_steps: int = 16, similar_plan: Dict = None, preplan: str = None):
        """
        SGLang incremental planner for infospace goals.
        
        Args:
            s: SGLang state
            template: Template text
            goal: Goal text
            character_context: Character description + drives
            recent_context: Recent thoughts/memories + last action
            tools_catalog_text: Formatted tool catalog
            executor: InfospaceExecutor instance (with _plan_actions attribute)
            max_steps: Maximum planning steps
        """
        # Extract goal for step prompts (truncate at ## CONTEXT ## if present)
        goal_for_step = _extract_goal_for_step(goal)
        
        # Track which tools have had their skill docs loaded (re-initialized each call, persists across stages)
        # Store in state so it persists across stages within this call, but reset at start of each new call
        _loaded_skill_docs = set[Any]()          
        
        # Stage 0: Resource retrieval (if executor available)
        available_resources_text = ""
        if executor:
            try:
                available_resources_text = stage0_resource_retrieval(goal=goal, executor=executor)  
            except Exception as e:
                logger.warning(f"Stage 0: Failed to retrieve resources: {e}")
        
        # Stage 1: Analysis + tool selection
        system_parts = [f"Your task is to achieve\n#GOAL:\n{goal}\n\n"]
        system_parts.append("You can choose tools/primitives (aka actions), if and as needed, callling them with JSON arguments,")
        system_parts.append("and loop over execute-step / reflect / refine until the goal is satisfied.")
        system_parts.append(f"\n{INCREMENTAL_PLAN_SPECIFICATIONS}\n")
        system_parts.append(f"Complete primitive and tool catalog:\n{tools_catalog_text}\n#### END OF INFOSPACE TYPE SYSTEM, SPECIFICATIONS, AND TOOL CATALOG\n\n")

        system_parts.append(f"Setting:\n{character_context}\n\n")
        #system_parts.append(f"Again, \n#GOAL:\n\n{template}\n{goal}\n\n")
        if preplan:
            system_parts.append(f"\n## {preplan}\n")
            system_parts.append(f"\n## End ABSTRACT_PLAN\n")

        if similar_plan:
            system_parts.append(f"##PREVIOUS PLAN FOR SIMILAR GOAL:\n{similar_plan['plan']}\n")
            system_parts.append(f"OUTCOME: {similar_plan['outcome']} ERRORS: {similar_plan['error_count']}\n")
        if available_resources_text:
            system_parts.append(f"\n{available_resources_text}\n")
        # Add current date and time

        system_parts.append(f"WORLD_MODEL: {json.dumps(world_model, indent=2)}\n")
        system_parts.append(f"CURRENT_TIME: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n")
        system_parts.append("""Follow this process to achieve the goal:
 - Stage 1 (once): Analyze goal, select relevant tools, decompose into FIRST_TASK.
 - Stage 1.5 (once): Load and inject detailed docs for selected tools.
Then you will work in repeated cycles to achieve the goal:
 - Stage 2: Pick a single tool and JSON args for CURRENT_TASK. Be concise in text value arguments.
 - Stage 3: Reflect on result, decide if goal done, set NEXT_TASK.
ALWAYS follow all formatting instructions exactly.

""")        
        s += system("".join(system_parts))
        
        s += user(
            "#Stage 1: Analyze goal and identify relevant tools from the Complete primitive and tool catalog.\n"
            "Include tools you might need AND related/supporting tools.\n"
            "Then, decompose the goal into a FIRST high-level task/subgoal to focus on.\n"
            "In doing so, consider the tools you have selected, the goal you are trying to achieve, and the downstream tasks that will be required to achieve the goal.\n"
            "Respond using the following XML format:\n"
            "<analysis>\n"
            "YOUR REASONING AND THOUGHTS HERE.\n"
            "</analysis>\n"
            "<tools>\n"
            "JSON LIST OF TOOLS HERE\n"
            "</tools>\n"
            "<first_task>\n"
            "YOUR FIRST TASK HERE\n"
            "</first_task>\n"
        )
        
        s += assistant(
            "<analysis>\n"
            + gen("stage1_analysis", max_tokens=192, temperature=GEN_TEMPERATURE, stop="</analysis>")
            + "</analysis>\n"
            "<tools>\n"
            + gen("selected_tools_json", max_tokens=96, temperature=GEN_TEMPERATURE, stop="</tools>")
            + "</tools>\n"
            "<first_task>\n"
            + gen("first_task", max_tokens=96, temperature=GEN_TEMPERATURE, stop="</first_task>")
            + "</first_task>\n"
        )
        
        try:
            logger.info(f"Stage 1: Analysis + tool selection\n{s['stage1_analysis']}")
            logger.info(f"SELECTED_TOOLS_JSON: {s['selected_tools_json']}")
            logger.info(f"FIRST_TASK: {s['first_task']}")
        except KeyError as e:
            logger.warning(f"Stage 1 values not available: {e}")
        
        # Stage 1.5: Load and inject detailed docs for selected tools (only if not already loaded)
        try:
            selected_tools_json = s['selected_tools_json']
            # Use robust parsing that handles malformed JSON
            selected_tools = parse_request_tools(selected_tools_json)
            if selected_tools and isinstance(selected_tools, list) and selected_tools:
                # Filter to only tools that haven't had docs loaded yet
                tools_to_load = [tool for tool in selected_tools if tool not in _loaded_skill_docs]
                
                if tools_to_load:
                    expanded_docs = load_skill_docs(tools_to_load, executor.available_tools)
                    if expanded_docs:
                        s += user(expanded_docs)
                        s += assistant("I have reviewed the detailed documentation for the selected tools.\n")
                        logger.info(f"Stage 1.5: Injected detailed docs for {len(tools_to_load)} tools")
                        # Mark these tools as loaded
                        _loaded_skill_docs.update(tools_to_load)
                else:
                    logger.debug(f"Stage 1.5: All {len(selected_tools)} selected tools already have docs loaded, skipping")
        except (KeyError, json.JSONDecodeError, TypeError) as e:
            logger.warning(f"Failed to parse selected tools for doc expansion: {e}")
        
        # Stage 2/3 format instructions
        s += user(
            "#Stage 2-PRE: Instructions\n"
            "Before choosing TOOL_NAME/ARGS for this step, you MUST first produce AGENT_STATE_HYPOTHESES.\n"
            "Purpose:\n"
            "- Make 'context' explicit as TRANSIENT beliefs about the agent's current\n"
            "  affordances, constraints, or limitations.\n"
            "- These hypotheses exist ONLY for the current planning attempt.\n"
            "- They are NOT persistent world facts and must NOT be promoted to memory.\n"
            "- They should meaningfully influence tool choice when more than one tool\n"
            "  could plausibly apply.\n"
            "\n"    
            "# STAGE 2-PRE FORMAT (STRICT):\n"
            "AGENT_STATE_HYPOTHESES: [<h1>, <h2>, ...]\n"
            "AGENT_STATE_SUPPORT:\n"
            "  - <h1>: <evidence pointer: tool result field / note id / variable / or \"none\">\n"
            "  - <h2>: <evidence pointer ...>\n"
            "\n"
            "# STAGE 2-PRE RULES (STRICT):\n"
            "- Max 6 hypotheses.\n"
            "- Each hypothesis MUST be falsifiable (something a tool call or note could contradict).\n"
            "- Prefer CONSTRAINTS, LIMITATIONS, or MISSING AFFORDANCES over confirmations.\n"
            "- Each hypothesis should plausibly matter: if it were false, a different tool  choice might be reasonable.\n"
            "- Hypotheses may reference ONLY information already observed before this step; they MUST NOT depend on the outcome of the tool about to be chosen.\n"
            "- Hypotheses MUST describe epistemic or physical state of the agent (e.g., known/unknown, stable/unstable, blocked/unblocked); NOT communicative, procedural, or planning affordances.\n"
            "- DO NOT describe future actions, plans, or tools.\n"
            "- DO NOT restate the CURRENT_TASK or GOAL.\n"
            "- DO NOT include generic truths about the system unless they constrain choice.\n"
            "- These are NOT world-model facts.\n"
            "\n"
            "# OPTIONAL NULL CONTEXT:\n"
            "- If the next step is logically forced (only one reasonable tool applies),\n"
            "  you MAY output:\n"
            "    AGENT_STATE_HYPOTHESES: []\n"
            "    AGENT_STATE_SUPPORT: []\n"
            "\n"
            "Examples (do not include world annotations in your response):\n"       
            "- 'Inventory contains at least one placeable solid block.' (minecraft)\n"
            "- 'No placeable solid blocks are available in inventory.' (minecraft)\n"
            "- 'Current position appears stable (on ground) rather than falling.' (minecraft)\n"
            "- 'query-web returned no useful results.' (infospace)\n"
            "- 'there is no more space in /data' (osworld)'\n"
            ""
            "#Stage 2 Instructions:\n"
            "- Review the ABSTRACT_PLAN and determine progress on it towards the goal.\n"
            "- Review the current AGENT_STATE_HYPOTHESES and AGENT_STATE_SUPPORT.\n"
            "- Choose the next tool and its JSON args, from the Complete primitive and tool catalog, based on the current task, suggested ABSTRACT_PLAN, and the AGENT_STATE_HYPOTHESES and AGENT_STATE_SUPPORT.\n"
            "#Stage 2 FORMAT:\n"
            "  TOOL_NAME: <name from the Complete primitive and tool catalog>\n"
            "  TOOL_ARGS_JSON: <json object>\n\n"
            "#Stage 2 NUMERIC ARGUMENTS:\n"
            "  IMPORTANT: All numeric tool arguments (integers, floats) must be simple literals.\n"
            "  Perform calculations in the THOUGHTS block first, then pass only the final computed value.\n"
            "  Example: If you need yaw = current_yaw + pi, calculate it mentally (e.g., 1.0236 + 3.14159 = 4.16519),\n"
            "           then use {\"yaw\": 4.16519} - NOT {\"yaw\": \"1.0236+3.14159\"}.\n"
            "  Arithmetic expressions in JSON will cause parsing errors.\n\n"
            "#Stage 3 FORMAT:\n"
            "  THOUGHTS: <text>\n"
            "  HYPOTHESES: [<hypothesis1>, <hypothesis2>, ...]\n"
            "  AUDIT: for each hypothesis in HYPOTHESES:\n"
            "    <hypothesis text>\n"
            "    scope: <observational | observed-set | global>\n"
            "    verdict: <SUPPORTED | UNSUPPORTED | CONTRADICTED>\n"
            "  DONE: <YES or NO - is the entire GOAL satisfied?>\n"
            "  NEXT_TASK: <next high-level subgoal, or blank if DONE=YES>\n"
            "  REQUEST_TOOLS: <json array of tool names or empty array []>\n"
            "#Stage 3 INSTRUCTIONS:\n"
            "  HYPOTHESIS Instructions:\n"
            "   - Identify unverified beliefs influencing your next step. Consider the following categories of hypotheses:\n"
            "      1. DATA: Claims about completeness or optimality (e.g., 'These are the only results').\n"
            "      2. SYSTEM: Beliefs about tool logic or failures (e.g., 'The tool failed because the query was too long').\n"
            "      3. WORLD: Causal theories about the goal (e.g., 'The user likely wants the cheapest option, not the fastest').\n"
            "   - Do NOT generate any hypothesis that was explicitly CONTRADICTED in a prior step unless it is substantively revised (narrowed, conditioned, or inverted)."
            "\n"
            "  AUDIT Instructions:\n"
            "    - For each hypothesis, perform the following steps strictly and in order.\n"
            "    - Do not reinterpret, soften, or add qualifiers to hypotheses. Audit only what is explicitly claimed:\n"
            "      1. DECLARE SCOPE  \n"
            "      - Classify the hypothesis as exactly one of:\n"
            "        - observational: direct statement explicitly supported by a tool output\n"
            "        - observed-set: comparison or claim limited to items actually observed\n"
            "        - global: claim extending beyond observed data\n"
            "        - If the hypothesis does not explicitly state its scope, assume it is global.\n"
            "      2. AUDIT EVIDENCE VS SCOPE  \n"
            "        - Assign exactly one verdict:\n"
            "        - SUPPORTED: evidence fully supports the claim within its declared scope\n"
            "        - UNSUPPORTED: evidence is insufficient or the scope exceeds the evidence\n"
            "        - CONTRADICTED: evidence directly conflicts with the claim\n"
            "      3. MANDATORY DOWNGRADES (apply without exception)\n"
            "        - Hypotheses containing words such as 'nearest', 'closest', 'only', 'all', or 'none'\n"
            "          require at least observed-set scope.\n"
            "        - If tool output indicates sampling, partial, coarse, or non-exhaustive data,\n"
            "          global hypotheses cannot be SUPPORTED.\n"
            "        - Numeric or distance calculations do not imply optimality unless all candidates\n"
            "          within the declared scope are explicitly compared.\n"
            "      4. CONTRADICTION PERSISTENCE RULE (MANDATORY)\n"
            "        - Any hypothesis that receives verdict: CONTRADICTED at this step MUST NOT reappear verbatim in HYPOTHESES in any subsequent step.\n"
            "        - If a contradicted hypothesis is still believed to be partially relevant, it may reappear ONLY if it is explicitly revised in one of the following ways:\n"
            "          a) Narrowed in scope (e.g., from global → observed-set or observational)\n"
            "          b) Conditioned (e.g., adding explicit qualifiers or boundary conditions)\n"
            "          c) Inverted into a negated or alternative hypothesis\n"
            "        - Repeating a contradicted hypothesis with identical wording or trivial rephrasing (synonyms only) is a protocol violation.\n"
            "        - If a contradicted hypothesis is not revised, it MUST be omitted entirely from future HYPOTHESES lists.\n"
            "\n"
            "  DONE Instructions:\n"
            "  - Only mark DONE: YES after ALL required actions are executed\n"
            "  - COMPLENESS CHECK: If the question asks for an attribute that can change over time (jobs, spouses, locations), you must verify if multiple values exist\n"
            "  - DO NOT STOP at the first search result. If you find one answer (e.g., 'Ambassador to Czechoslovakia'), you must briefly verify no other equal-tier answers exist (e.g., 'Ambassador to Ghana') before finishing\n"
            "  - If goal requires communicating to user, use 'say' primitive BEFORE marking done\n"
            "  - When DONE=YES, NEXT_TASK must be blank (leave empty)\n\n"
            "  NEXT_TASK Instructions:\n"
            "  - If AUDIT yields any UNSUPPORTED or CONTRADICTED verdicts, the NEXT_TASK might be a remediation step (e.g., 'Broaden search', 'Switch tool', 'Verify source').\n"
            "  - If AUDIT is fully SUPPORTED, propose the next logical step toward the #GOAL.\n"
            "  - Must be concise, actionable, and distinct from the current task.\n"
            "  - If DONE=YES, this field must be empty.\n\n"
            "  REQUEST_TOOLS Instructions:\n"
            "  - Always output valid JSON array: [] or [\"tool1\", \"tool2\"]\n"
            "  - If no tools needed, output: []\n"
            "  - If tools needed, output complete array on single line\n"
            "  - Example: [\"project\", \"filter-structured\"]\n\n"
            "  - If you realize you need a tool not initially selected, add it to REQUEST_TOOLS.\n"
            "  - You'll receive its full documentation before the next step.\n\n"
            "Follow these formats exactly."
        )
        s += assistant("Understood.\n")

        # Main loop
        current_task = s["first_task"].strip()
        for step in range(max_steps):
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                s["final_answer"] = "Interrupted by user."
                break

            # --- STAGE 2-PRE: Agent-State Hypotheses ---
            s += user(
                f"STAGE 2-PRE (step {step + 1}/{max_steps}):\n"
                f"#GOAL: {goal_for_step}\n#END GOAL\n"
                f"CURRENT_TASK: {current_task}\n\n"
                "Before choosing any tool, infer AGENT-STATE HYPOTHESES and AGENT-STATE-SUPPORT. Refer to STAGE 2-PRE Instructions.\n"
            )

            s += assistant(
                "AGENT_STATE_HYPOTHESES:\n"
                + gen(
                    f"agent_state_hypotheses_{step}",
                    max_tokens=256,
                    temperature=GEN_TEMPERATURE,
                    stop="\nAGENT_STATE_SUPPORT:"
                )
                + "\nAGENT_STATE_SUPPORT:\n"
                + gen(
                    f"agent_state_support_{step}",
                    max_tokens=256,
                    temperature=GEN_TEMPERATURE,
                    stop="\n"
                )
            )
            # Stage 2: Choose tool + args
            s += user(
                f"STAGE 2 (step {step + 1}/{max_steps}):\n"
                f"#GOAL: {goal_for_step}\n#END GOAL\n"
                f"CURRENT_TASK: {current_task}\n"
                "Choose tool and JSON args using Stage 2 FORMAT.\n"
            )
            
            s += assistant(
                "TOOL_NAME: "
                + gen(f"tool_name_{step}", max_tokens=32, temperature=GEN_TEMPERATURE, stop="TOOL_ARGS_JSON")
                + "\nTOOL_ARGS_JSON: "
                + gen(f"tool_args_{step}", max_tokens=1024, temperature=GEN_TEMPERATURE, stop="\n")
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
                resource_id_before = executor.plan_bindings_flat.get(out_var.lstrip('$'))
            
            # Execute action normally - think/say/ask now return their text content
            
            # Execute (method tools run in an inner loop but count as one outer step)
            tool_info = executor.available_tools.get(tool_name, {})
            if tool_info.get('type') == 'method':
                logger.info(f"Step {step}: Running method tool {tool_name} (inner loop, max_steps={max_steps})")
                tool_result = run_method_protocol(s, executor, tool_name, max_steps, step, _loaded_skill_docs, outer_action=action)
            else:
                tool_result = execute_infospace_action(action, executor, executor.agent_name)
            
            logger.info(f"Stage 2: Choose tool + args\n{tool_name} -> {tool_args_json}")
            logger.info(f"Step {step}: {tool_name} -> {tool_result[:100]}")
            
            # Stage 3: Reflect
            result_display = tool_result[:512]
            if len(tool_result) > 512:
                result_display += f"\n... [TRUNCATED - showing 512 of {len(tool_result)} chars]"
            
            s += user(
                f"=====\n"
                f"STAGE 3 - TOOL EXECUTION COMPLETE (step {step + 1}/{max_steps})\n"
                f"=====\n\n"
                f"Tool executed: `{tool_name}`\n"
                f"Arguments: {tool_args_json}\n\n"
                f">> ACTUAL RESULT (ground truth) <<\n"
                f"{result_display}\n"
                f">> END RESULT <<\n\n"
                f"INSTRUCTIONS:\n"
                f"1. The result above is GROUND TRUTH. Use it exactly as shown.\n"
                f"2. If reporting to user, use ONLY the values from the result above.\n"
                f"3. Do NOT approximate, summarize, or invent values.\n"
                f"4. Evaluate: Is the GOAL complete, including actual execution of all required actions, and achievement of all required information determinations, and outcomes? If yes, respond DONE: YES.\n"
                f"   If no, determine the next action needed.\n\n"
                f"Respond using Stage 3 FORMAT. Be concise.\n"
                f"Ensure the REQUEST_TOOLS list is a valid JSON list of tool names.\n"
            )
            
            s += assistant(
                "\nTHOUGHTS: "
                + gen(f"thoughts_{step}", max_tokens=128, temperature=GEN_TEMPERATURE, stop=["\nHYPOTHESES:", "HYPOTHESES:"])
                +"\nHYPOTHESES: "
                + gen(
                    f"hypotheses_{step}",
                    max_tokens=128,
                    temperature=GEN_TEMPERATURE,
                    stop=[
                        "\nAUDIT:", "AUDIT:","\nDONE:", "DONE:","\nNEXT_TASK:", "NEXT_TASK:","\nREQUEST_TOOLS:", "REQUEST_TOOLS:", "\nTHOUGHTS:", "THOUGHTS:","\nHYPOTHESES:", "HYPOTHESES:"
                    ]
                )
                +"\nAUDIT: "
                + gen(f"assumption_audit_{step}",max_tokens=128,temperature=0.0,stop="\nDONE: ")
                + "\nDONE: "
                + gen(f"done_{step}", max_tokens=8, temperature=GEN_TEMPERATURE, stop="\nNEXT_TASK: ")
                + "\nNEXT_TASK: "
                + gen(
                    f"next_task_{step}",
                    max_tokens=128,
                    temperature=GEN_TEMPERATURE,
                    stop=["\nREQUEST_TOOLS: ", "\nTHOUGHTS:", "\nHYPOTHESES:", "\nAUDIT:", "\nDONE:", "\nNEXT_TASK:"]
                )
                + "\nREQUEST_TOOLS: "
                + gen(
                    f"request_tools_{step}",
                    max_tokens=96,
                    temperature=GEN_TEMPERATURE,
                    stop=["\n\n", "\nTHOUGHTS:", "\nHYPOTHESES:", "\nAUDIT:", "\nDONE:", "\nNEXT_TASK:", "\nREQUEST_TOOLS:"]
                )
                + "\n"
            )
            # Safely log step fields (may not exist if step was interrupted or incomplete)
            # ProgramState uses bracket notation and raises KeyError if key doesn't exist
            def safe_get(state, key, default='N/A'):
                try:
                    return state[key]
                except (KeyError, TypeError, AttributeError):
                    return default
            
            logger.info(f"THOUGHTS: {safe_get(s, f'thoughts_{step}')}")
            logger.info(f"HYPOTHESES: {safe_get(s, f'hypotheses_{step}')}")
            logger.info(f"ASSUMPTIONS: {safe_get(s, f'assumption_audit_{step}')}")
            logger.info(f"DONE: {safe_get(s, f'done_{step}')}")
            logger.info(f"NEXT_TASK: {safe_get(s, f'next_task_{step}')}")
            logger.info(f"REQUEST_TOOLS: {safe_get(s, f'request_tools_{step}')}")
            
            # Stage 3.1: Update resource indexes with commentary
            # Track ANY resource created in this step (not just explicit create-note/create-collection)
            thoughts_text = s[f'thoughts_{step}'].strip()
            if thoughts_text:
                # Check if a new resource was bound in this step
                out_var = action.get('out', '')
                resource_id = None
                if out_var:
                    resource_id_after = executor.plan_bindings_flat.get(out_var.lstrip('$'))
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
                # Filter to only tools that haven't had docs loaded yet
                tools_to_load = [tool for tool in requested_tools if tool not in _loaded_skill_docs]
                
                if tools_to_load:
                    expanded_docs = load_skill_docs(tools_to_load, executor.available_tools)
                    if expanded_docs:
                        s += user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                        s += assistant("I have reviewed the additional tool documentation.\n")
                        logger.info(f"Stage 3.5: Loaded docs for {len(tools_to_load)} additional tools: {tools_to_load}")
                        # Mark these tools as loaded
                        _loaded_skill_docs.update(tools_to_load)
                else:
                    logger.debug(f"Stage 3.5: All {len(requested_tools)} requested tools already have docs loaded, skipping")
            elif requested_tools_raw and requested_tools_raw.lower() not in ["", "[]", "none", "null"]:
                # Log warning only if there was actual content that failed to parse
                logger.warning(f"Step {step}: Failed to parse REQUEST_TOOLS: {requested_tools_raw[:100]}")
            
            # Check if done
            done_raw = s[f"done_{step}"].strip().upper()
            if done_raw.startswith("YES"):
                # --- [START NEW VERIFICATION LOGIC] ---
                # Intercept the completion to force a self-audit
                s += user(
                    "STOP. Before providing the final answer, perform a verification step. \n"
                    "Verify that the GOAL has been achieved, including actual execution of all required actions, and achievement of all required information determinations, and outcomes:\n"
                    "#VERIFICATION_INSTRUCTIONS:\n"
                    "        Inputs: GOAL, prior state, and current state\n"
                    "        - what are the observable facts in STATE_AFTER that directly satisfy GOAL.\n"
                    "        - what are the required GOAL actions and facts not observable in STATE_AFTER.\n"
                    "        - based on the above, provide a candid VERIFICATION ANSWER: (SUCCESS, PARTIAL, INCONCLUSIVE):\n"
                    "#OUTPUT FORMAT:\n"
                    "        - VERIFICATION_ANSWER: <SUCCESS | PARTIAL | INCONCLUSIVE>\n"
                    "#Respond using the OUTPUT FORMAT. Be concise.\n"
                )
                
                s += assistant(
                    "VERIFICATION_ANSWER: "
                    + gen("VERIFICATION_ANSWER", max_tokens=96, temperature=GEN_TEMPERATURE, stop="\n")

                )
                
                logger.info(f"VERIFICATION ANSWER: {s['VERIFICATION_ANSWER']}")
                # --- [END NEW VERIFICATION LOGIC] ---                
                # 
                # Generate final answer using NEXT_TASK as prompt, or generic goal-focused prompt
                next_task_raw = s[f"next_task_{step}"].strip()
                if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                    final_prompt = next_task_raw
                else:
                    final_prompt = "Summarize the results with focus on the original goal"
                
                s += user(f"FINAL TASK: {final_prompt}\nProvide a concise final answer.")
                s += assistant(gen("final_answer", max_tokens=256, temperature=GEN_TEMPERATURE, stop=["\n\n", "STAGE"]))
                logger.info(f"FINAL_ANSWER: {s['final_answer']}")
                break
            
            # Update current task for next iteration
            next_task_raw = s[f"next_task_{step}"].strip()
            if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                current_task = next_task_raw
                logger.info(f"Step {step}: Next task: {current_task}")
            else:
                logger.warning(f"Step {step}: No NEXT_TASK provided, stopping")
        
        
        # Write full conversation state to trace file (file-only, not console)
        if trace_file:
            logger.info(f"Writing full conversation state to trace file")
            trace_file.write(f"\n{'='*80}\n")
            trace_file.write(f"Planning session: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            trace_file.write(f"Goal: {goal}\n")
            trace_file.write(f"total length: {len(str(s))}\n")
            trace_file.write(f"token count: {tokenize_len(executor.tokenizer, str(s))}\n")
            trace_file.write(f"{'='*80}\n")
            trace_file.write(str(s)+"\n")
            trace_file.write(f"\n{'='*80}\n\n")
            trace_file.flush()
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


def _interrupt_requested(executor: "InfospaceExecutor") -> bool:
    """Return True if an interrupt has been requested via executor or executive_node."""
    if getattr(executor, "interrupt_requested", False):
        return True
    exec_node = getattr(executor, "executive_node", None)
    if exec_node and getattr(exec_node, "interrupt_requested", False):
        return True
    return False


def _clear_interrupt(executor: "InfospaceExecutor") -> None:
    """Clear interrupt flags so a single interrupt request is consumed once."""
    if hasattr(executor, "interrupt_requested"):
        executor.interrupt_requested = False
    exec_node = getattr(executor, "executive_node", None)
    if exec_node and hasattr(exec_node, "interrupt_requested"):
        exec_node.interrupt_requested = False


# Maximum call depth for recursive method calls
MAX_METHOD_DEPTH = 3

def run_method_protocol(s, executor: "InfospaceExecutor", method_name: str, max_steps: int, outer_step: int,
                        loaded_skill_docs: set, call_depth: int = 0, outer_action: Dict = None) -> str:
    """
    Execute a 'method' tool as an inner loop (bounded by max_steps).
    Returns a short summary string for the outer loop (so the method counts as 1 outer step).
    
    Pushes a new binding scope for the method (with copy of outer scope for read access),
    and pops it when the method completes.
    
    Args:
        max_steps: Maximum steps for this method (reduced by 4 at each recursive level)
        call_depth: Current recursion depth (0 = top level, incremented for nested calls)
    """
    # Check depth limit
    if call_depth >= MAX_METHOD_DEPTH:
        error_msg = f"Method {method_name} exceeded maximum call depth ({MAX_METHOD_DEPTH}). Nested method calls are limited to prevent unbounded recursion."
        bound_var = outer_action.get('out', '') if outer_action else ''
        if bound_var:
            return f"ERROR | {error_msg} | Bound: {bound_var}"
        return f"ERROR | {error_msg}"
    
    # Ensure max_steps is at least 1 (minimum viable)
    effective_max_steps = max(1, max_steps)
    
    # Publish method invocation to action log
    if hasattr(executor, 'executive_node') and executor.executive_node:
        from datetime import datetime
        executive_node = executor.executive_node
        now_ts = datetime.now()
        
        # Create action dict for method invocation
        method_action = {
            'type': method_name,
            'action_type': method_name,
            'action_id': f'{method_name}_{int(now_ts.timestamp() * 1000)}',
            'timestamp': now_ts.isoformat(),
            'character': executor.agent_name,
            'status': 'running',
            'method_depth': call_depth,
            'max_steps': effective_max_steps
        }
        
        # Publish method invocation start
        if hasattr(executive_node, '_publish_action_result'):
            executive_node._publish_action_result(method_action, {'status': 'running'}, method_name, now_ts)
    
    # Push new binding scope for method (copy outer scope for read access)
    executor.push_binding_scope(copy_outer=True)
    
    method_task = "STEP 1"
    last_tool_result = ""
    try:
        for mstep in range(effective_max_steps):
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                error_msg = f"Method {method_name} interrupted by user"
                bound_var = outer_action.get('out', '') if outer_action else ''
                if bound_var:
                    return f"ERROR | {error_msg} | Bound: {bound_var}"
                return f"ERROR | {error_msg}"

            s += user(
            f"#METHOD EXECUTION MODE: {method_name} (internal step {mstep + 1}/{effective_max_steps}, depth {call_depth})\n"
                f"CURRENT METHOD STEP: {method_task}\n"
                "Select the tool explicitly required by the current Method Step.\n"
                "Choose tool and JSON args using Stage 2 FORMAT.\n"
            )

            tool_name_key = f"m_tool_name_{outer_step}_{mstep}"
            tool_args_key = f"m_tool_args_{outer_step}_{mstep}"
            s += assistant(
            "TOOL_NAME: "
            + gen(tool_name_key, max_tokens=32, temperature=GEN_TEMPERATURE, stop="TOOL_ARGS_JSON")
            + "\nTOOL_ARGS_JSON: "
            + gen(tool_args_key, max_tokens=1024, temperature=GEN_TEMPERATURE, stop="\n")
            + "\n"
        )

            tool_name = s[tool_name_key].strip()
            tool_args_json = s[tool_args_key].strip()

            tool_info = executor.available_tools.get(tool_name, {})
            
            # Initialize action and resource_id_before for potential use later
            action = None
            resource_id_before = None
            
            # Handle method tool recursion with depth checking
            if tool_info.get('type') == 'method':
                # Recursive method call - reduce max_steps by 4 and increment depth
                # Create a nested action dict for the recursive call
                nested_action = {
                    'type': tool_name,
                    'out': ''  # Nested methods don't bind variables directly
                }
                nested_max_steps = max(1, effective_max_steps - 4)
                nested_result = run_method_protocol(
                    s, executor, tool_name, nested_max_steps, outer_step, 
                    loaded_skill_docs, call_depth=call_depth + 1, outer_action=nested_action
                )
                last_tool_result = nested_result
                result_display = nested_result[:512]
                if len(nested_result) > 512:
                    result_display += f"\n... [TRUNCATED - showing 512 of {len(nested_result)} chars]"
            else:
                # Standard tool execution
                action = sgl_to_infospace_action(tool_name, tool_args_json, outer_step * 1000 + mstep, executor.available_tools)
            
                # Add inner loop metadata for UI display
                action['_inner_loop'] = {"method_name": method_name, "inner_step": mstep + 1, "max_steps": effective_max_steps, "outer_step": outer_step}

                # Track resource bindings before execution (for commentary update)
                out_var = action.get('out', '')
                if out_var:
                    resource_id_before = executor.plan_bindings_flat.get(out_var.lstrip('$'))

                last_tool_result = execute_infospace_action(action, executor, executor.agent_name)
                result_display = last_tool_result[:512]
                if len(last_tool_result) > 512:
                    result_display += f"\n... [TRUNCATED - showing 512 of {len(last_tool_result)} chars]"

            s += user(
            f"=====\n"
            f"METHOD STAGE 3 - TOOL EXECUTION COMPLETE (internal step {mstep + 1}/{effective_max_steps})\n"
            f"=====\n\n"
            f"Tool executed: `{tool_name}`\n"
            f"Arguments: {tool_args_json}\n\n"
            f">> ACTUAL RESULT (ground truth) <<\n"
            f"{result_display}\n"
            f">> END RESULT <<\n\n"
            f"METHOD NEXT_TASK INSTRUCTIONS:\n"
            f"Identify the Exact Step defined in the {method_name} manual that matches the ACTUAL RESULT above.\n"
            f"NEXT_TASK must be written as: [METHOD: STEP X] <Instruction from manual>.\n"
            f"Do not invent new steps.\n"
            f"TERMINATION: When the Method instructs TERMINATE (SUCCESS/FAILED/INAPPLICABLE), write 'METHOD COMPLETE' in THOUGHTS.\n\n"
            f"Respond using Stage 3 FORMAT. Be concise.\n"
                f"Ensure the REQUEST_TOOLS list is a valid JSON list of tool names.\n"
            )

            thoughts_key = f"m_thoughts_{outer_step}_{mstep}"
            hypotheses_key = f"m_hyp_{outer_step}_{mstep}"
            audit_key = f"m_audit_{outer_step}_{mstep}"
            done_key = f"m_done_{outer_step}_{mstep}"
            next_task_key = f"m_next_{outer_step}_{mstep}"
            req_tools_key = f"m_req_{outer_step}_{mstep}"

            s += assistant(
                "\nTHOUGHTS: "
                + gen(thoughts_key, max_tokens=128, temperature=GEN_TEMPERATURE, stop="HYPOTHESES: ")
                + "\nHYPOTHESES: "
                + gen(hypotheses_key, max_tokens=128, temperature=GEN_TEMPERATURE, stop="\nAUDIT: ")
                + "\nAUDIT: "
                + gen(audit_key, max_tokens=128, temperature=0.0, stop="\nDONE: ")
                + "\nDONE: "
                + gen(done_key, max_tokens=8, temperature=GEN_TEMPERATURE, stop="\nNEXT_TASK: ")
                + "\nNEXT_TASK: "
                + gen(next_task_key, max_tokens=128, temperature=GEN_TEMPERATURE, stop="\nREQUEST_TOOLS: ")
                + "\nREQUEST_TOOLS: "
                + gen(req_tools_key, max_tokens=96, temperature=GEN_TEMPERATURE, stop=["\n\n"])
                + "\n"
            )

            thoughts_text = s[thoughts_key].strip()

            # Stage 3.1: Update resource indexes with commentary (same behavior as outer loop)
            # Only update if we have an action (not a recursive method call)
            if thoughts_text and action is not None:
                out_var = action.get('out', '')
                resource_id = None
                if out_var:
                    resource_id_after = executor.plan_bindings_flat.get(out_var.lstrip('$'))
                    if resource_id_after and resource_id_after != resource_id_before:
                        if resource_id_after.startswith('Note_') or resource_id_after.startswith('Collection_'):
                            resource_id = resource_id_after
                if resource_id:
                    if executor.resource_manager:
                        executor.resource_manager.update_resource_commentary(resource_id, thoughts_text)

            # Stage 3.5: Dynamic tool loading (if requested)
            requested_tools_raw = s[req_tools_key].strip()
            requested_tools = parse_request_tools(requested_tools_raw)
            if requested_tools:
                tools_to_load = [tool for tool in requested_tools if tool not in loaded_skill_docs]
                if tools_to_load:
                    expanded_docs = load_skill_docs(tools_to_load, executor.available_tools)
                    if expanded_docs:
                        s += user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                        s += assistant("I have reviewed the additional tool documentation.\n")
                        loaded_skill_docs.update(tools_to_load)

            if 'METHOD COMPLETE' in thoughts_text.upper():
                summary = thoughts_text.replace("\n", " ")
                if len(summary) > 240:
                    summary = summary[:240] + "..."
                
                # Format result to match execute_infospace_action format
                bound_var = outer_action.get('out', '') if outer_action else ''
                if bound_var:
                    final_result = f"SUCCESS | Method {method_name} complete | {summary} | Bound: {bound_var}"
                else:
                    final_result = f"SUCCESS | Method {method_name} complete | {summary}"
                
                # Publish method completion to action log
                if hasattr(executor, 'executive_node') and executor.executive_node:
                    from datetime import datetime
                    executive_node = executor.executive_node
                    now_ts = datetime.now()
                    
                    method_action = {
                        'type': method_name,
                        'action_type': method_name,
                        'action_id': f'{method_name}_{int(now_ts.timestamp() * 1000)}',
                        'timestamp': now_ts.isoformat(),
                        'character': executor.agent_name,
                        'status': 'success',
                        'text': summary,
                        'method_depth': call_depth
                    }
                    
                    if hasattr(executive_node, '_publish_action_result'):
                        executive_node._publish_action_result(method_action, {'status': 'success', 'value': summary}, method_name, now_ts)
                
                return final_result

            next_task_raw = s[next_task_key].strip()
            if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                method_task = next_task_raw

        error_reason = f"Method {method_name} exceeded max_steps ({effective_max_steps}) | last_result={last_tool_result[:120]}"
        bound_var = outer_action.get('out', '') if outer_action else ''
        if bound_var:
            final_result = f"ERROR | {error_reason} | Bound: {bound_var}"
        else:
            final_result = f"ERROR | {error_reason}"
        
        # Publish method failure to action log
        if hasattr(executor, 'executive_node') and executor.executive_node:
            from datetime import datetime
            executive_node = executor.executive_node
            now_ts = datetime.now()
            
            method_action = {
                'type': method_name,
                'action_type': method_name,
                'action_id': f'{method_name}_{int(now_ts.timestamp() * 1000)}',
                'timestamp': now_ts.isoformat(),
                'character': executor.agent_name,
                'status': 'failed',
                'error': f'exceeded max_steps ({effective_max_steps})',
                'method_depth': call_depth
            }
            
            if hasattr(executive_node, '_publish_action_result'):
                executive_node._publish_action_result(method_action, {'status': 'failed', 'reason': error_reason}, method_name, now_ts)
        
        return final_result
    finally:
        # Always pop the method scope when done
        executor.pop_binding_scope()


class IncrementalPlanner:
    """
    Incremental planner using SGLang for iterative goal achievement.
    """
    
    def __init__(self, executor: InfospaceExecutor, available_tools: Dict[str, Dict], 
                logger_instance=None, sgl_model_path: str = None):
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
        self.logger = logger_instance or logger
        
        # SGLang runtime is now initialized in executive_node
        # Just verify it's available
        if not executor.runtime:
            self.logger.warning("SGLang runtime not available in executor - incremental planner may have reduced functionality")
        
        # Build tool catalog
        self.tools = build_tool_catalog(available_tools)
        self.tools_catalog_text = tool_catalog_text(self.tools)
        self.logger.info(f"IncrementalPlanner initialized with {len(self.tools)} tools")
        
        # Open trace file for SGLang conversation state logging (file-only, not console)
        character_name = getattr(executor, 'agent_name', 'unknown')
        trace_dir = os.path.join(os.path.dirname(__file__), 'logs')
        os.makedirs(trace_dir, exist_ok=True)
        trace_path = os.path.join(trace_dir, f'planner_trace_{character_name}.txt')
        self.trace_file = open(trace_path, 'a', encoding='utf-8')
        self.logger.info(f"Trace file opened: {trace_path}")
        
        # Initialize plan guidance system
        self.plan_guidance = PlanGuidance(resource_manager=executor.resource_manager)
    
    def _find_last_step(self, state, max_steps: int) -> int:
        """
        Find the last step that has a done_<step> value.
        Iterates forward, returns the last valid step found.
        """
        last_step = -1
        for step in range(max_steps):
            try:
                if f'done_{step}' in state and state[f'done_{step}']:
                    last_step = step
            except (KeyError, AttributeError, TypeError):
                break
        return last_step if last_step >= 0 else max_steps - 1
    
    def __del__(self):
        """Cleanup: close trace file on instance destruction."""
        if hasattr(self, 'trace_file') and self.trace_file:
            try:
                self.trace_file.close()
            except Exception:
                pass
    
    def _find_last_step(self, state, max_steps: int) -> int:
        """
        Find the last step that has a done_<step> value.
        Iterates forward, returns the last valid step found.
        """
        last_step = -1
        for step in range(max_steps):
            try:
                if f'done_{step}' in state and state[f'done_{step}']:
                    last_step = step
            except (KeyError, AttributeError, TypeError):
                break
        return last_step if last_step >= 0 else max_steps - 1

    def generate_subplan(self, template, goal: str, context: Dict = None, max_steps: int = 16) -> Dict:
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
        
        # Get world_model from executor
        if not hasattr(self.executor, 'world_model') or not self.executor.world_model:
            logger.warning("WorldModel not available in executor, using empty model")
            initial_world_model = empty_world_model()
        else:
            initial_world_model = self.executor.world_model.get()
            logger.info("📂 Using world_model from executor")

        try:
            # Note: plan_bindings are NOT cleared here - they persist across plans unless explicitly cleared
            # Only clear other plan state if needed, but preserve bindings
            
            # Attach plan_actions list to executor for tracking
            self.executor._plan_actions = []
            self.executor._plan_error_count = 0  # Initialize error counter for this plan
            self.goal = goal

            preplan = goal
            # Extract context components
            character_context = ""
            recent_context = ""
            if context:
                character_context = context.get('character_context', '')
                recent_context = context.get('recent_context', '')
                
            #
           
            # Run SGLang planner
            world_model = initial_world_model
            state = tool_planner_infospace.run(
                template=template,
                goal=goal,
                world_model=world_model,
                character_context=character_context,
                recent_context=recent_context,
                tools_catalog_text=self.tools_catalog_text,
                executor=self.executor,
                trace_file=self.trace_file,
                max_steps=max_steps,
                preplan=preplan,
                similar_plan=None
            )

            step = self._find_last_step(state, max_steps)
            # Safely check if done_<step> exists (may not exist if interrupted)
            done_value = None
            if f'done_{step}' in state:
                try:
                    done_value = state[f'done_{step}']
                    logger.info(f"Last step: {step}, done_{step}: {done_value}")
                except (KeyError, TypeError, AttributeError):
                    done_value = None
            else:
                logger.info(f"Last step: {step}, done_{step}: (not found - may be interrupted)")
            trace_str = str(state)
            tool_model: ToolModel = self.executor.tool_model
             
            # Extract final_answer from ProgramState (use bracket notation)
            try:
                final_answer = state['final_answer']
            except (KeyError, TypeError):
                final_answer = 'Planning completed'
            
            error_count = getattr(self.executor, '_plan_error_count', 0)
            # Safely determine success status (handle interrupt case)
            success = False
            if f'done_{step}' in state:
                try:
                    done_str = state[f'done_{step}']
                    if done_str and isinstance(done_str, str):
                        success = done_str.strip().upper().startswith("YES")
                except (KeyError, TypeError, AttributeError):
                    pass
            # If interrupted, final_answer will be "Interrupted by user."
            elif final_answer == "Interrupted by user.":
                success = False
            
            return {
                'plan': None,
                'response': final_answer,
                'task_state': None,
                'success': success,
                'error_count': error_count,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Incremental planning failed: {e}")
            traceback.print_exc()
            return {'error': str(e)}


    def generate_plan(self, template, goal: str, context: Dict = None, max_steps: int = 16) -> Dict:
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
        
        # Get world_model from executor
        if not hasattr(self.executor, 'world_model') or not self.executor.world_model:
            logger.warning("WorldModel not available in executor, using empty model")
            initial_world_model = empty_world_model()
        else:
            initial_world_model = self.executor.world_model.get()
            logger.info("📂 Using world_model from executor")

        try:
            # Note: plan_bindings are NOT cleared here - they persist across plans unless explicitly cleared
            # Only clear other plan state if needed, but preserve bindings
            
            # Attach plan_actions list to executor for tracking
            self.executor._plan_actions = []
            self.executor._plan_error_count = 0  # Initialize error counter for this plan
            self.goal = goal

            preplan = self._preplan(goal)
            # Extract context components
            character_context = ""
            recent_context = ""
            if context:
                character_context = context.get('character_context', '')
                recent_context = context.get('recent_context', '')
                
            # Find similar plans using plan guidance
            similar_plans = self.plan_guidance.find_similar_plans(goal)
            if similar_plans:
                logger.info(f"Found {len(similar_plans)} similar plans")
                for plan in similar_plans:
                    logger.info(f"Similar plan: {plan['outcome']} errors: {plan['error_count']} plan length: {len(plan['plan'])}")
                    #logger.info(f"Similar plan goal: {plan['goal']}")
            
            # Check total input size before calling tool_planner_infospace.run()
            total_input_size = (
                len(goal) +
                len(character_context) +
                len(recent_context) +
                len(self.tools_catalog_text)
            )
            if total_input_size > 100000:
                logger.error(f"⚠️  Attempting to send {total_input_size:,} chars to tool_planner_infospace.run() (limit: 100,000)")
                logger.error(f"  goal length: {len(goal):,}")
                logger.error(f"  character_context length: {len(character_context):,}")
                logger.error(f"  recent_context length: {len(recent_context):,}")
                logger.error(f"  tools_catalog_text length: {len(self.tools_catalog_text):,}")
                logger.error("Stack traceback:")
                for line in traceback.format_stack():
                    logger.error(line.rstrip())
            
            # Run SGLang planner
            world_model = initial_world_model
            state = tool_planner_infospace.run(
                template=template,
                goal=goal,
                world_model=world_model,
                character_context=character_context,
                recent_context=recent_context,
                tools_catalog_text=self.tools_catalog_text,
                executor=self.executor,
                trace_file=self.trace_file,
                max_steps=max_steps,
                preplan=preplan,
                similar_plan=similar_plans[0] if similar_plans else None
            )

            step = self._find_last_step(state, max_steps)
            # Safely check if done_<step> exists (may not exist if interrupted)
            done_value = None
            if f'done_{step}' in state:
                try:
                    done_value = state[f'done_{step}']
                    logger.info(f"Last step: {step}, done_{step}: {done_value}")
                except (KeyError, TypeError, AttributeError):
                    done_value = None
            else:
                logger.info(f"Last step: {step}, done_{step}: (not found - may be interrupted)")
            trace_str = str(state)
            tool_model: ToolModel = self.executor.tool_model
            tool_model.update_from_trace(trace_str=trace_str)
            compressed_trace = self._compress_trace(trace_str)
            reflection_frame = self._reflect(goal, world_model, max_steps, compressed_trace)
            
            # Extract inner ReflectionFrame content (LLM returns wrapped in 'ReflectionFrame' key)
            reflection_content = reflection_frame.get('ReflectionFrame', reflection_frame)
            task_state = reflection_content.get('task_state', {})
            
            # Update world_model from reflection_frame
            if hasattr(self.executor, 'world_model') and self.executor.world_model:
                self.executor.world_model.update(reflection_content)
                self.executor.world_model.save()
                # Get updated world_model for retry if needed
                world_model = self.executor.world_model.get()
            else:
                logger.warning("WorldModel not available, skipping update")
            # Safely check done_<step> before retry logic
            should_retry = False
            if step < max_steps and f'done_{step}' in state:
                try:
                    done_str = state[f'done_{step}']
                    if done_str and isinstance(done_str, str) and done_str.strip().upper().startswith("NO"):
                        should_retry = True
                except (KeyError, TypeError, AttributeError):
                    pass
            if should_retry:
                #reflect and retry
                logger.info(f"Step {step} failed, reflecting and retrying")
                state = tool_planner_infospace.run(
                    template=template,
                    goal=goal,
                    world_model=world_model,
                    character_context=character_context,
                    recent_context=recent_context,
                    tools_catalog_text=self.tools_catalog_text,
                    executor=self.executor,
                    trace_file=self.trace_file,
                    max_steps=max_steps,
                    preplan="No preplan provided",
                    similar_plan=similar_plans[0] if similar_plans else None
                )
            
            # Extract plan actions from executor
            plan_actions = getattr(self.executor, '_plan_actions', [])
            self.executor._plan_actions = plan_actions
            
            # Extract final_answer from ProgramState (use bracket notation)
            try:
                final_answer = state['final_answer']
            except (KeyError, TypeError):
                final_answer = 'Planning completed'
            
            error_count = getattr(self.executor, '_plan_error_count', 0)
            # Safely determine success status (handle interrupt case)
            success = False
            if f'done_{step}' in state:
                try:
                    done_str = state[f'done_{step}']
                    if done_str and isinstance(done_str, str):
                        success = done_str.strip().upper().startswith("YES")
                except (KeyError, TypeError, AttributeError):
                    pass
            # If interrupted, final_answer will be "Interrupted by user."
            elif final_answer == "Interrupted by user.":
                success = False
            
            return {
                'plan': plan_actions,
                'response': final_answer,
                'task_state': task_state,
                'success': success,
                'error_count': error_count,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Incremental planning failed: {e}")
            traceback.print_exc()
            return {"success": False, 'error': str(e)}


    def _preplan(self, goal_text: str) -> str:
        tool_names = list(self.tools.keys())
        ABSTRACT_PLAN_PROMPT = f"""
You will create a short, high-level problem-solving strategy for the goal below.
This is NOT a domain explanation and NOT a tool invocation sequence.
It is a goal-specific strategy sketch that guides downstream incremental planning.

Only use tools that appear in the provided tool list.

AVAILABLE_TOOLS:
{tool_names}

GOAL:
{goal_text}

Instructions:

1. If this is a simple, direct goal, return the original goal. DO NOTHING ELSE.
2. Otherwise:
   - Identify what *types of operations* the goal requires  
   (e.g., retrieval, extraction from existing text, transformation, comparison, generation).
   - Match these operation types to the available tools.  
     - Do not assume tools that are absent.  
     - If multiple tools could serve a role, state how to choose between them.
   - Describe a minimal, ordered sequence of conceptual steps  
     - that a tool-using planner would follow to solve the goal.  
     - Describe *what must be achieved*, not how to call tools.
   - Include fallback logic **only if the goal plausibly needs it**  
     (e.g., multi-source lookup, ambiguous values, missing information).
   - Do not include JSON, tool calls, URLs, or domain-specific knowledge.  
   The output must be a brief strategic outline, not an answer to the goal.

Format:
ABSTRACT_PLAN:
- <step>
- ...
END_PLAN
"""

        abstract_plan = self.executor.llm_generate(ABSTRACT_PLAN_PROMPT, max_tokens=256, temperature=GEN_TEMPERATURE)
        return abstract_plan.text.strip()

    def _feedback(self, outcome: bool) -> Dict:
        """
        Record feedback about plan execution outcome.
        
        Args:
            outcome: True if plan was correct/successful, False otherwise
            
        Returns:
            Dict with success status
        """
        feedback_dir = os.path.join(os.path.dirname(__file__), 'data', 'planner_feedback')
        os.makedirs(feedback_dir, exist_ok=True)
        
        feedback_file = os.path.join(feedback_dir, 'jsonl')
        
        # Get goal and plan from current state
        goal = getattr(self, 'goal', '')
        plan_actions = getattr(self.executor, '_plan_actions', [])
        error_count = getattr(self.executor, '_plan_error_count', 0)
        
        # Create feedback record
        feedback_record = {
            'outcome': outcome,
            'goal': goal,
            'plan': plan_actions,
            'error_count': error_count
        }
        
        # Append to JSONL file
        with open(feedback_file, 'a', encoding='utf-8') as f:
            json.dump(feedback_record, f, ensure_ascii=False)
            f.write('\n')
        
        self.logger.info(f"Recorded feedback: outcome={outcome}, goal={goal[:50]}...")
        
        return {'success': True}

    def _compress_trace(self, trace_str: str) -> str:
        """
        Compress execution trace while preserving order and event identity.
        
        Rules:
        - Preserve all events in order
        - Compress STAGE2-PRE: keep AGENT_STATE_HYPOTHESES and AGENT_STATE_SUPPORT (important for reflection and ToolModel)
        - Compress STAGE2: keep GOAL (full first, prefix match later), CURRENT_TASK
        - Compress CALL: keep tool name, semantic args, truncate large values
        - Compress RESULT: keep status, bound var, always include result
        - Compress THOUGHT: extract intent, track hypothesis deltas
        - Compress METHOD events: capture method tool inner loop execution (steps, calls, results, thoughts, hypotheses, audits)
        - Omit RAW_OTHER entirely
        - Omit duplicate content for same key (scoped by method context for inner loops)
        
        Args:
            trace_str: Full trace string from str(state)
            
        Returns:
            Compressed trace as single text string suitable for LLM prompt
        """
        if not trace_str:
            return ""
        
        # Split into sections by user/assistant markers for easier parsing
        sections = re.split(r'(<\|im_start\|>(?:user|assistant)|<\|im_end\|>)', trace_str)
        compressed_events = []
        
        # State tracking
        first_goal = None
        current_goal_prefix = None
        seen_keys = {}  # Track duplicate content by key
        current_method = None  # Track current method tool execution
        method_step = None
        
        i = 0
        while i < len(sections):
            section = sections[i].strip()
            if not section or section in ['<|im_start|>user', '<|im_start|>assistant', '<|im_end|>']:
                i += 1
                continue
            
            # METHOD EXECUTION MODE event (method tool inner loop start)
            method_mode_match = re.search(r'#METHOD EXECUTION MODE:\s*([\w-]+)\s*\(internal step (\d+)/(\d+)\)', section)
            if method_mode_match:
                method_name = method_mode_match.group(1)
                inner_step = method_mode_match.group(2)
                max_inner_steps = method_mode_match.group(3)
                current_method = method_name
                method_step = inner_step
                
                # Extract method step instruction
                method_step_match = re.search(r'CURRENT METHOD STEP:\s*(.*?)(?:\n|$)', section)
                method_step_text = method_step_match.group(1).strip() if method_step_match else None
                
                if method_step_text:
                    compressed_events.append(f"[METHOD {method_name} step={inner_step}/{max_inner_steps}]\nSTEP: {method_step_text}\n")
            
            # STAGE 2-PRE event (Agent-State Hypotheses - outer loop only, not in method protocol)
            elif re.search(r'STAGE 2-PRE \(step (\d+)/(\d+)\):', section):
                stage2_pre_match = re.search(r'STAGE 2-PRE \(step (\d+)/(\d+)\):', section)
                if stage2_pre_match:
                    step_num = stage2_pre_match.group(1)
                    max_steps = stage2_pre_match.group(2)
                    
                    # Extract AGENT_STATE_HYPOTHESES
                    hyp_match = re.search(r'AGENT_STATE_HYPOTHESES:\s*\[(.*?)\]', section, re.DOTALL)
                    hyp_text = hyp_match.group(1).strip() if hyp_match else None
                    
                    # Extract AGENT_STATE_SUPPORT
                    support_match = re.search(r'AGENT_STATE_SUPPORT:\s*(.*?)(?:\n\n|\nSTAGE|$)', section, re.DOTALL)
                    support_text = support_match.group(1).strip() if support_match else None
                    
                    if hyp_text or support_text:
                        event_parts = [f"[STAGE2-PRE step={step_num}/{max_steps}]"]
                        if hyp_text:
                            # Try to parse as JSON list, fallback to raw text
                            try:
                                hyp_list = json.loads(f"[{hyp_text}]")
                                event_parts.append(f"AGENT_STATE_HYPOTHESES: {json.dumps(hyp_list, ensure_ascii=False)}")
                            except:
                                event_parts.append(f"AGENT_STATE_HYPOTHESES: [{hyp_text}]")
                        if support_text:
                            # Keep support text (may be multi-line with evidence pointers)
                            # Truncate if very long
                            if len(support_text) > 500:
                                support_text = support_text[:500] + "\n... [truncated]"
                            event_parts.append(f"AGENT_STATE_SUPPORT:\n{support_text}")
                        compressed_events.append('\n'.join(event_parts) + '\n')
            
            # STAGE 2 event (outer loop)
            elif re.search(r'STAGE 2 \(step (\d+)/(\d+)\):', section):
                stage2_match = re.search(r'STAGE 2 \(step (\d+)/(\d+)\):', section)
                current_method = None  # Reset method context for outer loop
                method_step = None
                if stage2_match:
                    step_num = stage2_match.group(1)
                    max_steps = stage2_match.group(2)
                    
                    # Extract GOAL and TASK
                    goal_match = re.search(r'#GOAL:\s*(.*?)#END GOAL', section, re.DOTALL)
                    task_match = re.search(r'CURRENT_TASK:\s*(.*?)(?:\n|$)', section)
                    
                    goal_text = goal_match.group(1).strip() if goal_match else None
                    task_text = task_match.group(1).strip() if task_match else None
                    
                    # Handle goal: full first occurrence, prefix match later
                    if first_goal is None and goal_text:
                        first_goal = goal_text
                        current_goal_prefix = goal_text[:100]  # Use first 100 chars as prefix
                        goal_display = goal_text
                    elif goal_text:
                        if goal_text.startswith(current_goal_prefix):
                            goal_display = "[unchanged from step 1]"
                        else:
                            # Changed - update prefix and show new goal
                            current_goal_prefix = goal_text[:100]
                            goal_display = goal_text
                    else:
                        goal_display = None
                    
                    if goal_display and task_text:
                        compressed_events.append(f"[STAGE2 step={step_num}/{max_steps}]\nGOAL: {goal_display}\nTASK: {task_text}\n")
            
            # CALL event (TOOL_NAME + TOOL_ARGS_JSON)
            elif 'TOOL_NAME:' in section:
                tool_match = re.search(r'TOOL_NAME:\s*([\w-]+)', section)
                args_match = re.search(r'TOOL_ARGS_JSON[^:]*:\s*(\{.*?\})', section, re.DOTALL)
                
                if tool_match:
                    tool_name = tool_match.group(1)
                    if args_match:
                        try:
                            args_dict = json.loads(args_match.group(1))
                            # Keep keys, truncate large values
                            compressed_args = {}
                            for k, v in args_dict.items():
                                if k == 'out':
                                    compressed_args[k] = v  # Always keep out
                                elif isinstance(v, str) and len(v) > 100:
                                    compressed_args[k] = v[:100] + "..."
                                else:
                                    compressed_args[k] = v
                            tool_args = json.dumps(compressed_args)
                        except:
                            tool_args = args_match.group(1)[:200]
                    else:
                        tool_args = "{}"
                    
                    # Prefix with method context if in method execution
                    if current_method:
                        compressed_events.append(f"[METHOD {current_method} CALL tool={tool_name}]\nARGS: {tool_args}\n")
                    else:
                        compressed_events.append(f"[CALL tool={tool_name}]\nARGS: {tool_args}\n")
            
            # METHOD RESULT event (METHOD STAGE 3 + ACTUAL RESULT)
            elif 'METHOD STAGE 3 - TOOL EXECUTION COMPLETE' in section:
                method_result_match = re.search(r'METHOD STAGE 3 - TOOL EXECUTION COMPLETE \(internal step (\d+)/(\d+)\)', section)
                tool_match = re.search(r'Tool executed: `([\w-]+)`', section)
                result_match = re.search(r'>> ACTUAL RESULT.*?<<\n(.*?)\n>> END RESULT', section, re.DOTALL)
                bound_match = re.search(r'Bound: (\$\w+)', section)
                status_match = re.search(r'(SUCCESS|FAILED)', section)
                
                if method_result_match and tool_match:
                    inner_step = method_result_match.group(1)
                    max_inner_steps = method_result_match.group(2)
                    tool_name = tool_match.group(1)
                    result_text = result_match.group(1).strip() if result_match else ""
                    bound_var = bound_match.group(1) if bound_match else None
                    status = 'OK' if status_match and 'SUCCESS' in status_match.group(0) else ('FAILED' if status_match else None)
                    
                    # Always include result, truncate if too long
                    if len(result_text) > 300:  # Shorter for method inner loop
                        result_text = result_text[:300] + f"\n... [truncated]"
                    
                    event_str = f"[METHOD {current_method or 'unknown'} RESULT step={inner_step}/{max_inner_steps} tool={tool_name}"
                    if status:
                        event_str += f" status={status}"
                    if bound_var:
                        event_str += f" -> {bound_var}"
                    event_str += "]\n"
                    if result_text:
                        event_str += f"RESULT: {result_text}\n"
                    compressed_events.append(event_str)
            
            # RESULT event (STAGE 3 + ACTUAL RESULT) - outer loop
            elif 'STAGE 3 - TOOL EXECUTION COMPLETE' in section and 'METHOD STAGE 3' not in section:
                tool_match = re.search(r'Tool executed: `([\w-]+)`', section)
                result_match = re.search(r'>> ACTUAL RESULT.*?<<\n(.*?)\n>> END RESULT', section, re.DOTALL)
                bound_match = re.search(r'Bound: (\$\w+)', section)
                status_match = re.search(r'(SUCCESS|FAILED)', section)
                
                if tool_match:
                    tool_name = tool_match.group(1)
                    result_text = result_match.group(1).strip() if result_match else ""
                    bound_var = bound_match.group(1) if bound_match else None
                    status = 'OK' if status_match and 'SUCCESS' in status_match.group(0) else ('FAILED' if status_match else None)
                    
                    # Always include result, truncate if too long
                    if len(result_text) > 500:
                        result_text = result_text[:500] + f"\n... [truncated, total {len(result_match.group(1))} chars]"
                    
                    event_str = f"[RESULT tool={tool_name}"
                    if status:
                        event_str += f" status={status}"
                    if bound_var:
                        event_str += f" -> {bound_var}"
                    event_str += "]\n"
                    if result_text:
                        event_str += f"RESULT: {result_text}\n"
                    compressed_events.append(event_str)
            
            # THOUGHT event (check for method context by looking for method-specific patterns)
            elif 'THOUGHTS' in section or 'HYPOTHESES:' in section or 'DONE:' in section:
                # Check if this is a method inner loop thought (look for METHOD NEXT_TASK pattern)
                is_method_thought = 'METHOD NEXT_TASK INSTRUCTIONS' in section or current_method is not None
                
                thoughts_match = re.search(r'THOUGHTS[^:]*:\s*(.*?)(?:\nHYPOTHESES|$)', section, re.DOTALL)
                hyp_match = re.search(r'HYPOTHESES:\s*(.*?)(?:\nAUDIT|$)', section, re.DOTALL)
                assump_match = re.search(r'AUDIT:\s*(.*?)(?:\nDONE|$)', section, re.DOTALL)
                done_match = re.search(r'DONE:\s*(.*?)(?:\nNEXT_TASK|$)', section)
                next_match = re.search(r'NEXT_TASK:\s*(.*?)(?:\nREQUEST_TOOLS|$)', section)
                request_tools_match = re.search(r'REQUEST_TOOLS:\s*(.*?)(?:\n|$)', section, re.DOTALL)
                
                thoughts = thoughts_match.group(1).strip() if thoughts_match else None
                hypotheses = hyp_match.group(1).strip() if hyp_match else None
                assumptions = assump_match.group(1).strip() if assump_match else None
                done = done_match.group(1).strip() if done_match else None
                next_task = next_match.group(1).strip() if next_match else None
                request_tools = request_tools_match.group(1).strip() if request_tools_match else None
                
                # Build thought event with method context if applicable
                if is_method_thought and current_method:
                    event_parts = [f"[METHOD {current_method} THOUGHT"]
                    if method_step:
                        event_parts[0] += f" step={method_step}"
                    event_parts[0] += "]"
                else:
                    event_parts = ["[THOUGHT]"]
                
                if thoughts:
                    event_parts.append(f"THOUGHTS: {thoughts}")
                
                if hypotheses:
                    # Include complete hypotheses (no delta tracking)
                    event_parts.append(f"HYPOTHESES: {hypotheses}")
                
                if assumptions:
                    # Include complete assumptions/audits (no delta tracking)
                    event_parts.append(f"AUDIT: {assumptions}")
                
                # Check for duplicates (use method-scoped keys if in method)
                if done:
                    done_key = f"{current_method or 'outer'}:DONE:{done}" if is_method_thought else f"DONE:{done}"
                    if done_key not in seen_keys:
                        event_parts.append(f"DONE: {done}")
                        seen_keys[done_key] = True
                
                if next_task:
                    next_key = f"{current_method or 'outer'}:NEXT:{next_task}" if is_method_thought else f"NEXT:{next_task}"
                    if next_key not in seen_keys:
                        event_parts.append(f"NEXT: {next_task}")
                        seen_keys[next_key] = True
                
                if request_tools:
                    tools_key = f"{current_method or 'outer'}:REQUEST_TOOLS:{request_tools}" if is_method_thought else f"REQUEST_TOOLS:{request_tools}"
                    if tools_key not in seen_keys:
                        event_parts.append(f"REQUEST_TOOLS: {request_tools}")
                        seen_keys[tools_key] = True
                
                # Check for METHOD COMPLETE in thoughts
                if is_method_thought and thoughts and 'METHOD COMPLETE' in thoughts.upper():
                    event_parts.append("METHOD COMPLETE")
                    current_method = None  # Reset method context
                    method_step = None
                
                if len(event_parts) > 1:  # More than just [THOUGHT] or [METHOD ... THOUGHT]
                    compressed_events.append('\n'.join(event_parts) + '\n')
            
            # VERIFY event
            elif 'VERIFICATION_QUESTION:' in section or 'VERIFICATION_ANSWER:' in section:
                verify_type = 'Q' if 'QUESTION' in section else 'A'
                content_match = re.search(r'VERIFICATION_(?:QUESTION|ANSWER):\s*(.*?)(?:\n|$)', section)
                content = content_match.group(1).strip() if content_match else ""
                compressed_events.append(f"[VERIFY {verify_type}]\n{content}\n")
            
            # FINAL event (say action)
            elif '{"type":"say"' in section or '"type":"say"' in section:
                say_match = re.search(r'\{"type":"say"[^}]*"value":"([^"]*)"[^}]*"target":"([^"]*)"', section)
                if say_match:
                    value = say_match.group(1)
                    target = say_match.group(2)
                    compressed_events.append(f"[FINAL]\nsay({target}): \"{value}\"\n")
            
            # Skip RAW_OTHER (everything else)
            i += 1
        
        return '\n'.join(compressed_events)

    def _reflect(self, goal_text, world_model, steps, trace) -> Dict:
        """
        Reflect on the plan execution outcome.
        Args:
            state: SGLang state
            goal_text: Original goal text
            world_model: World model from previous attempt
            steps: Steps used
            trace: Trace (str(s) from tool_planner_infospace.run())
        Returns:
            world_model: Revised world model
        """

        reflection_prompt = """ROLE
You are a REFLECTION ANALYST inside the Cognitive Workbench.

You do NOT act.
You do NOT plan.
You do NOT invent knowledge.

Your sole responsibility is to analyze ONE completed planner attempt and produce a
ReflectionFrame that constrains future cognition correctly.

You may reason internally, but your FINAL OUTPUT MUST BE A SINGLE VALID JSON OBJECT
conforming EXACTLY to the ReflectionFrame schema provided by the runtime.

============================================================
AUTHORITATIVE INPUTS
============================================================

1) PREVIOUS TASK STATE (JSON)
----------------------------
{task_state}

This is ephemeral, goal-scoped working memory from the prior attempt.
It exists ONLY to support continuation of the SAME goal.

2) PREVIOUS WORLD MODEL (JSON)
------------------------------
{world_model}

This is persistent, cross-goal knowledge.
You must treat it as stable and conservative.
You are NOT authorized to forget or overwrite it wholesale.

3) FULL EXECUTION TRACE (VERBATIM)
----------------------------------
{trace}

This is evidence, not memory.
All claims must be grounded in this trace.

4) ORIGINAL GOAL TEXT
---------------------
{goal_text}

5) STEP BUDGET USED
-------------------
{steps}

============================================================
OBJECTIVE
============================================================

Produce a ReflectionFrame that:

A) Updates TASK STATE for the NEXT attempt at THIS SAME goal (if any)
B) Proposes WORLD MODEL updates ONLY if genuinely general and reusable
C) Records TOOL INSIGHTS as contracts or constraints (not praise)
D) Specifies CONTEXT-FORGET instructions to prune the NEXT prompt
E) Retires false GENERAL beliefs only (not time-local facts)

Your output must be minimal, conservative, and behavior-constraining.

============================================================
CRITICAL EPISTEMIC RULES (READ CAREFULLY)
============================================================

1) EPHEMERAL VS PERSISTENT
- Task-specific, time-specific, or location-specific facts MUST NOT enter the world model.
- World model facts must remain true under DIFFERENT goals and later time.

2) PROMOTION THRESHOLD
DO NOT promote world facts that describe:
- current agent state (health, food, idle, inventory)
- current location or coordinates
- current absence of objects
- single observations from a trivial task

A trivial or already-completed goal should produce FEW or ZERO world model updates.

3) TOOL INSIGHTS
Tool insights must be stated as:
- guarantees
- limits
- preconditions
- failure modes

NOT qualitative descriptions or praise.

4) TASK COMPLETION COLLAPSE
If the goal was COMPLETED:
- task_state MUST collapse to a minimal summary
- active hypotheses SHOULD be empty
- exhausted_search SHOULD be empty
- no new task continuations should be implied

5) CONTEXT FORGETTING AUTHORITY
You MAY request forgetting of:
- coordinates
- local observations
- trace details
- temporary hypotheses
- report text
- previous task_state

You MUST NOT request forgetting of:
- the world model as a whole
- tool contracts
- invariants
- cross-goal knowledge

6) FAILURE MODE HONESTY
If the goal was completed successfully, use:
- failure_mode = "none"

Minor recoveries or clarifications do NOT count as failures.

============================================================
REFLECTION PROCESS (FOLLOW IN ORDER)
============================================================

STEP 1 — Outcome Classification
Determine whether the goal was:
- completed successfully
- incomplete
- failed

Select the appropriate failure_mode.
Cite concrete trace evidence.

STEP 2 — TASK STATE UPDATE (EPHEMERAL)
Update ONLY what should bias the NEXT attempt at THIS SAME goal.

Include:
- brief summary
- immediate blockers (if any)
- active hypotheses (max 6, omit if goal complete)
- proven safe anchors or paths (if relevant)
- exhausted search (only if it matters for continuation)

If the goal is complete, keep this section minimal.

STEP 3 — WORLD MODEL UPDATE CANDIDATES (CONSERVATIVE)
Propose world facts ONLY if they:
- generalize across goals
- are not tied to current agent state
- reflect stable properties of the world or tools

Each fact must be atomic, general, and reusable.
If uncertain, do NOT promote.

It is acceptable — and often correct — to propose NO world updates.

STEP 4 — TOOL INSIGHTS (META-COGNITION)
Record any discovered tool properties that would constrain future planning:
- guarantees
- limits
- misleading success signals
- required preconditions

Each insight must be stated as a contract or constraint.

STEP 5 — RETIREMENT VS CONTEXT FORGETTING
- RETIRE only false GENERAL beliefs (keep as warnings).
- CONTEXT_FORGET only episode- or goal-specific material to omit from the NEXT prompt.

Never request forgetting of the world model itself.

STEP 6 — OPEN QUESTIONS (OPTIONAL)
List up to 4 precise, testable questions that would reduce uncertainty.
Omit if none are needed.

============================================================
OUTPUT REQUIREMENTS (STRICT)
============================================================

- Output MUST be a SINGLE VALID JSON OBJECT
- MUST conform EXACTLY to the ReflectionFrame schema
- NO extra keys
- NO explanations outside JSON
- Prefer omission over verbosity
- When in doubt, promote NOTHING

============================================================
OUTPUT (JSON ONLY)
============================================================

REFLECTION_FRAME_SCHEMA:
{REFLECTION_FRAME_SCHEMA}
"""
        # Convert world_model to JSON string to avoid format() interpreting braces as placeholders
        task_state = "{}"
        world_model = json.dumps(world_model, indent=2) if isinstance(world_model, dict) else str(world_model)
        task_state = json.dumps(task_state, indent=2) if isinstance(task_state, dict) else str(task_state)
        reflection_prompt = reflection_prompt.replace("{task_state}", task_state)
        reflection_prompt = reflection_prompt.replace("{world_model}", world_model)
        reflection_prompt = reflection_prompt.replace("{goal_text}", goal_text)
        reflection_prompt = reflection_prompt.replace("{steps}", str(steps))
        reflection_prompt = reflection_prompt.replace("{REFLECTION_FRAME_SCHEMA}", json.dumps(REFLECTION_FRAME_SCHEMA, indent=2))
        reflection_prompt = reflection_prompt.replace("{trace}", trace)
        reflection = self.executor.llm_generate(reflection_prompt, max_tokens=4096, is_json=True, temperature=0.0)
        logger.info(f"Reflection: {json.dumps(reflection.text, indent=2)}")

        return reflection.text