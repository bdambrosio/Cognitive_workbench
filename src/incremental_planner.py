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
import textwrap
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
    "version": "2.0",

    "_size_limits": {
      "task_state.active_hypotheses": 6,
      "task_state.proven_safe_paths": 6,
      "task_state.exhausted_search.locations": 6,
      "task_state.exhausted_search.objects": 6,
      "task_state.exhausted_search.actions": 6,
      "world_model_updates": 10,
      "tool_insights": 6,
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
        "polarity": "support | contradict",
        "source": "observation | tool_guarantee",
        "evidence": "string"
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
- Relation: Typed directed link between two resources (source -> target)
  - Relation IDs use format "Relation_###"
  - Use create-relation / find-relations / related to manage graph links
- Variable: a session-local name referencing Notes/Collections/Relations. Variables *always* start with "$".

Note Content Format (text-only model):
- Note content is always a string.
- Metadata should be represented as a separate Note linked by Relation type "meta" (source content Note -> target metadata Note).
- Structured ops (project/pluck/filter-structured/sort/join) work on Notes whose content is valid JSON text.

Action Syntax:
- An action is a JSON object specifying the application of a primitive or tool. 
  - e.g. {"type": "primitive or tool name", "target": "$variable_a", "value": "literal_value", "out": "$variable_b"} 
- ALWAYS use "$variable" for references (e.g. target, value, source, or out fields)
- Correct: {"target": "$my_variable"}
- Wrong: {"target": "my_variable"}
- Note reference by ID or name: Use directly without $ (e.g., "target": "Note_123" or "target": "attention-note")
- Collection reference by ID or name: Use directly without $ (e.g., "target": "Collection_456" or "target": "research-papers")
- Relation reference by ID: Use directly without $ (e.g., "target": "Relation_12")
- Literal strings: Use directly without $ (e.g., "value": "hello")
- Literal numbers: Use directly without $ (e.g., "value": 123)
- Literal booleans: Use directly without $ (e.g., "value": true)

Parameter Naming Convention:
- `target`: Typically references a Variable (starts with "$"), Note ID/name, or Collection ID/name
- `value`: Typically contains a literal (string, number, boolean) or may reference a Variable/Note/Collection
- This convention is consistent across primitives and tools, though specific tools may have tool-specific parameter names

InfospacePrimitive Action / Type Compatibility:
This table applies only to native infospace primitives. World/skill tools are governed by their own contracts.
Operation_name: applicable to: <Note | Collection | Note, Collection>;   Purpose
 - split: Note;  Note structure → Collection
 - flatten: Collection;  Collection → single Note
 - as-json, extract, coerce: Note;  Transform Note content
 - extract: Note;  LLM-guided extraction, compression, or transformation of single-Note content
 - synthesize: Collection;  Cross-document integration, comparison, and reporting
 - map: Collection;  Apply op to each Collection item
 - project, pluck, sort, filter: Collection;  SQL-like Collection ops
 - join: Collection;  Merge 2 Collections (SQL JOIN)
 - add, remove, size: Collection;  Collection mutation operations
 - union, intersection, difference: Collection;  Set operations on Collections
 - load: Note, Collection;  Load persistent resource
 - persist: Note, Collection;  Mark resource as persistent
 - discover-notes, discover-collections: N/A;  Global discovery (return Coll.)
 - search-within-collection: Collection;  Search indexed Collection
- create-relation: Note/Collection + Note/Collection;  Create typed directed edge
- find-relations: N/A;  Query relations by source/target/type (returns Collection of Relation IDs)
- related: Note/Collection;  Get neighbors by relation type and direction

Key distinctions:
- split (Note→Coll): Transforms internal structure (array/lines) into separate items
- flatten (Coll→Note): Opposite of split, merges Collection into single Note
- load: Use to GET content *into planner context*. Supports slice parameter for controlled access (e.g., slice="0:3" for first 3 Collection items, slice=":" for full Note content). Slicing a Collection returns a new Collection.
- persist: Mark resource as persistent (saved to filesystem)

Discovery and Search Primitives:
- discover-notes: Global discovery across all Notes (no target needed). Returns Collection of Notes with text content.
- discover-collections: Global discovery across all Collections (no target needed). Returns Collection of Notes with text content.
- search-within-collection: Search within a specific indexed Collection (requires target Collection, must be indexed first). Returns Collection of Notes with matched chunk text.

Discovery tools return Note or Collection candidates. Search-within tools search known containers. If you think you need a target field, you are not doing discovery.

Note content is always a string. For extracting information FROM text content, use extract (LLM-based).

Persistence Operations:
- load: Retrieve persistent Note or Collection by ID or name. Returns prefixed content ("Note Content: <text>" or "Collection Content: <ids>"). Use slice=":" for full content or slice="0:N" for explicit amount; no length limit when slice is explicit. Default slice "0:4096" (Notes) or "0:5" (Collections). Use to get content into planner context.
- persist: Mark Note or Collection as persistent (saved to filesystem). Use after creating resources you want to keep.

Collection Mutation Operations (require Collection):
- add: Add Note or Collection to existing Collection (mutates in place)
- remove: Remove Note from Collection (mutates in place)
- size: Get item count of Collection (returns Note with integer)
- union: Combine two Collections (A ∪ B) - all items, deduplicated
- intersection: Items in both Collections (A ∩ B)
- difference: Items in A but not B (A - B)

Structured-data Collection Operations (require Notes whose content is JSON string):
- project: Extract fields from JSON content (SELECT columns) → new Collection
- pluck: Extract single field as simple values → Collection of values
- filter-structured: Filter by field conditions (WHERE clauses) → filtered Collection
- sort: Sort by field value (ORDER BY) → sorted Collection
- join: Combine two Collections on matching field (INNER JOIN) → merged Collection

To take first N items from a Collection, use load with slice (e.g., load with slice="0:5"). This replaces the old head primitive.

IMPORTANT: project/pluck/filter-structured work only on Notes whose content is valid JSON (e.g., from create-note with dict value).
To extract information FROM plain text content, use extract (LLM-based extraction).

Use cases:
- project: Extract fields from JSON Notes (e.g., create-note with structured data)
- pluck: Get single field from JSON Notes
- filter-structured: Filter JSON Notes by field conditions
- sort: Sort JSON Notes by field value
- load with slice: Get top 5 after sorting (slice="0:5"), take first result (slice="0")
- join: Merge JSON Notes from two Collections on matching field

Efficiency Heuristics:
- Use tools directly on Notes for single items
- Create Collections only for 2+ Notes together
- split, extract, as-json work on Notes ONLY, not Collections
- Use extract directly on Notes for single-item work
- Use map(extract) for per-item extraction across a Collection
- Use synthesize for cross-item integration, comparison, or reporting
- Two-phase pattern: map(extract) → synthesize (extract per item, then integrate)
- Single-phase: synthesize with focus (when per-item extraction isn't needed)
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
        "create-relation": {
            "description": "Create a typed directed relation between two resources.",
            "full_description": "Create a Relation edge source -> target with a required relation label (e.g., meta, related, supports). Source and target must resolve to Note or Collection IDs. Returns a Relation resource ID bound to out.",
            "parameters": {
                "source": "required: $variable or resource ID (source endpoint)",
                "target_id": "required: $variable or resource ID (target endpoint)",
                "relation": "required: relation type label string",
                "out": "required: $variable for created Relation"
            },
            "examples": [
                '{"type":"create-relation","source":"$paper_note","target_id":"$paper_meta_note","relation":"meta","out":"$paper_meta_rel"}',
                '{"type":"create-relation","source":"$note_a","target_id":"$note_b","relation":"related","out":"$rel"}'
            ],
            "schema_hint": {"source": "$variable or resource ID", "target_id": "$variable or resource ID", "relation": "string", "out": "$variable"}
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
        "bind": {
            "description": "Bind a variable to an existing resource without creating or changing content.",
            "full_description": "Alias an existing Note/Collection/Relation to a new variable via out. This is useful to mark a canonical artifact variable (for example $goal_artifact) at the end of a code block. bind does not mutate resources.",
            "parameters": {
                "target": "required: $variable, resource ID, or named resource",
                "out": "required: destination $variable"
            },
            "examples": [
                '{"type":"bind","target":"$final_report","out":"$goal_artifact"}',
                '{"type":"bind","target":"Note_42","out":"$goal_artifact"}'
            ],
            "schema_hint": {"target": "$variable or resource ID or name", "out": "$variable"}
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
        "create-relation": {
            "description": PRIMITIVE_DOCS["create-relation"]["description"],
            "full_description": PRIMITIVE_DOCS["create-relation"].get("full_description"),
            "examples": PRIMITIVE_DOCS["create-relation"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["create-relation"]["schema_hint"]
        },
        "load": {
            "description": "Load persistent Note or Collection by ID or name, with optional slice. Notes: slice is characters (default 0:4096). Collections: slice is items (default 0:5), returns a new Collection. Use slice=':' for full content (no limit) or slice='0:N' for explicit amount. For large Notes, use chunked pattern: load slice='0:500' then slice='500:1000' etc.",
            "full_description": "Load a Note or Collection into planner context. The 'slice' parameter controls how much content is returned, using Python-style syntax: '0:1000' (first 1000), '1500:2000' (chars 1500–2000), '-500:' (last 500), ':' (everything, no limit), '5' (single item). Standard Python semantics including negative indices. Rejects only when both start and stop are non-negative and stop<start. For Notes, units are characters. For Collections, units are items — the result is a new Collection bound to out. Chunked pattern for large content: load slice='0:500' → process → load slice='500:1000' → process → load slice='1500:2000' → process.",
            "examples": [
                '{"type":"load","target":"$report","slice":":","out":"$full_report"}',
                '{"type":"load","target":"$papers","slice":"0:3","out":"$top_papers"}',
                '{"type":"load","target":"$doc","slice":"0:500","out":"$chunk1"}',
                '{"type":"load","target":"$doc","slice":"1500:2000","out":"$chunk3"}'
            ],
            "schema_hint": {"target": "string (ID or name) or $variable", "slice": "string (optional, e.g. '0:4096', ':', '0:5')", "out": "$variable"}
        },
        "persist": {
            "description": "Mark Note/Collection as persistent. Use this to save the Note/Collection to the filesystem.",
            "schema_hint": {"target": "$variable"}
        },
        "discover-notes": {
            "description": "Global discovery across all Notes using embedding-based retrieval. Returns Collection of Notes with text content.",
            "schema_hint": {"query": "string (query)", "out": "$variable", "limit": "int (optional, default 5)", "threshold": "float (optional, default 0.3)"}
        },
        "discover-collections": {
            "description": "Global discovery across all Collections using embedding-based retrieval. Returns Collection of Notes with text content.",
            "schema_hint": {"query": "string (query)", "out": "$variable", "limit": "int (optional, default 3)", "threshold": "float (optional, default 0.3)"}
        },
        "search-within-collection": {
            "description": "Search within a specific indexed Collection. Returns Collection of Notes with matched chunk text. Requires Collection to be indexed first.",
            "schema_hint": {"target": "$variable (indexed Collection)", "query": "string (query)", "out": "$variable", "limit": "int (optional, default 5)", "threshold": "float (optional, default 0.0)"}
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
            "description": "Transform Note structure into Collection: JSON array → Collection of Notes (one per element), or plain text → Collection of Notes (default: by sentence). NOT for inspecting Collection contents. Use flatten to merge Collection into single Note, or load to read content.",
            "full_description": "Split transforms a single Note's internal structure into a Collection. Input must be a Note (not Collection) containing either: (1) JSON array - each element becomes a Note in output Collection, (2) JSON object with array field - extracts array from specified field (default 'results'), (3) JSONL format - multiple JSON objects separated by newlines, or (4) plain text - splits by delimiter (default 'sentence' for semantic processing). For plain text, default delimiter is 'sentence' which splits on sentence boundaries (. ! ? followed by space/newline), normalizes whitespace (removes internal newlines), and filters empty segments. Optional delimiter parameter: 'sentence' (default), 'paragraph' (double newlines), 'line' (single newlines), or custom string. This is a STRUCTURE TRANSFORMATION, not content inspection. To view Collection contents, use flatten (merge into single Note) or load to read content. Common mistake: trying to split a Collection to 'see inside it' - Collections are already split, use display instead.",
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
        "think": {
            "description": PRIMITIVE_DOCS["think"]["description"],
            "full_description": PRIMITIVE_DOCS["think"].get("full_description"),
            "examples": PRIMITIVE_DOCS["think"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["think"]["schema_hint"]
        },
        "bind": {
            "description": PRIMITIVE_DOCS["bind"]["description"],
            "full_description": PRIMITIVE_DOCS["bind"].get("full_description"),
            "examples": PRIMITIVE_DOCS["bind"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["bind"]["schema_hint"]
        },
        "ask": {
            "description": "Send question to target and terminate current turn. Response will arrive in a future turn. Requires: value, out. Optional: target (default User).",
            "schema_hint": {"target": "User or agent name (optional)", "value": "string", "out": "$variable"}
        },
        "find-relations": {
            "description": "Find relations by source, target, and/or relation type. Returns a Collection of Relation IDs.",
            "schema_hint": {"source": "$variable or resource ID (optional)", "target": "$variable or resource ID (optional)", "relation": "string (optional)", "out": "$variable"}
        },
        "related": {
            "description": "Get resources related to target via relations. Returns a Collection of neighboring resource IDs.",
            "schema_hint": {"target": "$variable or resource ID", "relation": "string (optional)", "direction": "string (optional: out|in|both)", "out": "$variable"}
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
            "description": "Extract fields from each Note in Collection (SQL SELECT). Works only on Notes whose content is JSON string. For extracting info FROM plain text, use extract instead.",
            "full_description": "Project extracts specified fields from each Note in a Collection, similar to SQL SELECT. Input Collection must contain Notes whose content is valid JSON (parseable). Output is a new Collection of projected Notes. Notes missing any requested field are excluded. Nested fields use dot notation (e.g., 'title', 'metadata.year'). Does NOT parse unstructured text — use extract for that.",
            "examples": [
                '{"type":"project","target":"$results","fields":["uri","score"],"out":"$filtered"}',
                '{"type":"project","target":"$papers","fields":["title","year"],"out":"$paper_info"}'
            ],
            "schema_hint": {"target": "$variable (Collection of dict Notes)", "fields": "array of field paths (strings)", "out": "$variable"}
        },
        "pluck": {
            "description": "Extract single field value from each Note in Collection. Works only on Notes whose content is JSON string.",
            "full_description": "Pluck extracts a single field from each Note in a Collection. Input Notes must have JSON content. Returns a Collection of Notes with just that field's value. Notes missing the field are excluded.",
            "examples": [
                '{"type":"pluck","target":"$papers","field":"title","out":"$titles"}',
                '{"type":"pluck","target":"$results","field":"score","out":"$scores"}'
            ],
            "schema_hint": {"target": "$variable (Collection of dict Notes)", "field": "string (field path)", "out": "$variable"}
        },
        # head: deprecated, use load with slice instead. Executor still accepts head for backward compat.
        "sort": {
            "description": "Sort Collection by a field value (SQL ORDER BY). Notes must have JSON content with sortable field.",
            "full_description": "Sort a Collection by a field in each Note. Input Notes must have JSON content. Default ascending, use order:'desc' for descending. Notes missing the sort field are placed at end.",
            "examples": [
                '{"type":"sort","target":"$papers","by":"year","out":"$sorted_papers"}',
                '{"type":"sort","target":"$results","by":"score","order":"desc","out":"$ranked"}'
            ],
            "schema_hint": {"target": "$variable (Collection of dict Notes)", "by": "string (field path)", "order": "string (optional, 'asc' or 'desc', default 'asc')", "out": "$variable"}
        },
        "filter-structured": {
            "description": "Filter Collection by field conditions (SQL WHERE clause). Works only on Notes whose content is JSON string.",
            "full_description": "Filter a Collection using SQL-like WHERE clause. Input Notes must have JSON content. Syntax: 'field > 100', 'year >= 2020', 'score == 1.0'. Multiple conditions: 'year > 2020 AND citations >= 100'. Notes missing fields are excluded.",
            "examples": [
                '{"type":"filter-structured","target":"$papers","where":"year >= 2020","out":"$recent"}',
                '{"type":"filter-structured","target":"$results","where":"score > 0.5","out":"$high_quality"}'
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
        "search-web": "Search web and return a single Note with synthesized text. Use extract/synthesize for further processing.",
        "semantic-scholar": "Search academic papers and return Collection of Notes with text content (full paper via GROBID or abstract). Use extract/synthesize directly on the Collection.",
        "fetch-text": "Fetch text from a SINGLE specific URL. Use ONLY when you have one URL to fetch directly and do not already have the text.",
        "discover-notes": "Global discovery across all Notes. Returns Collection of Notes with text content.",
        "discover-collections": "Global discovery across all Collections. Returns Collection of Notes with text content.",
        "search-within-collection": "Search within indexed Collection. Returns Collection of Notes with matched chunk text.",
    }
    TOOL_SCHEMA_HINT_OVERRIDE = {}
    
    # Add available tools from map (exclude hidden tools from catalog)
    for tool_name, tool_meta in available_tools.items():
        # Skip hidden tools - they're executable but not shown to planner
        if tool_meta.get('hidden', False):
            continue
        
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
            if name == "ask":
                lines.append("#COMMUNICATION (ask=send+end-turn)")
            lines.extend(format_tool(name, meta))
        lines.append("")  # Blank line before workflows
    
    lines.append("Preferred tool order: World tools first when they apply, then Infospace Core")
    # Add critical workflows to prevent common mistakes
    lines.append("# CRITICAL WORKFLOWS:")
    lines.append("- search-web → synthesize (with focus) — direct Collection analysis")
    lines.append("- search-web → map(extract) → synthesize — two-phase with per-item extraction")
    lines.append("- search-web → filter-structured → synthesize — filtered then analyzed")
    lines.append("- search-web already returns substantial page content in 'text' field (keyword-filtered, up to ~8K chars per result) — use extract/synthesize directly")
    lines.append("- semantic-scholar → synthesize (with focus) — direct Collection analysis")
    lines.append("- semantic-scholar → map(extract) → synthesize — two-phase with per-item extraction")
    lines.append("- semantic-scholar → filter-structured → synthesize — filtered then analyzed")
    lines.append("- semantic-scholar already returns full paper text (via GROBID) in 'text' field of each Note — do NOT project metadata.uri for fetching")
    lines.append("- For comparison: synthesize with format=\"comparison\" and other= (requires two inputs)")
    lines.append("- fetch-text is for SINGLE URLs only, NOT for Collections from search-web/semantic-scholar")
    lines.append("- Level 4 tools (search-web, semantic-scholar) return Collections with substantial content in the 'text' field of each Note — use extract/synthesize directly")
    
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
    # Exception: 'target' in say is a character name (literal), not a variable
    # Exception: 'value' in create-note/add/say/think/ask accepts literals, so don't normalize
    # These primitives accept both literals and variables - trust LLM to add $ when needed
    variable_fields = ['out', 'source']
    # Primitives/tools that accept literal values in 'value' field (don't normalize)
    literal_value_primitives = [
        'create-note', 'create-collection', 'add', 'remove', 'say', 'think', 'ask',
        'search-web', 'semantic-scholar', 'discover-notes', 'discover-collections',
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
    output_producing = ["create-note", "create-collection", "load", "discover-notes", "discover-collections", "search-within-collection", "map", 
                       "split", "flatten", "search-web", "semantic-scholar", "extract", "synthesize",
                       "generate-note", "assess", "extract-entities", "filter-semantic", "bind",
                       "fetch-text", "as-json", "as-markdown"]
    if tool_name in output_producing and "out" not in action:
        action["out"] = f"$step_{step}_result"
    
    # Add expect if needed
    uncertain_tools = ["search-web", "semantic-scholar", "discover-notes", "discover-collections", "search-within-collection", "load"]
    if tool_name in uncertain_tools and "expect" not in action:
        action["expect"] = f"should get result from {tool_name}"
    
    return action


def format_result_text(result: Dict, action: Dict) -> str:
    """
    Format a uniform_return result dict into a text string for Stage 3 reflection.
    
    Args:
        result: uniform_return dict from execute_action_tracked / execute_action
        action: The action dict that produced this result
        
    Returns:
        Text string (format: SUCCESS | <result> | <action> | Bound: <var>)
    """
    action_type = action.get('type', 'unknown')
    if result.get('status') == 'success':
        value = result.get('value', '')
        resource_id = result.get('resource_id')
        bound_var = action.get('out', '')
        
        value_str = str(value).replace('\n', ' | ') if value else ''
        
        if value_str:
            if bound_var and resource_id:
                return f"SUCCESS | {value_str} | {action_type} completed | Bound: {bound_var} to {resource_id}"
            elif bound_var:
                return f"SUCCESS | {value_str} | {action_type} completed | Bound: {bound_var}"
            else:
                return f"SUCCESS | {value_str} | {action_type} completed"
        else:
            if bound_var and resource_id:
                return f"SUCCESS | {action_type} completed | Bound: {bound_var} to {resource_id}"
            elif bound_var:
                return f"SUCCESS | {action_type} completed | Bound: {bound_var}"
            else:
                return f"SUCCESS | {action_type} completed"
    else:
        error_reason = result.get('reason', 'Unknown error')
        return f"ERROR | {action_type} failed: {error_reason}"


def execute_infospace_action(action: Dict, executor: InfospaceExecutor, agent_name: str) -> str:
    """
    Execute single action via infospace_executor, return result text.
    
    Delegates to executor.execute_action_tracked() for all side-effects
    (plan_actions, ActionRecord, UI publish, compliance), then formats to text.
    
    Args:
        action: Infospace action dict
        executor: InfospaceExecutor instance
        agent_name: Agent name for logging
        
    Returns:
        Result text for Stage 3 reflection (format: SUCCESS | <result> | <action> | Bound: <var>)
    """
    try:
        result = executor.execute_action_tracked(action)
        return format_result_text(result, action)
    except Exception as e:
        if hasattr(executor, '_plan_error_count'):
            executor._plan_error_count += 1
        logger.error(f"Execution error: {e}")
        traceback.print_exc()
        return f"ERROR | Exception: {str(e)}"


# ============================================================================
# Code-block generation: validation, extraction, execution
# ============================================================================

_CODEGEN_FORBIDDEN_PATTERNS = [
    r'\bexec\b\s*\(', r'\beval\b\s*\(', r'\bcompile\b\s*\(',
    r'\b__\w+__\b',
    r'\bsys\b\.', r'\bsubprocess\b',
    r'\bglobals\b\s*\(', r'\blocals\b\s*\(',
    r'\bsetattr\b\s*\(', r'\bdelattr\b\s*\(',
]


def validate_codegen_block(code: str) -> tuple:
    """
    Static validation of LLM-generated code blocks before exec.
    
    Checks:
    - Forbidden patterns (imports, exec, file I/O, class/def, etc.)
    - Must contain 1-10 execute_action_tracked calls
    - Must contain at least one return executor._create_uniform_return
    - Max 512 executable code lines (comments and blanks are free)
    - Hard cap of 512 total lines
    
    Args:
        code: Python code string
        
    Returns:
        (ok: bool, reason: str) - reason is empty on success
    """
    if not code or not code.strip():
        return False, "Empty code block"
    
    lines = code.strip().splitlines()
    code_lines = [l for l in lines if l.strip() and not l.strip().startswith('#')]
    if len(code_lines) > 512:
        return False, f"Code block too long ({len(code_lines)} code lines, max 512)"
    if len(lines) > 512:
        return False, f"Code block too long ({len(lines)} total lines, max 512)"

    # Prevent premature user delivery from Stage 2; final delivery happens after done gate.
    if re.search(r'["\']type["\']\s*:\s*["\']say["\']', code):
        return False, "Forbidden action in code block: say"
    
    # Check forbidden patterns
    for pattern in _CODEGEN_FORBIDDEN_PATTERNS:
        match = re.search(pattern, code)
        if match:
            return False, f"Forbidden pattern: {match.group()}"
    
    # Count execute_action_tracked calls (must be 1-10)
    call_count = len(re.findall(r'executor\.execute_action_tracked\s*\(', code))
    if call_count < 1:
        return False, "No execute_action_tracked() calls found"
    if call_count > 10:
        return False, f"Too many execute_action_tracked() calls ({call_count}, max 10)"
    
    # Must have a return with _create_uniform_return
    if 'executor._create_uniform_return' not in code:
        return False, "Missing return executor._create_uniform_return(...)"
    
    return True, ""


def extract_code_block(raw_text: str) -> str:
    """
    Extract Python code from LLM-generated Stage 2 output for _code_block_ tool.
    
    Handles:
    - Triple backtick fenced blocks (```python ... ```)
    - CODE: label prefix
    - Raw code (fallback)
    
    Args:
        raw_text: Raw text from Stage 2 generation (tool_args area)
        
    Returns:
        Extracted Python code string
    """
    if not raw_text:
        return ""
    
    text = raw_text.strip()
    
    # Remove CODE: prefix if present
    if text.upper().startswith("CODE:"):
        text = text[5:].strip()
    
    # Try to extract from triple backtick fences
    fence_match = re.search(r'```(?:python)?\s*\n(.*?)```', text, re.DOTALL)
    if fence_match:
        return fence_match.group(1).strip()
    
    # Fallback: return everything (may already be raw code)
    return text.strip()


def count_codegen_action_calls(code: str) -> int:
    """Count execute_action_tracked calls in a generated Stage 2 code block."""
    if not code:
        return 0
    return len(re.findall(r'executor\.execute_action_tracked\s*\(', code))


def execute_codegen_block(code: str, executor, method_name: str = "codegen") -> Dict:
    """
    Validate and execute an LLM-generated code block in a sandboxed namespace.
    
    The code is wrapped in a function so that 'return' statements work.
    Executor helpers for resource access are also available in the namespace.
    
    Args:
        code: Python code string (from extract_code_block)
        executor: InfospaceExecutor instance
        method_name: Method name for UI logging (default: "codegen")
        
    Returns:
        uniform_return dict
    """
    ok, reason = validate_codegen_block(code)
    if not ok:
        logger.warning(f"Code block validation failed: {reason}")
        return executor._create_uniform_return("failed", reason=f"Code validation failed: {reason}")
    
    # Wrap code in a function so `return` works
    indented = textwrap.indent(code, "    ")
    wrapped = f"def _codegen_fn(executor):\n{indented}\n"

    def _resolve_resource_id(ref):
        if not isinstance(ref, str):
            return None
        if ref.startswith("$"):
            return executor.plan_bindings_flat.get(ref[1:])
        if ref in executor.plan_bindings_flat:
            return executor.plan_bindings_flat.get(ref)
        if ref.startswith("Note_") or ref.startswith("Collection_"):
            return ref
        return None

    def _get_resource_content(ref):
        resource_id = _resolve_resource_id(ref)
        if not resource_id:
            return None
        return executor._get_content(resource_id)

    def get_text(ref):
        content = _get_resource_content(ref)
        if content is None:
            return ""
        if isinstance(content, str):
            return content
        return json.dumps(content, ensure_ascii=False)

    def get_json(ref):
        content = _get_resource_content(ref)
        if isinstance(content, dict):
            return content
        if isinstance(content, str):
            try:
                return json.loads(content)
            except Exception:
                return None
        return None

    def get_items(ref):
        content = _get_resource_content(ref)
        if isinstance(content, list):
            return content
        return []

    namespace = {
        "executor": executor,
        "json": json,
        "logger": logger,
        "get_text": get_text,
        "get_json": get_json,
        "get_items": get_items,
    }
    try:
        exec(wrapped, namespace)
        result = namespace["_codegen_fn"](executor)
    except Exception as e:
        logger.error(f"Code block execution error: {e}")
        logger.error(traceback.format_exc())
        return executor._create_uniform_return("failed", reason=f"Code block exception: {str(e)}")
    
    if not isinstance(result, dict) or result.get("type") != "uniform_return":
        return executor._create_uniform_return("failed", reason="Code block did not return uniform_return")
    return result


def _compress_trace(trace_str: str) -> str:
    """
    Compress execution trace while preserving order and event identity.
    
    Handles both SGLang (<|im_start|>user) and vLLM (<|user|>) marker formats.
    
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
        trace_str: Full trace string from str(state) or vLLM prompt
        
    Returns:
        Compressed trace as single text string suitable for LLM prompt
    """
    if not trace_str:
        return ""
    
    # Split into sections by user/assistant markers for easier parsing
    # Handles both SGLang format (<|im_start|>user / <|im_end|>) and vLLM format (<|user|> / <|assistant|>)
    sections = re.split(r'(<\|im_start\|>(?:user|assistant)|<\|im_end\|>|<\|(?:user|assistant)\|>)', trace_str)
    compressed_events = []
    
    # Markers to skip (both formats)
    _SKIP_MARKERS = {'<|im_start|>user', '<|im_start|>assistant', '<|im_end|>', '<|user|>', '<|assistant|>'}
    
    # State tracking
    first_goal = None
    current_goal_prefix = None
    seen_keys = {}  # Track duplicate content by key
    current_method = None  # Track current method tool execution
    method_step = None
    
    i = 0
    while i < len(sections):
        section = sections[i].strip()
        if not section or section in _SKIP_MARKERS:
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


# Tools whose output is worth evaluating against the quality vision.
# Auto-detected from bindings after each step — no LLM involvement.
_ARTIFACT_PRODUCING_TOOLS = frozenset({"synthesize", "extract", "generate-note"})

# Subset of artifact-producing tools that typically produce goal-level output.
# A FAIL from the lightweight classifier on these triggers deep (subplanner) evaluation.
_GOAL_LEVEL_TOOLS = frozenset({"synthesize", "_code_block_"})


def _vision_eval_check(vision_criteria: str, eval_target: str, executor, compressed_context: str = "") -> str:
    """
    Lightweight vision evaluation against planner-declared eval_target.
    Returns evaluation text (pass/fail per criterion) or empty string if skipped.
    """
    if not vision_criteria or not eval_target:
        return ""
    
    # Resolve eval_target variable to resource content
    var_key = eval_target.lstrip('$')
    res_id = executor.plan_bindings_flat.get(var_key)
    if not res_id or not executor.resource_manager:
        return ""
    
    # Use load with slice=":" to get full content up to ceiling
    load_result = executor.execute_action({"type": "load", "target": eval_target, "slice": ":", "out": f"$_vision_eval_{var_key}"})
    artifact_preview = load_result.get('data', '')
    if not artifact_preview:
        return ""
    artifact_preview = str(artifact_preview)[:4096]
    
    # Build context section if available
    context_section = ""
    if compressed_context:
        context_section = f"\nEXECUTION CONTEXT (compressed trace of steps so far):\n{compressed_context}\n"
    
    eval_prompt = f"""Evaluate the following artifact against each quality criterion.

RULES:
- PASS means the criterion IS satisfied by the artifact. FAIL means it is NOT satisfied.
- If the artifact meets the criterion, you MUST label it PASS regardless of minor imperfections.
- Only use FAIL when the criterion is clearly unmet — do not label FAIL if your reason describes success.
- Provide a one-sentence reason for FAIL verdicts. No reason needed for PASS.

CRITERIA:
{vision_criteria}
{context_section}
ARTIFACT ({eval_target}):
{artifact_preview}

Format (one line per criterion):
criterion_name: PASS or FAIL - reason
END_EVAL"""
    
    try:
        result = executor.llm_generate(eval_prompt, max_tokens=256, temperature=0.1, stops=["\nEND_EVAL", "END_EVAL"])
        if result.success and result.text:
            eval_text = result.text.strip()
            logger.info(f"Vision eval for {eval_target}: {eval_text}")
            return eval_text
    except Exception as e:
        logger.warning(f"Vision eval failed: {e}")
    return ""


def _vision_eval_deep(vision_criteria: str, eval_target: str, classifier_result: str, executor, compressed_context: str = "") -> str:
    """
    Deep vision evaluation via subplanner. Has full access to all pipeline bindings
    and can issue load/inspect calls to verify content, coverage, and grounding.
    
    Invoked at the done gate as a read-only quality check on a goal-level artifact.
    Falls back to empty string on vLLM/OpenRouter (subplanner requires SGLang).
    
    Returns evaluation text or empty string if skipped/unavailable.
    """
    if not vision_criteria or not eval_target:
        return ""
    
    # Subplanner requires SGLang runtime
    if not HAS_SGLANG or not getattr(executor, 'runtime', None):
        logger.debug("Vision eval deep: skipped (requires SGLang runtime)")
        return ""
    
    # Build the evaluation goal for the subplanner
    bindings_summary = ""
    try:
        bindings = executor.plan_bindings_flat
        if bindings:
            binding_lines = [f"  ${k} = {v}" for k, v in bindings.items() if not k.startswith('_')]
            bindings_summary = "\n".join(binding_lines)
    except Exception:
        pass
    
    context_block = ""
    if compressed_context:
        context_block = f"\n\nEXECUTION TRACE (compressed):\n{compressed_context}"
    
    eval_goal = (
        f"You are a quality evaluator. Your task is to evaluate the artifact in {eval_target} "
        f"against the quality criteria below. You have access to ALL pipeline variables via load.\n\n"
        f"QUALITY CRITERIA:\n{vision_criteria}\n\n"
        f"CLASSIFIER RESULT (lightweight, may be based on truncated content):\n{classifier_result}\n\n"
        f"AVAILABLE VARIABLES:\n{bindings_summary}\n"
        f"{context_block}\n\n"
        f"INSTRUCTIONS:\n"
        f"1. Use load with slice=\":\" to inspect {eval_target} in full.\n"
        f"2. If criteria involve coverage or completeness, load upstream variables "
        f"(e.g., source collections) to check how many items were retained vs. discarded.\n"
        f"3. If criteria involve specificity or accuracy, load source Notes to verify "
        f"that claimed details actually appear in source material.\n"
        f"4. For each criterion, report PASS or FAIL with a one-sentence evidence-based reason.\n"
        f"5. End with a one-line RECOMMENDATION for the parent planner (e.g., 'lower filter threshold', "
        f"'add section headers', 'broaden search terms').\n"
        f"6. On the final line, write exactly: STATUS: SATISFIED (if all criteria PASS and no revision needed) "
        f"or STATUS: NEEDS_REVISION (if any FAIL or revision recommended).\n"
        f"7. Return the evaluation report as your final answer. Do NOT regenerate the artifact and do NOT use say — the report is returned "
        f"to the parent planner as text, not delivered to any user."
    )
    
    try:
        logger.info(f"Vision eval deep: starting subplanner for {eval_target}")
        result = executor.call_subplanner(goal=eval_goal, max_steps=6)
        if result and not result.startswith("Subplanner failed") and not result.startswith("Subplanner requires"):
            logger.info(f"Vision eval deep for {eval_target}: {result[:200]}")
            return result
        else:
            logger.warning(f"Vision eval deep returned: {result[:100] if result else '(empty)'}")
    except Exception as e:
        logger.warning(f"Vision eval deep failed: {e}")
    return ""


def _resolve_eval_target_text(eval_target: str, executor) -> str:
    """
    Resolve eval target ($var or Note_ID) to stored Note content string.
    Used for read-only quality-gate fallback delivery.
    """
    if not eval_target or not executor or not getattr(executor, "resource_manager", None):
        return ""
    try:
        resource_id = None
        if isinstance(eval_target, str) and eval_target.startswith("$"):
            resource_id = executor.plan_bindings_flat.get(eval_target.lstrip("$"))
        elif isinstance(eval_target, str) and eval_target.startswith("Note_"):
            resource_id = eval_target
        if not resource_id:
            return ""
        resource = executor.resource_manager.get_resource(resource_id)
        if not resource:
            return ""
        content = resource.get("properties", {}).get("content", "")
        if content is None:
            return ""
        return content if isinstance(content, str) else str(content)
    except Exception:
        return ""


def _ensure_goal_artifact_binding(executor) -> str:
    """Ensure a canonical $goal_artifact Note binding exists."""
    if not executor:
        return ""
    try:
        existing = executor.plan_bindings_flat.get("goal_artifact")
        if isinstance(existing, str) and existing.startswith("Note_"):
            return "$goal_artifact"
        executor.execute_action({"type": "create-note", "value": "", "out": "$goal_artifact"})
        return "$goal_artifact"
    except Exception as e:
        logger.warning(f"Failed to initialize $goal_artifact: {e}")
        return ""


def _sync_goal_artifact_from_target(executor, source_eval_target: str) -> str:
    """Copy source eval target text into canonical $goal_artifact Note."""
    if not executor or not source_eval_target:
        return ""
    try:
        goal_artifact_id = executor.plan_bindings_flat.get("goal_artifact")
        if not goal_artifact_id or not isinstance(goal_artifact_id, str) or not goal_artifact_id.startswith("Note_"):
            return ""
        source_text = _resolve_eval_target_text(source_eval_target, executor)
        if source_text is None:
            source_text = ""
        if executor.resource_manager:
            ok, err = executor.resource_manager.update_note_content(goal_artifact_id, source_text)
            if not ok:
                logger.warning(f"Failed to update $goal_artifact ({goal_artifact_id}): {err}")
                return ""
        return "$goal_artifact"
    except Exception as e:
        logger.warning(f"Failed to sync $goal_artifact from {source_eval_target}: {e}")
        return ""


def _parse_deep_eval_status(deep_text: str) -> str:
    """
    Parse deep-eval status robustly.
    Returns one of: "satisfied", "needs_revision", "unknown".
    """
    if not deep_text:
        return "unknown"
    text = str(deep_text).strip()
    m = re.search(r"^\s*STATUS:\s*(SATISFIED|NEEDS_REVISION)\s*$", text, flags=re.IGNORECASE | re.MULTILINE)
    if m:
        status = m.group(1).strip().upper()
        return "satisfied" if status == "SATISFIED" else "needs_revision"
    lo = text.lower()
    if any(k in lo for k in ["needs_revision", "needs revision", "revision needed", "any fail", "criterion_"]):
        if "pass" not in lo or "fail" in lo:
            return "needs_revision"
    if any(k in lo for k in ["no revision needed", "all criteria pass", "all criteria are pass", "status: satisfied"]):
        return "satisfied"
    return "unknown"


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
                              tools_catalog_text: str, executor, trace_file=None, max_steps: int = 16, similar_plan: Dict = None, preplan: str = None, vision_criteria: str = ""):
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
        
        # Interrupt checkpoint: entry to tool_planner_infospace
        if executor and _interrupt_requested(executor):
            _clear_interrupt(executor)
            s["final_answer"] = "Interrupted by user."
            return s
        
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
        system_parts.append("You can write code blocks to achieve the goal, including using tools/primitives (aka actions), if and as needed,")
        system_parts.append("and loop over execute-step / reflect until the goal is satisfied.")
        system_parts.append(f"\n{INCREMENTAL_PLAN_SPECIFICATIONS}\n")
        system_parts.append(f"Complete primitive and tool catalog:\n{tools_catalog_text}\n#### END OF INFOSPACE TYPE SYSTEM, SPECIFICATIONS, AND TOOL CATALOG\n\n")

        system_parts.append(f"Setting:\n{character_context}\n\n")
        system_parts.append(f"Situation/Context:\n{recent_context}\n\n")
        #system_parts.append(f"Again, \n#GOAL:\n\n{template}\n{goal}\n\n")
        if preplan:
            system_parts.append(f"\n## {preplan}\n")
            system_parts.append(f"\n## End ABSTRACT_PLAN\n")

        if vision_criteria:
            system_parts.append(f"\n## QUALITY VISION\nEvaluate your output against these criteria. If an intermediate artifact fails a criterion, adjust your approach (e.g., use more detailed instructions, re-extract with richer prompts).\n{vision_criteria}\n## End QUALITY VISION\n")

        if similar_plan:
            system_parts.append(f"##PREVIOUS PLAN FOR SIMILAR GOAL:\n{similar_plan['plan']}\n")
            system_parts.append(f"OUTCOME: {similar_plan['outcome']} ERRORS: {similar_plan['error_count']}\n")
        if available_resources_text:
            system_parts.append(f"\n{available_resources_text}\n")
        
        # Add world-specific prompt context (generic, no world-specific code)
        if executor:
            world_prompt_context = executor.get_world_prompt_context()
            if world_prompt_context:
                for section_name, context_text in world_prompt_context.items():
                    system_parts.append(f"\n# {section_name.upper().replace('_', ' ')}\n{context_text}\n")
        
        # Add current date and time

        system_parts.append(f"WORLD_MODEL: {json.dumps(world_model, indent=2)}\n")
        system_parts.append(f"CURRENT_TIME: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n")
        system_parts.append("""Follow this process to achieve the goal:
 - Stage 1 (once): Read the ABSTRACT_PLAN, select tools, state first task.
 - Stage 1.5 (once): Load and inject detailed docs for selected tools.
Then you will work in repeated cycles:
 - Stage 2: Write a Python code block for the current task.
 - Execute the code block.
 - Stage 3: Evaluate result — is the goal done? What is the next task?
ALWAYS follow all formatting instructions exactly.

""")        
        s += system("".join(system_parts))
        
        s += user(
            "#Stage 1: Read the ABSTRACT_PLAN above and prepare for execution.\n"
            "\n"
            "1. REASONING: Brief assessment of available resources and plan feasibility (1-2 sentences).\n"
            "\n"
            "2. TOOLS: Select tools you will need from the Complete primitive and tool catalog.\n"
            "   Include tools for the full plan, not just the first step.\n"
            "\n"
            "3. FIRST TASK: State what to do first.\n"
            "   Describe the task concisely (e.g., 'Search for papers, filter to recent,\n"
            "   extract metadata, and build comparison table').\n"
            "\n"
            "Respond using this format:\n"
            "<reasoning>\n"
            "BRIEF ASSESSMENT\n"
            "</reasoning>\n"
            "<tools>\n"
            "JSON LIST OF TOOLS\n"
            "</tools>\n"
            "<first_task>\n"
            "TASK DESCRIPTION\n"
            "</first_task>\n"
        )
        
        s += assistant(
            "<reasoning>\n"
            + gen("stage1_reasoning", max_tokens=64, temperature=GEN_TEMPERATURE, stop="</reasoning>")
            + "</reasoning>\n"
            "<tools>\n"
            + gen("selected_tools_json", max_tokens=96, temperature=GEN_TEMPERATURE, stop="</tools>")
            + "</tools>\n"
            "<first_task>\n"
            + gen("first_task", max_tokens=128, temperature=GEN_TEMPERATURE, stop="</first_task>")
            + "</first_task>\n"
        )
        
        try:
            logger.info(f"Stage 1: Reasoning: {s['stage1_reasoning']}")
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
        
        # Interrupt checkpoint: after Stage 1, before step loop
        if executor and _interrupt_requested(executor):
            _clear_interrupt(executor)
            s["final_answer"] = "Interrupted by user."
            return s
        
        # Stage 2/3 format instructions
        s += user(
            "#Stage 2 FORMAT:\n"
            "Write a Python code block to accomplish the CURRENT_TASK.\n"
            "```python\n"
            "r1 = executor.execute_action_tracked({\"type\": \"search-web\", \"query\": \"transformers survey\", \"out\": \"$papers\"}, \"codegen\")\n"
            "if r1[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=\"search failed\")\n"
            "r2 = executor.execute_action_tracked({\"type\": \"synthesize\", \"target\": \"$papers\", \"focus\": \"key findings\", \"out\": \"$summary\"}, \"codegen\")\n"
            "if r2[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=\"synthesize failed\")\n"
            "return executor._create_uniform_return(\"success\", value=\"done\")\n"
            "```\n"
            "Rules:\n"
            "- use python code and / or tool calls to accomplish the CURRENT_TASK.\n"
            "- Max 16 tool calls in a single code block via executor.execute_action_tracked(action_dict, \"codegen\")\n"
            "- Prefer longer cohesive blocks: if CURRENT_TASK has connected subtasks, combine them in one block.\n"
            "- Avoid single-call blocks unless that single call fully completes CURRENT_TASK.\n"
            "- VARIABLE BINDING: \"out\": \"$name\" binds the result. Chain tool actions via target=\"$name\".\n"
            "- RETURN CONTRACT: r[\"status\"] — \"success\"/\"failed\". r[\"resource_id\"] — resource ID.\n"
            "  r[\"data\"] — for Notes always string; for Collections list of {\"text\": content} per item.\n"
            "  Use get_text(\"$var\") or get_json(\"$var\") to inspect Note content.\n"
            "  r.get(\"extra\",{}).get(\"item_count\") — item count for Collections.\n"
            "  r[\"value\"] — display string (for humans, not for code logic).\n"
            "- CODE BLOCK SCOPE: each step executes in a fresh function. Python locals do NOT persist across steps.\n"
            "- CROSS-STEP ACCESS: use $bindings only. To inspect existing resources use helpers:\n"
            "  get_text(\"$var\") -> string, get_json(\"$var\") -> dict or None, get_items(\"$collection\") -> list.\n"
            "- TYPE SAFETY: r[\"data\"] for Notes is always string; use get_json() when structured access needed.\n"
            "- if/else control flow and loops are allowed.\n"
            "- Must end with: return executor._create_uniform_return(status, value=..., extra=...)\n"
            "\n"
            "#Stage 2 VARIABLE LIFETIME:\n"
            "  Only variables created via 'out' persist across steps or code blocks.\n"
            "  Chain by passing the variable name (e.g. target='$papers') to the next action.\n"
            "  Temporary Python variables (r1, text, data, etc.) are code-block-local only.\n"
            "\n"
            "#Stage 3 FORMAT:\n"
            "  THOUGHTS: <brief assessment of result and progress>\n"
            "  DONE: <YES or NO — is the entire GOAL satisfied?>\n"
            "  NEXT_TASK: <next task description, or blank if DONE=YES>\n"
            "  REQUEST_TOOLS: <json array of tool names needing docs, or []>\n"
            "\n"
            "#Stage 3 RULES:\n"
            "- DONE=YES only when ALL required actions are complete.\n"
            "- If DONE=YES, NEXT_TASK must be blank.\n"
            "- Do NOT use 'say' inside Stage 2 code blocks; final user delivery happens only after DONE=YES and quality checks.\n"
            "- Use bind only when you have a concrete final candidate artifact (e.g., bind $final_report -> $goal_artifact); if no final candidate yet, do not bind.\n"
            "- Efficiency rule: if prior step used only one tool call and GOAL is not done, set NEXT_TASK to combine adjacent subtasks.\n"
            "- SPIRAL DETECTION: If the same tool has failed 2+ times consecutively,\n"
            "  use a different approach or proceed with available data.\n"
            "- REQUEST_TOOLS: Always valid JSON: [] or [\"tool1\", \"tool2\"].\n"
            "\n"
            "Follow these formats exactly."
        )
        s += assistant("Understood.\n\n")

        # Main loop
        current_task = s["first_task"].strip()
        goal_artifact_var = _ensure_goal_artifact_binding(executor)
        last_eval_target = goal_artifact_var if goal_artifact_var else ""
        stall_guard_state = {"prev_signature": None, "repeat_count": 0}
        for step in range(max_steps):
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                s["final_answer"] = "Interrupted by user."
                break

            # Stage 2: Generate and execute code block
            s += user(
                f"STAGE 2 (step {step + 1}/{max_steps}):\n"
                f"#GOAL: {goal_for_step}\n#END GOAL\n"
                f"CURRENT_TASK: {current_task}\n"
                "Write a Python code block using Stage 2 FORMAT.\n"
            )
            
            s += assistant(
                "```python\n"
                + gen(f"code_block_{step}", max_tokens=2048, temperature=GEN_TEMPERATURE, stop=["```"])
                + "```\n"
            )
            
            code_text = s[f"code_block_{step}"].strip()
            logger.info(f"Step {step}: Code block ({len(code_text)} chars):\n{code_text}")
            call_count = count_codegen_action_calls(code_text)
            logger.info(f"Step {step}: Code block action calls={call_count} (logged before execution)")
            
            # Interrupt checkpoint: after code-block gen, before execute
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                s["final_answer"] = "Interrupted by user."
                break
            
            # Snapshot bindings before execution
            bindings_before = dict(executor.plan_bindings_flat)
            result_dict = execute_codegen_block(code_text, executor, "codegen")
            action = {"type": "_code_block_"}
            tool_result = format_result_text(result_dict, action)
            
            # Track new bindings
            new_bindings = {k: v for k, v in executor.plan_bindings_flat.items() if bindings_before.get(k) != v}
            
            # Persist code block return value as Note when substantial
            code_block_output_created = False
            if result_dict.get('status') == 'success':
                val = result_dict.get('value', '')
                if isinstance(val, str) and len(val) > 80:
                    executor.execute_action({"type": "create-note", "value": val, "out": "$code_block_output"})
                    code_block_output_created = True
            
            logger.info(f"Step {step}: -> {tool_result[:100]}")
            if new_bindings:
                logger.info(f"Step {step}: New bindings: {new_bindings}")
            
            # Interrupt checkpoint: after code-block execution
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                s["final_answer"] = "Interrupted by user."
                break
            
            # Stage 3: Evaluate result
            s += user(
                f"=====\n"
                f"STAGE 3 (step {step + 1}/{max_steps})\n"
                f"=====\n\n"
                f">> RESULT (ground truth) <<\n"
                f"{tool_result}\n"
                f">> END RESULT <<\n\n"
                f"Evaluate: Is the GOAL fully achieved? Use ONLY the result above as ground truth.\n"
                f"Respond using Stage 3 FORMAT.\n"
            )
            
            # Vision evaluation before planner reflects
            eval_target = ""
            if new_bindings:
                preferred_keys = [k for k, rid in new_bindings.items() if isinstance(rid, str) and (rid.startswith("Note_") or rid.startswith("Collection_")) and not str(k).startswith("_") and str(k) != "code_block_output"]
                if preferred_keys:
                    eval_target = "$" + preferred_keys[-1].lstrip("$")
                else:
                    last_key = list(new_bindings.keys())[-1]
                    if not last_key.startswith('$'):
                        last_key = '$' + last_key
                    eval_target = last_key
            elif code_block_output_created:
                eval_target = "$code_block_output"
            if eval_target:
                synced_eval_target = _sync_goal_artifact_from_target(executor, eval_target)
                last_eval_target = synced_eval_target or eval_target
            if eval_target and vision_criteria:
                logger.info(f"Step {step}: Vision eval target: {last_eval_target}")
                compressed_ctx = _compress_trace(str(s))
                vision_eval_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)
                if vision_eval_text:
                    s += user(f"VISION EVALUATION:\n{vision_eval_text}\nFAILs are advisory — attempt to address if easy, do NOT loop repeatedly. Proceed after at most three retries.\n")
                    s += assistant("Noted.\n")
            
            # Stage 3: Simplified reflection (4 fields)
            s += assistant(
                "THOUGHTS: "
                + gen(f"thoughts_{step}", max_tokens=128, temperature=GEN_TEMPERATURE, stop="\nDONE:")
                + "\nDONE: "
                + gen(f"done_{step}", max_tokens=8, temperature=GEN_TEMPERATURE, stop="\nNEXT_TASK:")
                + "\nNEXT_TASK: "
                + gen(f"next_task_{step}", max_tokens=128, temperature=GEN_TEMPERATURE, stop="\nREQUEST_TOOLS:")
                + "\nREQUEST_TOOLS: "
                + gen(f"request_tools_{step}", max_tokens=96, temperature=GEN_TEMPERATURE, stop=["\n\n", "\nTHOUGHTS:", "\nDONE:", "\nNEXT_TASK:", "\nREQUEST_TOOLS:"])
                + "\n"
            )
            
            def safe_get(state, key, default='N/A'):
                try:
                    return state[key]
                except (KeyError, TypeError, AttributeError):
                    return default
            
            logger.info(f"THOUGHTS: {safe_get(s, f'thoughts_{step}')}")
            logger.info(f"DONE: {safe_get(s, f'done_{step}')}")
            logger.info(f"NEXT_TASK: {safe_get(s, f'next_task_{step}')}")
            logger.info(f"REQUEST_TOOLS: {safe_get(s, f'request_tools_{step}')}")
            
            # Stage 3.1: Update resource indexes with commentary
            thoughts_text = s[f'thoughts_{step}'].strip()
            if thoughts_text and new_bindings:
                for var_name, rid in new_bindings.items():
                    if rid and (rid.startswith('Note_') or rid.startswith('Collection_')):
                        try:
                            if executor.resource_manager:
                                executor.resource_manager.update_resource_commentary(rid, thoughts_text)
                                logger.debug(f"Stage 3.1: Updated commentary for {rid}")
                        except Exception as e:
                            logger.debug(f"Stage 3.1: Failed to update commentary for {rid}: {e}")
            
            # Stage 3.5: Dynamic tool loading (if requested)
            requested_tools_raw = s[f"request_tools_{step}"].strip()
            requested_tools = parse_request_tools(requested_tools_raw)
            if requested_tools:
                logger.info(f"Step {step}: LLM requested additional tools: {requested_tools}")
                tools_to_load = [tool for tool in requested_tools if tool not in _loaded_skill_docs]
                if tools_to_load:
                    expanded_docs = load_skill_docs(tools_to_load, executor.available_tools)
                    if expanded_docs:
                        s += user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                        s += assistant("I have reviewed the additional tool documentation.\n")
                        logger.info(f"Stage 3.5: Loaded docs for {len(tools_to_load)} additional tools: {tools_to_load}")
                        _loaded_skill_docs.update(tools_to_load)
                else:
                    logger.debug(f"Stage 3.5: All requested tools already have docs loaded, skipping")
            elif requested_tools_raw and requested_tools_raw.lower() not in ["", "[]", "none", "null"]:
                logger.warning(f"Step {step}: Failed to parse REQUEST_TOOLS: {requested_tools_raw[:100]}")
            
            # Check if done
            done_raw = s[f"done_{step}"].strip().upper()
            next_task_raw = s[f"next_task_{step}"].strip()
            if done_raw.startswith("YES"):
                if _interrupt_requested(executor):
                    _clear_interrupt(executor)
                    s["final_answer"] = "Interrupted by user."
                    break
                
                # Verification
                s += user(
                    "STOP. Verify that the GOAL has been achieved:\n"
                    "- What observable facts in the current state satisfy the GOAL?\n"
                    "- What required actions or outcomes are missing?\n"
                    "VERIFICATION_ANSWER: <SUCCESS | PARTIAL | INCONCLUSIVE>\n"
                )
                s += assistant(
                    "VERIFICATION_ANSWER: "
                    + gen("VERIFICATION_ANSWER", max_tokens=96, temperature=GEN_TEMPERATURE, stop="\n")
                )
                logger.info(f"VERIFICATION ANSWER: {s['VERIFICATION_ANSWER']}")
                
                # Deep vision evaluation at done gate
                if vision_criteria and last_eval_target:
                    logger.info(f"Done gate: Running deep vision eval on {last_eval_target}")
                    compressed_ctx = _compress_trace(str(s))
                    shallow_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)
                    deep_text = _vision_eval_deep(vision_criteria, last_eval_target, shallow_text or "", executor, compressed_context=compressed_ctx)
                    if deep_text:
                        s += user(f"DEEP VISION EVALUATION (final quality gate, read-only):\n{deep_text}\nUse this only as QA signal. Do NOT run additional tool steps at done gate.\n")
                        s += assistant("Noted.\n")
                        logger.info(f"Done gate: Deep eval result injected")
                        deep_status = _parse_deep_eval_status(deep_text)
                        if deep_status == "needs_revision":
                            logger.info("Done gate: Deep eval returned NEEDS_REVISION; returning caveated frozen artifact (no reopen, no regeneration)")
                            draft_text = _resolve_eval_target_text(last_eval_target, executor)
                            caveat = (
                                "Quality gate warning: deep evaluation found issues (STATUS: NEEDS_REVISION). "
                                "Delivering best available draft from this cycle (not revised)."
                            )
                            s["VERIFICATION_ANSWER"] = "PARTIAL"
                            s[f"done_{step}"] = "NO"
                            s["final_answer"] = f"{caveat}\n\n{draft_text}" if draft_text else caveat
                            break
                    elif shallow_text and "fail" in shallow_text.lower():
                        s += user(f"VISION EVALUATION (final quality gate, shallow, read-only):\n{shallow_text}\nFAILs are advisory. Do NOT run additional tool steps at done gate.\n")
                        s += assistant("Noted.\n")
                        logger.info("Done gate: Shallow eval has FAILs (read-only advisory); proceeding to finalization")
                
                # Generate final answer
                next_task_raw = s[f"next_task_{step}"].strip()
                if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                    final_prompt = next_task_raw
                else:
                    final_prompt = "Summarize the results with focus on the original goal"
                
                s += user(f"FINAL TASK: {final_prompt}\nProvide a final response or answer. End your response with </end>")
                s += assistant(gen("final_answer", max_tokens=256, temperature=GEN_TEMPERATURE, stop=["</end>"]))
                logger.info(f"FINAL_ANSWER: {s['final_answer']}")
                break

            stall_reason = _stall_guard_check(
                stall_guard_state,
                done_raw=done_raw,
                next_task_raw=next_task_raw,
                code_text=code_text
            )
            if stall_reason:
                logger.info(stall_reason)
                draft_text = _resolve_eval_target_text(last_eval_target, executor) if last_eval_target else ""
                caveat = "Loop guard: repeated planning pattern detected. Delivering best available draft from this cycle."
                s["VERIFICATION_ANSWER"] = "PARTIAL"
                s[f"done_{step}"] = "NO"
                s["final_answer"] = f"{caveat}\n\n{draft_text}" if draft_text else caveat
                break
            
            # Update current task for next iteration
            if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                current_task = next_task_raw
                logger.info(f"Step {step}: Next task: {current_task}")
            else:
                logger.warning(f"Step {step}: No NEXT_TASK provided, stopping")
        
        # Interrupt checkpoint: exit from tool_planner_infospace
        if executor and _interrupt_requested(executor):
            _clear_interrupt(executor)
            s["final_answer"] = "Interrupted by user."
        
        # Step-limit fallback: one last synthesis pass if we have a usable candidate.
        if "final_answer" not in s:
            draft_text = _resolve_eval_target_text(last_eval_target, executor) if last_eval_target else ""
            if isinstance(draft_text, str) and draft_text.strip():
                logger.info("Step limit reached: running one last-chance synthesize pass")
                synth_r = executor.execute_action({
                    "type": "synthesize",
                    "target": last_eval_target,
                    "focus": "Produce the best final artifact for the goal using available draft content.",
                    "format": "comprehensive",
                    "out": "$last_chance_final"
                })
                final_text = draft_text
                if isinstance(synth_r, dict) and synth_r.get("status") == "success":
                    executor.execute_action({"type": "bind", "target": "$last_chance_final", "out": "$goal_artifact"})
                    synthesized = _resolve_eval_target_text("$last_chance_final", executor)
                    if isinstance(synthesized, str) and synthesized.strip():
                        final_text = synthesized
                caveat = "Step limit reached. Delivering best available draft after one last-chance synthesis pass."
                s["VERIFICATION_ANSWER"] = "PARTIAL"
                if max_steps > 0:
                    s[f"done_{max_steps - 1}"] = "NO"
                s["final_answer"] = f"{caveat}\n\n{final_text}"
            else:
                caveat = "Step limit reached before a usable final candidate artifact was produced."
                s["VERIFICATION_ANSWER"] = "PARTIAL"
                if max_steps > 0:
                    s[f"done_{max_steps - 1}"] = "NO"
                s["final_answer"] = caveat
        
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


# ============================================================================
# vLLM-based planner (alternative to SGLang version)
# ============================================================================

def format_system(text: str) -> str:
    """Format system message for vLLM prompt."""
    return f"<|system|>\n{text}\n"

def format_user(text: str) -> str:
    """Format user message for vLLM prompt."""
    return f"<|user|>\n{text}\n"

def format_assistant(text: str) -> str:
    """Format assistant message for vLLM prompt."""
    return f"<|assistant|>\n{text}\n"

def _parse_prompt_to_messages(prompt: str) -> List[Dict[str, str]]:
    """
    Parse prompt string with role markers into messages array.
    
    Args:
        prompt: Prompt string with <|system|>, <|user|>, <|assistant|> markers
        
    Returns:
        List of message dicts with 'role' and 'content' keys
    """
    messages = []
    current_role = "user"
    current_content = []
    
    lines = prompt.split('\n')
    for line in lines:
        if line.startswith('<|system|>'):
            if current_content:
                messages.append({"role": current_role, "content": '\n'.join(current_content)})
            current_role = "system"
            current_content = []
            if len(line) > 11:
                current_content.append(line[11:].strip())
        elif line.startswith('<|user|>'):
            if current_content:
                messages.append({"role": current_role, "content": '\n'.join(current_content)})
            current_role = "user"
            current_content = []
            if len(line) > 9:
                current_content.append(line[9:].strip())
        elif line.startswith('<|assistant|>'):
            if current_content:
                messages.append({"role": current_role, "content": '\n'.join(current_content)})
            current_role = "assistant"
            current_content = []
            if len(line) > 14:
                current_content.append(line[14:].strip())
        else:
            current_content.append(line)
    
    if current_content:
        messages.append({"role": current_role, "content": '\n'.join(current_content)})
    
    if not messages:
        messages = [{"role": "user", "content": prompt}]
    
    return messages


def vllm_gen(slot_name: str, prompt: str, state: Dict[str, Any], max_tokens: int, 
             temperature: float, stop: Any = None, executor: InfospaceExecutor = None) -> str:
    """
    Replacement for SGLang gen(). Uses executor.llm_generate() for unified LLM interface.
    Appends response to prompt, stores in state.
    
    Args:
        slot_name: Name for state storage
        prompt: Current prompt string (will be converted to messages array)
        state: State dictionary for storing results
        max_tokens: Max tokens to generate
        temperature: Temperature setting
        stop: Stop sequence(s) - can be string, list, or None
        executor: InfospaceExecutor instance (required for llm_generate)
        
    Returns:
        Generated text string
    """
    if not executor:
        raise ValueError("executor is required for vllm_gen()")
    
    # Convert stop to list format
    stop_list = []
    if stop:
        if isinstance(stop, list):
            stop_list = stop
        else:
            stop_list = [stop]
    
    messages = _parse_prompt_to_messages(prompt)
    
    # Use unified llm_generate interface
    try:
        response = executor.llm_generate(
            messages=messages,
            max_tokens=max_tokens+256,  # allow for reasoning
            temperature=temperature,
            stops=stop_list if stop_list else None
        )
        
        if not response.success:
            error_msg = f"llm_generate failed: {response.error}"
            logger.error(error_msg)
            raise RuntimeError(error_msg)
        
        text = response.text
        if text is None:
            error_msg = "llm_generate returned None text"
            logger.error(error_msg)
            raise RuntimeError(error_msg)
        
        state[slot_name] = text  # Store in state for later access
        return text
    except Exception as e:
        logger.error(f"vllm_gen failed: {e}")
        raise  # Fail fast


def vllm_gen_multi(prompt: str, state: Dict[str, Any], specs: List[Dict], 
                   executor: InfospaceExecutor = None) -> str:
    """
    Generate multiple slots in a single LLM call (mirrors SGLang's chained gen() in assistant block).
    
    Each spec dict has:
        - slot_name: Name for state storage
        - prefix: Text to prepend before this generation (e.g., "\\nTOOL_NAME: ")
        - suffix: Text to append after generation (e.g., "\\n")
        - max_tokens: Max tokens for this slot
        - temperature: Temperature for generation (uses max across all specs)
        - stop: Stop sequence(s) for this slot
    
    The function makes a single LLM call with the combined template, then parses out
    individual slot values using the stop sequences as delimiters.
    
    Args:
        prompt: Current prompt string (will be converted to messages array)
        state: State dictionary for storing results
        specs: List of generation specs
        executor: InfospaceExecutor instance (required for llm_generate)
        
    Returns:
        Combined generated text (to append to prompt)
    """
    if not executor:
        raise ValueError("executor is required for vllm_gen_multi()")
    
    if not specs:
        return ""
    
    # Build the template with all prefixes/suffixes
    # The LLM will generate text that fills in between the fixed parts
    template_parts = []
    total_max_tokens = 0
    max_temperature = 0.0
    final_stops = []
    
    for i, spec in enumerate(specs):
        prefix = spec.get('prefix', '')
        suffix = spec.get('suffix', '')
        max_tokens = spec.get('max_tokens', 128)
        temperature = spec.get('temperature', GEN_TEMPERATURE)
        stop = spec.get('stop')
        
        template_parts.append(prefix)
        total_max_tokens += max_tokens
        max_temperature = max(max_temperature, temperature)
        
        # Collect stop sequences - the final slot's stop sequences are used for the LLM call
        if i == len(specs) - 1 and stop:
            if isinstance(stop, list):
                final_stops.extend(stop)
            else:
                final_stops.append(stop)
    
    # Parse prompt to messages
    messages = _parse_prompt_to_messages(prompt)
    
    # Make single LLM call
    try:
        response = executor.llm_generate(
            messages=messages,
            max_tokens=total_max_tokens + 256,  # allow for reasoning
            temperature=max_temperature,
            stops=final_stops if final_stops else None
        )
        
        if not response.success:
            error_msg = f"llm_generate failed: {response.error}"
            logger.error(error_msg)
            raise RuntimeError(error_msg)
        
        full_text = response.text
        if full_text is None:
            error_msg = "llm_generate returned None text"
            logger.error(error_msg)
            raise RuntimeError(error_msg)
        
        # Parse out individual slot values using stop sequences as delimiters
        remaining_text = full_text
        result_parts = []
        
        for i, spec in enumerate(specs):
            slot_name = spec['slot_name']
            prefix = spec.get('prefix', '')
            suffix = spec.get('suffix', '')
            stop = spec.get('stop')
            
            result_parts.append(prefix)
            
            # Find where this slot's content ends
            if i < len(specs) - 1:
                # Not the last slot - find the next slot's prefix or this slot's stop
                next_prefix = specs[i + 1].get('prefix', '')
                
                # Normalize stops to list
                stop_list = []
                if stop:
                    if isinstance(stop, list):
                        stop_list = stop
                    else:
                        stop_list = [stop]
                
                # Find earliest delimiter
                end_pos = len(remaining_text)
                matched_stop = None
                
                # Check for stop sequences first
                for s in stop_list:
                    pos = remaining_text.find(s)
                    if pos != -1 and pos < end_pos:
                        end_pos = pos
                        matched_stop = s
                
                # Check for next prefix if no stop found earlier
                if next_prefix and matched_stop is None:
                    pos = remaining_text.find(next_prefix)
                    if pos != -1 and pos < end_pos:
                        end_pos = pos
                
                slot_value = remaining_text[:end_pos]
                state[slot_name] = slot_value
                result_parts.append(slot_value)
                result_parts.append(suffix)
                
                # Advance past the slot content and any matched stop
                if matched_stop and remaining_text[end_pos:].startswith(matched_stop):
                    remaining_text = remaining_text[end_pos + len(matched_stop):]
                else:
                    remaining_text = remaining_text[end_pos:]
                
                # Skip next prefix if it's at the start
                if next_prefix and remaining_text.startswith(next_prefix):
                    remaining_text = remaining_text[len(next_prefix):]
            else:
                # Last slot - use all remaining text
                state[slot_name] = remaining_text
                result_parts.append(remaining_text)
                result_parts.append(suffix)
        
        return ''.join(result_parts)
    except Exception as e:
        logger.error(f"vllm_gen_multi failed: {e}")
        raise


def tool_planner_infospace_vllm(template, goal: str, world_model: Dict, character_context: str, recent_context: str, 
                                 tools_catalog_text: str, executor, trace_file=None, max_steps: int = 16, 
                                 similar_plan: Dict = None, preplan: str = None, vllm_url: str = None,
                                 model: str = None, vision_criteria: str = ""):
    """
    vLLM-based incremental planner for infospace goals (alternative to SGLang version).
    
    Uses executor.llm_generate() for unified LLM interface.
    
    Args:
        template: Template text
        goal: Goal text
        world_model: World model dict
        character_context: Character description + drives
        recent_context: Recent thoughts/memories + last action
        tools_catalog_text: Formatted tool catalog
        executor: InfospaceExecutor instance (with _plan_actions attribute and vllm_model/vllm_url configured)
        trace_file: Optional trace file
        max_steps: Maximum planning steps
        similar_plan: Optional similar plan dict
        preplan: Optional preplan text
        vllm_url: vLLM API endpoint (deprecated - now read from executor.vllm_url)
        model: Model name for vLLM (deprecated - now read from executor.vllm_model)
    """
    # Initialize prompt (string) and state (dict) - replaces SGLang's ProgramState
    prompt = ""
    state = {}

    # -------------------- deterministic parsing helpers (Approach A) --------------------
    def _extract_tag_block(text: str, tag: str) -> str:
        """
        Extract <tag>...</tag> content. Robust to whitespace/newlines after the tag name.
        """
        open_prefix = f"<{tag}"
        close_tag = f"</{tag}>"
        start = text.find(open_prefix)
        if start == -1:
            return ""
        start_gt = text.find(">", start)
        if start_gt == -1:
            return ""
        start_content = start_gt + 1
        end = text.find(close_tag, start_content)
        if end == -1:
            return ""
        return text[start_content:end].strip()

    def _extract_after_label(text: str, label: str) -> str:
        idx = text.find(label)
        if idx == -1:
            return ""
        return text[idx + len(label):]

    def _extract_line_value(text: str, label: str) -> str:
        remainder = _extract_after_label(text, label)
        if not remainder:
            return ""
        # Strip leading whitespace and take first line
        remainder = remainder.lstrip()
        line = remainder.splitlines()[0] if remainder.splitlines() else remainder
        return line.strip()

    def _strip_code_fences(text: str) -> str:
        t = text.strip()
        if t.startswith("```"):
            # Remove the first fence line and the last fence if present
            lines = t.splitlines()
            if len(lines) >= 2:
                # drop first line (``` or ```json)
                lines = lines[1:]
                # drop last line if it's ```
                if lines and lines[-1].strip() == "```":
                    lines = lines[:-1]
                return "\n".join(lines).strip()
        return t

    def _extract_braced_json_object(text: str) -> str:
        """Extract first balanced {...} JSON object from text (best-effort)."""
        t = _strip_code_fences(text)
        start = t.find("{")
        if start == -1:
            return t.strip()
        depth = 0
        in_str = False
        esc = False
        for i in range(start, len(t)):
            ch = t[i]
            if esc:
                esc = False
                continue
            if ch == "\\":
                esc = True
                continue
            if ch == "\"":
                in_str = not in_str
                continue
            if in_str:
                continue
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    return t[start:i + 1].strip()
        return t[start:].strip()

    def _extract_between_labels(text: str, start_label: str, end_label: str) -> str:
        start_idx = text.find(start_label)
        if start_idx == -1:
            return ""
        start_idx += len(start_label)
        end_idx = text.find(end_label, start_idx)
        if end_idx == -1:
            return text[start_idx:].strip()
        return text[start_idx:end_idx].strip()
    
    # Extract goal for step prompts (truncate at ## CONTEXT ## if present)
    goal_for_step = _extract_goal_for_step(goal)
    
    # Interrupt checkpoint: entry to tool_planner_infospace
    if executor and _interrupt_requested(executor):
        _clear_interrupt(executor)
        state["final_answer"] = "Interrupted by user."
        return state
    
    # Track which tools have had their skill docs loaded
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
    system_parts.append("You can write code blocks to achieve the goal, including using tools/primitives (aka actions), if and as needed,")
    system_parts.append("and loop over execute-step / reflect until the goal is satisfied.")
    system_parts.append(f"\n{INCREMENTAL_PLAN_SPECIFICATIONS}\n")
    system_parts.append(f"Complete primitive and tool catalog:\n{tools_catalog_text}\n#### END OF INFOSPACE TYPE SYSTEM, SPECIFICATIONS, AND TOOL CATALOG\n\n")
    
    system_parts.append(f"Setting:\n{character_context}\n\n")
    system_parts.append(f"Situation/Context:\n{recent_context}\n\n")
    if preplan:
        system_parts.append(f"\n## {preplan}\n")
        system_parts.append(f"\n## End ABSTRACT_PLAN\n")
    
    if vision_criteria:
        system_parts.append(f"\n## QUALITY VISION\nEvaluate your output against these criteria. If an intermediate artifact fails a criterion, adjust your approach (e.g., use more detailed instructions, re-extract with richer prompts).\n{vision_criteria}\n## End QUALITY VISION\n")
    
    if similar_plan:
        system_parts.append(f"##PREVIOUS PLAN FOR SIMILAR GOAL:\n{similar_plan['plan']}\n")
        system_parts.append(f"OUTCOME: {similar_plan['outcome']} ERRORS: {similar_plan['error_count']}\n")
    if available_resources_text:
        system_parts.append(f"\n{available_resources_text}\n")
    
    # Add world-specific prompt context (generic, no world-specific code)
    if executor:
        world_prompt_context = executor.get_world_prompt_context()
        if world_prompt_context:
            for section_name, context_text in world_prompt_context.items():
                system_parts.append(f"\n# {section_name.upper().replace('_', ' ')}\n{context_text}\n")
    
    system_parts.append(f"WORLD_MODEL: {json.dumps(world_model, indent=2)}\n")
    system_parts.append(f"CURRENT_TIME: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    system_parts.append("""Follow this process to achieve the goal:
 - Stage 1 (once): Read the ABSTRACT_PLAN, select tools, state first task.
 - Stage 1.5 (once): Load and inject detailed docs for selected tools.
Then you will work in repeated cycles:
 - Stage 2: Write a Python code block for the current task.
 - Execute the code block.
 - Stage 3: Evaluate result — is the goal done? What is the next task?
ALWAYS follow all formatting instructions exactly.

""")
    
    prompt += format_system("".join(system_parts))
    
    prompt += format_user(
        "#Stage 1: Read the ABSTRACT_PLAN above and prepare for execution.\n"
        "\n"
        "1. REASONING: Brief assessment of available resources and plan feasibility (1-2 sentences).\n"
        "\n"
        "2. TOOLS: Select tools you will need from the Complete primitive and tool catalog.\n"
        "   Include tools for the full plan, not just the first step.\n"
        "\n"
        "3. FIRST TASK: State what to do first.\n"
        "   Describe the task concisely (e.g., 'Search for papers, filter to recent,\n"
        "   extract metadata, and build comparison table').\n"
        "\n"
        "Respond using this format:\n"
        "<reasoning>\n"
        "BRIEF ASSESSMENT\n"
        "</reasoning>\n"
        "<tools>\n"
        "JSON LIST OF TOOLS\n"
        "</tools>\n"
        "<first_task>\n"
        "TASK DESCRIPTION\n"
        "</first_task>\n"
    )
    
    prompt += format_assistant("")
    stage1_block = vllm_gen("stage1_block", prompt, state, max_tokens=256, temperature=GEN_TEMPERATURE, stop="</first_task>", executor=executor)
    if "</first_task>" in stage1_block:
        stage1_block = stage1_block.split("</first_task>", 1)[0].rstrip()
    prompt += stage1_block + "</first_task>\n"
    stage1_full = stage1_block + "</first_task>"
    state["stage1_reasoning"] = _extract_tag_block(stage1_full, "reasoning")
    state["selected_tools_json"] = _extract_tag_block(stage1_full, "tools")
    state["first_task"] = _extract_tag_block(stage1_full, "first_task")
    
    try:
        logger.info(f"Stage 1: Reasoning: {state.get('stage1_reasoning', 'N/A')}")
        logger.info(f"SELECTED_TOOLS_JSON: {state.get('selected_tools_json', 'N/A')}")
        logger.info(f"FIRST_TASK: {state.get('first_task', 'N/A')}")
    except KeyError as e:
        logger.warning(f"Stage 1 values not available: {e}")
    
    # Stage 1.5: Load and inject detailed docs for selected tools
    try:
        selected_tools_json = state.get('selected_tools_json', '')
        selected_tools = parse_request_tools(selected_tools_json)
        if selected_tools and isinstance(selected_tools, list) and selected_tools:
            tools_to_load = [tool for tool in selected_tools if tool not in _loaded_skill_docs]
            
            if tools_to_load:
                expanded_docs = load_skill_docs(tools_to_load, executor.available_tools)
                if expanded_docs:
                    prompt += format_user(expanded_docs)
                    prompt += format_assistant("I have reviewed the detailed documentation for the selected tools.\n")
                    logger.info(f"Stage 1.5: Injected detailed docs for {len(tools_to_load)} tools")
                    _loaded_skill_docs.update(tools_to_load)
            else:
                logger.debug(f"Stage 1.5: All {len(selected_tools)} selected tools already have docs loaded, skipping")
    except (KeyError, json.JSONDecodeError, TypeError) as e:
        logger.warning(f"Failed to parse selected tools for doc expansion: {e}")
    
    # Interrupt checkpoint: after Stage 1, before step loop
    if executor and _interrupt_requested(executor):
        _clear_interrupt(executor)
        state["final_answer"] = "Interrupted by user."
        return state
    
    # Stage 2/3 format instructions
    prompt += format_user(
        "#Stage 2 FORMAT:\n"
        "Write a Python code block to accomplish the CURRENT_TASK.\n"
        "```python\n"
        "r1 = executor.execute_action_tracked({\"type\": \"search-web\", \"query\": \"transformers survey\", \"out\": \"$papers\"}, \"codegen\")\n"
        "if r1[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=\"search failed\")\n"
        "r2 = executor.execute_action_tracked({\"type\": \"synthesize\", \"target\": \"$papers\", \"focus\": \"key findings\", \"out\": \"$summary\"}, \"codegen\")\n"
        "if r2[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=\"synthesize failed\")\n"
        "return executor._create_uniform_return(\"success\", value=\"done\")\n"
        "```\n"
        "Rules:\n"
        "- use python code and / or tool calls to accomplish the CURRENT_TASK.\n"
        "- Max 16 tool calls in a single code block via executor.execute_action_tracked(action_dict, \"codegen\")\n"
        "- Prefer longer cohesive blocks: if CURRENT_TASK has connected subtasks, combine them in one block.\n"
        "- Avoid single-call blocks unless that single call fully completes CURRENT_TASK.\n"
        "- VARIABLE BINDING: \"out\": \"$name\" binds the result. Chain tool actions via target=\"$name\".\n"
        "- RETURN CONTRACT: r[\"status\"] — \"success\"/\"failed\". r[\"resource_id\"] — resource ID.\n"
        "  r[\"data\"] — for Notes always string; for Collections list of {\"text\": content} per item.\n"
        "  Use get_text(\"$var\") or get_json(\"$var\") to inspect Note content.\n"
        "  r.get(\"extra\",{}).get(\"item_count\") — item count for Collections.\n"
        "  r[\"value\"] — display string (for humans, not for code logic).\n"
        "- CODE BLOCK SCOPE: each step executes in a fresh function. Python locals do NOT persist across steps.\n"
        "- CROSS-STEP ACCESS: use $bindings only. To inspect existing resources use helpers:\n"
        "  get_text(\"$var\") -> string, get_json(\"$var\") -> dict or None, get_items(\"$collection\") -> list.\n"
        "- TYPE SAFETY: r[\"data\"] for Notes is always string; use get_json() when structured access needed.\n"
        "- if/else control flow and loops are allowed.\n"
        "- Must end with: return executor._create_uniform_return(status, value=..., extra=...)\n"
        "\n"
        "#Stage 2 VARIABLE LIFETIME:\n"
        "  Only variables created via 'out' persist across steps or code blocks.\n"
        "  Chain by passing the variable name (e.g. target='$papers') to the next action.\n"
        "  Temporary Python variables (r1, text, data, etc.) are code-block-local only.\n"
        "\n"
        "#Stage 3 FORMAT:\n"
        "  THOUGHTS: <brief assessment of result and progress>\n"
        "  DONE: <YES or NO — is the entire GOAL satisfied?>\n"
        "  NEXT_TASK: <next task description, or blank if DONE=YES>\n"
        "  REQUEST_TOOLS: <json array of tool names needing docs, or []>\n"
        "\n"
        "#Stage 3 RULES:\n"
        "- DONE=YES only when ALL required actions are complete.\n"
        "- If DONE=YES, NEXT_TASK must be blank.\n"
        "- Do NOT use 'say' inside Stage 2 code blocks; final user delivery happens only after DONE=YES and quality checks.\n"
        "- Use bind only when you have a concrete final candidate artifact (e.g., bind $final_report -> $goal_artifact); if no final candidate yet, do not bind.\n"
        "- Efficiency rule: if prior step used only one tool call and GOAL is not done, set NEXT_TASK to combine adjacent subtasks.\n"
        "- SPIRAL DETECTION: If the same tool has failed 2+ times consecutively,\n"
        "  use a different approach or proceed with available data.\n"
        "- REQUEST_TOOLS: Always valid JSON: [] or [\"tool1\", \"tool2\"].\n"
        "\n"
        "Follow these formats exactly."
    )
    
    prompt += format_assistant("Understood.\n\n")
    
    # Main loop
    current_task = state.get("first_task", "").strip()
    if not current_task:
        logger.warning("No first_task found, using goal as initial task")
        current_task = goal_for_step
    goal_artifact_var = _ensure_goal_artifact_binding(executor)
    last_eval_target = goal_artifact_var if goal_artifact_var else ""
    stall_guard_state = {"prev_signature": None, "repeat_count": 0}
    for step in range(max_steps):
        if _interrupt_requested(executor):
            _clear_interrupt(executor)
            state["final_answer"] = "Interrupted by user."
            break
        
        # Stage 2: Generate code block
        prompt += format_user(
            f"STAGE 2 (step {step + 1}/{max_steps}):\n"
            f"#GOAL: {goal_for_step}\n#END GOAL\n"
            f"CURRENT_TASK: {current_task}\n"
            "Write a Python code block using Stage 2 FORMAT.\n"
        )

        prompt += format_assistant("```python\n")
        code_raw = vllm_gen(f"code_block_{step}", prompt, state, max_tokens=2048, temperature=GEN_TEMPERATURE, stop=["```"], executor=executor)
        prompt += code_raw + "```\n"
        code_text = code_raw.strip()
        
        logger.info(f"Step {step}: Code block ({len(code_text)} chars):\n{code_text}")
        call_count = count_codegen_action_calls(code_text)
        logger.info(f"Step {step}: Code block action calls={call_count} (logged before execution)")
        
        # Interrupt checkpoint: after code-block gen, before execute
        if _interrupt_requested(executor):
            _clear_interrupt(executor)
            state["final_answer"] = "Interrupted by user."
            break
        
        # Snapshot bindings before execution
        bindings_before = dict(executor.plan_bindings_flat)
        result_dict = execute_codegen_block(code_text, executor, "codegen")
        action = {"type": "_code_block_"}
        tool_result = format_result_text(result_dict, action)
        
        # Track new bindings
        new_bindings = {k: v for k, v in executor.plan_bindings_flat.items() if bindings_before.get(k) != v}
        
        # Persist code block return value as Note when substantial
        code_block_output_created = False
        if result_dict.get('status') == 'success':
            val = result_dict.get('value', '')
            if isinstance(val, str) and len(val) > 80:
                executor.execute_action({"type": "create-note", "value": val, "out": "$code_block_output"})
                code_block_output_created = True
        
        logger.info(f"Step {step}: -> {tool_result[:100]}")
        if new_bindings:
            logger.info(f"Step {step}: New bindings: {new_bindings}")
        
        # Interrupt checkpoint: after code-block execution
        if _interrupt_requested(executor):
            _clear_interrupt(executor)
            state["final_answer"] = "Interrupted by user."
            break
        
        # Stage 3: Evaluate result
        prompt += format_user(
            f"=====\n"
            f"STAGE 3 (step {step + 1}/{max_steps})\n"
            f"=====\n\n"
            f">> RESULT (ground truth) <<\n"
            f"{tool_result}\n"
            f">> END RESULT <<\n\n"
            f"Evaluate: Is the GOAL fully achieved? Use ONLY the result above as ground truth.\n"
            f"Respond using Stage 3 FORMAT.\n"
        )
        
        # Vision evaluation before planner reflects
        eval_target = ""
        if new_bindings:
            preferred_keys = [k for k, rid in new_bindings.items() if isinstance(rid, str) and (rid.startswith("Note_") or rid.startswith("Collection_")) and not str(k).startswith("_") and str(k) != "code_block_output"]
            if preferred_keys:
                eval_target = "$" + preferred_keys[-1].lstrip("$")
            else:
                last_key = list(new_bindings.keys())[-1]
                if not last_key.startswith('$'):
                    last_key = '$' + last_key
                eval_target = last_key
        elif code_block_output_created:
            eval_target = "$code_block_output"
        if eval_target:
            synced_eval_target = _sync_goal_artifact_from_target(executor, eval_target)
            last_eval_target = synced_eval_target or eval_target
        if eval_target and vision_criteria:
            logger.info(f"Step {step}: Vision eval target: {last_eval_target}")
            compressed_ctx = _compress_trace(prompt)
            vision_eval_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)
            if vision_eval_text:
                prompt += format_user(f"VISION EVALUATION:\n{vision_eval_text}\nFAILs are advisory — attempt to address if easy, do NOT loop repeatedly. Proceed after at most three retries.\n")
                prompt += format_assistant("Noted.\n")

        # Stage 3: simplified reflection (4 fields)
        prompt += format_assistant("")
        stage3_block = vllm_gen(f"stage3_block_{step}", prompt, state, max_tokens=384, temperature=GEN_TEMPERATURE, executor=executor)
        prompt += stage3_block + "\n"

        thoughts_val = _extract_between_labels(stage3_block, "THOUGHTS:", "DONE:")
        done_val = _extract_line_value(stage3_block, "DONE:")
        next_task_val = _extract_between_labels(stage3_block, "NEXT_TASK:", "REQUEST_TOOLS:")
        request_tools_val = _extract_after_label(stage3_block, "REQUEST_TOOLS:").strip()

        state[f"thoughts_{step}"] = thoughts_val
        state[f"done_{step}"] = done_val
        state[f"next_task_{step}"] = next_task_val
        state[f"request_tools_{step}"] = _strip_code_fences(request_tools_val)
        
        def safe_get(state_dict, key, default='N/A'):
            try:
                return state_dict[key]
            except (KeyError, TypeError, AttributeError):
                return default
        
        logger.info(f"THOUGHTS: {safe_get(state, f'thoughts_{step}')}")
        logger.info(f"DONE: {safe_get(state, f'done_{step}')}")
        logger.info(f"NEXT_TASK: {safe_get(state, f'next_task_{step}')}")
        logger.info(f"REQUEST_TOOLS: {safe_get(state, f'request_tools_{step}')}")
        
        # Stage 3.1: Update resource indexes with commentary
        thoughts_text = state.get(f'thoughts_{step}', '').strip()
        if thoughts_text and new_bindings:
            for var_name, rid in new_bindings.items():
                if rid and (rid.startswith('Note_') or rid.startswith('Collection_')):
                    try:
                        if executor.resource_manager:
                            executor.resource_manager.update_resource_commentary(rid, thoughts_text)
                            logger.debug(f"Stage 3.1: Updated commentary for {rid}")
                    except Exception as e:
                        logger.debug(f"Stage 3.1: Failed to update commentary for {rid}: {e}")
        
        # Stage 3.5: Dynamic tool loading
        requested_tools_raw = state.get(f"request_tools_{step}", "").strip()
        requested_tools = parse_request_tools(requested_tools_raw)
        if requested_tools:
            logger.info(f"Step {step}: LLM requested additional tools: {requested_tools}")
            tools_to_load = [tool for tool in requested_tools if tool not in _loaded_skill_docs]
            if tools_to_load:
                expanded_docs = load_skill_docs(tools_to_load, executor.available_tools)
                if expanded_docs:
                    prompt += format_user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                    prompt += format_assistant("I have reviewed the additional tool documentation.\n")
                    logger.info(f"Stage 3.5: Loaded docs for {len(tools_to_load)} additional tools: {tools_to_load}")
                    _loaded_skill_docs.update(tools_to_load)
            else:
                logger.debug(f"Stage 3.5: All requested tools already have docs loaded, skipping")
        elif requested_tools_raw and requested_tools_raw.lower() not in ["", "[]", "none", "null"]:
            logger.warning(f"Step {step}: Failed to parse REQUEST_TOOLS: {requested_tools_raw[:100]}")
        
        # Check if done
        done_raw = state.get(f"done_{step}", "").strip().upper()
        next_task_raw = state.get(f"next_task_{step}", "").strip()
        if done_raw.startswith("YES"):
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                state["final_answer"] = "Interrupted by user."
                break
            
            # Verification
            prompt += format_user(
                "STOP. Verify that the GOAL has been achieved:\n"
                "- What observable facts in the current state satisfy the GOAL?\n"
                "- What required actions or outcomes are missing?\n"
                "VERIFICATION_ANSWER: <SUCCESS | PARTIAL | INCONCLUSIVE>\n"
            )
            prompt += format_assistant("VERIFICATION_ANSWER: ")
            verification_raw = vllm_gen("VERIFICATION_ANSWER", prompt, state, max_tokens=96, temperature=GEN_TEMPERATURE, stop="\n", executor=executor)
            vlines = [ln.strip() for ln in str(verification_raw).splitlines() if ln.strip()]
            cleaned = vlines[-1] if vlines else str(verification_raw).strip()
            if cleaned.startswith("VERIFICATION_ANSWER:"):
                cleaned = cleaned[len("VERIFICATION_ANSWER:"):].strip()
            state["VERIFICATION_ANSWER"] = cleaned
            prompt += cleaned + "\n"
            logger.info(f"VERIFICATION ANSWER: {state.get('VERIFICATION_ANSWER', 'N/A')}")
            
            # Deep vision evaluation at done gate
            if vision_criteria and last_eval_target:
                logger.info(f"Done gate: Running deep vision eval on {last_eval_target}")
                compressed_ctx = _compress_trace(prompt)
                shallow_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)
                deep_text = _vision_eval_deep(vision_criteria, last_eval_target, shallow_text or "", executor, compressed_context=compressed_ctx)
                if deep_text:
                    prompt += format_user(f"DEEP VISION EVALUATION (final quality gate, read-only):\n{deep_text}\nUse this only as QA signal. Do NOT run additional tool steps at done gate.\n")
                    prompt += format_assistant("Noted.\n")
                    logger.info(f"Done gate: Deep eval result injected")
                    deep_status = _parse_deep_eval_status(deep_text)
                    if deep_status == "needs_revision":
                        logger.info("Done gate: Deep eval returned NEEDS_REVISION; returning caveated frozen artifact (no reopen, no regeneration)")
                        draft_text = _resolve_eval_target_text(last_eval_target, executor)
                        caveat = (
                            "Quality gate warning: deep evaluation found issues (STATUS: NEEDS_REVISION). "
                            "Delivering best available draft from this cycle (not revised)."
                        )
                        state["VERIFICATION_ANSWER"] = "PARTIAL"
                        state[f"done_{step}"] = "NO"
                        state["final_answer"] = f"{caveat}\n\n{draft_text}" if draft_text else caveat
                        break
                elif shallow_text and "fail" in shallow_text.lower():
                    prompt += format_user(f"VISION EVALUATION (final quality gate, shallow, read-only):\n{shallow_text}\nFAILs are advisory. Do NOT run additional tool steps at done gate.\n")
                    prompt += format_assistant("Noted.\n")
                    logger.info("Done gate: Shallow eval has FAILs (read-only advisory); proceeding to finalization")
            
            # Generate final answer
            next_task_raw = state.get(f"next_task_{step}", "").strip()
            if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                final_prompt = next_task_raw
            else:
                final_prompt = "Summarize the results with focus on the original goal"
            
            prompt += format_user(f"FINAL TASK: {final_prompt}\nProvide a concise final answer.")
            final_answer = vllm_gen("final_answer", prompt, state, max_tokens=256, temperature=GEN_TEMPERATURE, stop=["\n\n", "STAGE"], executor=executor)
            prompt += final_answer
            logger.info(f"FINAL_ANSWER: {state.get('final_answer', 'N/A')}")
            break

        stall_reason = _stall_guard_check(
            stall_guard_state,
            done_raw=done_raw,
            next_task_raw=next_task_raw,
            code_text=code_text
        )
        if stall_reason:
            logger.info(stall_reason)
            draft_text = _resolve_eval_target_text(last_eval_target, executor) if last_eval_target else ""
            caveat = "Loop guard: repeated planning pattern detected. Delivering best available draft from this cycle."
            state["VERIFICATION_ANSWER"] = "PARTIAL"
            state[f"done_{step}"] = "NO"
            state["final_answer"] = f"{caveat}\n\n{draft_text}" if draft_text else caveat
            break
        
        # Update current task for next iteration
        if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
            current_task = next_task_raw
            logger.info(f"Step {step}: Next task: {current_task}")
        else:
            logger.warning(f"Step {step}: No NEXT_TASK provided, stopping")
    
    # Interrupt checkpoint: exit from tool_planner_infospace
    if executor and _interrupt_requested(executor):
        _clear_interrupt(executor)
        state["final_answer"] = "Interrupted by user."
    
    # Step-limit fallback: one last synthesis pass if we have a usable candidate.
    if "final_answer" not in state:
        draft_text = _resolve_eval_target_text(last_eval_target, executor) if last_eval_target else ""
        if isinstance(draft_text, str) and draft_text.strip():
            logger.info("Step limit reached: running one last-chance synthesize pass")
            synth_r = executor.execute_action({
                "type": "synthesize",
                "target": last_eval_target,
                "focus": "Produce the best final artifact for the goal using available draft content.",
                "format": "comprehensive",
                "out": "$last_chance_final"
            })
            final_text = draft_text
            if isinstance(synth_r, dict) and synth_r.get("status") == "success":
                executor.execute_action({"type": "bind", "target": "$last_chance_final", "out": "$goal_artifact"})
                synthesized = _resolve_eval_target_text("$last_chance_final", executor)
                if isinstance(synthesized, str) and synthesized.strip():
                    final_text = synthesized
            caveat = "Step limit reached. Delivering best available draft after one last-chance synthesis pass."
            state["VERIFICATION_ANSWER"] = "PARTIAL"
            if max_steps > 0:
                state[f"done_{max_steps - 1}"] = "NO"
            state["final_answer"] = f"{caveat}\n\n{final_text}"
        else:
            caveat = "Step limit reached before a usable final candidate artifact was produced."
            state["VERIFICATION_ANSWER"] = "PARTIAL"
            if max_steps > 0:
                state[f"done_{max_steps - 1}"] = "NO"
            state["final_answer"] = caveat
    
    # Write full conversation state to trace file
    if trace_file:
        logger.info(f"Writing full conversation state to trace file")
        trace_file.write(f"\n{'='*80}\n")
        trace_file.write(f"Planning session: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        trace_file.write(f"Goal: {goal}\n")
        trace_file.write(f"total length: {len(prompt)}\n")
        tokenizer = getattr(executor, "tokenizer", None)
        if tokenizer:
            trace_file.write(f"token count: {tokenize_len(tokenizer, prompt)}\n")
        trace_file.write(f"{'='*80}\n")
        trace_file.write(prompt + "\n")
        trace_file.write(f"\n{'='*80}\n\n")
        trace_file.flush()
    
    return state



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


def _normalize_loop_text(text: str) -> str:
    """Normalize text for stable loop-signature comparisons."""
    if not text:
        return ""
    normalized = re.sub(r"[^a-z0-9\s]", " ", str(text).lower())
    normalized = re.sub(r"\s+", " ", normalized).strip()
    return normalized


def _extract_action_types_from_code(code_text: str) -> List[str]:
    """Extract action types from code block for simple repetition detection."""
    if not code_text:
        return []
    return re.findall(r'["\']type["\']\s*:\s*["\']([^"\']+)["\']', code_text)


def _stall_guard_check(
    guard_state: Dict[str, Any],
    done_raw: str,
    next_task_raw: str,
    code_text: str
) -> Optional[str]:
    """
    Detect repeated plan loops using deterministic structural signatures.
    Returns a reason string when the loop should be stopped, else None.
    """
    if str(done_raw).strip().upper().startswith("YES"):
        guard_state["prev_signature"] = None
        guard_state["repeat_count"] = 0
        return None

    next_task_sig = _normalize_loop_text(next_task_raw)[:220]
    action_sig = tuple(_extract_action_types_from_code(code_text)[:8])
    signature = (next_task_sig, action_sig)

    prev_signature = guard_state.get("prev_signature")
    repeat_count = int(guard_state.get("repeat_count", 0))
    if signature == prev_signature and (next_task_sig or action_sig):
        repeat_count += 1
    else:
        repeat_count = 0

    guard_state["prev_signature"] = signature
    guard_state["repeat_count"] = repeat_count

    if repeat_count >= 2:
        return "Loop guard: repeated NEXT_TASK/code signature without convergence."
    return None



class IncrementalPlanner:
    """
    Incremental planner using SGLang for iterative goal achievement.
    """
    
    def __init__(self, executor: InfospaceExecutor, available_tools: Dict[str, Dict], 
                logger_instance=None, sgl_model_path: str = None, vllm_model_path: str = None,
                vllm_url: str = "http://localhost:5000/v1/chat/completions", vllm_model: str = None,
                openrouter_model_path: str = None, anthropic_model_path: str = None):
        """
        Initialize incremental planner.
        
        Args:
            executor: InfospaceExecutor instance
            available_tools: Dict of tool_name -> metadata
            primitives_reference: Primitives reference text
            logger_instance: Optional logger
            sgl_model_path: Path to local model for SGLang (optional if vLLM/OpenRouter/Anthropic is used)
            vllm_model_path: Path to model for vLLM (optional if SGLang/OpenRouter/Anthropic is used)
            vllm_url: vLLM API endpoint (default: http://localhost:5000/v1/chat/completions)
            vllm_model: vLLM model name (resolved from vLLM server if not provided)
            openrouter_model_path: Model name for OpenRouter (optional if SGLang/vLLM/Anthropic is used)
            anthropic_model_path: Model name for Anthropic API (optional if SGLang/vLLM/OpenRouter is used)
        """
        # Require either SGLang, vLLM, OpenRouter, or Anthropic
        if not HAS_SGLANG and not vllm_model_path and not openrouter_model_path and not anthropic_model_path:
            raise ImportError("Neither SGLang, vLLM, OpenRouter, nor Anthropic available - at least one backend required")
        
        self.executor: InfospaceExecutor = executor
        self.available_tools = available_tools
        self.logger = logger_instance or logger
        
        # Store vLLM config
        self.vllm_model_path = vllm_model_path
        self.vllm_url = vllm_url
        self.vllm_model = vllm_model
        
        # Store OpenRouter config
        self.openrouter_model_path = openrouter_model_path
        
        # Store Anthropic config
        self.anthropic_model_path = anthropic_model_path
        
        # SGLang runtime is now initialized in executive_node (optional)
        # Verify availability if SGLang is expected
        if sgl_model_path and not executor.runtime:
            self.logger.warning("SGLang runtime not available in executor - will use vLLM/OpenRouter if configured")
        elif not vllm_model_path and not openrouter_model_path and not anthropic_model_path and not executor.runtime:
            self.logger.warning("Neither SGLang runtime nor vLLM/OpenRouter config available - incremental planner may have reduced functionality")
        
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

    def _planner_history_dir(self) -> Path:
        """
        Directory for authoritative planner history logs (JSONL).
        Prefer world resource base_dir if available; fallback to src/logs.
        """
        base_dir = None
        if hasattr(self.executor, "resource_manager") and self.executor.resource_manager:
            base_dir = getattr(self.executor.resource_manager, "base_dir", None)
        if isinstance(base_dir, Path):
            d = base_dir / "planner_history"
        else:
            d = Path(os.path.dirname(__file__)) / "logs" / "planner_history"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def _next_planner_seq(self) -> int:
        """
        Monotonic sequence number for planner history records.
        Stored in a small file in planner_history_dir.
        """
        seq_path = self._planner_history_dir() / "seq.txt"
        try:
            if seq_path.exists():
                raw = seq_path.read_text(encoding="utf-8").strip()
                last = int(raw) if raw else 0
            else:
                last = 0
        except Exception:
            last = 0
        nxt = last + 1
        try:
            tmp = seq_path.with_suffix(".tmp")
            tmp.write_text(str(nxt), encoding="utf-8")
            tmp.replace(seq_path)
        except Exception as e:
            self.logger.warning(f"Failed to persist planner seq to {seq_path}: {e}")
        return nxt

    def _append_jsonl(self, path: Path, record: Dict[str, Any]) -> None:
        try:
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception as e:
            self.logger.warning(f"Failed to append planner history to {path}: {e}")

    def build_context(
        self,
        goal: Optional[str] = None,
        max_core_tools: int = 2,
        max_world_tools: int = 3,
        max_chars_per_tool: int = 900,
    ) -> str:
        """
        Build a concise, two-section situation/context string for planning.

        Design goals:
        - KISS: robust, deterministic, and bounded
        - No hard-coded tool names
        - Prefer tools explicitly marked situational (SKILL.md frontmatter: situational: true)
        - Split into two sections: infospace vs world tools (if any)

        Notes:
        - Calls situational tools with no arguments. Tools that require args are skipped.
        - Ordering is heuristic-but-generic: status/observe/inventory-like names first.
        """

        executor = self.executor
        if not executor:
            return ""

        # Allow future executor-provided index/hint
        situational_names = None
        if hasattr(executor, "situational_tools"):
            try:
                v = getattr(executor, "situational_tools")
                if isinstance(v, list) and all(isinstance(x, str) for x in v):
                    situational_names = v
            except Exception:
                situational_names = None

        if situational_names is None:
            situational_names = []
            for tool_name, meta in (self.available_tools or {}).items():
                if isinstance(meta, dict) and meta.get("situational") is True:
                    situational_names.append(tool_name)

        # Partition: core vs world, based on path (same logic as build_tool_catalog)
        def _infer_source(tool_meta: Dict[str, Any]) -> str:
            path = tool_meta.get('path') or tool_meta.get('python_file') or ''
            if not isinstance(path, str) or not path:
                return 'core'
            p = path.replace('\\', '/')
            marker = '/src/world-tools/'
            if marker in p:
                after = p.split(marker, 1)[1]
                world_name = after.split('/', 1)[0].strip()
                return world_name or 'core'
            return 'core'

        def _rank_name(n: str) -> int:
            ln = (n or "").lower()
            if "status" in ln:
                return 0
            if "observe" in ln:
                return 1
            if "inventory" in ln:
                return 2
            return 10

        core = []
        world = []
        for name in situational_names:
            meta = (self.available_tools or {}).get(name, {})
            source = _infer_source(meta) if isinstance(meta, dict) else "core"
            if source == "core":
                core.append(name)
            else:
                world.append(name)

        core = sorted(core, key=lambda n: (_rank_name(n), n))
        world = sorted(world, key=lambda n: (_rank_name(n), n))

        def _run_tools(tool_names: List[str], max_tools: int) -> List[str]:
            out_lines = []
            used = 0
            for tool_name in tool_names:
                if used >= max_tools:
                    break
                try:
                    res = executor.execute_action({"type": tool_name})
                except Exception:
                    continue
                if not isinstance(res, dict) or res.get("status") != "success":
                    continue
                val = res.get("value")
                if not isinstance(val, str) or not val.strip():
                    continue
                val = val.strip()
                if len(val) > max_chars_per_tool:
                    val = val[:max_chars_per_tool] + "..."
                out_lines.append(f"- {tool_name}: {val}")
                used += 1
            return out_lines

        lines = []
        if goal:
            lines.append(f"Goal: {goal}")

        # Infospace section: only if we have any core situational tools enabled
        core_lines = _run_tools(core, max_core_tools) if max_core_tools > 0 else []
        lines.append("## Infospace")
        if core_lines:
            lines.extend(core_lines)
        else:
            lines.append("(none)")

        # World section: include if any world situational tools exist
        world_lines = _run_tools(world, max_world_tools) if max_world_tools > 0 else []
        if world or world_lines:
            lines.append("## World")
            if world_lines:
                lines.extend(world_lines)
            else:
                lines.append("(none)")

        return "\n".join(lines).strip()
    
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
        Generate plan incrementally using SGLang or vLLM.
        
        Args:
            goal: Goal text
            context: Optional dict with character_context, recent_context
            max_steps: Maximum planning steps
            
        Returns:
            Plan dict with 'plan' key containing actions
        """
        if not HAS_SGLANG and not self.vllm_model_path and not self.openrouter_model_path and not self.anthropic_model_path:
            return {'error': 'Neither SGLang, vLLM, OpenRouter, nor Anthropic available'}
        
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
           
            # Run planner (SGLang or vLLM)
            world_model = initial_world_model
            if self.vllm_model_path and self.vllm_model:
                # Use vLLM planner
                state = tool_planner_infospace_vllm(
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
            elif HAS_SGLANG and self.executor.runtime:
                # Use SGLang planner
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
            else:
                return {'error': 'No planner backend available (SGLang, vLLM, Anthropic, or OpenRouter required)'}

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
            verification_answer = str(state.get('VERIFICATION_ANSWER', '')).strip().upper() if isinstance(state, dict) else ""
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
            if final_answer == "Interrupted by user.":
                quality_status = "interrupted"
            elif success:
                quality_status = "passed"
            elif verification_answer == "PARTIAL":
                quality_status = "needs_revision"
            else:
                quality_status = "failed"
            
            return {
                'plan': None,
                'response': final_answer,
                'task_state': None,
                'success': success,
                'quality_status': quality_status,
                'verification_answer': verification_answer,
                'error_count': error_count,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Incremental planning failed: {e}")
            traceback.print_exc()
            return {'error': str(e)}


    def generate_plan(self, template, goal: str, context: Dict = None, max_steps: int = 16) -> Dict:
        """
        Generate plan incrementally using SGLang or vLLM.
        
        Args:
            goal: Goal text
            context: Optional dict with character_context, recent_context
            max_steps: Maximum planning steps
            
        Returns:
            Plan dict with 'plan' key containing actions
        """
        if not HAS_SGLANG and not self.vllm_model_path and not self.openrouter_model_path and not self.anthropic_model_path:
            return {'error': 'Neither SGLang, vLLM, OpenRouter, nor Anthropic available'}
        
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
            vision_criteria = self._generate_vision(goal)
            # Extract context components
            character_context = ""
            recent_context = ""
            if context:
                character_context = context.get('character_context', '')
                recent_context = context.get('recent_context', '')
            recent_context += self.build_context(goal=goal)
                
            # Find similar plans using plan guidance
            #similar_plans = self.plan_guidance.find_similar_plans(goal)
            similar_plans = None
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
                logger.error(f"⚠️  Attempting to send {total_input_size:,} chars to planner (limit: 100,000)")
                logger.error("Stack traceback:")
                for line in traceback.format_stack():
                    logger.error(line.rstrip())
            
            # Run planner (SGLang, vLLM, or OpenRouter)
            world_model = initial_world_model
            if (self.vllm_model_path and self.vllm_model) or (self.openrouter_model_path and self.executor.openrouter_model) or (self.anthropic_model_path and self.executor.anthropic_model):
                # Use vLLM/OpenRouter planner (both use same function since executor.llm_generate routes correctly)
                state = tool_planner_infospace_vllm(
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
                    similar_plan=similar_plans[0] if similar_plans else None,
                    vllm_url=self.vllm_url,
                    model=self.vllm_model,
                    vision_criteria=vision_criteria
                )
            elif HAS_SGLANG and self.executor.runtime:
                # Use SGLang planner
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
                    similar_plan=similar_plans[0] if similar_plans else None,
                    vision_criteria=vision_criteria
                )
            else:
                return {'error': 'No planner backend available (SGLang, vLLM, Anthropic, or OpenRouter required)'}

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
            compressed_trace = _compress_trace(trace_str)
            reflection_frame = self._reflect(goal, world_model, max_steps, compressed_trace)
            
            # Extract inner ReflectionFrame content (LLM returns wrapped in 'ReflectionFrame' key)
            reflection_content = reflection_frame.get('ReflectionFrame', reflection_frame)
            task_state = reflection_content.get('task_state', {})
            self._last_task_state = task_state
            
            # Update world_model from reflection_frame
            if hasattr(self.executor, 'world_model') and self.executor.world_model:
                self.executor.world_model.update(reflection_content)
                self.executor.world_model.save()
                # Get updated world_model snapshot
                world_model = self.executor.world_model.get()
            else:
                logger.warning("WorldModel not available, skipping update")

            # Append authoritative planner history records (JSONL)
            seq = self._next_planner_seq()
            now_iso = datetime.datetime.utcnow().isoformat()
            agent_name = getattr(self.executor, "agent_name", "unknown")
            world_name = getattr(self.executor, "world_name", "unknown")
            hist_dir = self._planner_history_dir()
            self._append_jsonl(
                hist_dir / "compressed_trace.jsonl",
                {"seq": seq, "ts": now_iso, "agent": agent_name, "world": world_name, "goal": goal, "compressed_trace": compressed_trace},
            )
            self._append_jsonl(
                hist_dir / "reflection_frame.jsonl",
                {"seq": seq, "ts": now_iso, "agent": agent_name, "world": world_name, "goal": goal, "reflection_frame": reflection_content},
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
            verification_answer = str(state.get('VERIFICATION_ANSWER', '')).strip().upper() if isinstance(state, dict) else ""
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
            if final_answer == "Interrupted by user.":
                quality_status = "interrupted"
            elif success:
                quality_status = "passed"
            elif verification_answer == "PARTIAL":
                quality_status = "needs_revision"
            else:
                quality_status = "failed"
            
            return {
                'plan': plan_actions,
                'response': final_answer,
                'task_state': task_state,
                'success': success,
                'quality_status': quality_status,
                'verification_answer': verification_answer,
                'error_count': error_count,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Incremental planning failed: {e}")
            traceback.print_exc()
            return {"success": False, 'error': str(e)}


    def _generate_vision(self, goal_text: str) -> str:
        """Generate evaluable quality criteria from the goal (vision for the target artifact)."""
        VISION_PROMPT = f"""Given the following goal, generate 2-4 concrete, evaluable quality criteria that define what a good output looks like.

GOAL:
{goal_text}

Instructions:
- Each criterion should be a testable predicate (answerable as true/false against the output).
- Focus on quality dimensions: depth, specificity, coverage, structure.
- Be deliberately open on content (which topics, which specifics) — those emerge from the material.
- Numeric criteria (e.g., min_sources: 8) will be checked in code.
- Qualitative criteria (e.g., "contains specific method names, not just category labels") will be checked via LLM assessment.
- If the goal is simple or direct (e.g., a factual lookup), return "No vision criteria needed."

Format (one criterion per line):
criterion_name: "testable predicate or numeric threshold"
END_VISION"""

        result = self.executor.llm_generate(VISION_PROMPT, max_tokens=192, temperature=GEN_TEMPERATURE, stops=["\nEND_VISION", "END_VISION"])
        if not result.success or not result.text:
            logger.warning(f"_generate_vision failed: {result.error if hasattr(result, 'error') else 'unknown'}")
            return ""
        vision_text = result.text.strip()
        if "no vision criteria needed" in vision_text.lower():
            logger.info("Vision generator: no criteria needed for this goal")
            return ""
        logger.info(f"Vision generator: {vision_text}")
        return vision_text

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
     (e.g., retrieval, extraction, transformation, comparison, generation).
   - Match these operation types to the available tools.  
     - Do not assume tools that are absent.  
     - If multiple tools could serve a role, state how to choose between them.
   - Describe a minimal, ordered sequence of conceptual steps  
     that a tool-using planner would follow to solve the goal.  
     Describe *what must be achieved*, not how to call tools.
   - Include fallback logic **only if the goal plausibly needs it**  
     (e.g., multi-source lookup, ambiguous values, missing information).
   - Do not include JSON, tool calls, URLs, or domain-specific knowledge.  
     The output must be a brief strategic outline, not an answer to the goal.

Format:
ABSTRACT_PLAN:
STEP 1: <what to achieve>
STEP 2: <what to achieve>
STEP 3: <what to achieve>
END_PLAN
"""

        abstract_plan = self.executor.llm_generate(ABSTRACT_PLAN_PROMPT, max_tokens=256, temperature=GEN_TEMPERATURE, stops=["\nEND_PLAN:", "END_PLAN:"])
        if not abstract_plan.success or not abstract_plan.text:
            error_msg = abstract_plan.error or "Unknown error"
            logger.error(f"_preplan failed: {error_msg}")
            return "No preplan available"
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
E) Adds CONTRADICTION evidence only via world_model_updates with polarity="contradict"

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
- reflect stable properties of the world (mechanics, structures, affordances)

Each fact must be atomic, general, and reusable.
If uncertain, do NOT promote.

IMPORTANT: world_model_updates are EVIDENCE EVENTS, not belief labels.
- polarity="support": the trace provides positive evidence the general fact holds.
- polarity="contradict": the trace provides evidence the general fact is false or has a counterexample.
- Do NOT output confidence/stability labels. Those are computed downstream from accumulated evidence.
- WORLD FACTS ONLY: world_model_updates MUST NOT describe tool behavior and MUST NOT mention tool names.
  - Bad (tool behavior): "path-explore can successfully navigate to frontier cells"
  - Good (world fact): "Frontier cells may be obstructed even if adjacent cells look reachable"
  - If a statement involves a specific tool (e.g., "mc-dig", "path-explore", "nav-*"), it belongs in tool_insights, not world_model_updates.

It is acceptable — and often correct — to propose NO world updates.

STEP 4 — TOOL INSIGHTS (META-COGNITION)
Record any discovered tool properties that would constrain future planning:
- guarantees
- limits
- misleading success signals
- required preconditions

Each insight must be stated as a contract or constraint.

STEP 5 — CONTRADICTION VS CONTEXT FORGETTING
- If the trace provides a counterexample to a GENERAL belief, add a world_model_updates entry with polarity="contradict".
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
        task_state = getattr(self, "_last_task_state", {}) or {}
        world_model = json.dumps(world_model, indent=2) if isinstance(world_model, dict) else str(world_model)
        task_state = json.dumps(task_state, indent=2) if isinstance(task_state, dict) else str(task_state)
        reflection_prompt = reflection_prompt.replace("{task_state}", task_state)
        reflection_prompt = reflection_prompt.replace("{world_model}", world_model)
        reflection_prompt = reflection_prompt.replace("{goal_text}", goal_text)
        reflection_prompt = reflection_prompt.replace("{steps}", str(steps))
        reflection_prompt = reflection_prompt.replace("{REFLECTION_FRAME_SCHEMA}", json.dumps(REFLECTION_FRAME_SCHEMA, indent=2))
        reflection_prompt = reflection_prompt.replace("{trace}", trace)
        reflection = self.executor.llm_generate(reflection_prompt, max_tokens=4096, is_json=True, temperature=0.0)

        return reflection.text