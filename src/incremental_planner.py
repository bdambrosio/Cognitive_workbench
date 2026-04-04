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
import difflib
import textwrap
from pathlib import Path
from typing import Dict, List, Any, Optional
from infospace_executor import InfospaceExecutor
from plan_guidance import PlanGuidance
# ToolModel disabled — tool contracts in WorldModel serve this role
from world_model import WORLD_MODEL_SCHEMA, empty_world_model
# Per-stage temperature settings for gen() calls
GEN_TEMPERATURE = 0.5          # Default / fallback
CODE_TEMPERATURE = 0.2         # Stage 2 code generation — low for correctness
SELECT_TEMPERATURE = 0.1       # Tool selection, DONE/EVAL_TARGET classification — low for reliability
REFLECT_TEMPERATURE = GEN_TEMPERATURE  # Reasoning, thoughts, next-task planning

# Regex to strip <think>...</think> blocks that leak into response content.
#
# OpenRouter routes requests across multiple providers (Google, DeepSeek,
# Together, Fireworks, etc.) with inconsistent reasoning-token handling:
#   - DeepSeek/Qwen3 models natively emit <think>...</think> inline in content.
#   - Gemini models (2.5 Flash, 3 Flash, etc.) are supposed to return reasoning
#     in a structured `reasoning_details` field, but <think> blocks can leak
#     into the content field depending on provider/routing.
#   - Other providers may inject think tags for reasoning-enabled models.
#
# Think blocks contain internal chain-of-thought reasoning, NOT answer content.
# Always discard them entirely. If no content remains after stripping, the
# response is malformed — do NOT extract think-block internals as a fallback,
# since that would promote raw reasoning tokens (which may be speculative,
# contradictory, or incomplete) to answer status.
_THINK_TAG_RE = re.compile(r'<think>.*?</think>\s*', re.DOTALL)

def _strip_think_tags(text: str) -> str:
    """Strip <think>...</think> blocks from model output, returning the actual answer."""
    if not isinstance(text, str):
        return text
    result = _THINK_TAG_RE.sub('', text).strip()
    if not result and '<think>' in text:
        logger.warning(
            "_strip_think_tags: entire response was inside <think> block — "
            "no answer content (provider returned reasoning only, no response)"
        )
    return result

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

# ── Self-model data helpers (thin wrappers, fail silently) ────────────────

def _safe_get_scheduler_status(executor: InfospaceExecutor) -> dict:
    try:
        en = getattr(executor, 'executive_node', None)
        if en and hasattr(en, 'goal_scheduler'):
            return en.goal_scheduler.get_status()
    except Exception:
        pass
    return {}

def _safe_get_all_tasks(executor: InfospaceExecutor) -> list:
    """Read all task WIP notes (establishing + operational) as parsed dicts."""
    try:
        rm = getattr(executor, 'resource_manager', None)
        if not rm:
            return []
        tasks = []
        for name, note_id in list(rm.named_notes.items()):
            if not name.startswith('_task_wip_'):
                continue
            res = rm.get_resource(note_id)
            content = getattr(res, 'content', '') if res else ''
            if content:
                import json as _json
                tasks.append(_json.loads(content))
        return tasks
    except Exception:
        return []

def _safe_get_scheduled_goals(executor: InfospaceExecutor) -> list:
    try:
        en = getattr(executor, 'executive_node', None)
        if en and hasattr(en, '_all_scheduled_goals'):
            return list(en._all_scheduled_goals())
    except Exception:
        pass
    return []

def _safe_get_sensor_configs(executor: InfospaceExecutor) -> list:
    try:
        en = getattr(executor, 'executive_node', None)
        return getattr(en, 'sensor_configs', []) if en else []
    except Exception:
        return []

def _safe_get_execution_mode(executor: InfospaceExecutor) -> str:
    try:
        en = getattr(executor, 'executive_node', None)
        if en:
            return getattr(en, 'execution_mode', 'step')
    except Exception:
        pass
    return 'step'


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
    from sglang import function, system, user, assistant, gen, select
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
  "world_model_updates": [
    {
      "fact": "string  -- general, reusable world fact (NOT tool behavior, NOT agent state)",
      "polarity": "support | contradict",
      "source": "observation | tool_guarantee"
    }
  ],
  "tool_insights": [
    {
      "tool": "string  -- tool name",
      "insight": "string  -- contract, limit, precondition, or failure mode",
      "status": "reliable | unreliable | constrained"
    }
  ]
}

def _build_bindings_inventory(plan_local_bindings: set, executor) -> str:
    """Build a compact one-line-per-binding inventory of plan-local resources.

    Only includes bindings created during the current plan execution.
    Skips internal/temp bindings (prefixed with '_').
    Returns empty string if no bindings or on error.
    """
    if not plan_local_bindings or not executor:
        return ""
    try:
        bindings_flat = executor.plan_bindings_flat
        rm = executor.resource_manager
        if not rm:
            return ""

        parts = []
        for name in sorted(plan_local_bindings):
            if name.startswith('_') or name.startswith('map_temp'):
                continue
            rid = bindings_flat.get(name)
            if not rid or not isinstance(rid, str):
                continue
            res = rm.get_resource(rid)
            if not res:
                parts.append(f"${name}: {rid} (unresolved)")
                continue
            rtype = res.get('type')
            type_name = getattr(rtype, 'name', str(rtype)) if rtype else '?'
            props = res.get('properties', {})
            content = props.get('content')
            if type_name == 'Collection' and isinstance(content, list):
                parts.append(f"${name}: Collection ({len(content)} items)")
            elif content is not None:
                chars = len(str(content))
                parts.append(f"${name}: Note ({chars} chars)")
            else:
                parts.append(f"${name}: {type_name}")

        if not parts:
            return ""
        return "BINDINGS: " + " | ".join(parts) + "\n"
    except Exception as e:
        logger.debug(f"_build_bindings_inventory failed: {e}")
        return ""


INCREMENTAL_PLAN_SPECIFICATIONS = """
# INFOSPACE TYPE SYSTEM & RULES

Types:
- Note: Single value/document (content is always a string). Can be named for stable referencing via load.
- Collection: List of Note/Collection IDs. Can be named for stable referencing via load.
- $variable: A session-local binding referencing a Note or Collection. Always starts with "$".
  Use "$var" for bindings, bare IDs (Note_123) or names ("my-note") for direct references.

Note Content:
- Always a string. Metadata via get-metadata/set-metadata.
- Structured ops (project/pluck/filter-structured/sort/join) require valid JSON text content.
- For plain text, use extract (LLM-based).

Primitive Type Compatibility (world/skill tools have their own contracts):
 - split: Note → Collection (array/lines → items)
 - flatten: Collection → Note (merge items)
 - extract: Note; LLM-guided extraction/transformation
 - synthesize: Collection; cross-document integration/comparison/reporting
 - map: Collection; apply op to each item
 - project, pluck, sort, filter-structured: Collection; SQL-like ops (JSON Notes only)
 - join: Collection; merge 2 Collections (SQL JOIN, JSON Notes only)
 - add, remove, size: Collection; mutation
 - union, intersection, difference: Collection; set ops
 - load: Note, Collection; bind persistent resource, slice for subset
 - persist: Note, Collection; save to filesystem
 - discover-notes, discover-collections: global search (no target), returns Collection
 - search-within-collection: search indexed Collection, returns Collection

Key Semantics:
- load: ONLY for (a) binding named persistent notes, or (b) slicing Collections. To read content, use get_text/get_json/get_items.
  slice=":" for full content; default "0:4096" (Notes) or "0:5" (Collections).
- persist: Mark resource as persistent (saved to filesystem).
- discover-*: No target needed. search-within-collection: requires indexed target Collection.
- Slice a Collection for first N items: tool("load", target="$coll", slice="0:5", out="$top")

Efficiency Heuristics:
- extract on Notes, synthesize on Collections, map(extract) for per-item then synthesize to integrate
- project/pluck/filter-structured: JSON Notes only. For plain text use extract.
- Create Collections only for 2+ items

#TOOLS
Tools include primitives (above) and external tools loaded at runtime. See catalog below.
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
            "schema_hint": {"value": "[$var, ...] or []", "name": "optional, stable name", "out": "$var"}
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
            "schema_hint": {"value": "any (string, number, array, object, $var)", "name": "optional, stable name", "out": "$var"}
        },
        "get-metadata": {
            "description": "Retrieve metadata attached to a Note or Collection.",
            "full_description": "Returns the metadata text associated with the target Note or Collection (stored transparently by the system). Returns an empty string if no metadata exists. If out is provided, the variable is bound to the metadata Note ID.",
            "parameters": {
                "target": "required: $variable, Note/Collection ID, or name",
                "out": "optional: $variable — bound to the metadata Note ID if metadata exists"
            },
            "examples": [
                '{"type":"get-metadata","target":"$search_result","out":"$meta"}',
                '{"type":"get-metadata","target":"Note_123","out":"$note_meta"}'
            ],
            "schema_hint": {"target": "$var, ID, or name", "out": "$var (optional)"}
        },
        "set-metadata": {
            "description": "Attach or update metadata on a Note or Collection.",
            "full_description": "Associates a metadata text string with the target Note or Collection. Idempotent: updates existing metadata if already present. If out is provided, the variable is bound to the metadata Note ID.",
            "parameters": {
                "target": "required: $variable, Note/Collection ID, or name",
                "value": "required: metadata text (string, may be JSON)",
                "out": "optional: $variable — bound to the metadata Note ID"
            },
            "examples": [
                '{"type":"set-metadata","target":"$report","value":"{\\"source\\": \\"weather.gov\\", \\"retrieved\\": \\"2026-02-21\\"}"}',
                '{"type":"set-metadata","target":"$report","value":"$meta_text","out":"$meta_note"}'
            ],
            "schema_hint": {"target": "$var, ID, or name", "value": "string or $var", "out": "$var (optional)"}
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
            "schema_hint": {"value": "string or $var"}
        },
        "bind": {
            "description": "Bind a variable to an existing resource without creating or changing content.",
            "full_description": "Alias an existing Note/Collection/Relation to a new variable via out. bind does not mutate resources.",
            "parameters": {
                "target": "required: $variable, resource ID, or named resource",
                "out": "required: destination $variable"
            },
            "examples": [
                '{"type":"bind","target":"$draft","out":"$final_report"}',
                '{"type":"bind","target":"Note_42","out":"$summary"}'
            ],
            "schema_hint": {"target": "$var, ID, or name", "out": "$var"}
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
        "get-metadata": {
            "description": PRIMITIVE_DOCS["get-metadata"]["description"],
            "full_description": PRIMITIVE_DOCS["get-metadata"].get("full_description"),
            "examples": PRIMITIVE_DOCS["get-metadata"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["get-metadata"]["schema_hint"]
        },
        "set-metadata": {
            "description": PRIMITIVE_DOCS["set-metadata"]["description"],
            "full_description": PRIMITIVE_DOCS["set-metadata"].get("full_description"),
            "examples": PRIMITIVE_DOCS["set-metadata"].get("examples", []),
            "schema_hint": PRIMITIVE_DOCS["set-metadata"]["schema_hint"]
        },
        "load": {
            "description": "Load Note/Collection by ID or name. Use slice=':' for full content. Omit out to load into context only.",
            "full_description": "Load a Note or Collection into planner context. The 'slice' parameter controls how much content is returned, using Python-style syntax: '0:1000' (first 1000), '1500:2000' (chars 1500–2000), '-500:' (last 500), ':' (everything, no limit), '5' (single item). Standard Python semantics including negative indices. Rejects only when both start and stop are non-negative and stop<start. For Notes, units are characters. For Collections, units are items — the result is a new Collection bound to out. Omit 'out' to load content into context only (no variable binding). Chunked pattern for large content: load slice='0:500' → process → load slice='500:1000' → process.",
            "examples": [
                '{"type":"load","target":"$report","slice":":","out":"$full_report"}',
                '{"type":"load","target":"Note_42","slice":":"}',
                '{"type":"load","target":"$papers","slice":"0:3","out":"$top_papers"}',
                '{"type":"load","target":"$doc","slice":"0:500","out":"$chunk1"}'
            ],
            "schema_hint": {"target": "ID, name, or $var", "slice": "optional, e.g. ':', '0:5'", "out": "$var (optional)"}
        },
        "persist": {
            "description": "Mark Note/Collection as persistent (saved to filesystem). Optional name assigns a stable name for load-by-name (e.g., persist target=$report name=berkeley_weather_report).",
            "schema_hint": {"target": "$var", "name": "optional, stable name"}
        },
        "discover-notes": {
            "description": "Global discovery across all Notes using embedding-based retrieval. Returns Collection.",
            "schema_hint": {"query": "string (required)", "out": "$var", "limit": "int (optional, default 5)", "threshold": "float (optional, default 0.3)"}
        },
        "discover-collections": {
            "description": "Global discovery across all Collections using embedding-based retrieval. Returns Collection.",
            "schema_hint": {"query": "string (required)", "out": "$var", "limit": "int (optional, default 3)", "threshold": "float (optional, default 0.3)"}
        },
        "search-within-collection": {
            "description": "Search within an indexed Collection. Requires index first.",
            "schema_hint": {"target": "$var (indexed Collection)", "query": "string (required)", "out": "$var", "limit": "int (optional)", "threshold": "float (optional)"}
        },
        "index": {
            "description": "Build embedding index for Collection (required before search-within-collection).",
            "schema_hint": {"target": "$var"}
        },
        "map": {
            "description": "Apply operation to each item in Collection.",
            "schema_hint": {"target": "$var (Collection)", "operation": "string (tool name)", "out": "$var"}
        },
        "split": {
            "description": "Note → Collection: JSON array or text → one Note per element/sentence.",
            "schema_hint": {"target": "$var (Note)", "delimiter": "optional: sentence|paragraph|line|custom", "out": "$var (Collection)"}
        },
        "flatten": {
            "description": "Collection → single Note (merge all items).",
            "schema_hint": {"target": "$var (Collection)", "out": "$var"}
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
        "say": {
            "description": "Send message to user/agent. DEFAULT for reporting results. Does not wait for response.",
            "schema_hint": {"value": "string or $var", "target": "optional, default User"}
        },
        "ask": {
            "description": "Send question and BLOCK for response (5 min timeout). Use ONLY when you need the answer to continue.",
            "schema_hint": {"value": "string (required)", "out": "$var (required)", "target": "optional, default User"}
        },
        "add": {
            "description": "Add Note to Collection (mutates in place).",
            "schema_hint": {"target": "$var (Collection)", "value": "$var or literal", "out": "$var"}
        },
        "remove": {
            "description": "Remove Note from Collection (mutates).",
            "schema_hint": {"target": "$var (Collection)", "value": "$var or Note ID", "out": "$var"}
        },
        "union": {
            "description": "Union of two Collections (A ∪ B), deduplicated.",
            "schema_hint": {"target": "$var (A)", "value": "$var (B)", "out": "$var"}
        },
        "intersection": {
            "description": "Intersection of two Collections (A ∩ B).",
            "schema_hint": {"target": "$var (A)", "value": "$var (B)", "out": "$var"}
        },
        "difference": {
            "description": "Difference of two Collections (A - B).",
            "schema_hint": {"target": "$var (A)", "value": "$var (B)", "out": "$var"}
        },
        "size": {
            "description": "Get item count of a Collection.",
            "schema_hint": {"target": "$var (Collection)", "out": "$var"}
        },
        "project": {
            "description": "Extract fields from JSON Notes in Collection (SQL SELECT). Use extract for plain text.",
            "schema_hint": {"target": "$var (Collection)", "fields": "[\"field1\", \"field2\"]", "out": "$var"}
        },
        "pluck": {
            "description": "Extract single field from JSON Notes in Collection.",
            "schema_hint": {"target": "$var (Collection)", "field": "string", "out": "$var"}
        },
        # head: deprecated, use load with slice instead. Executor still accepts head for backward compat.
        "sort": {
            "description": "Sort Collection by field (SQL ORDER BY). JSON Notes only.",
            "schema_hint": {"target": "$var (Collection)", "by": "string (field)", "order": "optional, asc|desc", "out": "$var"}
        },
        "filter-structured": {
            "description": "Filter Collection by field conditions (SQL WHERE). JSON Notes only.",
            "schema_hint": {"target": "$var (Collection)", "where": "e.g. 'year >= 2020 AND score > 0.5'", "out": "$var"}
        },
        "join": {
            "description": "Inner join two Collections on a common field (SQL JOIN).",
            "schema_hint": {"target": "$var (left)", "value": "$var (right)", "on": "string (field name)", "out": "$var"}
        },
        "coerce": {
            "description": "Convert Note content type: to-string|to-int|to-float|to-bool|to-json|to-list.",
            "schema_hint": {"target": "$var (Note)", "coercion": "string (type)", "delimiter": "optional, for to-list", "out": "$var"}
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
    # Each tool gets: description, params, and an example tool() call.
    # Full docs with examples are loaded on-demand via REQUEST_TOOLS → load_skill_docs().
    def format_tool(name: str, meta: Dict) -> List[str]:
        tool_lines = []
        description = meta.get('description', 'No description')
        schema = meta.get('schema_hint')
        if schema and isinstance(schema, dict):
            # Line 1: description
            tool_lines.append(f"- {name}: {description}")
            # Line 2: compact params
            params = ", ".join(f"{k}: {v}" for k, v in schema.items())
            tool_lines.append(f"  params: {{{params}}}")
            # Line 3: example tool() call from schema keys
            example_args = []
            for k, v in schema.items():
                if 'optional' in str(v).lower():
                    continue
                hint = str(v).lower()
                if k == 'out':
                    example_args.append(f'out="$result"')
                elif k == 'target':
                    example_args.append(f'target="$var"')
                elif k == 'value':
                    example_args.append(f'value="..."')
                elif 'string' in hint or '$var' in hint:
                    example_args.append(f'{k}="..."')
                elif 'int' in hint:
                    example_args.append(f'{k}=5')
                elif 'float' in hint:
                    example_args.append(f'{k}=0.5')
                else:
                    example_args.append(f'{k}="..."')
            tool_lines.append(f'  e.g. tool("{name}", {", ".join(example_args)})')
        else:
            tool_lines.append(f"- {name}: {description}")
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
                lines.append("#COMMUNICATION (say=deliver message, ask=block for response)")
            lines.extend(format_tool(name, meta))
        lines.append("")  # Blank line before workflows
    
    lines.append("Preferred tool order: World tools first when they apply, then Infospace Core")
    # Add critical workflows to prevent common mistakes
    lines.append("# CRITICAL WORKFLOWS:")
    lines.append("- search-web → synthesize (with focus) — direct Collection analysis")
    lines.append("- search-web → map(extract) → synthesize — two-phase with per-item extraction")
    lines.append("- search-web → filter-structured → synthesize — filtered then analyzed")
    lines.append("- search-web → filter-semantic(predicate=...) → synthesize — semantic content filtering")
    lines.append("- search-web already returns substantial page content in 'text' field (keyword-filtered, up to ~8K chars per result) — use extract/synthesize directly")
    lines.append("- semantic-scholar → synthesize (with focus) — direct Collection analysis")
    lines.append("- semantic-scholar → map(extract) → synthesize — two-phase with per-item extraction")
    lines.append("- semantic-scholar → filter-structured → synthesize — filtered then analyzed")
    lines.append("- semantic-scholar → filter-semantic(predicate=...) → synthesize — semantic content filtering")
    lines.append("- semantic-scholar already returns full paper text (via GROBID) in 'text' field of each Note — do NOT project metadata.uri for fetching")
    lines.append("- For comparison: synthesize with format=\"comparison\" and other= (requires two inputs)")
    lines.append("- fetch-text is for SINGLE URLs only, NOT for Collections from search-web/semantic-scholar")
    lines.append("- Level 4 tools (search-web, semantic-scholar) return Collections with substantial content in the 'text' field of each Note — use extract/synthesize directly")
    
    return "\n".join(lines)


def _resolve_tool_name(tool_name: str, available_tools: Dict[str, Dict],
                       primitives_dir: Path,
                       executor_action_names: Optional[set] = None) -> str:
    """
    Resolve a tool name to its canonical form, handling hyphen/underscore variants.

    LLMs frequently swap hyphens and underscores in tool names (e.g. 'get_text'
    instead of 'get-text'). This tries the exact name first, then the alternate
    separator form.

    Returns the canonical name if found, otherwise the original name unchanged.
    """
    # Exact match — fast path
    if (tool_name in available_tools
            or (primitives_dir / tool_name).exists()
            or (executor_action_names and tool_name in executor_action_names)):
        return tool_name

    # Try swapping hyphens ↔ underscores
    if '-' in tool_name:
        alt = tool_name.replace('-', '_')
    elif '_' in tool_name:
        alt = tool_name.replace('_', '-')
    else:
        return tool_name  # no separator to swap

    if (alt in available_tools
            or (primitives_dir / alt).exists()
            or (executor_action_names and alt in executor_action_names)):
        logger.info(f"Resolved tool name '{tool_name}' → '{alt}' (separator normalization)")
        return alt

    return tool_name


def load_skill_docs(tool_names: List[str], available_tools: Dict[str, Dict],
                    executor_action_names: Optional[set] = None) -> str:
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
        # Normalize hyphen/underscore variants before lookup
        tool_name = _resolve_tool_name(tool_name, available_tools, primitives_dir, executor_action_names)
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
        # Always define primitive_dir for the "is_known" check below
        primitive_dir = primitives_dir / tool_name
        if tool_name not in available_tools:
            # Check if it's a primitive with Skill.md
            logger.debug(f"Stage 1.5: {tool_name} not in available_tools, checking primitives")
            if primitive_dir.exists():
                for variant in ['SKILL.md', 'Skill.md', 'skill.md']:
                    candidate = primitive_dir / variant
                    if candidate.exists():
                        skill_file = candidate
                        logger.debug(f"Stage 1.5: {tool_name} found primitive {variant} at {skill_file}")
                        break
        
        if not skill_file:
            # Distinguish "real tool, no docs" from "nonexistent tool"
            is_known = (tool_name in available_tools
                        or primitive_dir.exists()
                        or (executor_action_names and tool_name in executor_action_names))
            if not is_known:
                # Suggest closest matching tool names
                all_names = list(available_tools.keys())
                if executor_action_names:
                    all_names.extend(executor_action_names)
                # Also include primitive directory names
                if primitives_dir.exists():
                    all_names.extend(d.name for d in primitives_dir.iterdir() if d.is_dir())
                suggestions = difflib.get_close_matches(tool_name, all_names, n=3, cutoff=0.5)
                suggest_str = f" Did you mean: {', '.join(suggestions)}?" if suggestions else ""
                logger.warning(f"Stage 1.5: No tool named '{tool_name}' exists.{suggest_str}")
                lines.append(f"\n## {tool_name.upper()}")
                lines.append(f"⚠ ERROR: No tool named '{tool_name}' exists. Do NOT use this tool. "
                             f"Check the tool catalog above for the correct tool name."
                             f"{suggest_str}")
            else:
                logger.debug(f"Stage 1.5: No SKILL.md found for {tool_name} (tool exists, no docs)")
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
    # Normalize hyphen/underscore variants (LLMs frequently swap them)
    primitives_dir = Path(__file__).parent / 'primitives'
    tool_name = _resolve_tool_name(tool_name, available_tools, primitives_dir)

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


def _get_exemplar_hint_for_failure(tool_result: str, executor) -> str:
    """If tool_result indicates a failure, retrieve successful exemplars for the failed tool.

    Returns a hint string to inject into the Stage 3 prompt, or empty string if
    no failure or no exemplars available.
    """
    if "ERROR |" not in tool_result:
        return ""
    # Extract tool name from "ERROR | tool_name failed: reason"
    # or from "_code_block_ failed: reason" (code block wraps tool calls)
    import re
    m = re.search(r'ERROR \| (\S+) failed:', tool_result)
    if not m:
        return ""
    failed_tool = m.group(1)
    # _code_block_ failures: scan the error for a tool name
    if failed_tool == "_code_block_":
        # Look for tool name in the error text
        for candidate in re.findall(r'tool\("([^"]+)"', tool_result):
            failed_tool = candidate
            break
        else:
            return ""

    wm = getattr(executor, 'world_model', None)
    if not wm or not hasattr(wm, 'get_tool_exemplars'):
        return ""
    exemplars = wm.get_tool_exemplars(failed_tool)
    if not exemplars:
        return ""
    lines = [f"HINT: Previous successful calls to '{failed_tool}':"]
    for ex in exemplars[-3:]:  # Show at most 3 most recent
        lines.append(f"  {ex['call_line']}")
    return "\n".join(lines) + "\n\n"


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
    r'\b__import__\b', r'\b__builtins__\b', r'\b__subclasses__\b',
    r'\b__code__\b', r'\b__globals__\b', r'\b__loader__\b',
    r'\b__spec__\b', r'\b__bases__\b', r'\b__mro__\b',
    r'\bsys\b\.', r'\bsubprocess\b',
    r'\bglobals\b\s*\(', r'\blocals\b\s*\(',
    r'\bsetattr\b\s*\(', r'\bdelattr\b\s*\(',
    r':=',  # walrus operator — invalid inside exec() on some Python versions and causes subtle bugs
]


def validate_codegen_block(code: str) -> tuple:
    """
    Static validation of LLM-generated code blocks before exec.
    
    Checks:
    - Forbidden patterns (imports, exec, file I/O, class/def, etc.)
    - Must contain 1-16 execute_action_tracked calls
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

    # Check forbidden patterns
    for pattern in _CODEGEN_FORBIDDEN_PATTERNS:
        match = re.search(pattern, code)
        if match:
            return False, f"Forbidden pattern: {match.group()}"
    
    # Count tool calls: tool() shorthand + legacy execute_action_tracked (0-16; 0 allowed for read-only)
    call_count = (len(re.findall(r'\btool\s*\(', code))
                  + len(re.findall(r'executor\.execute_action_tracked\s*\(', code)))
    if call_count > 16:
        return False, f"Too many tool calls ({call_count}, max 16)"

    # Must have a return with _create_uniform_return
    if 'executor._create_uniform_return' not in code:
        return False, "Missing return executor._create_uniform_return(...)"

    # Reject silent-failure patterns: except blocks whose body is only continue or pass.
    # Allow except blocks that handle errors visibly (e.g. errors.append, return failure).
    if re.search(r'^\s*except\b[^:]*:\s*\n\s*(continue|pass)\s*$', code, re.MULTILINE):
        return False, "Silent 'except: continue/pass' not allowed. Track errors explicitly and check r['status']."

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
    
    # Try to extract from complete triple backtick fences
    fence_match = re.search(r'```(?:python)?\s*\n(.*?)```', text, re.DOTALL)
    if fence_match:
        return fence_match.group(1).strip()

    # Handle incomplete fenced blocks (common when generation is stopped at closing fence)
    # e.g. "```python\n...code..." or "```\n...code..."
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].strip().startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        return "\n".join(lines).strip()
    
    # Fallback: return everything (may already be raw code)
    return text.strip()


def count_codegen_action_calls(code: str) -> int:
    """Count tool calls (tool() shorthand + legacy execute_action_tracked) in a code block."""
    if not code:
        return 0
    return (len(re.findall(r'\btool\s*\(', code))
            + len(re.findall(r'executor\.execute_action_tracked\s*\(', code)))


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
    # Defensive normalization: some backends may still return fenced blocks.
    code = extract_code_block(code)
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

    def tool(type_name, **kwargs):
        """Shorthand for executor.execute_action_tracked.

        Usage:  r = tool("search-web", query="transformers survey", out="$papers")
        Returns the same dict as execute_action_tracked (keys: status, resource_id, value, extra).
        """
        action = {"type": type_name, **kwargs}
        return executor.execute_action_tracked(action, method_name)

    # Expose helpers both as free functions and executor methods.
    # This prevents common codegen failures like calling executor.get_json(...).
    executor.get_text = get_text
    executor.get_json = get_json
    executor.get_items = get_items

    namespace = {
        "executor": executor,
        "json": json,
        "logger": logger,
        "tool": tool,
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


def _execute_and_record_code_block(code_text: str, executor, step: int) -> tuple:
    """Execute a code block, record it as a single _code_block_ plan action, and
    persist the output as a Note when substantial.

    Returns:
        (result_dict, new_bindings, tool_result_text, code_block_output_created)
    """
    bindings_before = dict(executor.plan_bindings_flat)
    # Snapshot plan_actions so we can replace individual tool calls with one _code_block_
    _pa = getattr(executor, '_plan_actions', [])
    _pa_start = len(_pa)

    result_dict = execute_codegen_block(code_text, executor, "codegen")

    if hasattr(executor, "_done_gate_retry_active"):
        executor._done_gate_retry_active = False

    # Capture the last tool action before replacing individual actions.
    # The last out= binding is the code block's primary artifact.
    last_tool_action = None
    if hasattr(executor, '_plan_actions') and len(executor._plan_actions) > _pa_start:
        # Walk backwards to find the last action with an 'out' param
        for i in range(len(executor._plan_actions) - 1, _pa_start - 1, -1):
            act = executor._plan_actions[i]
            if act.get('out'):
                last_tool_action = act
                break
        # If no out= action, just take the very last action
        if not last_tool_action:
            last_tool_action = executor._plan_actions[-1]

    # Replace individual actions recorded during the code block with a single
    # _code_block_ action containing the source, making cached plans replayable.
    if hasattr(executor, '_plan_actions'):
        replaced_count = len(executor._plan_actions) - _pa_start
        del executor._plan_actions[_pa_start:]
        executor._plan_actions.append({"type": "_code_block_", "source": code_text})
        logger.info(f"Step {step}: Replaced {replaced_count} tracked actions with 1 _code_block_ (source={len(code_text)} chars)")

    logger.info(f"Step {step}: plan_actions count: {len(getattr(executor, '_plan_actions', []))}")

    new_bindings = {k: v for k, v in executor.plan_bindings_flat.items() if bindings_before.get(k) != v}

    # Build Stage 3 result text from the last tool's binding (deterministic)
    # rather than the code block's return value (LLM-chosen, often wrong).
    # The code block return is still used for status (success/failed/reason).
    action = {"type": "_code_block_", "source": code_text}
    status = result_dict.get('status', 'failed')
    if status == 'success' and new_bindings:
        # Use the last binding as the primary result
        last_var, last_rid = list(new_bindings.items())[-1]
        # Try to get a content preview from the bound resource
        content_preview = ''
        if isinstance(last_rid, str) and last_rid.startswith('Note_'):
            try:
                content = executor._get_content(last_rid)
                if content and isinstance(content, str):
                    content_preview = content[:800].replace('\n', ' | ')
                elif content:
                    content_preview = str(content)[:800]
            except Exception:
                pass
        if content_preview:
            tool_result = f"SUCCESS | {content_preview} | _code_block_ completed | Bound: ${last_var} → {last_rid}"
        else:
            tool_result = f"SUCCESS | _code_block_ completed | Bound: ${last_var} → {last_rid}"
        if last_tool_action:
            tool_name = last_tool_action.get('type', 'unknown')
            tool_result = tool_result.replace('_code_block_ completed', f'{tool_name} → _code_block_ completed')
    else:
        tool_result = format_result_text(result_dict, action)

    # Persist code block return value as Note when substantial, but only if the
    # code block didn't already create Note bindings (avoids duplicate content).
    code_block_output_created = False
    note_bindings_created = any(
        isinstance(v, str) and v.startswith("Note_")
        for v in new_bindings.values()
    )
    if result_dict.get('status') == 'success' and not note_bindings_created:
        raw = result_dict.get('data')
        if raw is None:
            raw = result_dict.get('value', '')
        if isinstance(raw, list):
            raw = '\n'.join(str(x) for x in raw)
        elif isinstance(raw, dict):
            raw = json.dumps(raw, ensure_ascii=False)
        else:
            raw = str(raw) if raw is not None else ''
        if isinstance(raw, str) and len(raw) > 80:
            executor.execute_action_tracked({"type": "create-note", "value": raw, "out": "$code_block_output"}, "codegen")
            code_block_output_created = True

    return result_dict, new_bindings, tool_result, code_block_output_created


def _strip_numbered_prefix(text: str) -> str:
    """Strip leading numbered list prefix like '1. ' or '2. ' from LLM output."""
    stripped = text.lstrip()
    m = re.match(r'^\d+\.\s+', stripped)
    if m:
        return stripped[m.end():]
    return stripped


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
    # Handles:
    #   - SGLang ProgramState format: USER: / ASSISTANT: at start of line
    #   - SGLang chat template: <|im_start|>user / <|im_end|>
    #   - vLLM format: <|user|> / <|assistant|>
    # Note: ProgramState(SYSTEM:...) is the initial system section, split only on
    # USER:/ASSISTANT: at line boundaries to avoid matching examples in prompt text.
    sections = re.split(
        r'(<\|im_start\|>(?:user|assistant)|<\|im_end\|>|<\|(?:user|assistant)\|>'
        r'|^(?:USER|ASSISTANT):|\n(?:USER|ASSISTANT):)',
        trace_str,
        flags=re.MULTILINE,
    )
    compressed_events = []

    # Markers to skip (all formats)
    _SKIP_MARKERS = {
        '<|im_start|>user', '<|im_start|>assistant', '<|im_end|>',
        '<|user|>', '<|assistant|>',
        'USER:', 'ASSISTANT:', '\nUSER:', '\nASSISTANT:',
    }
    
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
_SIDE_EFFECT_TOOLS = frozenset({"send-email", "post-bluesky"})


def _normalize_artifact_refs(values: Any) -> List[str]:
    if values is None:
        return []
    if isinstance(values, str):
        values = [values]
    if not isinstance(values, list):
        return []
    refs = []
    for value in values:
        if value is None:
            continue
        if not isinstance(value, str):
            value = str(value)
        value = value.strip()
        if value:
            refs.append(value)
    return list(dict.fromkeys(refs))


def _normalize_artifact_name_map(values: Any) -> Dict[str, str]:
    if not isinstance(values, dict):
        return {}
    normalized = {}
    for key, value in values.items():
        if key is None or value is None:
            continue
        if not isinstance(key, str):
            key = str(key)
        if not isinstance(value, str):
            value = str(value)
        key = key.strip()
        value = value.strip()
        if key and value:
            normalized[key] = value
    return normalized


def _resolve_artifact_reference_id(reference: str, executor, output_artifact_names: Optional[Dict[str, str]] = None) -> str:
    """Resolve resource IDs, $bindings, stable names, and declared output name mappings."""
    if not reference or not executor:
        return ""
    ref = str(reference).strip()
    if not ref:
        return ""
    if ref.startswith(("Note_", "Collection_")):
        return ref

    if ref.startswith("$"):
        binding_key = ref.lstrip("$")
        rid = executor.plan_bindings_flat.get(binding_key) or executor.plan_bindings_flat.get("$" + binding_key)
        if isinstance(rid, str) and rid.startswith(("Note_", "Collection_")):
            return rid
        ref = binding_key

    output_artifact_names = _normalize_artifact_name_map(output_artifact_names)
    mapped = output_artifact_names.get(ref)
    if mapped:
        return _resolve_artifact_reference_id(mapped, executor)

    rid = executor.plan_bindings_flat.get(ref) or executor.plan_bindings_flat.get("$" + ref)
    if isinstance(rid, str) and rid.startswith(("Note_", "Collection_")):
        return rid

    if getattr(executor, "resource_manager", None) and hasattr(executor.resource_manager, "_resolve_resource_id"):
        rid = executor.resource_manager._resolve_resource_id(ref)
        if isinstance(rid, str) and rid.startswith(("Note_", "Collection_")):
            return rid
    return ""


def _get_resource_content_text(resource_id: str, executor) -> str:
    if not resource_id or not executor or not getattr(executor, "resource_manager", None):
        return ""
    try:
        resource = executor.resource_manager.get_resource(resource_id)
        if not resource:
            return ""
        content = resource.get("properties", {}).get("content", "")
        if content is None:
            return ""
        return content if isinstance(content, str) else str(content)
    except Exception:
        return ""


def _is_side_effect_artifact(resource_id: str, executor) -> bool:
    if not resource_id or not executor or not getattr(executor, "resource_manager", None):
        return False
    try:
        resource = executor.resource_manager.get_resource(resource_id)
        if not resource:
            return False
        source_skill = str(resource.get("properties", {}).get("source_skill", "")).strip().lower()
        return source_skill in _SIDE_EFFECT_TOOLS
    except Exception:
        return False


def _resolve_artifact_candidates(references: List[str], executor, output_artifact_names: Optional[Dict[str, str]] = None) -> List[str]:
    candidates: List[str] = []
    for reference in _normalize_artifact_refs(references):
        rid = _resolve_artifact_reference_id(reference, executor, output_artifact_names=output_artifact_names)
        if rid and rid not in candidates:
            candidates.append(rid)
    return candidates


def _is_explicit_artifact_reference(
    raw_reference: str,
    resource_id: str,
    declared_output_artifacts: List[str],
    resolved_output_artifacts: List[str],
    output_artifact_names: Optional[Dict[str, str]] = None,
) -> bool:
    reference = str(raw_reference or "").strip()
    explicit_refs = set(_normalize_artifact_refs(declared_output_artifacts))
    explicit_refs.update(_normalize_artifact_refs(resolved_output_artifacts))
    explicit_refs.update(_normalize_artifact_name_map(output_artifact_names).keys())
    if reference in explicit_refs or reference.lstrip("$") in explicit_refs:
        return True
    explicit_ids = set(_normalize_artifact_refs(resolved_output_artifacts))
    if resource_id in explicit_ids:
        return True
    mapped = _normalize_artifact_name_map(output_artifact_names).get(reference) or _normalize_artifact_name_map(output_artifact_names).get(reference.lstrip("$"))
    return bool(mapped and mapped == resource_id)


def _find_side_effect_content_source(executor) -> str:
    """
    Scan executed plan actions for the last side-effect tool (send-email, post-bluesky)
    and return the resource ID of its input content — the artifact that was consumed
    by the delivery action.

    The primary product of a goal like "write report and email it" is the report,
    not the email confirmation.  The report is whatever the side-effect tool received
    as its ``target`` or ``value``.

    Returns resource ID string, or "" if no side-effect tool was used or input
    cannot be resolved.
    """
    plan_actions = getattr(executor, '_plan_actions', [])
    if not plan_actions:
        return ""

    bindings = getattr(executor, 'plan_bindings_flat', {})

    # Walk backwards to find the last side-effect action
    for action in reversed(plan_actions):
        action_type = action.get('type', '')
        if action_type not in _SIDE_EFFECT_TOOLS:
            continue
        # The content fed to the tool lives in target or value
        for field in ('target', 'value'):
            ref = action.get(field, '')
            if not ref or not isinstance(ref, str):
                continue
            if ref.startswith('$'):
                rid = bindings.get(ref.lstrip('$'))
                if rid and isinstance(rid, str) and (rid.startswith('Note_') or rid.startswith('Collection_')):
                    return rid
            elif ref.startswith('Note_') or ref.startswith('Collection_'):
                return ref
        # Side-effect found but input unresolvable — stop searching
        break

    return ""


def _select_primary_artifact_id(
    executor,
    declared_output_artifacts: List[str],
    resolved_output_artifacts: List[str],
    output_artifact_names: Optional[Dict[str, str]] = None,
    code_block_output_created: bool = False,
) -> str:
    output_artifact_names = _normalize_artifact_name_map(output_artifact_names)

    for rid in _resolve_artifact_candidates(resolved_output_artifacts, executor, output_artifact_names=output_artifact_names):
        return rid

    for rid in _resolve_artifact_candidates(declared_output_artifacts, executor, output_artifact_names=output_artifact_names):
        if not _is_side_effect_artifact(rid, executor):
            return rid

    if code_block_output_created:
        rid = _resolve_artifact_reference_id("$code_block_output", executor)
        if rid:
            return rid

    return ""


def _allow_llm_eval_target_override(
    raw_reference: str,
    resource_id: str,
    executor,
    declared_output_artifacts: List[str],
    resolved_output_artifacts: List[str],
    output_artifact_names: Optional[Dict[str, str]] = None,
) -> bool:
    if not resource_id:
        return False
    if not _is_side_effect_artifact(resource_id, executor):
        return True
    return _is_explicit_artifact_reference(
        raw_reference,
        resource_id,
        declared_output_artifacts=declared_output_artifacts,
        resolved_output_artifacts=resolved_output_artifacts,
        output_artifact_names=output_artifact_names,
    )


def _vision_eval_check(vision_criteria: str, eval_target: str, executor, compressed_context: str = "") -> str:
    """
    Lightweight vision evaluation against planner-declared eval_target.
    eval_target may be a concrete resource ID (Note_xxx) or a $variable.
    Returns evaluation text (pass/fail per criterion) or empty string if skipped.
    """
    if not vision_criteria or not eval_target:
        return ""
    
    res_id = _resolve_eval_target_id(eval_target, executor)
    if not res_id or not executor.resource_manager:
        return ""
    
    load_target = res_id if res_id == eval_target else eval_target
    load_result = executor.execute_action({"type": "load", "target": load_target, "slice": ":", "out": f"$_vision_eval_{res_id}"})
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


def _vision_eval_deep(vision_criteria: str, eval_target: str, classifier_result: str, executor: InfospaceExecutor, compressed_context: str = "", plan_bindings: set = None) -> str:
    """
    Deep vision evaluation via single LLM call with full artifact and upstream
    source context.  Cross-references the artifact against source materials and
    reports PASS/FAIL/INAPPLICABLE per criterion.

    Invoked at the done gate as a read-only quality check on a goal-level artifact.

    Returns evaluation text (containing STATUS: SATISFIED or STATUS: NEEDS_REVISION)
    or empty string if skipped/unavailable.
    """
    if not vision_criteria or not eval_target:
        return ""

    # --- 1. Parse criteria ------------------------------------------------
    criteria_parts = re.split(r'^\d+\.\s*', vision_criteria.strip(), flags=re.MULTILINE)
    criteria_parts = [c.strip() for c in criteria_parts if c.strip()]
    if not criteria_parts:
        return ""
    numbered_criteria = "\n".join(f"{i+1}. {c}" for i, c in enumerate(criteria_parts))

    # --- 2. Load full artifact --------------------------------------------
    res_id = _resolve_eval_target_id(eval_target, executor)
    if not res_id or not executor.resource_manager:
        return ""

    load_target = res_id if res_id == eval_target else eval_target
    try:
        load_result = executor.execute_action({"type": "load", "target": load_target, "slice": ":"})
        artifact_text = str(load_result.get("data", ""))
    except Exception as e:
        logger.warning(f"Vision eval deep: failed to load artifact {eval_target}: {e}")
        return ""
    if not artifact_text:
        return ""

    # --- 3. Load upstream source previews (max 3, ~2000 chars each) -------
    #   Only Note_ bindings created during THIS plan execution.
    #   Stale bindings from prior goals would pollute the cross-reference.
    source_sections = []
    try:
        bindings = executor.plan_bindings_flat or {}
        seen = 0
        for var_name, var_val in bindings.items():
            if seen >= 3:
                break
            if var_name.startswith("_"):
                continue
            if not isinstance(var_val, str):
                continue
            # Only consider bindings created during this plan's execution
            if plan_bindings is not None and var_name not in plan_bindings:
                continue
            # Skip the eval target itself
            if var_val == res_id:
                continue
            # Only load Notes (pipeline artifacts), not Collections (ambient context)
            if not var_val.startswith("Note_"):
                continue
            try:
                src_result = executor.execute_action({"type": "load", "target": f"${var_name}", "slice": ":"})
                src_data = str(src_result.get("data", ""))[:2000]
                if src_data:
                    source_sections.append(f"--- ${var_name} ({var_val}) ---\n{src_data}")
                    seen += 1
            except Exception:
                pass
    except Exception:
        pass

    sources_block = ""
    if source_sections:
        sources_block = "\nUPSTREAM SOURCES (for cross-reference):\n" + "\n\n".join(source_sections) + "\n"

    # --- 4. Build prompt --------------------------------------------------
    classifier_block = ""
    if classifier_result:
        classifier_block = f"\nSHALLOW CLASSIFIER RESULT (may be based on truncated content):\n{classifier_result}\n"

    eval_prompt = f"""You are a quality evaluator. Evaluate the artifact below against each criterion.
Cross-reference the artifact against the upstream sources to verify coverage, accuracy, and grounding.

QUALITY CRITERIA:
{numbered_criteria}
{classifier_block}{sources_block}
ARTIFACT ({eval_target}):
{artifact_text}

INSTRUCTIONS:
- For each criterion, report PASS, FAIL, or INAPPLICABLE with a one-sentence evidence-based reason.
- Use INAPPLICABLE when a criterion does not apply to the artifact (e.g., json_parse_error for plain-text output).
- End with a one-line RECOMMENDATION for the parent planner (e.g., 'lower filter threshold', 'add section headers', 'broaden search terms', or 'remove X criterion' if inapplicable).
- On the final line, write exactly: STATUS: SATISFIED (if all applicable criteria PASS; INAPPLICABLE criteria do not affect STATUS) or STATUS: NEEDS_REVISION (only when an applicable criterion FAILs).

Format:
1. criterion_name: PASS|FAIL|INAPPLICABLE - reason
...
RECOMMENDATION: <one line>
STATUS: SATISFIED or NEEDS_REVISION
END_EVAL"""

    # --- 5. Single LLM call -----------------------------------------------
    try:
        logger.info(f"Vision eval deep: evaluating {eval_target} ({len(criteria_parts)} criteria, {len(source_sections)} upstream sources)")
        result = executor.llm_generate(eval_prompt, max_tokens=512, temperature=0.1, stops=["\nEND_EVAL", "END_EVAL"])
        if result.success and result.text:
            eval_text = result.text.strip()
            logger.info(f"Vision eval deep for {eval_target}: {eval_text[:200]}")
            return eval_text
    except Exception as e:
        logger.warning(f"Vision eval deep failed: {e}")
    return ""


def _resolve_eval_target_id(eval_target: str, executor) -> str:
    """Resolve $var or Note_ID/Collection_ID to a concrete resource ID, or return ''."""
    return _resolve_artifact_reference_id(eval_target, executor)


def _resolve_eval_target_text(eval_target: str, executor) -> str:
    """
    Resolve eval target ($var or Note_ID) to stored Note content string.
    Used for read-only quality-gate fallback delivery.
    """
    if not eval_target or not executor or not getattr(executor, "resource_manager", None):
        return ""
    try:
        resource_id = _resolve_eval_target_id(eval_target, executor)
        if not resource_id:
            return ""
        return _get_resource_content_text(resource_id, executor)
    except Exception:
        return ""




def _count_eval_verdicts(eval_text: str) -> dict:
    """Parse per-criterion PASS/FAIL/INAPPLICABLE counts from eval text.

    Looks for lines matching: criterion_name: PASS|FAIL|INAPPLICABLE [- reason]
    Returns {'pass': N, 'fail': N, 'inapplicable': N, 'total': N, 'applicable': N, 'pass_ratio': float}.
    """
    counts = {'pass': 0, 'fail': 0, 'inapplicable': 0}
    for line in eval_text.splitlines():
        line = line.strip()
        if not line or line.startswith('RECOMMENDATION') or line.startswith('STATUS'):
            continue
        lo = line.lower()
        if ': pass' in lo or lo.endswith(': pass'):
            counts['pass'] += 1
        elif ': fail' in lo:
            counts['fail'] += 1
        elif ': inapplicable' in lo:
            counts['inapplicable'] += 1
    total = counts['pass'] + counts['fail'] + counts['inapplicable']
    applicable = counts['pass'] + counts['fail']
    counts['total'] = total
    counts['applicable'] = applicable
    counts['pass_ratio'] = counts['pass'] / applicable if applicable > 0 else 1.0
    return counts


# Threshold: if this fraction of applicable criteria pass, treat as satisfied
# even if the LLM said NEEDS_REVISION (handles hallucinated criteria)
_VISION_PASS_RATIO_THRESHOLD = 0.75


def _parse_deep_eval_status(deep_text: str) -> str:
    """
    Parse deep-eval status robustly.
    Returns one of: "satisfied", "needs_revision", "unknown".

    Override rules (checked in order):
    1. If RECOMMENDATION says to remove/reframe a criterion → satisfied
    2. If pass ratio among applicable criteria >= threshold → satisfied
    3. Otherwise respect the LLM's STATUS line
    """
    if not deep_text:
        return "unknown"
    text = str(deep_text).strip()

    # Count actual verdicts for ratio-based override
    verdicts = _count_eval_verdicts(text)

    m = re.search(r"^\s*STATUS:\s*(SATISFIED|NEEDS_REVISION)\s*$", text, flags=re.IGNORECASE | re.MULTILINE)
    if m:
        status = m.group(1).strip().upper()
        raw = "satisfied" if status == "SATISFIED" else "needs_revision"
        if raw == "needs_revision":
            # Override 1: recommendation says criterion is inapplicable
            rec_m = re.search(r"RECOMMENDATION[:\s]+(.+?)(?:\n|$)", text, flags=re.IGNORECASE | re.DOTALL)
            if rec_m:
                rec = rec_m.group(1).lower()
                if ("remove" in rec or "reframe" in rec) and "criterion" in rec:
                    logger.info(f"Vision eval: overriding NEEDS_REVISION → satisfied (recommendation: remove/reframe criterion)")
                    return "satisfied"
            # Override 2: pass ratio above threshold
            if verdicts['applicable'] > 0 and verdicts['pass_ratio'] >= _VISION_PASS_RATIO_THRESHOLD:
                logger.info(f"Vision eval: overriding NEEDS_REVISION → satisfied "
                            f"(pass ratio {verdicts['pass']}/{verdicts['applicable']} = {verdicts['pass_ratio']:.0%} "
                            f">= {_VISION_PASS_RATIO_THRESHOLD:.0%} threshold)")
                return "satisfied"
        return raw
    lo = text.lower()
    if any(k in lo for k in ["needs_revision", "needs revision", "revision needed", "any fail", "criterion_"]):
        if "pass" not in lo or "fail" in lo:
            rec_m = re.search(r"RECOMMENDATION[:\s]+(.+?)(?:\n|$)", text, flags=re.IGNORECASE | re.DOTALL)
            if rec_m:
                rec = rec_m.group(1).lower()
                if ("remove" in rec or "reframe" in rec) and "criterion" in rec:
                    return "satisfied"
            # Pass-ratio override for fallback parsing path too
            if verdicts['applicable'] > 0 and verdicts['pass_ratio'] >= _VISION_PASS_RATIO_THRESHOLD:
                logger.info(f"Vision eval: overriding needs_revision → satisfied (pass ratio {verdicts['pass_ratio']:.0%})")
                return "satisfied"
            return "needs_revision"
    if any(k in lo for k in ["no revision needed", "all criteria pass", "all criteria are pass", "status: satisfied"]):
        return "satisfied"
    return "unknown"


ENVISIONMENT_PROMPT = """\
You are generating a retrieval plan for an agent's memory system.
Given the goal, the orient assessment, and active user concerns, produce hypothetical \
answer statements that describe what relevant memory entries would look like if they \
existed. These will be used as search queries against an embedding index.

Rules:
- Each envisionment is a declarative statement, not a question.
- Prefer compound statements capturing relationships between entities over decomposed \
single-entity queries.
- Use the orient rationale and concerns to resolve ambiguous references.
- Do not invent specific values (numbers, dates, names not in the input). Use hedging: \
"approximately," "around," "related to."
- If no memory retrieval would help, set retrieval_needed to false.
- Generate 0-4 envisionments. Most goals need 1-2.

Goal: {goal}

Orient assessment:
  Action choice: {action_choice}
  Rationale: {rationale}

Active concerns:
{concerns_text}

Respond with JSON only. Schema:
{{"retrieval_needed": bool, "envisionments": ["statement 1", "statement 2", ...]}}"""

# Orient action choices that skip envisionment generation (reactive, no retrieval needed)
_SKIP_ENVISIONMENT_ACTIONS = {'no_action', 'inform_user'}


def generate_envisionments(goal: str, orient_assessment, active_concerns, executor) -> List[str]:
    """Generate HyDE-style envisionment strings for memory retrieval.

    Returns a list of declarative statements suitable as FAISS queries,
    or an empty list if retrieval is not needed or generation fails.
    """
    # Check trigger: skip for reactive action choices
    if orient_assessment:
        action_eval = orient_assessment.get('action_evaluation') or {}
        action_choice = action_eval.get('action_choice', 'no_action')
        rationale = (orient_assessment.get('action_evaluation') or {}).get('rationale', '')
        if not rationale:
            rationale = orient_assessment.get('overall_rationale', '')
    else:
        action_choice = 'unknown'
        rationale = ''

    if action_choice in _SKIP_ENVISIONMENT_ACTIONS:
        logger.debug(f"Envisionment skipped: action_choice={action_choice}")
        return []

    # Format concerns
    concern_lines = []
    for c in (active_concerns or [])[:8]:
        label = c.get('concern_label', '?')
        desc = c.get('concern_description', '')
        concern_lines.append(f"  - {label}: {desc}")
    concerns_text = "\n".join(concern_lines) if concern_lines else "  (none)"

    prompt = ENVISIONMENT_PROMPT.format(
        goal=goal,
        action_choice=action_choice,
        rationale=rationale[:300],
        concerns_text=concerns_text,
    )

    try:
        result = executor.llm_generate(
            prompt, max_tokens=256, temperature=0.3,
            is_json=True, stops=["\n\n"]
        )
        if not result.success or not result.text:
            logger.warning(f"Envisionment LLM call failed: {getattr(result, 'error', 'unknown')}")
            return []

        # Parse response
        text = result.text
        if isinstance(text, dict):
            data = text
        else:
            text = str(text).strip()
            if text.startswith('```'):
                lines = text.split('\n')
                lines = [l for l in lines if not l.strip().startswith('```')]
                text = '\n'.join(lines).strip()
            data = json.loads(text)

        if not data.get('retrieval_needed', True):
            logger.info("Envisionment: retrieval not needed (LLM second gate)")
            return []

        envisionments = data.get('envisionments', [])
        if not isinstance(envisionments, list):
            return []
        # Filter to strings, cap at 4
        envisionments = [str(e) for e in envisionments if e and str(e).strip()][:4]
        if envisionments:
            logger.info(f"Envisionment: generated {len(envisionments)} queries: {[e[:80] for e in envisionments]}")
        return envisionments

    except (json.JSONDecodeError, KeyError, TypeError) as e:
        logger.warning(f"Envisionment parse error: {e}")
        return []
    except Exception as e:
        logger.warning(f"Envisionment generation error: {e}")
        return []


if HAS_SGLANG:
    #@function
    def stage0_resource_retrieval(goal: str, executor, orient_assessment=None, active_concerns=None):
        """
        Stage 0: Generate envisionment queries and retrieve relevant resources.

        Uses HyDE-style envisionments when orient context is available,
        falling back to the raw goal text as query.

        Args:
            goal: Goal text
            executor: InfospaceExecutor instance
            orient_assessment: Orient output dict (optional)
            active_concerns: List of active user concern dicts (optional)

        Returns:
            Formatted string with available resources to inject into Stage 1 prompt
        """

        try:
            # Generate envisionment queries (HyDE-style hypothetical answers)
            envisionments = generate_envisionments(goal, orient_assessment, active_concerns, executor)

            # Use envisionments as primary queries, keep raw goal as fallback
            if envisionments:
                queries = envisionments + [goal]  # envisionments first, raw goal as broad fallback
                k_notes = min(3 + len(envisionments), 8)  # slightly more candidates with multiple queries
            else:
                queries = [goal]
                k_notes = 3

            # Search for resources
            search_result = executor.search_resources(queries, k_notes=k_notes, k_collections=2, threshold=0.3)
            
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
            
            # Format results for prompt injection — compact table format
            lines = ["# Available Resources\n"]

            if notes:
                # Filter out internal notes (underscore-prefixed names are bookkeeping)
                notes = [n for n in notes
                         if not (n.get('name', '') or '').startswith('_')]
            if notes:
                lines.append("Notes:")
                for note in notes:
                    name = note.get('name', note.get('resource_id', ''))
                    resource_id = note.get('resource_id', '')
                    lines.append(f"  {resource_id} \"{name}\"")

            if collections:
                lines.append("Collections:")
                for coll in collections:
                    name = coll.get('name', coll.get('resource_id', ''))
                    resource_id = coll.get('resource_id', '')
                    item_count = coll.get('item_count', 0)
                    lines.append(f"  {resource_id} \"{name}\" ({item_count} items)")

            lines.append("\nUse `load` by name or ID to access these resources.")
            
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
                              tools_catalog_text: str, executor, trace_file=None, max_steps: int = 16, similar_plan: Dict = None, preplan: str = None, vision_criteria: str = "", output_artifacts: List[str] = None, resolved_output_artifacts: List[str] = None, output_artifact_names: Dict[str, str] = None):
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

        # Extract output sizing guidance from recent_context (if present)
        _output_guidance_line = ""
        if "OUTPUT GUIDANCE" in recent_context and "target_tokens:" in recent_context:
            import re as _re
            _m = _re.search(r'target_tokens:\s*(\d+)', recent_context)
            if _m:
                _output_guidance_line = f"OUTPUT SIZING: pass target_tokens={_m.group(1)} to the tool call that produces the FINAL output artifact.\n"          
        
        # Stage 0: Resource retrieval with envisionment-driven queries
        available_resources_text = ""
        if executor:
            try:
                available_resources_text = stage0_resource_retrieval(
                    goal=goal, executor=executor,
                    orient_assessment=getattr(executor, '_orient_assessment', None),
                    active_concerns=getattr(executor, '_user_concerns_snapshot', None),
                )
            except Exception as e:
                logger.warning(f"Stage 0: Failed to retrieve resources: {e}")

        # Stage 1: Analysis + tool selection
        # Goal is placed at the end of the system prompt (high-attention zone)
        system_parts = ["You can write code blocks to achieve the goal, including using tools/primitives (aka actions), if and as needed,"]
        system_parts.append("and loop over execute-step / reflect until the goal is satisfied.")
        system_parts.append(f"\n{INCREMENTAL_PLAN_SPECIFICATIONS}\n")
        system_parts.append(f"Complete primitive and tool catalog:\n{tools_catalog_text}\n#### END OF INFOSPACE TYPE SYSTEM, SPECIFICATIONS, AND TOOL CATALOG\n\n")

        system_parts.append(f"Setting:\n{character_context}\n\n")
        system_parts.append(f"Situation/Context:\n{recent_context}\n\n")
        #system_parts.append(f"Again, \n#GOAL:\n\n{template}\n{goal}\n\n")
        if preplan:
            system_parts.append(f"\n## ABSTRACT_PLAN\n{preplan}\n## End ABSTRACT_PLAN\n")

        if vision_criteria:
            system_parts.append(f"\n## QUALITY VISION (external — checked by the system, NOT by your code)\n"
            f"These criteria are evaluated externally at the done gate. Do NOT implement them as checks in your code blocks.\n"
            f"They are shown here so you understand what quality looks like — use them to guide your approach, not to build verification loops.\n"
            f"{vision_criteria}\n## End QUALITY VISION\n")

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

        # Compact world model: only fact text + confidence (booking fields are noise)
        wm_facts = [{"fact": f["fact"], "confidence": f.get("confidence", "?")}
                     for f in world_model.get("facts", []) if f.get("fact")]
        wm_contracts = world_model.get("tool_contracts", [])
        wm_lines = ["WORLD_MODEL:"]
        if wm_facts:
            wm_lines.append("Facts:")
            for f in wm_facts:
                wm_lines.append(f"- [{f['confidence']}] {f['fact']}")
        if wm_contracts:
            wm_lines.append(f"Tool contracts: {json.dumps(wm_contracts)}")
        system_parts.append("\n".join(wm_lines) + "\n")

        system_parts.append(f"CURRENT_TIME: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n")
        system_parts.append(f"\n#GOAL:\n{goal}\n")
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
            "   Consult WORLD_MODEL tool_contracts for known tool capabilities and constraints.\n"
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
            + gen("stage1_reasoning", max_tokens=192, temperature=REFLECT_TEMPERATURE, stop="</reasoning>")
            + "</reasoning>\n"
            "<tools>\n"
            + gen("selected_tools_json", max_tokens=96, temperature=SELECT_TEMPERATURE, stop="</tools>")
            + "</tools>\n"
            "<first_task>\n"
            + gen("first_task", max_tokens=128, temperature=REFLECT_TEMPERATURE, stop="</first_task>")
            + "</first_task>\n"
        )
        
        # Strip <think> tags and partial XML fragments from Stage 1 outputs
        for _key in ['stage1_reasoning', 'selected_tools_json', 'first_task']:
            if _key in s:
                val = _strip_think_tags(s[_key])
                # Remove trailing partial XML tags (e.g., "</" left by stop-token truncation)
                val = re.sub(r'</?[a-z_]*\s*$', '', val).rstrip()
                s[_key] = val

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
                    _exec_actions = set(executor._get_known_action_names()) if hasattr(executor, '_get_known_action_names') else set()
                    expanded_docs = load_skill_docs(tools_to_load, executor.available_tools, _exec_actions)
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
        
        # Snapshot planner reasoning state for generate-reflective-note tool
        if executor:
            try:
                base_state = str(s)
                # Append OODA orientation snapshot if available
                try:
                    from ooda_snapshot_renderer import render_reflective_snapshot
                    _ls = getattr(executor, '_ooda_living_state', None)
                    _dc = getattr(executor, '_derived_concerns_snapshot', [])
                    _uc = getattr(executor, '_user_concerns_snapshot', [])
                    snapshot = render_reflective_snapshot(_ls, _dc, _uc)
                    if snapshot:
                        base_state += f"\n\n{snapshot}"
                except Exception:
                    pass
                # Append operational self-model section
                try:
                    from ooda_snapshot_renderer import render_self_model_section
                    self_model = render_self_model_section(
                        scheduler_status=_safe_get_scheduler_status(executor),
                        tasks=_safe_get_all_tasks(executor),
                        derived_concerns=_dc,
                        scheduled_goals=_safe_get_scheduled_goals(executor),
                        sensor_configs=_safe_get_sensor_configs(executor),
                        execution_mode=_safe_get_execution_mode(executor),
                        tool_count=len(getattr(executor, 'available_tools', {})),
                    )
                    if self_model:
                        base_state += f"\n\n{self_model}"
                except Exception:
                    pass
                executor._planner_reflective_state = base_state
            except Exception:
                executor._planner_reflective_state = ""

        # Interrupt checkpoint: after Stage 1, before step loop
        if executor and _interrupt_requested(executor):
            _clear_interrupt(executor)
            s["final_answer"] = "Interrupted by user."
            return s

        # Stage 2/3 format instructions
        s += user(
            "#Stage 2 FORMAT:\n"
            "Write a Python code block to accomplish the CURRENT_TASK.\n"
            "Linear example:\n"
            "```python\n"
            "r1 = tool(\"search-web\", query=\"transformers survey\", out=\"$papers\")\n"
            "if r1[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=f\"search failed: {r1.get('reason', 'unknown')}\")\n"
            "r2 = tool(\"synthesize\", target=\"$papers\", focus=\"key findings\", out=\"$summary\")\n"
            "if r2[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=f\"synthesize failed: {r2.get('reason', 'unknown')}\")\n"
            "return executor._create_uniform_return(\"success\", value=\"done\")\n"
            "```\n"
            "Reading content example:\n"
            "```python\n"
            "# To read content from a bound $var, use get_text/get_json/get_items directly.\n"
            "# NO need to call load first — these read straight from the resource store.\n"
            "text = get_text(\"$my_note\")       # returns Note content as string\n"
            "data = get_json(\"$my_note\")       # returns Note content parsed as dict (or None)\n"
            "ids  = get_items(\"$my_collection\") # returns list of Note IDs in the Collection\n"
            "```\n"
            "Loop example (multi-item processing):\n"
            "```python\n"
            "items = get_items(\"$inbox\")  # list of resource IDs from a Collection\n"
            "total = len(items)\n"
            "ok = 0\n"
            "errors = []\n"
            "for item_id in items:\n"
            "    r = tool(\"extract\", target=item_id, instruction=\"get subject\", out=\"$subject\")\n"
            "    if r[\"status\"] == \"success\":\n"
            "        subj = get_json(\"$subject\")\n"
            "        ok += 1\n"
            "    else:\n"
            "        errors.append(r.get(\"reason\", \"unknown\"))\n"
            "if ok == 0:\n"
            "    return executor._create_uniform_return(\"failed\", reason=f\"All {total} items failed: {errors[:3]}\")\n"
            "return executor._create_uniform_return(\"success\", value=f\"Processed {ok}/{total}\", extra={\"errors\": errors})\n"
            "```\n"
            "Rules:\n"
            "- Call tools via: r = tool(\"tool-name\", param=value, out=\"$var\")\n"
            "- tool() returns dict with r[\"status\"] (\"success\"/\"failed\"), r[\"resource_id\"], r[\"value\"] (display string).\n"
            "- Max 16 tool() calls per code block.\n"
            "- Prefer longer cohesive blocks: if CURRENT_TASK has connected subtasks, combine them in one block.\n"
            "- out=\"$name\" binds the result resource. Chain via target=\"$name\" in the next call.\n"
            "- Read content: get_text(\"$var\") → string, get_json(\"$var\") → dict or None, get_items(\"$var\") → list of Note IDs.\n"
            "  These work on any $var that has been bound. No need to call load first.\n"
            "- load: ONLY for (a) binding a named persistent note, or (b) slicing a Collection.\n"
            "- Pass dicts/lists directly as value — do not pre-serialize with json.dumps().\n"
            "- CODE BLOCK SCOPE: each step executes in a fresh function. Python locals do NOT persist across steps.\n"
            "  Only $bindings (created via out=) persist. Use get_text/get_json/get_items for cross-step access.\n"
            "- if/else control flow and loops are allowed.\n"
            "- Must end with: return executor._create_uniform_return(status, value=..., extra=...)\n"
            "- LOOP PATTERN: tool() never raises — do NOT use try/except. Track ok/errors counts explicitly.\n"
            "- OUTPUT SIZING: If OUTPUT GUIDANCE appears in the context with a target_tokens value,\n"
            "  pass target_tokens=<N> to the tool call that produces the FINAL output artifact of the goal\n"
            "  (the primary product — e.g. the last synthesize, generate-note, or extract call).\n"
            "  Do NOT apply target_tokens to intermediate/preparatory tool calls.\n"
            "\n"
            "#Stage 3 FORMAT:\n"
            "  THOUGHTS: <brief assessment of result and progress>\n"
            "  EVAL_TARGET: <single $variable (e.g. $report) of the key output artifact from this step, or NONE if no significant artifact>\n"
            "  DONE: YES or NO — is the entire GOAL satisfied?\n"
            "  NEXT_TASK: <next task description, or blank if DONE=YES>\n"
            "  REQUEST_TOOLS: YES or NO — do you need detailed docs for tools not yet loaded?\n"
            "  ASK_USER: YES or NO — are you stuck and need user guidance?\n"
            "\n"
            "#Stage 3 RULES:\n"
            "- DONE=YES only when ALL required actions are complete.\n"
            "- If DONE=YES, NEXT_TASK must be blank.\n"
            "- Use 'say' in code blocks when communication is part of the goal (e.g., reporting results, echoing data). Use 'ask' when you need the user's response to continue.\n"
            "- Efficiency rule: if prior step used only one tool call and GOAL is not done, set NEXT_TASK to combine adjacent subtasks.\n"
            "- SPIRAL DETECTION: If the same tool has failed 2+ times consecutively,\n"
            "  use a different approach or proceed with available data.\n"
            "- PARTIAL FAILURE: For tasks processing multiple items, check the value string for counts.\n"
            "  '0 of N processed' is a failure — set DONE=NO and retry with a corrected approach.\n"
            "- REQUEST_TOOLS=YES: you will be prompted for tool names (e.g. search-web, extract).\n"
            "- ASK_USER=YES: use only when you genuinely lack information the user has — wrong target,\n"
            "  ambiguous goal, repeated failure you cannot diagnose, or a preference you cannot resolve.\n"
            "  Always try at least once before asking. At most one ASK_USER per goal.\n"
            "\n"
            "Follow these formats exactly."
        )
        s += assistant("Understood.\n\n")

        # Main loop
        current_task = s["first_task"].strip()
        _declared_output_artifacts = _normalize_artifact_refs(output_artifacts)
        _resolved_output_artifacts = _normalize_artifact_refs(resolved_output_artifacts)
        _output_artifact_names = _normalize_artifact_name_map(output_artifact_names)
        last_eval_target = _select_primary_artifact_id(
            executor,
            declared_output_artifacts=_declared_output_artifacts,
            resolved_output_artifacts=_resolved_output_artifacts,
            output_artifact_names=_output_artifact_names,
        )
        if last_eval_target:
            logger.info(f"Eval target seeded from explicit artifact context: {last_eval_target}")
        stall_guard_state = {"prev_signature": None, "repeat_count": 0}
        deep_eval_retried = False
        deep_eval_prev_artifact = None
        plan_local_bindings = set()
        _asked_user_this_goal = False
        _mid_vision_eval_counts = {}  # {eval_target_id: count} — suppress after cap
        _MID_VISION_EVAL_CAP = 2     # max evals per artifact before suppressing
        for step in range(max_steps):
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                s["final_answer"] = "Interrupted by user."
                break

            # Stage 2: Generate and execute code block
            _bindings_line = _build_bindings_inventory(plan_local_bindings, executor) if step > 0 else ""
            s += user(
                f"STAGE 2 (step {step + 1}/{max_steps}):\n"
                f"#GOAL: {goal_for_step}\n#END GOAL\n"
                f"{_bindings_line}"
                f"CURRENT_TASK: {current_task}\n"
                f"{_output_guidance_line}"
                "Write a Python code block using Stage 2 FORMAT.\n"
                "Reminder: chain via $bindings (out=\"$name\"), end with return executor._create_uniform_return(...), max 16 tool calls. Locals don't persist across steps. No try/except — check r[\"status\"] instead.\n"
            )

            s += assistant(
                "```python\n"
                + gen(f"code_block_{step}", max_tokens=2048, temperature=CODE_TEMPERATURE, stop=["\n```"])
                + "\n```\n"
            )
            
            code_text = _strip_think_tags(s[f"code_block_{step}"].strip())
            # Strip trailing incomplete backtick fences (model sometimes emits `` or ` without newline)
            code_text = re.sub(r'`{1,3}\s*$', '', code_text).rstrip()
            s[f"code_block_{step}"] = code_text
            logger.info(f"Step {step}: Code block ({len(code_text)} chars):\n{code_text}")
            call_count = count_codegen_action_calls(code_text)
            logger.info(f"Step {step}: Code block action calls={call_count} (logged before execution)")

            # Interrupt checkpoint: after code-block gen, before execute
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                s["final_answer"] = "Interrupted by user."
                break

            # Set task context for tool exemplar recording
            executor._current_task_context = current_task

            result_dict, new_bindings, tool_result, code_block_output_created = _execute_and_record_code_block(code_text, executor, step)
            
            logger.info(f"Step {step}: -> {tool_result[:100]}")
            if new_bindings:
                plan_local_bindings.update(new_bindings.keys())
                logger.info(f"Step {step}: New bindings: {new_bindings}")
            
            # Interrupt checkpoint: after code-block execution
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                # Ask to User intentionally sets interrupt to pause for response - treat as success, not failure
                plan_actions = getattr(executor, '_plan_actions', [])
                last_ask = plan_actions and plan_actions[-1].get('type') == 'ask' and str(plan_actions[-1].get('target', '')).lower() == 'user'
                if last_ask:
                    s["ask_completed_successfully"] = True
                s["final_answer"] = "Interrupted by user."
                break
            
            # Stage 3: Evaluate result
            bindings_summary = ", ".join(f"${k}={v}" for k, v in new_bindings.items() if not str(k).startswith("_")) if new_bindings else "none"
            exemplar_hint = _get_exemplar_hint_for_failure(tool_result, executor)
            s += user(
                f"=====\n"
                f"STAGE 3 (step {step + 1}/{max_steps})\n"
                f"=====\n\n"
                f"CURRENT_TASK: {current_task}\n"
                f"NEW_BINDINGS: {bindings_summary}\n\n"
                f">> RESULT (ground truth) <<\n"
                f"{tool_result}\n"
                f">> END RESULT <<\n\n"
                f"{exemplar_hint}"
                f"Evaluate: Is the GOAL fully achieved? Use ONLY the result above as ground truth.\n"
                f"Respond using Stage 3 FORMAT.\n"
            )
            
            # Vision evaluation before planner reflects — resolve to concrete resource ID
            eval_target = _select_primary_artifact_id(
                executor,
                declared_output_artifacts=_declared_output_artifacts,
                resolved_output_artifacts=_resolved_output_artifacts,
                output_artifact_names=_output_artifact_names,
                code_block_output_created=code_block_output_created,
            )
            # Fallback: if no declared/resolved artifact, use the last out= binding
            # from the code block. The last binding is almost always the primary result.
            if not eval_target and new_bindings:
                last_var, last_rid = list(new_bindings.items())[-1]
                if isinstance(last_rid, str) and (last_rid.startswith('Note_') or last_rid.startswith('Collection_')):
                    eval_target = last_rid
                    logger.info(f"Step {step}: Eval target from last binding: ${last_var} → {last_rid}")
            if eval_target:
                last_eval_target = eval_target
            # If a side-effect tool ran, prefer its input content as eval target
            # (e.g. the report that was emailed, not the email confirmation)
            side_effect_content = _find_side_effect_content_source(executor)
            if side_effect_content:
                last_eval_target = side_effect_content
                logger.info(f"Step {step}: Eval target updated to side-effect content source: {side_effect_content}")
            # Mid-loop vision eval: only run on declared output artifacts (not every intermediate binding)
            run_mid_vision = False
            if eval_target and vision_criteria and _declared_output_artifacts and executor:
                for oa in _declared_output_artifacts:
                    oa_key = oa.strip().lstrip("$")
                    oa_rid = executor.plan_bindings_flat.get(oa_key) or executor.plan_bindings_flat.get("$" + oa_key)
                    if oa_rid and oa_rid == eval_target:
                        run_mid_vision = True
                        break
            if run_mid_vision:
                # Enforce per-artifact eval cap to prevent loops on unfixable criteria
                mid_key = _resolve_eval_target_id(last_eval_target, executor) or last_eval_target
                _mid_vision_eval_counts[mid_key] = _mid_vision_eval_counts.get(mid_key, 0) + 1
                if _mid_vision_eval_counts[mid_key] <= _MID_VISION_EVAL_CAP:
                    logger.info(f"Step {step}: Vision eval target (declared artifact): {last_eval_target} (eval #{_mid_vision_eval_counts[mid_key]})")
                    compressed_ctx = _compress_trace(str(s))
                    vision_eval_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)
                    if vision_eval_text:
                        s += user(f"VISION EVALUATION:\n{vision_eval_text}\nFAILs are advisory — attempt to address if easy, do NOT loop on unfixable criteria.\n")
                        s += assistant("Noted.\n")
                else:
                    logger.info(f"Step {step}: Vision eval suppressed for {last_eval_target} (eval cap {_MID_VISION_EVAL_CAP} reached)")

            # Stage 3: Structured reflection (6 fields, select() for categorical decisions)
            s += assistant(
                "THOUGHTS: "
                + gen(f"thoughts_{step}", max_tokens=128, temperature=REFLECT_TEMPERATURE, stop="\nEVAL_TARGET:")
                + "\nEVAL_TARGET: "
                + gen(f"eval_target_{step}", max_tokens=32, temperature=SELECT_TEMPERATURE, stop="\nDONE:")
                + "\nDONE: "
                + select(f"done_{step}", ["YES", "NO"])
                + "\nNEXT_TASK: "
                + gen(f"next_task_{step}", max_tokens=128, temperature=REFLECT_TEMPERATURE, stop="\nREQUEST_TOOLS:")
                + "\nREQUEST_TOOLS: "
                + select(f"request_tools_{step}", ["YES", "NO"])
                + "\nASK_USER: "
                + select(f"ask_user_{step}", ["YES", "NO"])
                + "\n"
            )
            # Conditional follow-up: generate tool list only when REQUEST_TOOLS=YES
            if s[f"request_tools_{step}"] == "YES":
                s += assistant(
                    "REQUEST_TOOLS_LIST: "
                    + gen(f"request_tools_list_{step}", max_tokens=96, temperature=SELECT_TEMPERATURE, stop=["\n\n", "\nTHOUGHTS:", "\nSTAGE", "\nASK_USER"])
                    + "\n"
                )
            # Conditional follow-up: generate question text only when ASK_USER=YES
            if s[f"ask_user_{step}"] == "YES":
                s += assistant(
                    "ASK_USER_QUESTION: "
                    + gen(f"ask_user_question_{step}", max_tokens=192, temperature=REFLECT_TEMPERATURE, stop=["\n\n", "\nTHOUGHTS:", "\nSTAGE"])
                    + "\n"
                )
            
            def safe_get(state, key, default='N/A'):
                try:
                    return state[key]
                except (KeyError, TypeError, AttributeError):
                    return default
            
            logger.info(f"THOUGHTS: {safe_get(s, f'thoughts_{step}')}")
            logger.info(f"EVAL_TARGET: {safe_get(s, f'eval_target_{step}')}")
            logger.info(f"DONE: {safe_get(s, f'done_{step}')}")
            logger.info(f"NEXT_TASK: {safe_get(s, f'next_task_{step}')}")
            if s[f"request_tools_{step}"] == "YES":
                logger.info(f"REQUEST_TOOLS: YES — {safe_get(s, f'request_tools_list_{step}', '')[:200]}")
            if s[f"ask_user_{step}"] == "YES":
                logger.info(f"ASK_USER: YES — {safe_get(s, f'ask_user_question_{step}', '')[:200]}")

            # Update last_eval_target from LLM-declared EVAL_TARGET (overrides heuristic)
            llm_eval_target_raw = safe_get(s, f"eval_target_{step}", "").strip()
            if llm_eval_target_raw and llm_eval_target_raw.upper() != "NONE":
                llm_rid = _resolve_artifact_reference_id(
                    llm_eval_target_raw,
                    executor,
                    output_artifact_names=_output_artifact_names,
                )
                if llm_rid and _allow_llm_eval_target_override(
                    llm_eval_target_raw,
                    llm_rid,
                    executor,
                    declared_output_artifacts=_declared_output_artifacts,
                    resolved_output_artifacts=_resolved_output_artifacts,
                    output_artifact_names=_output_artifact_names,
                ):
                    last_eval_target = llm_rid
                    logger.info(f"Step {step}: LLM-declared eval target resolved: {last_eval_target}")
            
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
            if s[f"request_tools_{step}"] == "YES":
                requested_tools_raw = _strip_numbered_prefix(
                    _strip_think_tags(safe_get(s, f"request_tools_list_{step}", "")).strip()
                )
                requested_tools = parse_request_tools(requested_tools_raw)
                if requested_tools:
                    logger.info(f"Step {step}: LLM requested additional tools: {requested_tools}")
                    tools_to_load = [tool for tool in requested_tools if tool not in _loaded_skill_docs]
                    if tools_to_load:
                        _exec_actions = set(executor._get_known_action_names()) if hasattr(executor, '_get_known_action_names') else set()
                        expanded_docs = load_skill_docs(tools_to_load, executor.available_tools, _exec_actions)
                        if expanded_docs:
                            s += user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                            s += assistant("I have reviewed the additional tool documentation.\n")
                            logger.info(f"Stage 3.5: Loaded docs for {len(tools_to_load)} additional tools: {tools_to_load}")
                            _loaded_skill_docs.update(tools_to_load)
                    else:
                        logger.debug(f"Stage 3.5: All requested tools already have docs loaded, skipping")
                elif requested_tools_raw:
                    logger.warning(f"Step {step}: REQUEST_TOOLS=YES but failed to parse tool list: {requested_tools_raw[:100]}")
            
            # Strip <think> tags from model outputs (Qwen3 thinking mode / OpenRouter provider variance)
            # Note: done_{step}, request_tools_{step}, and ask_user_{step} use select(), not gen(), so they're immune.
            for _key in [f"thoughts_{step}", f"eval_target_{step}", f"next_task_{step}", f"request_tools_list_{step}", f"ask_user_question_{step}"]:
                if _key in s:
                    s[_key] = _strip_think_tags(s[_key])

            # Stage 3.6: ASK_USER — pause and ask the user for guidance (at most once per goal)
            if s[f"ask_user_{step}"] == "YES" and not _asked_user_this_goal:
                ask_question = _strip_think_tags(safe_get(s, f"ask_user_question_{step}", "")).strip()
                if ask_question:
                    _asked_user_this_goal = True
                    logger.info(f"Step {step}: ASK_USER triggered — pausing for user input")
                    ask_result = executor.execute_action_tracked(
                        {"type": "ask", "value": ask_question, "out": "$_user_guidance"},
                        "codegen"
                    )
                    if ask_result.get("status") == "success":
                        user_response = ask_result.get("value", "")
                        s += user(f"USER_GUIDANCE (response to your question): {user_response}\nIncorporate this guidance into your next step.")
                        s += assistant("Understood, I will incorporate the user's guidance.\n")
                        logger.info(f"Step {step}: User responded to ASK_USER: {user_response[:200]}")
                    else:
                        s += user("ASK_USER timed out or was interrupted. Proceed with your best judgment.")
                        s += assistant("Understood, proceeding without user input.\n")
                        logger.info(f"Step {step}: ASK_USER failed or timed out — continuing")
                    # Check for interrupt after ask (user may have cancelled)
                    if _interrupt_requested(executor):
                        _clear_interrupt(executor)
                        s["ask_completed_successfully"] = True
                        s["final_answer"] = "Interrupted by user."
                        break

            # Check if done
            done_raw = _strip_numbered_prefix(s[f"done_{step}"].strip()).upper()
            next_task_raw = _strip_numbered_prefix(s[f"next_task_{step}"].strip())
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
                    + gen("VERIFICATION_ANSWER", max_tokens=96, temperature=SELECT_TEMPERATURE, stop="\n")
                )
                logger.info(f"VERIFICATION ANSWER: {s['VERIFICATION_ANSWER']}")
                
                # Deep vision evaluation at done gate
                if vision_criteria and last_eval_target:
                    logger.info(f"Done gate: Running deep vision eval on {last_eval_target}")
                    compressed_ctx = _compress_trace(str(s))
                    shallow_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)
                    deep_text = _vision_eval_deep(vision_criteria, last_eval_target, shallow_text or "", executor, compressed_context=compressed_ctx, plan_bindings=plan_local_bindings)
                    if deep_text:
                        s += user(f"DEEP VISION EVALUATION (final quality gate, read-only):\n{deep_text}\nUse this only as QA signal. Do NOT run additional tool steps at done gate.\n")
                        s += assistant("Noted.\n")
                        logger.info(f"Done gate: Deep eval result injected")
                        deep_status = _parse_deep_eval_status(deep_text)
                        if deep_status == "needs_revision":
                            if not deep_eval_retried:
                                deep_eval_retried = True
                                deep_eval_prev_artifact = _resolve_eval_target_text(last_eval_target, executor)
                                logger.info("Done gate: Deep eval returned NEEDS_REVISION; reopening loop for one retry")
                                if hasattr(executor, "_done_gate_retry_active"):
                                    executor._done_gate_retry_active = True
                                s += user(
                                    f"QUALITY GATE FAILED — you must revise the artifact in {last_eval_target}.\n"
                                    f"Issues found:\n{deep_text}\n\n"
                                    f"Regenerate or fix the artifact to address the FAILed criteria, then mark DONE again."
                                )
                                s += assistant("Understood. I will revise the artifact to address the failed criteria.\n")
                                s[f"done_{step}"] = "NO"
                                continue
                            else:
                                retry_artifact = _resolve_eval_target_text(last_eval_target, executor)
                                if deep_eval_prev_artifact and retry_artifact == deep_eval_prev_artifact:
                                    logger.info("Done gate: Deep eval NEEDS_REVISION on retry but artifact unchanged (tool limitation); accepting as satisfied")
                                else:
                                    logger.info("Done gate: Deep eval returned NEEDS_REVISION on retry; freezing artifact")
                                    draft_text = retry_artifact
                                    caveat = (
                                        "Quality gate warning: deep evaluation found issues after retry (STATUS: NEEDS_REVISION). "
                                        "Delivering best available draft from this cycle."
                                    )
                                    s["VERIFICATION_ANSWER"] = "PARTIAL"
                                    s[f"done_{step}"] = "NO"
                                    s["final_answer"] = f"{caveat}\n\n{draft_text}" if draft_text else caveat
                                    break
                    if shallow_text and "fail" in shallow_text.lower():
                        s += user(f"VISION EVALUATION (final quality gate, shallow, read-only):\n{shallow_text}\nFAILs are advisory. Do NOT run additional tool steps at done gate.\n")
                        s += assistant("Noted.\n")
                        logger.info("Done gate: Shallow eval has FAILs (read-only advisory); proceeding to finalization")
                
                # Generate final answer
                next_task_raw = s[f"next_task_{step}"].strip()
                if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a", "blank", "(blank)"]:
                    final_prompt = next_task_raw
                else:
                    final_prompt = "Summarize the results with focus on the original goal"
                
                s += user(f"FINAL TASK: {final_prompt}\nProvide a final response or answer. End your response with </end>")
                s += assistant(gen("final_answer", max_tokens=256, temperature=REFLECT_TEMPERATURE, stop=["</end>"]))
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
                # If we haven't nudged yet, give one more step with ASK_USER hint
                if not stall_guard_state.get("nudged"):
                    s += user(
                        "STALL DETECTED: You are repeating the same approach without progress. "
                        "Consider using ASK_USER in your next Stage 3 to get guidance from the user, "
                        "or try a fundamentally different approach. One more step allowed."
                    )
                    s += assistant("Understood, I will reassess.\n")
                    logger.info(f"Step {step}: Stall guard triggered — nudging ASK_USER before giving up")
                    stall_guard_state["prev_signature"] = None
                    stall_guard_state["repeat_count"] = 0
                    stall_guard_state["nudged"] = True
                    continue  # Give one more iteration
                draft_text = _resolve_eval_target_text(last_eval_target, executor) if last_eval_target else ""
                caveat = "Loop guard: repeated planning pattern detected. Delivering best available draft from this cycle."
                s["VERIFICATION_ANSWER"] = "PARTIAL"
                s[f"done_{step}"] = "NO"
                s["final_answer"] = f"{caveat}\n\n{draft_text}" if draft_text else caveat
                break
            
            # Update current task for next iteration
            if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a", "blank", "(blank)"]:
                current_task = next_task_raw
                logger.info(f"Step {step}: Next task: {current_task}")
            else:
                logger.warning(f"Step {step}: No NEXT_TASK provided, stopping")
                break

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
        s["last_eval_target"] = last_eval_target
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


def _build_compressed_prompt(full_prompt: str, keep_last_n_steps: int = 2) -> str:
    """
    Build a token-efficient prompt by compressing earlier conversation turns
    while keeping the system prefix and recent steps in full.

    Structure: [full system prefix] + [compressed earlier trace] + [full last N steps]

    Args:
        full_prompt: The full accumulated prompt string with role markers
        keep_last_n_steps: Number of recent Stage 2/3 step pairs to keep uncompressed

    Returns:
        Compressed prompt string with same role-marker format
    """
    if not full_prompt:
        return full_prompt

    # Split prompt into system prefix and conversation turns
    # Find the end of the system block (first <|user|> after <|system|>)
    first_user_idx = full_prompt.find("<|user|>")
    if first_user_idx == -1:
        return full_prompt  # No conversation turns yet

    system_prefix = full_prompt[:first_user_idx]
    conversation = full_prompt[first_user_idx:]

    # Find Stage 2 boundaries to identify step pairs
    # Each step consists of: Stage 2 user msg + assistant code + Stage 3 user msg + assistant reflection
    stage2_pattern = re.compile(r'<\|user\|>\nSTAGE 2 \(step (\d+)/(\d+)\):')
    step_positions = [(m.start(), int(m.group(1))) for m in stage2_pattern.finditer(conversation)]

    if len(step_positions) <= keep_last_n_steps:
        return full_prompt  # Not enough steps to compress

    # Split: pre-steps (Stage 1 + format instructions) | compressible steps | recent steps
    first_step_pos = step_positions[0][0]
    compress_boundary = step_positions[-keep_last_n_steps][0]

    pre_steps = conversation[:first_step_pos]  # Stage 1 exchanges + format instructions
    compressible = conversation[first_step_pos:compress_boundary]
    recent_steps = conversation[compress_boundary:]

    # Compress the earlier steps using existing _compress_trace
    compressed = _compress_trace(compressible)

    if not compressed.strip():
        return full_prompt  # Compression produced nothing, use full prompt

    # Rebuild: system + pre-steps + compressed trace as user msg + recent steps
    result = system_prefix + pre_steps
    result += format_user(f"[COMPRESSED EXECUTION TRACE — earlier steps]\n{compressed}\n[END COMPRESSED TRACE]")
    result += format_assistant("Understood. Continuing from the compressed trace.\n")
    result += recent_steps

    return result


def vllm_gen(slot_name: str, prompt: str, state: Dict[str, Any], max_tokens: int,
             temperature: float, stop: Any = None, executor: InfospaceExecutor = None,
             llm_prompt: str = None, reasoning_effort: str = None) -> str:
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
        llm_prompt: Optional override prompt to send to the LLM instead of prompt.
                    When set, this is what the LLM sees, while prompt remains the
                    canonical full trace for bookkeeping.

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
    
    effective_prompt = llm_prompt if llm_prompt is not None else prompt
    messages = _parse_prompt_to_messages(effective_prompt)

    # Use unified llm_generate interface
    try:
        response = executor.llm_generate(
            messages=messages,
            max_tokens=max_tokens+256,  # allow for reasoning
            temperature=temperature,
            stops=stop_list if stop_list else None,
            reasoning_effort=reasoning_effort
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

        text = _strip_think_tags(text)
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
                                 model: str = None, vision_criteria: str = "", output_artifacts: List[str] = None, resolved_output_artifacts: List[str] = None, output_artifact_names: Dict[str, str] = None):
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
        # Strip think tags before line extraction (think blocks may span multiple lines)
        remainder = _strip_think_tags(remainder)
        remainder = remainder.lstrip()
        line = remainder.splitlines()[0] if remainder.splitlines() else remainder
        return _strip_numbered_prefix(line.strip())

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

    # Extract output sizing guidance from recent_context (if present)
    _output_guidance_line = ""
    if "OUTPUT GUIDANCE" in recent_context and "target_tokens:" in recent_context:
        import re as _re
        _m = _re.search(r'target_tokens:\s*(\d+)', recent_context)
        if _m:
            _output_guidance_line = f"OUTPUT SIZING: pass target_tokens={_m.group(1)} to the tool call that produces the FINAL output artifact.\n"

    # Stage 0: Resource retrieval with envisionment-driven queries
    available_resources_text = ""
    if executor:
        try:
            available_resources_text = stage0_resource_retrieval(
                goal=goal, executor=executor,
                orient_assessment=getattr(executor, '_orient_assessment', None),
                active_concerns=getattr(executor, '_user_concerns_snapshot', None),
            )
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
        system_parts.append(f"\n## ABSTRACT_PLAN\n{preplan}\n## End ABSTRACT_PLAN\n")

    if vision_criteria:
        system_parts.append(f"\n## QUALITY VISION (external — checked by the system, NOT by your code)\n"
            f"These criteria are evaluated externally at the done gate. Do NOT implement them as checks in your code blocks.\n"
            f"They are shown here so you understand what quality looks like — use them to guide your approach, not to build verification loops.\n"
            f"{vision_criteria}\n## End QUALITY VISION\n")

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
    
    # Compact world model: only fact text + confidence (booking fields are noise)
    wm_facts = [{"fact": f["fact"], "confidence": f.get("confidence", "?")}
                 for f in world_model.get("facts", []) if f.get("fact")]
    wm_contracts = world_model.get("tool_contracts", [])
    wm_lines = ["WORLD_MODEL:"]
    if wm_facts:
        wm_lines.append("Facts:")
        for f in wm_facts:
            wm_lines.append(f"- [{f['confidence']}] {f['fact']}")
    if wm_contracts:
        wm_lines.append(f"Tool contracts: {json.dumps(wm_contracts)}")
    system_parts.append("\n".join(wm_lines) + "\n")

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
        "   Consult WORLD_MODEL tool_contracts for known tool capabilities and constraints.\n"
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
                _exec_actions = set(executor._get_known_action_names()) if hasattr(executor, '_get_known_action_names') else set()
                expanded_docs = load_skill_docs(tools_to_load, executor.available_tools, _exec_actions)
                if expanded_docs:
                    prompt += format_user(expanded_docs)
                    prompt += format_assistant("I have reviewed the detailed documentation for the selected tools.\n")
                    logger.info(f"Stage 1.5: Injected detailed docs for {len(tools_to_load)} tools")
                    _loaded_skill_docs.update(tools_to_load)
            else:
                logger.debug(f"Stage 1.5: All {len(selected_tools)} selected tools already have docs loaded, skipping")
    except (KeyError, json.JSONDecodeError, TypeError) as e:
        logger.warning(f"Failed to parse selected tools for doc expansion: {e}")
    
    # Snapshot planner reasoning state for generate-reflective-note tool
    if executor:
        try:
            base_state = prompt
            # Append OODA orientation snapshot
            try:
                from ooda_snapshot_renderer import render_reflective_snapshot
                _ls = getattr(executor, '_ooda_living_state', None)
                _dc = getattr(executor, '_derived_concerns_snapshot', [])
                _uc = getattr(executor, '_user_concerns_snapshot', [])
                snapshot = render_reflective_snapshot(_ls, _dc, _uc)
                if snapshot:
                    base_state += f"\n\n{snapshot}"
            except Exception:
                pass
            # Append operational self-model section
            try:
                from ooda_snapshot_renderer import render_self_model_section
                self_model = render_self_model_section(
                    scheduler_status=_safe_get_scheduler_status(executor),
                    tasks=_safe_get_all_tasks(executor),
                    derived_concerns=_dc,
                    scheduled_goals=_safe_get_scheduled_goals(executor),
                    sensor_configs=_safe_get_sensor_configs(executor),
                    execution_mode=_safe_get_execution_mode(executor),
                    tool_count=len(getattr(executor, 'available_tools', {})),
                )
                if self_model:
                    base_state += f"\n\n{self_model}"
            except Exception:
                pass
            executor._planner_reflective_state = base_state
        except Exception:
            executor._planner_reflective_state = ""

    # Interrupt checkpoint: after Stage 1, before step loop
    if executor and _interrupt_requested(executor):
        _clear_interrupt(executor)
        state["final_answer"] = "Interrupted by user."
        return state

    # Stage 2/3 format instructions
    prompt += format_user(
        "#Stage 2 FORMAT:\n"
        "Write a Python code block to accomplish the CURRENT_TASK.\n"
        "Linear example:\n"
        "```python\n"
        "r1 = tool(\"search-web\", query=\"transformers survey\", out=\"$papers\")\n"
        "if r1[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=f\"search failed: {r1.get('reason', 'unknown')}\")\n"
        "r2 = tool(\"synthesize\", target=\"$papers\", focus=\"key findings\", out=\"$summary\")\n"
        "if r2[\"status\"] != \"success\": return executor._create_uniform_return(\"failed\", reason=f\"synthesize failed: {r2.get('reason', 'unknown')}\")\n"
        "return executor._create_uniform_return(\"success\", value=\"done\")\n"
        "```\n"
        "Reading content example:\n"
        "```python\n"
        "# To read content from a bound $var, use get_text/get_json/get_items directly.\n"
        "# NO need to call load first — these read straight from the resource store.\n"
        "text = get_text(\"$my_note\")       # returns Note content as string\n"
        "data = get_json(\"$my_note\")       # returns Note content parsed as dict (or None)\n"
        "ids  = get_items(\"$my_collection\") # returns list of Note IDs in the Collection\n"
        "```\n"
        "Rules:\n"
        "- Call tools via: r = tool(\"tool-name\", param=value, out=\"$var\")\n"
        "- tool() returns dict with r[\"status\"] (\"success\"/\"failed\"), r[\"resource_id\"], r[\"value\"] (display string).\n"
        "- Max 16 tool() calls per code block.\n"
        "- Prefer longer cohesive blocks: if CURRENT_TASK has connected subtasks, combine them in one block.\n"
        "- out=\"$name\" binds the result resource. Chain via target=\"$name\" in the next call.\n"
        "- Read content: get_text(\"$var\") → string, get_json(\"$var\") → dict or None, get_items(\"$var\") → list of Note IDs.\n"
        "  These work on any $var that has been bound. No need to call load first.\n"
        "- load: ONLY for (a) binding a named persistent note, or (b) slicing a Collection.\n"
        "- Pass dicts/lists directly as value — do not pre-serialize with json.dumps().\n"
        "- CODE BLOCK SCOPE: each step executes in a fresh function. Python locals do NOT persist across steps.\n"
        "  Only $bindings (created via out=) persist. Use get_text/get_json/get_items for cross-step access.\n"
        "- if/else control flow and loops are allowed.\n"
        "- Must end with: return executor._create_uniform_return(status, value=..., extra=...)\n"
        "- LOOP PATTERN: tool() never raises — do NOT use try/except. Track ok/errors counts explicitly.\n"
        "- OUTPUT SIZING: If OUTPUT GUIDANCE appears in the context with a target_tokens value,\n"
        "  pass target_tokens=<N> to the tool call that produces the FINAL output artifact of the goal\n"
        "  (the primary product — e.g. the last synthesize, generate-note, or extract call).\n"
        "  Do NOT apply target_tokens to intermediate/preparatory tool calls.\n"
        "\n"
        "#Stage 3 FORMAT:\n"
        "  THOUGHTS: <brief assessment of result and progress>\n"
        "  EVAL_TARGET: <single $variable (e.g. $report) of the key output artifact from this step, or NONE if no significant artifact>\n"
        "  DONE: <YES or NO — is the entire GOAL satisfied?>\n"
        "  VERIFICATION: <if DONE=YES: list observable facts proving GOAL is met, and any missing outcomes. Write SUCCESS, PARTIAL, or INCONCLUSIVE. If DONE=NO, leave blank.>\n"
        "  NEXT_TASK: <next task description, or blank if DONE=YES>\n"
        "  REQUEST_TOOLS: <json array of tool names needing docs, or []>\n"
        "  ASK_USER_NEEDED: <YES or NO — do you need to ask the user a question to continue?>\n"
        "\n"
        "#Stage 3 RULES:\n"
        "- DONE=YES only when ALL required actions are complete.\n"
        "- If DONE=YES, NEXT_TASK must be blank and VERIFICATION must be filled.\n"
        "- Use 'say' in code blocks when communication is part of the goal (e.g., reporting results, echoing data). Use 'ask' when you need the user's response to continue.\n"
        "- Efficiency rule: if prior step used only one tool call and GOAL is not done, set NEXT_TASK to combine adjacent subtasks.\n"
        "- SPIRAL DETECTION: If the same tool has failed 2+ times consecutively,\n"
        "  use a different approach or proceed with available data.\n"
        "- ASK_USER_NEEDED: Answer YES only when you genuinely lack information the user has — wrong target,\n"
        "  ambiguous goal, repeated failure you cannot diagnose, or a preference you cannot resolve.\n"
        "  Always try at least once before asking. At most one ASK per goal.\n"
        "  Do NOT use ASK_USER_NEEDED to show your next planned code block — just execute it.\n"
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
    _declared_output_artifacts = _normalize_artifact_refs(output_artifacts)
    _resolved_output_artifacts = _normalize_artifact_refs(resolved_output_artifacts)
    _output_artifact_names = _normalize_artifact_name_map(output_artifact_names)
    last_eval_target = _select_primary_artifact_id(
        executor,
        declared_output_artifacts=_declared_output_artifacts,
        resolved_output_artifacts=_resolved_output_artifacts,
        output_artifact_names=_output_artifact_names,
    )
    if last_eval_target:
        logger.info(f"Eval target seeded from explicit artifact context: {last_eval_target}")
    stall_guard_state = {"prev_signature": None, "repeat_count": 0}
    deep_eval_retried = False
    deep_eval_prev_artifact = None
    plan_local_bindings = set()
    _asked_user_this_goal = False
    _mid_vision_eval_counts = {}  # {eval_target_id: count} — suppress after cap
    _MID_VISION_EVAL_CAP = 2     # max evals per artifact before suppressing
    for step in range(max_steps):
        if _interrupt_requested(executor):
            _clear_interrupt(executor)
            state["final_answer"] = "Interrupted by user."
            break
        
        # Stage 2: Generate code block
        _bindings_line = _build_bindings_inventory(plan_local_bindings, executor) if step > 0 else ""
        prompt += format_user(
            f"STAGE 2 (step {step + 1}/{max_steps}):\n"
            f"#GOAL: {goal_for_step}\n#END GOAL\n"
            f"{_bindings_line}"
            f"CURRENT_TASK: {current_task}\n"
            f"{_output_guidance_line}"
            "Write a Python code block using Stage 2 FORMAT.\n"
        )

        prompt += format_assistant("```python\n")
        # Use compressed prompt for LLM call to reduce token count on later steps
        compressed_for_llm = _build_compressed_prompt(prompt, keep_last_n_steps=2) if step >= 3 else None
        code_raw = vllm_gen(f"code_block_{step}", prompt, state, max_tokens=2048, temperature=GEN_TEMPERATURE, stop=["\n```"], executor=executor, llm_prompt=compressed_for_llm, reasoning_effort="medium")
        prompt += code_raw + "\n```\n"
        # Strip echoed code fences if model includes them
        code_text = code_raw.strip()
        import re as _re
        code_text = _re.sub(r'^```(?:python)?\s*', '', code_text)
        code_text = _re.sub(r'`{1,3}\s*$', '', code_text).rstrip()
        
        logger.info(f"Step {step}: Code block ({len(code_text)} chars):\n{code_text}")
        call_count = count_codegen_action_calls(code_text)
        logger.info(f"Step {step}: Code block action calls={call_count} (logged before execution)")
        
        # Interrupt checkpoint: after code-block gen, before execute
        if _interrupt_requested(executor):
            _clear_interrupt(executor)
            state["final_answer"] = "Interrupted by user."
            break

        # Set task context for tool exemplar recording
        executor._current_task_context = current_task

        result_dict, new_bindings, tool_result, code_block_output_created = _execute_and_record_code_block(code_text, executor, step)

        logger.info(f"Step {step}: -> {tool_result[:100]}")
        if new_bindings:
            plan_local_bindings.update(new_bindings.keys())
            logger.info(f"Step {step}: New bindings: {new_bindings}")
        
        # Interrupt checkpoint: after code-block execution
        if _interrupt_requested(executor):
            _clear_interrupt(executor)
            # Ask to User intentionally sets interrupt to pause for response - treat as success, not failure
            plan_actions = getattr(executor, '_plan_actions', [])
            last_ask = plan_actions and plan_actions[-1].get('type') == 'ask' and str(plan_actions[-1].get('target', '')).lower() == 'user'
            if last_ask:
                state["ask_completed_successfully"] = True
            state["final_answer"] = "Interrupted by user."
            break
        
        # Stage 3: Evaluate result
        exemplar_hint = _get_exemplar_hint_for_failure(tool_result, executor)
        prompt += format_user(
            f"=====\n"
            f"STAGE 3 (step {step + 1}/{max_steps})\n"
            f"=====\n\n"
            f">> RESULT (ground truth) <<\n"
            f"{tool_result}\n"
            f">> END RESULT <<\n\n"
            f"{exemplar_hint}"
            f"Evaluate: Is the GOAL fully achieved? Use ONLY the result above as ground truth.\n"
            f"Respond using Stage 3 FORMAT.\n"
        )
        
        # Vision evaluation before planner reflects — resolve to concrete resource ID
        eval_target = _select_primary_artifact_id(
            executor,
            declared_output_artifacts=_declared_output_artifacts,
            resolved_output_artifacts=_resolved_output_artifacts,
            output_artifact_names=_output_artifact_names,
            code_block_output_created=code_block_output_created,
        )
        # Fallback: if no declared/resolved artifact, use the last out= binding
        if not eval_target and new_bindings:
            last_var, last_rid = list(new_bindings.items())[-1]
            if isinstance(last_rid, str) and (last_rid.startswith('Note_') or last_rid.startswith('Collection_')):
                eval_target = last_rid
                logger.info(f"Step {step}: Eval target from last binding: ${last_var} → {last_rid}")
        if eval_target:
            last_eval_target = eval_target
        # If a side-effect tool ran, prefer its input content as eval target
        side_effect_content = _find_side_effect_content_source(executor)
        if side_effect_content:
            last_eval_target = side_effect_content
            logger.info(f"Step {step}: Eval target updated to side-effect content source: {side_effect_content}")
        if eval_target and vision_criteria:
            # Enforce per-artifact eval cap to prevent loops on unfixable criteria
            mid_key = _resolve_eval_target_id(last_eval_target, executor) or last_eval_target
            _mid_vision_eval_counts[mid_key] = _mid_vision_eval_counts.get(mid_key, 0) + 1
            if _mid_vision_eval_counts[mid_key] <= _MID_VISION_EVAL_CAP:
                logger.info(f"Step {step}: Vision eval target: {last_eval_target} (eval #{_mid_vision_eval_counts[mid_key]})")
                compressed_ctx = _compress_trace(prompt)
                vision_eval_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)
                if vision_eval_text:
                    prompt += format_user(f"VISION EVALUATION:\n{vision_eval_text}\nFAILs are advisory — attempt to address if easy, do NOT loop on unfixable criteria.\n")
                    prompt += format_assistant("Noted.\n")
            else:
                logger.info(f"Step {step}: Vision eval suppressed for {last_eval_target} (eval cap {_MID_VISION_EVAL_CAP} reached)")

        # Stage 3: simplified reflection (5 fields + inline verification when DONE)
        prompt += format_assistant("")
        compressed_for_llm = _build_compressed_prompt(prompt, keep_last_n_steps=2) if step >= 3 else None
        stage3_block = vllm_gen(f"stage3_block_{step}", prompt, state, max_tokens=384, temperature=GEN_TEMPERATURE, executor=executor, llm_prompt=compressed_for_llm)
        prompt += stage3_block + "\n"

        thoughts_val = _strip_think_tags(_extract_between_labels(stage3_block, "THOUGHTS:", "EVAL_TARGET:"))
        eval_target_val = _strip_think_tags(_extract_line_value(stage3_block, "EVAL_TARGET:"))
        done_val = _strip_think_tags(_extract_line_value(stage3_block, "DONE:"))
        verification_val = _strip_think_tags(_extract_between_labels(stage3_block, "VERIFICATION:", "NEXT_TASK:"))
        next_task_val = _strip_numbered_prefix(_strip_think_tags(_extract_between_labels(stage3_block, "NEXT_TASK:", "REQUEST_TOOLS:")))
        request_tools_val = _strip_numbered_prefix(_strip_think_tags(_extract_between_labels(stage3_block, "REQUEST_TOOLS:", "ASK_USER_NEEDED:") or _extract_after_label(stage3_block, "REQUEST_TOOLS:")).strip())
        ask_user_needed_val = _strip_think_tags(_extract_line_value(stage3_block, "ASK_USER_NEEDED:")).strip().upper()
        ask_user_val = ""  # Will be populated by follow-up call if needed

        # JSON fallback: some models output Stage 3 as JSON instead of label format
        if not done_val:
            try:
                import json as _json
                stage3_json = _json.loads(_extract_braced_json_object(stage3_block))
                if isinstance(stage3_json, dict):
                    thoughts_val = thoughts_val or str(stage3_json.get("thoughts", ""))
                    eval_target_val = eval_target_val or str(stage3_json.get("eval_target", ""))
                    done_raw_json = stage3_json.get("done", "")
                    # Normalize JSON booleans: true → "YES", false → "NO"
                    if done_raw_json is True:
                        done_val = "YES"
                    elif done_raw_json is False:
                        done_val = "NO"
                    else:
                        done_val = str(done_raw_json)
                    verification_val = verification_val or str(stage3_json.get("verification", ""))
                    next_task_val = next_task_val or str(stage3_json.get("next_task", ""))
                    rt = stage3_json.get("request_tools", [])
                    if isinstance(rt, list):
                        request_tools_val = request_tools_val or _json.dumps(rt)
                    else:
                        request_tools_val = request_tools_val or str(rt)
                    ask_user_needed_raw = stage3_json.get("ask_user_needed", stage3_json.get("ask_user", ""))
                    if isinstance(ask_user_needed_raw, bool):
                        ask_user_needed_val = "YES" if ask_user_needed_raw else "NO"
                    else:
                        ask_user_needed_val = ask_user_needed_val or str(ask_user_needed_raw).strip().upper()
                    logger.info(f"Step {step}: Stage 3 parsed via JSON fallback (DONE={done_val})")
            except (ValueError, KeyError, TypeError):
                pass

        state[f"thoughts_{step}"] = thoughts_val
        state[f"eval_target_{step}"] = eval_target_val
        state[f"done_{step}"] = done_val
        state[f"verification_{step}"] = verification_val
        state[f"next_task_{step}"] = next_task_val
        state[f"request_tools_{step}"] = _strip_code_fences(request_tools_val)
        state[f"ask_user_needed_{step}"] = ask_user_needed_val
        state[f"ask_user_{step}"] = ask_user_val

        def safe_get(state_dict, key, default='N/A'):
            try:
                return state_dict[key]
            except (KeyError, TypeError, AttributeError):
                return default
        
        logger.info(f"THOUGHTS: {safe_get(state, f'thoughts_{step}')}")
        logger.info(f"EVAL_TARGET: {safe_get(state, f'eval_target_{step}')}")
        logger.info(f"DONE: {safe_get(state, f'done_{step}')}")
        if verification_val:
            logger.info(f"VERIFICATION: {verification_val}")
        logger.info(f"NEXT_TASK: {safe_get(state, f'next_task_{step}')}")
        logger.info(f"REQUEST_TOOLS: {safe_get(state, f'request_tools_{step}')}")
        if ask_user_needed_val.startswith("YES"):
            logger.info(f"ASK_USER_NEEDED: YES (will generate question in follow-up call)")

        llm_eval_target_raw = state.get(f"eval_target_{step}", "").strip()
        if llm_eval_target_raw and llm_eval_target_raw.upper() != "NONE":
            llm_rid = _resolve_artifact_reference_id(
                llm_eval_target_raw,
                executor,
                output_artifact_names=_output_artifact_names,
            )
            if llm_rid and _allow_llm_eval_target_override(
                llm_eval_target_raw,
                llm_rid,
                executor,
                declared_output_artifacts=_declared_output_artifacts,
                resolved_output_artifacts=_resolved_output_artifacts,
                output_artifact_names=_output_artifact_names,
            ):
                last_eval_target = llm_rid
                logger.info(f"Step {step}: LLM-declared eval target resolved: {last_eval_target}")
        
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
                _exec_actions = set(executor._get_known_action_names()) if hasattr(executor, '_get_known_action_names') else set()
                expanded_docs = load_skill_docs(tools_to_load, executor.available_tools, _exec_actions)
                if expanded_docs:
                    prompt += format_user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                    prompt += format_assistant("I have reviewed the additional tool documentation.\n")
                    logger.info(f"Stage 3.5: Loaded docs for {len(tools_to_load)} additional tools: {tools_to_load}")
                    _loaded_skill_docs.update(tools_to_load)
            else:
                logger.debug(f"Stage 3.5: All requested tools already have docs loaded, skipping")
        elif requested_tools_raw and requested_tools_raw.lower() not in ["", "[]", "none", "null"]:
            logger.warning(f"Step {step}: Failed to parse REQUEST_TOOLS: {requested_tools_raw[:100]}")

        # Stage 3.6: ASK_USER — two-stage: check if needed (YES/NO), then get question text
        ask_user_needed = state.get(f"ask_user_needed_{step}", "").strip().upper()
        if ask_user_needed.startswith("YES") and not _asked_user_this_goal:
            # Stage 2: Follow-up LLM call to get the actual question text
            ask_gen_prompt = prompt + format_user(
                "You indicated ASK_USER_NEEDED: YES. Write a concise question for the user.\n"
                "Include: what you tried, why you're stuck, and 2-3 options if possible.\n"
                "Respond with ONLY the question text, nothing else.\n</end>"
            )
            try:
                ask_messages = _parse_prompt_to_messages(ask_gen_prompt)
                ask_resp = executor.llm_generate(
                    messages=ask_messages, max_tokens=712, temperature=0.3, stops=['</end>'])
                ask_question = _strip_think_tags(ask_resp.text).strip() if ask_resp.success and ask_resp.text else ""
            except Exception as e:
                logger.warning(f"Step {step}: ASK_USER question generation failed: {e}")
                ask_question = ""

            if ask_question and ask_question.lower() not in ("", "none", "null", "n/a"):
                _asked_user_this_goal = True
                state[f"ask_user_{step}"] = ask_question
                logger.info(f"Step {step}: ASK_USER triggered — pausing for user input")
                ask_result = executor.execute_action_tracked(
                    {"type": "ask", "value": ask_question, "out": "$_user_guidance"},
                    "codegen"
                )
                if ask_result.get("status") == "success":
                    user_response = ask_result.get("value", "")
                    prompt += format_user(f"USER_GUIDANCE (response to your question): {user_response}\nIncorporate this guidance into your next step.")
                    prompt += format_assistant("Understood, I will incorporate the user's guidance.\n")
                    logger.info(f"Step {step}: User responded to ASK_USER: {user_response[:200]}")
                else:
                    prompt += format_user("ASK_USER timed out or was interrupted. Proceed with your best judgment.")
                    prompt += format_assistant("Understood, proceeding without user input.\n")
                    logger.info(f"Step {step}: ASK_USER failed or timed out — continuing")
            else:
                logger.info(f"Step {step}: ASK_USER_NEEDED=YES but question generation returned empty — skipping")

        # Check if done
        done_raw = _strip_numbered_prefix(state.get(f"done_{step}", "").strip()).upper()
        next_task_raw = _strip_numbered_prefix(state.get(f"next_task_{step}", "").strip())
        if done_raw.startswith("YES"):
            if _interrupt_requested(executor):
                _clear_interrupt(executor)
                state["final_answer"] = "Interrupted by user."
                break

            # Use inline verification from Stage 3 (no separate LLM call)
            inline_verification = state.get(f"verification_{step}", "").strip()
            if inline_verification:
                # Parse verification status from inline field
                cleaned = inline_verification.upper()
                if "SUCCESS" in cleaned:
                    cleaned = "SUCCESS"
                elif "PARTIAL" in cleaned:
                    cleaned = "PARTIAL"
                elif "INCONCLUSIVE" in cleaned:
                    cleaned = "INCONCLUSIVE"
                else:
                    cleaned = "SUCCESS"  # Default if DONE=YES with verification text
                state["VERIFICATION_ANSWER"] = cleaned
                logger.info(f"VERIFICATION (inline): {cleaned} — {inline_verification[:200]}")
            else:
                # Fallback: lightweight verification without full system prefix
                lightweight_verify_prompt = format_system(
                    f"You are verifying whether a goal has been achieved.\n"
                    f"#GOAL:\n{goal_for_step}\n#END GOAL\n"
                )
                compressed_trace = _compress_trace(prompt)
                lightweight_verify_prompt += format_user(
                    f"Execution trace:\n{compressed_trace}\n\n"
                    f"What observable facts in the current state satisfy the GOAL?\n"
                    f"What required actions or outcomes are missing?\n"
                    f"VERIFICATION_ANSWER: <SUCCESS | PARTIAL | INCONCLUSIVE>\n"
                )
                lightweight_verify_prompt += format_assistant("VERIFICATION_ANSWER: ")
                verification_raw = vllm_gen("VERIFICATION_ANSWER", prompt, state, max_tokens=96, temperature=GEN_TEMPERATURE, stop="\n", executor=executor, llm_prompt=lightweight_verify_prompt)
                vlines = [ln.strip() for ln in str(verification_raw).splitlines() if ln.strip()]
                cleaned = vlines[-1] if vlines else str(verification_raw).strip()
                if cleaned.startswith("VERIFICATION_ANSWER:"):
                    cleaned = cleaned[len("VERIFICATION_ANSWER:"):].strip()
                state["VERIFICATION_ANSWER"] = cleaned
                prompt += format_user("VERIFICATION_ANSWER: " + cleaned)
                prompt += format_assistant("Noted.\n")
                logger.info(f"VERIFICATION (lightweight fallback): {cleaned}")
            
            # Deep vision evaluation at done gate
            if vision_criteria and last_eval_target:
                logger.info(f"Done gate: Running deep vision eval on {last_eval_target}")
                compressed_ctx = _compress_trace(prompt)
                shallow_text = _vision_eval_check(vision_criteria, last_eval_target, executor, compressed_context=compressed_ctx)

                deep_text = _vision_eval_deep(vision_criteria, last_eval_target, shallow_text or "", executor, compressed_context=compressed_ctx, plan_bindings=plan_local_bindings)
                if deep_text:
                    prompt += format_user(f"DEEP VISION EVALUATION (final quality gate, read-only):\n{deep_text}\nUse this only as QA signal. Do NOT run additional tool steps at done gate.\n")
                    prompt += format_assistant("Noted.\n")
                    logger.info(f"Done gate: Deep eval result injected:\n{deep_text}")
                    deep_status = _parse_deep_eval_status(deep_text)
                    if deep_status == "needs_revision":
                        if not deep_eval_retried:
                            deep_eval_retried = True
                            deep_eval_prev_artifact = _resolve_eval_target_text(last_eval_target, executor)
                            logger.info("Done gate: Deep eval returned NEEDS_REVISION; reopening loop for one retry")
                            if hasattr(executor, "_done_gate_retry_active"):
                                executor._done_gate_retry_active = True
                            ask_nudge = ""
                            if not _asked_user_this_goal:
                                ask_nudge = (
                                    "\nIf you lack the information needed to fix these issues (e.g., the source is "
                                    "inaccessible, the goal is ambiguous, or you need the user's input to proceed), "
                                    "set ASK_USER_NEEDED: YES in your next Stage 3 instead of retrying the same approach."
                                )
                            prompt += format_user(
                                f"QUALITY GATE FAILED — you must revise the artifact in {last_eval_target}.\n"
                                f"Issues found:\n{deep_text}\n\n"
                                f"Regenerate or fix the artifact to address the FAILed criteria, then mark DONE again."
                                f"{ask_nudge}"
                            )
                            prompt += format_assistant("Understood. I will revise the artifact to address the failed criteria.\n")
                            state[f"done_{step}"] = "NO"
                            continue
                        else:
                            retry_artifact = _resolve_eval_target_text(last_eval_target, executor)
                            if deep_eval_prev_artifact and retry_artifact == deep_eval_prev_artifact:
                                logger.info("Done gate: Deep eval NEEDS_REVISION on retry but artifact unchanged (tool limitation); accepting as satisfied")
                            else:
                                logger.info("Done gate: Deep eval returned NEEDS_REVISION on retry; freezing artifact")
                                draft_text = retry_artifact
                                caveat = (
                                    "Quality gate warning: deep evaluation found issues after retry (STATUS: NEEDS_REVISION). "
                                    "Delivering best available draft from this cycle."
                                )
                                state["VERIFICATION_ANSWER"] = "PARTIAL"
                                state[f"done_{step}"] = "NO"
                                state["final_answer"] = f"{caveat}\n\n{draft_text}" if draft_text else caveat
                                break
                if shallow_text and "fail" in shallow_text.lower():
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
            # Use compressed prompt for final answer generation
            compressed_final = _build_compressed_prompt(prompt, keep_last_n_steps=2) if step >= 3 else None
            final_answer = vllm_gen("final_answer", prompt, state, max_tokens=256, temperature=GEN_TEMPERATURE, stop=["\n\n", "STAGE"], executor=executor, llm_prompt=compressed_final)
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
            # If we haven't nudged yet, give one more step with ASK_USER hint
            if not stall_guard_state.get("nudged"):
                prompt += format_user(
                    "STALL DETECTED: You are repeating the same approach without progress. "
                    "Consider setting ASK_USER_NEEDED: YES in your next Stage 3 to get guidance from the user, "
                    "or try a fundamentally different approach. One more step allowed."
                )
                prompt += format_assistant("Understood, I will reassess.\n")
                logger.info(f"Step {step}: Stall guard triggered — nudging ASK_USER before giving up")
                # Reset stall guard so the nudge step gets a clean pass
                stall_guard_state["prev_signature"] = None
                stall_guard_state["repeat_count"] = 0
                stall_guard_state["nudged"] = True  # Don't nudge again — next stall triggers normal bailout
                continue  # Give one more iteration
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
            break

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

    state["last_eval_target"] = last_eval_target
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

    # Strip numbered list prefix (e.g. "1. []" → "[]")
    m = re.match(r'^\d+\.\s+', text)
    if m:
        text = text[m.end():]

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
                openrouter_model_path: str = None, anthropic_model_path: str = None,
                openai_model_path: str = None):
        """
        Initialize incremental planner.
        
        Args:
            executor: InfospaceExecutor instance
            available_tools: Dict of tool_name -> metadata
            primitives_reference: Primitives reference text
            logger_instance: Optional logger
            sgl_model_path: Path to local model for SGLang (optional if vLLM/OpenRouter/OpenAI/Anthropic is used)
            vllm_model_path: Path to model for vLLM (optional if SGLang/OpenRouter/OpenAI/Anthropic is used)
            vllm_url: vLLM API endpoint (default: http://localhost:5000/v1/chat/completions)
            vllm_model: vLLM model name (resolved from vLLM server if not provided)
            openrouter_model_path: Model name for OpenRouter (optional if SGLang/vLLM/OpenAI/Anthropic is used)
            anthropic_model_path: Model name for Anthropic API (optional if SGLang/vLLM/OpenAI/OpenRouter is used)
            openai_model_path: Model name for OpenAI API (optional if SGLang/vLLM/OpenRouter/Anthropic is used)
        """
        # Require either SGLang, vLLM, OpenRouter, OpenAI, or Anthropic
        if not HAS_SGLANG and not vllm_model_path and not openrouter_model_path and not anthropic_model_path and not openai_model_path:
            raise ImportError("Neither SGLang, vLLM, OpenRouter, OpenAI, nor Anthropic available - at least one backend required")
        
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
        
        # Store OpenAI config
        self.openai_model_path = openai_model_path
        
        # SGLang runtime is now initialized in executive_node (optional)
        # Verify availability if SGLang is expected
        if sgl_model_path and not executor.runtime:
            self.logger.warning("SGLang runtime not available in executor - will use vLLM/OpenRouter if configured")
        elif not vllm_model_path and not openrouter_model_path and not anthropic_model_path and not openai_model_path and not executor.runtime:
            self.logger.warning("Neither SGLang runtime nor vLLM/OpenRouter/OpenAI config available - incremental planner may have reduced functionality")
        
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
    
    def _run_tool_planner_backend(
        self,
        template,
        goal: str,
        world_model: Dict,
        character_context: str,
        recent_context: str,
        max_steps: int,
        preplan: str,
        similar_plan: Optional[Dict] = None,
        vision_criteria: str = "",
        output_artifacts: Optional[List[str]] = None,
        resolved_output_artifacts: Optional[List[str]] = None,
        output_artifact_names: Optional[Dict[str, str]] = None,
    ):
        """Run the core tool planner using any configured backend."""
        if (
            (self.vllm_model_path and self.vllm_model) or
            (self.openrouter_model_path and self.executor.openrouter_model) or
            (self.anthropic_model_path and self.executor.anthropic_model) or
            (self.openai_model_path and self.executor.openai_model)
        ):
            return tool_planner_infospace_vllm(
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
                similar_plan=similar_plan,
                vllm_url=self.vllm_url,
                model=self.vllm_model,
                vision_criteria=vision_criteria,
                output_artifacts=output_artifacts or [],
                resolved_output_artifacts=resolved_output_artifacts or [],
                output_artifact_names=output_artifact_names or {},
            )
        if HAS_SGLANG and self.executor.runtime:
            return tool_planner_infospace.run(
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
                similar_plan=similar_plan,
                vision_criteria=vision_criteria,
                output_artifacts=output_artifacts or [],
                resolved_output_artifacts=resolved_output_artifacts or [],
                output_artifact_names=output_artifact_names or {},
            )
        raise RuntimeError("No planner backend available (SGLang, vLLM, Anthropic, OpenAI, or OpenRouter required)")

    def switch_llm(self, mode: str, alt_config: dict):
        """Switch planner's LLM routing between primary and alt.

        When switching to 'alt', the planner's model path attributes are set from
        alt_config so that _run_core_planner routes to the correct function
        (tool_planner_infospace for SGLang, tool_planner_infospace_vllm for API backends).
        When switching back to 'primary', the original config is restored.

        Should only be called between goals.
        """
        if mode == 'alt':
            # Save primary config on first switch
            if not hasattr(self, '_primary_config'):
                self._primary_config = {
                    'vllm_model_path': self.vllm_model_path,
                    'vllm_url': self.vllm_url,
                    'vllm_model': self.vllm_model,
                    'openrouter_model_path': self.openrouter_model_path,
                    'anthropic_model_path': self.anthropic_model_path,
                    'openai_model_path': self.openai_model_path,
                }
            # Apply alt config — set the matching backend, clear others
            self.vllm_model_path = alt_config.get('vllm_model_path')
            self.vllm_model = getattr(self.executor, 'alt_vllm_model', None)
            self.vllm_url = getattr(self.executor, 'alt_vllm_url', None)
            self.openrouter_model_path = alt_config.get('openrouter_model_path')
            self.anthropic_model_path = alt_config.get('anthropic_model_path')
            self.openai_model_path = alt_config.get('openai_model_path')
            self.logger.info(f"Planner switched to alt LLM config")
        elif mode == 'primary':
            if hasattr(self, '_primary_config'):
                for k, v in self._primary_config.items():
                    setattr(self, k, v)
                self.logger.info(f"Planner switched to primary LLM config")
        self.executor.switch_llm(mode)

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
        if not HAS_SGLANG and not self.vllm_model_path and not self.openrouter_model_path and not self.anthropic_model_path and not self.openai_model_path:
            return {'error': 'Neither SGLang, vLLM, OpenRouter, OpenAI, nor Anthropic available'}
        
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
            self.executor._side_effect_cache = {}  # Reset dedup cache per planner session
            self.executor._successful_side_effect_results = {}
            self.executor._done_gate_retry_active = False
            self.executor._tool_dedup_cache = {}
            self.goal = goal

            preplan = goal
            # Extract context components
            character_context = ""
            recent_context = ""
            situation_context = ""
            if context:
                character_context = context.get('character_context', '')
                recent_context = context.get('recent_context', '')
                situation_context = context.get('situation_context', '')
            if situation_context:
                recent_context = f"\n# SITUATION AWARENESS\n{situation_context}\n" + recent_context
                
            #
           
            world_model = initial_world_model
            state = self._run_tool_planner_backend(
                template=template,
                goal=goal,
                world_model=world_model,
                character_context=character_context,
                recent_context=recent_context,
                max_steps=max_steps,
                preplan=preplan,
                similar_plan=None,
                vision_criteria="",
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

            # Extract final_answer from ProgramState (use bracket notation)
            try:
                final_answer = state['final_answer']
            except (KeyError, TypeError):
                final_answer = 'Planning completed'
            
            error_count = getattr(self.executor, '_plan_error_count', 0)
            # Extract verification_answer from state (works for both dict and ProgramState)
            try:
                verification_answer = _strip_numbered_prefix(str(state['VERIFICATION_ANSWER']).strip()).upper() if 'VERIFICATION_ANSWER' in state else ""
            except (KeyError, TypeError):
                verification_answer = ""
            # Determine success and quality_status from planner signals
            interrupted = (final_answer == "Interrupted by user.")
            ask_success = False
            try:
                ask_success = bool(state['ask_completed_successfully']) if 'ask_completed_successfully' in state else False
            except (KeyError, TypeError):
                pass

            done_yes = False
            if f'done_{step}' in state:
                try:
                    done_str = str(state[f'done_{step}']).strip()
                    done_yes = _strip_numbered_prefix(done_str).upper().startswith("YES")
                except (KeyError, TypeError, AttributeError):
                    pass

            has_result = bool(final_answer and not interrupted)

            if interrupted:
                success = ask_success
                quality_status = "passed" if ask_success else "interrupted"
            elif done_yes:
                success = True
                quality_status = "needs_revision" if verification_answer == "PARTIAL" else "passed"
            elif has_result and verification_answer == "PARTIAL":
                success = True
                quality_status = "needs_revision"
            else:
                success = False
                quality_status = "failed"

            primary_product = _find_side_effect_content_source(self.executor)
            if not primary_product:
                try:
                    raw_target = state['last_eval_target'] if 'last_eval_target' in state else ''
                    if raw_target:
                        primary_product = _resolve_eval_target_id(raw_target, self.executor) or raw_target
                except (KeyError, TypeError):
                    pass

            return {
                'plan': None,
                'response': final_answer,
                'success': success,
                'quality_status': quality_status,
                'verification_answer': verification_answer,
                'error_count': error_count,
                'primary_product': primary_product,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Incremental planning failed: {e}")
            traceback.print_exc()
            return {"success": False, 'error': str(e)}


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
        if not HAS_SGLANG and not self.vllm_model_path and not self.openrouter_model_path and not self.anthropic_model_path and not self.openai_model_path:
            return {'error': 'Neither SGLang, vLLM, OpenRouter, OpenAI, nor Anthropic available'}
        
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
            self.executor._side_effect_cache = {}  # Reset dedup cache per planner session
            self.goal = goal

            with self.executor.turn_metrics.perf_phase("preplan"):
                preplan = self._preplan(goal)
            with self.executor.turn_metrics.perf_phase("vision"):
                vision_criteria = self._generate_vision(context.get('vision_goal', goal) if context else goal)
            # Extract context components
            character_context = ""
            recent_context = ""
            situation_context = ""
            output_artifacts = []
            resolved_output_artifacts = []
            output_artifact_names = {}
            if context:
                character_context = context.get('character_context', '')
                recent_context = context.get('recent_context', '')
                situation_context = context.get('situation_context', '')
                output_artifacts = context.get('output_artifacts', []) or []
                resolved_output_artifacts = context.get('resolved_output_artifacts', []) or []
                output_artifact_names = context.get('output_artifact_names', {}) or {}
            if situation_context:
                recent_context = f"\n# SITUATION AWARENESS\n{situation_context}\n" + recent_context
            recent_context += self.build_context(goal=goal)

            # Inject output size guidance for primary-product sizing
            output_guidance = context.get('output_guidance') if context else None
            if output_guidance and output_guidance.get('target_tokens'):
                from output_sizing import format_guidance_for_planner
                recent_context += f"\n{format_guidance_for_planner(output_guidance)}\n"
                
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
            
            world_model = initial_world_model
            with self.executor.turn_metrics.perf_phase("plan"):
                state = self._run_tool_planner_backend(
                    template=template,
                    goal=goal,
                    world_model=world_model,
                    character_context=character_context,
                    recent_context=recent_context,
                    max_steps=max_steps,
                    preplan=preplan,
                    similar_plan=similar_plans[0] if similar_plans else None,
                    vision_criteria=vision_criteria,
                    output_artifacts=output_artifacts,
                    resolved_output_artifacts=resolved_output_artifacts,
                    output_artifact_names=output_artifact_names,
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
            compressed_trace = _compress_trace(trace_str)
            with self.executor.turn_metrics.perf_phase("reflect"):
                reflection_frame = self._reflect(goal, world_model, max_steps, compressed_trace)

            # Update world_model in memory (persisted on /save or shutdown, not per-cycle)
            if hasattr(self.executor, 'world_model') and self.executor.world_model:
                self.executor.world_model.update(reflection_frame)
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
                {"seq": seq, "ts": now_iso, "agent": agent_name, "world": world_name, "goal": goal, "reflection_frame": reflection_frame},
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
            # Extract verification_answer from state (works for both dict and ProgramState)
            try:
                verification_answer = _strip_numbered_prefix(str(state['VERIFICATION_ANSWER']).strip()).upper() if 'VERIFICATION_ANSWER' in state else ""
            except (KeyError, TypeError):
                verification_answer = ""
            # Determine success and quality_status from planner signals
            interrupted = (final_answer == "Interrupted by user.")
            ask_success = False
            try:
                ask_success = bool(state['ask_completed_successfully']) if 'ask_completed_successfully' in state else False
            except (KeyError, TypeError):
                pass

            done_yes = False
            if f'done_{step}' in state:
                try:
                    done_str = str(state[f'done_{step}']).strip()
                    done_yes = _strip_numbered_prefix(done_str).upper().startswith("YES")
                except (KeyError, TypeError, AttributeError):
                    pass

            has_result = bool(final_answer and not interrupted)

            if interrupted:
                success = ask_success
                quality_status = "passed" if ask_success else "interrupted"
            elif done_yes:
                success = True
                quality_status = "needs_revision" if verification_answer == "PARTIAL" else "passed"
            elif has_result and verification_answer == "PARTIAL":
                success = True
                quality_status = "needs_revision"
            else:
                success = False
                quality_status = "failed"
            
            # Extract primary product.
            # Strategy: if a side-effect tool (email, post) ran, the primary product
            # is the content it consumed (e.g. the report that was emailed), not the
            # delivery confirmation.  Otherwise fall back to last_eval_target.
            primary_product = _find_side_effect_content_source(self.executor)
            if not primary_product:
                try:
                    raw_target = state['last_eval_target'] if 'last_eval_target' in state else ''
                    if raw_target:
                        primary_product = _resolve_eval_target_id(raw_target, self.executor) or raw_target
                except (KeyError, TypeError):
                    pass

            return {
                'plan': plan_actions,
                'response': final_answer,
                'success': success,
                'quality_status': quality_status,
                'verification_answer': verification_answer,
                'error_count': error_count,
                'primary_product': primary_product,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Incremental planning failed: {e}")
            traceback.print_exc()
            return {"success": False, "error": str(e)}


    def _generate_vision(self, goal_text: str) -> str:
        """Generate evaluable quality criteria from the goal (vision for the target artifact)."""
        VISION_PROMPT = f"""Given the following goal, generate 1-3 criteria that check for CLEAR FAILURE MODES only.

GOAL:
{goal_text}

Instructions:
- Only check for obvious failures: empty output, completely off-topic content, or structurally broken output (e.g., JSON parse error when JSON is required).
- Do NOT generate json_parse_error or JSON-related criteria unless the goal explicitly requires JSON output.
- For run-script, shell scripts, or fire-and-forget tools, output is typically plain text — do NOT assume JSON.
- For simple execution goals (e.g., "run script X", "execute Y"), prefer "No vision criteria needed."
- Do include minimal criteria to check relevance of content to the goal.
- Do NOT check content correctness (dates, numbers, facts) — the agent is not a fact-checker.
- If no obvious failure modes apply (e.g., a simple lookup or report), return "No vision criteria needed."
- Prefer returning "No vision criteria needed." over generating speculative criteria.

Format — number each criterion with a short FAILURE label (e.g., "output_empty", "missing_X"):
Label names must describe the FAILURE being detected, not the passing state.
For example: use "output_empty" not "no_output"; use "topic_mismatch" not "off_topic".
1. failure_label: "testable predicate that is True when the failure is present"
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
        tool_lines = [f"- {name}: {meta.get('description', 'no description')}"
                      for name, meta in self.tools.items()]
        tools_text = "\n".join(tool_lines)
        ABSTRACT_PLAN_PROMPT = f"""
You will create a short, high-level problem-solving strategy for the goal below.
This is NOT a domain explanation and NOT a tool invocation sequence.
It is a goal-specific strategy sketch that guides downstream incremental planning.

Only use tools that appear in the provided tool list.

AVAILABLE_TOOLS:
{tools_text}

GOAL:
{goal_text}

Instructions:

1. If this is a simple, conversational, or single-operation goal (e.g., opinion,
   greeting, single lookup, direct question), use a single step:
   ABSTRACT_PLAN:
   STEP 1: <the goal itself>
   END_PLAN
2. Otherwise:
   - Identify what *types of operations* the goal requires
     (e.g., retrieval, extraction, transformation, comparison, generation).
   - Match these operation types to the available tools.
     - Do not assume tools that are absent.
     - If multiple tools could serve a role, state how to choose between them.
   - Describe the minimum necessary sequence of conceptual steps.
     Prefer fewer steps. Do not decompose what can be done in a single tool call.
     Describe *what must be achieved*, not how to call tools.
   - Include fallback logic **only if the goal plausibly needs it**
     (e.g., multi-source lookup, ambiguous values, missing information).
   - Do not include JSON, tool calls, URLs, or domain-specific knowledge.
     The output must be a brief strategic outline, not an answer to the goal.

Format (use ONLY as many steps as needed — one step is fine for simple goals):
ABSTRACT_PLAN:
STEP 1: <what to achieve>
[STEP 2: <what to achieve> — only if genuinely needed]
END_PLAN
"""

        abstract_plan = self.executor.llm_generate(ABSTRACT_PLAN_PROMPT, max_tokens=256, temperature=GEN_TEMPERATURE, stops=["\nEND_PLAN", "END_PLAN"])
        if not abstract_plan.success or not abstract_plan.text:
            error_msg = getattr(abstract_plan, "error", None) or "Unknown error"
            logger.error(f"_preplan failed: {error_msg}")
            return "No preplan available"
        # Strip anything after END_PLAN (LLM may echo prompt after the plan)
        text = abstract_plan.text.strip()
        end_marker = "END_PLAN"
        idx = text.find(end_marker)
        if idx >= 0:
            text = text[:idx + len(end_marker)]
        return text.strip()

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
        Reflect on plan execution. Extract general world facts and tool insights.
        Returns dict with world_model_updates[] and tool_insights[].
        """

        reflection_prompt = """Extract GENERAL lessons from this execution trace. Output a single JSON object.

GOAL: {goal_text}
STEPS USED: {steps}

WORLD MODEL (existing knowledge):
{world_model}

EXECUTION TRACE:
{trace}

RULES:
- world_model_updates: only GENERAL facts that hold across different goals/times.
  Do NOT include: agent state, current location, task-specific observations, tool behavior.
  Use polarity="contradict" if the trace disproves an existing world model fact.
  It is often correct to return an empty list.
- tool_insights: contracts, limits, preconditions, or failure modes discovered about tools.
  State as constraints, not praise. Only include if genuinely useful for future planning.

OUTPUT SCHEMA (JSON only, no other text):
{REFLECTION_FRAME_SCHEMA}
</end>
"""
        world_model_str = json.dumps(world_model, indent=2) if isinstance(world_model, dict) else str(world_model)
        reflection_prompt = reflection_prompt.replace("{world_model}", world_model_str)
        reflection_prompt = reflection_prompt.replace("{goal_text}", goal_text)
        reflection_prompt = reflection_prompt.replace("{steps}", str(steps))
        reflection_prompt = reflection_prompt.replace("{REFLECTION_FRAME_SCHEMA}", json.dumps(REFLECTION_FRAME_SCHEMA, indent=2))
        reflection_prompt = reflection_prompt.replace("{trace}", trace)
        reflection = self.executor.llm_generate(reflection_prompt, max_tokens=1536, is_json=True, temperature=0.0, stops=['</end>'])

        if reflection.text is None:
            logger.warning("Reflection LLM returned None (JSON parse failed or empty response); returning empty frame")
            return {}
        return reflection.text