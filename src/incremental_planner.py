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
from pathlib import Path
from typing import Dict, List, Any, Optional

logger = logging.getLogger(__name__)

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
try:
    import sglang as sgl
    from sglang import function, system, user, assistant, gen
    sgl.set_default_backend(
        sgl.Runtime(
            model_path="/home/bruce/vllm/models/Qwen3-Coder-30B-A3B-Instruct",
            tokenizer_path="/home/bruce/vllm/models/Qwen3-Coder-30B-A3B-Instruct",
            device="cuda",
            dtype="auto",
            tp_size=1,
            mem_fraction_static=0.8,
            tool_call_parser="qwen",
        )
    )
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False
    logger.warning("SGLang not available - incremental planner disabled")


INCREMENTAL_PLAN_SPECIFICATIONS = """
# INFOSPACE TYPE SYSTEM & RULES

Types:
- Note: Single value/document (persists across restarts)
- Collection: List of Note/Collection IDs (session-local only)
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
└─────────────────────────┴──────┴────────────┘

Efficiency Rules:
- Use tools directly on Notes for single items
- Create Collections only for 2+ Notes together
- expand, refine, as-json work on Notes ONLY, not Collections
- Use map to apply Note operations to each Collection item

Tool Selection:
- Academic papers: semantic-scholar (provides abstracts, citations, PDFs)
- General web: query-web (broad coverage, recent content)
- Single URL fetch: fetch-text (NOT for query-web/semantic-scholar results)
- as-markdown: EXTRACT existing markdown from mixed text (NOT for converting TO markdown)
- as-json: EXTRACT existing JSON from mixed text (NOT for converting TO JSON)
"""


def build_tool_catalog(available_tools: Dict[str, Dict], primitives_reference: str) -> Dict[str, Dict]:
    """
    Build tool catalog from available tools + infospace primitives.
    
    Returns dict mapping tool_name -> {fn, description, schema_hint}
    """
    tools = {}
    
    # Add infospace primitives (from reference doc)
    # Key primitives that need to be in catalog
    primitive_tools = {
        "create-note": {
            "description": "Create a persistent Note object",
            "schema_hint": {"value": "any content", "out": "$variable"}
        },
        "create-collection": {
            "description": "Create a Collection of Notes",
            "schema_hint": {"value": "array of $variables", "out": "$variable"}
        },
        "load": {
            "description": "Load persistent Note or Collection by ID",
            "schema_hint": {"resource_id": "string", "out": "$variable", "expect": "string"}
        },
        "persist": {
            "description": "Mark Note/Collection as persistent",
            "schema_hint": {"target": "$variable"}
        },
        "search": {
            "description": "Semantic search on indexed Collection",
            "schema_hint": {"source": "$variable", "query": "string", "out": "$variable", "expect": "string"}
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
        "query-web": "Search web and return Collection with FULL TEXT CONTENT already extracted. Each Note contains 'text' field with complete page content. NO need for fetch-text after this.",
        "fetch-text": "Fetch text from a SINGLE specific URL (NOT for query-web results - those already have full text). Use ONLY when you have one URL to fetch directly.",
        "semantic-scholar": "Search academic papers and return Collection with FULL METADATA (abstracts, citations, authors, venue, PDF URLs). NO need for fetch-text after this.",
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
        lines.append(f"- {name}: {meta['description']}")
        schema = json.dumps(meta['schema_hint'])
        lines.append(f"  expected_args_schema: {schema}")
    
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
        logger.warning(f"Stage 1.5: Processing {tool_name}")
        
        # Skip primitives (no SKILL.md files)
        if tool_name not in available_tools:
            logger.warning(f"Stage 1.5: {tool_name} not in available_tools (primitive, skipping)")
            continue
            
        tool_meta = available_tools[tool_name]
        # Use 'path' field which is the tool directory (not 'python_file' which is tool.py)
        tool_dir_path = tool_meta.get('path')
        logger.warning(f"Stage 1.5: {tool_name} path = {tool_dir_path}")
        
        if not tool_dir_path:
            logger.warning(f"Stage 1.5: {tool_name} has no path field, skipping")
            continue
        
        # Resolve to absolute path
        tool_dir = Path(tool_dir_path).resolve()
        logger.warning(f"Stage 1.5: {tool_name} tool_dir = {tool_dir}")
        
        # Look for SKILL.md or Skill.md in tool directory
        skill_file = None
        for variant in ['SKILL.md', 'Skill.md', 'skill.md']:
            candidate = tool_dir / variant
            if candidate.exists():
                skill_file = candidate
                logger.warning(f"Stage 1.5: {tool_name} found {variant} at {skill_file}")
                break
        
        if not skill_file:
            logger.warning(f"Stage 1.5: No SKILL.md found for {tool_name} in {tool_dir}")
            continue
        
        try:
            with open(skill_file, 'r', encoding='utf-8') as f:
                content = f.read()
            logger.warning(f"Stage 1.5: {tool_name} loaded {len(content)} chars from {skill_file}")
            
            # Robust frontmatter stripping using regex
            # Match first complete frontmatter block: --- ... ---
            original_len = len(content)
            frontmatter_match = re.search(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
            if frontmatter_match:
                # Extract content after the closing ---
                content = content[frontmatter_match.end():].strip()
                logger.warning(f"Stage 1.5: {tool_name} stripped frontmatter, {original_len} -> {len(content)} chars")
            else:
                logger.warning(f"Stage 1.5: {tool_name} no frontmatter found, using full content")
            
            if not content:
                logger.warning(f"Stage 1.5: {tool_name} content empty after stripping, skipping")
                continue
            
            lines.append(f"\n## {tool_name.upper()}")
            lines.append(content)
            lines.append("\n" + "="*80 + "\n")
            logger.warning(f"Stage 1.5: {tool_name} added to docs ({len(content)} chars)")
            
        except Exception as e:
            logger.warning(f"Stage 1.5: Failed to load SKILL.md for {tool_name}: {e}")
            traceback.print_exc()
            continue
    
    if len(lines) <= 2:  # Only header, no docs loaded
        logger.warning("Stage 1.5: No docs loaded for any tools")
        return ""
    
    total_docs = "\n".join(lines)
    logger.warning(f"Stage 1.5: Returning {len(total_docs)} total chars of docs")
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
    
    # Normalize variable references: ensure $ prefix for common variable fields
    # These fields typically reference variables (not literal values)
    variable_fields = ['out', 'target', 'value', 'source']
    for field in variable_fields:
        if field in args:
            val = args[field]
            # Only normalize if it looks like a variable name (alphanumeric + underscore)
            # Skip if it's already prefixed with $, or if it looks like a resource ID, or contains special chars
            if isinstance(val, str) and val and not val.startswith('$'):
                # Check if it looks like a variable (not a URL, path, or resource ID)
                if re.match(r'^[a-zA-Z_]\w*$', val) and not val.startswith(('Note_', 'Collection_', 'http://', 'https://')):
                    args[field] = f"${val}"
                    logger.warning(f"Normalized '{field}' field: '{val}' -> '${val}'")
    
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
    output_producing = ["create-note", "create-collection", "load", "search", "map", 
                       "expand", "flatten", "query-web", "semantic-scholar", "summarize",
                       "refine", "assess", "relate", "extract-entities", "filter-collection",
                       "fetch-text", "as-json", "as-markdown"]
    if tool_name in output_producing and "out" not in action:
        action["out"] = f"$step_{step}_result"
    
    # Add expect if needed
    uncertain_tools = ["query-web", "semantic-scholar", "search", "load"]
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
    def tool_planner_infospace(s, goal: str, tools_catalog_text: str, executor, max_steps: int = 8):
        """
        SGLang incremental planner for infospace goals.
        
        Args:
            s: SGLang state
            goal: Goal text
            tools_catalog_text: Formatted tool catalog
            executor: InfospaceExecutor instance (with _plan_actions attribute)
            max_steps: Maximum planning steps
        """
        # Stage 1: Analysis + tool selection
        s += system(
            "You are a planning-and-acting assistant for information space operations.\n"
            "You can choose tools/primitives, call them via JSON arguments, "
            "and iteratively refine your plan until the goal is satisfied.\n\n"
            f"{INCREMENTAL_PLAN_SPECIFICATIONS}\n\n"
            "You will work in repeated cycles:\n"
            "Stage 1 (once): Analyze goal and select relevant tools.\n"
            "Stage 2 (loop): Pick a single tool and JSON args.\n"
            "Stage 3 (loop): Reflect on result, decide if DONE, update goal if needed.\n\n"
            "ALWAYS follow formatting instructions exactly."
        )
        
        s += user(
            f"Goal: {goal}\n\n"
            f"Tool catalog:\n{tools_catalog_text}\n\n"
            "Stage 1: Analyze goal and select relevant tools.\n"
            "Include tools you might need AND related/supporting tools.\n"
            "Err on the side of including MORE tools for better context.\n"
            "Respond:\n"
            "ANALYSIS: <text>\n"
            "SELECTED_TOOLS_JSON: <json list>\n"
        )
        
        s += assistant(
            "ANALYSIS: "
            + gen("stage1_analysis", max_tokens=256, stop="\n")
            + "\nSELECTED_TOOLS_JSON: "
            + gen("selected_tools_json", max_tokens=256, stop="\n")
            + "\n"
        )
        print(f"Stage 1: Analysis + tool selection\n{s['stage1_analysis']}")
        print(f"SELECTED_TOOLS_JSON: {s['selected_tools_json']}")
        
        # Stage 1.5: Load and inject detailed docs for selected tools
        try:
            selected_tools = json.loads(s['selected_tools_json'])
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
            "CRITICAL: Variable references MUST start with $ (dollar sign):\n"
            "  - Correct: {\"value\": \"$my_variable\"}\n"
            "  - Wrong: {\"value\": \"my_variable\"}\n"
            "  - Fields like 'value', 'target', 'source', 'out' typically reference variables\n\n"
            "Stage 3 FORMAT:\n"
            "  THOUGHTS: <text>\n"
            "  DONE: <YES or NO>\n"
            "  UPDATED_GOAL: <text>\n"
            "  REQUEST_TOOLS: <optional: json list of tool names you need docs for, or leave blank>\n\n"
            "If you realize you need a tool not initially selected, add it to REQUEST_TOOLS.\n"
            "You'll receive its full documentation before the next step.\n\n"
            "Follow these formats exactly."
        )
        s += assistant("Understood.\n")
        
        # Main loop
        current_goal = goal
        for step in range(max_steps):
            # Stage 2: Choose tool + args
            s += user(
                f"STAGE 2 (step {step}):\n"
                f"CURRENT_GOAL: {current_goal}\n"
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
            tool_result = execute_infospace_action(action, executor, executor.agent_name)
            
            logger.info(f"Step {step}: {tool_name} -> {tool_result[:100]}")
            
            # Stage 3: Reflect
            s += user(
                f"STAGE 3 (step {step}):\n"
                f"Tool `{tool_name}` with args:\n{tool_args_json}\n\n"
                f"Result:\n{tool_result}\n\n"
                "Respond using Stage 3 FORMAT.\n"
                "IMPORTANT: Only set DONE: YES when ALL goal requirements are met.\n"
                "- If goal mentions 'display', 'show', or 'present', you MUST use display primitive before marking done.\n"
                "- If goal mentions 'save' or 'store', you MUST persist before marking done.\n"
            )
            
            s += assistant(
                "THOUGHTS: "
                + gen(f"thoughts_{step}", max_tokens=256, stop="\n")
                + "\nDONE: "
                + gen(f"done_{step}", max_tokens=8, stop="\n")
                + "\nUPDATED_GOAL: "
                + gen(f"updated_goal_{step}", max_tokens=256, stop="\n")
                + "\nREQUEST_TOOLS: "
                + gen(f"request_tools_{step}", max_tokens=64, stop="\n")
                + "\n"
            )
            print(f"THOUGHTS: {s[f'thoughts_{step}']}")
            print(f"DONE: {s[f'done_{step}']}")
            print(f"UPDATED_GOAL: {s[f'updated_goal_{step}']}")
            print(f"REQUEST_TOOLS: {s[f'request_tools_{step}']}")
            
            # Stage 3.5: Dynamic tool loading (if requested)
            requested_tools_raw = s[f"request_tools_{step}"].strip()
            if requested_tools_raw and requested_tools_raw.lower() not in ["", "[]", "none", "null"]:
                try:
                    requested_tools = json.loads(requested_tools_raw)
                    if isinstance(requested_tools, list) and requested_tools:
                        logger.warning(f"Step {step}: LLM requested additional tools: {requested_tools}")
                        expanded_docs = load_skill_docs(requested_tools, executor.available_tools)
                        if expanded_docs:
                            s += user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                            s += assistant("I have reviewed the additional tool documentation.\n")
                            logger.info(f"Stage 3.5: Loaded docs for {len(requested_tools)} additional tools: {requested_tools}")
                except (json.JSONDecodeError, TypeError, ValueError) as e:
                    logger.warning(f"Step {step}: Failed to parse REQUEST_TOOLS: {e}")
            
            # Check if done
            done_raw = s[f"done_{step}"].strip().upper()
            if done_raw.startswith("YES"):
                s["final_answer"] = s[f"thoughts_{step}"]
                return
            
            # Update goal
            current_goal = s[f"updated_goal_{step}"]
            s["current_goal"] = current_goal
        
        # Max steps reached
        s["final_answer"] = (
            f"Max steps reached. Last goal: {current_goal}\n"
            f"Last thoughts: {s[f'thoughts_{max_steps-1}']}"
        )


class IncrementalPlanner:
    """
    Incremental planner using SGLang for iterative goal achievement.
    """
    
    def __init__(self, executor, available_tools: Dict[str, Dict], 
                 primitives_reference: str, logger_instance=None):
        """
        Initialize incremental planner.
        
        Args:
            executor: InfospaceExecutor instance
            available_tools: Dict of tool_name -> metadata
            primitives_reference: Primitives reference text
            logger_instance: Optional logger
        """
        if not HAS_SGLANG:
            raise ImportError("SGLang not available")
        
        self.executor = executor
        self.available_tools = available_tools
        self.primitives_reference = primitives_reference
        self.logger = logger_instance or logger
        
        # Build tool catalog
        self.tools = build_tool_catalog(available_tools, primitives_reference)
        self.tools_catalog_text = tool_catalog_text(self.tools)
        print (self.tools_catalog_text)
        self.logger.info(f"IncrementalPlanner initialized with {len(self.tools)} tools")
    
    def generate_plan(self, goal: str, max_steps: int = 8) -> Dict:
        """
        Generate plan incrementally using SGLang.
        
        Args:
            goal: Goal text
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
            
            # Run SGLang planner
            state = tool_planner_infospace.run(
                goal=goal,
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

