"""
Infospace Planner - Generates plans using information space primitives.

Provides templates and planning logic for cognitive/information processing tasks.
"""

import json
import logging
from typing import Dict, List, Any

logger = logging.getLogger(__name__)

# Phase 1 & 2 Template - Core + Data Operations
INFOSPACE_PLAN_TEMPLATE = """
# PLAN SPECIFICATION

The following section defines the canonical JSON plan format, available actions, 
type system, and semantic and efficiency rules used by all planning tasks.  
It serves as a shared reference for both plan generation and plan validation.  
Do not interpret it as an instruction to produce or modify a plan — it describes the 
language in which plans are written and evaluated.
# PLAN FORMAT

{
  "plan": [
    {"type": "action_name", ...},
    {"type": "action_name", ...}
  ]
}

# AVAILABLE ACTIONS:

{primitives_reference}

# SEMANTIC RULES

Planning Language Architecture (4 Levels):
1. Resource & Control: Primitives operating on Note/Collection IDs (create-note, create-collection, persist, load, add, remove, union, intersection, difference, size, if, while, wait)
2. Text Operations: Primitives and tools operating on unstructured text Notes (expand, flatten, say, display, think, filter-collection, refine, assess, summarize, relate)
3. Coercion: Tools that extract structure from text (as-json, as-markdown)
4. Structured Operations: Tools/primitives that produce/consume Collections of structured Notes
   - Tools: query-web, semantic-scholar (produce Collections)
   - Primitives: project, pluck, filter-structured, sort, join (SQL-like operations on Collections)

Type System:
- Note: typed object storing a single value/data structure. Persists across restarts.
  - All Notes have envelope schema: {name, created, creator, content, content_type}
- Collection: typed object storing a list of Note/Collection resource IDs (references). Session-local only (lost on restart).
  - Collections store references (e.g., ["Note_123", "Note_456"]), not raw data
  - Literals in Collection values are auto-wrapped in Notes
- Named Collection: Collection with stable name for referencing across plans within session.
- Both Note and Collection are created with id and stored in plan_bindings.

Variables:
- Variables are plan-local names that reference Note/Collection objects
- Actions with "out" create new Note/Collection objects and bind to the variable name in the "out" field
- Always use "$variable" syntax for variables (in target, value, out fields)
- Variables are cleared after plan completion

Argument Conventions:
- Literal strings/values: Use directly without "$" prefix (e.g., "hello")
- Variable references: Use "$variable" to resolve Note/Collection content
- Output names: In "out" fields, use "$variable" syntax (e.g., "out":"$result")
- Tools: Use tool name directly as action type (e.g., {"type":"tool-name",...})
- Resource names: Case-sensitive

CONSTRAINTS
- Use only primitives / tools listed in the AVAILABLE ACTIONS and TOOLS sections.
- Variables must be created before use (create-note, create-collection, tools, search-notes, search-collections, search-within-collection, or index may bind them).
- All JSON must be syntactically valid (no comments or trailing commas).
- Keep plans concise (recommended: 12 steps or fewer).
- Output only valid JSON.
- REQUIRED FIELDS: All output-producing tools MUST include 'out' field. Tools with uncertain outcomes (query-web, semantic-scholar, search-notes, search-collections, search-within-collection, load, summarize, relate, refine, assess, extract-entities, filter-collection) MUST also include 'expect' field.
- Note: 'display' and 'think' accept either 'value' or 'target' (both work).

EFFICIENCY RULES
- Use tools directly on Notes when processing single items
- Create Collections only when you need to process 2+ Notes together
- REMEMBER: expand, refine, as-json work on Notes ONLY, not Collections
- Use map to apply Note operations to each item in a Collection

TOOL SELECTION RULES
- For academic papers (research, citations, scholarly articles): Use semantic-scholar first
- For general web content, news, documentation, tutorials: Use query-web
- semantic-scholar provides: abstracts, citations, authors, venue, PDF URLs (when available)
- query-web provides: broad web coverage, recent content, mixed-quality sources

# COMMON PATTERNS:

Pattern: Persisting Collections
  create-collection → persist → (optional: use name parameter for named Collection)

  Example - Save research findings:
  {"type":"create-collection","value":["$methodology_summary","$gaps_analysis"],"name":"constitutional-ai-findings","out":"$findings"}
  {"type":"persist","target":"$findings"}

Pattern: Combining Multiple Collections
  Use union to merge results from multiple sources:
  {"type":"union","target":"$collection1","value":"$collection2","out":"$combined"}

Pattern: Adding filtered items to existing Collection
  load → semantic-scholar/query-web (returns Collection) → map with add to append each item → persist
  
  Example - Add new research papers to existing collection:
  {"type":"load","resource_id":"papers","out":"$papers","expect":"should have existing paper collection"}
  {"type":"semantic-scholar","args":{"query":"LLM agents 2025"},"out":"$new_papers","expect":"should find recent papers"}
  {"type":"map","target":"$new_papers","operation":"add","args":{"target":"$papers"},"out":"$papers"}

Pattern: Global resource discovery (search-notes/search-collections)
  search-notes → load → (use found Note IDs)
  search-collections → load → (use found Collection IDs)
  
  Example - Find existing Notes:
  {"type":"search-notes","value":"transformer architecture","limit":5,"out":"$found_notes","expect":"should find Notes about transformers"}
  {"type":"load","resource_id":"$found_notes[0]","out":"$note1"}
  
  Example - Find existing Collections:
  {"type":"search-collections","value":"research papers","limit":3,"out":"$found_collections","expect":"should find Collections of papers"}
  
Pattern: Semantic search within indexed Collection (index/search-within-collection)
  index → search-within-collection → (optional: use return_mode to control what search returns)
  
  Example - Find relevant passages:
  {"type":"index","source":"$documents"}
  {"type":"search-within-collection","target":"$documents","value":"transformer attention","return_mode":"chunks","limit":10,"out":"$passages","expect":"should return relevant text excerpts"}
  
  Example - Find relevant documents:
  {"type":"index","source":"$papers"}
  {"type":"search-within-collection","target":"$papers","value":"block-rank algorithm","return_mode":"notes","limit":5,"out":"$relevant_papers","expect":"should return papers discussing block-rank"}

Pattern: Multi-item tool application
  When you need to apply a tool to two or more Notes together (e.g. relate, analyze together), create a Collection.
  create-collection → apply → (optional: use tool with target to apply to Collection)
  
  Example - Compare two search results:
  {"type":"create-collection","value":["$results_2024","$results_2025"],"out":"$both_years"}
  {"type":"relate","target":"$both_years","out":"$comparison"}
  
  Tools that work with Collections: relate, summarize, extract-entities

Pattern: Optional tool arguments
  Many tools accept optional "focus" or "mode" parameters via "args" field:
  {"type":"summarize","target":"$doc","args":{"focus":"key findings"},"out":"$summary"}
  
  expect field documents the expected outcome. args.focus guides the tool's behavior for better precision.
  Both can reference the same topic - expect describes outcome, focus guides processing.

Pattern: Single vs. Multiple Item Processing
  Collections are for handling MULTIPLE Notes together. For single Notes, use tools directly.
  
  CRITICAL: Most operations work on Notes only. Only relate, summarize, and 
  extract-entities accept Collections directly.
  
  WRONG - Applying Note-only operations to Collections:
  {"type":"create-collection","value":["$r1","$r2"],"out":"$both"}
  {"type":"expand","target":"$both","out":"$items"}  // ❌ expand needs Note, not Collection
  
  RIGHT - Expand first, then combine:
  {"type":"expand","target":"$r1","out":"$items1"}
  {"type":"expand","target":"$r2","out":"$items2"}
  {"type":"union","target":"$items1","value":"$items2","out":"$all_items"}

Pattern: Universal LLM Transformations (refine, assess)
  Use refine for editing/transforming existing Note content (does not add new content).
  Use assess for complex boolean conditions.  
  refine → assess → (optional: use target to apply to Collection)

Pattern: Level 4 Structured Data Operations (SQL-like)
  Use project, pluck, filter-structured, sort, join on Collections of JSON Notes:
  
  project - Extract specific fields from structured Notes (preserves nested structure):
  {"type":"project","target":"$collection","fields":["field1","nested.field2"],"out":"$subset"}
  
  pluck - Extract single field value from each Note (for tools needing scalar values):
  {"type":"pluck","target":"$collection","field":"metadata.uri","out":"$urls"}
  
  filter-structured - Deterministic WHERE clause for structured Notes (no LLM):
  {"type":"filter-structured","target":"$collection","where":"field > 100","out":"$filtered"}
  {"type":"filter-structured","target":"$collection","where":"year > 2020 AND citations >= 50","out":"$recent_cited"}
  
  sort - Order structured Notes by field value:
  {"type":"sort","target":"$collection","by":"field","order":"desc","out":"$sorted"}
  
  join - Merge two Collections of structured Notes on key:
  {"type":"join","target":"$left","value":"$right","args":{"on":"id"},"out":"$merged"}
  {"type":"join","target":"$left","value":"$right","args":{"left_key":"paper_id","right_key":"id","type":"left"},"out":"$merged"}

# TOOLS - use tool names directly as action types. Tools available to the agent are listed below.

#FIELD REQUIREMENTS QUICK REFERENCE:
┌──────────────────────┬─────┬─────┬────────┐
│ Tool Category        │ out │ tgt │ expect │
├──────────────────────┼─────┼─────┼────────┤
│ LLM Tools            │  ✓  │  ✓  │   ✓    │
│ Search/Load          │  ✓  │  ✓  │   ✓    │
│ Transform (map/exp)  │  ✓  │ opt │   -    │
│ Side-effects (say)   │  -  │  -  │   -    │
└──────────────────────┴─────┴─────┴────────┘

#OPERATION TYPE COMPATIBILITY:
┌─────────────────────────┬──────┬────────────┐
│ Operation               │ Note │ Collection │
├─────────────────────────┼──────┼────────────┤
│ expand, as-json, refine │  ✓   │     ❌     │
│ summarize, relate       │  ✓   │     ✓      │
│ map, flatten            │  ❌  │     ✓      │
│ create-collection       │  N/A │    N/A     │
└─────────────────────────┴──────┴────────────┘

ALL tools that produce output MUST include 'out' field with $variable syntax.

Example:
{"type":"summarize","target":"$doc","out":"$summary","expect":"should extract key points"}

{{tools}}


# EXAMPLES

Minimal Example (say hello):
{
  "plan": [
    {"type": "say", "target": "user", "value": "Hello!"}
  ]
}

Research Example (academic paper search):
{
  "plan": [
    {"type": "semantic-scholar","args":{"query":"machine learning interpretability"},"out": "$papers","expect":"should find ML papers"},
    {"type": "summarize","target": "$papers","out": "$summary","expect":"should provide overview of interpretability methods"},
    {"type": "display","value": "$summary"}
  ]
}

# END OF PLAN SPECIFICATION
The next section will contain task-specific instructions (e.g., generation, validation, or editing).  

"""

class InfospacePlanner:
    """
    Generates plans for information space operations.
    Uses infospace-specific templates and primitives.
    """
    
    def __init__(self, llm_client, available_tools: Dict[str, Dict] = None, logger=None):
        """
        Initialize infospace planner.
        
        Args:
            llm_client: LLM client for plan generation
            available_tools: Dict of tool_name -> metadata (from tool_loader)
            logger: Optional logger instance
        """
        self.llm_client = llm_client
        self.available_tools = available_tools or {}
        self.logger = logger or logging.getLogger(__name__)
    
    def _validate_plan(self, plan: Dict) -> str:
        """
        Validate plan structure.
        Check required fields per action type.
        
        Returns:
            String with ACTION/REPAIR format errors (empty if valid)
        """
        errors = []
        logger.info(f"Validating plan")
        if 'plan' not in plan:
            errors.append("ACTION 0:\nREPAIR: Insert missing 'plan' field key at start of plan")
            return "\n".join(errors)
        
        actions = plan['plan']
        if not isinstance(actions, list):
            errors.append("ACTION 0:\nREPAIR: 'plan' field value must be a list of actions")
            return "\n".join(errors)
        
        if len(actions) > 12:
            self.logger.warning(f'Plan has {len(actions)} steps (recommended max: 12)')
        
        # Validate each action - collect all errors
        for i, action in enumerate(actions):
            if not isinstance(action, dict):
                errors.append(f"ACTION {i}:\nREPAIR: Action must be a dict/object with 'type' field")
                continue
            
            if 'type' not in action:
                errors.append(f"ACTION {i}:\nREPAIR: Add 'type' field specifying action or tool name")
                continue
            
            # Check required fields per type
            action_type = action['type']
            validation_error = self._validate_action_fields(action_type, action, i)
            if validation_error:
                errors.append(validation_error)
        
        return "\n".join(errors)
    
    def _validate_action_fields(self, action_type: str, action: Dict, action_index: int) -> str:
        """
        Validate required fields for action type.
        
        Returns:
            String in ACTION/REPAIR format if error found, empty string if valid
        """
        # Known primitive action types with their required fields
        required_fields = {
            'apply': ['target', 'out'],
            'focus': ['target'],
            'create-note': ['value', 'out'],
            'create-collection': ['value', 'out'],
            'persist': ['target'],
            'load': ['resource_id', 'out', 'expect'],
            'index': ['source'],
            'search': ['source', 'query', 'out', 'expect'],
            'expand': ['target', 'out'],
            'flatten': ['target', 'out'],
            'coerce': ['target', 'operation', 'out'],
            'add': ['target', 'value', 'out'],
            'remove': ['target', 'value', 'out'],
            'size': ['target', 'out'],
            'union': ['target', 'value', 'out'],
            'intersection': ['target', 'value', 'out'],
            'difference': ['target', 'value', 'out'],
            'map': ['target', 'operation', 'out'],
            'if': ['condition', 'then'],
            'while': ['condition', 'body'],
            'wait': ['condition'],
            'say': ['target', 'value'],
            'display': ['value'],  # Accepts target or value (both allowed)
            'think': ['value'],  # Accepts target or value (both allowed)
            'ask': ['value', 'out'],
            # Level 4: Structured data operations
            'project': ['target', 'fields', 'out'],
            'pluck': ['target', 'field', 'out'],
            'filter-structured': ['target', 'where', 'out'],
            'sort': ['target', 'by', 'out'],
            'join': ['target', 'value', 'out'],
        }
        
        required = required_fields.get(action_type, [])
        
        # If action_type is not a known primitive, treat it as a tool name
        if not required:
            # Validate tool name exists in available_tools if provided
            if self.available_tools and action_type not in self.available_tools:
                return f"ACTION {action_index}: {action_type}\nREPAIR: Unknown tool name '{action_type}'. Check available tools or use correct primitive name"
            
            # Check if tool has custom parameter source (e.g., args.query)
            tool_info = self.available_tools.get(action_type, {})
            param_source = tool_info.get('parameter_source')
            
            if param_source and param_source.startswith('args.'):
                # Tool uses args.* for input parameter
                param_name = param_source.split('.', 1)[1]
                if 'args' not in action or param_name not in action.get('args', {}):
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'args' field with '{param_name}' parameter: \"args\": {{\"{param_name}\": \"your input\"}}"
                if 'out' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'out' field with $variable syntax: \"out\": \"$results\""
                if 'expect' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'expect' field describing expected outcome: \"expect\": \"should find relevant results\""
                # Validate out field uses $variable syntax
                out_val = action.get('out', '')
                if not isinstance(out_val, str) or not out_val.startswith('$'):
                    return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
                return ""
            
            # LLM-based tools require expect
            llm_tools = ['summarize', 'relate', 'refine', 'assess', 'extract-entities', 'filter-collection']
            if action_type in llm_tools:
                if 'target' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'target' field referencing the variable to process: \"target\": \"$variable_name\""
                if 'out' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'out' field with $variable syntax: \"out\": \"$result\""
                if 'expect' not in action:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'expect' field describing expected outcome: \"expect\": \"should extract/transform...\""
                out_val = action.get('out', '')
                if not isinstance(out_val, str) or not out_val.startswith('$'):
                    return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
                return ""
            
            # Other tools need target and out
            if 'target' not in action:
                return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'target' field referencing the variable to process: \"target\": \"$variable_name\""
            if 'out' not in action:
                return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'out' field with $variable syntax: \"out\": \"$result\""
            # Validate out field uses $variable syntax
            out_val = action.get('out', '')
            if not isinstance(out_val, str) or not out_val.startswith('$'):
                return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
            return ""
        
        # Check required fields for known primitives
        for field in required:
            if field not in action:
                # Special handling: display and think accept either value or target
                if field == 'value' and action_type in ['display', 'think']:
                    if 'target' not in action:
                        return f"ACTION {action_index}: {action_type}\nREPAIR: Add 'value' or 'target' field (either works for {action_type})"
                else:
                    return f"ACTION {action_index}: {action_type}\nREPAIR: Add required field '{field}'"
        
        # Validate 'out' field format if present
        if 'out' in action:
            out_val = action.get('out', '')
            if not isinstance(out_val, str) or not out_val.startswith('$'):
                return f"ACTION {action_index}: {action_type}\nREPAIR: 'out' field must use $variable syntax, change \"{out_val}\" to \"${out_val}\""
        
        return ""
    
    def verify_plan(self, plan: Dict) -> str:
        """
        Validate plan structure.
        Public instance method for plan validation.
        
        Args:
            plan: Plan dict with 'plan' key
            
        Returns:
            String with ACTION/REPAIR format errors (empty if valid)
        """
        return self._validate_plan(plan)

