"""
Infospace Planner - Generates plans using information space primitives.

Provides templates and planning logic for cognitive/information processing tasks.
"""

import json
import logging
from typing import Dict, List, Any

logger = logging.getLogger(__name__)

# Phase 1 & 2 Template - Core + Data Operations
INFOSPACE_PLAN_TEMPLATE = """TASK: Generate a JSON format plan for the goal below using the ACTIONS and CONDITIONS listed below.

Your goal is:
{{goal}}

OUTPUT: only valid JSON — no reasoning, no prose, no code fences.

# PLAN FORMAT

{
  "plan": [
    {"type": "action_name", ...},
    {"type": "action_name", ...}
  ]
}

# AVAILABLE ACTIONS:

# Primitives:
map - apply operation to each item in a Collection
flatten - convert Collection to single Note by concatenating items
transform - convert data format or structure (whole-value operations only)
move - change current location or approach a resource
createNote - create a persistent Note object and bind to variable
createCollection - create a session-local Collection object and bind to variable
persist - mark a Collection as persistent (saved to filesystem)
load - retrieve a persistent Note or Collection by resource ID or name
index (organize) - build an embedding index for a Collection
search - query an indexed Collection
if - conditional branch 
while - loop until condition false
wait - pause until condition true
say - produce output (inline)
display - show formatted content (popup)
think - internal note

# ARGUMENT TYPE CONVENTIONS:

Variables: Use "$variable" to reference Note/Collection content previously bound
Literals: Use plain strings/values (no $) for literal data or names
Names: Output variable names in "out" fields use plain strings (no $)
Tools/Resources: Can be literal "tool-name/resource-name" or "$variable" holding name
Resource IDs: Cannot be referenced directly - use "load" action first to bind to variable


# COMMON PATTERNS:

Pattern: Adding filtered items to existing Collection
  When you need to search, filter, then add results to a persistent collection:
  1. Load the persistent collection
  2. Search/expand to get new items
  3. Optionally filter with map + tool
  4. Use map with add to append each item
  
  Example - Add new research papers to existing collection:
  {"type":"load","resource_id":"papers","out":"$papers"}
  {"type":"web-search","args":{"query":"LLM agents 2025"},"out":"$results"}
  {"type":"expand","target":"$results","out":"$new_papers"}
  {"type":"map","target":"$new_papers","operation":"add","args":{"target":"$papers"},"out":"$papers"}
  {"type":"persist","target":"$papers"}

Pattern: Multi-item tool application
  When you need to apply a tool to *two or more* Notes (e.g., compare, analyze together):
  1. Create a Collection containing the Notes
  2. Apply the tool to the Collection (tool receives all items)
  
  Example - Compare two search results:
  {"type":"createCollection","value":["$results_2024","$results_2025"],"out":"$both_years"}
  {"type":"compare-notes","target":"$both_years","out":"$comparison"}
  
  Tools that work with Collections: compare-notes, summarize-content, extract-entities

Pattern: Optional tool arguments
  Many tools accept optional "focus" or "mode" parameters via "args" field:
  {"type":"summarize-content","target":"$doc","args":{"focus":"key findings"},"out":"$summary"}
  
  Using focus is optional - tools work generically without it, but focus can improve precision.

Pattern: Single vs. Multiple Item Processing
  Collections are for handling MULTIPLE Notes together. For single Notes, use tools directly.
  
  AVOID - Unnecessary Collection wrapper:
  {"type":"createCollection","value":["$single_result"],"out":"$wrapper"}
  {"type":"summarize-content","target":"$wrapper","out":"$summary"}
  
  PREFER - Direct tool use:
  {"type":"summarize-content","target":"$single_result","out":"$summary"}
  
  Use Collections ONLY when you have 2+ Notes to process together:
  {"type":"createCollection","value":["$result1","$result2","$result3"],"out":"$multiple"}
  {"type":"summarize-content","target":"$multiple","out":"$summary"}

Pattern: Universal LLM Transformations (transform-note, test-note)
  For ad-hoc transformations without specialized tools, use transform-note with natural language commands.
  PREFER specialized tools when available (faster, cheaper, deterministic).
  USE transform-note for novel/exploratory operations.
  
  Example - Extract schema when no specialized tool exists:
  {"type":"transform-note","target":"$data","args":{"command":"extract schema as JSON"},"out":"$schema"}
  
  Example - Complex reasoning (use premium model):
  {"type":"transform-note","target":"$paper","args":{"command":"identify all citations","model":"sonnet"},"out":"$citations"}
  
  Example - Boolean test in conditional:
  {"type":"if","condition":{"type":"tool_condition","tool":"test-note","target":"$content","args":{"predicate":"contains citations?"}},"then":[...]}
  
  Cost aware: transform-note uses LLM per call. Reserve for cases where specialized tools don't exist.


# ACTION SCHEMAS  (each must be valid JSON)

# Tools - use tool name directly as action type:
{"type":"tool-name","target":"input text or $data","args":{"optional":"param"},"out":"$result_variable"}
Example: {"type":"download-pdf","target":"https://example.com/doc.pdf","out":"$pdf_note"}
Example: {"type":"summarize-content","target":"$doc","args":{"focus":"key points"},"out":"$summary"}

map — apply operation to each item in Collection (input: Collection → output: Collection)
{"type":"map","target":"$collection","operation":"tool-name or {'tool':'name','args':{}}","out":"$result_collection"}
{"type":"map","target":"$collection","operation":"tool-name","filter_null":true,"out":"$filtered_results"}  # exclude null results
{"type":"map","target":"$new_items","operation":"add","args":{"target":"$existing_collection"},"out":"$existing_collection"}  # add each item to collection

flatten — convert Collection to single Note by concatenating items (input: Collection → output: Note)
{"type":"flatten","target":"$collection","out":"$combined_note"}
{"type":"flatten","target":"$collection","separator":"\\n---\\n","out":"$combined"}  # custom separator

add — add a Note to an existing Collection (mutates Collection in place)
{"type":"add","target":"$collection","value":"$new_note","out":"$collection"}  # out should match target
{"type":"add","target":"$dialog_history","value":"Hello user","out":"$dialog_history"}  # literal value creates new Note

expand — expand a Note containing JSON array into a Collection of Notes (input: Note → output: Collection)
{"type":"expand","target":"$search_results","out":"$results_collection"}  # default field is 'results'
{"type":"expand","target":"$data","field":"items","out":"$items_collection"}  # custom field name

transform — convert data format or structure (whole-value) (input: Note → output: Note)
{"type":"transform","target":"$data","operation":"flatten|normalize|pivot|reshape","out":"$transformed"}

move — change current location or approach a resource
{"type":"move","target":"resource-name or {"location": [x,y]}"}

createNote — create a persistent Note object
{"type":"createNote","value":"some data","out":"$my_note"}
{"type":"createNote","value":"$variable","out":"$new_note"}

createCollection — create a session-local Collection object
{"type":"createCollection","value":["$note1","$note2"],"out":"$my_collection"}  # Collection of Note references
{"type":"createCollection","name":"research","value":[],"out":"$papers"}  # Named empty Collection
{"type":"createCollection","value":"$note","out":"$single_item"}  # Collection with one item

persist — mark Collection as persistent (saved to filesystem)
{"type":"persist","target":"$collection"}

load — retrieve a persistent Note or Collection by resource ID or name
{"type":"load","resource_id":"Note_123","out":"$my_note"}
{"type":"load","resource_id":"Collection_5","out":"$items"}
{"type":"load","resource_id":"shopping_list","out":"$list"}  # Load by collection name

index (organize) — create embeddings index for a Collection
{"type":"index","source":"$collection","index_type":"semantic"}
{"type":"index","source":"$papers","index_type":"semantic","fields":{"title":"embed","content":"embed"}}  # optional fields

search — query an indexed Collection
{"type":"search","source":"$collection","query":"search text or $query","mode":"semantic","limit":5,"out":"$results"}
{"type":"search","source":"$papers","query":"quantum computing","limit":3,"out":"$top_papers"}  # search indexed Collection

if — conditional branch
{"type":"if","condition":{"type":"has_value","target":"$results"},"then":[/* steps */],"else":[/* optional steps */]}

while — loop until condition false
{"type":"while","condition":{"type":"has_value","target":"$results"},"body":[/* steps */],"max_iterations":10}

wait — pause until condition true
{"type":"wait","condition":{"type":"has_value","target":"$results"},"timeout":30}

say — produce output (inline display)
{"type":"say","target":"user","value":"literal text or $variable"}

display — show formatted content in popup (for documents/formatted output)
{"type":"display","target":"user","value":"$formatted_note"}

think — internal note (logged only)
{"type":"think","value":"thought text or $variable"}

# CONDITION SCHEMA (uniform form)
All conditions evaluate to a boolean:
{"type": "<condition_type>", "target": "$variable", "field?": "optional_field", "value?": "<literal or variable>"}

# CONDITION SYNTAX (all evaluate to boolean)

Variable state - test existence/binding (works on both Note and Collection):
{"type": "bound", "target": "$var"}           // true if $var exists
{"type": "notbound", "target": "$var"}        // true if $var doesn't exist
{"type": "has_value", "target": "$var"}       // true if $var is truthy
{"type": "empty", "target": "$var"}           // true if $var is falsy/empty (or empty Collection)

Value comparison - test content (target must be Note with comparable content):
{"type": "equals", "target": "$var", "value": "expected"}
{"type": "not_equals", "target": "$var", "value": "unwanted"}
{"type": "greater_than", "target": "$score", "value": 0.8}         // numeric comparison
{"type": "less_than", "target": "$count", "value": 100}            // numeric comparison
{"type": "gte", "target": "$score", "value": 0.7}                  // numeric comparison
{"type": "lte", "target": "$size", "value": 1000}                  // numeric comparison

Membership - test content (target must be Note):
{"type": "contains", "target": "$text", "value": "keyword"}        // substring (string) or element (list)
{"type": "not_contains", "target": "$tags", "value": "spam"}

Pattern matching (target must be Note with string content):
{"type": "matches_pattern", "target": "$text", "pattern": "regex_pattern"}

Tool-based conditions - delegate complex predicates (target can be Note or Collection):
{"type": "tool_condition", "tool": "has_field", "target": "$data", "args": {"field": "urls"}}    // tool returns boolean
{"type": "tool_condition", "tool": "json_valid", "target": "$text", "args": {}}                  // validate structure


# TOOLS - use tool names directly as action types. Tools available to the agent are listed below.

{{tools}}


# EXAMPLES

Minimal Example (say hello):
{
  "plan": [
    {"type": "say", "target": "user", "value": "Hello!"}
  ]
}

Research Example (multi-step information flow):
{
  "plan": [
    {"type": "createNote","value": "LLM cognitive agents 2025","out": "$query"},
    {"type": "web-search","args":{"query":"$query"},"out": "$result1"},
    {"type": "createNote","value": "transformer architecture papers","out": "$query2"},
    {"type": "web-search","args":{"query":"$query2"},"out": "$result2"},
    {"type": "createCollection","value": ["$result1","$result2"],"out": "$research_collection"},
    {"type": "index","source": "$research_collection","index_type": "semantic","fields": {"title":"embed","content":"embed"}},
    {"type": "say","target": "user","value": "Research complete and indexed."}
  ]
}

# SEMANTIC RULES

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

CONSTRAINTS
- Use only primitives listed above or tools from the tools list below.
- Variables must be created before use (createNote, createCollection, tools, search, or index may bind them).
- All JSON must be syntactically valid (no comments or trailing commas).
- Keep plans concise (recommended: 12 steps or fewer).
- Output only valid JSON.

EFFICIENCY RULES
- Use tools directly on Notes when processing single items
- Create Collections only when you need to process 2+ Notes together
- Minimize intermediate steps - prefer direct operations over unnecessary wrapping

Respond only with the complete JSON plan for the goal, no other text.
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
        self.template = INFOSPACE_PLAN_TEMPLATE
    
    def _validate_plan(self, plan: Dict) -> Dict:
        """
        Validate plan structure.
        Check required fields per action type.
        
        Returns:
            Dict with 'valid' (bool) and 'reason' (str if invalid)
        """
        if 'plan' not in plan:
            return {'valid': False, 'reason': 'Missing plan field'}
        
        actions = plan['plan']
        if not isinstance(actions, list):
            return {'valid': False, 'reason': 'Plan must be array'}
        
        if len(actions) > 12:
            self.logger.warning(f'Plan has {len(actions)} steps (recommended max: 12)')
        
        # Validate each action
        for i, action in enumerate(actions):
            if not isinstance(action, dict):
                return {'valid': False, 'reason': f'Action {i} is not dict'}
            
            if 'type' not in action:
                return {'valid': False, 'reason': f'Action {i} missing type'}
            
            # Check required fields per type
            action_type = action['type']
            validation = self._validate_action_fields(action_type, action)
            if not validation['valid']:
                return {'valid': False, 'reason': f'Action {i} ({action_type}): {validation["reason"]}'}
        
        return {'valid': True}
    
    def _validate_action_fields(self, action_type: str, action: Dict) -> Dict:
        """Validate required fields for action type"""
        # Known primitive action types with their required fields
        required_fields = {
            'apply': ['target', 'out'],
            'move': ['target'],
            'createNote': ['value', 'out'],
            'createCollection': ['value', 'out'],
            'persist': ['target'],
            'load': ['resource_id', 'out'],
            'index': ['source'],
            'search': ['source', 'query', 'out'],
            'expand': ['target', 'out'],
            'flatten': ['target', 'out'],
            'transform': ['target', 'operation', 'out'],
            'add': ['target', 'value', 'out'],
            'map': ['target', 'operation', 'out'],
            'if': ['condition', 'then'],
            'while': ['condition', 'body'],
            'wait': ['condition'],
            'say': ['target', 'value'],
            'display': ['target', 'value'],
            'think': ['value'],
        }
        
        required = required_fields.get(action_type, [])
        
        # If action_type is not a known primitive, treat it as a tool name
        if not required:
            # Validate tool name exists in available_tools if provided
            if self.available_tools and action_type not in self.available_tools:
                return {'valid': False, 'reason': f'Unknown tool name: {action_type}'}
            
            # Tool names should have 'target' and 'out' fields per template examples
            # Exception: web-search uses args.query instead of target
            if action_type == 'web-search':
                if 'args' not in action or 'query' not in action.get('args', {}):
                    return {'valid': False, 'reason': f'Missing required field: args.query'}
                if 'out' not in action:
                    return {'valid': False, 'reason': f'Missing required field: out'}
                # Validate out field uses $variable syntax
                out_val = action.get('out', '')
                if not isinstance(out_val, str) or not out_val.startswith('$'):
                    return {'valid': False, 'reason': f'out field must use $variable syntax, got: {out_val}'}
                return {'valid': True}
            
            if 'target' not in action:
                return {'valid': False, 'reason': f'Missing required field: target'}
            if 'out' not in action:
                return {'valid': False, 'reason': f'Missing required field: out'}
            # Validate out field uses $variable syntax
            out_val = action.get('out', '')
            if not isinstance(out_val, str) or not out_val.startswith('$'):
                return {'valid': False, 'reason': f'out field must use $variable syntax, got: {out_val}'}
            return {'valid': True}
        
        for field in required:
            if field not in action:
                return {'valid': False, 'reason': f'Missing required field: {field}'}
        
        return {'valid': True}


def verify_plan(plan_json: Any, available_tools: Dict[str, Dict] = None) -> Dict:
    """
    Validate infospace plan structure.
    Public interface for plan validation.
    
    Args:
        plan_json: Plan as JSON string or dict
        available_tools: Optional dict of available tool_name -> metadata for validating tool names
        
    Returns:
        Dict with 'valid' (bool) and 'reason' (str if invalid)
    """
    planner = InfospacePlanner(None, available_tools=available_tools)  # Validator only, no LLM needed
    validation = planner._validate_plan(plan_json if isinstance(plan_json, dict) else json.loads(plan_json))
    return validation

