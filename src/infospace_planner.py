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

apply - apply a tool/skill to input data and bind result to variable
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

Pattern: Multi-item tool application
  When you need to apply a tool to *two or more* Notes (e.g., compare, analyze together):
  1. Create a Collection containing the Notes
  2. Apply the tool to the Collection (tool receives all items)
  
  Example - Compare two search results:
  {"type":"createCollection","value":["$results_2024","$results_2025"],"out":"both_years"}
  {"type":"apply","target":"compare-notes","value":"$both_years","out":"comparison"}
  
  Tools that work with Collections: compare-notes, summarize-content, extract-entities

Pattern: Optional tool arguments
  Many tools accept optional "focus" or "mode" parameters via "args" field:
  {"type":"apply","target":"summarize-content","value":"$doc","args":{"focus":"key findings"},"out":"summary"}
  
  Using focus is optional - tools work generically without it, but focus can improve precision.

Pattern: Single vs. Multiple Item Processing
  Collections are for handling MULTIPLE Notes together. For single Notes, apply tools directly.
  
  AVOID - Unnecessary Collection wrapper:
  {"type":"createCollection","value":["$single_result"],"out":"wrapper"}
  {"type":"apply","target":"summarize-content","value":"$wrapper","out":"summary"}
  
  PREFER - Direct application:
  {"type":"apply","target":"summarize-content","value":"$single_result","out":"summary"}
  
  Use Collections ONLY when you have 2+ Notes to process together:
  {"type":"createCollection","value":["$result1","$result2","$result3"],"out":"multiple"}
  {"type":"apply","target":"summarize-content","value":"$multiple","out":"summary"}


# ACTION SCHEMAS  (each must be valid JSON)

apply — apply a tool/skill to input data (input: Note → output: Note)
{"type":"apply","target":"tool-name/resource-name or $tool_var","value":"input text or $data","reason":"purpose (optional)","out":"result_variable"}

map — apply operation to each item in Collection (input: Collection → output: Collection)
{"type":"map","target":"$collection","operation":"tool-name or {'tool':'name','args':{}}","out":"result_collection"}
{"type":"map","target":"$collection","operation":"tool-name","filter_null":true,"out":"filtered_results"}  # exclude null results

flatten — convert Collection to single Note by concatenating items (input: Collection → output: Note)
{"type":"flatten","target":"$collection","out":"combined_note"}
{"type":"flatten","target":"$collection","separator":"\\n---\\n","out":"combined"}  # custom separator

add — add a Note to an existing Collection (mutates Collection in place)
{"type":"add","target":"$collection","value":"$new_note","out":"collection"}  # out should match target
{"type":"add","target":"$dialog_history","value":"Hello user","out":"dialog_history"}  # literal value creates new Note

expand — expand a Note containing JSON array into a Collection of Notes (input: Note → output: Collection)
{"type":"expand","target":"$search_results","out":"results_collection"}  # default field is 'results'
{"type":"expand","target":"$data","field":"items","out":"items_collection"}  # custom field name

transform — convert data format or structure (whole-value) (input: Note → output: Note)
{"type":"transform","target":"$data","operation":"flatten|normalize|pivot|reshape","out":"transformed"}

move — change current location or approach a resource
{"type":"move","target":"resource-name or {"location": [x,y]}"}

createNote — create a persistent Note object
{"type":"createNote","value":"some data","out":"my_note"}
{"type":"createNote","value":"$variable","out":"new_note"}

createCollection — create a session-local Collection object
{"type":"createCollection","value":["$note1","$note2"],"out":"my_collection"}  # Collection of Note references
{"type":"createCollection","name":"research","value":[],"out":"papers"}  # Named empty Collection
{"type":"createCollection","value":"$note","out":"single_item"}  # Collection with one item

persist — mark Collection as persistent (saved to filesystem)
{"type":"persist","target":"$collection"}

load — retrieve a persistent Note or Collection by resource ID or name
{"type":"load","resource_id":"Note_123","out":"my_note"}
{"type":"load","resource_id":"Collection_5","out":"items"}
{"type":"load","resource_id":"shopping_list","out":"list"}  # Load by collection name

index (organize) — create embeddings index for a Collection
{"type":"index","source":"$collection","index_type":"semantic"}
{"type":"index","source":"$papers","index_type":"semantic","fields":{"title":"embed","content":"embed"}}  # optional fields

search — query an indexed Collection
{"type":"search","source":"$collection","query":"search text or $query","mode":"semantic","limit":5,"out":"results"}
{"type":"search","source":"$papers","query":"quantum computing","limit":3,"out":"top_papers"}  # search indexed Collection

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


# TOOLS - the tools available to the agent are listed below, make sure to use the correct name when calling apply.

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
    {"type": "createNote","value": "LLM cognitive agents 2025","out": "query"},
    {"type": "apply","target": "web-search","value": "$query","reason": "find recent work","out": "result1"},
    {"type": "createNote","value": "transformer architecture papers","out": "query2"},
    {"type": "apply","target": "web-search","value": "$query2","reason": "find architecture papers","out": "result2"},
    {"type": "createCollection","value": ["$result1","$result2"],"out": "research_collection"},
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
- "$variable" syntax resolves to the content stored in the Note/Collection
- Variables are cleared after plan completion

Argument Conventions:
- Literal strings/values: Use directly without "$" prefix (e.g., "tool-name", "hello")
- Variable references: Use "$variable" to resolve Note/Collection content
- Output names: In "out" fields, use plain strings without "$" (e.g., "out":"result")

CONSTRAINTS
- Use only primitives listed above.
- Variables must be created before use (createNote, createCollection, apply, search, or index may bind them).
- All JSON must be syntactically valid (no comments or trailing commas).
- Keep plans concise (recommended: 12 steps or fewer).
- Output only valid JSON.

EFFICIENCY RULES
- Apply tools directly to Notes when processing single items
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
    
    def generate_plan(self, goal: str, context: Dict) -> Dict:
        """
        Generate plan for infospace goal.
        
        Args:
            goal: String goal description
            context: Dict with available_tools, variables, state
            
        Returns:
            Plan dict with 'plan' array of actions, or error dict
        """
        self.logger.info(f"Generating infospace plan for goal: {goal}")
        
        # Format context into template
        formatted_template = self._format_template(goal, context)
        
        # Generate plan using LLM
        response = self.llm_client.generate(
            [formatted_template],
            max_tokens=2000,
            temperature=0.6,
            is_json=True
        )
        
        # Log response for debugging
        self.logger.info(f"LLM response length: {len(response.text) if response.text else 0} chars")
        if not isinstance(response.text, dict) and (not response.text or len(response.text) < 10):
            self.logger.error(f"LLM response too short or empty: '{response.text}'")
        
        if not isinstance(response.text, dict): # Parse JSON response
            plan = self._parse_plan_response(response.text)
        else:
            plan = response.text
        
        if plan.get('error'):
            self.logger.error(f"Plan generation failed: {plan['error']}")
            return plan
        
        # Validate plan structure
        validation = self._validate_plan(plan)
        if not validation['valid']:
            self.logger.error(f"Plan validation failed: {validation['reason']}")
            return {'error': validation['reason']}
        
        self.logger.info(f"Generated plan with {len(plan.get('plan', []))} steps")
        return plan
    
    def _format_template(self, goal: str, context: Dict) -> str:
        """Format template with goal and context"""
        variables = context.get('variables', {})
        
        # Format tools list from available_tools
        if self.available_tools:
            tools_text = '\n'.join([
                f"- {name}: {info.get('description', 'No description')}\n  Parameters: {info.get('parameters', 'none')}"
                for name, info in self.available_tools.items()
            ])
        else:
            tools_text = "None available"
        
        # Format variables
        vars_text = '\n'.join([f"- ${k}: {type(v).__name__}" for k, v in variables.items()]) if variables else "None"
        
        # Format context description
        context_text = context.get('situation', 'No additional context')
        
        # Fill in template
        filled = self.template
        filled = filled.replace('{{context}}', context_text)
        filled = filled.replace('{{tools}}', tools_text)
        filled = filled.replace('{{variables}}', vars_text)
        filled = filled.replace('{{goal}}', goal)
        
        return filled
    
    def _parse_plan_response(self, response_text: str) -> Dict:
        """Parse LLM response into plan dict"""
        # Check for empty response
        if not response_text or not response_text.strip():
            self.logger.error("LLM returned empty response")
            return {'error': 'Empty response from LLM'}
        
        # Remove markdown code fences if present
        text = response_text.strip()
        if text.startswith('```'):
            # Find first and last ```
            lines = text.split('\n')
            start_idx = 0
            end_idx = len(lines)
            
            for i, line in enumerate(lines):
                if line.startswith('```'):
                    if start_idx == 0:
                        start_idx = i + 1
                    else:
                        end_idx = i
                        break
            
            text = '\n'.join(lines[start_idx:end_idx])
        
        # Parse JSON
        parsed = json.loads(text)
        return parsed
    
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
            'if': ['condition', 'then'],
            'while': ['condition', 'body'],
            'wait': ['condition'],
            'say': ['target', 'value'],
            'display': ['target', 'value'],
            'think': ['value'],
        }
        
        required = required_fields.get(action_type, [])
        
        for field in required:
            if field not in action:
                return {'valid': False, 'reason': f'Missing required field: {field}'}
        
        return {'valid': True}


def verify_plan(plan_json: Any) -> bool:
    """
    Validate infospace plan structure.
    Public interface for plan validation (matches plan.py API).
    
    Args:
        plan_json: Plan as JSON string or dict
        
    Returns:
        True if valid, False otherwise
    """
    planner = InfospacePlanner(None)  # Validator only, no LLM needed
    validation = planner._validate_plan(plan_json if isinstance(plan_json, dict) else json.loads(plan_json))
    return validation['valid']

