"""
Infospace Planner - Generates plans using information space primitives.

Provides templates and planning logic for cognitive/information processing tasks.
"""

import json
import logging
from typing import Dict, List, Any

logger = logging.getLogger(__name__)

# Primitives and Conditions Reference - Reusable for planning and validation
INFOSPACE_PRIMITIVES_REFERENCE = """# INFOSPACE PRIMITIVES AND CONDITIONS

# ARGUMENT TYPE CONVENTIONS:

Variables: Use "$variable" to reference Note/Collection content previously bound
Literals: Use plain strings/values (no $) for literal data or names
Names: Output variable names in "out" fields use plain strings (no $)
Tools/Resources: Can be literal "tool-name/resource-name" or "$variable" holding name
Resource IDs: Cannot be referenced directly - use "load" action first to bind to variable

# PRIMITIVES:

## map
Description: Apply operation to each item in a Collection
Input: target: $variable → Collection (must be Collection)
Output: out: $variable → Collection (same length or filtered)
Parameters:
  - target (required): $variable referencing Collection
  - operation (required): tool-name string or {"tool":"name","args":{}} object
  - filter_null (optional): boolean, exclude null/failed results (default: true)
  - out (required): $variable name for resulting Collection
Preconditions: target variable must be bound to Collection
Postconditions: out variable bound to Collection with transformed items (failures excluded by default)
Examples:
  {"type":"map","target":"$collection","operation":"tool-name","out":"$result_collection"}
  {"type":"map","target":"$collection","operation":"tool-name","filter_null":false,"out":"$all_results"}
  {"type":"map","target":"$new_items","operation":"add","args":{"target":"$existing_collection"},"out":"$existing_collection"}

## flatten
Description: Convert Collection to single Note by concatenating items
Input: target: $variable → Collection (must be Collection)
Output: out: $variable → Note (concatenated content)
Parameters:
  - target (required): $variable referencing Collection
  - separator (optional): string separator between items (default: newline)
  - out (required): $variable name for resulting Note
Preconditions: target variable must be bound to Collection
Postconditions: out variable bound to Note containing concatenated content
Examples:
  {"type":"flatten","target":"$collection","out":"$combined_note"}
  {"type":"flatten","target":"$collection","separator":"\\n---\\n","out":"$combined"}

## expand
Description: Expand a Note containing JSON array into a Collection of Notes
Input: target: $variable → Note (must contain JSON with array field)
Output: out: $variable → Collection (one Note per array item)
Parameters:
  - target (required): $variable referencing Note with JSON content
  - field (optional): string field name containing array (default: "results")
  - out (required): $variable name for resulting Collection
Preconditions: target variable must be bound to Note containing JSON with array field
Postconditions: out variable bound to Collection with one Note per array element
Examples:
  {"type":"expand","target":"$search_results","out":"$results_collection"}
  {"type":"expand","target":"$data","field":"items","out":"$items_collection"}

## coerce
Description: Convert data format or structure (whole-value operations only)
Input: target: $variable → Note
Output: out: $variable → Note (transformed content)
Parameters:
  - target (required): $variable referencing Note
  - operation (required): string operation name (e.g., "flatten")
  - out (required): $variable name for resulting Note
Preconditions: target variable must be bound to Note
Postconditions: out variable bound to Note with transformed content
Examples:
  {"type":"coerce","target":"$data","operation":"flatten","out":"$coerced"}

## add
Description: Add a Note to an existing Collection (mutates Collection in place)
Input: target: $variable → Collection, value: $variable → Note or literal
Output: out: $variable → Collection (same variable, mutated)
Parameters:
  - target (required): $variable referencing Collection
  - value (required): $variable referencing Note or literal value
  - out (required): $variable name (should match target)
Preconditions: target variable must be bound to Collection
Postconditions: Collection has new item added, out variable references same Collection
Examples:
  {"type":"add","target":"$collection","value":"$new_note","out":"$collection"}
  {"type":"add","target":"$dialog_history","value":"Hello user","out":"$dialog_history"}

## size
Description: Get item count of a Collection
Input: target: $variable → Collection
Output: out: $variable → Note (containing integer count)
Parameters:
  - target (required): $variable referencing Collection
  - out (required): $variable name for resulting Note
Preconditions: target variable must be bound to Collection
Postconditions: out variable bound to Note containing Collection size
Examples:
  {"type":"size","target":"$collection","out":"$count"}

## union
Description: Union of two Collections (A ∪ B) - all items from both, deduplicated
Input: target: $variable → Collection A, value: $variable → Collection B
Output: out: $variable → Collection (union result)
Parameters:
  - target (required): $variable referencing first Collection
  - value (required): $variable referencing second Collection
  - out (required): $variable name for resulting Collection
Preconditions: both variables must be bound to Collections
Postconditions: out variable bound to new Collection containing union
Examples:
  {"type":"union","target":"$collection1","value":"$collection2","out":"$combined"}

## intersection
Description: Intersection of two Collections (A ∩ B) - items in both
Input: target: $variable → Collection A, value: $variable → Collection B
Output: out: $variable → Collection (intersection result)
Parameters:
  - target (required): $variable referencing first Collection
  - value (required): $variable referencing second Collection
  - out (required): $variable name for resulting Collection
Preconditions: both variables must be bound to Collections
Postconditions: out variable bound to new Collection containing common items
Examples:
  {"type":"intersection","target":"$papers_2024","value":"$papers_2025","out":"$common_papers"}

## difference
Description: Difference of two Collections (A - B) - items in A but not in B
Input: target: $variable → Collection A, value: $variable → Collection B
Output: out: $variable → Collection (difference result)
Parameters:
  - target (required): $variable referencing first Collection
  - value (required): $variable referencing second Collection
  - out (required): $variable name for resulting Collection
Preconditions: both variables must be bound to Collections
Postconditions: out variable bound to new Collection containing items only in A
Examples:
  {"type":"difference","target":"$all_papers","value":"$reviewed_papers","out":"$unreviewed"}

## remove
Description: Remove a Note from a Collection (mutates Collection)
Input: target: $variable → Collection, value: $variable → Note or Note ID literal
Output: out: $variable → Collection (same variable, mutated)
Parameters:
  - target (required): $variable referencing Collection
  - value (required): $variable referencing Note or literal Note ID
  - out (required): $variable name (should match target)
Preconditions: target variable must be bound to Collection
Postconditions: Collection has item removed, out variable references same Collection
Examples:
  {"type":"remove","target":"$collection","value":"$note_id","out":"$collection"}
  {"type":"remove","target":"$collection","value":"Note_123","out":"$collection"}

## create-note
Description: Create a persistent Note object and bind to variable
Input: value: literal or $variable → any content
Output: out: $variable → Note (newly created)
Parameters:
  - value (required): literal value or $variable referencing content
  - out (required): $variable name for resulting Note
Preconditions: none (creates new Note)
Postconditions: out variable bound to new Note with persistent ID
Examples:
  {"type":"create-note","value":"some data","out":"$my_note"}
  {"type":"create-note","value":"$variable","out":"$new_note"}

## create-collection
Description: Create a session-local Collection object and bind to variable
Input: value: array of $variables → [$note1, $note2, ...] or empty array
Output: out: $variable → Collection (newly created)
Parameters:
  - value (required): array of $variables or empty array
  - name (optional): string name for named Collection
  - out (required): $variable name for resulting Collection
Preconditions: referenced $variables must be bound to Notes
Postconditions: out variable bound to new Collection containing Note references
Examples:
  {"type":"create-collection","value":["$note1","$note2"],"out":"$my_collection"}
  {"type":"create-collection","name":"research","value":[],"out":"$papers"}
  {"type":"create-collection","value":"$note","out":"$single_item"}

## persist
Description: Mark Note or Collection as persistent (saved to filesystem)
Input: target: $variable → Note or Collection
Output: none (mutates resource)
Parameters:
  - target (required): $variable referencing Note or Collection
Preconditions: target variable must be bound to Note or Collection
Postconditions: Note or Collection marked persistent, saved to filesystem
Examples:
  {"type":"persist","target":"$collection"}
  {"type":"persist","target":"$note"}

## load
Description: Retrieve a persistent Note or Collection by resource ID or name
Input: resource_id: literal string → resource ID or name
Output: out: $variable → Note or Collection
Parameters:
  - resource_id (required): string resource ID (e.g., "Note_123") or name
  - out (required): $variable name for resulting Note/Collection
  - expect (required): string describing expected content
Preconditions: resource must exist in persistent storage
Postconditions: out variable bound to loaded Note or Collection
Examples:
  {"type":"load","resource_id":"Note_123","out":"$my_note","expect":"should contain previous data"}
  {"type":"load","resource_id":"papers","out":"$papers","expect":"should have saved papers"}

## index
Description: Build an embedding index for a Collection
Input: source: $variable → Collection
Output: none (mutates Collection)
Parameters:
  - source (required): $variable referencing Collection
  - index_type (optional): string type (default: "semantic")
  - fields (optional): dict mapping field names to "embed"
Preconditions: source variable must be bound to Collection
Postconditions: Collection has embedding index built
Examples:
  {"type":"index","source":"$collection","index_type":"semantic"}
  {"type":"index","source":"$papers","index_type":"semantic","fields":{"title":"embed","content":"embed"}}

## search
Description: Query an indexed Collection
Input: source: $variable → Collection (must be indexed), query: string or $variable
Output: out: $variable → Collection (matching items)
Parameters:
  - source (required): $variable referencing indexed Collection
  - query (required): string search text or $variable referencing query
  - mode (optional): string mode (default: "semantic")
  - limit (optional): integer max results (default: 10)
  - out (required): $variable name for resulting Collection
  - expect (required): string describing expected results
Preconditions: source Collection must be indexed (call index first)
Postconditions: out variable bound to Collection containing matching Notes
Examples:
  {"type":"search","source":"$collection","query":"search text","mode":"semantic","limit":5,"out":"$results","expect":"should return 3-5 relevant items"}

## focus
Description: Change current location or approach a resource
Input: target: literal string or object → resource name or location
Output: none
Parameters:
  - target (required): string resource name or {"location": [x,y]} object
Preconditions: none
Postconditions: agent position updated
Examples:
  {"type":"focus","target":"resource-name"}
  {"type":"focus","target":{"location": [10, 20]}}

## if
Description: Conditional branch
Input: condition: condition object → boolean, then: array of actions, else: optional array
Output: executes actions in then or else branch
Parameters:
  - condition (required): condition object evaluating to boolean
  - then (required): array of action objects
  - else (optional): array of action objects
Preconditions: variables referenced in condition must be bound
Postconditions: actions in chosen branch executed
Examples:
  {"type":"if","condition":{"type":"has_value","target":"$results"},"then":[/* steps */],"else":[/* optional steps */]}

## while
Description: Loop until condition false
Input: condition: condition object → boolean, body: array of actions
Output: executes body repeatedly until condition false
Parameters:
  - condition (required): condition object evaluating to boolean
  - body (required): array of action objects
  - max_iterations (optional): integer max loops (default: 100)
Preconditions: variables referenced in condition must be bound
Postconditions: body executed until condition false or max_iterations reached
Examples:
  {"type":"while","condition":{"type":"has_value","target":"$results"},"body":[/* steps */],"max_iterations":10}

## wait
Description: Pause until condition true
Input: condition: condition object → boolean
Output: blocks until condition true or timeout
Parameters:
  - condition (required): condition object evaluating to boolean
  - timeout (optional): integer seconds (default: 30)
Preconditions: variables referenced in condition must be bound
Postconditions: execution resumes when condition true or timeout
Examples:
  {"type":"wait","condition":{"type":"has_value","target":"$results"},"timeout":30}

## say
Description: Produce output (inline display)
Input: target: literal "user", value: literal or $variable
Output: none (side effect: displays to user)
Parameters:
  - target (required): string "user"
  - value (required): literal string or $variable referencing content
Preconditions: if value is $variable, it must be bound
Postconditions: message displayed to user
Examples:
  {"type":"say","target":"user","value":"literal text or $variable"}

## display
Description: Show formatted content in popup (for documents/formatted output)
Input: value: $variable → Note (or target: $variable, both accepted)
Output: none (side effect: displays formatted content)
Parameters:
  - value (required): $variable referencing Note with formatted content (or use target)
Preconditions: value/target variable must be bound to Note
Postconditions: formatted content displayed in popup to user
Examples:
  {"type":"display","value":"$formatted_note"}
  {"type":"display","target":"$formatted_note"}  # Also accepted

## think
Description: Internal note (logged only)
Input: value: literal or $variable (or target: $variable, both accepted)
Output: none (side effect: logged)
Parameters:
  - value (required): literal string or $variable referencing content (or use target)
Preconditions: if value/target is $variable, it must be bound
Postconditions: thought logged internally
Examples:
  {"type":"think","value":"thought text or $variable"}
  {"type":"think","target":"$variable"}  # Also accepted

# CONDITIONS:

All conditions evaluate to boolean. Uniform form:
{"type": "<condition_type>", "target": "$variable", "field?": "optional_field", "value?": "<literal or variable>"}

## Variable State Conditions (work on both Note and Collection):
- bound: {"type": "bound", "target": "$var"} - true if $var exists
- notbound: {"type": "notbound", "target": "$var"} - true if $var doesn't exist
- has_value: {"type": "has_value", "target": "$var"} - true if $var is truthy
- empty: {"type": "empty", "target": "$var"} - true if Note is falsy/empty or Collection has 0 items

## Value Comparison Conditions (target must be Note with comparable content):
- equals: {"type": "equals", "target": "$var", "value": "expected"}
- not_equals: {"type": "not_equals", "target": "$var", "value": "unwanted"}
- greater_than: {"type": "greater_than", "target": "$score", "value": 0.8} (numeric)
- less_than: {"type": "less_than", "target": "$count", "value": 100} (numeric)
- gte: {"type": "gte", "target": "$score", "value": 0.7} (numeric)
- lte: {"type": "lte", "target": "$size", "value": 1000} (numeric)

## Membership Conditions (target can be Note or Collection):
- contains: {"type": "contains", "target": "$text", "value": "keyword"} (substring/element for Note, Note ID membership for Collection)
- not_contains: {"type": "not_contains", "target": "$tags", "value": "spam"}

## Pattern Matching (target must be Note with string content):
- matches_pattern: {"type": "matches_pattern", "target": "$text", "pattern": "regex_pattern"}

## Tool-based Conditions (target can be Note or Collection):
- tool_condition: {"type": "tool_condition", "tool": "has_field", "target": "$data", "args": {"field": "urls"}} (tool returns boolean)
"""

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

{primitives_reference}

# COMMON PATTERNS:

Pattern: Persisting Collections
  When you need to save a Collection to filesystem for persistence:
  1. Create Collection with create-collection
  2. Mark it persistent with persist
  3. Optionally use name parameter for named Collection
  
  Example - Save research findings:
  {"type":"create-collection","value":["$methodology_summary","$gaps_analysis"],"name":"constitutional-ai-findings","out":"$findings"}
  {"type":"persist","target":"$findings"}

Pattern: Downloading PDFs from web search results
  When query-web returns results with JSON objects containing URLs:
  1. Query web to get results
  2. Expand to get individual result items
  3. Use map with as-json to extract URL field from each item
  4. Use map with download-pdf to download each PDF
  
  Example:
  {"type":"query-web","args":{"query":"attention mechanism papers"},"out":"$results","expect":"should find papers"}
  {"type":"expand","target":"$results","out":"$items"}
  {"type":"map","target":"$items","operation":"as-json","args":{"field":"url"},"out":"$urls"}
  {"type":"map","target":"$urls","operation":"download-pdf","out":"$pdfs"}

Pattern: Adding filtered items to existing Collection
  When you need to search, filter, then add results to a persistent collection:
  1. Load the persistent collection
  2. Search/expand to get new items
  3. Optionally filter with map + tool
  4. Use map with add to append each item
  
  Example - Add new research papers to existing collection:
  {"type":"load","resource_id":"papers","out":"$papers","expect":"should have existing paper collection"}
  {"type":"query-web","args":{"query":"LLM agents 2025"},"out":"$results","expect":"should find recent papers"}
  {"type":"expand","target":"$results","out":"$new_papers"}
  {"type":"map","target":"$new_papers","operation":"add","args":{"target":"$papers"},"out":"$papers"}
  {"type":"persist","target":"$papers"}

Pattern: Multi-item tool application
  When you need to apply a tool to *two or more* Notes (e.g., compare, analyze together):
  1. Create a Collection containing the Notes
  2. Apply the tool to the Collection (tool receives all items)
  
  Example - Compare two search results:
  {"type":"create-collection","value":["$results_2024","$results_2025"],"out":"$both_years"}
  {"type":"relate","target":"$both_years","out":"$comparison"}
  
  Tools that work with Collections: relate, summarize, extract-entities

Pattern: Optional tool arguments
  Many tools accept optional "focus" or "mode" parameters via "args" field:
  {"type":"summarize","target":"$doc","args":{"focus":"key findings"},"out":"$summary"}
  
  Using focus is optional - tools work generically without it, but focus can improve precision.

Pattern: Single vs. Multiple Item Processing
  Collections are for handling MULTIPLE Notes together. For single Notes, use tools directly.
  
  AVOID - Unnecessary Collection wrapper:
  {"type":"create-collection","value":["$single_result"],"out":"$wrapper"}
  {"type":"summarize","target":"$wrapper","out":"$summary"}
  
  PREFER - Direct tool use:
  {"type":"summarize","target":"$single_result","out":"$summary"}
  
  Use Collections ONLY when you have 2+ Notes to process together:
  {"type":"create-collection","value":["$result1","$result2","$result3"],"out":"$multiple"}
  {"type":"summarize","target":"$multiple","out":"$summary"}

Pattern: Universal LLM Transformations (refine, assess)
  For ad-hoc transformations without specialized tools, use refine with natural language instructions.
  PREFER specialized tools when available (faster, cheaper, deterministic).
  USE refine for novel/exploratory operations.
  USE assess for complex boolean conditions in control flow.

# TOOLS - use tool names directly as action types. Tools available to the agent are listed below.

# REQUIRED FIELDS FOR TOOLS:

Tools REQUIRING 'expect' field (uncertain outcomes - must include expect):
- query-web, search, load (infospace primitives)
- summarize, relate, refine, assess, extract-entities, filter-by-predicate (LLM-based tools)

Tools with OPTIONAL 'expect' field:
- expand, map, index, as-json, as-markdown, text-find, matches

Tools WITHOUT 'expect' field (deterministic/internal):
- create-note, create-collection, add, flatten, coerce, persist, focus, say, display, think

IMPORTANT: Always include 'expect' field for tools listed above as REQUIRING it. Example:
{"type":"refine","target":"$text","args":{"instruction":"extract citations"},"out":"$citations","expect":"should find DOI list"}

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
    {"type": "create-note","value": "LLM cognitive agents 2025","out": "$query"},
    {"type": "query-web","args":{"query":"$query"},"out": "$result1","expect":"should find recent papers on LLM agents"},
    {"type": "create-note","value": "transformer architecture papers","out": "$query2"},
    {"type": "query-web","args":{"query":"$query2"},"out": "$result2","expect":"should find transformer papers"},
    {"type": "create-collection","value": ["$result1","$result2"],"out": "$research_collection"},
    {"type": "index","source": "$research_collection","index_type": "semantic","fields": {"title":"embed","content":"embed"}},
    {"type": "search","source": "$research_collection","query":"attention mechanisms","mode":"semantic","limit":5,"out":"$top_papers","expect":"should find 5 papers on attention"},
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
- Variables must be created before use (create-note, create-collection, tools, search, or index may bind them).
- All JSON must be syntactically valid (no comments or trailing commas).
- Keep plans concise (recommended: 12 steps or fewer).
- Output only valid JSON.
- REQUIRED FIELDS: Tools requiring 'expect' (query-web, search, load, summarize, relate, refine, assess, extract-entities, filter-by-predicate) MUST include 'expect' field in every action.
- Note: 'display' and 'think' accept either 'value' or 'target' (both work).

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
        self.template = self._build_template()
    
    def _build_template(self) -> str:
        """
        Build the complete template with dynamically loaded tool definitions.
        
        Returns:
            Complete template string with tools section populated
        """
        tools_section = self._build_tools_section()
        template = INFOSPACE_PLAN_TEMPLATE.replace("{{tools}}", tools_section)
        template = template.replace("{primitives_reference}", INFOSPACE_PRIMITIVES_REFERENCE)
        return template
    
    def _build_tools_section(self) -> str:
        """
        Build tools section from loaded tool metadata.
        Format matches primitive definitions for consistency.
        
        Returns:
            Formatted tools section string
        """
        if not self.available_tools:
            return "# No tools currently available"
        
        lines = []
        for tool_name in sorted(self.available_tools.keys()):
            tool = self.available_tools[tool_name]
            description = tool.get('description', 'No description')
            examples = tool.get('examples', [])
            
            # Tool header: name — description
            lines.append(f"{tool_name} — {description}")
            
            # Add examples
            for example in examples:
                lines.append(example)
            
            # Blank line between tools
            lines.append("")
        
        return "\n".join(lines)
    
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
            'map': ['target', 'operation', 'out'],
            'if': ['condition', 'then'],
            'while': ['condition', 'body'],
            'wait': ['condition'],
            'say': ['target', 'value'],
            'display': ['value'],  # Accepts target or value (both allowed)
            'think': ['value'],  # Accepts target or value (both allowed)
        }
        
        required = required_fields.get(action_type, [])
        
        # If action_type is not a known primitive, treat it as a tool name
        if not required:
            # Validate tool name exists in available_tools if provided
            if self.available_tools and action_type not in self.available_tools:
                return {'valid': False, 'reason': f'Unknown tool name: {action_type}'}
            
            # Tool names should have 'target' and 'out' fields per template examples
            # Exception: query-web uses args.query instead of target
            if action_type == 'query-web':
                if 'args' not in action or 'query' not in action.get('args', {}):
                    return {'valid': False, 'reason': f'Missing required field: args.query'}
                if 'out' not in action:
                    return {'valid': False, 'reason': f'Missing required field: out'}
                if 'expect' not in action:
                    return {'valid': False, 'reason': f'Missing required field: expect'}
                # Validate out field uses $variable syntax
                out_val = action.get('out', '')
                if not isinstance(out_val, str) or not out_val.startswith('$'):
                    return {'valid': False, 'reason': f'out field must use $variable syntax, got: {out_val}'}
                return {'valid': True}
            
            # LLM-based tools require expect
            llm_tools = ['summarize', 'relate', 'refine', 'assess', 'extract-entities', 'filter-by-predicate']
            if action_type in llm_tools:
                if 'target' not in action:
                    return {'valid': False, 'reason': f'Missing required field: target'}
                if 'out' not in action:
                    return {'valid': False, 'reason': f'Missing required field: out'}
                if 'expect' not in action:
                    return {'valid': False, 'reason': f'Missing required field: expect'}
                out_val = action.get('out', '')
                if not isinstance(out_val, str) or not out_val.startswith('$'):
                    return {'valid': False, 'reason': f'out field must use $variable syntax, got: {out_val}'}
                return {'valid': True}
            
            # Other tools need target and out
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
                # Special handling: display and think accept either value or target
                if field == 'value' and action_type in ['display', 'think']:
                    if 'target' not in action:
                        return {'valid': False, 'reason': f'Missing required field: value or target'}
                else:
                    return {'valid': False, 'reason': f'Missing required field: {field}'}
        
        return {'valid': True}
    
    def verify_plan(self, plan: Dict) -> Dict:
        """
        Validate plan structure.
        Public instance method for plan validation.
        
        Args:
            plan: Plan dict with 'plan' key
            
        Returns:
            Dict with 'valid' (bool) and 'reason' (str if invalid)
        """
        return self._validate_plan(plan)

