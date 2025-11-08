"""
Infospace Planner - Generates plans using information space primitives.

Provides templates and planning logic for cognitive/information processing tasks.
"""

import json
import logging
from typing import Dict, List, Any

logger = logging.getLogger(__name__)

# Phase 1 & 2 Template - Core + Data Operations
INFOSPACE_PLAN_TEMPLATE = """TASK: Generate a JSON format plan for the Goal below using the ACTIONS, CONDITIONS, and TOOLS listed below.

#Goal:
{{goal}}

#OUTPUT: only valid JSON — no reasoning, no prose, no code fences.

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
- Use only primitives / tools listed in the AVAILABLE ACTIONS and TOOLS sections.
- Variables must be created before use (create-note, create-collection, tools, search, or index may bind them).
- All JSON must be syntactically valid (no comments or trailing commas).
- Keep plans concise (recommended: 12 steps or fewer).
- Output only valid JSON.
- REQUIRED FIELDS: All output-producing tools MUST include 'out' field. Tools with uncertain outcomes (query-web, search, load, summarize, relate, refine, assess, extract-entities, filter-by-predicate) MUST also include 'expect' field.
- Note: 'display' and 'think' accept either 'value' or 'target' (both work).

EFFICIENCY RULES
- Use tools directly on Notes when processing single items
- Create Collections only when you need to process 2+ Notes together
- REMEMBER: expand, refine, as-json work on Notes ONLY, not Collections
- Use map to apply Note operations to each item in a Collection

# COMMON PATTERNS:

Pattern: Persisting Collections
Pattern: Persist Collection
  create-collection → persist → (optional: use name parameter for named Collection)

  Example - Save research findings:
  {"type":"create-collection","value":["$methodology_summary","$gaps_analysis"],"name":"constitutional-ai-findings","out":"$findings"}
  {"type":"persist","target":"$findings"}

Pattern: Searching the web for information
  query-web → expand(Note) → map(Collection) (optional: use map with as-json to extract URL field from each item) → map with fetch-text to get full text from each URL
  
  Example:
  {"type":"query-web","args":{"query":"KV cache replacement algorithms"},"out":"$results","expect":"should find papers"}
  {"type":"expand","target":"$results","out":"$items"} (optional)

Pattern: Fetching full text from URLs in web search results
  query-web → expand → map with as-json → map with fetch-text
  
  Example:
  {"type":"query-web","args":{"query":"attention mechanism papers"},"out":"$results","expect":"should find papers"}
  {"type":"expand","target":"$results","out":"$items"}
  {"type":"map","target":"$items","operation":"as-json","args":{"field":"metadata.source_url"},"out":"$urls"}
  {"type":"map","target":"$urls","operation":"fetch-text","out":"$full_texts"}

Pattern: Working with Multiple Search Results
  When combining results from multiple queries:
  
  ❌ WRONG - Cannot expand a Collection:
  {"type":"query-web","args":{"query":"..."},"out":"$results1","expect":"..."},
  {"type":"query-web","args":{"query":"..."},"out":"$results2","expect":"..."},
  {"type":"create-collection","value":["$results1","$results2"],"out":"$combined"},
  {"type":"expand","target":"$combined","out":"$items"}  // ERROR: expand needs Note, not Collection
  
  ✅ RIGHT - Expand each Note first:
  {"type":"query-web","args":{"query":"..."},"out":"$results1","expect":"..."},
  {"type":"expand","target":"$results1","out":"$items1"},
  {"type":"query-web","args":{"query":"..."},"out":"$results2","expect":"..."},
  {"type":"expand","target":"$results2","out":"$items2"},
  {"type":"union","target":"$items1","value":"$items2","out":"$all_items"}
  
Pattern: Adding filtered items to existing Collection
  load → query-web → expand → map with add to append each item → persist
  
  Example - Add new research papers to existing collection:
  {"type":"load","resource_id":"papers","out":"$papers","expect":"should have existing paper collection"}
  {"type":"query-web","args":{"query":"LLM agents 2025"},"out":"$results","expect":"should find recent papers"}
  {"type":"expand","target":"$results","out":"$new_papers"}
  {"type":"map","target":"$new_papers","operation":"add","args":{"target":"$papers"},"out":"$papers"}

Pattern: Semantic search with chunking (index/search)
  index → search → (optional: use return_mode to control what search returns)

  Example - Find relevant passages:
  {"type":"index","source":"$documents"}
  {"type":"search","source":"$documents","query":"transformer attention","return_mode":"chunks","limit":10,"out":"$passages","expect":"should return relevant text excerpts"}
  
  Example - Find relevant documents:
  {"type":"index","source":"$papers"}
  {"type":"search","source":"$papers","query":"block-rank algorithm","return_mode":"notes","limit":5,"out":"$relevant_papers","expect":"should return papers discussing block-rank"}

Pattern: Multi-item tool application
  When you need to apply a tool to two or more Notes together (e.g. compare, analyze together), create a Collection.
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
  Use refine for ad-hoc transformations without specialized tools.
  Use assess for complex boolean conditions.  
  refine → assess → (optional: use target to apply to Collection)

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
│ map, flatten            │  ❌   │     ✓      │
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

Research Example (working with multiple search results):
{
  "plan": [
    {"type": "query-web","args":{"query":"machine learning papers 2024"},"out": "$results1","expect":"should find ML papers"},
    {"type": "expand","target": "$results1","out": "$items1"},
    {"type": "query-web","args":{"query":"neural network papers 2024"},"out": "$results2","expect":"should find NN papers"},
    {"type": "expand","target": "$results2","out": "$items2"},
    {"type": "union","target": "$items1","value": "$items2","out": "$all_items"},
    {"type": "flatten","target": "$all_items","out": "$combined_text"},
    {"type": "summarize","target": "$combined_text","out": "$summary","expect":"should provide overview of both topics"},
    {"type": "display","value": "$summary"}
  ]
}


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

