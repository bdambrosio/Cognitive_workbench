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

Pattern: Searching the web for information
  When query-web is provided with an information-specific query:
  1. Query web to get results (already has filtered excerpts)
  2. Expand to get individual result items (optional, if you need to process each result individually)
  
  Example:
  {"type":"query-web","args":{"query":"KV cache replacement algorithms"},"out":"$results","expect":"should find papers"}
  {"type":"expand","target":"$results","out":"$items"} (optional)

Pattern: Fetching full text from URLs in web search results
  When query-web returns results with URLs and you want full text content:
  1. Query web to get results (already has filtered excerpts)
  2. Expand to get individual result items
  3. Use map with as-json to extract URL field from each item
  4. Use map with fetch-text to get full text from each URL
  
  Example:
  {"type":"query-web","args":{"query":"attention mechanism papers"},"out":"$results","expect":"should find papers"}
  {"type":"expand","target":"$results","out":"$items"}
  {"type":"map","target":"$items","operation":"as-json","args":{"field":"metadata.source_url"},"out":"$urls"}
  {"type":"map","target":"$urls","operation":"fetch-text","out":"$full_texts"}

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
  
  IMPORTANT: When a goal specifies a specific topic, theme, or aspect to focus on, ALWAYS include it in args.focus:
  - Goal: "summarize core block-rank algorithm" → use args.focus: "core block-rank algorithm"
  - Goal: "extract key findings about transformers" → use args.focus: "key findings about transformers"
  - Goal mentions what to extract/focus on → include that in args.focus
  
  The "expect" field documents the expected outcome. The "args.focus" field guides the tool's behavior for better precision.
  Both can reference the same topic - expect describes outcome, focus guides processing.

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

ALL tools that produce output MUST include 'out' field with $variable syntax.

Tools REQUIRING 'target', 'out', AND 'expect':
- summarize, relate, refine, assess, extract-entities, filter-by-predicate (LLM-based tools)
- search, load (infospace primitives)

Tools REQUIRING 'args.query', 'out', AND 'expect':
- query-web

Tools REQUIRING 'out' (expect optional):
- expand, map, index, as-json, as-markdown, text-find, matches

Tools WITHOUT 'out' (side effects only):
- persist, focus, say, display, think

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
- REQUIRED FIELDS: All output-producing tools MUST include 'out' field. Tools with uncertain outcomes (query-web, search, load, summarize, relate, refine, assess, extract-entities, filter-by-predicate) MUST also include 'expect' field.
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

