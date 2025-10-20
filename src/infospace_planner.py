"""
Infospace Planner - Generates plans using information space primitives.

Provides templates and planning logic for cognitive/information processing tasks.
"""

import json
import logging
from typing import Dict, List, Any

logger = logging.getLogger(__name__)

# Phase 1 Template - Core primitives
INFOSPACE_PLAN_TEMPLATE = """Task: Decompose your goal into a plan using information space primitives.
Output: only valid JSON – no prose, no code fences.

Meta-spec (strictly observe these rules in your output):
{
  "primitives": [
    "scan", "use", "move", "store", "index", "search",
    "if", "while", "wait", "say", "think"
  ],
  "conditions": [
    "bound", "notbound", "has_value", "empty", "equals",
    "near", "can_see"
  ],
  "required_fields": {
    "scan": ["type", "target", "out", "prediction"],
    "use": ["type", "target", "reason", "prediction"],
    "move": ["type", "target"],
    "store_variable": ["type", "value", "variable"],
    "store_collection": ["type", "target", "collection"],
    "index": ["type", "source", "store_name", "index_type", "fields"],
    "search": ["type", "store_name", "query", "mode", "limit", "out", "prediction"],
    "if": ["type", "condition", "then"],
    "while": ["type", "condition", "body"],
    "wait": ["type", "condition"],
    "say": ["type", "target", "value"],
    "think": ["type", "value"]
  },
  "variables": {"syntax": "$name", "must_be_bound_before_use": true},
  "max_steps": 12
}

Action Schemas:

scan - Locate resource/skill by name or interface:
{
  "type": "scan",
  "target": "skill-name or interface-type",
  "out": "variable_name",
  "prediction": "will find matching skill"
}

use - Apply skill to input data:
{
  "type": "use",
  "target": "skill-name or $variable",
  "value": "input text",  // optional
  "reason": "brief purpose",
  "out": "result_variable",  // optional
  "prediction": "expected outcome"
}

move - Navigate to resource location:
{
  "type": "move",
  "target": "resource-name or $variable"
}

store - Persist to variable:
{
  "type": "store",
  "value": "$computed_value or literal",
  "variable": "name"
}

store - Append to collection:
{
  "type": "store",
  "target": "$item",
  "collection": "collection_name"
}

index - Create searchable store with embeddings:
{
  "type": "index",
  "source": "$data_array",
  "store_name": "my_memory",
  "index_type": "semantic",
  "fields": {
    "content": "embed",
    "title": "embed",
    "metadata": "store"
  }
}

search - Query indexed store:
{
  "type": "search",
  "store_name": "my_memory",
  "query": "search text",
  "mode": "semantic",
  "limit": 5,
  "threshold": 0.0,
  "out": "results",
  "prediction": "will find relevant items"
}

if - Conditional branching:
{
  "type": "if",
  "condition": {"type": "bound", "target": "results"},
  "then": [/* steps */],
  "else": [/* steps */]  // optional
}

while - Loop with condition:
{
  "type": "while",
  "condition": {"type": "notbound", "target": "done"},
  "body": [/* steps */],
  "max_iterations": 10  // optional, default 100
}

wait - Block until condition satisfied:
{
  "type": "wait",
  "condition": {"type": "bound", "target": "user_input"},
  "timeout": 60  // optional, default 60
}

say - Output to user or agent:
{
  "type": "say",
  "target": "user",
  "value": "$message or literal text"
}

think - Internal reasoning (for transparency):
{
  "type": "think",
  "value": "thought process"
}

Condition Types:
- bound: {"type": "bound", "target": "variable_name"}
- notbound: {"type": "notbound", "target": "variable_name"}
- has_value: {"type": "has_value", "target": "$variable"}
- empty: {"type": "empty", "target": "$variable"}
- equals: {"type": "equals", "target": "$var", "value": "expected"}

Example Workflow - Research and Store:
{
  "plan": [
    {
      "type": "scan",
      "target": "web-search",
      "out": "search_tool",
      "prediction": "will find search capability"
    },
    {
      "type": "use",
      "target": "$search_tool",
      "value": "LLM cognitive agents 2025",
      "reason": "find recent articles",
      "out": "search_results",
      "prediction": "will return article URLs"
    },
    {
      "type": "index",
      "source": "$search_results",
      "store_name": "research_memory",
      "index_type": "semantic",
      "fields": {
        "title": "embed",
        "content": "embed"
      }
    },
    {
      "type": "say",
      "target": "user",
      "value": "Research complete and indexed"
    }
  ]
}

Example with Search and Conditional:
{
  "plan": [
    {
      "type": "search",
      "store_name": "research_memory",
      "query": "cognitive persistence",
      "mode": "semantic",
      "limit": 3,
      "out": "prior_research",
      "prediction": "will find related past work"
    },
    {
      "type": "if",
      "condition": {"type": "has_value", "target": "$prior_research"},
      "then": [
        {
          "type": "think",
          "value": "Found related prior work to build on"
        },
        {
          "type": "use",
          "target": "synthesizer",
          "value": "$prior_research",
          "reason": "combine findings",
          "out": "summary"
        }
      ],
      "else": [
        {
          "type": "think",
          "value": "No prior work found, starting fresh"
        }
      ]
    },
    {
      "type": "say",
      "target": "user",
      "value": "$summary"
    }
  ]
}

CURRENT CONTEXT:
{{context}}

AVAILABLE SKILLS:
{{skills}}

AVAILABLE STORES:
{{stores}}

CURRENT VARIABLES:
{{variables}}

GOAL:
{{goal}}

CONSTRAINTS:
- Use only primitives listed above
- Variables must be bound before use (use scan/use/search to bind)
- Stores must be indexed before search
- Maximum 12 steps total (including nested if/while bodies)
- Output only valid JSON, no markdown, no prose
"""


class InfospacePlanner:
    """
    Generates plans for information space operations.
    Uses infospace-specific templates and primitives.
    """
    
    def __init__(self, llm_client, logger=None):
        """
        Initialize infospace planner.
        
        Args:
            llm_client: LLM client for plan generation
            logger: Optional logger instance
        """
        self.llm_client = llm_client
        self.logger = logger or logging.getLogger(__name__)
        self.template = INFOSPACE_PLAN_TEMPLATE
    
    def generate_plan(self, goal: str, context: Dict) -> Dict:
        """
        Generate plan for infospace goal.
        
        Args:
            goal: String goal description
            context: Dict with available_skills, stores, variables, state
            
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
            temperature=0.7
        )
        
        # Parse JSON response
        plan = self._parse_plan_response(response.text)
        
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
        skills = context.get('available_skills', [])
        stores = context.get('available_stores', [])
        variables = context.get('variables', {})
        
        # Format skills list
        skills_text = '\n'.join([f"- {skill}" for skill in skills]) if skills else "None visible (use scan)"
        
        # Format stores list
        stores_text = '\n'.join([f"- {store}" for store in stores]) if stores else "None yet (create with index)"
        
        # Format variables
        vars_text = '\n'.join([f"- ${k}: {type(v).__name__}" for k, v in variables.items()]) if variables else "None"
        
        # Format context description
        context_text = context.get('situation', 'No additional context')
        
        # Fill in template
        filled = self.template
        filled = filled.replace('{{context}}', context_text)
        filled = filled.replace('{{skills}}', skills_text)
        filled = filled.replace('{{stores}}', stores_text)
        filled = filled.replace('{{variables}}', vars_text)
        filled = filled.replace('{{goal}}', goal)
        
        return filled
    
    def _parse_plan_response(self, response_text: str) -> Dict:
        """Parse LLM response into plan dict"""
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
            return {'valid': False, 'reason': f'Plan too long: {len(actions)} steps (max 12)'}
        
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
            'scan': ['target', 'out', 'prediction'],
            'use': ['target', 'reason', 'prediction'],
            'move': ['target'],
            'store': [],  # Either variable or collection required
            'index': ['source', 'store_name', 'index_type', 'fields'],
            'search': ['store_name', 'query', 'mode', 'limit', 'out', 'prediction'],
            'if': ['condition', 'then'],
            'while': ['condition', 'body'],
            'wait': ['condition'],
            'say': ['target', 'value'],
            'think': ['value']
        }
        
        # Special case for store
        if action_type == 'store':
            if 'variable' not in action and 'collection' not in action:
                return {'valid': False, 'reason': 'store requires variable or collection'}
            return {'valid': True}
        
        required = required_fields.get(action_type, [])
        
        for field in required:
            if field not in action:
                return {'valid': False, 'reason': f'Missing required field: {field}'}
        
        return {'valid': True}

