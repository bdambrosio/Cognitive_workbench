"""
Unified Planner - Single planner for both infospace and physical worlds.

Handles plan generation with pluggable primitive sets and tool libraries.
"""

import json
import logging
from typing import Dict, Any, Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class UnifiedPlanner:
    """
    Unified planner that handles both infospace and physical world planning.
    
    Swaps primitive sets, tool libraries, and templates based on world type.
    """
    
    def __init__(self, 
                 llm_client,
                 world_type: str,  # 'infospace' or 'physical'
                 map_name: str,
                 logger_instance=None):
        """
        Initialize unified planner.
        
        Args:
            llm_client: LLM client for plan generation
            world_type: 'infospace' or 'physical'
            map_name: Name of map (for tool library path)
            logger_instance: Optional logger
        """
        self.llm_client = llm_client
        self.world_type = world_type
        self.map_name = map_name
        self.logger = logger_instance or logger
        
        # Load tools for this map
        self.available_tools = self._load_tools()
        
        # Load appropriate template
        self.template = self._load_template()
        
        self.logger.info(f"UnifiedPlanner initialized: {world_type} world, map={map_name}, {len(self.available_tools)} tools")
    
    def _load_tools(self) -> Dict[str, Dict]:
        """Load base tools + map-specific tools (map overrides base on name collision)."""
        from utils.tool_loader import load_tools
        
        maps_base = Path(__file__).parent / 'maps'
        tools = {}
        
        # Load base tools first
        base_tools_dir = maps_base / 'tools'
        if base_tools_dir.exists():
            self.logger.info(f"Loading base tools from: {base_tools_dir}")
            tools.update(load_tools(str(base_tools_dir)))
        else:
            self.logger.warning(f"Base tools directory not found: {base_tools_dir}")
        
        # Load map-specific tools (can override base)
        map_tools_dir = maps_base / self.map_name / 'tools'
        if map_tools_dir.exists():
            self.logger.info(f"Loading map-specific tools from: {map_tools_dir}")
            map_tools = load_tools(str(map_tools_dir))
            if map_tools:
                # Log any overrides
                for name in map_tools:
                    if name in tools:
                        self.logger.info(f"Map tool '{name}' overrides base tool")
                tools.update(map_tools)
        
        self.logger.info(f"Total tools loaded: {len(tools)} (base + map-specific)")
        return tools
    
    def _load_template(self) -> str:
        """Load appropriate planning template."""
        if self.world_type == 'infospace':
            from infospace_planner import INFOSPACE_PLAN_TEMPLATE
            return INFOSPACE_PLAN_TEMPLATE
        else:
            from templates import PLAN_TEMPLATE
            return PLAN_TEMPLATE
    
    def generate_plan(self, goal: str, context: Dict) -> Dict:
        """
        Generate plan for goal.
        
        Args:
            goal: String goal description
            context: Dict with character state, situation, variables, etc.
            
        Returns:
            Plan dict with 'plan' array of actions, or error dict
        """
        self.logger.info(f"Generating {self.world_type} plan for goal: {goal}")
        
        # Route to appropriate planner
        if self.world_type == 'infospace':
            return self._generate_infospace_plan(goal, context)
        else:
            return self._generate_physical_plan(goal, context)
    
    def _generate_infospace_plan(self, goal: str, context: Dict) -> Dict:
        """Generate infospace plan."""
        # Format tools for template
        tools_formatted = self._format_tools()
        
        # Fill template
        template_filled = self.template.replace('{{goal}}', goal)
        template_filled = template_filled.replace('{{tools}}', tools_formatted)
        
        # Generate plan using LLM
        response = self.llm_client.generate(
            [template_filled],
            max_tokens=2000,
            temperature=0.6,
            is_json=True
        )
        
        # Parse response
        if isinstance(response.text, dict):
            plan = response.text
        else:
            plan = self._parse_plan_response(response.text)
        
        if plan.get('error'):
            self.logger.error(f"Plan generation failed: {plan.get('error')}")
            return plan
        
        self.logger.info(f"Generated plan with {len(plan.get('plan', []))} steps")
        return plan
    
    def _generate_physical_plan(self, goal: str, context: Dict) -> Dict:
        """Generate physical world plan."""
        from plan import generate_plan_with_context
        
        # Extract character from context
        character = context.get('character')
        if not character:
            return {'error': 'No character in context'}
        
        # Format tools for template (add to context)
        tools_formatted = self._format_tools()
        
        # Build prompt text with tools
        prompt_text = context.get('situation', '')
        if tools_formatted:
            prompt_text += f"\n\nAvailable tools:\n{tools_formatted}"
        
        # Use existing physical planner
        plan = generate_plan_with_context(
            character=character,
            current_activity=context.get('current_activity'),
            goal_text=goal,
            prompt_text=prompt_text,
            percepts_at_plan=context.get('percepts'),
            server_name=context.get('server_name', 'vllm'),
            model_name=context.get('model_name', 'llama3.3-70B-instruct')
        )
        
        return plan
    
    def _format_tools(self) -> str:
        """Format tools for template inclusion."""
        if not self.available_tools:
            return "No tools available."
        
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
        
        return '\n'.join(lines)
    
    def _parse_plan_response(self, response_text: str) -> Dict:
        """Parse LLM response into plan dict."""
        if not response_text or not response_text.strip():
            return {'error': 'Empty response from LLM'}
        
        # Remove markdown code fences if present
        text = response_text.strip()
        if text.startswith('```'):
            lines = text.split('\n')
            text = '\n'.join(lines[1:-1]) if len(lines) > 2 else text
        
        try:
            plan = json.loads(text)
            if not isinstance(plan, dict) or 'plan' not in plan:
                return {'error': 'Invalid plan format - missing plan field'}
            return plan
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse plan JSON: {e}")
            return {'error': f'JSON parse error: {e}'}

