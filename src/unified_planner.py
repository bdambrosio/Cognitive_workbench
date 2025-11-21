from __future__ import annotations
"""
Unified Planner - Single planner for both infospace and physical worlds.

Handles plan generation with pluggable primitive sets and tool libraries.
"""

# Type checking imports
from typing import TYPE_CHECKING, List
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode
import json
import logging
import traceback
from typing import Dict, Any, Optional
from pathlib import Path
from infospace_planner import INFOSPACE_PLAN_TEMPLATE
from templates import INFOSPACE_PRIMITIVES_REFERENCE, PLAN_TEMPLATE
# IncrementalPlanner imported conditionally to avoid SGLang initialization for non-Jill characters
logger = logging.getLogger(__name__)


class UnifiedPlanner:
    """
    Unified planner that handles both infospace and physical world planning.
    
    Swaps primitive sets, tool libraries, and templates based on world type.
    """
    
    def __init__(self, 
                 llm_client,
                 character: ZenohExecutiveNode,
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
        self.character = character
        self.world_type = world_type
        self.map_name = map_name
        self.logger = logger_instance or logger
        
        # Load tools for this map
        self.available_tools = self._load_tools()
        
        # Load appropriate template
        self.template = self._load_template()
        
        # Create infospace planner instance if needed (for validation and reuse)
        self.infospace_planner = None
        if world_type == 'infospace':
            from infospace_planner import InfospacePlanner
            self.infospace_planner = InfospacePlanner(
                llm_client=llm_client,
                available_tools=self.available_tools,
                logger=self.logger
            )
        
        self.logger.info(f"UnifiedPlanner initialized: {world_type} world, map={map_name}, {len(self.available_tools)} tools")
    
    def _load_tools(self) -> Dict[str, Dict]:
        """Load tools from src/tools directory."""
        from utils.tool_loader import load_tools
        
        tools_dir = Path(__file__).parent / 'tools'
        tools = {}
        
        if tools_dir.exists():
            self.logger.info(f"Loading tools from: {tools_dir}")
            tools.update(load_tools(str(tools_dir)))
        else:
            self.logger.warning(f"Tools directory not found: {tools_dir}")
        
        self.logger.info(f"Total tools loaded: {len(tools)} (base + map-specific)")
        return tools
    
    def _load_template(self) -> str:
        """Load appropriate planning template."""
        if self.world_type == 'infospace':
            formatted_tools = self._format_tools()
            template = INFOSPACE_PLAN_TEMPLATE.replace('{{tools}}', formatted_tools)
            template = template.replace('{primitives_reference}', INFOSPACE_PRIMITIVES_REFERENCE)
            return template
        else:
            return PLAN_TEMPLATE
    
    def generate_plan(self, goal: str, context: Dict) -> Dict:
        self.logger.info(f"Generating {self.world_type} plan for goal: {goal}")
        
        # Route to appropriate planner
        if self.world_type == 'infospace':
            plan = self._generate_infospace_plan(goal, context)
        else:
            logger.error("Physical world planning no longer supported")
            plan=None
        
        return plan
    
    def _generate_infospace_plan(self, goal: str, context: Dict) -> Dict:
        """Generate infospace plan."""
        from templates import INFOSPACE_PRIMITIVES_REFERENCE
        
        # Try incremental planner only for Jill (SGLang backend is singleton)
        executor = context.get('executor')
        character_name = self.character.character_name if hasattr(self.character, 'character_name') else None
        
        if executor:
            try:
                # Import only for Jill to avoid SGLang initialization for other characters
                from incremental_planner import IncrementalPlanner, HAS_SGLANG
                
                if HAS_SGLANG:
                    self.logger.info("Using incremental SGLang planner for Jill")
                    
                    # Get SGLang model path from config
                    sgl_model_path = context.get('sgl_model_path')
                    if not sgl_model_path and hasattr(self.character, 'character_config'):
                        llm_config = self.character.character_config.get('llm_config', {})
                        sgl_model_path = llm_config.get('sgl_model_path')
                    
                    incremental_planner = IncrementalPlanner(
                        executor=executor,
                        available_tools=self.available_tools,
                        primitives_reference=INFOSPACE_PRIMITIVES_REFERENCE,
                        logger_instance=self.logger,
                        sgl_model_path=sgl_model_path
                    )
                    plan_result = incremental_planner.generate_plan(goal=goal, context=context, max_steps=24)
                    if not plan_result.get('error'):
                        return plan_result
                    else:
                        self.logger.warning(f"Incremental planner failed: {plan_result.get('error')}, falling back")
                        return {'error': plan_result.get('error')}
            except Exception as e:
                self.logger.warning(f"Incremental planner error: {e}, falling back to standard planner")
                traceback.print_exc()
        else:
            self.logger.error("No executor found for infospace planning")
            return {'error': 'No executor found for infospace planning'}
    
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
    
    def verify_plan(self, plan_json: Any) -> Dict:
        """
        Validate plan structure.
        Delegates to infospace planner for infospace plans, returns error for physical.
        
        Args:
            plan_json: Plan as JSON string or dict
            
        Returns:
            Dict with 'valid' (bool) and 'reason' (str if invalid)
        """
        if self.world_type == 'infospace':
            if self.infospace_planner:
                plan = plan_json if isinstance(plan_json, dict) else json.loads(plan_json)
                error_string = self.infospace_planner.verify_plan(plan)
                # Convert string format to dict for backward compatibility
                if error_string:
                    return {'valid': False, 'reason': error_string, 'error_string': error_string}
                else:
                    return {'valid': True}
            else:
                return {'valid': False, 'reason': 'Infospace planner not initialized'}
        else:
            # Physical world validation handled by plan_module in executive_node
            return {'valid': True}


