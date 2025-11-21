from __future__ import annotations
"""
Semantic Plan Validator for Infospace Plans

Validates plans semantically using LLM to check parameter flow, tool preconditions,
and workflow correctness. Outputs actionable repair suggestions.
"""

# Type checking imports
from typing import TYPE_CHECKING, List
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode
import json
import logging
from pathlib import Path
import traceback
from typing import Dict, List, Any, Optional

from utils.llm_api import LLM
from Messages import SystemMessage, UserMessage
from utils.tool_loader import load_tools, parse_yaml_frontmatter
from templates import INFOSPACE_PRIMITIVES_REFERENCE
logger = logging.getLogger(__name__)


class InfospaceSemanticValidator:
    """
    Validates infospace plans semantically by checking:
    - Parameter flow between actions
    - Tool preconditions (e.g., fetch-text accepts URLs directly)
    - Map operation correctness
    - Variable type consistency
    """
    
    def __init__(self, character: ZenohExecutiveNode, tools_dir: Optional[str] = None, llm_server: str = "vllm", llm_model: str = "Qwen/Qwen3--Next-80B-A3B-FP8", prefix_prompt: str = ""):
        """
        Initialize validator.
        
        Args:
            tools_dir: Optional path to an additional tools directory (e.g., data/tools/custom).
                      Always loads the default infospace tools first, then custom tools if provided.
            llm_server: LLM server name
            llm_model: LLM model name
        """
        self.llm = LLM(server_name=llm_server, model_name=llm_model)
        self.character = character
        
        # Load tools from src/tools directory
        tools_dir_path = Path(__file__).parent / 'tools'
        self.available_tools = {}
        self.prefix_prompt = prefix_prompt
        
        if tools_dir_path.exists():
            self.available_tools = load_tools(str(tools_dir_path))
            logger.info(f"Loaded {len(self.available_tools)} tools from {tools_dir_path}")
        else:
            logger.warning(f"Tools directory not found: {tools_dir_path}")
        
        logger.info(f"Total tools loaded: {len(self.available_tools)}")
    
    def validate(self, plan: Dict[str, Any]) -> str:
        """
        Validate plan semantically and return repair suggestions.
        
        Args:
            plan: Plan dict with 'plan' key containing list of actions
            
        Returns:
            Clear text repair suggestions that can be passed to 'edit:' command.
            Empty string if plan is valid.
        """
        logger.info(f"Validating plan for {self.character.character_name}")
        if 'plan' not in plan:
            return "ACTION 0: \nREPAIR: Insert missing 'plan' field key at start of plan"
        
        actions = plan['plan']
        if not isinstance(actions, list):
            return "ACTION 0: \nREPAIR: 'plan' field value must be a list of actions"
        
        
        # Build plan context
        plan_json = json.dumps(plan, indent=2)
        
        # LLM validation prompt
        user_prompt = """\n\n#TASK: You are a semantic plan validator for infospace operations.
Your task is to analyze a plan and identify semantic errors such as:
- Missing workflow steps (e.g., fetch-text accepts URLs directly, no download step needed)
- Parameter mismatches (e.g., map passes Note as 'value' but tool expects specific parameter)
- Missing intermediate steps (e.g., trying to fetch text from URLs without proper URL extraction)
- Incorrect variable usage (e.g., using Collection where Note expected, or vice versa)

#Plan to validate:
{plan_json}

# VALIDATION NOTES:

- 'map' operation passes each Note from the Collection as 'value' parameter to the tool
- Tools may require specific parameters (e.g., url_or_content) that must come from previous steps
- Check tool SKILL.md documentation for required parameters and usage patterns
- fetch-text accepts URLs directly - no intermediate download step needed

Analyze this plan and identify any semantic errors. 
For each error found, provide a clear, actionable repair instruction in the format:

ACTION <index>: <action to repair>
REPAIR: <repair instruction>

Output VALID -or- the ACTION and REPAIR statements only, no additional introductory, reasoning, code fences, or commentary.
For each error found, provide a clear, concise, actionable repair instruction in the format:

ACTION <index>: <action to repair>
REPAIR: <repair instruction>

If the plan is valid, output: VALID

Output ONLY 'VALID' OR the ACTION and REPAIR statements, no additional introductory, reasoning, code fences, or commentary.

"""

        user_prompt = user_prompt.replace('{plan_json}', plan_json)
        
        try:
            response = self.llm.ask(
                {},
                [SystemMessage(content=self.prefix_prompt), UserMessage(content=user_prompt)],
                max_tokens=200,
                stops=['</end>'],
                log=True
            )
            
            result_text = response if isinstance(response, str) else getattr(response, 'text', str(response))
            result_text = result_text.strip()
            
            # Check if valid
            if 'VALID' in result_text.upper():
                return ""
            
            # Format repair suggestions
            return self._format_repair_suggestions(result_text, plan)
            
        except Exception as e:
            logger.error(f"Semantic validation error: {e}")
            traceback.print_exc()
            return f"ERROR: Validation failed: {e}"
    
    def _build_tool_context(self) -> str:
        """Build context string with tool documentation."""
        if not self.available_tools:
            return "# No tools available"
        
        lines = []
        for tool_name in sorted(self.available_tools.keys()):
            tool = self.available_tools[tool_name]
            name = tool.get('name', tool_name)
            description = tool.get('description', '')
            tool_md = tool.get('tool_md_content', '')
            
            # Extract key information from SKILL.md
            lines.append(f"## {name}")
            lines.append(f"Description: {description}")
            
            # Extract parameters section
            if tool_md:
                # Parse metadata from frontmatter
                metadata = parse_yaml_frontmatter(tool_md)
                params = metadata.get('parameters', []) if metadata else []
                if params and params != "none":
                    lines.append("Required parameters:")
                    # Handle different parameter formats
                    if isinstance(params, str):
                        # Plain string description (e.g., "predicate (required) - ...")
                        lines.append(f"  - {params}")
                    elif isinstance(params, list):
                        # List of dicts or strings
                        for param in params:
                            if isinstance(param, dict):
                                param_name = param.get('name', '')
                                param_desc = param.get('description', '')
                                lines.append(f"  - {param_name}: {param_desc}")
                            elif isinstance(param, str):
                                lines.append(f"  - {param}")
                
                # Extract usage section if present
                usage_section = self._extract_section(tool_md, "## Usage")
                if usage_section:
                    lines.append("Usage:")
                    lines.append(usage_section[:500])  # Limit length
                
                # Extract input section for requirements
                input_section = self._extract_section(tool_md, "## Input")
                if input_section:
                    lines.append("Input requirements:")
                    lines.append(input_section[:300])  # Limit length
            
            lines.append("")
        
        return "\n".join(lines)
    
    def _extract_section(self, content: str, header: str) -> Optional[str]:
        """Extract markdown section by header."""
        lines = content.split('\n')
        section_lines = []
        in_section = False
        
        for line in lines:
            if line.strip() == header or line.strip().startswith(header + ' '):
                in_section = True
                continue
            
            if in_section:
                if line.startswith('#') and not line.startswith('###'):
                    break
                section_lines.append(line)
        
        return '\n'.join(section_lines).strip() if section_lines else None
    
    def _format_repair_suggestions(self, validation_text: str, plan: Dict) -> str:
        """Format validation results into actionable repair suggestions."""
        
        lines = validation_text.split('\n')
        formatted_lines = []
        for line in lines:
            line = line.strip()
            if line.startswith('ACTION') or line.startswith('REPAIR'):
                formatted_lines.append(line)

        return "\n".join(formatted_lines)


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 infospace_semantic_validator.py <plan.json> [tool-dir]")
        print("  plan.json: Path to plan JSON file")
        print("  tool-dir: Optional custom tools directory (e.g., data/tools/custom)")
        print("           Always loads the built-in infospace tools first")
        sys.exit(1)
    
    plan_path = Path(sys.argv[1])
    tools_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not plan_path.exists():
        print(f"Error: Plan file not found: {plan_path}")
        sys.exit(1)
    
    # Load plan
    with open(plan_path, 'r') as f:
        plan = json.load(f)
    
    # Validate
    validator = InfospaceSemanticValidator(character=None, tools_dir=tools_dir)
    result = validator.validate(plan)
    
    if result:
        print(result)
        sys.exit(1)
    else:
        print("✅ Plan is semantically valid")
        sys.exit(0)
