"""
Semantic Plan Validator for Infospace Plans

Validates plans semantically using LLM to check parameter flow, tool preconditions,
and workflow correctness. Outputs actionable repair suggestions.
"""

import json
import logging
from pathlib import Path
from typing import Dict, List, Any, Optional

from utils.llm_api import LLM
from Messages import SystemMessage, UserMessage
from utils.tool_loader import load_tools, parse_yaml_frontmatter

logger = logging.getLogger(__name__)


class InfospaceSemanticValidator:
    """
    Validates infospace plans semantically by checking:
    - Parameter flow between actions
    - Tool preconditions (e.g., extract-paper-text needs pdf_content from download-pdf)
    - Map operation correctness
    - Variable type consistency
    """
    
    def __init__(self, tools_dir: Optional[str] = None, llm_server: str = "vllm", llm_model: str = "Qwen/Qwen3--Next-80B-A3B-FP8"):
        """
        Initialize validator.
        
        Args:
            tools_dir: Optional path to map-specific tools directory (e.g., maps/infolab/tools).
                      Always loads maps/tools first, then map-specific tools if provided.
            llm_server: LLM server name
            llm_model: LLM model name
        """
        self.llm = LLM(server_name=llm_server, model_name=llm_model)
        
        # Load tools: ALWAYS load base tools first, then map-specific if provided
        # Matches unified_planner._load_tools() logic
        maps_base = Path(__file__).parent / 'maps'
        self.available_tools = {}
        
        # Load base tools first
        base_tools_dir = maps_base / 'tools'
        if base_tools_dir.exists():
            base_tools = load_tools(str(base_tools_dir))
            self.available_tools.update(base_tools)
            logger.info(f"Loaded {len(base_tools)} base tools from {base_tools_dir}")
        else:
            logger.warning(f"Base tools directory not found: {base_tools_dir}")
        
        # Load map-specific tools if provided (overrides base on name collision)
        if tools_dir:
            map_tools_dir = Path(tools_dir)
            if map_tools_dir.exists():
                map_tools = load_tools(str(map_tools_dir))
                if map_tools:
                    # Log any overrides
                    for name in map_tools:
                        if name in self.available_tools:
                            logger.info(f"Map tool '{name}' overrides base tool")
                    self.available_tools.update(map_tools)
                    logger.info(f"Loaded {len(map_tools)} map-specific tools from {map_tools_dir}")
            else:
                logger.warning(f"Map-specific tools directory not found: {map_tools_dir}")
        
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
        if 'plan' not in plan:
            return "ERROR: Plan missing 'plan' field"
        
        actions = plan['plan']
        if not isinstance(actions, list):
            return "ERROR: Plan must be array"
        
        # Build tool documentation context
        tool_context = self._build_tool_context()
        
        # Build plan context
        plan_json = json.dumps(plan, indent=2)
        
        # LLM validation prompt
        system_prompt = """You are a semantic plan validator for infospace operations.

Your task is to analyze a plan and identify semantic errors such as:
- Missing workflow steps (e.g., extract-paper-text requires pdf_content from download-pdf)
- Parameter mismatches (e.g., map passes Note as 'value' but tool expects 'pdf_content')
- Missing intermediate steps (e.g., trying to extract text from URLs without downloading PDFs first)
- Incorrect variable usage (e.g., using Collection where Note expected, or vice versa)

For each error found, provide a clear, actionable repair suggestion in the format:
ACTION <index>: <error description>
REPAIR: <specific fix, e.g., "Insert download-pdf step before extract-paper-text">

If the plan is valid, output: VALID

Output ONLY the validation results, no additional commentary."""
        
        # Import primitives reference from planner
        from infospace_planner import INFOSPACE_PRIMITIVES_REFERENCE
        
        user_prompt = f"""{INFOSPACE_PRIMITIVES_REFERENCE}

# AVAILABLE TOOLS AND THEIR REQUIREMENTS:

{tool_context}

# PLAN TO VALIDATE:

{plan_json}

# VALIDATION NOTES:

- 'map' operation passes each Note from the Collection as 'value' parameter to the tool
- Tools may require specific parameters (e.g., pdf_content, url) that must come from previous steps
- Check tool SKILL.md documentation for required parameters and usage patterns
- Check for missing intermediate steps (e.g., download-pdf before extract-paper-text)

Analyze this plan and identify any semantic errors."""
        
        try:
            response = self.llm.ask(
                {},
                [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
                max_tokens=2000,
                stops=['</end>'],
                log=True
            )
            
            result_text = response if isinstance(response, str) else getattr(response, 'text', str(response))
            result_text = result_text.strip()
            
            # Check if valid
            if result_text.upper().startswith('VALID'):
                return ""
            
            # Format repair suggestions
            return self._format_repair_suggestions(result_text, plan)
            
        except Exception as e:
            logger.error(f"Semantic validation error: {e}")
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
        lines = []
        lines.append("SEMANTIC VALIDATION ERRORS FOUND:")
        lines.append("")
        lines.append(validation_text)
        lines.append("")
        lines.append("# To repair, use 'edit:' command with the suggestions above")
        
        return "\n".join(lines)


def validate_plan_semantically(plan: Dict[str, Any], tools_dir: Optional[str] = None) -> str:
    """
    Public interface for semantic plan validation.
    
    Args:
        plan: Plan dict with 'plan' key
        tools_dir: Optional map-specific tools directory path
        
    Returns:
        Repair suggestions text (empty if valid)
    """
    validator = InfospaceSemanticValidator(tools_dir=tools_dir)
    return validator.validate(plan)


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 infospace_semantic_validator.py <plan.json> [tool-dir]")
        print("  plan.json: Path to plan JSON file")
        print("  tool-dir: Optional map-specific tools directory (e.g., maps/infolab/tools)")
        print("           Always loads maps/tools base tools first")
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
    validator = InfospaceSemanticValidator(tools_dir=tools_dir)
    result = validator.validate(plan)
    
    if result:
        print(result)
        sys.exit(1)
    else:
        print("✅ Plan is semantically valid")
        sys.exit(0)

