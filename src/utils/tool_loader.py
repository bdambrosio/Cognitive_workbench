"""
Tool loading utility for infospace operations.
Scans tool directories and returns metadata for executor and planner.
"""
import logging
import re
from pathlib import Path
from typing import Dict, Optional
import yaml

logger = logging.getLogger(__name__)


def parse_yaml_frontmatter(content: str) -> Optional[Dict]:
    """
    Parse YAML frontmatter from markdown file.
    
    Args:
        content: Full file content with frontmatter
        
    Returns:
        Dict of metadata, or None if parsing fails
    """
    if not content.startswith('---'):
        return None
    
    parts = content.split('---', 2)
    if len(parts) < 3:
        return None
    
    yaml_content = parts[1].strip()
    metadata = yaml.safe_load(yaml_content)
    return metadata


def extract_markdown_section(content: str, section_header: str) -> Optional[str]:
    """
    Extract a markdown section by header.
    
    Args:
        content: Full markdown content
        section_header: Header to find (e.g., "## Common Workflows")
        
    Returns:
        Section content (excluding header), or None if not found
    """
    lines = content.split('\n')
    section_lines = []
    in_section = False
    
    for line in lines:
        # Check if we hit the target section
        if line.strip() == section_header:
            in_section = True
            continue
        
        # If in section and hit another header of same/higher level, stop
        if in_section:
            if line.startswith('#') and not line.startswith('###'):
                break
            section_lines.append(line)
    
    if section_lines:
        return '\n'.join(section_lines).strip()
    return None


def load_tools(tools_dir_path: str) -> Dict[str, Dict]:
    """
    Load all tools from directory and return metadata dict.
    
    Args:
        tools_dir_path: Absolute path to tools directory
        
    Returns:
        Dict mapping tool_name -> {name, description, type, tool_md_content, path, additional_files}
    """
    tools_dir = Path(tools_dir_path)
    
    if not tools_dir.exists():
        logger.error(f"Tools directory not found: {tools_dir_path}")
        return {}
    
    if not tools_dir.is_dir():
        logger.error(f"Tools path is not a directory: {tools_dir_path}")
        return {}
    
    tools = {}
    
    # Primitive action names that tools cannot collide with
    primitive_names = {
        'move', 'say', 'think', 'take', 'place', 'inspect', 'use', 'scan',
        'apply', 'display', 'create-note', 'create-collection', 'persist', 'load', 'index', 'organize', 'search',
        'extract', 'filter', 'merge', 'coerce',
        'aggregate', 'sort', 'group_by', 'compare', 'map', 'flatten', 'add', 'split',
        'while', 'if', 'wait', 'sleep', 'focus'
    }
    
    # Scan immediate subdirectories
    for tool_dir in sorted(tools_dir.iterdir()):
        if not tool_dir.is_dir():
            continue
            
        # Check for SKILL.md (all caps) or Skill.md (capital S)
        tool_md_path = tool_dir / "SKILL.md"
        if not tool_md_path.exists():
            tool_md_path = tool_dir / "Skill.md"
        
        if not tool_md_path.exists():
            logger.warning(f"No SKILL.md or Skill.md found in {tool_dir.name}, skipping")
            continue
        
        # Read SKILL.md
        with open(tool_md_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Parse frontmatter
        metadata = parse_yaml_frontmatter(content)
        
        if metadata is None:
            logger.error(f"Failed to parse YAML frontmatter in {tool_md_path}")
            continue
        
        # Extract required fields with defaults
        tool_name = metadata.get('name')
        if not tool_name:
            tool_name = tool_dir.name
            logger.warning(f"Missing 'name' in {tool_md_path}, using directory name: {tool_name}")
        
        tool_description = metadata.get('description')
        if not tool_description:
            tool_description = "No description available"
            logger.warning(f"Missing 'description' in {tool_md_path}")
        
        tool_type = metadata.get('type', 'code_execution')
        
        # Check for duplicate names
        if tool_name in tools:
            logger.warning(f"Duplicate tool name '{tool_name}' in {tool_dir}, skipping")
            continue
        
        # Check for name collision with primitives
        if tool_name in primitive_names:
            logger.warning(f"Tool name '{tool_name}' collides with primitive action name - tool will shadow primitive")
        
        # Store tool metadata
        
        # For plan tools, load plan.json file
        plan_data = None
        if tool_type == 'plan':
            # Look for plan.json or tool.json
            plan_file = tool_dir / 'plan.json'
            if not plan_file.exists():
                plan_file = tool_dir / 'tool.json'
            
            if not plan_file.exists():
                logger.error(f"Plan tool {tool_name} missing plan.json or tool.json, skipping")
                continue
            
            # Load and parse plan JSON
            import json as json_lib
            with open(plan_file, 'r', encoding='utf-8') as f:
                plan_data = json_lib.load(f)
            
            # Validate plan structure
            if 'plan' not in plan_data:
                logger.error(f"Plan tool {tool_name} plan.json missing 'plan' field, skipping")
                continue
            
            if 'out' not in plan_data:
                logger.error(f"Plan tool {tool_name} plan.json missing 'out' field, skipping")
                continue
            
            plan_steps = plan_data.get('plan', [])
            logger.info(f"Plan tool {tool_name}: {len(plan_steps)} steps")
        
        # For Python tools, find Python file and validate trust
        python_file = None
        if tool_type == 'python':
            # Check for tool.py first, then {tool-name}.py
            candidate = tool_dir / 'tool.py'
            if candidate.exists():
                python_file = str(candidate.absolute())
            else:
                candidate = tool_dir / f"{tool_name}.py"
                if candidate.exists():
                    python_file = str(candidate.absolute())
            
            if not python_file:
                logger.warning(f"Tool {tool_name} type is 'python' but no tool.py or {tool_name}.py found")
                    
        # Collect additional files in the tool directory
        additional_files = [
            f.name for f in tool_dir.iterdir() 
            if f.is_file() and f.name not in ("SKILL.md", "Skill.md")
        ]
        
        # Extract common workflows section if present
        workflows = extract_markdown_section(content, "## Common Workflows")
        
        # Extract examples from frontmatter for planner
        examples = metadata.get('examples', [])
        if not examples:
            logger.warning(f"Tool {tool_name} missing examples in SKILL.md frontmatter")
        
        # For instruction tools, extract body text (everything after frontmatter)
        body_text = None
        if tool_type == 'instruction':
            # Match first complete frontmatter block: --- ... ---
            frontmatter_match = re.search(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
            if frontmatter_match:
                # Extract content after the closing ---
                body_text = content[frontmatter_match.end():].strip()
            else:
                # No frontmatter found, use full content
                body_text = content.strip()
            if not body_text:
                logger.warning(f"Instruction tool {tool_name} has no body text after frontmatter")
        
        # Extract schema_hint from frontmatter if present
        schema_hint = metadata.get('schema_hint', {})
        
        tools[tool_name] = {
            'name': tool_name,
            'description': tool_description,
            'type': tool_type,
            'tool_md_content': content,
            'body_text': body_text,  # Body text for instruction tools
            'workflows': workflows,
            'examples': examples,
            'schema_hint': schema_hint,
            'path': str(tool_dir.absolute()),
            'python_file': python_file,
            'additional_files': additional_files,
            'plan_data': plan_data
        }
        
        logger.info(f"Loaded tool: {tool_name} (type: {tool_type})")
    
    logger.info(f"Successfully loaded {len(tools)} tools from {tools_dir_path}")
    
    if len(tools) == 0:
        logger.warning("No tools found in directory")
    
    return tools

