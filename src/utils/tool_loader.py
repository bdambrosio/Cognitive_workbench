"""
Tool loading utility for infospace operations.
Scans tool directories and returns metadata for executor and planner.
"""
import logging
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
        
        # Collect additional files in the tool directory
        additional_files = [
            f.name for f in tool_dir.iterdir() 
            if f.is_file() and f.name != "SKILL.md"
        ]
        
        # Store tool metadata
        tools[tool_name] = {
            'name': tool_name,
            'description': tool_description,
            'type': tool_type,
            'tool_md_content': content,
            'path': str(tool_dir.absolute()),
            'additional_files': additional_files
        }
        
        logger.info(f"Loaded tool: {tool_name} (type: {tool_type})")
    
    logger.info(f"Successfully loaded {len(tools)} tools from {tools_dir_path}")
    
    if len(tools) == 0:
        logger.warning("No tools found in directory")
    
    return tools

