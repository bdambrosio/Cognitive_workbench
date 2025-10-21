"""
skill_executor.py - Skill execution engine for cognitive workbench

Handles execution of skills from the infospace, supporting:
- Prompt-augmentation skills (LLM with enhanced context)
- Code-execution skills (Python/shell scripts)
- Hybrid skills (code + LLM synthesis)
"""

import json
import logging
import os
import subprocess
import traceback
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional

logger = logging.getLogger('skill_executor')


class SkillExecutor:
    """
    Executes skills on behalf of agents.
    
    Skills are cognitive tools that extend agent capabilities beyond
    basic physical actions. They can augment LLM prompts with structured
    thinking frameworks or execute computational operations.
    """
    
    def __init__(self, character_node):
        """
        Initialize skill executor.
        
        Args:
            character_node: Reference to ZenohExecutiveNode that owns this executor
        """
        self.character = character_node
        self.character_name = character_node.character_name
        self.llm_client = character_node.llm_client
        self.session = character_node.session
        self.debug = getattr(character_node, 'debug', False)
        
        # Execution statistics
        self.skill_use_count = {}
        self.skill_errors = {}
        
    def execute_skill(self, skill_name: str, value: Optional[str], 
                     reason: str, skill_config: dict) -> dict:
        """
        Execute a skill with given input value.
        
        Args:
            skill_name: Name of the skill to execute
            value: Input value/argument to the skill (may be None for some skills)
            reason: Why the agent is using this skill
            skill_config: Skill configuration from resource rules
            
        Returns:
            dict with:
                - success: bool
                - result: str or dict (skill output)
                - error: str (if failed)
                - execution_time: float (seconds)
        """
        start_time = datetime.now()
        
        try:
            # Track usage
            self.skill_use_count[skill_name] = self.skill_use_count.get(skill_name, 0) + 1
            
            # Extract skill metadata
            skill_type = skill_config.get('skill_type', 'prompt_augmentation')
            skill_path = skill_config.get('skill_path')
            skill_content = skill_config.get('skill_md_content', '')
            
            logger.info(f"Executing {skill_type} skill: {skill_name}")
            logger.debug(f"  Value: {value[:100] if value else 'None'}...")
            logger.debug(f"  Reason: {reason}")
            
            # Route to appropriate execution method
            if skill_type == 'prompt_augmentation':
                result = self._execute_prompt_skill(
                    skill_name, value, reason, skill_content, skill_config
                )
            elif skill_type == 'code_execution':
                result = self._execute_code_skill(
                    skill_name, value, reason, skill_path, skill_config
                )
            elif skill_type == 'hybrid':
                result = self._execute_hybrid_skill(
                    skill_name, value, reason, skill_path, skill_content, skill_config
                )
            else:
                raise ValueError(f"Unknown skill_type: {skill_type}")
            
            # Add execution metadata
            execution_time = (datetime.now() - start_time).total_seconds()
            result['execution_time'] = execution_time
            result['skill_name'] = skill_name
            result['skill_type'] = skill_type
            
            logger.info(f"✓ Skill {skill_name} completed in {execution_time:.2f}s")
            return result
            
        except Exception as e:
            # Track errors
            self.skill_errors[skill_name] = self.skill_errors.get(skill_name, 0) + 1
            
            execution_time = (datetime.now() - start_time).total_seconds()
            error_msg = f"Skill execution failed: {str(e)}"
            logger.error(f"✗ {error_msg}")
            logger.debug(traceback.format_exc())
            
            return {
                'success': False,
                'result': None,
                'error': error_msg,
                'execution_time': execution_time,
                'skill_name': skill_name
            }
    
    def _execute_prompt_skill(self, skill_name: str, value: Optional[str], 
                             reason: str, skill_content: str, 
                             skill_config: dict) -> dict:
        """
        Execute a prompt-augmentation skill.
        
        These skills enhance the LLM prompt with structured guidelines
        and frameworks for reasoning about the input value.
        
        Args:
            skill_name: Name of skill
            value: Input to process
            reason: Why using this skill
            skill_content: Full SKILL.md content
            skill_config: Configuration dict
            
        Returns:
            dict with success, result, error
        """
        try:
            # Extract configuration
            context_injection = skill_config.get('context_injection', 'full_content')
            result_type = skill_config.get('result_type', 'text')
            
            # Prepare skill context based on injection mode
            if context_injection == 'full_content':
                skill_context = skill_content
            elif context_injection == 'instructions_only':
                # Extract just the instructions, skip frontmatter
                lines = skill_content.split('\n')
                in_frontmatter = False
                instruction_lines = []
                for line in lines:
                    if line.strip() == '---':
                        if not in_frontmatter:
                            in_frontmatter = True
                        else:
                            in_frontmatter = False
                        continue
                    if not in_frontmatter:
                        instruction_lines.append(line)
                skill_context = '\n'.join(instruction_lines)
            elif context_injection == 'summary':
                # Use just the description
                skill_context = skill_config.get('description', '')
            else:
                skill_context = skill_content
            
            # Build enhanced prompt
            system_prompt = self.character._update_system_prompt()
            user_prompt = self.character.observations['dynamic']
            
            skill_directive = f"""You are using the skill: {skill_name}

# Skill Guidelines:
{skill_context}

# Your Task:
Apply this skill to the following input:

Input: {value if value else '(no input provided)'}

Reason for using this skill: {reason}

Expected output format: {result_type}

Provide your response following the skill's guidelines. Be thorough but concise.
End your response with:
</end>
"""
            
            # Call LLM
            if not self.llm_client:
                raise RuntimeError("LLM client not available")
            
            response = self.llm_client.generate(
                messages=[system_prompt, user_prompt, skill_directive],
                max_tokens=1000,  # Generous for skill outputs
                temperature=0.7,
                timeout=None if self.debug else 30.0,
                stops=['</end>']
            )
            
            if not response.success:
                raise RuntimeError(f"LLM call failed: {response.error}")
            
            result_text = response.text.strip()
            
            # Parse result if structured format expected
            if result_type == 'json':
                try:
                    result_data = json.loads(result_text)
                except json.JSONDecodeError:
                    # LLM didn't return valid JSON, wrap it
                    result_data = {'raw_response': result_text}
            else:
                result_data = result_text
            
            logger.info(f"Prompt skill {skill_name} result: {result_text[:200]}...")
            
            return {
                'success': True,
                'result': result_data,
                'error': None,
                'raw_response': result_text
            }
            
        except Exception as e:
            return {
                'success': False,
                'result': None,
                'error': f"Prompt skill execution failed: {str(e)}"
            }
    
    def _execute_code_skill(self, skill_name: str, value: Optional[str],
                           reason: str, skill_path: str,
                           skill_config: dict) -> dict:
        """
        Execute a code-execution skill.
        
        Runs Python or shell scripts from the skill directory.
        
        Args:
            skill_name: Name of skill
            value: Input to process
            reason: Why using this skill
            skill_path: Path to skill directory
            skill_config: Configuration dict
            
        Returns:
            dict with success, result, error
        """
        try:
            # Extract configuration
            entry_point = skill_config.get('entry_point')
            if not entry_point:
                raise ValueError(f"Code skill {skill_name} missing entry_point")
            
            input_format = skill_config.get('input_format', 'stdin')
            output_format = skill_config.get('output_format', 'stdout')
            timeout = 30.0 if not self.debug else 300.0
            
            # Resolve entry point path
            skill_dir = Path(skill_path)
            entry_script = skill_dir / entry_point
            
            if not entry_script.exists():
                raise FileNotFoundError(f"Entry point not found: {entry_script}")
            
            # Prepare command
            if entry_script.suffix == '.py':
                cmd = ['python', str(entry_script)]
            elif entry_script.suffix == '.sh':
                cmd = ['bash', str(entry_script)]
            else:
                cmd = [str(entry_script)]
            
            # Prepare input based on format
            stdin_input = None
            if input_format == 'stdin':
                stdin_input = value if value else ''
            elif input_format == 'args':
                if value:
                    cmd.append(value)
            elif input_format == 'file':
                # Write value to temp file, pass path as arg
                temp_file = skill_dir / '.input_temp.txt'
                temp_file.write_text(value if value else '')
                cmd.append(str(temp_file))
            
            # Execute
            logger.debug(f"Running: {' '.join(cmd)}")
            result = subprocess.run(
                cmd,
                cwd=str(skill_dir),
                capture_output=True,
                text=True,
                input=stdin_input,
                timeout=timeout
            )
            
            # Clean up temp file if used
            if input_format == 'file':
                temp_file.unlink(missing_ok=True)
            
            # Check result
            if result.returncode != 0:
                error_msg = result.stderr or f"Exit code {result.returncode}"
                logger.error(f"Code skill failed: {error_msg}")
                return {
                    'success': False,
                    'result': None,
                    'error': error_msg,
                    'stdout': result.stdout,
                    'stderr': result.stderr
                }
            
            # Parse output based on format
            output = result.stdout.strip()
            
            if output_format == 'json':
                try:
                    result_data = json.loads(output)
                except json.JSONDecodeError:
                    result_data = {'raw_output': output}
            else:
                result_data = output
            
            logger.info(f"Code skill {skill_name} completed successfully")
            
            return {
                'success': True,
                'result': result_data,
                'error': None,
                'stdout': result.stdout,
                'stderr': result.stderr
            }
            
        except subprocess.TimeoutExpired:
            return {
                'success': False,
                'result': None,
                'error': f"Skill execution timeout after {timeout}s"
            }
        except Exception as e:
            return {
                'success': False,
                'result': None,
                'error': f"Code skill execution failed: {str(e)}"
            }
    
    def _execute_hybrid_skill(self, skill_name: str, value: Optional[str],
                             reason: str, skill_path: str, skill_content: str,
                             skill_config: dict) -> dict:
        """
        Execute a hybrid skill (code + prompt).
        
        First runs code for preprocessing/scaffolding,
        then feeds result to LLM with skill guidelines.
        
        Args:
            skill_name: Name of skill
            value: Input to process
            reason: Why using this skill
            skill_path: Path to skill directory
            skill_content: SKILL.md content
            skill_config: Configuration dict
            
        Returns:
            dict with success, result, error
        """
        try:
            # Step 1: Execute code phase
            logger.info(f"Hybrid skill {skill_name}: Code phase")
            code_result = self._execute_code_skill(
                skill_name, value, reason, skill_path, skill_config
            )
            
            if not code_result['success']:
                return code_result  # Code phase failed
            
            code_output = code_result['result']
            
            # Step 2: Execute prompt phase with code output
            logger.info(f"Hybrid skill {skill_name}: LLM synthesis phase")
            
            # Prepare enhanced value that includes code output
            enhanced_value = f"""Original Input:
{value if value else '(none)'}

Code Processing Result:
{json.dumps(code_output, indent=2) if isinstance(code_output, dict) else code_output}
"""
            
            prompt_result = self._execute_prompt_skill(
                skill_name, enhanced_value, reason, skill_content, skill_config
            )
            
            if not prompt_result['success']:
                return prompt_result
            
            # Combine results
            return {
                'success': True,
                'result': prompt_result['result'],
                'error': None,
                'code_output': code_output,
                'synthesis': prompt_result['result']
            }
            
        except Exception as e:
            return {
                'success': False,
                'result': None,
                'error': f"Hybrid skill execution failed: {str(e)}"
            }
    
    def get_statistics(self) -> dict:
        """
        Get skill usage statistics.
        
        Returns:
            dict with usage counts and error rates
        """
        return {
            'total_uses': sum(self.skill_use_count.values()),
            'skills_used': list(self.skill_use_count.keys()),
            'use_counts': self.skill_use_count,
            'error_counts': self.skill_errors,
            'most_used': max(self.skill_use_count.items(), 
                           key=lambda x: x[1])[0] if self.skill_use_count else None
        }


def extract_skill_metadata(yaml_content: str) -> dict:
    """
    Extract metadata from YAML frontmatter in skill content.
    
    Args:
        yaml_content: Full SKILL.md content with frontmatter
        
    Returns:
        dict with metadata fields
    """
    import re
    
    # Extract YAML frontmatter
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.search(frontmatter_pattern, yaml_content, re.DOTALL | re.MULTILINE)
    
    if not match:
        return {}
    
    yaml_text = match.group(1)
    
    # Simple YAML parsing (handles basic key: value pairs)
    metadata = {}
    for line in yaml_text.split('\n'):
        line = line.strip()
        if ':' in line:
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            
            # Handle quoted strings
            if value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            elif value.startswith("'") and value.endswith("'"):
                value = value[1:-1]
            
            metadata[key] = value
    
    return metadata


def get_skill_instances_from_map(session, timeout: float = 10.0) -> list:
    """
    Query map_node for all skill instances.
    
    Args:
        session: Zenoh session
        timeout: Query timeout
        
    Returns:
        List of skill instance dicts with name, description, etc.
    """
    from zenoh import ConsolidationMode, QueryTarget
    
    try:
        for reply in session.get(
            "cognitive/map/types",
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            timeout=timeout
        ):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    # Check if Skill type exists
                    if 'Skill' in data.get('resource_types', []):
                        # Query for skill details
                        for skill_reply in session.get(
                            "cognitive/map/resource_rules/Skill",
                            target=QueryTarget.BEST_MATCHING,
                            consolidation=ConsolidationMode.NONE,
                            timeout=timeout
                        ):
                            if skill_reply.ok:
                                skill_data = json.loads(skill_reply.ok.payload.to_bytes().decode('utf-8'))
                                if skill_data.get('success'):
                                    resource_rules = skill_data.get('resource_rules', {})
                                    return resource_rules.get('skill_instances', [])
                            break
            break
    except Exception as e:
        logger.error(f"Failed to get skill instances: {e}")
    
    return []

