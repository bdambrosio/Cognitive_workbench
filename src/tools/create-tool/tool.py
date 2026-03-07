"""
Meta-tool: generate a new Cognitive Workbench tool using the LLM.

Writes tool.py + Skill.md to src/tools-staging/{name}/.
After review, the user promotes the tool by moving it to src/tools/{name}/
and restarting the character process.
"""

import logging
import os
import py_compile
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

# ── Paths ──────────────────────────────────────────────────────────────────
_SRC_DIR = Path(__file__).parent.parent.parent   # …/src
_TOOLS_DIR = _SRC_DIR / "tools"
_STAGING_DIR = _SRC_DIR / "tools-staging"

# ── Reference tools shown to the LLM as examples ──────────────────────────
_REFERENCE_TOOLS = ["stock-price", "run-script", "calculate"]


# ── Helpers ────────────────────────────────────────────────────────────────
def _fail(executor: InfospaceExecutor, reason: str, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str, data: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=value, resource_id=None, extra=extra, data=data or value)


def _read_tool_source(tool_name: str) -> str:
    """Read tool.py + Skill.md for a reference tool."""
    tool_dir = _TOOLS_DIR / tool_name
    parts = []
    for fname in ("Skill.md", "tool.py"):
        p = tool_dir / fname
        if p.exists():
            parts.append(f"=== {tool_name}/{fname} ===\n{p.read_text()}")
    return "\n\n".join(parts)


def _llm_call(llm_generate, system: str, user: str, max_tokens: int = 3000) -> Optional[str]:
    """Single LLM call. Returns text or None."""
    response = llm_generate(
        messages=[f"{system}\n\n{user}"],
        max_tokens=max_tokens,
        temperature=0.2,
        stops=["</end>"],
    )
    if not response.success:
        logger.error(f"create-tool LLM call failed: {response.error}")
        return None
    return response.text.strip()


def generate_tool_py(llm_generate, name: str, description: str, requirements: str) -> Optional[str]:
    """Generate tool.py source. Returns code or None on failure."""
    refs = []
    for ref_name in _REFERENCE_TOOLS:
        src = _read_tool_source(ref_name)
        if src:
            refs.append(src)
    reference_block = "\n\n".join(refs)
    system = """You are an expert Python developer writing a Cognitive Workbench tool.
Follow the EXACT pattern shown in the reference tools. Key rules:
- Import InfospaceExecutor from infospace_executor
- Define _fail() and _success() helpers using executor._create_uniform_return()
- def tool(input_value=None, runtime=None, **kwargs): is the entry point
- Always get executor from kwargs.get("executor") and return early if not present
- Get llm_generate, agent_name, resource_manager from kwargs if needed
- Use _create_note() to persist output as a Note when the result is content
- Return _fail() or _success() — never a raw dict
- Keep it concise and correct"""
    user = f"""Here are reference tools to follow as patterns:

{reference_block}

Now write tool.py for a new tool with:
- Name: {name}
- Description: {description}
- Requirements: {requirements}

Write ONLY the complete Python source code for tool.py.
Do not include any explanation, markdown fences, or commentary.
End with </end>"""
    return _llm_call(llm_generate, system, user, max_tokens=3500)


def generate_skill_md(llm_generate, name: str, description: str, requirements: str) -> Optional[str]:
    """Generate Skill.md content. Returns text or None on failure."""
    system = """You are writing a Skill.md for a Cognitive Workbench tool.
It must start with YAML frontmatter containing: name, type, description, schema_hint.
type is always 'python'. schema_hint lists the tool's parameters as key: "type description" pairs.
After the frontmatter write a brief markdown body: Input, Output, Behavior, and a Common Workflows code block."""
    user = f"""Write Skill.md for:
- Name: {name}
- Description: {description}
- Requirements: {requirements}

Write ONLY the Skill.md content. No extra commentary.
End with </end>"""
    return _llm_call(llm_generate, system, user, max_tokens=1000)


def generate_and_stage_tool(llm_generate, name: str, description: str, requirements: str) -> tuple[bool, str, Optional[str]]:
    """
    Generate tool.py + Skill.md and write to staging.
    Returns (success, message, staging_dir_path).
    """
    name = name.strip().lower().replace(" ", "-")
    if not name:
        return False, "name cannot be empty", None
    if not description or not requirements:
        return False, "description and requirements required", None
    staging_dir = _STAGING_DIR / name
    tool_py_path = staging_dir / "tool.py"
    if tool_py_path.exists():
        return False, f"Tool '{name}' already exists in staging. Review or delete: {staging_dir}", None
    tool_code = generate_tool_py(llm_generate, name, description, requirements)
    if not tool_code:
        return False, "LLM failed to generate tool.py", None
    ok, err = _syntax_ok(tool_code)
    if not ok:
        return False, f"Generated tool.py has syntax errors: {err}", None
    skill_text = generate_skill_md(llm_generate, name, description, requirements)
    if not skill_text:
        return False, "LLM failed to generate Skill.md", None
    staging_dir.mkdir(parents=True, exist_ok=True)
    tool_py_path.write_text(tool_code)
    (staging_dir / "Skill.md").write_text(skill_text)
    return True, f"Tool '{name}' staged at {staging_dir}", str(staging_dir)


def _syntax_ok(code: str) -> tuple[bool, str]:
    """Check Python syntax. Returns (ok, error_message)."""
    with tempfile.NamedTemporaryFile(suffix=".py", mode="w", delete=False) as f:
        f.write(code)
        tmp = f.name
    try:
        py_compile.compile(tmp, doraise=True)
        return True, ""
    except py_compile.PyCompileError as e:
        return False, str(e)
    finally:
        os.unlink(tmp)


# ── Main tool function ─────────────────────────────────────────────────────
def _generate_instruction_skill_md(name: str, description: str, requirements: str) -> str:
    """Build a Skill.md for an instruction-type tool (no LLM needed)."""
    return (
        f"---\n"
        f"name: {name}\n"
        f"type: instruction\n"
        f'description: "{description}"\n'
        f"schema_hint:\n"
        f'  target: "$variable (optional context)"\n'
        f'  out: "$variable (optional)"\n'
        f"---\n\n"
        f"# {name}\n\n"
        f"{requirements}\n"
    )


def tool(input_value=None, runtime=None, **kwargs):
    """
    Generate a new tool (tool.py + Skill.md) using the LLM.

    Args:
        name: Tool directory/action name, e.g. "post-bluesky" (required)
        description: One-line description (required)
        requirements: What the tool should do, what APIs/libs it uses,
                      what env vars it needs, what parameters it accepts,
                      what it returns (required)
        tool_type: "python" (default) or "instruction" (no code, Skill.md body is the tool output)
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    # Per-goal guard: only one create-tool per planner session
    plan_id = getattr(getattr(executor, 'executive_node', None), 'current_plan_id', None)
    if plan_id:
        used = getattr(executor, '_side_effect_used_goals', None)
        if used is None:
            used = {}
            executor._side_effect_used_goals = used
        goal_key = f"create-tool:{plan_id}"
        if goal_key in used:
            logger.warning(f"create-tool: blocked — already created a tool in plan {plan_id}")
            return used[goal_key]

    name = (kwargs.get("name") or input_value or "").strip().lower().replace(" ", "-")
    if not name:
        return _fail(executor, "name parameter required (e.g. post-bluesky)")

    description = (kwargs.get("description") or "").strip()
    requirements = (kwargs.get("requirements") or "").strip()
    if not description or not requirements:
        return _fail(executor, "description and requirements parameters are required")

    tool_type = (kwargs.get("tool_type") or "python").strip().lower()

    if tool_type == "instruction":
        # Instruction tools: no code generation, write Skill.md directly to tools dir
        target_dir = _TOOLS_DIR / name
        if target_dir.exists():
            return _fail(executor, f"Tool '{name}' already exists at {target_dir}")
        skill_text = _generate_instruction_skill_md(name, description, requirements)
        target_dir.mkdir(parents=True, exist_ok=True)
        (target_dir / "Skill.md").write_text(skill_text)
        # Hot-register into running system
        executive_node = getattr(executor, 'executive_node', None)
        if executive_node and hasattr(executive_node, '_register_tool'):
            executive_node._register_tool(str(target_dir))
        summary = f"Instruction tool '{name}' created and registered at {target_dir}"
        logger.info(f"create-tool: instruction tool '{name}' written to {target_dir}")
        result = _success(executor, summary, data=summary,
                          extra={"name": name, "tool_type": "instruction", "path": str(target_dir)})
    else:
        # Python tools: generate code via LLM, stage for review
        llm_generate = kwargs.get("llm_generate")
        if not llm_generate:
            return _fail(executor, "llm_generate callback is required")
        success, msg, staging_dir_str = generate_and_stage_tool(llm_generate, name, description, requirements)
        if not success:
            return _fail(executor, msg)
        staging_dir = Path(staging_dir_str)
        promote_cmd = f"mv {staging_dir} {_TOOLS_DIR / name}"
        summary = (
            f"Tool '{name}' generated and saved to staging.\n\n"
            f"Files:\n  {staging_dir / 'tool.py'}\n  {staging_dir / 'Skill.md'}\n\n"
            f"To activate: review the files, then run:\n  {promote_cmd}\n"
            f"Then restart the character process."
        )
        logger.info(f"create-tool: '{name}' written to {staging_dir}")
        result = _success(executor, f"Tool '{name}' staged at {staging_dir}", data=summary,
                          extra={"name": name, "staging_dir": str(staging_dir), "promote_cmd": promote_cmd})

    # Record for per-goal guard
    if plan_id and hasattr(executor, '_side_effect_used_goals'):
        executor._side_effect_used_goals[f"create-tool:{plan_id}"] = result
    return result
