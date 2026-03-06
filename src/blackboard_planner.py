"""
Blackboard Planner - SGLang-based iterative planning with blackboard pattern.

Extends IncrementalPlanner with blackboard-based meta-cognitive planning.

Note: SGLang backend must be configured before use:
    import sglang as sgl
    sgl.set_default_backend(sgl.Runtime(model_path="...", ...))
"""
import json
import logging
import traceback
import time
import re
import os
from pathlib import Path
from typing import Dict, List, Any, Optional

# Import shared functionality from incremental_planner
from incremental_planner import (
    IncrementalPlanner,
    HAS_SGLANG,
    INCREMENTAL_PLAN_SPECIFICATIONS,
    build_tool_catalog,
    tool_catalog_text,
    load_skill_docs,
    repair_json_string,
    sgl_to_infospace_action,
    execute_infospace_action,
    parse_request_tools,
    stage0_resource_retrieval,
)

# Import SGLang functions if available
if HAS_SGLANG:
    from sglang import function, system, user, assistant, gen, select
else:
    # Mock function decorator to avoid ImportErrors
    def function(f): return f

# Configure logging with file handler
# Add file handler directly to this module's logger (doesn't interfere with root logger config)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Only add handlers if they don't already exist (to avoid duplicates on re-import)
if not logger.handlers:
    # File handler - ensure logs directory exists
    try:
        import os
        _log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        os.makedirs(_log_dir, exist_ok=True)
        _log_path = os.path.join(_log_dir, 'blackboard_planner.log')
        file_handler = logging.FileHandler(_log_path, mode='w')
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            '%Y-%m-%d %H:%M:%S'
        ))
        logger.addHandler(file_handler)
    except Exception:
        # Fall back to console-only if file handler setup fails
        pass


# Blackboard-specific functions (keep these)
if HAS_SGLANG:
    @function
    def tool_planner_infospace(s, goal: str, character_context: str, recent_context: str,
                              tools_catalog_text: str, executor, trace_file=None, max_steps: int = 16, initial_blackboard: Optional[str] = None):
        """
        SGLang incremental planner for infospace goals with blackboard pattern.
        
        Args:
            s: SGLang state
            goal: Goal text
            character_context: Character description + drives
            recent_context: Recent thoughts/memories + last action
            tools_catalog_text: Formatted tool catalog
            executor: InfospaceExecutor instance (with _plan_actions attribute)
            trace_file: Optional file handle for tracing
            max_steps: Maximum planning steps
            initial_blackboard: Optional initial blackboard state (JSON string)
        """
        # Stage 0: Resource retrieval (if executor available)
        available_resources_text = ""
        if executor:
            try:
                available_resources_text = stage0_resource_retrieval(goal=goal, executor=executor)
            except Exception as e:
                logger.warning(f"Stage 0: Resource retrieval failed: {e}")
        
        # Stage 1: Analysis + tool selection
        system_parts = [
            "You are a planning-and-acting assistant for information space operations.",
            "You can choose tools/primitives, call them via JSON arguments,",
            "and iteratively refine your plan until the goal is satisfied.\n"
        ]
        if initial_blackboard:
            system_parts.append(
                "\nCRITICAL CONTEXT: You are a SUB-PLANNER working on a specific delegated task.\n"
                "Do NOT decompose further unless absolutely necessary.\n"
                "Prioritize DIRECT_EXECUTION strategies to finish the task quickly.\n"
            )
        
        system_parts.append(f"\n{INCREMENTAL_PLAN_SPECIFICATIONS}\n")
        system_parts.append(
            "You will work in repeated cycles:\n"
            "Stage 1 (once): Analyze goal, select relevant tools, decompose into FIRST_TASK.\n"
            "Stage 2 (loop): Pick a single tool and JSON args for CURRENT_TASK.\n"
            "Stage 3 (loop): Reflect on result, decide if goal done, set NEXT_TASK.\n\n"
            "ALWAYS follow formatting instructions exactly."
        )
        
        s += system("".join(system_parts))
        
        # Add character context if available - constant for run, so ok to add here
        if character_context:
            system_parts.append(f"\n# CHARACTER CONTEXT\n{character_context}\n")
        
        # Add recent context if available
        if recent_context:
            system_parts.append(f"{recent_context}\n")
        
        # Add available resources from Stage 0
        if available_resources_text:
            system_parts.append(f"\n{available_resources_text}\n")
        # Add current date and time
        import datetime
        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        system_parts.append(f"\n# CURRENT CONTEXT\nCurrent date and time: {current_time}\n")

        s += user(
            f"Goal: {goal}\n\n"
            f"Tool catalog:\n{tools_catalog_text}\n\n"
            "Stage 1: Analyze goal and identify relevant tools. Be concise in your analysis.\n"
            "Include tools you might need AND related/supporting tools.\n"
            "Err on the side of including additional tools in SELECTED_TOOLS_JSON for better coverage.\n"
            "Then, decompose the goal into a FIRST high-level task/subgoal to focus on.\n"
            "In doing so, consider the tools you have selected, the goal you are trying to achieve, and the downstream tasks that will be required to achieve the goal.\n"
            "Respond with the following fields:\n"
            "ANALYSIS: <text>\n"
            "CARDINALITY_CHECK: <\"SINGLE\" or \"MULTIPLE/LIST\">. Does the question imply a unique answer (e.g., 'date of birth') or a potentially multi-value answer (e.g., 'children')? If MULTIPLE, your plan must be exhaustive.\n"
            "SELECTED_TOOLS_JSON: <json list of tool names>\n"
            "FIRST_TASK: <high-level subgoal to tackle first>\n"
        )
        
        s += assistant(
            "ANALYSIS: "
            + gen("stage1_analysis", max_tokens=256, stop="\n")
            + "\nSELECTED_TOOLS_JSON: "
            + gen("selected_tools_json", max_tokens=256, stop="\n")
            + "\nFIRST_TASK: "
            + gen("first_task", max_tokens=128, stop="\n")
            + "\n"
        )
        
        try:
            logger.info(f"Stage 1: Analysis + tool selection\n{s['stage1_analysis']}")
            logger.info(f"SELECTED_TOOLS_JSON: {s['selected_tools_json']}")
            logger.info(f"FIRST_TASK: {s['first_task']}")
        except KeyError as e:
            logger.warning(f"Stage 1 values not available: {e}")
        
        # Stage 1.5: Load and inject detailed docs for selected tools
        try:
            selected_tools_json = s['selected_tools_json']
            selected_tools = json.loads(selected_tools_json)
            if isinstance(selected_tools, list) and selected_tools:
                expanded_docs = load_skill_docs(selected_tools, executor.available_tools)
                if expanded_docs:
                    s += user(expanded_docs)
                    s += assistant("I have reviewed the detailed documentation for the selected tools.\n")
                    logger.info(f"Stage 1.5: Injected detailed docs for {len(selected_tools)} tools")
        except (KeyError, json.JSONDecodeError, TypeError) as e:
            logger.warning(f"Failed to parse selected tools for doc expansion: {e}")
        
        # Stage 2/3 format instructions
        s += user(
            "Stage 2 FORMAT:\n"
            "  TOOL_NAME: <name>\n"
            "  TOOL_ARGS_JSON: <json object>\n\n"
            "Stage 3 FORMAT:\n"
            "  THOUGHTS: <text>\n"
            "  DONE: <YES or NO - is the entire GOAL satisfied?>\n"
            "  NEXT_TASK: <next high-level subgoal, or blank if DONE=YES>\n"
            "  REQUEST_TOOLS: <json array of tool names or empty array []>\n\n"
            "  CRITICAL for DONE:\n"
            "  - Only mark DONE: YES after ALL required actions are executed\n"
            "  - COMPLENESS CHECK: If the question asks for an attribute that can change over time (jobs, spouses, locations), you must verify if multiple values exist\n"
            "  - DO NOT STOP at the first search result. If you find one answer (e.g., 'Ambassador to Czechoslovakia'), you must briefly verify no other equal-tier answers exist (e.g., 'Ambassador to Ghana') before finishing\n"
            "  - If goal requires communicating to user, use 'say' primitive BEFORE marking done\n"
            "  - When DONE=YES, NEXT_TASK must be blank (leave empty)\n\n"
            "  CRITICAL for REQUEST_TOOLS:\n"
            "  - Always output valid JSON array: [] or [\"tool1\", \"tool2\"]\n"
            "  - If no tools needed, output: []\n"
            "  - If tools needed, output complete array on single line\n"
            "  - Example: [\"project\", \"filter-structured\"]\n\n"
            "  If you realize you need a tool not initially selected, add it to REQUEST_TOOLS.\n"
            "  You'll receive its full documentation before the next step.\n\n"
            "Follow these formats exactly."
        )
        s += assistant("Understood.\n")

        # Main loop
        current_task = s["first_task"].strip()
        
        # Initialize blackboard from parent context if available, else empty
        current_blackboard = initial_blackboard if initial_blackboard else "{}"
        tool_result = ""
        
        for step in range(max_steps):
            # Initialize variables for this step
            action = {} 
            resource_id_before = None
            tool_name = "unknown"
            tool_args_json = "{}"
            
            # 1. Update the "Mental Model" (Blackboard) first
            if step > 0: 
                s = prompt_blackboard_update(s, current_blackboard, tool_result)
                current_blackboard = s["blackboard_json"] 

            # 2. The Meta-Cognitive Checkpoint
            s = prompt_meta_strategy(s, current_blackboard, tools_catalog_text)
            strategy = s["strategy_select"].strip()

            tool_result = "" # Initialize for this turn

            # 3. Branch based on strategy
            if strategy == "DECOMPOSITION":
                # HANDLING COMPLEX SUBGOALS
                sub_goal = s["next_intent"]
                s += assistant(f"I will delegate '{sub_goal}' to a sub-planner.\n")
                
                # === Recursive Call Primitive ===
                # Assuming executor has this primitive as requested
                tool_name = "sub_planner"  # Virtual tool name for logging
                tool_args_json = json.dumps({"goal": sub_goal})
                
                sub_plan_result = executor.call_subplanner(
                    goal=sub_goal,
                    context=current_blackboard, # Pass the parent's knowledge state
                    initial_blackboard=initial_blackboard
                )
                tool_result = f"SUB-PLANNER REPORT:\n{sub_plan_result}"

            elif strategy == "BACKTRACK":
                # ERROR CORRECTION
                s += assistant(f"Current path is invalid. Reverting state.\n")
                # In a real implementation, you might revert the 's' object or flag the blackboard
                tool_result = "Action: BACKTRACK. The agent decided the previous path was a dead end."
                tool_name = "system_backtrack"

            elif strategy == "REFLECTION":
                # PURE THINKING
                thought_content = s["strategy_rationale"]
                tool_result = f"INTERNAL REFLECTION: {thought_content}"
                tool_name = "think"

            else: # strategy == "DIRECT_EXECUTION"
                # === THE ORIGINAL STAGE 2 LOGIC GOES HERE ===
                # Only now do we actually ask for a specific tool
                
                s += user(
                    f"Strategy: DIRECT_EXECUTION.\n"
                    f"Intent: {s['next_intent']}\n"
                    "Select the precise tool and arguments to fulfill this intent.\n"
                )
                
                s += assistant(
                    "TOOL_NAME: "
                    + gen(f"tool_name_{step}", max_tokens=32, stop="\n")
                    + "\nTOOL_ARGS_JSON: "
                    + gen(f"tool_args_{step}", max_tokens=1024, stop="\n")
                    + "\n"
                )
                
                # Execute standard tool (Your original execution logic)
                tool_name = s[f"tool_name_{step}"].strip()
                tool_args_json = s[f"tool_args_{step}"].strip()
                action = sgl_to_infospace_action(tool_name, tool_args_json, step, executor.available_tools)
                # FIX 2 (Part B): Capture resource_id_before ONLY here, where 'action' is valid
                out_var = action.get('out', '')
                if out_var:
                    resource_id_before = executor.plan_bindings.get(out_var.lstrip('$'))

                tool_result = execute_infospace_action(action, executor, executor.agent_name)

            # 4. Proceed to Stage 3 (Reflection)
            result_display = tool_result[:512]
            if len(tool_result) > 512:
                result_display += f"\n... [TRUNCATED - showing 512 of {len(tool_result)} chars]"
            
            s += user(
                f"========================================\n"
                f"STAGE 3 - TOOL EXECUTION COMPLETE (step {step + 1}/{max_steps})\n"
                f"========================================\n\n"
                f"Tool executed: `{tool_name}`\n"
                f"Arguments: {tool_args_json}\n\n"
                f">>> ACTUAL RESULT (ground truth) >>>\n"
                f"{result_display}\n"
                f">>> END RESULT >>>\n\n"
                f"INSTRUCTIONS:\n"
                f"1. The result above is GROUND TRUTH. Use it exactly as shown.\n"
                f"2. If reporting to user, use ONLY the values from the result above.\n"
                f"3. Do NOT approximate, summarize, or invent values.\n"
                f"4. Evaluate: Is the goal complete, including actual execution of all planned actions? If yes, respond DONE: YES.\n"
                f"   If no, determine the next action needed.\n\n"
                f"Respond using Stage 3 FORMAT. Be concise.\n"
                f"Ensure the REQUEST_TOOLS list is a valid JSON list of tool names.\n"
            )
            
            s += assistant(
                "THOUGHTS (Be concise): "
                + gen(f"thoughts_{step}", max_tokens=128, stop="\n")
                + "\nDONE: "
                + gen(f"done_{step}", max_tokens=8, stop="\n")
                + "\nNEXT_TASK: "
                + gen(f"next_task_{step}", max_tokens=128, stop="\n")
                + "\nREQUEST_TOOLS: "
                + gen(f"request_tools_{step}", max_tokens=128, stop=["\n\n", "\n\nSTAGE"])
                + "\n"
            )
            logger.info(f"THOUGHTS: {s[f'thoughts_{step}']}")
            logger.info(f"DONE: {s[f'done_{step}']}")
            logger.info(f"NEXT_TASK: {s[f'next_task_{step}']}")
            logger.info(f"REQUEST_TOOLS: {s[f'request_tools_{step}']}")
            
            # Stage 3.1: Update resource indexes with commentary
            # Track ANY resource created in this step (not just explicit create-note/create-collection)
            thoughts_text = s[f'thoughts_{step}'].strip()
            if thoughts_text:
                # Check if a new resource was bound in this step
                out_var = action.get('out', '')
                resource_id = None
                if strategy == "DIRECT_EXECUTION" and out_var:
                    resource_id_after = executor.plan_bindings.get(out_var.lstrip('$'))
                    # If resource_id changed (new resource created) or didn't exist before
                    if resource_id_after and resource_id_after != resource_id_before:
                        if resource_id_after.startswith('Note_') or resource_id_after.startswith('Collection_'):
                            resource_id = resource_id_after
                
                # Update index with commentary if resource was created
                if resource_id:
                    try:
                        if executor.resource_manager:
                            executor.resource_manager.update_resource_commentary(resource_id, thoughts_text)
                            logger.debug(f"Stage 3.1: Updated commentary for {resource_id} (created via {action.get('type')})")
                        else:
                            logger.debug(f"Stage 3.1: Resource manager not available for {resource_id}")
                    except Exception as e:
                        logger.debug(f"Stage 3.1: Failed to update commentary for {resource_id}: {e}")
            
            # Stage 3.5: Dynamic tool loading (if requested)
            requested_tools_raw = s[f"request_tools_{step}"].strip()
            requested_tools = parse_request_tools(requested_tools_raw)
            if requested_tools:
                logger.info(f"Step {step}: LLM requested additional tools: {requested_tools}")
                expanded_docs = load_skill_docs(requested_tools, executor.available_tools)
                if expanded_docs:
                    s += user(f"ADDITIONAL TOOL DOCUMENTATION:\n{expanded_docs}")
                    s += assistant("I have reviewed the additional tool documentation.\n")
                    logger.info(f"Stage 3.5: Loaded docs for {len(requested_tools)} additional tools: {requested_tools}")
            elif requested_tools_raw and requested_tools_raw.lower() not in ["", "[]", "none", "null"]:
                # Log warning only if there was actual content that failed to parse
                logger.warning(f"Step {step}: Failed to parse REQUEST_TOOLS: {requested_tools_raw[:100]}")
            
            # Check if done
            done_raw = s[f"done_{step}"].strip().upper()
            if done_raw.startswith("YES"):
                # Generate final answer using NEXT_TASK as prompt, or generic goal-focused prompt
                next_task_raw = s[f"next_task_{step}"].strip()
                if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                    final_prompt = next_task_raw
                else:
                    final_prompt = "Summarize the results with focus on the original goal"
                
                s += user(f"FINAL TASK: {final_prompt}\nProvide a concise final answer.")
                s += assistant(gen("final_answer", max_tokens=256, stop=["\n\n", "STAGE"]))
                logger.info(f"FINAL_ANSWER: {s['final_answer']}")
                break
            
            # Update current task for next iteration
            next_task_raw = s[f"next_task_{step}"].strip()
            if next_task_raw and next_task_raw.lower() not in ["", "none", "null", "n/a"]:
                current_task = next_task_raw
                logger.info(f"Step {step}: Next task: {current_task}")
            else:
                logger.warning(f"Step {step}: No NEXT_TASK provided, keeping current task")
        
        
        # Write full conversation state to trace file (file-only, not console)
        if trace_file:
            logger.info(f"Writing full conversation state to trace file")
            trace_file.write(f"\n{'='*80}\n")
            trace_file.write(f"Planning session: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            trace_file.write(f"Goal: {goal}\n")
            trace_file.write(f"total length: {len(str(s))}\n")
            trace_file.write(f"{'='*80}\n")
            trace_file.write(str(s)+"\n")
            trace_file.write(f"\n{'='*80}\n\n")
            trace_file.flush()
        return s


# Blackboard-specific prompt functions
def prompt_blackboard_update(s, current_blackboard, recent_observation):
    """
    Forces the model to explicitly update its mental model based on the last action.
    Generates a single JSON string variable.
    """
    s += user(
        f"### UPDATE KNOWLEDGE BLACKBOARD\n"
        f"Review your PREVIOUS BLACKBOARD and the RECENT OBSERVATION.\n"
        f"Update the blackboard to reflect the current state of investigation.\n\n"
        f"PREVIOUS BLACKBOARD:\n{current_blackboard}\n\n"
        f"RECENT OBSERVATION:\n{recent_observation}\n\n"
        f"INSTRUCTIONS:\n"
        f"Update the state JSON. Maintain these 4 fields:\n"
        f"1. confirmed_facts: Verified information (Immutable unless disproven)\n"
        f"2. open_hypotheses: Current intentions / theories\n"
        f"3. discarded_paths: Failed searches (to avoid loops)\n"
        f"4. pending_questions: Specific knowledge / informationgaps that need filling\n"
        f"Output valid JSON only."
    )
    
    s += assistant(
        gen("blackboard_json", max_tokens=1024, stop="\n\n")
    )

    logger.info(f"Generated Blackboard Update:\n{s['blackboard_json']}")
    return s

def prompt_meta_strategy(s, blackboard_state, tools_catalog):
    """
    Decides the high-level move: Breakdown, Execute, or Pivot.
    """
    if isinstance(blackboard_state, dict):
        blackboard_dict = blackboard_state
    elif isinstance(blackboard_state, str):
        parsed_blackboard = repair_json_string(blackboard_state)
        blackboard_dict = parsed_blackboard if isinstance(parsed_blackboard, dict) else {}
    else:
        blackboard_dict = {}
    
    # Extract tool status to force the LLM to acknowledge it
    has_tools = "Yes" if blackboard_dict.get('selected_tools') else "No"
    
    s += user(
        f"### META-COGNITIVE STRATEGY CHECK\n"
        f"Current Blackboard State: {blackboard_state}\n"
        f"Tools Selected and Ready: {has_tools}\n\n"
        f"Analyze your situation. Select the operational mode for the IMMEDIATE next step:\n\n"
        f"A) DIRECT_EXECUTION\n"
        f"   - Condition: You have identified specific tools (e.g., 'search-web') AND you know the specific input for the very first step.\n"
        f"   - Behavior: Stop planning. Execute the first tool immediately.\n"
        f"B) BACKTRACK\n"
        f"   - Condition: The current path has failed or produced empty results. Revert.\n"
        f"C) REFLECTION\n"
        f"   - Condition: You are confused or need to read documentation before selecting tools.\n\n"
        f"CRITICAL RULE: If you have already selected tools (e.g., search-web), you MUST choose DIRECT_EXECUTION to use them. Do not decompose an already-planned task.\n\n"
        f"Respond with:\n"
        f"STRATEGY: <A/B/C>\n"
        f"RATIONALE: <One sentence justification>\n"
        f"NEXT_INTENT: <The specific intent for the tool or sub-planner>"
    )

    s += assistant(
        "STRATEGY: " + select("strategy_select", choices=["DIRECT_EXECUTION", "BACKTRACK", "REFLECTION"]) + "\n"
        "RATIONALE: " + gen("strategy_rationale", max_tokens=64, stop="\n") + "\n"
        "NEXT_INTENT: " + gen("next_intent", max_tokens=64, stop="\n")
    )
    logger.info(f"Generated Meta-Strategy:\n   Strategy: {s['strategy_select']}\n   Rationale: {s['strategy_rationale']}\n   Next Intent: {s['next_intent']}")
    return s


class BlackboardPlanner(IncrementalPlanner):
    """
    Blackboard planner extending IncrementalPlanner with blackboard-based meta-cognitive planning.
    """
    
    def generate_plan(self, goal: str, context: Dict = None, max_steps: int = 16) -> Dict:
        """
        Generate plan incrementally using SGLang with blackboard pattern.
        
        Overrides IncrementalPlanner.generate_plan to handle initial_blackboard parameter.
        
        Args:
            goal: Goal text
            context: Optional dict with character_context, recent_context, initial_blackboard
            max_steps: Maximum planning steps
            
        Returns:
            Plan dict with 'plan' key containing actions
        """
        if not HAS_SGLANG:
            return {'error': 'SGLang not available'}
        
        try:
            # Clear executor state for new plan
            self.executor.clear_plan_state()
            
            # Attach plan_actions list to executor for tracking
            self.executor._plan_actions = []
            self.executor._side_effect_cache = {}  # Reset dedup cache per planner session

            # Extract context components
            character_context = ""
            recent_context = ""
            initial_blackboard = None
            if context:
                character_context = context.get('character_context', '')
                recent_context = context.get('recent_context', '')
                initial_blackboard = context.get('initial_blackboard')
            if hasattr(self.executor, 'character_context'):
                self.executor.character_context = character_context
            
            # Check total input size before calling tool_planner_infospace.run()
            total_input_size = (
                len(goal) +
                len(character_context) +
                len(recent_context) +
                len(self.tools_catalog_text)
            )
            if total_input_size > 100000:
                import traceback
                logger.error(f"⚠️  Attempting to send {total_input_size:,} chars to tool_planner_infospace.run() via BlackboardPlanner (limit: 100,000)")
                logger.error(f"  goal length: {len(goal):,}")
                logger.error(f"  character_context length: {len(character_context):,}")
                logger.error(f"  recent_context length: {len(recent_context):,}")
                logger.error(f"  tools_catalog_text length: {len(self.tools_catalog_text):,}")
                logger.error("Stack traceback:")
                for line in traceback.format_stack():
                    logger.error(line.rstrip())
            
            # Run SGLang planner (blackboard version)
            state = tool_planner_infospace.run(
                goal=goal,
                character_context=character_context,
                recent_context=recent_context,
                tools_catalog_text=self.tools_catalog_text,
                executor=self.executor,
                trace_file=self.trace_file,
                max_steps=max_steps,
                initial_blackboard=initial_blackboard
            )
            
            # Extract plan actions from executor
            plan_actions = getattr(self.executor, '_plan_actions', [])
            
            # Extract final_answer from ProgramState (use bracket notation)
            try:
                final_answer = state['final_answer']
            except (KeyError, TypeError):
                final_answer = 'Planning completed'
            
            return {
                'plan': plan_actions,
                'reasoning': final_answer,
                'success': True,
                'skip_validation': True  # Plan already executed, no need to validate
            }
        except Exception as e:
            self.logger.error(f"Blackboard planning failed: {e}")
            traceback.print_exc()
            return {'error': str(e)}
