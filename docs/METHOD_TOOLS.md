# Method Tools: Protocol-Based Tool Execution

## Overview

Method tools are a special class of tools that encapsulate multi-step protocols executed by the planner itself. Unlike standard tools that execute code or primitives directly, method tools define a sequence of steps that the planner interprets and executes iteratively, creating an "inner loop" within the outer planning loop.

This document describes the method tool format, how they're executed using SGLang, and best practices for creating method tools.

---

## What Are Method Tools?

Method tools are tools that define **protocols** rather than direct actions. They specify a sequence of steps that the planner should follow, with conditional logic and termination conditions. The planner executes these steps iteratively, selecting and executing appropriate tools at each step based on the protocol definition.

### Key Characteristics

- **Protocol-based**: Defined as a sequence of steps with conditions
- **Inner loop execution**: Executed as a bounded inner loop within the outer planning loop
- **LLM-driven**: The planner (LLM) interprets the protocol and selects tools at each step
- **Self-contained**: Encapsulate complex multi-step behaviors
- **Non-recursive**: Method tools cannot invoke other method tools (prevents complexity)

### Example Use Cases

Method tools are ideal for:
- **Exploration protocols**: Systematic exploration of environments (e.g., exploring terrain in a game world)
- **Recovery procedures**: Multi-step recovery from error states
- **Systematic operations**: Operations that require following a specific sequence with conditional branching
- **Complex behaviors**: Behaviors too complex for a single tool but too structured for ad-hoc planning

---

## Method Tool Format

Method tools are defined using `Skill.md` files with YAML frontmatter. The key difference from standard tools is the `type: method` field.

### Frontmatter

```yaml
---
name: tool-name
type: method
description: "Brief description of what this method does"
resumable: yes  # Optional: indicates method can be resumed
invalidates: [position, orientation]  # Optional: state that becomes invalid during execution
---
```

**Required fields:**
- `name`: Tool identifier
- `type`: Must be `"method"` to mark as method tool
- `description`: Brief description for tool catalog

**Optional fields:**
- `resumable`: Whether method can be resumed if interrupted
- `invalidates`: List of state variables that become invalid during execution

### Protocol Definition

The body of the `Skill.md` file defines the protocol using structured sections:

#### 1. PURPOSE
Describes the high-level goal of the method.

```
#PURPOSE:
Incrementally explore nearby terrain to gain situational awareness and discover
navigable paths, exits, hazards, or geometry that warrants a more specialized skill.
```

#### 2. PRECONDITIONS
Conditions that must be met before the method should be used.

```
#PRECONDITIONS (WHEN THIS SKILL APPLIES)

- The agent is not currently executing a higher-priority recovery or escape skill.
- The agent can rotate in place and attempt forward movement.
- No immediate vertical entrapment is detected.
```

#### 3. DESIGN CONSTRAINTS
Intentional constraints (not enforced, but guide behavior).

```
#DESIGN CONSTRAINTS (INTENT, NOT ENFORCED)

- Avoid stepping onto blocks with unknown or unsafe support below.
- Prefer solid ground over air when probing movement.
- Prefer unexplored directions over recently attempted ones.
```

#### 4. LIMITATIONS
Important limitations, especially regarding recursion.

```
#LIMITATIONS:
- Method tools cannot invoke other method tools (prevents recursion and complexity)
- To loop within a method, use "Return to STEP X" (internal loop within this method execution)
- If a method needs another method's functionality, return control to the outer planner
```

#### 5. RUNTIME STATE
State variables maintained during method execution (conceptual, not enforced).

```
#RUNTIME STATE (LOCAL TO THIS SKILL)

- explored_directions : set of {N, E, S, W} (initially empty)
- last_safe_position : agent position at skill entry
- steps_since_observation : integer (initially 0)
```

#### 6. MAIN PROTOCOL LOOP
The core protocol definition with numbered steps.

```
#MAIN PROTOCOL LOOP

Repeat until a termination condition is met.

---
STEP 1 — ALIGN
---

- Snap yaw to the nearest cardinal direction
- If current facing direction ∈ explored_directions:
    Loop back to STEP 1 (within this method execution).

---
STEP 2 — OBSERVE FORWARD AFFORDANCES
---

- Call mc-observe-blocks.
- Evaluate forward distance, block at (forward:1, up:-1), clearance
- If forward distance < 1.0:
    Mark direction explored and loop back to STEP 1 (within this method execution).
- Otherwise:
    Proceed to STEP 3.

---
STEP 3 — PROBE MOVE
---

- Execute mc-move with forward: true
- Execute mc-wait for 0.5 seconds.

---
STEP 4 — VERIFY SAFETY
---

- Observe agent position.
- If Y decreased unexpectedly OR footing appears unsafe:
    Attempt to recover if possible.
    Mark direction explored.
    Loop back to STEP 1 (within this method execution).
- Otherwise:
    Proceed to STEP 5.

---
STEP 5 — PERIODIC SCAN
---

- Call mc-observe-blocks.
- Check for sky visible above, clear downward shafts, open terrain
- If any are detected:
    Proceed to TERMINATION (DISCOVERY).
- Otherwise:
    Proceed to STEP 6.

---
STEP 6 — MARK PROGRESS
---

- Add current facing direction to explored_directions.
- Loop back to STEP 1 (within this method execution).

#TERMINATION CONDITIONS

DISCOVERY:
- Sky becomes visible.
- The agent can gain height without terrain modification.
- Proceed to TERMINATION (DISCOVERY).

FAILED:
- All directions explored without discovery.
- Return FAILED.

INAPPLICABLE:
- Method cannot proceed due to preconditions not met.
- Return INAPPLICABLE.
```

---

## How SGLang Executes Method Tools

Method tools are executed using SGLang's structured generation capabilities, creating an inner loop that runs within the outer planning loop.

### Execution Flow

1. **Detection**: When the planner selects a tool with `type: method`, it triggers `run_method_protocol()`
2. **Inner Loop**: A bounded loop (up to `max_steps`) executes protocol steps
3. **Step Execution**: At each step, the LLM:
   - Reads the current method step
   - Selects the appropriate tool for that step
   - Executes the tool
   - Reflects on the result
   - Determines the next step based on the protocol
4. **Termination**: Loop exits when:
   - Method completes (SUCCESS/FAILED/INAPPLICABLE)
   - Maximum steps reached
   - User interrupt

### SGLang Implementation

The `run_method_protocol()` function uses SGLang's structured generation:

```python
def run_method_protocol(s, executor, method_name: str, max_steps: int, 
                        outer_step: int, loaded_skill_docs: set) -> str:
    """
    Execute a 'method' tool as an inner loop (bounded by max_steps).
    Returns a short summary string for the outer loop.
    """
    method_task = "STEP 1"
    for mstep in range(max_steps):
        # Prompt LLM with current method step
        s += user(
            f"#METHOD EXECUTION MODE: {method_name} (internal step {mstep + 1}/{max_steps})\n"
            f"CURRENT METHOD STEP: {method_task}\n"
            "Select the tool explicitly required by the current Method Step.\n"
            "Choose tool and JSON args using Stage 2 FORMAT.\n"
        )
        
        # Generate tool selection
        tool_name_key = f"m_tool_name_{outer_step}_{mstep}"
        tool_args_key = f"m_tool_args_{outer_step}_{mstep}"
        s += assistant(
            "TOOL_NAME: "
            + gen(tool_name_key, max_tokens=32, temperature=GEN_TEMPERATURE, stop="TOOL_ARGS_JSON")
            + "\nTOOL_ARGS_JSON: "
            + gen(tool_args_key, max_tokens=1024, temperature=GEN_TEMPERATURE, stop="\n")
            + "\n"
        )
        
        # Execute tool
        tool_name = s[tool_name_key].strip()
        tool_args_json = s[tool_args_key].strip()
        
        # Prevent method recursion
        if tool_info.get('type') == 'method':
            return f"FAILED | Method {method_name} cannot invoke method tool '{tool_name}'"
        
        action = sgl_to_infospace_action(tool_name, tool_args_json, ...)
        last_tool_result = execute_infospace_action(action, executor, ...)
        
        # Reflect on result and determine next step
        s += user(
            f"METHOD STAGE 3 - TOOL EXECUTION COMPLETE (internal step {mstep + 1}/{max_steps})\n"
            f"Tool executed: `{tool_name}`\n"
            f"Arguments: {tool_args_json}\n\n"
            f">> ACTUAL RESULT (ground truth) <<\n"
            f"{result_display}\n"
            f">> END RESULT <<\n\n"
            f"METHOD NEXT_TASK INSTRUCTIONS:\n"
            f"Identify the Exact Step defined in the {method_name} manual that matches the ACTUAL RESULT above.\n"
            f"NEXT_TASK must be written as: [METHOD: STEP X] <Instruction from manual>.\n"
            f"Do not invent new steps.\n"
            f"TERMINATION: When the Method says terminate (SUCCESS/FAILED/INAPPLICABLE), write 'METHOD COMPLETE' in THOUGHTS.\n"
        )
        
        # Generate reflection
        thoughts_key = f"m_thoughts_{outer_step}_{mstep}"
        next_task_key = f"m_next_{outer_step}_{mstep}"
        done_key = f"m_done_{outer_step}_{mstep}"
        
        s += assistant(
            "\nTHOUGHTS: "
            + gen(thoughts_key, max_tokens=128, temperature=GEN_TEMPERATURE, stop="HYPOTHESES: ")
            + "\nNEXT_TASK: "
            + gen(next_task_key, max_tokens=128, temperature=GEN_TEMPERATURE, stop="\nDONE: ")
            + "\nDONE: "
            + gen(done_key, max_tokens=8, temperature=GEN_TEMPERATURE, stop="\n")
        )
        
        # Check for termination
        if "METHOD COMPLETE" in s[thoughts_key]:
            return f"SUCCESS | Method {method_name} completed"
        
        # Update method_task for next iteration
        method_task = s[next_task_key].strip()
    
    return f"FAILED | Method {method_name} exceeded max_steps"
```

### Key SGLang Features Used

1. **Structured Generation**: Uses `gen()` with specific keys for each field
2. **Stop Tokens**: Controls generation boundaries (e.g., `stop="TOOL_ARGS_JSON"`)
3. **State Persistence**: The `s` object maintains conversation state across iterations
4. **Temperature Control**: Different temperatures for different generation tasks
5. **Bounded Loops**: Maximum step limit prevents infinite loops

---

## Inner Loop vs Outer Loop

Method tools create a **two-level execution model**:

### Outer Loop (Planning)
- **Purpose**: High-level goal achievement
- **Steps**: Select tools, execute, reflect
- **Scope**: Entire plan execution
- **Tools**: Can select method tools or standard tools

### Inner Loop (Method Execution)
- **Purpose**: Execute method protocol steps
- **Steps**: Follow protocol, select tools per step, reflect
- **Scope**: Single method execution
- **Tools**: Can only select standard tools (no method recursion)

### Visual Representation

```
Outer Loop (Planning)
├── Step 1: Select tool → mc-explore (method tool)
│   └── Inner Loop (Method Execution)
│       ├── Step 1: Select tool → mc-look
│       ├── Step 2: Select tool → mc-observe-blocks
│       ├── Step 3: Select tool → mc-move
│       ├── Step 4: Select tool → mc-status
│       └── ... (continues until termination)
├── Step 2: Select tool → mc-map-update
└── Step 3: Select tool → persist
```

### Metadata Tracking

Actions executed within method tools include `_inner_loop` metadata:

```python
action['_inner_loop'] = {
    "method_name": method_name,
    "inner_step": mstep + 1,
    "max_steps": max_steps,
    "outer_step": outer_step
}
```

This metadata enables:
- UI display of inner loop progress
- Debugging and trace analysis
- Understanding execution context

---

## Best Practices

### Protocol Design

1. **Clear Step Definitions**: Each step should have a single, clear purpose
2. **Explicit Conditions**: Clearly state when to proceed vs. loop back
3. **Termination Conditions**: Define clear SUCCESS/FAILED/INAPPLICABLE conditions
4. **Tool Selection Guidance**: Specify which tools are required for each step

### Step Formatting

- Use numbered steps: `STEP 1`, `STEP 2`, etc.
- Include step titles: `STEP 1 — ALIGN`
- Use clear conditional logic: `If X, then Y, otherwise Z`
- Specify loop targets: `Loop back to STEP X (within this method execution)`

### Tool Selection

- **Be explicit**: Specify exact tool names required for each step
- **Avoid ambiguity**: Don't leave tool selection open-ended
- **Document dependencies**: Note which tools are required

### Termination

- **Always define termination**: Every method should have clear termination conditions
- **Use standard outcomes**: SUCCESS, FAILED, INAPPLICABLE
- **Provide context**: Explain why termination occurred

### Limitations

- **No method recursion**: Method tools cannot invoke other method tools
- **Bounded execution**: Methods have maximum step limits
- **State is conceptual**: Runtime state (like `explored_directions`) is tracked by the LLM, not enforced

---

## Example: Complete Method Tool

Here's a simplified example of a method tool for systematic exploration:

```markdown
---
name: explore-area
type: method
description: "Systematically explores an area by checking cardinal directions"
---

#PURPOSE:
Explore all four cardinal directions from current position to discover
exits, hazards, and affordances.

#PRECONDITIONS (WHEN THIS SKILL APPLIES)

- Agent is at a stable position
- Agent can rotate and move forward
- No immediate hazards detected

#LIMITATIONS:
- Method tools cannot invoke other method tools
- To loop within a method, use "Return to STEP X" (internal loop)

#MAIN PROTOCOL LOOP

---
STEP 1 — ALIGN TO CARDINAL
---

- Call tool: mc-look
- Set yaw to nearest cardinal direction (0°, 90°, 180°, -90°)
- If this direction already explored:
    Proceed to STEP 4.
- Otherwise:
    Proceed to STEP 2.

---
STEP 2 — OBSERVE FORWARD
---

- Call tool: mc-observe-blocks
- Evaluate forward distance and clearance
- If forward distance < 1.0 blocks:
    Mark direction as explored
    Proceed to STEP 4.
- Otherwise:
    Proceed to STEP 3.

---
STEP 3 — MOVE FORWARD
---

- Call tool: mc-move with forward: true, duration: 2.0
- Call tool: mc-status to verify safety
- If position changed safely:
    Mark direction as explored
    Proceed to STEP 4.
- Otherwise:
    Mark direction as explored (unsafe)
    Proceed to STEP 4.

---
STEP 4 — CHECK COMPLETION
---

- If all four cardinal directions explored:
    Proceed to TERMINATION (SUCCESS).
- Otherwise:
    Loop back to STEP 1 (within this method execution).

#TERMINATION CONDITIONS

SUCCESS:
- All four cardinal directions have been explored.
- Return SUCCESS.

FAILED:
- Cannot proceed due to obstacles or hazards.
- Return FAILED.
```

---

## Integration with Tool Catalog

Method tools are included in the tool catalog with their `type` field:

```python
tools[tool_name] = {
    "description": description,
    "schema_hint": schema_hint,
    "type": "method"  # Identifies as method tool
}
```

The planner uses this `type` field to:
1. Identify method tools during tool selection
2. Route execution to `run_method_protocol()` instead of direct execution
3. Prevent method recursion (methods cannot call other methods)

---

## Debugging Method Tools

### Common Issues

1. **Infinite loops**: Method doesn't terminate properly
   - **Solution**: Ensure clear termination conditions and step limits

2. **Wrong tool selection**: LLM selects incorrect tool for a step
   - **Solution**: Be more explicit in step descriptions about required tools

3. **Step confusion**: LLM doesn't follow protocol correctly
   - **Solution**: Use clearer step formatting and explicit loop targets

4. **Method recursion**: Method tries to call another method
   - **Solution**: Explicitly prevented by code, but ensure protocol doesn't suggest it

### Tracing Execution

Method execution traces include:
- Inner loop step numbers
- Tool selections at each step
- Reflection and next task decisions
- Termination reasons

Look for patterns like:
```
#METHOD EXECUTION MODE: explore-area (internal step 1/24)
CURRENT METHOD STEP: [METHOD: STEP 1] Align to cardinal direction
Tool executed: `mc-look`
METHOD STAGE 3 - TOOL EXECUTION COMPLETE (internal step 1/24)
NEXT_TASK: [METHOD: STEP 2] Observe forward affordances
```

---

## Summary

Method tools provide a powerful way to encapsulate multi-step protocols that require LLM interpretation and conditional execution. They enable:

- **Structured behaviors**: Complex protocols with clear steps
- **Conditional logic**: Branching based on observations
- **Reusable patterns**: Encapsulate common multi-step operations
- **LLM-driven execution**: Leverage LLM reasoning for step selection

By combining SGLang's structured generation with protocol definitions, method tools bridge the gap between rigid code execution and flexible ad-hoc planning, enabling agents to follow complex, conditional protocols while maintaining the flexibility to adapt to unexpected situations.

