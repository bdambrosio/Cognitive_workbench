# Method Tools: Twitter Thread

## Tweet 1 (Hook)
🧵 Just shipped something cool: "Method Tools" - a new way to give LLMs structured protocols they can execute autonomously.

Instead of hardcoding behavior or letting the LLM free-form plan, we define multi-step protocols that the planner interprets and executes iteratively.

#AI #LLM #AgenticAI

---

## Tweet 2 (The Problem)
The challenge: How do you give an AI agent a complex multi-step procedure (like "explore systematically") without:
- Hardcoding every step (too rigid)
- Letting it free-form plan (too unpredictable)
- Writing custom code for each procedure (not scalable)

Method tools solve this. 🧵

---

## Tweet 3 (The Solution)
Method tools define protocols as markdown documents with numbered steps, conditions, and termination criteria.

The LLM reads the protocol, selects tools at each step, reflects on results, and decides what to do next - all within a bounded inner loop.

It's like giving the LLM a recipe it can follow intelligently. 🧵

---

## Tweet 4 (Example - Exploration Protocol)
Example: An exploration method tool defines steps like:

```
STEP 1: Align to cardinal direction
STEP 2: Observe forward affordances  
STEP 3: Move forward if safe
STEP 4: Loop back to STEP 1 if more directions to explore
TERMINATION: All directions explored OR discovery made
```

The LLM executes this systematically, choosing tools at each step. 🧵

---

## Tweet 5 (Architecture - Inner/Outer Loops)
The architecture creates two execution levels:

**Outer loop**: High-level planning ("explore the area")
**Inner loop**: Method execution (follows exploration protocol step-by-step)

Each method execution counts as ONE outer step, but internally it's a bounded loop of tool selections and reflections. 🧵

---

## Tweet 6 (SGLang Implementation)
We use SGLang's structured generation to make this work:

- At each step, prompt LLM with current protocol step
- Generate tool selection (structured: TOOL_NAME + TOOL_ARGS_JSON)
- Execute tool
- Generate reflection + next step decision
- Repeat until termination

The LLM maintains protocol state across iterations. 🧵

---

## Tweet 7 (Key Innovation - Protocol as Data)
The key insight: Protocols are DATA, not CODE.

A method tool is just a markdown file with:
- YAML frontmatter (`type: method`)
- Structured protocol definition
- Step-by-step instructions

No custom code needed. Add a new protocol = add a new markdown file. 🧵

---

## Tweet 8 (Real-World Application)
We've built exploration protocols for game worlds, but the pattern applies anywhere you need systematic, conditional procedures:

- Recovery protocols (multi-step error recovery)
- Systematic operations (data collection, verification)
- Complex workflows (with branching logic)

The LLM interprets and adapts. 🧵

---

## Tweet 9 (Preventing Recursion)
Important constraint: Method tools can't call other method tools.

This prevents:
- Infinite recursion
- Unbounded complexity
- Hard-to-debug execution trees

If a method needs another method's functionality, it returns control to the outer planner, which can chain methods. 🧵

---

## Tweet 10 (Benefits)
Why this matters:

✅ **Structured**: Clear protocols with defined steps
✅ **Flexible**: LLM adapts to unexpected situations  
✅ **Scalable**: New protocols = new markdown files
✅ **Debuggable**: Clear execution traces with step numbers
✅ **Composable**: Outer planner chains methods as needed

🧵

---

## Tweet 11 (Code Example - Protocol Definition)
Here's what a method tool looks like:

```yaml
---
name: explore-area
type: method
description: "Systematically explores cardinal directions"
---

STEP 1 — ALIGN
- Call tool: mc-look
- Set yaw to nearest cardinal direction
- If direction explored: proceed to STEP 4
- Otherwise: proceed to STEP 2

STEP 2 — OBSERVE
- Call tool: mc-observe-blocks
- Evaluate forward distance
- If blocked: mark explored, proceed to STEP 4
- Otherwise: proceed to STEP 3
```

🧵

---

## Tweet 12 (Execution Flow)
When the planner selects a method tool:

1. Detects `type: method` → triggers inner loop
2. Loads protocol document
3. Starts at STEP 1
4. At each step:
   - Prompts LLM with current step
   - LLM selects tool + args
   - Executes tool
   - LLM reflects → determines next step
5. Loops until termination (SUCCESS/FAILED/INAPPLICABLE)

🧵

---

## Tweet 13 (SGLang Structured Generation)
SGLang makes this possible with structured generation:

```python
# Generate tool selection
s += assistant(
    "TOOL_NAME: " + gen(tool_name_key, stop="TOOL_ARGS_JSON")
    + "\nTOOL_ARGS_JSON: " + gen(tool_args_key, stop="\n")
)

# Generate reflection
s += assistant(
    "THOUGHTS: " + gen(thoughts_key)
    + "\nNEXT_TASK: " + gen(next_task_key)
    + "\nDONE: " + gen(done_key)
)
```

The `s` object maintains state across iterations. 🧵

---

## Tweet 14 (Metadata Tracking)
Every action executed within a method includes metadata:

```python
action['_inner_loop'] = {
    "method_name": "explore-area",
    "inner_step": 3,
    "max_steps": 24,
    "outer_step": 1
}
```

This enables:
- UI progress display
- Debugging traces
- Understanding execution context

🧵

---

## Tweet 15 (Termination)
Methods terminate when:

✅ **SUCCESS**: Protocol goal achieved
❌ **FAILED**: Cannot proceed (obstacles, errors)
🚫 **INAPPLICABLE**: Preconditions not met
⏱️ **TIMEOUT**: Max steps exceeded

The LLM writes "METHOD COMPLETE" in THOUGHTS when done, which triggers termination. 🧵

---

## Tweet 16 (Best Practices)
Writing effective method tools:

1. **Clear steps**: Each step = single purpose
2. **Explicit conditions**: When to proceed vs loop back
3. **Tool guidance**: Specify exact tools needed
4. **Termination**: Always define SUCCESS/FAILED/INAPPLICABLE
5. **No recursion**: Methods can't call methods

The protocol is the contract. 🧵

---

## Tweet 17 (Why Not Just Code?)
"Why not just write code for these protocols?"

Because:
- Code is rigid (can't adapt to unexpected situations)
- Code requires deployment (can't add protocols dynamically)
- Code is hard to modify (requires recompilation)

Method tools let the LLM interpret and adapt protocols while maintaining structure. 🧵

---

## Tweet 18 (The Balance)
Method tools find the sweet spot between:

**Too rigid** (hardcoded behavior)
↕️
**Too flexible** (free-form planning)

They provide **structured flexibility**: clear protocols that the LLM can interpret intelligently and adapt to context. 🧵

---

## Tweet 19 (Use Cases)
Where method tools shine:

🎮 **Game agents**: Exploration, navigation, resource gathering
🤖 **Robotics**: Systematic procedures with conditional logic
📊 **Data collection**: Multi-step verification workflows
🔧 **Recovery**: Error recovery protocols
🧪 **Experiments**: Systematic testing procedures

Anywhere you need systematic + adaptive behavior. 🧵

---

## Tweet 20 (Future Directions)
What's next:

- Method composition (chaining methods intelligently)
- Dynamic protocol generation (LLM creates protocols)
- Protocol learning (learn protocols from demonstrations)
- Multi-agent protocols (coordination between agents)

The foundation is there. Now we build on it. 🧵

---

## Tweet 21 (Closing)
Method tools: Protocols as data, executed by LLMs, structured by SGLang.

They bridge the gap between rigid code and free-form planning, enabling agents to follow complex procedures while maintaining the flexibility to adapt.

More details: [link to docs/METHOD_TOOLS.md]

🧵 /end

---

## Alternative Shorter Thread (10 tweets)

### Tweet 1
🧵 Just shipped "Method Tools" - a way to give LLMs structured protocols they execute autonomously.

Instead of hardcoding behavior or free-form planning, we define multi-step protocols as markdown. The LLM interprets and executes them step-by-step.

#AI #LLM

---

### Tweet 2
The architecture: Two-level execution.

**Outer loop**: High-level planning
**Inner loop**: Method protocol execution (bounded, step-by-step)

Each method counts as ONE outer step, but internally it's a loop of tool selections and reflections.

🧵

---

### Tweet 3
Example: Exploration protocol

```
STEP 1: Align to cardinal direction
STEP 2: Observe forward
STEP 3: Move if safe
STEP 4: Loop back if more directions
TERMINATION: All explored OR discovery
```

LLM executes this systematically, choosing tools at each step.

🧵

---

### Tweet 4
We use SGLang's structured generation:

- Prompt with current protocol step
- Generate tool selection (TOOL_NAME + ARGS)
- Execute tool
- Generate reflection + next step
- Repeat until termination

The LLM maintains protocol state across iterations.

🧵

---

### Tweet 5
Key insight: Protocols are DATA, not CODE.

A method tool = markdown file with:
- YAML frontmatter (`type: method`)
- Structured protocol steps
- Termination conditions

No custom code. New protocol = new markdown file.

🧵

---

### Tweet 6
Why this matters:

✅ Structured (clear protocols)
✅ Flexible (LLM adapts)
✅ Scalable (just add markdown files)
✅ Debuggable (clear execution traces)
✅ Composable (chain methods)

🧵

---

### Tweet 7
Constraint: Methods can't call methods.

Prevents recursion/complexity. If a method needs another method, it returns to outer planner, which chains them.

This keeps execution trees manageable and debuggable.

🧵

---

### Tweet 8
Use cases:

🎮 Game agents (exploration, navigation)
🤖 Robotics (systematic procedures)
📊 Data collection (multi-step workflows)
🔧 Recovery (error protocols)

Anywhere you need systematic + adaptive behavior.

🧵

---

### Tweet 9
The balance:

Method tools find the sweet spot between:
- Too rigid (hardcoded)
- Too flexible (free-form)

They provide **structured flexibility**: clear protocols that LLMs interpret intelligently.

🧵

---

### Tweet 10
Method tools: Protocols as data, executed by LLMs, structured by SGLang.

They bridge rigid code and free-form planning, enabling agents to follow complex procedures while adapting to context.

More: [link to docs]

🧵 /end

