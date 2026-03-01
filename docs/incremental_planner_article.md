# What's wrong with React as an Agent loop,

## Building a real-time agentic loop that plans, executes, recovers from errors, and checks its own work

---

Most LLM agent frameworks treat tool use as a one-shot affair: the model picks a tool, calls it, and moves on. If something goes wrong, you either retry the same call or bail out. That's fine for demos. It falls apart when you need an agent to execute multi-step plans against live data sources — web searches, document synthesis, structured extraction — where each step's output feeds the next, errors are routine, and the final artifact needs to actually be *good*.

The Cognitive Workbench's `incremental_planner` takes a different approach. It implements a ReAct-style planning loop with three layers of intelligence: an abstract plan that provides strategic direction, a reflective execution loop that reasons about each action's results before choosing the next, and an independent quality assurance system that evaluates the final output against machine-generated acceptance criteria. The whole thing runs in real-time — under 30 seconds for most goals — thanks to SGLang's guided generation.

Here's how each piece works.

---

## 1. The Abstract Plan: Strategy Before Tactics

Before executing any tools, the planner runs two preliminary stages that frame the entire execution.

**Stage 0** retrieves relevant resources from the agent's knowledge base — prior results, saved documents, entity context — so the planner knows what it already has before searching for more.

**Stage 1** performs strategic analysis. Given the goal, available tools, and retrieved context, the LLM produces three things:

- A **reasoning block** — 1-2 sentences explaining whether the goal is feasible and what approach to take
- A **tool selection** — a JSON list of tools needed (e.g., `["search-web", "extract", "synthesize"]`)
- A **first task** — a concrete description of what to do first

This is *not* a rigid multi-step plan. It's closer to a compass heading. The planner doesn't try to script every step in advance — it identifies the right tools and the right starting point, then lets the execution loop figure out the details incrementally. This matters because real execution is full of surprises: a web search might return unexpected data, an extraction might need a different instruction, and error recovery might require a completely different approach than originally planned.

Stage 1.5 then loads detailed documentation for the selected tools — not the full tool catalog, just the ones the LLM chose. This keeps the context window lean while giving the code generator the specifics it needs.

---

## 2. Reflective Execution: Think, Act, Reflect

The core of the planner is a `for step in range(max_steps)` loop that alternates between code generation and structured reflection.

### Stage 2: Act

Each step generates a Python code block that calls tools through a tracked executor:

```python
r1 = executor.execute_action_tracked({
    "type": "search-web",
    "query": "current weather forecast for Berkeley, CA",
    "out": "$weather_forecast"
}, "codegen")

if r1["status"] != "success":
    return executor._create_uniform_return("failed",
        reason="Web search failed")

return executor._create_uniform_return("success",
    value="Weather forecast retrieved",
    extra={"resource_id": r1["resource_id"]})
```

Code generation uses a low temperature (0.2) for correctness. The generated code runs in a sandboxed namespace — exceptions are caught and returned as structured error results rather than crashing the loop.

This is a key design decision: **errors are data, not exceptions.** When a code block throws an `IndexError` because it tried to parse text with a brittle `.split()`, the error message flows back into the next iteration as part of the execution result. The LLM sees exactly what went wrong and can adjust.

### Stage 3: Reflect

After each action executes, the LLM produces structured reflection across five fields:

- **THOUGHTS** — What happened? What does the result mean? What should change?
- **EVAL_TARGET** — Which resource contains the current best output?
- **DONE** — Is the goal achieved? (YES/NO)
- **NEXT_TASK** — If not done, what's the next concrete step?
- **REQUEST_TOOLS** — Any additional tools needed that weren't in the original selection?

This reflection is where error recovery actually happens. Consider a real execution trace: at step 2, the code block tries to format a weather report using `text.split("Temperature: ")` and crashes with `IndexError: list index out of range`. The error flows into stage 3, and the LLM's THOUGHTS field produces:

> *"The previous attempt to format the report failed due to an index error, likely because the extracted text did not contain the expected labels in the assumed format. I should use the full content directly without assuming internal label structure."*

The next step's code block uses `executor.get_text("$summary")` directly instead of fragile string parsing. No retry logic. No error-handling scaffolding. The LLM *understood what went wrong* and *changed its approach*. This is what makes reflective execution fundamentally different from simple retry loops.

A stall guard monitors for degenerate patterns — if the LLM generates the same code or the same NEXT_TASK repeatedly, the guard intervenes and delivers the best available artifact rather than burning through step budget.

---

## 3. Quality Assurance: Envision, Check, Verify

The execution loop can produce an artifact that the *planner* thinks is done, but that doesn't mean it's actually good. The quality assurance system provides independent evaluation through three mechanisms.

### Envision: Define Success Up Front

Before any execution begins, a separate LLM call generates **vision criteria** — 1-3 testable predicates that define what a failed output looks like for this specific goal. For a weather forecast goal:

```
1. empty_output: "Output is completely empty or contains only whitespace."
2. off_topic: "Output contains no mention of weather, forecast, or Berkeley, CA."
3. structurally_broken: "Output is malformed such that it cannot be read as a
   coherent text report."
```

These criteria are deliberately minimal — they define the floor, not the ceiling. The point isn't to specify what a *great* result looks like, but to catch results that are clearly broken: empty outputs, off-topic content, garbled text.

### Shallow Check: Fast Per-Step Screening

`_vision_eval_check` runs a lightweight evaluation after significant tool actions during execution. It loads a preview of the current artifact (first 4096 characters) and asks the LLM to verdict each criterion as PASS or FAIL. This runs in about 2-3 seconds and provides mid-loop advisory feedback — if a FAIL is detected, it's injected into the conversation as a signal the planner can optionally act on.

### Deep Check: Cross-Referenced Final Gate

When the planner declares DONE, `_vision_eval_deep` runs a more thorough evaluation. Unlike the shallow check, it:

1. Loads the **full artifact** (no truncation)
2. Loads up to 3 **upstream source Notes** — the intermediate artifacts that fed into the final output
3. **Cross-references** the artifact against those sources

This is the critical difference. The shallow check asks "does this look like a weather report?" The deep check asks "does this weather report actually reflect what the web search returned?" It catches cases where the formatting step corrupted the data, dropped important details, or hallucinated content that wasn't in the sources.

The deep check returns PASS, FAIL, or INAPPLICABLE per criterion, plus a STATUS line: `SATISFIED` or `NEEDS_REVISION`.

If the status is `NEEDS_REVISION`, the planner gets **one retry**. The evaluation feedback is injected directly into the conversation:

> *QUALITY GATE FAILED — you must revise the artifact. Issues found: [specific failures]. Regenerate or fix the artifact to address the FAILed criteria, then mark DONE again.*

The planner re-enters the execution loop with full knowledge of what went wrong and why. If the retry also fails the deep check, the best available draft is delivered with a quality caveat rather than silently passing through a broken artifact.

---

## 4. SGLang: Making This Fast Enough to Be Useful

All of the above — abstract planning, reflective execution, vision evaluation — involves many LLM calls per goal. A naive implementation with standard API calls would take minutes. SGLang's guided generation makes it practical for real-time use.

The key mechanism is the `@function` decorator and the `gen()` primitive. Instead of making separate API calls for each field in stage 3, SGLang fills multiple **named slots** within a single assistant turn:

```python
s += assistant(
    "THOUGHTS: "
    + gen("thoughts", max_tokens=192,
          temperature=0.5, stop="\nEVAL_TARGET")
    + "\nEVAL_TARGET: "
    + gen("eval_target", max_tokens=64,
          temperature=0.1, stop="\nDONE")
    + "\nDONE: "
    + gen("done", max_tokens=8,
          temperature=0.1, stop="\n")
    + "\nNEXT_TASK: "
    + gen("next_task", max_tokens=128,
          temperature=0.5, stop="\nREQUEST_TOOLS")
    + "\nREQUEST_TOOLS: "
    + gen("request_tools", max_tokens=64,
          temperature=0.1, stop="\n")
)
```

This is a single forward pass that produces all five fields. Each `gen()` call defines a named slot with its own `max_tokens`, `temperature`, and stop sequence. The state object `s` accumulates the full conversation as a dict — `s["thoughts"]`, `s["done"]`, etc. — making slot access trivial.

Temperature tuning per slot is important. Code generation (0.2) and tool selection (0.1) need precision. Reasoning and task planning (0.5) benefit from more creativity. With standard APIs, you'd need separate calls to use different temperatures; SGLang handles this within a single structured generation.

The `s += user(...)` / `s += assistant(...)` pattern manages the full conversation state as a growing dict structure. Each step's results are appended as new user messages, and the next stage's generation sees the full history. This is functionally equivalent to multi-turn chat, but managed as a single program with typed slots rather than a raw message array.

For deployments without SGLang (using vLLM, OpenRouter, or Anthropic backends), the planner falls back to a string-based path that builds the prompt as a formatted string and uses a `vllm_gen_multi()` helper to approximate multi-slot generation in a single API call. The logic is identical; only the generation mechanics differ.

---

## The Result

A typical goal — "search for Berkeley, CA weather forecast, summarize, format, and report" — executes in 25-35 seconds total:

- Stage 0/1 (planning): ~3 seconds
- Stage 2/3 loop (3-4 steps): ~20 seconds
- Vision eval (shallow + deep): ~5 seconds

The planner searches the web, extracts key data points, formats them into a structured report, persists it, and independently verifies the output quality — all within a single real-time interaction cycle. When things go wrong (and they do — brittle code generation, unexpected data formats, incomplete extractions), the reflective loop recovers without external intervention.

The deeper lesson is architectural: an effective agent loop needs more than just tool calling. It needs *strategic framing* (what tools, what approach), *tactical reflection* (what happened, what to do next), and *independent verification* (is the output actually good). Each layer operates at a different level of abstraction, and each catches failures that the others miss.

---

*The incremental planner is part of [Cognitive Workbench](https://github.com/yourusername/cognitive-workbench), an open-source framework for building reflective AI agents with persistent knowledge and structured tool use.*
