# Sensors vs. Tools: Adding Asynchronous Perception to an Agent Architecture

*This is a design note from ongoing work on Cognitive Workbench, a Python-based agent orchestration platform focused on giving AI agents genuine planning, reflection, and self-monitoring capabilities.*

---

## The synchronous tool model

Most older LLM agent frameworks — AutoGPT, CrewAI, LangGraph, and others — treat information gathering the same way they treat action: the agent decides it needs something, calls a tool, waits for the result, and continues. Search the web. Query a database. Read a file. It's a synchronous, pull-based model. The agent is in control of when information arrives.

This works well for deliberate, goal-directed work. If an agent is writing a research summary and needs to look up a paper, a synchronous tool call is exactly the right pattern. The agent knows what it wants, asks for it, and gets it.

In Cognitive Workbench, tools live in `src/tools/`, each in its own directory with a `SKILL.md` descriptor and either a Python implementation or a static plan. The tool loader scans the directory at startup, builds a catalog, and makes tools available to the planner. When the agent needs to use a tool, the planner emits an action, the executor calls the tool, and the result flows back into the planning loop. Synchronous. Pull. Agent-initiated.

## Where synchronous breaks down

The problem shows up when you want an agent to be *aware* of things it didn't ask about.

Consider an agent that should stay current on new papers in its research area. Under the synchronous model, the agent either needs a manually programmed 'cron' job, or has to *decide* to check for new papers. That means some part of its reasoning loop needs to periodically think "I should go look at arxiv now." This is cognitive overhead — the agent is spending inference cycles deciding *when* to gather information rather than *what to do with it*.

Worse, it makes the agent unable to react to events it doesn't know to look for. If something happens between polling intervals, the agent misses it entirely until its next scheduled check. And if the agent is deep in a multi-step plan, it may not check at all — the planning loop has its own momentum, and "maybe I should poll my information sources" isn't part of the current task.

This is the classic polling-vs-push problem, and in Cognitive Workbench we chose the same answer as elsewhere in systems design: if you want responsiveness to external events, you need a push model.

## Sensors: asynchronous push

We're adding a second category of extension library to Cognitive Workbench: **sensors**. Where tools are synchronous calls invoked by the agent, sensors are asynchronous processes that run independently and push results to the agent.

Sensors live in `src/sensors/`, following the same directory convention as tools. Each has a `SKILL.md` with metadata and either a Python implementation or a static plan. But that's where the similarity ends.

A sensor is configured per-character in the scenario YAML file:

```yaml
characters:
  - name: Researcher
    sensors:
      - name: arxiv-monitor
        schedule: "30m"
        disposition: "inform"
        parameters:
          search_terms: ["multi-agent systems", "LLM planning"]
      - name: api-health
        schedule: "1m"
        disposition: "alert"
        parameters:
          endpoint: "https://..."
```

The scenario designer — not the agent — decides what sensors a character has. Each sensor runs on its own thread, on its own schedule, and pushes results to the agent's existing sense-data channel (a Zenoh pub/sub topic). The agent encounters sensor output the same way it encounters any other incoming information: during the observe phase of its OODA loop.

The agent doesn't know sensors exist. It just lives in a world where information sometimes arrives.

## Two types of sensor

Not every monitoring task needs an LLM. Checking an RSS feed for new items is deterministic string matching. Checking whether a file has changed is a stat call. Forcing these through an LLM-driven execution loop wastes inference for no benefit.

So sensors come in two types:

**Code-type sensors** run a Python function directly. The function receives a context (parameters, resource manager access, last-run timestamp) and returns either content to push or a "nothing to report" signal. An RSS watcher, a file monitor, a health check endpoint — these are code-type sensors.

**Plan-type sensors** execute a fixed plan — a predefined sequence of infospace actions — via the same `execute_plan_sync` mechanism used for plan-type tools or cached goal plans. The plan can include LLM-powered steps (generate, refine, summarize), but the plan structure itself is static. The sensor doesn't *decide what to do*; it executes the same plan every time. This is the right fit for tasks like "search for new papers on topic X and summarize anything interesting" — you need the LLM for summarization, but the workflow is the same every run.

The key architectural constraint: **sensors execute, they don't plan.** A sensor never invokes the incremental planner. It never generates new plans in response to what it finds. If a sensor discovers something that needs adaptive reasoning, it pushes the finding to the agent and lets the agent reason about it. This is a firm design choice, a boundary between perception and cognition. If you want more planning / reasoning, launch another agent.

## The disposition problem

Once sensors are pushing information to the agent, there's an immediate question: what does the agent do with it?

Our first instinct was to make sensors fully invisible — the agent just sees new information appearing and figures out what it means. This is philosophically appealing (perception below the cognitive horizon, like peripheral vision) but practically unworkable. The agent has no framework for evaluating unsolicited information. Everything arriving on the sense-data channel currently gets treated as either a goal command or conversational input. Sensor output is neither.

Our solution (hack?) is a **disposition** — a declaration, made by the scenario designer in the sensor config, of what the agent should do when the sensor fires. The disposition travels with the sensor output as metadata:

- **`inform`** — Add to the agent's awareness context. No immediate action. The agent may use it in its next planning cycle, or not.
- **`trigger:goal-name`** — Activate or reprioritize a specific scheduled goal. The sensor has detected a condition that a particular goal should respond to.
- **`alert`** — High-priority item. Surfaces immediately in the agent's next reasoning cycle.

This keeps the routing decision with the scenario designer. No LLM inference spent on salience evaluation. No ambiguity about what the information means. The agent doesn't reason about sensors; it reasons about information that arrives pre-tagged with how urgently it matters.

## What this is and isn't

This is an application of established patterns — decoupled perception, push-based event delivery, declarative routing — to the LLM agent context. Subsumption architectures, BDI systems, and robotic middleware like ROS have had equivalent mechanisms for years. The ideas aren't new.

What's less common in the current LLM agent ecosystem is having a persistent agent reasoning loop (our OODA loop) that can meaningfully *consume* asynchronous input. Most LLM agent frameworks are fundamentally synchronous: run a task, produce a result, stop. They don't have a continuously running reasoning process that can be interrupted or informed by external events. Adding sensors to a system like that would mean adding the reasoning loop too, which is a much larger architectural commitment.

Cognitive Workbench already has the OODA loop, the goal scheduler, the infospace for persistent state, and a sense-data channel that multiplexes input from users, other agents, and now sensors. The sensor subsystem is a natural extension of infrastructure that already exists for other reasons.

The real test will be in agent-side consumption — how the OODA loop triages sensor input against ongoing work, and whether the disposition vocabulary is expressive enough for real scenarios. We've deliberately kept it minimal (three dispositions) and expect to expand it once we have concrete cases that break the current model. The sensor production side is specified and ready for implementation. The consumption side is the next design problem.

---

*Cognitive Workbench is an ongoing research project exploring depth-of-cognition in AI agents — planning, reflection, self-monitoring, and architectural patterns for genuine agent autonomy. These posts are working notes, not announcements.*
https://github.com/bdambrosio/Cognitive_workbench.git