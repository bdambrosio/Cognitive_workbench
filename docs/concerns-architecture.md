# Concerns: How an Autonomous Agent Decides What Matters

## The Problem

An autonomous agent sitting idle has no reason to act. Events arrive — user messages, sensor alerts, scheduled triggers — and the agent responds. But between events, it's inert. Worse, even when events do arrive, the agent has no persistent model of what's important. Each interaction starts from scratch, with no memory of what the user cares about or what operational priorities the agent should maintain.

This is the problem concerns solve. They give the agent a continuous, multi-layered model of what matters — from the user's ongoing interests to the agent's own operational health — and connect that model to autonomous action through a cognitive control loop that runs even when nothing is happening.

## Three Layers of Concern

The concern system operates at three levels, each tracking a different kind of "what matters" and updating at a different cadence.

### Character Concerns: The Evaluation Lens

At the base are three fixed character concerns that act as evaluation dimensions during orientation:

- **homeostasis** — the agent's operational health, stability, and resource usage
- **attend_to_user** — responsiveness to the user's communicative intent
- **attend_to_user_concerns** — sensitivity to the user's persistent, ongoing interests

These aren't things the agent "works on." They're the lens through which it evaluates every incoming event. When a user message arrives, the orientation step assesses it against all three dimensions: does this affect system health? does the user expect a response? does this touch something the user has been persistently interested in?

Each assessment produces a strength signal — none, weak, moderate, or strong — that feeds into a running activation level for each concern.

### User Concerns: What the User Cares About

The second layer tracks the user's actual interests. These are not concerns the agent chooses — they're concerns the agent *observes* in the user's behavior and maintains as a persistent model.

Each user concern records:

- What it's about and a brief description
- The user's stance (exploratory, committed, frustrated, etc.)
- Current status (open or closed)
- A weight reflecting perceived importance
- How many interactions have touched it
- Evidence references linking back to specific conversations

User concerns are updated incrementally after conversations and goal completions. A single LLM call examines the interaction against the existing concern list and emits one patch: add a new concern, update an existing one, close one, or make no change. This conservative, one-patch-at-a-time design prevents the model from drifting wildly on any single interaction.

The result is a compact model of the user's ongoing interests that persists across sessions. When the agent is deciding whether an event matters, it can consult this model — not just react to the literal content of the current message.

### Derived Concerns: The Agent's Working Priorities

The third layer is where the agent develops its own operational priorities. Derived concerns are things the agent chooses to care about, based on what it observes in user concerns and its own orientation state.

They have a richer lifecycle than user concerns:

```
surfaced → active → satisfied → [revisit period] → active → ...
                  → abandoned
```

A derived concern starts as **surfaced** — identified but not yet committed to. When the agent decides to act on it, it becomes **active**. When adequately addressed, it moves to **satisfied** — but this is not terminal. After a configurable revisit period (typically 4 hours for operational concerns, 24 hours for others), a satisfied concern automatically returns to active status.

This revisit mechanism is key. It means the agent's system health monitoring concern doesn't get permanently closed by one successful health check. It comes back, ensuring ongoing vigilance without requiring the user to re-request it.

Derived concerns can be seeded from the agent's character configuration — standing operational priorities like system health monitoring, workspace maintenance, knowledge improvement, and user responsiveness. These provide the agent with reasons to act from the moment it starts, before any user interaction has occurred.

```
User Concerns          Derived Concerns
(what user cares       (what agent decides
 about)                 to work on)
     │                       │
     └──── derives from ─────┘
                             │
                    ┌────────┴────────┐
                    │   active        │
                    │   surfaced      │
                    │   satisfied ──→ revisit → active
                    │   abandoned     │
                    └─────────────────┘
```

## The OODA Loop and Idle Cycle

The concern system is embedded in a continuous OODA (Observe-Orient-Decide-Act) cognitive control loop that runs on a 0.2-second tick.

### The Active Cycle

When events are present:

**Observe** dequeues one event from priority-ordered queues — alerts first, then sensor triggers, then user text input.

**Orient** runs a single LLM evaluation of the event against all three character concerns (plus any active derived concerns and user concerns). The output includes strength ratings per concern and an action recommendation. Post-evaluation, concern activations update via exponential decay plus bump:

```
activation = old_activation * 0.9 + bump

where bump = 0.30 (strong), 0.15 (moderate), 0.05 (weak), 0.0 (none)
```

This means activations build over sustained relevant events but fade when not reinforced — a natural attention mechanism.

**Decide** is pure routing with no LLM call. It maps the event classification and orient assessment to an action type: respond to user, dispatch a goal, proceed an existing goal, or take no action.

**Act** executes the decision, typically by launching a goal on a background thread that runs through the planner.

### The Idle Cycle

When no events are observed, the OODA loop enters an idle tick. This is where concerns drive autonomous behavior.

During idle ticks:

1. **Derived concern maintenance** — the derived concern model re-evaluates its concerns against the current orientation state. Concerns may be surfaced, activated, satisfied, or abandoned based on what the agent observes in user concerns and its own operational state.

2. **Revisit expiration** — satisfied concerns whose revisit period has elapsed are reactivated, ensuring periodic attention to ongoing priorities.

3. **Concern triage** — the bridge between concerns and action.

```
┌─────────────────────────────────────────────┐
│              OODA Loop (0.2s tick)           │
│                                             │
│  Event present?                             │
│   YES → Observe → Orient → Decide → Act    │
│                     │                       │
│                     └─→ activation updates  │
│                     └─→ orient nominations  │
│                                             │
│   NO  → Idle Tick                           │
│           ├─ derived concern maintenance    │
│           ├─ revisit expiration check       │
│           ├─ activation-monitor nominations │
│           └─ triage (if candidates + cooldown elapsed)
│                 └─→ task proposals          │
└─────────────────────────────────────────────┘
```

## From Concerns to Autonomous Action

Concern triage is the mechanism that converts persistent attention into concrete work. It operates through two nomination paths, each capturing a different kind of signal.

### Activation Monitor

During idle ticks, the system checks concern activations in the living state. If a concern's activation exceeds 0.55 and its trend is rising, it's nominated for triage. This captures sustained, building attention — something the agent has been noticing repeatedly across multiple events.

### Orient Bypass

During orientation, if an event produces a strong bump on a concern whose activation is still low (below 0.35), the concern is nominated immediately. This captures novel, high-impact events — something contextually important that hasn't had time to build activation through repeated exposure.

### Triage

When candidates accumulate and a 120-second cooldown has elapsed, an LLM triage call evaluates up to 6 candidates against existing tasks and agent context. For each candidate, triage decides:

- **create_task** — this concern needs new work; generates a task intention
- **attach_to_task** — an existing task already addresses this concern
- **defer** — not actionable now; suppress for a period and re-evaluate later
- **dismiss** — false alarm; suppress for a longer period

Task creation proposals flow into the task system, where they begin an establishment process: specification, capability evaluation, and incremental execution through the same planner that handles user-initiated goals.

### The Single-Thread Challenge

The current implementation runs the OODA loop, concern triage, and goal execution on a single-threaded main loop (with goals executing on background threads that the loop polls). This creates a practical tension: when the agent autonomously launches a task from concern triage, it occupies the same execution pathway that handles user requests. If a user message arrives during autonomous execution, the agent must interrupt its self-directed work to respond.

This is an active area of development — finding the right balance between autonomous initiative and user responsiveness within the constraints of a single cognitive control loop. The concern system itself is designed to handle this gracefully: the attend_to_user concern naturally gains activation when user messages arrive, and the triage system can defer autonomous work when user engagement is high.

## Why This Architecture

Several design choices are worth noting:

**Non-terminal satisfaction.** Most goal-tracking systems treat completion as final. The revisit mechanism treats satisfaction as temporary, which matches how real operational concerns work — system health doesn't stop mattering because you checked it once.

**Three-layer separation.** Character concerns (evaluation lens), user concerns (observed interests), and derived concerns (agent priorities) serve different functions at different update cadences. Character concerns are fixed and fast. User concerns update after interactions. Derived concerns evolve during idle time. This separation prevents the system from conflating "what the user said" with "what matters operationally."

**Conservative patching.** Both user and derived concern models update via single-patch LLM calls — one change per invocation. This prevents cascading updates where a single event rewrites the entire concern landscape.

**Activation dynamics.** The exponential decay plus bump model means the agent's attention naturally follows sustained signals. A single strong event produces a spike that fades. Repeated moderate events build activation that persists. This mirrors how human attention works — a single alarm is noticed then forgotten, but a recurring pattern demands action.

**Triage as bridge.** The separation between concern activation and task creation via an explicit triage step prevents the agent from compulsively acting on every activated concern. The triage LLM can consider context, existing work, and timing before committing to action.

The concern system gives the agent something most autonomous architectures lack: persistent, self-maintaining reasons to act that are grounded in what the user actually cares about, filtered through the agent's own operational judgment, and connected to concrete action through a principled cognitive control loop.
