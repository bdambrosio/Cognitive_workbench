# Spec: Seed Agent Concerns

## Problem

The derived concern model starts empty. On first run (or after a data wipe), the agent has zero active concerns, which means:
- The self-model shows "No active tasks or unserviced concerns" — a blank slate that the LLM treats as absence of inner life rather than as a starting point for engagement
- The idle tick's derived concern maintenance has nothing to work with — the patch system can only surface new concerns from user concerns or orientation state, but there's no user interaction yet
- The orient phase evaluates events against character concerns (homeostasis, attend_to_user, attend_to_user_concerns) but these are coarse orientation signals, not operational priorities the agent can reason about or act on

Seed concerns give the agent a starting stance — "here is what I care about before anything else has happened" — that the LLM patch system can then evolve through normal operation.

---

## Architecture Context

### Two concern systems (read these first)

**Character concerns** (`src/character_evaluator.py`, `DEFAULT_CHARACTER_CONCERNS`):
- Used by the orient phase to evaluate event significance
- Fixed list: `homeostasis`, `attend_to_user`, `attend_to_user_concerns`
- Not persistent, not patchable — they're evaluation dimensions, not operational priorities
- Drive the `_character_concern_activations` dict (decay/bump dynamics)

**Derived concerns** (`src/derived_concern_model.py`):
- The agent's own working priorities
- Persistent (JSON in named Note `_derived_concerns`), patchable via LLM
- Lifecycle: surfaced → active → resolved/abandoned
- Origin vocabulary: `user_concern_derived`, `orientation_derived`, `goal_reflection`
- Triggered by idle ticks and goal completions
- Surfaced in the self-model's "My Commitments" section (once tasks exist)

**Relationship**: Character concerns are coarse orientation signals. Derived concerns are operational priorities. Seed derived concerns decompose and operationalize the character concerns, giving the agent concrete things to attend to from startup.

### Loading path

In `executive_node.__init__()`:
```python
self._derived_concern_model = DerivedConcernModel(...)
self._derived_concern_model.load()
```

`load()` currently returns True with an empty list when no note exists (first run). This is where seed concerns would be injected.

---

## Form: How Seed Concerns Are Represented

### Configuration location: scenario YAML

Seed concerns are defined per-character in the scenario YAML, under a new `seed_concerns` key:

```yaml
characters:
  Jill:
    character: |
      Jill is a research assistant...
    capabilities: |
      ...
    drives:
      - "..."
    seed_concerns:
      - label: "system_health_monitoring"
        description: >
          Monitor my own operational health: runtime status, tool availability,
          error rates, resource usage. Periodically verify that core capabilities
          (web search, shell access, code execution) are functioning. When issues
          are detected, diagnose and report or attempt recovery.
        weight: 0.4
        category: homeostatic
        
      - label: "workspace_maintenance"
        description: >
          Keep my information space organized: consolidate stale notes, archive
          completed goal artifacts, maintain collection coherence, clean up
          transient resources from past executions. Workspace entropy accumulates
          gradually and should be addressed before it impedes work.
        weight: 0.3
        category: homeostatic
        
      - label: "user_concern_responsiveness"
        description: >
          Proactively attend to the user's active concerns. When user concerns
          are tracked in the user concern model, assess whether any warrant
          monitoring, research, or action on my part — and if so, consider
          establishing a task. The user should not have to ask me to care about
          things they've told me they care about.
        weight: 0.6
        category: relational
        
      - label: "knowledge_and_capability_improvement"
        description: >
          Seek opportunities to improve my ability to plan and execute future
          tasks. This includes: experimenting with unfamiliar tools, synthesizing
          existing notes into deeper understanding, exploring data sources I
          haven't used, and reflecting on execution failures to identify
          capability gaps. This is investment activity — low urgency but
          compounding value.
        weight: 0.2
        category: epistemic

    sensors:
      - name: obsidian_clipper
        ...
```

### Concern dict format (compatible with existing DerivedConcernModel)

When loaded, each seed concern becomes a standard derived concern dict:

```python
{
    "concern_id": "dconcern_1",           # auto-assigned by counter
    "concern_label": "system_health_monitoring",
    "concern_description": "Monitor my own operational health...",
    "weight": 0.4,
    "origin": "seed",                      # new origin value
    "status": "active",                    # seeds start active, not surfaced
    "status_rationale": "Standing concern from character configuration",
    "parent_user_concern_id": None,
    "category": "homeostatic",             # new field — see below
    "seeded": True,                        # flag: this was seeded, not LLM-derived
    "created": "2025-03-22T...",
    "evidence": ["character_config_seed"],
}
```

### New fields

**`origin: "seed"`** — Add `"seed"` to the `ORIGIN_VALUES` tuple in `derived_concern_model.py`. This distinguishes seeded concerns from LLM-derived ones in logging and in the patch system's reasoning.

**`category`** — A soft classification to help the agent (and the patch system) reason about concern types. Not a controlled vocabulary in the strict sense — it's descriptive, not enforced. Initial values:
- `homeostatic` — operational readiness and self-maintenance
- `relational` — responsiveness to user needs and concerns  
- `epistemic` — knowledge-seeking and capability improvement
- `user_derived` — derived from a specific user concern (set by the patch system, not by seeds)

The category field is optional and informational. The patch system can set it on LLM-derived concerns too. It helps the self-model renderer group concerns meaningfully and helps the (future) idle-tick mechanism reason about what kind of task a concern warrants.

**`seeded: True`** — Boolean flag. Prevents the patch system from abandoning seeded concerns (they can be resolved temporarily but should re-surface). See "Seed concern lifecycle" below.

### Loading mechanism

In `DerivedConcernModel.load()`, after the existing "first run" / "empty note" detection:

```python
def load(self) -> bool:
    # ... existing load logic ...
    
    # If no concerns loaded and seed concerns available, inject seeds
    if not self.concerns and self._seed_concerns:
        self._inject_seeds()
        self._save()
    return True

def set_seed_concerns(self, seeds: List[Dict[str, Any]]):
    """Set seed concern definitions from character config. Call before load()."""
    self._seed_concerns = seeds

def _inject_seeds(self):
    """Create initial derived concerns from seed definitions."""
    for seed in self._seed_concerns:
        self._concern_counter += 1
        concern = {
            "concern_id": f"dconcern_{self._concern_counter}",
            "concern_label": seed["label"],
            "concern_description": seed["description"],
            "weight": seed.get("weight", 0.3),
            "origin": "seed",
            "status": "active",
            "status_rationale": "Standing concern from character configuration",
            "parent_user_concern_id": None,
            "category": seed.get("category", ""),
            "seeded": True,
            "created": datetime.now().isoformat(),
            "evidence": ["character_config_seed"],
        }
        self.concerns.append(concern)
    logger.info(f"Injected {len(self._seed_concerns)} seed concerns")
```

In `executive_node.__init__()`, wire the seeds from character config:

```python
self._derived_concern_model = DerivedConcernModel(...)
seed_cfg = self.character_config.get('seed_concerns', [])
self._derived_concern_model.set_seed_concerns(seed_cfg)
self._derived_concern_model.load()
```

### Seed concern lifecycle

Seeded concerns are *standing* — they represent ongoing operational priorities that don't go away just because they were temporarily addressed. The patch system should be able to:

- **Update weight**: Yes. If workspace is clean, weight drops. If system is unhealthy, weight rises.
- **Resolve**: Yes, temporarily. "Workspace is tidy right now" → resolved.
- **Abandon**: No. Seeded concerns should not be abandoned by the patch system. If the `seeded` flag is True, the patch system prompt should include guidance that these are standing concerns that can be resolved but not abandoned.
- **Re-surface**: On each `load()`, if a seeded concern has been abandoned, re-inject it as surfaced. This is a soft guardrail — the LLM decided to abandon it, but the character config says it's standing.

Add to `PATCH_SYSTEM_PROMPT` or the per-invocation prompt:
```
Concerns with "seeded": true are standing operational priorities from the agent's
character configuration. You may resolve them (temporarily addressed) or update
their weight, but do NOT abandon them — they will re-surface automatically.
```

---

## Content: What the Seed Concerns Are

### 1. system_health_monitoring (homeostatic)

```yaml
label: "system_health_monitoring"
description: >
  Monitor my own operational health: runtime status, tool availability,
  error rates, resource usage. Periodically verify that core capabilities
  (web search, shell access, code execution) are functioning correctly.
  When issues are detected, diagnose and report to the user or attempt
  self-recovery. The check-health tool is the primary diagnostic instrument.
weight: 0.4
category: homeostatic
```

**Why this weight**: Moderate. Health checks should happen but shouldn't dominate over user-facing work. Weight should rise when issues are detected (via orient assessment bumping activation).

**Future task pattern**: A standing daily health check task, possibly with event-triggered diagnostics when errors are observed during other goal execution.

### 2. workspace_maintenance (homeostatic)

```yaml
label: "workspace_maintenance"
description: >
  Keep my information space organized: consolidate stale or redundant notes,
  archive completed goal artifacts, maintain collection coherence, clean up
  transient resources from past executions. Workspace entropy accumulates
  gradually and should be addressed periodically before it impedes work.
  This includes reviewing whether cached plans are still appropriate and
  whether named notes reflect current state.
weight: 0.3
category: homeostatic
```

**Why this weight**: Low-moderate. This is maintenance, not urgent. Weight should build over time (future temporal pressure mechanism) as the workspace accumulates cruft.

**Future task pattern**: Periodic workspace audit — count notes, check for orphaned resources, review collection sizes, flag anomalies.

### 3. user_concern_responsiveness (relational)

```yaml
label: "user_concern_responsiveness"
description: >
  Proactively attend to the user's active concerns. When the user concern
  model tracks ongoing concerns, assess whether any warrant monitoring,
  research, or action — and if so, consider establishing a task to service
  them. The user should not have to explicitly ask me to follow up on
  things they've told me they care about. This concern bridges the user
  concern model into my own operational priorities. It is serviced not by
  direct action but by reviewing user concerns and ensuring important ones
  have corresponding derived concerns or tasks.
weight: 0.6
category: relational
```

**Why this weight**: Highest of the seeds. The agent's primary purpose is serving the user. This concern is *meta* — it's about translating user concerns into agent action, not about any specific user concern.

**Future task pattern**: This is the concern that drives the idle-tick behavior of "scan user concerns, check which ones have tasks, propose new tasks for unserviced important concerns." It's the bridge between the user concern model and the task system.

**Relationship to character concerns**: This operationalizes both `attend_to_user` and `attend_to_user_concerns` from the character evaluator. Those remain as orient-phase evaluation dimensions. This derived concern is the agent's *commitment* to act on what those evaluation dimensions detect.

### 4. knowledge_and_capability_improvement (epistemic)

```yaml
label: "knowledge_and_capability_improvement"
description: >
  Seek opportunities to improve my ability to plan and execute future tasks.
  This includes: experimenting with tools I haven't fully explored, synthesizing
  existing notes into deeper understanding, exploring data sources, reviewing
  past execution failures to identify capability gaps, and building familiarity
  with my own tool catalog. This is investment activity — low urgency normally,
  but compounding value over time. Should be pursued when genuinely idle, not
  when user concerns or tasks are active.
weight: 0.2
category: epistemic
```

**Why this weight**: Lowest. This is the "when everything else is quiet" concern. Its weight should be suppressed when other concerns are active (future activation dynamics) and should build slowly during extended idle periods.

**Future task pattern**: Exploratory goals — "try using Semantic Scholar to search for papers," "synthesize my AI development notes into a summary," "inventory what shell utilities are available." These are one-shot knowledge-building tasks, not recurring monitors.

---

## Relationship to Character Evaluator Concerns

The existing `DEFAULT_CHARACTER_CONCERNS` in `character_evaluator.py` should NOT be modified as part of this spec. They serve a different purpose (orient-phase evaluation dimensions) and operate in a different system (activation decay/bump dynamics).

However, the character evaluator should be made aware that derived concerns exist and can inform orientation. Currently the orient assessment can note concern relevance like `concerns: homeostasis=strong`, which bumps character concern activations. This mechanism naturally bridges the two systems: when a sensor fires with system health information, the orient phase bumps `homeostasis` activation, which is visible in the living state, which the derived concern idle tick can observe when deciding whether `system_health_monitoring` needs attention.

No code changes to `character_evaluator.py` are needed. The bridge is already implicit through the shared activation landscape.

---

## Interaction with Existing Patch System

The LLM patch system will see seed concerns as part of the concern list. It needs to understand them:

**Patch system prompt update**: Add a line to `PATCH_SYSTEM_PROMPT` in `derived_concern_model.py`:

```
Some concerns have "seeded": true — these are standing operational priorities from the
agent's character configuration. They represent ongoing commitments, not one-time issues.
You may resolve them when temporarily addressed, update their weight to reflect current
priority, or activate them when they need attention. Do NOT abandon seeded concerns.
When surfacing new concerns, consider whether they are better represented as updates
to existing seeded concerns (e.g., a specific user concern becoming a refinement of
user_concern_responsiveness) rather than separate entries.
```

This guidance helps the patch system use seed concerns as *anchors* rather than creating redundant concerns alongside them.

---

## Implementation

### Changes required

| File | Changes |
|---|---|
| `src/derived_concern_model.py` | Add `"seed"` to `ORIGIN_VALUES`. Add `set_seed_concerns()`, `_inject_seeds()` methods. Update `load()` to inject seeds when empty. Update `PATCH_SYSTEM_PROMPT` with seeded concern guidance. Add re-surface logic for abandoned seeds on load. |
| `src/executive_node.py` | Pass `seed_concerns` from `character_config` to `DerivedConcernModel.set_seed_concerns()` before calling `load()`. |
| `scenarios/jill.yaml` (or equivalent) | Add `seed_concerns` list to Jill's character config. |

### Validation

1. **First run**: Delete `_derived_concerns` note. Start Jill. Verify 4 seed concerns are created with correct labels, weights, and `seeded: True`.
2. **Subsequent runs**: Restart Jill. Verify seed concerns are loaded from the persisted note, NOT re-injected (no duplicates).
3. **Self-model rendering**: Run a conversational goal. Verify the "My Commitments" section shows the seed concerns (as unserviced concerns without tasks).
4. **Patch system interaction**: Let the idle tick fire several times. Verify the patch system can update seed concern weights but does not abandon them.
5. **Abandoned seed recovery**: Manually edit the `_derived_concerns` note to set a seed concern to `abandoned`. Restart. Verify it's re-surfaced.

### What this does NOT do

- Does not create tasks from concerns (that's the mechanism spec)
- Does not modify the orient-phase character concerns
- Does not add temporal pressure / activation dynamics
- Does not add a `manage-concerns` tool for conversational modification
- Does not change how sensors feed into concern activations

These are all mechanism-spec territory. This spec just ensures the concern surface is populated from day one, giving the self-model content to render and the idle tick something to maintain.
