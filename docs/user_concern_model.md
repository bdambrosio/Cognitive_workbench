# user_concern_model.md

## Purpose

Maintain a compact, incrementally updated model of the user's active and recent concerns, based on completed conversations and completed goals.

This module is intended to support salience evaluation by providing one source of user-centered prior significance. It does **not** attempt full user modeling, personality inference, or broad autobiographical memory.

Its role is narrower:

- track what the user has been actively concerned with
- track the user's expressed stance toward those concerns
- track the user's apparent disposition by the end of a conversation or goal
- track whether a concern appears closed, ongoing, or dormant

This model becomes one input to future salience evaluation:

- does this matter to the user?
- does this connect to an ongoing concern?
- does this reopen a previously closed issue?
- does this align or conflict with the user's current stance?

---

## Design Principle for V1

The model should be updated **incrementally**, not regenerated wholesale on every pass.

A stable concern model should usually change locally in response to a new completed conversation or completed goal. Full re-assessment of the entire concern list is inefficient, likely to drift, and unnecessary in V1.

So this module should usually emit a **single patch operation** against an existing concern list:

- add a new concern
- update an existing concern
- close an existing concern
- or emit no change

Pruning and decay of older concerns should be handled separately by lightweight aging logic.

---

## Trigger Conditions

This module is invoked on:

1. **Conversation end**
   - input: full conversation or summarized conversation transcript
   - intended effect: update concern state based on what the user discussed, asked, worried about, decided, or left unresolved

2. **Goal completion**
   - input: goal statement + outcome summary
   - intended effect: update concern state based on whether the goal resolved, advanced, complicated, or reopened a concern

This module is **not** called on every message or every agent cycle in the initial version.

---

## High-Level Function

Given a completed interaction unit (conversation-close or goal-completion), the module should:

1. identify the **single most salient user concern implication** from the interaction
2. determine whether that implication corresponds to:
   - a new concern
   - an update to an existing concern
   - closure of an existing concern
   - no meaningful change
3. emit a single patch operation
4. allow separate aging/decay logic to handle untouched concerns over time

---

## Core Concept

This module should model **concerns**, not merely topics.

A topic is too thin:
- `solar`
- `agents`
- `portfolio`

A concern is more useful:
- `concern about inverter startup sag`
- `ongoing exploration of persistent orientation for Jill`
- `desire for lightweight portfolio monitoring`

Concerns are more directly useful for salience evaluation.

---

## Data Model

The maintained model is a list of `ConcernEntry` objects.

### ConcernEntry

Each entry should contain:

- `concern_id`
  - stable internal identifier

- `concern_label`
  - short human-readable label
  - examples:
    - `persistent orientation for Jill`
    - `off-grid inverter reliability`
    - `portfolio monitoring automation`

- `concern_description`
  - 1-3 sentence summary of what this concern means in the user's context

- `weight`
  - floating-point score representing current estimated importance / activation
  - should reflect both recurrence and recency
  - not merely frequency of mention

- `recency`
  - timestamp of last meaningful update

- `touch_count`
  - number of completed interaction units that materially affected this concern

- `stance`
  - structured estimate of the user's current expressed stance toward the concern
  - examples:
    - `exploratory`
    - `skeptical`
    - `committed`
    - `concerned`
    - `frustrated`
    - `enthusiastic`
    - `ambivalent`

- `stance_rationale`
  - short text explaining why the stance was assigned

- `end_disposition`
  - estimated user state by the end of the interaction relative to this concern
  - examples:
    - `resolved`
    - `clearer`
    - `still_uncertain`
    - `still_concerned`
    - `more_interested`
    - `blocked`
    - `deferred`
    - `satisfied`
    - `dissatisfied`

- `end_disposition_rationale`
  - short text explaining the estimated end-state

- `status`
  - enum:
    - `ongoing`
    - `closed`
    - `dormant`
    - `reopened`

- `status_rationale`
  - short text explaining the status judgment

- `evidence_refs`
  - references to conversation-close or goal-completion events that contributed to this concern

- `history_summary`
  - short cumulative note of major updates over time
  - optional in V1, but useful if cheap

---

## Inputs

### 1. ConversationCloseInput

Fields:

- `conversation_id`
- `conversation_summary`
- `major_user_questions`
- `major_topics`
- `notable_user_stances`
- `conversation_resolution_summary`
- `timestamp`

### 2. GoalCompletionInput

Fields:

- `goal_id`
- `goal_statement`
- `goal_outcome_summary`
- `goal_success_status`
- `user_visible_impact`
- `timestamp`

If no summary exists yet, an upstream summarization step may first produce a compact interaction summary.

---

## Output

The module outputs a **single patch operation** plus brief rationale.

### Patch Types

#### 1. `add_concern`

Create a new concern entry.

Use when:
- the interaction surfaces a materially new user concern
- no existing concern is a strong enough match

Fields:

- `op`: `add_concern`
- `new_concern`: full `ConcernEntry`
- `why_this_add`: short rationale

#### 2. `update_concern`

Update one existing concern.

Use when:
- the interaction materially advances, clarifies, intensifies, or shifts one known concern

Fields:

- `op`: `update_concern`
- `concern_id`
- `field_updates`
  - one or more updated fields, such as:
    - `weight`
    - `recency`
    - `touch_count`
    - `stance`
    - `end_disposition`
    - `status`
    - `evidence_refs`
    - `history_summary`
- `why_this_update`: short rationale

#### 3. `close_concern`

Mark one existing concern as closed.

Use when:
- the interaction strongly suggests that a previously ongoing concern has been resolved or no longer requires active tracking

Fields:

- `op`: `close_concern`
- `concern_id`
- `closing_updates`
  - typically:
    - `status = closed`
    - updated `end_disposition`
    - updated `weight`
    - updated `recency`
    - updated `evidence_refs`
- `why_this_close`: short rationale

#### 4. `no_change`

Emit when:
- no concern change seems material enough to patch the model

Fields:

- `op`: `no_change`
- `why_no_change`

---

## Patch Emission Rule for V1

For each invocation, emit **at most one primary patch**.

This is a deliberate simplification for stability and efficiency.

Rationale:
- keeps updates sparse and inspectable
- avoids model churn
- forces prioritization of what mattered most
- is sufficient for a first working version

Later versions may allow up to 2-3 patches per invocation.

---

## Update Logic

### A. Identify the main concern implication

From the completed conversation or goal, identify the single strongest concern-level implication.

Ask:
- what seems to have mattered most to the user here?
- was an existing concern advanced, intensified, resolved, or reopened?
- did a genuinely new concern emerge?

Do **not** emit patches for incidental topics or passing mentions.

### B. Match against existing concern list

Attempt to match the current interaction to one existing concern.

Matching should consider:
- semantic similarity
- continuity of the underlying issue/project/question
- user intent and concern shape
- not just lexical overlap

If match confidence is high, prefer updating an existing concern.
If not, add a new concern.

### C. Local update, not wholesale rewrite

Do **not** recompute the whole concern list.
Do **not** rewrite untouched concerns.
Do **not** regenerate historical stances from scratch.

The default behavior should be one local patch against one concern.

---

## Weight Logic

Weight should increase when:
- the concern recurs
- the concern remains unresolved
- the concern appears practically or emotionally consequential
- the concern generates follow-up questions or new goals

Weight may decrease when:
- the concern is closed
- the concern has gone untouched for a while
- the concern seems to be losing relevance

In V1, concern aging should mostly happen outside this module via separate decay logic.

This module only performs local weight adjustments for the touched concern.

---

## Status Logic

Allowed values:

- `ongoing`
- `closed`
- `dormant`
- `reopened`

Guidelines:

- `ongoing`: active concern still alive
- `closed`: concern appears resolved
- `dormant`: older concern not recently touched, but not explicitly resolved
- `reopened`: a previously quiet or closed concern has become active again

In V1, this module should directly set:
- `ongoing`
- `closed`
- `reopened`

A separate pruning/aging pass may later set untouched concerns to `dormant`.

---

## Aging and Pruning

Aging should be handled by separate lightweight infrastructure, not by full regeneration.

### Aging behavior

Untouched concerns should gradually:
- lose weight
- eventually shift from `ongoing` to `dormant`
- eventually be pruned if very old and low-weight

### Pruning strategy for V1

Simple initial strategy:
- if a concern has low weight and has not been touched for a long time, remove it from the active list

Optional:
- archive pruned concerns elsewhere if desired

This module should assume that aging/pruning occurs externally.

---

## Constraints

The module should **not**:

- infer broad personality traits
- infer deep motives unless explicitly grounded
- generate many new concerns per pass
- regenerate the full concern list every invocation
- equate frequency with importance
- overwrite concern history too aggressively
- store vague topical fragments with no salience value

Keep the concern model:
- sparse
- local
- interpretable
- stable across updates

---

## Salience Integration

This module provides one source of user-centered significance for salience evaluation.

Examples:

- this new event may matter because it touches an ongoing user concern
- this goal completion may reduce salience by moving a concern toward closure
- this new conversation may reopen a previously closed issue
- this sensor event may matter because it connects to a repeatedly active concern

The salience evaluator should use this module as one input among others, such as:
- homeostasis / system health
- recent exchange context
- active goals
- RSS or other sensors

---

## Initial Controlled Vocabularies

### stance
- exploratory
- skeptical
- committed
- concerned
- frustrated
- enthusiastic
- ambivalent

### end_disposition
- resolved
- clearer
- still_uncertain
- still_concerned
- more_interested
- blocked
- deferred
- satisfied
- dissatisfied

### status
- ongoing
- closed
- dormant
- reopened

---

## Initial Heuristics

Use simple, inspectable heuristics in V1:

- emit at most one patch
- prefer updating an existing concern over creating a new one
- create a new concern only when clearly warranted
- close a concern only when closure is reasonably explicit
- keep rationale short and concrete
- avoid speculative interpretation

---

## Example 1: update existing concern

### Input
Conversation close about Jill memory/orientation architecture.
User ends more focused, still actively engaged.

### Output
- `op`: `update_concern`
- `concern_id`: `concern_017`
- `field_updates`:
  - `weight`: increase
  - `recency`: now
  - `touch_count`: +1
  - `stance`: `committed`
  - `end_disposition`: `clearer`
  - `status`: `ongoing`
- `why_this_update`: `The conversation materially advanced an existing ongoing concern about Jill's persistent orientation.`

---

## Example 2: add new concern

### Input
Goal completion reveals a new recurring user desire for portfolio monitoring automation.

### Output
- `op`: `add_concern`
- `new_concern`:
  - `concern_label`: `portfolio monitoring automation`
  - `concern_description`: `User wants lightweight ongoing monitoring of portfolio holdings and related developments.`
  - `weight`: initial moderate value
  - `recency`: now
  - `touch_count`: 1
  - `stance`: `interested`
  - `stance_rationale`: `User expressed desire for this capability as a useful recurring function.`
  - `end_disposition`: `more_interested`
  - `end_disposition_rationale`: `The interaction surfaced this as a live area of interest rather than resolving it.`
  - `status`: `ongoing`
  - `status_rationale`: `This appears to be a continuing concern rather than a closed issue.`
  - `evidence_refs`: [...]
- `why_this_add`: `This interaction surfaced a distinct new ongoing concern not well matched by current entries.`

---

## Example 3: no change

### Input
Conversation touches an existing concern only incidentally, without materially shifting it.

### Output
- `op`: `no_change`
- `why_no_change`: `No concern-level update seemed strong enough to justify modifying the current model.`

---

## Open Questions

- Should `history_summary` be included in V1 or deferred?
- Should `touch_count` increment only on material updates or every mention?
- How aggressive should external decay be?
- Should pruned concerns be archived for later reactivation?

---

## Recommendation for V1

Implement the smallest useful version:

- concern list as persistent state
- one invocation on conversation end or goal completion
- at most one patch emitted
- patch types:
  - `add_concern`
  - `update_concern`
  - `close_concern`
  - `no_change`
- separate external aging/pruning
- concise rationale for every patch

Do not attempt full regeneration of the concern model in V1.