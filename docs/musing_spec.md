Great — then the right move is to **freeze the design as a concrete, readable spec**, not just an outline. What follows is a **ready-to-paste `docs/musing_spec.md`**, written to be consumed by *Codex desktop* and future-you as an engineering artifact, not as narrative prose.

You can paste this verbatim into your repo.

---

# Musing Adapter Specification

**Status:** Experimental / Research
**Audience:** Internal (planner + model-integration developers)
**Primary goal:** Enable transient, decaying internal biasing (“musing”) of a frozen base LLM during an incremental planning episode, without modifying base model weights.

---

## 1. Motivation and scope

Modern LLM-based planners often need a cognitive mode that is neither:

* pure recall (retrieval of explicit facts), nor
* permanent learning (weight updates or fine-tuning).

We define **musing** as a *temporary, context-triggered internal modulation* of the base model’s reasoning dynamics, which:

* biases generation across multiple planner steps,
* decays to zero over time,
* leaves no persistent trace in base weights.

This document specifies an implementation of musing using:

* a frozen base LLM,
* a fixed LoRA-style adapter scaffold,
* a small learned bias policy **G** that emits transient adapter parameters,
* decay schedules synchronized with planner segment boundaries.

This spec deliberately avoids:

* modifying base model weights,
* per-token adapter mutation during KV-cached generation,
* full RLHF or preference training on the base model.

---

## 2. Core definitions

### 2.1 Base model

The **base LLM** is a frozen transformer used by the planner.

Initial target:

* `Qwen3-Coder-30B-A3B-Instruct`
* Inference modes:

  * FP16 (evaluation / sanity checks)
  * AWQ 4-bit (rollouts / training)

The base model is *never updated* by gradient descent in this system.

---

### 2.2 Planner episode

A **planner episode** is one execution of `tool_planner_infospace`, from:

* receiving a goal (benchmark question),
* through incremental structured generation and tool execution,
* to final exit with success or failure.

Episodes provide:

* a natural unit of reward,
* clean boundaries for training signals.

---

### 2.3 Segment

A **segment** is a contiguous generation interval in which:

* KV cache is valid and reused,
* effective model parameters (base + adapters) are fixed.

Segment boundaries typically occur:

* after tool calls,
* after tool errors,
* after explicit reflection steps,
* when KV cache is intentionally reset.

**Invariant:** Adapter weights must not change inside a segment.

---

### 2.4 Adapter scaffold

The **adapter scaffold** is a fixed set of LoRA-style hooks attached to selected modules of the base model.

Properties:

* Architecture is static.
* Hooks are present for the lifetime of the process.
* Adapter parameters are supplied at runtime.
* Adapter strength is controlled by a scalar multiplier `α`.

The scaffold defines *where* musing can influence computation; **G** defines *how*.

---

### 2.5 Musing window

A **musing window** is a span of one or more segments during which:

* adapter weights are nonzero,
* scale `α > 0`,
* decay is applied across segment boundaries.

When `α → 0`, the system returns to baseline reasoning.

---

## 3. Operational definition of musing

**Musing** is defined operationally as:

> A bounded interval during which retrieved context and planner state induce a low-rank modulation of the base model’s internal computation, shaping reasoning trajectories across multiple steps, without committing changes to long-term model parameters.

Key properties:

* **Transient:** effects decay to zero.
* **Contextual:** triggered by goal/preplan or execution errors.
* **Low-rank:** modulation is limited and structured.
* **Reversible:** turning off musing restores baseline behavior.

---

## 4. System architecture

### 4.1 Two-plane design

**Inference plane (rollouts):**

* Runs frozen base model + adapter scaffold.
* Executes planner logic and tool calls.
* Collects trajectories and rewards.
* Target runtime:

  * SGLang structured generation, or
  * minimal “tiny SGLang” PyTorch runtime if adapter dynamics require it.

**Learning plane (training G):**

* Trains bias policy **G** from episode-level feedback.
* Runs on separate GPU if available.
* Does *not* backpropagate through the base model.

---

### 4.2 Episode flow

1. Receive goal (benchmark question).
2. Produce initial preplan (strategy sketch).
3. Evaluate triggers → decide whether to start musing.
4. If triggered:

   * compute observation vector `x`,
   * run bias policy `G(x)`,
   * install adapter parameters,
   * initialize scale `α₀`.
5. Run planner loop:

   * generation → tool execution → error handling.
6. At each segment boundary:

   * update decay (`α ← decay(α)`),
   * optionally stop musing if `α < ε`.
7. On tool errors:

   * optionally trigger recovery musing (G2).
8. Episode ends → compute terminal reward → update G.

---

## 5. Adapter scaffold specification

### 5.1 Attachment sites

Default attachment points:

* Attention output projection (`W_o`)
* MLP up-projection (`W_up`)

Layer selection:

* Middle third of transformer blocks by default.
* Avoid lowest layers to prevent syntactic degradation.
* Configurable list of layer indices.

---

### 5.2 Adapter parameterization

For each attachment site:

```
ΔW = α · s · (B @ A)
```

Where:

* `A ∈ ℝ^{r × d_in}`
* `B ∈ ℝ^{d_out × r}`
* `r` is small (2–8 initially)
* `s` is a fixed per-site scaling factor
* `α` is the **musing scale**, dynamic over time

---

### 5.3 Runtime invariants

* Adapter weights **must remain fixed within a segment**.
* Only `α` may change during an episode.
* Changing adapter weights requires ending the current segment and rebuilding KV cache.

---

### 5.4 Magnitude control

To ensure stability:

* Clamp norms of `(A,B)` or `ΔW`.
* Cap `α` globally.
* Optionally normalize `ΔW` per site.

---

## 6. Bias policy G

### 6.1 Role of G

**G** is a small policy network that maps planner state → adapter parameters (or a latent controlling them).

G:

* does *not* emit tokens,
* does *not* store memories,
* only biases the base model’s computation.

---

### 6.2 Inputs to G (observation vector)

The observation vector `x` is a concatenation of:

**A. Goal embedding**

* embedding of the benchmark question.

**B. Preplan embedding**

* embedding of the initial strategy sketch.

**C. Planner state features**

* step index,
* number of tool errors so far,
* last tool name (categorical),
* last error type (categorical).

**D. Tool catalog encoding (optional)**

* embedding or hashed representation of available tools.

**E. Retrieved memory embeddings (optional / later)**

* pooled embeddings of relevant prior episodes or notes.

---

### 6.3 Outputs from G

Two supported output modes:

**Mode A: Latent control vector (recommended)**

* `z = G(x)` where `z ∈ ℝ^k`, `k ≈ 64–256`.
* A decoder `D(z)` maps `z → {(Aℓ,Bℓ)}` for each adapter site.

**Mode B: Direct adapter weights**

* `G(x)` outputs flattened `(A,B)` tensors.
* Higher variance; mainly for experiments.

---

### 6.4 Triggering modes

Initial modes:

* **G1 (initial musing):** triggered after goal + preplan.
* **G2 (recovery musing):** triggered after tool execution errors.

Each mode may:

* use different observation subsets,
* have different decay schedules,
* activate different adapter sites.

---

## 7. Temporal mechanics and decay

### 7.1 Segment boundaries

Segment boundaries occur:

* after each tool call,
* after tool errors,
* after explicit reflection steps,
* when KV cache is reset.

---

### 7.2 Decay schedules

Supported decay schedules:

* Exponential: `α ← γ α`
* Linear over fixed number of segments
* Error-accelerated decay (optional)

Stopping condition:

* `α < ε` → musing ends.

---

## 8. Training objectives

### 8.1 Rewards

**Terminal reward:**

* Correct answer → `+1`
* Incorrect answer → `0` or `-1` (configurable)

**Intermediate shaping:**

* Tool execution errors → negative reward
* Error weight may depend on error class.

---

### 8.2 Credit assignment

* Actions correspond to **musing invocations**, not tokens.
* Reward is assigned per episode or per musing window.
* Multiple invocations in one episode may share credit.

---

### 8.3 Optimization approach

Initial phases:

**Phase 0:** fixed or random G (no learning)
**Phase 1:** bandit / REINFORCE-style updates on G
**Phase 2:** PPO-style with value baseline (optional)

Training updates apply **only to G**, never to the base model.

---

## 9. Runtime integration constraints

* Adapter weights must only change at segment boundaries.
* SGLang KV cache reuse assumes fixed parameters.
* If adapter parameters change:

  * end segment,
  * rebuild KV cache.

If SGLang constraints are too restrictive:

* use minimal PyTorch runtime (“tiny SGLang”) for experiments.

---

## 10. Logging and instrumentation

Required per episode:

* musing triggers and timing,
* `α` over time,
* adapter norms,
* tool calls and errors,
* final reward.

Logs must be machine-readable (e.g. JSONL).

---

## 11. Open questions / future work

* Optimal observation representation for G.
* Whether tool catalog embedding is useful.
* How to allocate credit across multiple musing windows.
* Whether decoder `D` should be learned or fixed.
* Detecting planner stall / rumination automatically.

---

**End of specification.**

---

### How to use this with Codex

When you move to Codex desktop, anchor *everything* on this file. For example:

> “Using `docs/musing_spec.md`, implement the adapter scaffold hooks for Qwen3-Coder-30B, respecting the segment-boundary invariants.”

This spec is now the **shared ground truth**.
Chat is no longer the memory; the repo is.

If you want, next we can:

* generate a **README.md** that explains this to an unfamiliar collaborator,
* or define the **exact Python interfaces** between planner ↔ adapter ↔ G.
