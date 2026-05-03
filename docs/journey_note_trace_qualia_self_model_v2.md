# Journey Note: Trace, Qualia, Self-Model

*A note to self, recording where the thinking has gotten to as of late April 2026. Not for publication. Audit-grade record rather than synthesis. Companion to the substack draft, which only covers parts of step 1-2 below.*

---

## Purpose of this document

To identify clearly what has been demonstrated, what has been conjectured, what remains undone, and where the relevant literature pushes back. The previous version of this document overstated coherence and inevitability. This version tries not to.

The honest division: steps 1 and 2 below are supported by working code and observed behavior. Step 3 is descriptive after-the-fact, not predictive. Steps 4, 5, and 6 are conjectures of varying strength, with literature objections that have not been engaged.

## 1. Trace access lets agents report what they did

**Status: Demonstrated.**

Adding a per-iteration trace (thought field plus action, captured at execution time, surfaced to the agent on subsequent turns) lets Jill answer "what were you just thinking about" by referring to actual prior reasoning rather than confabulating.

What is actually demonstrated:
- Trace-grounded responses are different in kind from confabulated ones (verifiable by ground-truth comparison against the trace record).
- Specific moves like "I noticed but didn't flag" become reportable when there's a structural record of the noticing.

What is *not* demonstrated:
- That this generalizes beyond Sonnet-4.6. Gemma and Granite tests showed degraded behavior, but the degradation involved both substrate-level (trace reading) and persona-level (introspective discipline) failures, and these were not separated by ablation.
- That the trace mechanism alone is what produces the behavior. Strong instruct-trained models produce introspective-sounding output even without architectural support, and we did not run the controlled ablation (same prompt, trace stripped) that would isolate the architectural contribution.

**Loose end:** the introspective fidelity benchmark we drafted is intended to address the second point but has not been executed.

## 2. The qualia framing motivates the trace mechanism

**Status: Working frame, not validated.**

The conceptual chain (qualia as unexplained inputs; traces as historical-trace-reading; sufficient for the facsimile of awareness) motivated the engineering and is consistent with what the engineering produces.

What this is: a frame that organizes the thinking and motivated the implementation choices. It is useful internally as a generative metaphor.

What this is *not*: an empirical theory that has been tested against alternatives. Other framings would also motivate trace access — for example, a more straightforwardly computational framing where "the trace is structured introspective context" makes no claim about qualia. The qualia framing was chosen because it suggested the historical-not-meta angle, which was generative for design. Whether the framing is *correct* is a different question we have not addressed.

**Loose end:** there is no evaluation that distinguishes the qualia framing from neutral computational framings. The frame might be doing rhetorical work without paying its way analytically.

## 3. The qualia framing is consistent with observed failure modes

**Status: Descriptive after-the-fact, not predictive.**

The "you put it there" line in the benchmark transcript was a confabulation: Jill attributed user-installed provenance to an inference-derived concern. The qualia framing maps this cleanly onto "concerns appear in Jill's input as artifacts whose provenance she cannot see, so she fills it in with the most plausible source."

What I previously claimed: the framing *predicted* this failure.

What is actually true: we noticed the failure first, then mapped it onto the framework. The framework is consistent with the failure. Calling that "prediction" overstates what happened. A prediction would specify in advance where future failures will appear, and we have not made or tested such predictions.

**Loose end:** if the framework is genuinely predictive, it should be able to specify *other* places in the architecture where similar confabulation will appear, and those predictions should be testable. We have not done this work. Until we do, "the framework predicts the failure modes" is not a claim we can make.

## 4. Claim: the small-q qualia boundary is structurally ineliminable

**Status: Defensible operational claim, narrower than I previously framed it.**

### Terminology note

This document uses *small-q qualia* throughout to mean the deflationary operational category: inputs to a computational step whose origin that step cannot inspect. This is a category from architecture, not from philosophy of mind. It is *not* the capital-Q phenomenal Qualia of the philosophical literature, and the document makes no claim about phenomenal Qualia. (Bruce holds a separate position on phenomenal Qualia developed independently of this work; that position is not being argued for here, and the engineering does not depend on it. Conflating the two is a mistake the previous revision of this document made.)

### The claim

Any provenance record an architecture provides is itself an artifact of the computational process that produced it. If you decide to expose the genesis of one layer (e.g., why a concern was instantiated), the genesis-recording mechanism becomes the new layer with its own ungrounded origin from the inspecting step's vantage. Adding architectural layers relocates the small-q qualia boundary; it does not eliminate it.

Therefore: any computational architecture with internal state and finite inspection has *some* boundary at which the current step receives inputs whose origin it cannot account for. Where the boundary sits is an architectural choice. That there is a boundary is not.

### What this claim is

- A statement about the structure of computational architectures with internal state.
- A methodological commitment: we draw the line at where provenance becomes architecturally inaccessible, name what is on this side, and decline to commit on what is beyond.
- A prediction-shaped claim: confabulation will appear at whatever boundary the architecture currently terminates at, because that is where the current step is filling in unexplained inputs with plausible source-attributions.

### What this claim is not

- It is not a claim about the existence or nature of phenomenal consciousness.
- It is not a homunculus claim. We do not posit an inspector beyond the boundary. We decline to commit on what is beyond it.
- It is not foundationalist epistemology. Calling small-q qualia "primary" within a system's self-knowledge means only that the system cannot get behind them from the inside. It does not mean qualia are epistemically basic in any external sense.
- It is not a claim that small-q qualia are the same kind of thing as capital-Q phenomenal Qualia. The shared word is a deliberate analogy by the deflationary definition, not an identity claim.

### Where literature objections actually land

When the claim is stated this narrowly, several of the objections I raised in the previous revision either do not apply or apply much more weakly than I implied:

- The *homunculus objection* does not land. The homunculus error is positing an inspector beyond the boundary; we are positing nothing beyond the boundary.
- The *eliminativist objection* lands only weakly. "Boundary" can be replaced with "the level at which provenance terminates in this architecture," which is purely operational and does not import folk-psychological vocabulary. There is some residual concern about the spatial metaphor, but not a serious one.
- *Higher-order theory* is mostly orthogonal. The claim is not about kinds of meta-cognition; it is about where provenance terminates in any given architecture.
- *Predictive processing* offers an alternative framing (residual prediction error rather than unexplained input), and the two framings are probably not in direct conflict — they may be different vocabularies for overlapping phenomena. Worth engaging if the work moves toward that literature, not blocking otherwise.

### What was wrong in the previous revision

The previous revision overcorrected. It framed the claim as a strong regress argument with serious literature objections, when the actual claim is narrower and most of the objections do not land on it. That overcorrection was its own form of drift — performing skepticism about a position I had not characterized accurately.

The honest framing: this is a defensible operational claim about computational architectures, not an embattled philosophical thesis. Public statement of it does not require months of literature engagement. It requires accurate framing.

### Loose ends that remain

- The predictive purchase of the claim is still untested. If the framework is genuinely predictive, it should specify *other* places in the architecture where similar confabulation will appear, and those predictions should be testable. The within-forward-pass observation in the prioritizing-todos analysis (below) is one such prediction, not yet validated.
- The relationship between small-q qualia (the architectural category) and capital-Q Qualia (the philosophical category Bruce holds a separate position on) deserves clarification if the work ever moves toward integrating them. It is *not* required for the engineering, and conflating them is a hazard worth guarding against.
- The architectural-choice framing — "where the boundary sits is an architectural choice" — is plausible but worth examining. Some boundaries may be more principled than others. Not all stopping points are equally defensible.

## 5. Conjecture: a self-model would satisfy the functional definition more fully

**Status: Architectural conjecture, not built. Definition is partly self-confirming.**

The proposal: extend the architecture with a self-model containing configured persona, in-flight self-commitments, and current focus. The functional definition I authored ("capacity of a system to maintain and update an accessible model of its own internal states in a way that influences subsequent behavior") would be satisfied more fully.

The circularity I should have flagged: the functional definition was authored by me, with the trace mechanism in mind, and gets satisfied more fully by adding more structure of the kind the definition was designed to capture. This is not independent confirmation. The definition is doing some self-justifying work.

What would actually validate the self-model proposal:
- Implementing it.
- Showing it produces behavior the trace-only architecture does not produce.
- Showing the new behavior is useful (for the agent, for the user, or both) rather than just architecturally elegant.
- Comparing against alternative architectures that achieve similar behavior through different mechanisms (e.g., richer prompt scaffolding, retrieval-augmented generation over conversation history) to see if the self-model contributes anything those alternatives don't.

None of this has been done.

**Loose end:** the prioritizing-todos thought experiment is the test case where the self-model's value would become measurable. If priority judgments require integrating multiple state structures, and the integration has to be reportable in trace-grounded terms, the self-model is what would make the integration legible. But this is exactly where the current architecture might fail — if priority reasoning happens at the LLM-prompt-integration layer rather than in inspectable structured state, the trace will cover the tool-call sequence but not the cognitive integration that produced the priority. The justification will be confabulated. We have not tested this, and it is genuinely possible that the current architecture cannot pass this test.

## 6. Conjecture: derived concerns as motivational source for self-commitments

**Status: Architectural conjecture with formal similarity but possibly different signal types.**

The proposal: the same mechanism that surfaces "this seems worth tracking" from interaction patterns (about the user) would, with reflexive scope, generate "I'm committed to maintaining attention to this" as an emergent self-commitment (about the agent's own behavior).

What is true: both processes have the same formal shape — inference from pattern producing structured state.

What I overstated: that the same machinery cleanly handles both. The signals are different in kind. Derived concerns about the user are inferred from the user's stated interests, repeated topics, expressed preferences, and ongoing tasks. Self-commitments would be inferred from the agent's own behavior — which patterns it tends to fall into, which moves it tends to make, which failure modes it tends to repeat. The agent observing its own behavior to derive commitments is a different epistemic situation than the agent observing the user's behavior.

This may unify cleanly in implementation. It may also turn out that observing one's own behavior produces different kinds of inferences than observing another's, requiring different machinery. This is empirical, not architectural-by-fiat.

**Loose end:** untested. Even building the self-model wouldn't directly answer it; you'd need a working self-observation loop and evidence that the inferences it generates are accurate and useful.

## What the prioritizing-todos thought experiment reveals

The thought experiment is more diagnostic than I credited initially. It is not just another use case; it is the place where several conjectures get tested at once.

A user asks Jill to prioritize a list of todos with reasoning. To do this well, the agent must:

1. Read the list (mechanical, no architectural challenge).
2. Integrate multiple state structures: companion model (what matters to the user, what's on their mind), concerns (durable commitments, derived patterns), discourse state (active commitments, unresolved issues). This is the hard part.
3. Produce an ordering with explicit reasons.
4. *Justify the reasons in trace-grounded terms* if asked.

Where this exposes loose ends:

- **The integration step happens in the LLM forward pass, not in inspectable structured state.** The agent reads the relevant state into its prompt, the LLM produces an integrated judgment, and the trace will record only the tool calls that surfaced the state plus the response. The actual cognitive integration that produces the priority ordering is opaque to the trace — it lives in the model weights' processing of the assembled prompt. So when the user asks "why is X above Y," the agent's answer is *post-hoc* about the integration. This is exactly the confabulation pattern the trace mechanism was supposed to prevent, occurring at a level the trace doesn't reach.

- **This is not a failure of the implementation; it is a structural feature of LLM-based architectures.** Cognition that happens in the forward pass is not introspectable by definition. The trace covers what is *between* forward passes (tool calls, structured state changes) but not what happens *inside* them. Priority judgment is the kind of thing that happens inside.

- **The qualia framing predicts this if it is genuinely predictive.** Anywhere the architecture relies on LLM forward-pass integration, there will be unexplained-input-shaped artifacts in the output that the agent cannot ground. Priority ordering is one such artifact. The agent's "reasons" for the ordering will be confabulated to varying degrees of plausibility.

- **The self-model conjecture (step 5) does not obviously fix this.** Adding self-model state gives the LLM more inputs to integrate, which might produce better priorities, but the integration still happens in the forward pass. More structured state does not by itself make integration inspectable.

This is the most concrete loose end identified so far. It suggests that the trace mechanism's coverage is narrower than initial framings suggested, and that "trace-grounded reasoning" only applies to reasoning that occurs *between* forward passes, not within them. Almost all interesting cognitive work happens within forward passes.

**Possible next directions, none yet developed:**

- *Decompose priority reasoning into multiple smaller LLM calls* with structured state changes between them, so the integration becomes visible at the trace level. Trades cognitive coherence for inspectability.
- *Use a separate LLM call to introspect on the priority judgment* and produce a trace-eligible explanation. This just moves the confabulation from one model to another; the introspecting model has no privileged access.
- *Accept the limitation*. Priority judgment is substrate-level; the agent should be honest that its reasoning is post-hoc reconstruction, not trace-reading. This requires the agent to reliably distinguish "I am reading my trace" from "I am post-hoc reconstructing" — which is itself a non-trivial discrimination.
- *Use mechanistic interpretability* to actually see what's happening in the forward pass during priority judgments. Way out of scope for current work but the literature here is moving fast.

## Other loose ends

- **The Gemma/Granite/Sonnet comparison was suggestive but not rigorous.** Same prompts on different backends produced different behaviors, but the differences confounded model capability with model register. Useful for engineering choices, not useful as evidence about what the architecture contributes.

- **The introspective fidelity benchmark is drafted but not run.** Until executed, claims about the architecture's introspective capacities are anecdotal.

- **The post is published as engineering, but step 3 is in it (the "you put it there" rewrite).** This means I have publicly committed to "the qualia framing illuminates failure modes" without having validated that the illumination is more than retrospective rationalization. Worth being aware of if the post draws engagement.

- **The relationship between this work and predictive processing has been waved at, not engaged.** The "prediction" angle for ReAct loops was raised but not followed up. PP has its own large literature; whether it offers a better framing than qualia for what the trace mechanism does is genuinely open.

- **No part of this engages the strong-AI vs. weak-AI debate properly.** The post brackets phenomenal awareness, which is fine, but the architecture is doing something that might or might not have implications for that debate, and I have not done the work to know.

## What I am not committing to

- The self-model architecture as currently sketched (step 5). It is unbuilt and the definition that motivates it is partly self-confirming.
- The unification of derived concerns and self-commitments (step 6). It is formally clean but the underlying signal types may differ in practice.
- The claim that the small-q qualia framing is *predictive* in the strong sense, rather than just consistent with observed failures and structurally suggesting where future failures will appear. Predictive purchase needs validation against tests.
- Any claim that the trace mechanism covers more than between-forward-pass reasoning.
- Any claim about phenomenal Qualia (capital-Q). That is a separate position Bruce holds, developed independently of this work, and is not argued for here.

## What I am committing to

- The trace mechanism makes between-forward-pass reasoning more accurately reportable than it would otherwise be. This is demonstrated for Sonnet-4.6 in specific test conversations. Generalizability is open.
- The small-q qualia framing was generative for the design and is consistent with observed behaviors. Whether it is empirically privileged over alternative framings (e.g., predictive processing) is open.
- The structural-ineliminability claim about small-q qualia (step 4 as revised) is defensible as an operational statement about computational architectures. It is narrower than a strong philosophical regress argument and most of the literature objections to such arguments do not land on it.
- The current architecture has identifiable limits, the clearest being that priority judgment and other within-forward-pass cognition is not trace-accessible.

## How to use this document

This is an audit document, not a manifesto. When something I have claimed gets falsified or refined by experiment, mark it. When a literature objection turns out to be answerable, note the answer and where. When a loose end gets tied off, record what tied it. The trajectory continues; this version is correct for the date stamp and probably wrong in interesting ways within months.

The previous version of this document overstated inevitability and coherence. The corrective is not to flatten everything into uncertainty, but to be specific about what has been done versus conjectured, and where the literature pushes back.

The synthesis-as-progress framing is seductive and partly false. Some of the steps below were arrived at in conversation through the kind of mutual elaboration that produces apparent depth without producing actual evidence. The check is always: would this conjecture survive contact with someone who disagrees and is competent?

Future-Bruce: read this skeptically. The trajectory was real but the writing-up was sometimes too eager.

---

*Bruce, late April 2026.*
*Companion to: substack draft "A Small Step Toward Agents That Can Talk About Themselves"*
*Repo: Cognitive Workbench / docs/*
*Revision 2: skeptical pass after sycophancy was flagged in the first version.*
*Revision 3: section 4 corrected after I had overcorrected — the strong regress argument I framed as embattled was not actually the claim being made; the narrower operational claim about small-q qualia stands. Terminology distinction between small-q and capital-Q Qualia made explicit throughout to prevent the conflation that produced the overcorrection.*
