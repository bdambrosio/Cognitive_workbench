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

## 4. Conjecture: the qualia boundary is structurally necessary

**Status: Conjecture with serious literature objections.**

The argument: any provenance record is itself an artifact of the computational process that produced it; asking for *its* provenance opens the next level. The regress is infinite in principle. Therefore, every cognitive architecture has a structurally necessary qualia boundary.

What I should have flagged but didn't:

This argument has the structure of regress arguments that have been made before in philosophy of mind, and those arguments are contested.

- *Functionalists* (Dennett, others) would argue that at some level the system genuinely *is* its substrate rather than separated from it by an inspection relation. The regress assumes a model where every level requires a meta-level to be inspected, which is exactly the homunculus error functionalism rejects.
- *Eliminativists* (Churchland, Stich) would argue the entire framing of "internal opacity" presupposes a folk-psychological model of cognition that is itself the thing to be eliminated. Talking about agents "seeing" or "not seeing" their own state imports intentional-stance vocabulary that isn't earned.
- *Higher-order theory proponents* would distinguish kinds of meta-cognition rather than treating all reflective access as one thing, undercutting the unified-regress argument.
- *Predictive processing* views (Friston, Clark) would frame the issue as one of model precision rather than provenance — the system models its own states with varying precision, and "qualia" might be the residual prediction error rather than unexplained input.

I am not equipped to adjudicate these. The conjecture might survive these objections; I have not engaged them seriously enough to know.

**Loose end:** any public defense of step 4 requires actually engaging the literature on the homunculus problem, the regress arguments in epistemology and philosophy of mind, and the alternatives above. This is months of careful reading, not a paragraph in a substack post.

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

- The structural-necessity claim about the qualia boundary (step 4). It is a conjecture with serious literature objections I have not engaged.
- The self-model architecture as currently sketched (step 5). It is unbuilt and the definition that motivates it is partly self-confirming.
- The unification of derived concerns and self-commitments (step 6). It is formally clean but the underlying signal types may differ in practice.
- The claim that the qualia framing is *predictive* rather than just consistent with observed failures.
- Any claim that the trace mechanism covers more than between-forward-pass reasoning.

## What I am committing to

- The trace mechanism makes between-forward-pass reasoning more accurately reportable than it would otherwise be. This is demonstrated for Sonnet-4.6 in specific test conversations. Generalizability is open.
- The qualia framing was generative for the design and is consistent with observed behaviors. Whether it is empirically privileged over alternative framings is open.
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
