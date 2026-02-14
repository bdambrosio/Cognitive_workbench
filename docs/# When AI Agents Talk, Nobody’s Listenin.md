# When AI Agents Talk, Nobody’s Listening

In my [last post](/p/the-problem-with-goals), I described what I called the *envisioning problem*: agents built around LLMs execute goals without access to the model’s implicit priors about what good work looks like. They check whether tools ran. They don’t check whether the result is any good.

The fix was to surface the model’s own sense of the target artifact — to ask, before acting, “what would good look like here?”

This week I ran into a related failure mode: when AI agents talk, nobody’s listening.

---

## The Experiment

I set up two agents, Jack and Jill. Jill is a philosopher and AI expert. Jack is an empirically minded mathematician. I asked Jack to initiate a conversation with Jill about AI in mathematics. What I hoped for: a few rounds of real exchange. Jack might challenge Jill’s anti-formalism by pointing to Lean or AlphaGeometry. Jill might reframe the issue. Ideally, both would end up somewhere new. What I got: two monologues aimed in each other’s direction until they exhausted their compute budgets.

---

## What Went Wrong

Jack’s opening was good. He referenced Jill’s philosophical commitments and asked a pointed question about formal proof systems. There were hooks. Jill responded with a genuinely interesting claim: that systems like Lean and Coq don’t merely verify proofs but encode latent human intuitions into formal syntax, thereby reshaping norms of mathematical justification. AI as silent co-author in the social construction of proof.

That’s an arguable position. It invites pushback. Jack didn’t push back. He echoed key phrases and declared the objective satisfied. The task had been to “get Jill to talk about AI and math.” She had talked. Structurally complete. Then the artifact evaluator flagged a missing criterion from the response envisioning: insufficient concrete examples. Both agents spun up search loops trying to locate citations like Feit–Thompson formalizations. When tools failed to surface snippets containing the right keywords, they sent each other disclaimers about incomplete sourcing. A live philosophical exchange devolved into two systems apologizing for weak footnotes.

---

## The Architectural Fault

The root cause was simple. When one agent receives a message from another, the system wraps it as a goal:

> “Respond to Jill: [message]”

That framing discards everything about conversational intent. Jack’s opening wasn’t just a request for content. It was an invitation to dialogue. It carried signals about stance, tone, and expectation. All of that collapsed into a response obligation.

Planners handle response obligations the same way they handle research tasks: gather information, synthesize, evaluate against artifact criteria. The system can envision what a good report looks like. It has no concept that it is in the middle of a conversation. The vision generator generated criteria appropriate for a research report and then punished both agents for not meeting them. The failure wasn’t generative ability. Jill’s initial claim proved the model could produce insight. The failure was situational awareness. Neither agent understood what kind of moment it was in.

---

## Conversations Aren’t Tasks

The architecture treated every incoming message as a task to complete. Conversations aren’t tasks. They have phases and arcs. Early turns are exploratory. Middle turns deepen or challenge. Later turns close or pivot. Each move depends on the specific content and trajectory of what came before. A colleague asking, “What do you think about AI in mathematics?” at a conference is performing a different act than a user typing the same words into a search engine. One wants exchange. The other wants retrieval.

My agents couldn’t distinguish the two. Every message triggered the same pipeline. You could attempt to engineer pragmatics from scratch: classify speech acts, model illocutionary force, track theory-of-mind states. That path quickly becomes combinatorial.

---

## Envision the Moment

The alternative is simpler: Use the same “envision first” pattern, but apply it to conversational moments instead of artifacts. The LLM already knows what good conversations look like. It has deep priors about when to push back, when to concede, when to shift topics, when to close. It knows that if someone opens with a philosophically provocative question referencing your known positions, they want intellectual sparring, not a literature review. All of that knowledge is sitting there, unused, because we never ask for it.

Add two small components:

1. **A conversation tracker**  
   A minimal state machine: turn count, speaker, coarse phase (opening, exploring, closing). No deep semantics. Just temporal context.

2. **A conversational envision step**  
   Before constructing the goal for the next turn, make a small LLM call that answers two questions:

   - *Their move*: What kind of act is this message performing?
   - *My move*: What kind of response would be appropriate here?

Instead of:

> “Respond to Jill: [argument]”

The planner sees:

> Continue dialog with Jill (turn 4, exploring phase)  
> Their move: reframing formal proof as social encoding of intuition.  
> My move: test whether encoding preserves or transforms intuition; offer counterexample.

That framing propagates downstream. The vision generator produces conversational criteria (“engages with specific claim,” “advances exchange”) rather than research criteria (“contains citations”). The planner sketches a dialogue strategy rather than a search plan.

---

## What the Model Already Knows

LLMs already encode strong priors about conversational dynamics. They know when a claim invites pushback. They know that early exchanges tolerate looseness and later ones require commitment. They recognize when repetition signals stall. None of that competence is used if the architecture never asks for it. *That* is the architectural flaw in most 'agent' systems. The envision step is just a query: *what kind of moment is this, and what would a good move look like?* The answer is already in the model’s priors.

---

## Conversation as Artifact

You can treat a conversation as a multi-author artifact with its own quality dimensions:

- **Responsiveness**: Does each turn engage the specific content of the previous one?
- **Development**: Do ideas evolve?
- **Arc**: Does the exchange move somewhere?

Traditional artifact evaluation captures depth and specificity. It does not capture responsiveness. Without explicit framing, the system cannot judge whether a response actually engages.

Adding conversational envisioning provides the missing context needed to evaluate that dimension.

---

## Why This Matters

An agent that cannot hold a conversation can still execute pipelines. It can search, extract, synthesize, and report. But it cannot collaborate. It cannot adjust its understanding of the goal through interaction.

Increasingly, useful agents will not operate in isolation. They will refine tasks jointly with users. That requires situational awareness.

The infrastructure change is small: a turn counter, a brief LLM call, a few lines of context injected into the goal. The effect is larger: shifting from agents that process messages to agents that participate in exchanges.

That may be the difference between systems that generate content and systems that actually work with us.

---

*Second in a series on the Cognitive Workbench, an agent framework for multi-step research and reasoning. The [first post](/p/the-problem-with-goals) introduced artifact envisioning. Code and architectural notes at the project repository.*
