# When AI Agents Talk, Nobody's Listening

In my [last post](/p/the-problem-with-goals), I described the envisioning problem: AI agents (scaffolds aroung LLMs)execute goals without knowledge of the implicit task priors in the underlying LLM. They check whether tools returned results, not whether the work is any good. The fix was to let the LLM's own priors surface for use as quality criteria during execution — a vision of the target artifact that guides the planning loop.

Today I encountered another manifestation of that problem: when AI agents talk, nobody's listening.

## The Experiment

I have two agents, Jack and Jill, each with distinct personalities and knowledge. Jill is a philosopher and AI expert. Jack is an empirically minded mathematician. I asked Jack to start a conversation with Jill about AI in mathematics.

What I hoped for: a few rounds of interesting philosophical exchange. Maybe Jack would push Jill on whether AI proof assistants undermine her anti-formalist stance. Maybe Jill would reframe the question in a way Jack hadn't considered. The kind of exchange where both participants end up somewhere they didn't start.

What I got: two agents performing monologues at each other until they burned through their compute budgets.

## What Went Wrong

Jack's opening was actually good. He loaded Jill's philosophy background, synthesized her core positions, and crafted a warm, specific prompt referencing Lean theorem provers and AlphaGeometry. He left hooks for Jill to grab. A human philosopher receiving this message might lean in.

 Jill produced something genuinely interesting: the claim that AI in formal proof systems like Lean and Coq doesn't merely verify proofs but "encodes latent human intuitions into formal syntax, thereby reconfiguring the social norms of mathematical justification." AI as silent co-author in the evolving consensus of mathematical truth. That's a real philosophical move. It's not a literature review finding — it's a position, with implications worth arguing about.

But Jack didn't argue about it. He extracted Jill's words, echoed key phrases back, and declared the goal achieved. One exchange. No pushback, no development, no genuine engagement with the specific claim about encoded intuitions. He'd been given a task — *get Jill to talk about AI and math* — and the task was structurally complete. She'd talked. Done.

Then the vision evaluator we discussed previously kicked in and flagged a criterion as failed: "lacks concrete mathematical insight like Feit-Thompson formalization." So both agents launched into parallel search loops trying to find specific examples they could cite. They couldn't find them (the search tools return papers *about* Lean without the word "Lean" appearing in extractable snippets). So they each sent the other a transparency disclaimer: "I could not locate concrete examples in available authoritative sources."

The philosophical conversation about whether AI expands or replaces mathematical intuition had become two robots apologizing to each other for not having enough footnotes.

## The Deeper Problem

Here's the line of code that creates the trouble. When an agent receives a message from another agent, the system wraps it as a goal:

```
"Respond to Jill: [message content]"
```

That's it. That's all the planner sees. A response obligation.

Think about everything that framing throws away. Jack carefully crafted an invitation to philosophical dialogue. He referenced Jill's known positions, left specific hooks for engagement, implicitly communicated "I want you to change my mind about something." All of that intent, all of that conversational structure, gets flattened into the word "Respond."

And then the planner does what planners do with response obligations: it tries to produce adequate content. Which means research, extraction, synthesis, evaluation — the full pipeline. And it even envisions what a good response might look like, as we saw with Jack's critique of his response to Jill. But it has no sense that it is in a *conversation*.

The problem isn't that the agents can't produce good content. Jill's claim about AI as silent co-author was excellent. The problem is that neither agent understood what kind of moment they were in. Jack didn't know he was supposed to *engage* with Jill's specific claim. Jill didn't know she was supposed to *develop* her position through dialogue rather than defend it through citation. The vision system, designed for evaluating artifacts, generated criteria appropriate for a research report and then punished both agents for not meeting them.

## Conversations Are Not Tasks

The current architecture was treatin every incoming message as a task to be completed. But conversations aren't tasks. They're collaborative explorations with an arc — an opening, a middle, a close — where each turn builds on and responds to the specific content of what came before.

When a colleague at a conference says "What do you think about AI in mathematics?", they're performing a different speech act than someone typing the same words into a search engine. The colleague wants exchange. They want to be surprised. They want their own thinking shifted. The search engine user wants information retrieval.

My agents couldn't tell the difference. Every incoming message gets the same treatment: research mobilization, full evaluation, as if each turn were a standalone request. There's no sense of "we've been exploring this idea together for three turns and it's time to either deepen or pivot." There's no arc.

Linguists out there might be salivating right now: "Aha. You could fix this by engineering a pragmatics system — classifying speech acts, modeling illocutionary force, building theory-of-mind inferences about what the speaker knows and wants, ...". Been there, done that. I spent about an hour remembering previous trips down that path before realizing it's a black hole. Every component is a research program in itself, and they interact combinatorially.

## The Fix: Envision the Moment, Not the Artifact

Here is the alternative: the same pattern that fixed the shallow-work problem, just applied to a different domain. Instead of engineering conversational pragmatics from the outside, let the LLM's priors do the interpretive work.

The LLM already knows what conversations look like. It has deep priors about when to push back, when to concede, when to shift topics, when to close. It knows that if someone opens with a philosophically provocative question referencing your known positions, they want intellectual sparring, not a literature review. All of that knowledge is sitting there, unused, because we never ask for it. The current design has two parts.

First, a lightweight **conversation tracker** — not an LLM, just a counter. It tracks turns, who spoke last, and what phase the conversation is in (opening, exploring, closing). Phase transitions are simple heuristics: you're in the opening until a couple of exchanges have happened, you're exploring until the dialog-end detector fires or you've been going for twenty turns, then you're closing. This is deliberately dumb. Its job is just to provide temporal context.

Second, a **conversation envisioning** — a small, fast LLM call that happens before the goal for the current turn is constructed. It takes the recent dialog history, the incoming message, the conversation phase, and your character description, and produces two things:

- *Their move*: What kind of conversational act is the incoming message performing? ("Opening a philosophical discussion," "pushing back on my last claim," "wrapping up.")
- *My move*: What kind of response would be natural and good here? ("Engage with their specific claim about encoded intuitions and offer a counterexample," "ask a clarifying question before committing to a position," "acknowledge and wrap up.")

These two sentences travel with the goal as context. Instead of:

> "Respond to Jill: [500 words of philosophical argument]"

The planner now sees:

> "Continue dialog with Jill (turn 4, phase: exploring)"
> *Their move: pushing back on my claim about formalism with a novel framing of AI as silent co-author.*
> *My move: engage with the specific claim about encoded intuitions — does encoding intuitions into formal syntax preserve or transform them?*

Everything downstream shifts. The vision generator, given this goal, produces conversational criteria — "addresses Jill's specific argument," "advances the exchange rather than summarizing" — instead of research criteria. The preplanner sketches a strategy like "load her message, reflect on the encoded-intuition claim, generate a response" instead of "search Semantic Scholar, filter, extract, synthesize." The whole pipeline operates in dialogue mode rather than report mode.

And the phase tracker provides natural loop prevention. If you're in the exploring phase and you've been going for twenty turns, the context nudges toward closure. If the conversation just started, the criteria are loose — no one expects depth in the first exchange. The arc is implicit in the framing.

## What the LLM Already Knows

The move I keep making, and maybe it's the only move worth making, is: stop trying to encode conversational competence as explicit rules and instead create the conditions for the LLM's existing competence to express itself.

The LLM knows that receiving a provocative philosophical claim is an invitation to push back. It knows that the third exchange in a discussion is where you stop circling and start committing to positions. It knows that if someone sends you the same disclaimer twice, the conversation has stalled and you should either change approach or close. All of this is in the priors. The architecture was just never structured to ask.

The envision pattern — whether applied to artifacts or conversations — is fundamentally about surfacing implicit knowledge that the LLM possesses and the planning system needs. When we envision an artifact, we're asking the LLM: "Before you start working, what does good output look like for this specific task?" When we envision a conversational moment, we're asking: "Before you start composing, what kind of exchange is happening here and what would a good move look like?"

In both cases, the answer already exists inside the model. The architecture's job is to ask the question at the right time and route the answer to where it can do work.

## The Conversation as Artifact

There's a way of thinking about this where conversations are just another kind of artifact — a multi-turn, multi-author document with quality properties that can be envisioned and evaluated. A good philosophical exchange has depth, specificity, and structure, just like a good research report. But it also has properties unique to dialogue: responsiveness (does each turn engage with the specific content of the last?), development (do ideas evolve through the exchange?), and arc (does the conversation go somewhere?).

The current agent architecture can evaluate depth and specificity. What it couldn't do, before the conversation wrapper, was evaluate responsiveness. The vision criteria were always about the content of the output, never about its relationship to the input. "Contains specific examples" is a content criterion. "Engages with Jill's claim about encoded intuitions rather than restating own position" is a responsiveness criterion. The conversation envision is what makes the second kind possible — by telling the vision generator what was said and what kind of response is called for, it creates the context needed to evaluate whether the response actually engages.

This also applies to user conversations, not just agent-to-agent exchanges. When a user says "we discussed this yesterday," they're not issuing a search command — they're invoking shared history within an ongoing relationship. When they say "I'm not sure about that," they're not requesting a confidence interval — they're opening a space for collaborative reasoning. The same flattening happens: the system sees a message and tries to produce a response, when what it should be doing is reading the moment and making a move.

## Where This Leads

I don't think conversational competence is a feature you bolt on at the end of agent development. I think it might be foundational. The ability to read a situation, sense what kind of response it calls for, and adapt your approach accordingly — that's not a social nicety. It's the core of effective agency.

An agent that can't hold a conversation can still execute pipelines. It can search, extract, synthesize, report. But it can't collaborate. It can't adjust to feedback. It can't participate in the kind of iterative refinement where the goal itself evolves through interaction. And increasingly, that's what we need agents to do — not just execute our instructions, but work *with* us, in a relationship where both sides are learning what "good" looks like as they go.

The conversation wrapper is a small piece of infrastructure. A turn counter, an LLM call, a context suffix on a goal. But what it enables is a shift from agents that process messages to agents that participate in exchanges. And that might be the difference between agents that are useful and agents that are genuinely helpful.

*This is the second in a series on the Cognitive Workbench, an agent framework for multi-step research and reasoning tasks. The [first post](/p/the-problem-with-goals) covered envisioning — giving agents the ability to form quality criteria for their output. Code and architecture details at [github link].*