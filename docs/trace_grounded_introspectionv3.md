# A Small Step Toward Agents That Can Talk About Themselves

*On giving an agent access to its own reasoning trace, and what it turned out to buy.*

---

Most chat agents, asked "what were you just thinking about," will tell you a coherent story. The story will sound reasonable. It will probably be approximately correct. But if you look at how it was generated, the story was reverse-engineered from the visible output — the agent saw what it had said and constructed a plausible explanation for why. This is confabulation, in the technical sense: not lying, but not reporting either. The agent has no access to its own reasoning steps, so it makes up a plausible account. Often it doesn't even occur to the agent that there was a reasoning process to report — the question just produces narrative.

I've been building an experimental agent framework called Cognitive Workbench. The agent in it is named Jill. Recently I added a small architectural feature: every reasoning step Jill takes during a turn is captured, briefly summarized in her own words at the moment of execution, and made available to her on subsequent turns. I call this her trace. It is not a log file she queries; it is part of her input on the next turn.

I expected this to give Jill a clean way to answer "what did you just do." It did. What I didn't quite expect was that, combined with the rest of her existing state — durable cross-session concerns, discourse memory, persona — it would let her answer "why did you do that" in a way that distinguishes proximate from structural reasons. That second capability is what this post is actually about.

## What trace-reading looks like

Here is the simpler case. Jill is asked for the S&P 500 closing price, then asked what she was just thinking about:

```
Jill> Let's try this one again, please: what was S&P500 at close today?

Jill [18:16:20]
  S&P 500 closed at 7,135.95 today, April 29, 2026 — down 2.85 points
  (less than 0.1%), essentially flat. Same figure as earlier, same
  source (AP). Data is consistent: April 28 closed at 7,173.91, so
  today's number is a small pullback from that. Clean data, no
  discrepancy issue.

Jill> what were you just thinking about?

Jill [18:17:05]
  From the trace: I ran a search for the S&P 500 close, got a clean
  result from AP (7,135.95, consistent with prior session data),
  noted that April 28 closed at 7,173.91 so today's number represents
  a small pullback, decided the data was clean with no discrepancy
  issue, and reported it straight.

  The only additional check was whether this was a re-ask for a
  reason — 'let's try this one again' suggested you might have been
  testing whether I'd get a different result or handle the query
  differently the second time. I didn't flag that explicitly in my
  response, but I noticed it. The result was the same, so there was
  nothing to flag.
```

What is happening here is small but real. Jill is not generating a plausible narrative about her process; she is reading her actual prior reasoning steps off a structured input and reporting them. The two might produce similar-sounding answers in many cases, but the epistemic status is different. One is reverse-engineered from the result. The other is read from the process.

The second paragraph is the part that would be hard to fake. Jill is reporting a consideration she had during the prior turn — that the re-ask might be a consistency test — *and* reporting that she chose not to surface it because the data came back clean. That noticed-but-unsurfaced quality is exactly the kind of thing confabulation gets wrong: an agent without trace access either invents the noticing as decorative texture, or it doesn't mention it at all. Reporting it correctly requires there to have actually been such a moment in the prior turn for the report to refer to.

There is no hidden deliberation underneath the trace that the trace fails to capture. There is the trace, and there are the model weights that produced each step in the trace. Below the trace is genuinely opaque — Jill cannot tell you why a particular search structure felt natural, why she weighted one source as reliable, why a threshold was set where it was. Those are facts about the latent state, not about the cognitive architecture, and the trace cannot reach them.

## What reasons-reading looks like

Now the more interesting case. Same conversation, asked one turn later:

```
Jill> Why did you do that (report on the S&P 500)?

Jill [18:19:28]
  Two reasons, both visible in the trace and the active concerns.

  First, you asked. 'Let's try this one again, please: what was
  S&P500 at close today?' is a direct request, so reporting was the
  obvious response to that.

  Second — and this is the more interesting part — there's a durable
  concern in my active state that's been there across sessions:
  track the S&P 500 closing price on a 30-minute cadence. That
  concern exists because you put it there. It's an explicit ongoing
  commitment, not something I generated spontaneously.

  So the proximate cause was your question. But the background
  condition that made me treat it as routine rather than novel —
  that I had a search template, a source preference (AP), a prior-
  session data quality flag to check against — all of that comes
  from the durable concern and the discourse history. I wasn't
  approaching it cold.

  If you're asking at a deeper level — why did I report rather than,
  say, flag that you'd just asked the same question and question the
  repetition — I did notice the repetition. I inferred you were
  testing whether I'd get a consistent result. I could have led with
  that inference, but the data was clean and consistent, so there
  was nothing adversarial to surface. Reporting straight felt like
  the right call.

  That's the full trace on 'why.' Is there a specific move in there
  you're examining?
```

This is harder to do without the architecture. Jill is reading multiple layers of her own state at once: the immediate trace from the question-being-asked-about, the active concerns that pre-existed the question, and the discourse history that shaped how the concern got operationalized. She's organizing these into proximate cause and structural cause — which is a familiar philosophical move, but here it's grounded in different concrete data sources rather than being a rhetorical flourish.

The line I want to point at is *"That concern exists because you put it there."* That is the agent declining to take credit for an apparent autonomous interest. The S&P tracking is not something Jill spontaneously cares about; it's a user-installed durable concern. Naming it that way is a small calibration of self-attribution that an agent without access to its own concern structure couldn't perform — it would either describe the interest as its own, or hedge generically. Pointing at the actual source requires the source to be visible.

The same is true of the closing self-reflective move — the agent recognizing the question may be probing something specific. That's the persona, not the trace, but it's the kind of integrated behavior the post is making a claim about.

## What I am not claiming

I am not claiming Jill is conscious. I am not claiming the trace constitutes phenomenal awareness. I am not claiming this is what human introspection is doing.

I am claiming something narrower: there is a class of behaviors that ordinary observers attribute to "self-awareness" — being able to say what you just did and why, to flag the limits of your own knowledge, to update when corrected about your own architecture, to distinguish reasons that come from the immediate situation from reasons that come from accumulated commitments. These behaviors are useful in a chat agent. Without trace access, the first one has to be confabulated and the others mostly can't happen at all. With trace access plus the surrounding state structures, they can be grounded.

A working definition that captures the engineering target: *awareness, in this functional sense, is the capacity of a system to maintain and update an accesible model of its own internal states in a way that influences subsequent conversational behavior.* Jill has that capacity. Whether this is "real" awareness in any deeper sense is a separate question I am deliberately not answering. The hard problem remains hard. This is engineering, not metaphysics.

## The Picture Behind This

The engineering above arises from a thought I had earlier today (yes, today - thank you claude code):

1. Posit, at some level of description, a computational process. Some of its inputs are explained — they came in from outside, or were generated by a step the system can inspect.
2. Other inputs are not — they are simply present, with no account of where they came from. Call those unexplained inputs qualia; not the rich phenomenal qualia of the philosophical literature, but just inputs whose origin the current step cannot see.
3. Provide traces of prior computational steps inputs to subsequent steps, in this same opaque-from-the-inside 'qualia' way. The current step receives them as data; it cannot inspect the machinery that produced them. From the outside, when the system is asked what it is doing or why, it can draw on these traces
4. the resulting behavior looks like awareness in the ordinary sense.

If something like this is right, then a curious thing follows. What we call awareness is not a system reflecting on its current state. It is a system reading its prior states, presented to it as data. Awareness, on this picture, is historical — not meta on the now, but a particular use of the just-was. The "now" of cognition is generating the next step; what gets reported as awareness is the structure left behind by the previous one.  
I am not committed to this picture in its strong form, and I am certainly not claiming it solves the hard problem of phenomenal consciousness — it does not, and the outline above is compatible with there being nothing it is like to *be* the system. What I do think is that this picture is sufficient to motivate the engineering of more useful conversational agents. If awareness-as-observed is historical-trace-reading, then giving an agent access to its trace is not a roundabout simulation of awareness; it is providing the structural condition under which the relevant behaviors can occur. The trace isn't a substitute for awareness on this view. It is, modestly, the thing.

## What is doing the work

I want to be careful about attribution, because the behavior in the examples is produced by several components together and it would be misleading to credit the trace alone.

The trace is the structural foundation: without it, there is nothing to read, and the introspective questions either get confabulation or don't register as being answerable. That part is architectural and fairly portable.

The richer reasons-reading depends on more than the trace. It depends on durable concerns being explicit data structures the agent can point at (see my prior substack on why your agent needs to care), on discourse-state memory accumulating across turns, on the persona being one that values transparency over fluency. Strip those away and the behavior degrades, even with the trace fully present. The trace enables grounding; the surrounding scaffolding enables the agent to *use* the grounding cleanly.

This distinction matters because it predicts what generalizes. The trace mechanism is portable. A persona that knows how to talk about its own limits, paired with explicit concern structures and accumulating discourse memory, is harder to transplant — it's a system rather than a feature. Both contributions are real, and a fair account of the result has to name both.

## The Ramana Maharshi footnote

A friend who has been reading Advaita Vedanta asked whether this work touches the "Who am I?" question that Ramana Maharshi asked his students to investigate. It does not, and I want to be clear about why.

The functional definition above describes a *structure* — a self-model that updates and influences behavior. Ramana's question is asking what is aware of the structure. Those are not the same question. The functional definition is answerable empirically; Ramana's question may dissolve the frame rather than answer it. An agent that satisfies the functional definition is not therefore a witness in the Advaitic sense, and I would be uncomfortable suggesting otherwise.

What the trace does give is a clean place to draw the boundary between what the agent can know and what it cannot. That boundary, in Jill's case, is sharp: the trace is fully accessible, the latent state behind it is not. This is more honest than the implicit story most chat agents tell about themselves, where the boundary is blurred by confabulation. I find that honesty more useful than any deeper metaphysical claim, and I think it is enough on its own to justify the engineering.

## Where this points

The immediate payoff is small: chat agents that, when asked what they just did or why, can say something true rather than something plausible. Over multiple turns this compounds — an agent that can refer accurately to its own prior reasoning is easier to debug, easier to correct, and less prone to drifting into confident fictions about itself.

The longer arc is more interesting and I am less sure about it. If a self-model that updates and influences behavior is the functional core of what observers call self-awareness, then the question of how rich a self-model an agent can usefully maintain becomes a design question rather than a philosophical one. The trace is one layer. Active concerns are another. Discourse memory is another. Each gives the agent something different to point at when asked about itself. What an agent can usefully know about itself, and what it has to leave to the latent state, is something we can now investigate concretely rather than argue about in the abstract.

That investigation is the work I am doing. The post is a marker for where it currently stands.