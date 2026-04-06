# Your Agent Needs to Care

AI agent frameworks handle events well — a user asks something, the agent responds; a trigger fires, the agent acts. The engineering goes into tool use, planning, memory retrieval, chain-of-thought reasoning. All of it assumes the agent already knows what matters. The question is just what to *do* about it.

But that assumption only holds when someone is standing there issuing commands. The moment you want an agent that operates autonomously — that notices things on its own, maintains ongoing priorities, acts without being asked - you hit a harder question: *why* should it do anything? What should it care about, and how does that manifest as behavior?

I've been working on this for several years, first in a system for multi-agent improvised plays, and for the past year in building Jill ([https://github.com/bdambrosio/Cognitive_workbench.git], Cognitive Workbench, ), an autonomous agent framework. My solution turns out to require something that looks less like a task queue and more like a model of concern.

## The Idle Agent Problem

Consider an agent that has finished responding to your last message. Nothing is happening. No events in the queue. What should it do?

In most frameworks, the answer is: nothing. Wait for the next event. The agent is a sophisticated listener with no inner life between stimuli — a telephone that does impressive things when it rings, but is otherwise inert.

This is fine for assistants. It's not fine for agents that are supposed to maintain your systems, track your evolving interests, or notice that something you asked about three days ago has a new development. Those capabilities require the agent to hold a persistent model of what matters and periodically act on it *without being prompted*. Otherwise our elaborate "agent harness" is just an expensive way to implement IFTTT.

The obvious solution — just give the agent a list of standing tasks — doesn't work well in practice. A task list is static. It doesn't know that your interest in a topic has been intensifying over the past week of conversations, or that the system health check it ran two hours ago is now stale, or that a pattern across several unrelated events adds up to something worth investigating. Tasks are answers to the 'what to do' question. What we need first is a mechanism for formulating the questions.

## Concerns Are Not Tasks

A task says what to do now. A concern does something more durable: it preserves the fact that some domain continues to matter, even when no immediate action is required.

That matters because what a concern demands is not fixed in advance. The same concern may later justify a routine check, a deeper investigation, a notification, a repair attempt, or no action at all. Which response is appropriate depends on context, timing, competing priorities, and what new evidence has accumulated since the last time the issue surfaced.

This is the core reason schedulers and standing task lists are not enough. They can repeat actions. They are much worse at preserving a live evaluative pressure that can later generate different actions.

In Jill, concerns are meant to play exactly that role. They are not queued jobs waiting to fire. They are persistent, revisable structures that keep certain domains of meaning in play: concerns keep important domains in play long enough for judgment to reappear.

## From Permanent Drivers to Working Concerns

What finally worked was not a bigger task list, but a different structure.

At the top is a small set of permanent concern sources: homeostasis, concern for the user, and concern for the user’s interests. You can think of these as the agent’s constitution: a minimal set of standing drivers that are always available to shape interpretation. Homeostasis is there because an agent that cannot preserve its own operational integrity cannot sustain any other form of autonomy. Concern for the user is there because the agent’s own behavior can affect the user, sometimes helpfully and sometimes harmfully; it has to track, however imperfectly, how its actions bear on the user and let that shape what it does next. Concern for the user’s interests is there because long-running helpfulness also depends on tracking what the user keeps returning to, exploring, and developing across time.

These are not tasks, and not even ordinary working concerns. They are enduring drivers — the stable reasons the agent is prepared to notice some things rather than others.

That distinction matters because these sources do not directly tell the agent what to do. They generate more specific derived concerns: situated concerns that arise in context. A homeostatic driver may produce a concern about stale health information or repeated tool failures. Concern for the user may produce a need to follow up on something time-sensitive. Concern for the user’s interests may produce a concern about tracking a topic that has been developing across several conversations.

Those derived concerns are what actually enter working attention. They can surface weakly, accumulate activation, subside when addressed, and later reactivate when time passes or new evidence arrives. This is the level where persistence becomes behavior.

The point of this extra layer is that reactivation is not just repetition. A scheduler can rerun the same action on a timer. A derived concern can come back in a different form and justify a different response: routine monitoring, investigation, notification, repair, or no action at all. That is the difference between repeated behavior and continuing judgment.

Tasks sit downstream of that process. They are not the source of salience; they are what happens when a derived concern becomes active enough to warrant action. That is why a flat task list is not enough. Tasks answer the question of what to do next. Concerns preserve the prior question of what still matters enough to watch, reinterpret, and possibly act on again.

One design choice turned out to be important here: these persistent models have to change cautiously. If the system updates too aggressively, one intense interaction can distort its whole picture of what matters. So after each interaction, the model makes at most one substantive change: add a concern, revise one, close one, or do nothing. The result is slower, but much less brittle.

## Attention as a Dynamic, Not a State

What matters is not just which concerns exist, but how they vary in activation over time.

A single event may briefly surface a concern and then let it fade. Repeated weaker signals may do something different: they may build into a concern that persists long enough to demand judgment. That distinction matters because novelty and importance are not the same thing. A one-off anomaly may deserve notice and nothing more. A pattern across events may deserve investigation even if no single event looked decisive on its own.

This is where the concern system parts company with simple triggers. A trigger says: when condition C occurs, do action A. The concern system says something looser and more useful: when evidence accumulates around a domain, reconsider what, if anything, now needs doing. Sometimes the answer is still “nothing yet.” Sometimes it is monitoring, sometimes investigation, sometimes attaching a concern to existing work, and sometimes generating a new task.

A morning health check may find nothing. Later, a weak anomaly and a sequence of tool failures may reactivate the same homeostatic source, but now the right response is not just “run the same check again.” It may be diagnosis, state preservation, or notifying the user. The point of the extra layer is exactly this: reactivation is not just repetition. A scheduler can rerun the same action on a timer. A derived concern can return in a different form and justify a different response.

That extra judgment step is deliberate. Activated concerns do not immediately become tasks. They become candidates. Periodically the agent asks: is this already being handled, is it actionable now, does it compete successfully against other concerns, and would acting actually help? That buffer is what keeps the system from becoming compulsive — the autonomous equivalent of checking your phone every time it buzzes.

## The Inference Problem

There's a tension at the heart of this design that I want to be honest about. The user model layer claims to track what the user *cares about*, but what it actually does is run an inference over interactions and make judgment calls. The agent doesn't observe the user's concerns — it *infers* them, with all the fragility that implies.

An LLM interpreting a conversation might conclude you're "frustrated with deployment tooling" when actually you were just venting on a bad day. It might miss a deep interest you've never quite articulated. It might over-index on frequency of mention and miss the thing you brought up once, quietly, that actually matters most. And this does not just apply to inferring interests; concern for the user also requires the agent to infer, imperfectly, the impact of its own actions on the user and what they are likely to cost or help.

This is a real limitation, and I don't think it has a clean engineering solution. The conservative one-patch-at-a-time update helps — it means a single misread doesn't corrupt the model. The evidence references help — you can trace back *why* the agent thinks you care about something. But fundamentally, the agent is building a theory of mind about its user, and theories of mind are always partial and sometimes wrong.

I think this is worth acknowledging rather than engineering around, because it points at something true about what we're building. An agent that maintains a persistent model of what you care about is doing something qualitatively different from one that responds to your messages. It's forming a view of you. That view will be incomplete. The question is whether an incomplete, self-correcting model is better than no model at all — whether imperfect sustained attention beats perfect reactive amnesia.

I believe it does, but it's an empirical claim, not a settled one.

## Where Autonomy Actually Lives

The punchline of all this machinery is simple: autonomy doesn't live in the agent's ability to use tools or make plans. It lives in the idle cycle — in what the agent does when nothing is happening. 

Most agent architecture work focuses on what happens between a request and a response. The concern system focuses on what happens in the silence between interactions. That's where the agent maintains its model of what matters, reactivates concerns that have gone stale, identifies patterns that have been building across events, and decides — through its own judgment, not through a trigger — that something is worth doing. Are there any new papers on agent architecture? I should review the web-search tool code to see why it times out so often. ...

This is also where the hardest unsolved problem lives. The agent's autonomous work and its responsiveness to the user currently share a single cognitive control loop. When the agent decides on its own to investigate a building concern, and you send a message in the middle of that investigation, something has to give. The concern system is designed to handle this gracefully — user responsiveness is itself a concern whose activation spikes on incoming messages — but the engineering of attention-switching under resource constraints is genuinely difficult, and I don't claim to have it solved.

## What This Isn't

This is not “caring” in the deep human sense, and I do not want to smuggle in phenomenology by rhetoric. What the system does is more limited: it gives the agent functional analogs of sustained attention, persistent interest, and self-directed initiative. Whether that is enough for useful autonomous behavior is an empirical question, but I think it probably is. An agent that remembers what you have been working on, tracks how its own behavior affects you, and periodically checks whether it can help without being asked is qualitatively different to interact with than one that only waits for instructions.

Most agent frameworks treat autonomy as something you bolt on: a scheduler, a cron job, a trigger. The view I have arrived at is different. If an agent is going to act on its own in a meaningful way, it needs to maintain some ongoing model of what matters, update that model continuously, and let action emerge from that model through its own judgment. That is a harder engineering problem than scheduling, but I think it is the right one. The concern system makes salience persistent and revisable. 