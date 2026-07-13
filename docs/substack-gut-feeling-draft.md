# Gut Feelings Are Memory With the Index Thrown Away

*A musing, not a result.*

When we build memory systems for LLM agents, if we think about it at all, we usually assume: working memory (the context window), episodic memory (retrieval over stored experiences), semantic memory (distilled facts). Each tier trades detail for horizon — episodes decay into summaries, summaries into facts.

But perhaps the taxonomy stops too early. There's a tier of human memory it doesn't cover, the one with the longest horizon of all: the gut feeling.

## Gut feelings

A gut feeling is a judgment you can't justify. You meet someone and don't trust them. You look at a plan and something is off. Pressed for reasons, you invent a story, concede the point, or just shrug. But let's think about this semi-formally: a memory that, given a state, returns a scalar value, or at least a binary-value {-,+}. The ultimate memory abstraction. The episodes that produced that judgment are gone — you can't retrieve them, cite them, or replay them — but their *valence* survived. Hundreds of experiences got compressed down to a single signed scalar attached to a situation-shape: this kind of thing tends to go badly. Damasio's somatic marker hypothesis^1 is roughly this story told through the body.

The compression is extreme, and that's the point. Keeping the episodes around would cost too much and retrieve too slowly. What you actually need at decision time is not the evidence — it's the verdict.

## Value learning is the same move

Now look at what value learning does — a value function in RL, or preference learning more generally. It takes long-horizon outcome data and condenses it into a disposition. No episode is stored. Nothing is retrievable. The training experiences are unrecoverable from the artifact they produced. What survives is exactly what survives in a gut feeling: a fast, unjustifiable, situation-conditional valence.

This leads me in two directions:
- (1) perhaps we 'rationalists' should be a bit more humble in the face of 'gut feelings' or 'intuition'.
- (2) Perhaps value learning isn't only the final stage of training. Perhaps it is also the last tier of the memory stack — the one past semantic memory:

- **Working memory** — full detail, horizon of minutes
- **Episodic** — compressed detail, horizon of days
- **Semantic** — no detail, just claims; horizon of months
- **Values / gut feeling** — no claims, just valence; horizon of a lifetime

Each tier down, you lose auditability and gain horizon.

## Why this might matter for agents

Most agent memory work today concentrates on the middle tiers — better retrieval, better consolidation of episodes into notes. The top of the stack is the context window and the bottom is a document store, and both ends bottom out in *text*: content you can inspect.

If the analogy holds, the longest-horizon tier of an agent's memory shouldn't be text at all. It should be learned disposition — outcome data from the agent's own history distilled into something like a value signal over situations, with the episodes discarded. Fine-tuning as memory consolidation. Or, cheaper and more legible, learned valence annotations that fire before deliberation rather than after retrieval.

The uncomfortable part is that the loss of auditability isn't a side effect you can engineer away — it's the price of the horizon. A gut feeling that could fully explain itself would just be episodic memory. Whether that trade is acceptable in a system you're supposed to be able to inspect is an open question.

## Caveats

Gut feelings are also famous for being wrong — miscalibrated priors, out-of-distribution confidence, prejudice wearing the costume of intuition. The compression that gives them their reach is the same compression that makes their failures invisible. Any agent-memory version inherits all of that. And, of course, the danger of invented justifications even the agent doesn't realize are merely invented.

RL as a mechanism for this at agent run-time seems to require inventive replay. Work in progress.

I don't have results, only the observation that two things I'd been thinking about separately — memory-tier design and value learning — might be one thing seen from two ends of a horizon axis. If someone has seen this worked out properly, I'd genuinely like the pointer.

##Damasio
1. Damasio, Antonio R. (2008) [1994]. Descartes' Error: Emotion, Reason and the Human Brain. Random House. ISBN 978-1-4070-7206-7. Descartes' Error