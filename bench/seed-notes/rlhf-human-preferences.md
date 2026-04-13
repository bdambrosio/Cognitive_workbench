# Human Preferences in RLHF — Reading Notes

## The preference collection pipeline

The entire RLHF framework rests on human preference data. The standard
pipeline: present labelers with a prompt and two (or more) model
completions, ask them to rank or choose the better one, collect at scale.
OpenAI's InstructGPT paper reports using about 40 labelers to collect
~33K comparison pairs for training the reward model.

## What "preference" actually means

This is murkier than it sounds. When a labeler says "A is better than B",
what are they evaluating? Helpfulness? Correctness? Safety? Style?
Fluency? In practice, labelers are given rubrics that try to decompose
the judgment, but the final comparison is still a single holistic ranking.

Anthropic's work on HHH (Helpful, Harmless, Honest) tried to make the
criteria explicit, but even within that framework there are tradeoffs:
a maximally helpful response to "how do I pick a lock?" conflicts with
harmlessness. The labeler's resolution of that tension becomes training
signal.

## Inter-annotator agreement

Labelers disagree a LOT. Ouyang et al. report ~73% pairwise agreement
among their trained labelers. Anthropic found similar numbers. This
means roughly 1 in 4 training examples has a label that another
qualified labeler would have reversed.

This noise has practical consequences:
- The reward model's accuracy ceiling is bounded by agreement rates
- Majority-vote aggregation helps but doesn't eliminate the issue
- Some preference pairs are genuinely ambiguous — reasonable people
  disagree, and there may be no "right" answer

## Labeler demographics and bias

Who the labelers are matters. InstructGPT used contractors recruited
via Upwork and Scale AI, predominantly English-speaking, skewing young
and tech-literate. Anthropic has discussed using "red team" labelers
with specific expertise for safety-related comparisons.

The concern: the model learns the preferences of a specific demographic,
not "human preferences" in the abstract. Cultural, linguistic, and
individual variation in what counts as "good" output gets collapsed
into a single reward signal. This is under-studied relative to its
importance.

## Preference data scaling

How much preference data do you need? The scaling isn't well understood.
InstructGPT used ~33K comparisons. Llama 2 used over 1M. It's unclear
whether 10x more data gives meaningfully better alignment or just
overfits to labeler quirks. The Anthropic 2023 paper on RLAIF
(Reinforcement Learning from AI Feedback) suggests that AI-generated
preferences can supplement human data and potentially scale further,
but this introduces its own issues around the AI's biases being
amplified.

## Constitutional AI angle

Bai et al. 2022 proposed replacing some human labeling with
AI-generated feedback guided by a constitution — a set of principles
the model should follow. The AI critiques and revises its own outputs,
generating synthetic preference pairs. This scales better than human
annotation but raises questions about whether AI preferences genuinely
align with human values or just approximate them.
