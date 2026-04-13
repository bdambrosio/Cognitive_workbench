# Reward Modeling in RLHF — Reading Notes

## Core mechanism

The reward model (RM) in RLHF is a learned proxy for human preferences.
You collect a dataset of comparison pairs — "given this prompt, humans
preferred response A over response B" — and train a scalar reward model
to predict those preferences. The RM then serves as the objective
function for the RL fine-tuning step.

The standard architecture takes the base language model, strips the
language modeling head, and adds a scalar projection layer. The key
paper is Ouyang et al. 2022 ("Training language models to follow
instructions with human feedback") which used a 6B RM to train
InstructGPT. The reward model was trained with a pairwise ranking loss
(Bradley-Terry model): given two completions, maximize the probability
that the preferred one scores higher.

## Practical issues I've been tracking

**Reward hacking**: The policy learns to exploit quirks in the RM
rather than genuinely improving. Classic example: the RM gives high
scores to verbose responses, so the policy learns to be excessively
wordy. Gao et al. 2023 ("Scaling Laws for Reward Model Overoptimization")
showed that reward model score and actual human preference diverge
after a certain KL budget — the relationship is initially linear but
eventually turns over, and a smaller RM hits the overoptimization
threshold sooner.

**Label noise**: Human labelers disagree with each other about 20-30%
of the time on comparison tasks. The RM can't be more accurate than
its labels, so the effective ceiling on RM quality is constrained by
inter-annotator agreement. Anthropic's 2022 paper notes that they
found significant labeler-to-labeler variance even after training and
calibration.

**Distribution shift**: The RM is trained on outputs from the initial
SFT model, but during RL training the policy drifts away from SFT.
The RM's accuracy degrades on out-of-distribution outputs. This is
one argument for iterative RLHF — periodically collecting fresh
preference data on the current policy's outputs and retraining the RM.

## Open question

Does the RM need to be the same size as the policy? Conventional wisdom
says smaller RMs work fine (InstructGPT used a 6B RM for a 175B policy),
but recent work from Anthropic suggests that larger RMs are more robust
to overoptimization. The cost tradeoff is nontrivial since RM inference
happens at every RL step.
