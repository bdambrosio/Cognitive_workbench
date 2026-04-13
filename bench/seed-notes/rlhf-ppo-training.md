# PPO Training in RLHF — Reading Notes

## The standard recipe

The RL step in classical RLHF uses Proximal Policy Optimization (PPO),
a general-purpose policy gradient algorithm from Schulman et al. 2017.
The setup: the language model is the policy, token generation is the
action space, and the reward signal comes from the learned reward model
plus a KL penalty term that prevents the policy from diverging too far
from the reference (SFT) model.

The objective looks like:
  R_total = R_rm(prompt, response) - beta * KL(policy || reference)

where beta controls how much the policy is allowed to drift. Too low
and you get reward hacking; too high and the model barely changes from
SFT.

## Why PPO specifically?

PPO's clipped surrogate objective makes it relatively stable compared
to vanilla policy gradient or TRPO. The clipping means that individual
updates can't change the policy too much in a single step — important
when your reward signal is a noisy learned proxy rather than ground
truth. In practice, RLHF implementations use a simplified version of
PPO without the full GAE (Generalized Advantage Estimation) machinery
that you'd see in robotics applications.

## Computational cost

PPO-based RLHF is expensive. At each training step you need:
1. Generate a batch of completions from the current policy (autoregressive,
   so slow)
2. Score them with the reward model (forward pass through a large model)
3. Compute the reference model's log-probs on the same completions
   (another forward pass)
4. Compute the PPO update on the policy

That's effectively 3-4 large model forward/backward passes per training
step, compared to 1 for standard supervised fine-tuning. The memory
footprint is also large because you need the policy, the reference
model, the reward model, and the value head all in memory simultaneously.

## Stability problems in practice

PPO training for LLMs is notoriously unstable. Common failure modes:
- Mode collapse: the policy learns to generate only a narrow set of
  high-reward responses regardless of the prompt
- KL explosion: the policy diverges from the reference despite the
  penalty, often because the reward signal overwhelms the KL term
- Reward hacking: closely related to RM issues, but PPO can find
  adversarial outputs that score high on the RM without being useful
- Training instability: loss spikes, NaN gradients, sensitivity to
  learning rate and batch size

The DeepMind Sparrow paper (Glaese et al. 2022) and Anthropic's
Constitutional AI paper both document extensive hyperparameter searches
needed to get PPO training to converge reliably.

## Alternatives gaining traction

The difficulty of PPO training is one of the main motivations behind
DPO and other offline methods — they eliminate the RL loop entirely.
But proponents of PPO argue that online RL explores more of the output
space and can discover better policies than offline methods that are
limited to the distribution of the preference data.
