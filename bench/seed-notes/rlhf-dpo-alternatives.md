# DPO and Other Alternatives to PPO — Reading Notes

## Direct Preference Optimization (DPO)

Rafailov et al. 2023 ("Direct Preference Optimization: Your Language
Model Is Secretly a Reward Model") showed that the RLHF objective can
be reparameterized to eliminate the reward model and the RL loop
entirely. The key insight: the optimal policy under the KL-constrained
reward maximization objective has a closed-form relationship to the
reward function. You can invert that relationship to express the reward
in terms of the policy, and then optimize the preference loss directly.

The DPO loss is a simple binary cross-entropy over preference pairs:
for each (prompt, preferred, dispreferred) triple, increase the
log-probability ratio of the preferred response relative to the
dispreferred one, compared to the reference model. No reward model,
no sampling, no PPO — just supervised learning on preference data.

## Why DPO took off

Practical advantages over PPO:
- Much simpler to implement (hundreds of lines vs. thousands)
- No need to hold 4 models in memory simultaneously
- Training is as stable as supervised fine-tuning
- Easier to debug and reproduce
- Scales straightforwardly with data

Llama 2 used PPO; by mid-2024 most open-source alignment work had
shifted to DPO or variants. The Zephyr models (Tunstall et al. 2023)
demonstrated that DPO on a modest preference dataset could match PPO
results with far less engineering effort.

## Known limitations of DPO

**Offline only**: DPO trains on a fixed preference dataset. It can't
explore — it only learns from the responses already in the data. If the
preference data doesn't cover some part of the output space, DPO has
nothing to learn from there. PPO can actively generate and evaluate new
responses.

**Distribution mismatch**: The preference data was collected from a
different model (usually the SFT model). As the DPO-trained policy
improves, it moves away from the data-generating distribution. This
is the same problem as off-policy RL, and DPO doesn't have a mechanism
to address it.

**Length bias**: Several groups have reported that DPO-trained models
tend to be more verbose than PPO-trained models, possibly because
longer responses are more likely to contain the features that made them
preferred in the original data.

## Other alternatives

**IPO** (Identity Preference Optimization, Azar et al. 2023): Addresses
a theoretical issue with DPO — the loss can overfit when preference
strength approaches deterministic. IPO adds a regularization term.

**KTO** (Kahneman-Tversky Optimization, Ethayarajh et al. 2024): Works
with binary feedback (thumbs up/down) rather than pairwise comparisons.
This is practically important because binary feedback is much cheaper
to collect than comparisons. Uses a loss inspired by prospect theory's
asymmetric treatment of gains and losses.

**ORPO** (Odds Ratio Preference Optimization, Hong et al. 2024):
Combines SFT and preference optimization into a single training stage,
eliminating the need for a separate SFT step.

**SimPO** (Simple Preference Optimization, Meng et al. 2024): Uses the
average log-probability as an implicit reward, removing the need for a
reference model entirely. Claimed to outperform DPO on several
benchmarks.

## My take

The field is moving toward simpler methods. PPO → DPO → KTO/SimPO is a
clear simplification trend. The question is whether the simplicity
comes at a cost in the tails — does online exploration find edge-case
improvements that offline methods miss? For frontier labs with the
engineering capacity, PPO may still win at the margin. For everyone
else, DPO or its variants are clearly the practical choice.
