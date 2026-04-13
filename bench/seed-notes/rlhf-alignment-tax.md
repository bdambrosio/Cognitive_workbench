# The Alignment Tax — Reading Notes

## What is the alignment tax?

The "alignment tax" is the cost — in capability, performance, or
computational resources — of making a model aligned with human
preferences versus leaving it unaligned. If RLHF makes a model
safer and more helpful but also makes it worse at coding, that
capability drop is part of the alignment tax. If the RL training
adds 2 weeks and $500K to the training pipeline, that's the
compute/cost component.

The term was popularized in the AI safety community but has become
mainstream in ML engineering discussions. The aspiration is a zero
alignment tax: alignment that improves helpfulness without degrading
capability.

## Evidence for capability degradation

**InstructGPT findings**: Ouyang et al. 2022 found that RLHF
improved helpfulness and reduced toxicity but caused small regressions
on some NLP benchmarks (particularly closed-book QA). The regressions
were modest — a few percentage points — but nonzero. They partially
mitigated this by mixing in pre-training data during the RL phase.

**Llama 2 Chat vs Llama 2**: Meta's technical report shows that
the RLHF-trained chat models scored lower on several academic
benchmarks (MMLU, BBH) compared to the base models. The gap was
1-3 points, varying by benchmark. On the other hand, the chat models
were dramatically better on human preference evaluations.

**The "alignment tax" on reasoning**: Anecdotally and in some
benchmarks, heavily RLHF'd models become more cautious and hedging.
They say "I'm not sure but..." more often, refuse edge cases that
the base model would handle fine, and occasionally sacrifice
precision for safety. This is hard to quantify but real — users of
GPT-4 vs GPT-4-base in the API report noticeable differences in
willingness to engage with technical edge cases.

## Arguments that the tax is shrinking

**Better RL recipes**: As the field matures, alignment teams are
finding ways to reduce regressions. DPO in particular seems to
cause less capability degradation than PPO, possibly because it
makes smaller updates and doesn't suffer from reward hacking.

**Capability and alignment aren't always opposed**: For many tasks,
being helpful IS being capable. A model that follows instructions
precisely, admits uncertainty, and avoids confabulation is both more
aligned and more useful. The tax is mainly visible on tasks where
the alignment objective conflicts with raw capability (e.g.,
generating potentially harmful content that happens to be
technically accurate).

**Scale helps**: Larger base models seem to absorb RLHF with less
relative degradation. The alignment tax as a percentage of total
capability appears to decrease with scale, though this needs more
rigorous measurement.

## The safety-usefulness tradeoff

The hardest version of the alignment tax isn't capability regression
on benchmarks — it's the tension between safety and usefulness in
deployment. An overly cautious model is safe but annoying. An overly
permissive model is useful but risky. Every refusal is a local
failure of helpfulness. Every harmful output is a failure of safety.

Current practice: labs tune this tradeoff differently for different
deployment contexts (API vs consumer chat vs enterprise) using
system prompts, content filters, and different RLHF checkpoints.
But the fundamental tension remains.

## Open question

Can you measure the alignment tax rigorously? You'd need a benchmark
suite that covers both capabilities (coding, reasoning, knowledge)
and alignment properties (safety, helpfulness, honesty) and tracks
them jointly across the alignment pipeline. I don't think anyone has
built this comprehensively. Most evals measure one axis or the other.
