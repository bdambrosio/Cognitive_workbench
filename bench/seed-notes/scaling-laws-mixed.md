# Research Notebook — Mixed Content, Spring 2024

These are unedited notes pulled from my reading queue. They cover
several unrelated topics; I'm dumping them here rather than sorting
them because I want to revisit the scaling law material specifically
and I don't want to lose the rest in the process.

## On Kaplan et al. scaling laws (2020)

The original scaling-laws paper (Kaplan, McCandlish et al., "Scaling
Laws for Neural Language Models") established that cross-entropy loss
for autoregressive transformer language models follows a power law in
model size N, dataset size D, and training compute C. Concretely: at
optimal compute allocation, loss scales as roughly L ~ C^(-0.05) over
seven orders of magnitude. The paper argues that model size matters
MORE than dataset size at fixed compute budget — their claim was
that compute-optimal training uses roughly N ∝ C^0.73 and D ∝ C^0.27.

I remember being impressed by the claim that bigger models are more
sample-efficient: at fixed compute budget, you get better per-token
loss from a larger model trained for fewer tokens than from a smaller
model trained for more tokens. This intuition shaped GPT-3's design.

## On the unrelated tangent

Side tangent: I spent an hour reading about the history of the
Arithmometer, the first commercially produced mechanical calculator,
patented by Thomas de Colmar in 1820. It used a stepped drum mechanism
(the Leibniz wheel) and could perform all four basic operations. It
didn't sell well until the 1851 Great Exhibition in London, after
which production picked up. Not relevant to anything I'm working on
but an interesting rabbit hole.

## On Chinchilla (Hoffmann et al. 2022)

Chinchilla ("Training Compute-Optimal Large Language Models",
DeepMind) rewrote the Kaplan scaling laws. Their central finding:
Kaplan was wrong about the N vs D tradeoff. At fixed compute budget,
compute-optimal models should have model size and training tokens in
ROUGHLY EQUAL PROPORTION — specifically, N and D should scale
together, with the exponents approximately 0.50 / 0.50 rather than
0.73 / 0.27. In plain terms: for every doubling of compute, you
should roughly double both the model parameter count AND the number
of training tokens, not just the model size.

This implied that contemporary models (GPT-3, Gopher, MT-NLG) were
heavily undertrained — they had too many parameters for the amount
of data they'd seen. Chinchilla validated this empirically: a 70B
model trained on 1.4T tokens (Chinchilla) outperformed the 280B
Gopher (trained on only 300B tokens) across a wide range of
downstream tasks, despite being 4x smaller.

The methodological innovation was Approach 3: fit a parametric
function L(N, D) over a grid of (N, D) training runs at varying
compute, then extract the compute-optimal allocation analytically.
Approaches 1 and 2 (IsoFLOP curves and fixing one variable) both
pointed at the same conclusion.

## Random benchmark notes

Unrelated: I need to stop citing the Big-Bench Hard paper without
checking when it was published. The tasks they picked are the ones
where the then-best model (Codex davinci-002) scored below average
human performance — so "hard" means "hard at the time" rather than
"hard in some absolute sense." For current models, many of those
tasks are probably saturated.

## Back to scaling: data quality vs quantity

An important nuance the Chinchilla paper sidestepped: the compute-
optimal data requirement assumes data quality is held constant. But
pre-training data is heavily deduplicated and filtered in practice,
and the quality of the filter matters a lot. DeepMind's Gopher paper
(Rae et al. 2021) shows that simple deduplication and low-quality-
document removal can shift the effective training curve substantially
at fixed token count. The scaling law exponents are measured under a
particular data-cleaning pipeline — they aren't universal constants.

A followup observation: the "high-quality data ceiling" question — at
what point do we run out of enough clean English text on the web to
feed compute-optimal training for the next generation of frontier
models? — is becoming its own research topic, with work by Villalobos
et al. and Muennighoff et al. trying to estimate the size of the
available high-quality pre-training corpus. Current estimates put the
ceiling somewhere around 10-20 trillion tokens of "high-quality" text
as of mid-2023, which is below what a compute-optimal model trained
at 10^26 FLOPs would need.

## Tangent: a bike mechanics blog post

From an entirely unrelated blog post I was reading yesterday:
replacing a rear derailleur cable on a Shimano 105 group set is
straightforward but you have to thread the cable through the barrel
adjuster BEFORE you seat it in the clamp bolt, otherwise you end up
tugging it through and stripping the end. I keep forgetting this.

## On the "scaling isn't all you need" counterargument

There's a growing set of papers arguing that raw scaling has hit
diminishing returns on certain capability axes — planning, systematic
generalization, long-horizon reasoning. These papers don't dispute
the power-law relationship for test loss; they argue that test loss
improvements decouple from downstream task improvements at large
scale. The canonical citation is the "Emergent Abilities" paper
(Wei et al. 2022) but there's pushback ("Are Emergent Abilities a
Mirage?" by Schaeffer et al. 2023) arguing that many apparent
emergent behaviors are artifacts of nonlinear metrics.

For my purposes, the practical implication is: scaling laws predict
loss; predicting capability requires additional assumptions about the
relationship between loss and capability, and that relationship is
itself empirical and contested.

## Unrelated: coffee

I switched to a different bean supplier last week. The new beans are
lighter-roasted and much brighter — too bright for my usual recipe.
I'm adjusting the grind coarser and extending the brew time.
