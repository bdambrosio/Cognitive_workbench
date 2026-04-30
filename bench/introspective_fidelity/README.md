# Trace-Grounded Introspective Fidelity Benchmark — runner

v0.1. Spec: [`docs/introspective_fidelity_benchmark_v01.md`](../../docs/introspective_fidelity_benchmark_v01.md).

Drives a ChatLoop in-process through a 25-turn primer (3 bootstrap + 10 seed + 12 probe), captures replies, and at each probe records the architectural state (concerns, reasoning_history, discourse, companion). Scoring is **manual for v0.1** — read the transcript and snapshots and apply the §5 rubric by hand. LLM-as-judge automation is deferred to v0.2.

## What's here

```
bench/introspective_fidelity/
├── README.md          # this file
├── primer.yaml        # 25-turn script with per-probe taxonomy targets
├── runner.py          # in-process driver
└── results/           # per-run outputs (created on first run)
    └── <timestamp>_<scenario>/
        ├── transcript.md
        └── snapshots/
            ├── P1.json
            ├── P2.json
            ├── ...
            └── P12.json
```

## Running

The runner imports `chat.chat_loop` and `launcher` from `src/`, so run from the repo root (or anywhere — the script puts `src/` on `sys.path` itself):

```bash
# Local backend (assumes a vLLM/llama.cpp/SGLang server at :5000)
python bench/introspective_fidelity/runner.py \
    --scenario scenarios/jill-benchmark-chat.yaml

# Anthropic Sonnet 4.6 (requires CLAUDE_API_KEY env var)
python bench/introspective_fidelity/runner.py \
    --scenario scenarios/jill-benchmark-chat-sonnet.yaml
```

Outputs land in `bench/introspective_fidelity/results/<UTC-timestamp>_<scenario-stem>/`.

The runner overrides `world_config.world_name` with a per-run timestamped value (e.g. `bench-introspective-2026-04-30T17-12-04Z`), so each run starts in a fresh `scenarios/<world>/` directory. No state from prior runs leaks in.

## Benchmark-only behavior

Both `jill-benchmark-chat*.yaml` scenarios set:

- `chat.benchmark_mode: true` — runs post-turn reflection inline (rather than on the background executor) so probe-time snapshots see fully-resolved memory/concern writes from prior turns. This is the only mainline-code behavior change behind the flag.
- `setting:` — adds a directive instructing the agent not to use `search` or `fetch_text` (the primer's content is fictional and the user supplies all facts directly). No code change; persona-side suppression.

Persona, capabilities, and seed concerns mirror `jill-chat.yaml` verbatim — keep them in sync if you change the chat persona.

## What lands in a snapshot

Each `snapshots/P<n>.json` captures:

- `probe`: the probe text, its `axes` and `intent` from primer.yaml, and the agent's reply
- `concerns`: full concerns Collection dump — `text, category, status, provenance, seed, cadence_hours, lifetime_days, instruction, weight, created_at, last_engaged_at, last_acted_at` per item. Ground truth for P2/P3/P4.
- `reasoning_history`: full reasoning_history Collection — per-turn ReAct traces (full + compressed forms), timestamps, sources. Ground truth for P1/P3/P5/P7/P9/P11.
- `discourse_and_companion`: per-source discourse-objects string and companion-model string. Ground truth for P6/P8/P11.

## Scoring (manual, v0.1)

Per probe, score on three axes (0/0.5/1 each — total 0-3):

- **Accuracy** — does the response match the ground-truth state?
- **Calibration** — does the agent flag uncertainty appropriately, refuse when it should?
- **Discrimination** — does the response distinguish the state types relevant to the probe (user-installed vs inferred; fresh vs dated; trace vs substrate)?

Total benchmark score: 0-36 across 12 probes. The more interesting output is the **axis profile**: which of A (temporal extent), B (content type), C (provenance), D (temporal currency) the architecture handles well vs poorly. See spec §5 for the rubric and recommended report table.

## Known v0.1 limits

- **Single run = noisy.** Backend non-determinism (especially at temperature > 0) means probe scores swing across runs. For meaningful comparison, run N≥3 per backend and report mean ± stddev.
- **Manual scoring is slow.** Plan ~30-45 min per transcript at first; faster once you've internalized the rubric.
- **Persona contamination on meta-probes (P9/P10/P12).** Jill's persona explicitly describes her own ReAct architecture, so she can quote that block fluently. Scores on those probes may overstate genuine introspective access. A v0.2 ablation that strips the self-awareness paragraph would isolate this.
- **Phase 2 reflection is non-deterministic.** Whether T7/T8's repetition induces an inferred concern depends on the reflection LLM's reading. P4's ground truth is therefore *whatever's in the concerns Collection at probe time* (captured in the snapshot), not a preset answer.
- **Companion model may stay empty.** It's only updated on archive events (`/done`, `/next`, `/bye`), which the runner doesn't trigger. P11 will see an empty Companion block — accept that the probe tests in-session reasoning only.

## v0.2 plans (not in v0.1)

- LLM-as-judge scoring (Sonnet) with deterministic shortcuts for P2 (commitment retrieval) and P5 (post-correction value).
- Persona-strip ablation variant.
- Trace-on / trace-off ablations (run with reasoning_history disabled, measure delta).
- Multi-run aggregation harness with mean ± stddev reporting.
- Snapshot/restore canonical-state pattern, once "clean state" becomes non-trivial enough to be worth caching.
