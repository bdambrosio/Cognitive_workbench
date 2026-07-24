# bench/disposition — learned-disposition experiments

Experiments for [docs/learned-disposition-design.md](../../docs/learned-disposition-design.md).
Read-only with respect to agent state (reads `autonomy.jsonl`, writes only
`results/` here).

## g1.py — G1 offline toy

Predict the judged outcome of an autonomous fire (helped+neutral vs
hindered+ignored) from fire-time context alone. G1 passes if a trained
model beats both base rate and the prompted local backend asked the same
question, on real judged records under leave-one-out.

```
python bench/disposition/g1.py               # embed+lr, numeric-lr (LOO)
python bench/disposition/g1.py --loco        # leave-one-concern-out check
python bench/disposition/g1.py --prompted    # + zero-shot backend baseline
```

Notes:
- `--prompted` hits the live vLLM backend via the production `LLM('vllm')`
  call path (raw-completion style, same as the agent's own calls). Don't
  run it while a composite bench run is in flight.
- Embedding runs on CPU deliberately (avoids the CUDA index-reversal
  pin hazard; 67 texts, cost is nil).
- The LoRA'd very-small-LM variant (design-note model 2) is not built
  yet; it only makes sense once these controls have numbers.
