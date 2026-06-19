# Reuse-gating capability-gap proposals

Design for forcing Jill's self-extension proposals to **start from "what can I
extend?"** instead of "what would I write from scratch?" — by requiring an
`inspect` pass over the codebase before any "build tool X" proposal.

Status: **Option A specified + chosen for implementation 2026-06-19.** Option B
held in reserve.

## Why

The self-extension capability-gap mechanism (Phase 2a) fired during an IB-style
financial-analysis chat and proposed wrapping **FMP** and building **`pdf-grep`**
from scratch — both wrong in the same way:

- FMP duplicated what the existing Alpha Vantage account/key already covers.
- `pdf-grep` duplicated `src/chat/code_subagent.py`, which already implements
  the list/read/grep/geofence architecture a document inspector needs.

In both cases the proposal reasoned from Jill's **tool surface** (the named
tools she can already call) and invented something new, rather than from the
**codebase** (what's already built that she could extend or reuse). The
financial-tools work then repeated the same miss at a smaller scale: the plan
chose `pdfplumber` without noticing `pymupdf` was already a dependency. The
failure mode is recursive — which is what makes a structural fix worthwhile.

## Where it hooks (existing code)

The pieces already exist; this is mostly a prompt change:

- **Gap detection** — `src/chat/reflection.py:363-370` extracts a
  `capability_gap` per turn → `src/chat/concerns.py:808` `_record_capability_gap`
  appends it to the self-extension concern's `wip` and evidence-bumps activation.
- **Proposal generation** — the concern fires on tick and runs its instruction
  (`scenarios/jill-chat.yaml:215-227`) through the **standard ReAct loop** via
  `_process_user_turn(..., autonomous=True)` (`src/chat/chat_loop.py:1463-1487`).
  Output is a conversational proposal tagged `kind: 'capability_proposal'` in
  `autonomy.jsonl`. **There is no separate proposal code — the instruction IS
  the generator.**
- **`inspect` is already wired into that loop** — `src/chat/react.py:462` →
  `_run_inspect` (`react.py:636`) → `code_subagent.inspect(query, repo_root=src/,
  llm_backend=self.backend, trace_dir=...)`.

## Scope

**In scope:** the deliberate proposal path — the fired self-extension concern
(the `capability_proposal` stream).

**Out of scope:** spontaneous suggestions Jill makes mid-conversation. Ordinary
chat can't be hard-gated, and those aren't "proposals" in the system's sense.
The fix there is ambient (inspect availability + general prompting), not a gate.
Stating this bounds what the change can promise.

## The geofence insight (design-review payoff)

`inspect` is geofenced to `src/` (`react.py:638`, `repo_root=Path(_SRC_DIR)`):

- It **will** catch architecture reuse (`code_subagent.py`, `stock-price`'s AV
  key) — those live under `src/`.
- It **will not** read `requirements.txt` (repo root, outside the fence), so the
  `pymupdf`-style "this dependency is already installed" miss isn't caught by
  reading the manifest.

Mitigation falls out of the query shape: target **existing import/usage sites in
`src/`**, not the manifest. `grep "import pymupdf"` inside `src/` finds
`fetch-text/tool.py:16` — so inspect surfaces dependency reuse *as long as the
query asks "what do I already use/import for X,"* not "what's in my
dependencies." This is baked into the prompt below.

## Option A — instruction-level gate (chosen)

No new code. Amend `scenarios/jill-chat.yaml:215-227` so the procedure
interleaves a mandatory inspect step before proposing:

1. Review WIP gaps + recent context.
2. For each real, recurring, un-proposed gap: **FIRST** `inspect` the codebase
   for something to reuse/extend, phrased as "what do I already import or
   implement under src/tools|chat|utils that does something like <gap>, or could
   be extended?" Read the answer.
3. **THEN** describe to the user: (1) the gap; (2) what inspect found — the
   closest existing capability with its file path, and whether to EXTEND it or,
   only if nothing fits, BUILD new; (3) the proposed tool — name/input/output;
   (4) why it's worth it.
4. If after inspecting nothing un-proposed stands out, stay silent.

**Why this is the right first move:** behavior here is already instruction-driven,
not hardcoded; this encodes the lesson at the exact decision point with zero new
code path, fully reversible, fitting the prototype framing. The inspect tool,
loop, geofence, and traces all already exist — pure reuse.

**Weakness:** it's a *soft* gate — the model can skip or perfunctorily perform
the inspect step. Acceptable for a prototype *if* we can observe whether it
happens (see Verification).

### Anti-bias guardrail

The gate must change the **starting point**, not forbid new tools. A reuse-check
that suppresses genuinely-novel tooling is its own failure. The prompt prefers
extend when a real candidate exists; building new is fine when nothing fits —
**but you must have checked and say so.** The deliverable is an explicit
extend-vs-build decision with evidence, not a bias toward "always extend."

## Verification / observability

Two existing artifacts make the gate auditable:

- `inspect` writes a trace per call → `<memory>/inspect_traces/inspect_*.txt`.
- Each proposal fire logs `kind: 'capability_proposal'` → `autonomy.jsonl`.

**Success criterion:** every `capability_proposal` event has (a) a matching
inspect trace within the same fire window, and (b) proposal text naming a
concrete existing capability + an explicit extend-vs-build call. Audit the first
handful of fires by hand. Low skip-rate → Option A suffices. Routine skipping →
escalate to Option B.

## Option B — code-enforced gate (reserve)

If audits show skipping, make the inspect pass structural, mirroring the
existing `_triage_fire_candidate` background-call pattern
(`src/chat/concerns.py:1142`). In the self-extension branch of `_handle_tick`
(`chat_loop.py` ~1463-1487), **before** building `wrapped_instruction`, run
`code_subagent.inspect(...)` for the candidate gap(s) and inject the findings
into `wip_section` as a "Reuse-check findings" block. The evidence is then
provably present even if the model wouldn't have called the tool itself.

**Cost (why it's not the default):** each inspect is a ~12-iteration subagent
loop; one per gap per fire is real latency/tokens in the autonomy tick, plus
gap-set selection and query construction move into code. Defer until Option A's
data justifies it.

## Risks & open questions

- **ReAct iteration budget** — interleaving inspect consumes loop iterations;
  several accumulated gaps could crowd the autonomous-fire budget. Confirm
  inspect-then-respond fits the autonomous path's max-iters.
- **Multi-gap fires** — a weekly fire may review several WIP gaps. Per-gap
  inspect is ideal but costs iterations; the prompt says "for each that stands
  out," letting the model triage to the strongest first. Acceptable for v1.
- **Geofence** — pure manifest-level dependency reuse is only caught indirectly
  (via import sites in `src/`). Manifest-aware checks would need a wider
  geofence or a dedicated query — a separate, larger change, not this prototype.

## Recommendation

Ship Option A (instruction amendment), instrument with the audit above, hold
Option B in reserve. One-file prompt change, reuses the entire inspect stack,
directly targets the failure mode — the smallest thing that could work.
