# Jill self-extension (recursive tool construction)

**Status: Phase 2a (judgment/propose-only) SHIPPED 2026-06.** Capability-gap
capture runs through reflection STAGE 5 (`src/chat/reflection.py`) →
`_record_capability_gap` (`src/chat/concerns.py`) → the `capability-gap`
self-extension concern in `scenarios/jill-chat.yaml`; fires are tagged for
scoring. Phase 1 / Phase 2b (the generative author-test-restart-gate
tool-writing path) remain unbuilt — for those phases this doc is still the
plan.

## Thesis

Jill already has *epistemic* tools — vision, web search, memory recall,
read-only introspection of her own source (`inspect`), and self-questioning.
Self-extension adds a *generative* one: when she repeatedly hits a task her
tools can't do, she designs, writes, and tests a **new tool** for it, then
proposes it for review. This is the highest-ceiling form of autonomy the
current substrate can reach — an agent that grows its own action space — and
the architecture is unusually close to supporting it.

The design separates two independent questions and de-risks them in order:

1. **Judgment** — does Jill notice *real, recurring* capability gaps and
   propose *sensible* tools? (Validated by text-only proposals; near-zero risk.)
2. **Generation** — can she write a tool that actually works? (Validated by the
   author/test machinery, isolated and human-gated.)

## What the substrate already provides

Self-extension is mostly *assembling parts that already exist*:

| Need | Already exists | Where |
|---|---|---|
| Read her own code to locate a limitation | `inspect` subagent — read-only, geofenced to `src/`, list/read/grep | `src/chat/tools.py` (`inspect` catalog entry) |
| Run code in isolation with a timeout + human gate | `exec-script` — `subprocess.run` in `scenarios/<world>/fs/`, 120s cap, optional ASK step | `src/tools/exec-script/tool.py` |
| Pattern for a constrained, persona-less subagent with typed primitives | `recall.py` is the canonical template (see README) — ReAct loop, geofenced inputs, per-call trace; `security.py` is the same shape with typed system probes and a wall-clock budget | `src/chat/subagents/recall.py` |
| Iterate-on-failure | ReAct loop + max-iters → successor concern | `src/chat/react.py`, `src/chat/concerns.py` |
| Per-event audit log | `autonomy.jsonl` (one record per fire/defer/successor) | concerns layer |
| Tool contract | `src/tools/<name>/` = `Skill.md` (frontmatter: `name`, `description`, `args`) + `tool.py` (`react_invoke(args, *, character_name, backend, logger) -> {status, text[, image]}`) | `src/chat/tools.py` (`_discover_tools`) |
| **Restart-as-gate** | `_discover_tools()` scans `src/tools/` exactly once at startup | `src/chat/tools.py` |

The genuinely new code is small: **one authoring subagent, one staging
convention, one reflection field, one seed concern.**

## Safety model (proportionate, not containerized)

The restart boundary is the centerpiece, not an obstacle. Because
`_discover_tools()` scans `src/tools/` only at startup, **Jill structurally
cannot make her own code execute in the running process.** That is a free
human-in-the-loop checkpoint. The model leans into it:

```
author -> static-check -> test in subprocess -> propose -> [HUMAN review + restart] -> live
```

Four proportionate layers, no Docker:

1. **Staging, not live.** Authored tools land in `src/tools_staging/<name>/`,
   *outside* the `src/tools/` scan root. Discovery never sees them (the scan is
   non-recursive — `os.listdir` of the top level only). A staged tool is a
   *proposal*, not a deployment.
2. **Subprocess test, never in-process import.** Testing a fresh `tool.py`
   means running its `react_invoke` — but importing it would execute
   module-level code inside Jill's interpreter. So the test runs in a
   **subprocess with a timeout** (reusing the `exec-script` `subprocess.run`
   pattern). Untrusted code stays out of the live process even during testing.
3. **Git working tree = blast-radius limit + undo.** Everything authored is
   uncommitted working-tree change under `src/tools_staging/`. It shows in
   `git status`/`git diff`; reverting is `git checkout`. The repo's own VCS is
   the pragmatic "not Docker" sandbox boundary, and the staging dir being
   tracked means review *is* the diff.
4. **Human promote + restart.** A staged tool goes live only when Bruce moves
   `src/tools_staging/<name>/` -> `src/tools/<name>/` and restarts. That is the
   approval handshake.

**Explicitly not doing:** static "dangerous-code" detection by banned-substring
grep. It is brittle, gives false confidence, and violates the project's
no-keyword-matching rule. The isolation + restart gate + human diff review are
the safety story; a keyword blocklist would only pretend to add to it.

## The loop, end to end

1. **Gap capture (lightest possible).** When Jill can't accomplish something
   mid-turn, she does *not* build inline (too heavy for a live turn). The
   post-turn reflection pass — already an LLM call — gains an optional
   `capability_gap` field: "if you needed a capability you don't have this turn,
   describe it in natural language." Gaps accrue as evidence on a standing
   self-extension concern. No new persistence; reuses the reflection schema and
   concern machinery.
2. **Trigger.** The self-extension seed concern fires on its (low-frequency)
   cadence. Fire-time triage decides whether there is a real, *recurring* gap
   worth acting on, or defers. (Reuses the 2026-06 triage step.)
3. **Design + author.** A new `build_tool` subagent (modeled on `security.py`)
   receives the gap as its query. Typed primitives: `read_existing_tool` (copy
   conventions), `write_staged_file`, `run_tool_test`, `respond`. It writes
   `Skill.md` + `tool.py` into `src/tools_staging/<name>/`.
4. **Static check.** `compile()` the `tool.py`; confirm `react_invoke` is
   defined; confirm the frontmatter parses; confirm the name does **not** collide
   with a live tool (no silent shadowing).
5. **Dynamic test.** The subagent synthesizes a sample arg set; `run_tool_test`
   calls `react_invoke` in a subprocess and checks for a well-formed
   `{status, text}`. On failure it iterates (the ReAct loop already does this);
   on max-iters it gives up cleanly and reports.
6. **Propose.** Write a human-readable proposal (gap -> what it does -> test
   transcript -> file list) plus an `autonomy.jsonl` event. Files remain staged.
7. **Human gate + restart.** Bruce reviews the diff, promotes, restarts. Next
   session, discovery registers the tool; Jill can confirm it is live.

## Components to build

- `src/chat/build_tool.py` — the authoring subagent (the one substantial new
  file; roughly `security.py`-sized). **Subagent, not straight-line** — authoring
  code reliably needs the test-failure retry loop.
- A `build_tool` entry in the ReAct catalog (`src/chat/tools.py`), wired like
  `security`/`inspect`. In-conversation invocation only for v1 (no slash command).
- `src/tools_staging/` convention + the subprocess test runner (small).
- An optional `capability_gap` field in the reflection schema (`reflection.py`).
- One seed concern in `jill-chat.yaml`.

Optional helper `scripts/promote_tool.py` (show diff, move, leave uncommitted) —
**skipped for v1**; `git mv` + restart is enough.

**Scope discipline:** v1 is **new tools only**. Modifying an *existing* tool is
the same pipeline with overwrite-on-promote, but it shadows working behavior and
deserves a harder look — deferred (Phase 3).

## Seed concern (draft wording)

> *"When I repeatedly encounter a task my current tools can't do, treat it as a
> standing interest: design, write, and test a new tool for it in isolation,
> then propose it for review. Build into staging only — never into my live
> toolkit. Don't propose a tool that duplicates one I already have. A capability
> I notice once is a note; one I hit repeatedly is worth building for."*

## Phasing and build order

The phases are independent enough that the cheapest, lowest-risk one ships
first. **Build order: 2a -> 1 -> 2b.**

### Phase 2a — autonomous gap detection, text-only proposal (SHIPPED 2026-06-14, uncommitted)

Near-pure config; zero code-generation risk. Validates the *judgment* path
alone. No staging, no `build_tool`, no subprocess — reuses concerns + triage +
the fire→ReAct→`respond` autonomous-reply path entirely.

What shipped:
- **Reflection STAGE 5 — `capability_gap`** (`reflection.py`): an optional
  one-sentence field. When the exchange evidences that Jill lacked a tool, it's
  routed (not to memories/concerns) to `_record_capability_gap`.
- **`_record_capability_gap`** (`concerns.py`): appends the gap to the
  self-extension concern's **WIP** (so it rides into the fire frame) and
  **evidence-bumps** its activation (so recurring gaps fire ahead of the weekly
  rhythm). No-op if the seed is absent. Repetition in WIP is *kept* — it's the
  recurrence signal the instruction looks for.
- **Seed concern** (`jill-chat.yaml`, `self_extension: true`, rhythm 168h) with
  a **propose-not-build** instruction. On fire the user sees, e.g.:

  > *"I've detected a gap: I keep converting timezones for you by hand. I'd like
  > to build a `convert-timezone` tool (in: time, from_tz, to_tz; out: the
  > converted time) — want me to?"*

- **`self_extension` flag** plumbed through the seed loader + property
  allowlist; `_self_extension_concern_id()` locates the concern.
- **Distinct autonomy events**: gap capture logs `event=capability_gap`;
  proposal fires are tagged `kind=capability_proposal` — both grep-able in
  `autonomy.jsonl` for `bench/autonomy_review` scoring.

**Dedup decision (resolved):** dedup is **WIP-based**, not a separate
declined-proposals note. The post-fire WIP summary records what Jill already
proposed, the fire frame injects WIP, and the instruction tells her not to
repeat. This is the same mechanism every concern uses to avoid redoing work —
zero new persistence. A dedicated *declined* record (user actively said "no")
has teeth only once proposals are actually accepted/declined, so it's deferred
to **2b**, where the accept/decline flow exists to populate it.

**Tests:** `tests/test_concern_dynamics.py` — gap→WIP+bump+event, no-op without
seed, empty-gap ignored (26 pass total).

**Success criteria (live):** over a few days of normal use with `--autonomy`,
the gaps Jill surfaces are real and recurring (not one-offs or hallucinated),
and the proposed tools are sensible and non-duplicative. Watch via
`bench/autonomy_review` (filter the new event/kind tags). NOT yet observed live.

### Phase 1 — `build_tool` subagent, manual (the generation core)

Independent of Phase 2a. Build the authoring subagent + staging + subprocess
test. Drive it manually in conversation ("Jill, build yourself a tool that does
X"). Verify: staged files are well-formed, the test transcript is honest, a
promoted tool goes live on restart, name-collisions are refused.

**Success criteria:** a manually requested tool is authored, tested green in
isolation, promoted, and dispatchable after restart — with a clean failure
report when the spec is infeasible.

### Phase 2b — connect detection to generation

Graduate the seed concern's fire instruction from "propose as text" to "invoke
`build_tool` -> stage + test -> propose with the test transcript." Still gated by
human promote + restart. This is the full loop.

### Phase 3 — deferred

- Self-*modification* of existing tools (same pipeline, overwrite-on-promote).
- The recursive case: Jill improving `build_tool` itself.

## Resolved decisions (2026-06-14)

- Staging location: **`src/tools_staging/`** (tracked, so review = git diff).
- Author as a **subagent** (iterates on test failure), not straight-line.
- Invocation: **in-conversation ReAct tool**, no slash command for v1.
- Phase 2's first cut: **text-only proposal** ("I have detected a gap... I
  propose a new tool...") before any build machinery.

## Open questions

- Sample-arg synthesis for `run_tool_test`: who writes the test call — the
  subagent from the `args` spec, or a fixed "smoke" convention per arg type?
  (Lean: subagent synthesizes from the spec.)
- **Resolved (2026-06-14):** gap dedup is WIP-based for 2a; a dedicated
  declined-proposals record is deferred to 2b. Phase 2a proposals *are* logged
  distinctly (`event=capability_gap`, `kind=capability_proposal`).
</content>
</invoke>
