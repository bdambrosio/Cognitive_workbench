# docs/ Status Index

Classification of every doc against the live codebase. Full review
**2026-06-09** (superseded set deleted 2026-06-11); second full review
**2026-07-19** (four entrypoint docs rewritten, drifted docs given
status banners, second superseded set deleted — list below); third pass
**2026-08-08** (essay/draft cleanup, third superseded set deleted, index
reconciled against the tree).

Context: the OODA executive, incremental planner (Stage 0–3, envisioning,
vision eval), GoalScheduler, and task-execution machinery were deleted
2026-05-02 (commit 65ef489d). The live runtime is the ChatLoop ReAct path
(`src/chat/chat_loop.py`): ReAct tool loop, concerns + autonomy (with
fire-time triage and per-concern WIP as of 2026-06), infospace
Notes/Collections, SKILL.md tools and sensors, resource browser / affect /
canvas displays.

## What this system is, as a coordinate

`Diving into Reliable Self-Evolving Agents: A Survey` (ICLR submission
53157) classifies a self-evolving agent by the deepest part of its
retained state that a change actually alters *and* persists into later
independent tasks. Retained state is X = (θ trainable parameters, σ
scaffold, U improver, C criterion), giving L0 output, L1 model, L2
scaffold, L3 improver, L4 criterion. Classification follows the changed
object, not the algorithm that changed it.

CW is **L2 with a human improver**. Three of the five levels are
deliberate non-goals rather than unbuilt features, and saying so is the
point of recording this here:

- **L0** — the ReAct turn. Trajectory and output change, nothing is
  retained. Most of what an observer watches Jill do is L0.
- **L1** — empty. Nothing here trains θ. `learned-disposition-design.md`
  would be the first, and is shadow-logging only.
- **L2** — where CW lives: concerns, memories, threads, WIP, the tool
  catalog, sensors, and the ReAct harness itself. All persist and change
  later behaviour. Note that Jill's memory writes are an L2 loop that
  runs unattended.
- **L3** — deliberately absent. Jill proposes tools; a human judges,
  builds, tests, installs. The procedure that produces tools is not
  itself modified, and the survey is explicit that a new artifact alone
  is not an L3 transition. `jill-self-extension.md` Phase 2b is what
  crosses this line.
- **L4** — deliberately absent, and load-bearing. The criterion (frozen
  composite bench, ship gate, Bruce's judgement) sits outside the update
  boundary. The survey's central reliability claim is that an
  improvement claim is credible only when the evidence *and the decision
  rule* remain outside the control of the update they judge.

## Categories

- **LIVE** — accurately describes current code/behavior
- **ASPIRATIONAL** — design for something not (or only partially) built; not invalidated by the deletions
- **SUPERSEDED** — described deleted or replaced machinery; these docs were removed (recover via git history)
- **REFERENCE** — essay / analysis / background / historical plan, not a spec of current system behavior

## LIVE

| Doc | Covers |
|---|---|
| [RESOURCE_BROWSER.md](RESOURCE_BROWSER.md) | Standalone web resource browser (`src/resource_browser.py`, port 3001); banner 2026-07-19 — now read-write (edit/delete/concern management), Graph tab non-functional |
| [REMOTE_VIEWER_DESIGN.md](REMOTE_VIEWER_DESIGN.md) | Remote viewing via `mirror.sh` (affect/canvas/CLI tunneling); runbook verified 2026-07-19 |
| [audio-out-design.md](audio-out-design.md) | ChatterBot audio-out ("say") — CW binding SHIPPED (`zenoh_io._publish_say` → `voice_pipeline.synthesize` → `chatter/audio/out`) |
| [character_evaluator.md](character_evaluator.md) | Orientation pass (`src/character_evaluator.py`); banner 2026-07-19 — module accurate, integration section stale (live hook is `src/chat/prompts.py`, config key `orientation.enabled`) |
| [commands.md](commands.md) | Chat-CLI slash commands; re-verified against `src/cli.py` 2026-07-19, fully accurate |
| [concerns-architecture.md](concerns-architecture.md) | Concern layers + dynamics — REWRITTEN 2026-07-19 against `src/chat/concerns.py` (two-layer strength/activation model, triage, WIP, yield, fire-outcome capture) |
| [configuration.md](configuration.md) | Scenario YAML / LLM / tool configuration — REWRITTEN 2026-07-19 against `jill-chat.yaml` + `launcher.py` |
| [cw-voice-sensor.md](cw-voice-sensor.md) | CW-side ChatterBot voice sensor (mic→turn): `src/chat/voice_sensor.py`, `src/utils/voice_pipeline.py` — shipped 2026-06-19 |
| [getting-started.md](getting-started.md) | Install, venv, credentials — REWRITTEN 2026-07-19 (previous version described the deleted OODA UI/scenarios) |
| [factorio-bridge-architecture.md](factorio-bridge-architecture.md) | Factorio game-embodiment bridge (separate subproject, `factorio/`); all build steps 1–5 shipped 2026-07-13/14 (server + fle-bridge mod, bridge process, fac-* tools, telemetry sensor); embodiment merged into jill-chat 2026-07-14; operational state in `factorio/README.md` |
| [game-embodiment-assessment.md](game-embodiment-assessment.md) | Factorio-vs-Satisfactory assessment + v1 plan (role, controls, success criteria); **v1 complete — both success criteria passed** (cooperative session 2026-07-15, CLI grounding 2026-07-17); results narrative in cohabitation-writeup.md |
| [harness-m0-m1-status.md](harness-m0-m1-status.md) | M0 complete (baseline 0.720 ± 0.007 frozen 2026-07-09, ship gate live), M1 collection running; next steps + session commit trail |
| [m1-collection-inputs.md](m1-collection-inputs.md) | Exemplar user inputs for M1 fire-outcome collection: reaction forms per outcome, latency spread, anti-patterns, cadence |
| [justification-taxonomy.md](justification-taxonomy.md) | Closed claim taxonomy (volatility, inference type) + deterministic reduction to ordinal grades; the read path is the `justify` built-in |
| [self-awareness-benchmarks.md](self-awareness-benchmarks.md) | Four-tier self-awareness bench (`bench/introspective_fidelity/`); verified 2026-07-19 |
| [sentinel-setup.md](sentinel-setup.md) | Sentinel desktop-security character: Tier 0 unprivileged probes, Tier 2 argument-exact sudoers fence, `adm`-group prerequisite; sudoers installed + verified live 2026-08-08. Ubuntu/systemd-specific (ufw + AppArmor) |
| [sensor_spec.md](sensor_spec.md) | Sensor system (`src/sensors/`, `sensor_runner.py`, SKILL.md metadata); banner 2026-07-19 — machinery live, OODA framing + launcher-integration section stale |
| [sensor_spec_change_order_1.md](sensor_spec_change_order_1.md) | Sensor disposition field; banner 2026-07-19 — data contract live, `trigger`/`alert` semantics inert (only `inform` meaningful) |
| [shared-world.md](shared-world.md) | Shared walkable 3D world behind `--world` (`src/world/`): terrain, authoritative server + ports, `world-*` tools, world-presence sensor, and the creature avatars (crow / kitten / owl) with their preview harness. Written 2026-08-10 against the code |
| [substack_awareness_evaluation.md](substack_awareness_evaluation.md) | Self-awareness bench writeup (implemented suite) |
| [substack_sensors_vs_tools.md](substack_sensors_vs_tools.md) | Tools-pull vs sensors-push essay; matches live architecture |
| [trace_grounded_introspectionv3.md](trace_grounded_introspectionv3.md) | Trace-as-input introspection; reasoning_history injection verified live 2026-07-19 |
| [ui-guide.md](ui-guide.md) | The live UI surfaces (CLI, resource browser, affect, canvas, telegram) — REWRITTEN 2026-07-19 (previous version described the deleted OODA UI + nonexistent port-3002 manager) |

## ASPIRATIONAL

| Doc | Covers |
|---|---|
| [capability-gap-reuse-gating.md](capability-gap-reuse-gating.md) | Reuse-gating self-extension proposals via an `inspect` pass; Option A shipped but found insufficient live 2026-06-19 — under review (author skeptical it's worth the machinery; superseded in intent by harness-roadmap M4) |
| [cognitive_graph_spec.md](cognitive_graph_spec.md) | CognitiveGraph data structure — store built (`src/cognitive_graph.py`) but OODA-era integration removed; unpopulated/dormant in the live runtime (banner 2026-07-19). Retained: reasoning-graph revival under consideration 2026-07-27 |
| [fire-outcome-capture.md](fire-outcome-capture.md) | Signed per-ledger outcome capture for autonomous fires — Phase 1 (capture, reflection stage 6) implemented 2026-07 (commit 8a2bacf7); Phase 2 (outcome-modulated dynamics) gated on data |
| [harness-roadmap.md](harness-roadmap.md) | Measurement-gated improvement loop (M0 frozen composite bench → fire-outcome data → weakness-mining cycles → re-grounded self-extension) — adopted 2026-07-08; M0 complete, M1 running (see harness-m0-m1-status.md); M2–M5 unbuilt |
| [jill-integration.md](jill-integration.md) | Jill↔ChatterBot head binding design; voice sensor + say path + head/camera tools live, rest design |
| [jill-self-extension.md](jill-self-extension.md) | Recursive tool construction; Phase 2a (judgment/propose-only) SHIPPED 2026-06 — capability-gap capture → self-extension concern; Phase 1/2b (generative author-test-restart path) unbuilt |
| [knowledge-base-system.md](knowledge-base-system.md) | Obsidian-backed three-tier knowledge base; obsidian tool (read/search/write→Notes) exists, decomposition/curation system doesn't |
| [learned-disposition-design.md](learned-disposition-design.md) | Tiny-LM state→value learning over fire decisions (RL on imagined trajectories, real judged outcomes as anchor); G1 offline anchor passed 2026-07-24 (`bench/disposition/`), build-order step 1 — state capture + render — SHIPPED 2026-07-25 (`src/chat/disposition.py`, shadow-log only); scorer, imagination gate, and triage coupling unbuilt. Descends from substack-gut-feeling-draft.md |
| [provenance-verifiability.md](provenance-verifiability.md) | Staged verifiability toward "justify your response" (claim-graph, no invented numerics); Levels 1–2 LIVE-VALIDATED; epistemic grader v1 SHIPPED 2026-08-04 (`f30d3f05`/`ad9bbda5`: verbatim-quote checks, [justification-taxonomy.md](justification-taxonomy.md) tags, ordinal grades + weakest-link + audit notes in `justify`) — refute/confirm/quiet branches all live-validated (SpaceX turns 2231/2236, Canberra 2239); Stage 5 background verification of suspect replies SHIPPED `8c1396e0` and LIVE-VALIDATED 2026-08-06 (confirm/silent turn 2247; unprompted correction turn 2295 — epistemic downgrade posted ~12 min post-reply); autonomous-turn claims + probe bench next; Levels 3–4 unbuilt |

## SUPERSEDED — deleted 2026-06-11, 2026-07-19 and 2026-08-08

Banner-stamped docs describing deleted machinery are removed from the
tree (recover any via `git log --diff-filter=D -- docs/<name>`).

**2026-08-08 set**: cognitive_graph_explorer.md (documented the Resource
Browser Graph tab, which queries the executive node deleted 2026-05-02
and renders nothing; its node vocabulary — triage nominations, spawned
tasks/goals, OODA assessments — was produced only by the deleted
pipeline), journey_note_trace_qualia_self_model.md (v1, explicitly
retracted by its own v2, which narrows the qualia claim to the
deflationary small-q reading and names the conflation as v1's mistake).
Also removed the same day, outside this index: three essay/draft files
and the justification substack drafts.

**2026-07-19 set** (all described the deleted OODA/planner/executor era,
verified against code before removal): METHOD_TOOLS.md,
METHOD_TOOLS_TWEETS.md, load-slice-spec.md,
conversation_goal_wrapper_spec.md, retrieval_envisionment_spec.md,
semantic_operators_spec.md, plan-review-eval-taskset.md,
tools-and-primitives.md, spec-seed-agent-concerns.md (live seeding is
`_seed_concerns_from_config`, a different model — see
concerns-architecture.md), text_input_state_machine.html,
musing_spec.md (deleted along with the orphaned `src/musing/`
experimental subsystem — unimported, planner integration target gone).

**2026-06-11 set** (27 docs):

OODA-executive / incremental-planner / task-pipeline era:
architecture.md, incremental_planner_article.md, TASK_EXECUTION_LOOP.md,
TASK_EXECUTION_SPEC.md, task_mechanism_spec.md, goals-and-scheduling.md,
phase_aware_execution_spec_v2.md, phase_aware_execution_spec_v3.md,
envisioning-spec.md, envisioning-and-quality-control.md,
mixed-initiative-plan-review.md, spec-agent-self-awareness.md,
STAGE0_RESOURCE_RETRIEVAL_DESIGN.md, TOOL_DEVELOPMENT_GUIDE.md,
TOOL_ARGUMENT_CONVENTION_ANALYSIS.md, MINECRAFT_UPDATES_2026-01.md

infospace_executor / map_node era:
AGENT_LOCATION_ANALYSIS.md, CONVERSATION_LOCKS_README.md,
FS_FILESYSTEM_MAPPING_REVIEW.md, IMPLEMENTATION_STATUS.md,
INFORMATION_TO_NOTE_MIGRATION.md, INFOSPACE_IMPLEMENTATION_SUMMARY.md,
INFOSPACE_INFO_REFACTOR.md, INFOSPACE_PRIMITIVES_UPDATE.md,
INFOSPACE_REVIEW_FIXES.md, NOTE_COLLECTION_EXTENSION_REVIEW.md,
SPACEMAP_ARCHITECTURE.md

## REFERENCE

| Doc | Covers |
|---|---|
| [cohabitation-writeup.md](cohabitation-writeup.md) | DRAFT essay: five days living with the Factorio agent — records both v1 criteria passed, the perception-layer lessons, FLE upstream issues |
| [cw-voice-sensor-plan.md](cw-voice-sensor-plan.md) | Historical build plan behind the shipped voice sensor (implemented 2026-06-19; retained for planning rationale) |
| [design_note_agreements_rag.md](design_note_agreements_rag.md) | Discourse/agreements design; write-side (triage+CRUD + date-stamp aging) implemented 2026-06-11, read-side RAG-push deferred |
| [design_note_threads.md](design_note_threads.md) | Threads design discussion (threads shipped 2026-05; see `src/chat/`) |
| [financial-analysis-tools-plan.md](financial-analysis-tools-plan.md) | Historical plan behind `get-financial-statements` (shipped 2026-06-19); PDF half redirected to code_subagent |
| [introspective_fidelity_benchmark_v01.md](introspective_fidelity_benchmark_v01.md) | Introspective-fidelity bench methodology |
| [journey_note_trace_qualia_self_model_v2.md](journey_note_trace_qualia_self_model_v2.md) | Audit-record essay: trace/qualia self-model. v1 deleted 2026-08-08 — this revision retracts its framing, so read this one only |
| [justification-technical-note.md](justification-technical-note.md) | Technical note (2026-08) on auditable justification: the failure that motivated it, and what a justification record has to be to count as one. Companion to the live spec in provenance-verifiability.md |
| [metadata-is-a-relation.md](metadata-is-a-relation.md) | Data-model essay (note: metadata is implemented as nested properties, not relations) |
| [substack-gut-feeling-draft.md](substack-gut-feeling-draft.md) | DRAFT essay: gut feelings as index-free memory (valence compression) — the origin idea behind learned-disposition-design.md |
| [user_concern_model.md](user_concern_model.md) | Historical user-concern design; the shipped model differs (per-turn decay/bump + reflection, not patch ops — see banner + concerns-architecture.md) |
| [what-agents-care-about.md](what-agents-care-about.md) | Concerns-not-tasks essay; the system's motivational thesis |
