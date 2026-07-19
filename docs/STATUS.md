# docs/ Status Index

Classification of every doc against the live codebase. Full review
**2026-06-09** (superseded set deleted 2026-06-11); second full review
**2026-07-19** (four entrypoint docs rewritten, drifted docs given
status banners, second superseded set deleted — list below).

Context: the OODA executive, incremental planner (Stage 0–3, envisioning,
vision eval), GoalScheduler, and task-execution machinery were deleted
2026-05-02 (commit 65ef489d). The live runtime is the ChatLoop ReAct path
(`src/chat/chat_loop.py`): ReAct tool loop, concerns + autonomy (with
fire-time triage and per-concern WIP as of 2026-06), infospace
Notes/Collections, SKILL.md tools and sensors, resource browser / affect /
canvas displays.

Categories:
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
| [self-awareness-benchmarks.md](self-awareness-benchmarks.md) | Four-tier self-awareness bench (`bench/introspective_fidelity/`); verified 2026-07-19 |
| [sensor_spec.md](sensor_spec.md) | Sensor system (`src/sensors/`, `sensor_runner.py`, SKILL.md metadata); banner 2026-07-19 — machinery live, OODA framing + launcher-integration section stale |
| [sensor_spec_change_order_1.md](sensor_spec_change_order_1.md) | Sensor disposition field; banner 2026-07-19 — data contract live, `trigger`/`alert` semantics inert (only `inform` meaningful) |
| [substack_awareness_evaluation.md](substack_awareness_evaluation.md) | Self-awareness bench writeup (implemented suite) |
| [substack_sensors_vs_tools.md](substack_sensors_vs_tools.md) | Tools-pull vs sensors-push essay; matches live architecture |
| [trace_grounded_introspectionv3.md](trace_grounded_introspectionv3.md) | Trace-as-input introspection; reasoning_history injection verified live 2026-07-19 |
| [ui-guide.md](ui-guide.md) | The live UI surfaces (CLI, resource browser, affect, canvas, telegram) — REWRITTEN 2026-07-19 (previous version described the deleted OODA UI + nonexistent port-3002 manager) |

## ASPIRATIONAL

| Doc | Covers |
|---|---|
| [capability-gap-reuse-gating.md](capability-gap-reuse-gating.md) | Reuse-gating self-extension proposals via an `inspect` pass; Option A shipped but found insufficient live 2026-06-19 — under review (author skeptical it's worth the machinery; superseded in intent by harness-roadmap M4) |
| [cognitive_graph_spec.md](cognitive_graph_spec.md) | CognitiveGraph data structure — store built (`src/cognitive_graph.py`) but OODA-era integration removed; unpopulated/dormant in the live runtime (banner 2026-07-19) |
| [cognitive_graph_explorer.md](cognitive_graph_explorer.md) | Graph tab visualization — non-functional under the live runtime (queries the deleted executive node; banner 2026-07-19) |
| [fire-outcome-capture.md](fire-outcome-capture.md) | Signed per-ledger outcome capture for autonomous fires — Phase 1 (capture, reflection stage 6) implemented 2026-07 (commit 8a2bacf7); Phase 2 (outcome-modulated dynamics) gated on data |
| [harness-roadmap.md](harness-roadmap.md) | Measurement-gated improvement loop (M0 frozen composite bench → fire-outcome data → weakness-mining cycles → re-grounded self-extension) — adopted 2026-07-08; M0 complete, M1 running (see harness-m0-m1-status.md); M2–M5 unbuilt |
| [jill-integration.md](jill-integration.md) | Jill↔ChatterBot head binding design; voice sensor + say path + head/camera tools live, rest design |
| [jill-self-extension.md](jill-self-extension.md) | Recursive tool construction; Phase 2a (judgment/propose-only) SHIPPED 2026-06 — capability-gap capture → self-extension concern; Phase 1/2b (generative author-test-restart path) unbuilt |
| [knowledge-base-system.md](knowledge-base-system.md) | Obsidian-backed three-tier knowledge base; obsidian tool (read/search/write→Notes) exists, decomposition/curation system doesn't |

## SUPERSEDED — deleted 2026-06-11 and 2026-07-19

Banner-stamped docs describing deleted machinery are removed from the
tree (recover any via `git log --diff-filter=D -- docs/<name>`).

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
| [# When AI Agents Talk, Nobody's Listenin.md](<# When AI Agents Talk, Nobody's Listenin.md>) | Essay on agent-to-agent conversation |
| [cohabitation-writeup.md](cohabitation-writeup.md) | DRAFT essay: five days living with the Factorio agent — records both v1 criteria passed, the perception-layer lessons, FLE upstream issues |
| [cw-voice-sensor-plan.md](cw-voice-sensor-plan.md) | Historical build plan behind the shipped voice sensor (implemented 2026-06-19; retained for planning rationale) |
| [design_note_agreements_rag.md](design_note_agreements_rag.md) | Discourse/agreements design; write-side (triage+CRUD + date-stamp aging) implemented 2026-06-11, read-side RAG-push deferred |
| [design_note_threads.md](design_note_threads.md) | Threads design discussion (threads shipped 2026-05; see `src/chat/`) |
| [financial-analysis-tools-plan.md](financial-analysis-tools-plan.md) | Historical plan behind `get-financial-statements` (shipped 2026-06-19); PDF half redirected to code_subagent |
| [introspective_fidelity_benchmark_v01.md](introspective_fidelity_benchmark_v01.md) | Introspective-fidelity bench methodology |
| [journey_note_trace_qualia_self_model.md](journey_note_trace_qualia_self_model.md) | Audit-record essay: trace/qualia self-model |
| [journey_note_trace_qualia_self_model_v2.md](journey_note_trace_qualia_self_model_v2.md) | v2 of the above |
| [metadata-is-a-relation.md](metadata-is-a-relation.md) | Data-model essay (note: metadata is implemented as nested properties, not relations) |
| [substack-gut-feeling-draft.md](substack-gut-feeling-draft.md) | DRAFT essay: gut feelings as index-free memory (valence compression) |
| [user_concern_model.md](user_concern_model.md) | Historical user-concern design; the shipped model differs (per-turn decay/bump + reflection, not patch ops — see banner + concerns-architecture.md) |
| [what-agents-care-about.md](what-agents-care-about.md) | Concerns-not-tasks essay; the system's motivational thesis |
