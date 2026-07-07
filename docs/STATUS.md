# docs/ Status Index

Classification of every doc against the live codebase as of **2026-06-09**
(superseded set deleted **2026-06-11**; financial-tools / voice-sensor /
capability-gap-gating docs registered **2026-06-19**).

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
- **SUPERSEDED** — described deleted or replaced machinery; these docs were removed 2026-06-11 (recover via git history)
- **REFERENCE** — essay / analysis / background, not a spec of system behavior

## LIVE

| Doc | Covers |
|---|---|
| [RESOURCE_BROWSER.md](RESOURCE_BROWSER.md) | Standalone web resource browser (`src/resource_browser.py`, port 3001) |
| [REMOTE_VIEWER_DESIGN.md](REMOTE_VIEWER_DESIGN.md) | Remote viewing via `mirror.sh` (affect/canvas/CLI tunneling) |
| [character_evaluator.md](character_evaluator.md) | Orientation pass (`src/character_evaluator.py`), integrated per turn |
| [cognitive_graph_explorer.md](cognitive_graph_explorer.md) | CognitiveGraph visualization via resource browser |
| [cognitive_graph_spec.md](cognitive_graph_spec.md) | CognitiveGraph data structure (`src/cognitive_graph.py`) |
| [commands.md](commands.md) | Chat-CLI slash commands (rewritten 2026-06-11 against `src/cli.py`; task/goal-era commands removed) |
| [concerns-architecture.md](concerns-architecture.md) | Concern layers + dynamics; triage step implemented 2026-06 |
| [configuration.md](configuration.md) | Scenario YAML / LLM / tool configuration |
| [cw-voice-sensor.md](cw-voice-sensor.md) | CW-side ChatterBot voice sensor (mic→turn): `src/chat/voice_sensor.py`, `src/utils/voice_pipeline.py` — shipped 2026-06-19 |
| [getting-started.md](getting-started.md) | Install, venv, credentials |
| [musing_spec.md](musing_spec.md) | Musing LoRA-adapter biasing (`src/musing/`) |
| [self-awareness-benchmarks.md](self-awareness-benchmarks.md) | Four-tier self-awareness bench (`bench/introspective_fidelity/`) |
| [sensor_spec.md](sensor_spec.md) | Sensor system (`src/sensors/`, `sensor_runner.py`, SKILL.md metadata) |
| [sensor_spec_change_order_1.md](sensor_spec_change_order_1.md) | Sensor disposition field (inform/trigger) |
| [spec-seed-agent-concerns.md](spec-seed-agent-concerns.md) | Seed concerns from scenario YAML (`_seed_concerns_from_config`) |
| [substack_awareness_evaluation.md](substack_awareness_evaluation.md) | Self-awareness bench writeup (implemented suite) |
| [substack_sensors_vs_tools.md](substack_sensors_vs_tools.md) | Tools-pull vs sensors-push essay; matches live architecture |
| [trace_grounded_introspectionv3.md](trace_grounded_introspectionv3.md) | Trace-as-input introspection; reasoning_history injection is live |
| [ui-guide.md](ui-guide.md) | The four web UI surfaces and their ports |
| [user_concern_model.md](user_concern_model.md) | User-concern single-patch update model (largely implemented) |

## ASPIRATIONAL

| Doc | Covers |
|---|---|
| [METHOD_TOOLS.md](METHOD_TOOLS.md) | Protocol-based multi-step "method tools" — not implemented |
| [audio-out-design.md](audio-out-design.md) | ChatterBot audio-out ("say"); Pi player built/bench-verified, CW binding + ElevenLabs live test pending |
| [capability-gap-reuse-gating.md](capability-gap-reuse-gating.md) | Reuse-gating self-extension proposals via an `inspect` pass; Option A shipped but found insufficient live 2026-06-19 — under review (author skeptical it's worth the machinery) |
| [fire-outcome-capture.md](fire-outcome-capture.md) | Signed per-ledger outcome capture for autonomous fires (reflection stage 6 + pending registry) — drafted 2026-07-06, not implemented; serves the autonomy-bench gap and the modular-memory bridge |
| [conversation_goal_wrapper_spec.md](conversation_goal_wrapper_spec.md) | Conversational context wrapper draft |
| [cw-voice-sensor-plan.md](cw-voice-sensor-plan.md) | Build plan behind the shipped voice sensor (cross-repo: Pi mic path + CW consumer) |
| [financial-analysis-tools-plan.md](financial-analysis-tools-plan.md) | AV `get_financial_statements` tool + PDF→markdown helper LIVE 2026-06-19; document-inspection subagent deferred |
| [jill-integration.md](jill-integration.md) | Jill↔ChatterBot head binding design; voice sensor is the first piece live, rest design |
| [knowledge-base-system.md](knowledge-base-system.md) | Obsidian-backed three-tier knowledge base; obsidian tool exists, full KB system doesn't |
| [load-slice-spec.md](load-slice-spec.md) | Unified load-with-slicing primitive |
| [plan-review-eval-taskset.md](plan-review-eval-taskset.md) | 18-goal review eval protocol (machinery it tested is gone; methodology reusable) |
| [retrieval_envisionment_spec.md](retrieval_envisionment_spec.md) | HyDE-style retrieval envisionments — not implemented |
| [semantic_operators_spec.md](semantic_operators_spec.md) | extract/synthesize operator split — not implemented |
| [tools-and-primitives.md](tools-and-primitives.md) | Primitive/tool catalog spec; implementation status varies |

## SUPERSEDED — deleted 2026-06-11

27 banner-stamped docs describing deleted machinery were removed from the
tree (recover any via `git log --diff-filter=D -- docs/<name>`).

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
| [METHOD_TOOLS_TWEETS.md](METHOD_TOOLS_TWEETS.md) | Thread outline for the method-tools idea |
| [design_note_agreements_rag.md](design_note_agreements_rag.md) | Discourse/agreements design; write-side (triage+CRUD + date-stamp aging) implemented 2026-06-11, read-side RAG-push deferred |
| [design_note_threads.md](design_note_threads.md) | Threads design discussion (threads shipped 2026-05; see `src/chat/`) |
| [introspective_fidelity_benchmark_v01.md](introspective_fidelity_benchmark_v01.md) | Introspective-fidelity bench methodology |
| [journey_note_trace_qualia_self_model.md](journey_note_trace_qualia_self_model.md) | Audit-record essay: trace/qualia self-model |
| [journey_note_trace_qualia_self_model_v2.md](journey_note_trace_qualia_self_model_v2.md) | v2 of the above |
| [metadata-is-a-relation.md](metadata-is-a-relation.md) | Data-model essay (note: metadata is implemented as nested properties, not relations) |
| [what-agents-care-about.md](what-agents-care-about.md) | Concerns-not-tasks essay; the system's motivational thesis |
