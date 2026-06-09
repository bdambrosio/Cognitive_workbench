# docs/ Status Index

Classification of every doc against the live codebase as of **2026-06-09**.

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
- **SUPERSEDED** — describes deleted or replaced machinery (these docs carry a status banner)
- **REFERENCE** — essay / analysis / background, not a spec of system behavior

## LIVE

| Doc | Covers |
|---|---|
| [RESOURCE_BROWSER.md](RESOURCE_BROWSER.md) | Standalone web resource browser (`src/resource_browser.py`, port 3001) |
| [REMOTE_VIEWER_DESIGN.md](REMOTE_VIEWER_DESIGN.md) | Remote viewing via `mirror.sh` (affect/canvas/CLI tunneling) |
| [character_evaluator.md](character_evaluator.md) | Orientation pass (`src/character_evaluator.py`), integrated per turn |
| [cognitive_graph_explorer.md](cognitive_graph_explorer.md) | CognitiveGraph visualization via resource browser |
| [cognitive_graph_spec.md](cognitive_graph_spec.md) | CognitiveGraph data structure (`src/cognitive_graph.py`) |
| [commands.md](commands.md) | CLI / browser commands for concerns, goals, system control |
| [concerns-architecture.md](concerns-architecture.md) | Concern layers + dynamics; triage step implemented 2026-06 |
| [configuration.md](configuration.md) | Scenario YAML / LLM / tool configuration |
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
| [conversation_goal_wrapper_spec.md](conversation_goal_wrapper_spec.md) | Conversational context wrapper draft |
| [knowledge-base-system.md](knowledge-base-system.md) | Obsidian-backed three-tier knowledge base; obsidian tool exists, full KB system doesn't |
| [load-slice-spec.md](load-slice-spec.md) | Unified load-with-slicing primitive |
| [plan-review-eval-taskset.md](plan-review-eval-taskset.md) | 18-goal review eval protocol (machinery it tested is gone; methodology reusable) |
| [retrieval_envisionment_spec.md](retrieval_envisionment_spec.md) | HyDE-style retrieval envisionments — not implemented |
| [semantic_operators_spec.md](semantic_operators_spec.md) | extract/synthesize operator split — not implemented |
| [tools-and-primitives.md](tools-and-primitives.md) | Primitive/tool catalog spec; implementation status varies |

## SUPERSEDED (banner in each file)

OODA-executive / incremental-planner / task-pipeline era:

| Doc | Covered (then) |
|---|---|
| [architecture.md](architecture.md) | System architecture centered on the executive node + planner stages |
| [incremental_planner_article.md](incremental_planner_article.md) | The incremental planner deep-dive (Stages 0–3, SGLang slots) |
| [TASK_EXECUTION_LOOP.md](TASK_EXECUTION_LOOP.md) | Outer task loop + inner planner pipeline |
| [TASK_EXECUTION_SPEC.md](TASK_EXECUTION_SPEC.md) | Task lifecycle (proposed→establishing→active→cooldown) |
| [task_mechanism_spec.md](task_mechanism_spec.md) | Persistent task entities + milestone loop |
| [goals-and-scheduling.md](goals-and-scheduling.md) | Goal submission + GoalScheduler modes |
| [phase_aware_execution_spec_v2.md](phase_aware_execution_spec_v2.md) | Phase-aware planning (pipeline vs deliberative) |
| [phase_aware_execution_spec_v3.md](phase_aware_execution_spec_v3.md) | Phase-aware execution, trace-informed revision |
| [envisioning-spec.md](envisioning-spec.md) | Vision criteria generation + evaluation |
| [envisioning-and-quality-control.md](envisioning-and-quality-control.md) | Shallow/deep vision checks in the planner loop |
| [mixed-initiative-plan-review.md](mixed-initiative-plan-review.md) | Offline plan review + learning injection |
| [spec-agent-self-awareness.md](spec-agent-self-awareness.md) | Self-model injection into the planner snapshot |
| [STAGE0_RESOURCE_RETRIEVAL_DESIGN.md](STAGE0_RESOURCE_RETRIEVAL_DESIGN.md) | Stage 0 retrieval for the planner |
| [TOOL_DEVELOPMENT_GUIDE.md](TOOL_DEVELOPMENT_GUIDE.md) | Planner-facing `tool()` convention (live convention: SKILL.md + `react_invoke`) |
| [TOOL_ARGUMENT_CONVENTION_ANALYSIS.md](TOOL_ARGUMENT_CONVENTION_ANALYSIS.md) | value/target argument conventions for the executor |
| [MINECRAFT_UPDATES_2026-01.md](MINECRAFT_UPDATES_2026-01.md) | Executive-era Minecraft world-tools work |

infospace_executor / map_node era:

| Doc | Covered (then) |
|---|---|
| [AGENT_LOCATION_ANALYSIS.md](AGENT_LOCATION_ANALYSIS.md) | map.py → map_agent.py refactor analysis |
| [CONVERSATION_LOCKS_README.md](CONVERSATION_LOCKS_README.md) | Conversation locks in map_node/executive_node |
| [FS_FILESYSTEM_MAPPING_REVIEW.md](FS_FILESYSTEM_MAPPING_REVIEW.md) | fs-* tools + executor integration |
| [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) | SpaceMap implementation tracker |
| [INFORMATION_TO_NOTE_MIGRATION.md](INFORMATION_TO_NOTE_MIGRATION.md) | Information→Note rename in deleted modules |
| [INFOSPACE_IMPLEMENTATION_SUMMARY.md](INFOSPACE_IMPLEMENTATION_SUMMARY.md) | InfospaceMap + map_node integration |
| [INFOSPACE_INFO_REFACTOR.md](INFOSPACE_INFO_REFACTOR.md) | infospace_executor plan_bindings refactor |
| [INFOSPACE_PRIMITIVES_UPDATE.md](INFOSPACE_PRIMITIVES_UPDATE.md) | Executor primitive handlers |
| [INFOSPACE_REVIEW_FIXES.md](INFOSPACE_REVIEW_FIXES.md) | Executor bug-fix log |
| [NOTE_COLLECTION_EXTENSION_REVIEW.md](NOTE_COLLECTION_EXTENSION_REVIEW.md) | Note/Collection extension review against the executor |
| [SPACEMAP_ARCHITECTURE.md](SPACEMAP_ARCHITECTURE.md) | SpaceMap/InfospaceMap class hierarchy |

## REFERENCE

| Doc | Covers |
|---|---|
| [# When AI Agents Talk, Nobody's Listenin.md](<# When AI Agents Talk, Nobody's Listenin.md>) | Essay on agent-to-agent conversation |
| [METHOD_TOOLS_TWEETS.md](METHOD_TOOLS_TWEETS.md) | Thread outline for the method-tools idea |
| [design_note_agreements_rag.md](design_note_agreements_rag.md) | Discourse/agreements RAG design discussion |
| [design_note_threads.md](design_note_threads.md) | Threads design discussion (threads shipped 2026-05; see `src/chat/`) |
| [introspective_fidelity_benchmark_v01.md](introspective_fidelity_benchmark_v01.md) | Introspective-fidelity bench methodology |
| [journey_note_trace_qualia_self_model.md](journey_note_trace_qualia_self_model.md) | Audit-record essay: trace/qualia self-model |
| [journey_note_trace_qualia_self_model_v2.md](journey_note_trace_qualia_self_model_v2.md) | v2 of the above |
| [metadata-is-a-relation.md](metadata-is-a-relation.md) | Data-model essay (note: metadata is implemented as nested properties, not relations) |
| [what-agents-care-about.md](what-agents-care-about.md) | Concerns-not-tasks essay; the system's motivational thesis |
