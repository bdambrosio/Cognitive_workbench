# docs/ — where to look for what

A topical map of every document here. It answers *where do I read about X*.

It does **not** say whether a document is still true — that is
[STATUS.md](STATUS.md)'s job, which classifies all of these as LIVE,
ASPIRATIONAL, or REFERENCE against the current code and is re-reviewed
periodically. Check there before trusting a design doc as a description of
what runs today. Every file appears exactly once below.

For the project itself — what it is, why, and a quick start — see the
[README](../README.md) at the repo root.

## Start here

| Doc | |
|---|---|
| [getting-started.md](getting-started.md) | Install, venv, credentials |
| [configuration.md](configuration.md) | Scenario YAML, LLM and tool configuration |
| [commands.md](commands.md) | Chat-CLI slash commands |
| [ui-guide.md](ui-guide.md) | The live surfaces: CLI, resource browser, affect, canvas, Telegram |

## How the agent decides what to do

| Doc | |
|---|---|
| [what-agents-care-about.md](what-agents-care-about.md) | Concerns, not tasks — the motivational thesis |
| [concerns-architecture.md](concerns-architecture.md) | Concern layers and dynamics: strength/activation, triage, WIP, yield |
| [user_concern_model.md](user_concern_model.md) | The earlier user-concern design; shipped model differs |
| [fire-outcome-capture.md](fire-outcome-capture.md) | Per-ledger outcome capture for autonomous fires |
| [learned-disposition-design.md](learned-disposition-design.md) | Tiny-LM state→value learning over fire decisions |
| [character_evaluator.md](character_evaluator.md) | The orientation pass |
| [trace_grounded_introspectionv3.md](trace_grounded_introspectionv3.md) | Reasoning traces as turn input |

## Provenance and justification

| Doc | |
|---|---|
| [provenance-verifiability.md](provenance-verifiability.md) | Staged verifiability toward "justify your response" |
| [justification-taxonomy.md](justification-taxonomy.md) | Closed claim taxonomy and its reduction to ordinal grades |
| [justification-technical-note.md](justification-technical-note.md) | What a justification record has to be to count as one |
| [justify-resume-2026-08-09.md](justify-resume-2026-08-09.md) | Working notes from the justify hardening pass |

## Memory and data model

| Doc | |
|---|---|
| [design_note_threads.md](design_note_threads.md) | Conversation threads |
| [design_note_agreements_rag.md](design_note_agreements_rag.md) | Discourse and agreements; write side shipped, read side deferred |
| [knowledge-base-system.md](knowledge-base-system.md) | Obsidian-backed three-tier knowledge base |
| [cognitive_graph_spec.md](cognitive_graph_spec.md) | CognitiveGraph structure; store built, integration dormant |
| [metadata-is-a-relation.md](metadata-is-a-relation.md) | Data-model essay on metadata as relation |

## Sensors and tools

| Doc | |
|---|---|
| [sensor_spec.md](sensor_spec.md) | The sensor system: `src/sensors/`, runner, SKILL.md metadata |
| [sensor_spec_change_order_1.md](sensor_spec_change_order_1.md) | Sensor disposition field |
| [substack_sensors_vs_tools.md](substack_sensors_vs_tools.md) | Tools pull, sensors push — the distinction, as an essay |
| [financial-analysis-tools-plan.md](financial-analysis-tools-plan.md) | Plan behind the financial-statements tool |

## Self-extension

| Doc | |
|---|---|
| [jill-self-extension.md](jill-self-extension.md) | Recursive tool construction; propose-only phase shipped |
| [capability-gap-reuse-gating.md](capability-gap-reuse-gating.md) | Gating self-extension proposals on reuse; under review |

## Embodiment

| Doc | |
|---|---|
| [shared-world.md](shared-world.md) | The shared walkable 3D world behind `--world` |
| [factorio-bridge-architecture.md](factorio-bridge-architecture.md) | The Factorio bridge — server, mod, tools, telemetry |
| [game-embodiment-assessment.md](game-embodiment-assessment.md) | Why Factorio, and the v1 plan and success criteria |
| [cohabitation-writeup.md](cohabitation-writeup.md) | Five days living with the Factorio agent |
| [jill-integration.md](jill-integration.md) | The ChatterBot head binding |
| [cw-voice-sensor.md](cw-voice-sensor.md) | The voice sensor: mic → STT → turn |
| [cw-voice-sensor-plan.md](cw-voice-sensor-plan.md) | The build plan behind it |
| [audio-out-design.md](audio-out-design.md) | Audio out — the "say" path |

## Other surfaces

| Doc | |
|---|---|
| [RESOURCE_BROWSER.md](RESOURCE_BROWSER.md) | The standalone web resource browser |
| [REMOTE_VIEWER_DESIGN.md](REMOTE_VIEWER_DESIGN.md) | Remote viewing via `mirror.sh` |

## Measurement

| Doc | |
|---|---|
| [measurement-v3.md](measurement-v3.md) | Score traces, not tasks — the current approach |
| [../measure/README.md](../measure/README.md) | How to run it, the join traps, and what `bench/` was |
| [m1-collection-inputs.md](m1-collection-inputs.md) | Exemplar inputs for M1 collection — stale, M0–M5 retired |
| [substack_awareness_evaluation.md](substack_awareness_evaluation.md) | Write-up of a suite deleted 2026-08-18 |

## Security

| Doc | |
|---|---|
| [sentinel-setup.md](sentinel-setup.md) | The Sentinel character: probe tiers and the sudoers fence |

## Essays

| Doc | |
|---|---|
| [journey_note_trace_qualia_self_model_v2.md](journey_note_trace_qualia_self_model_v2.md) | Trace and self-model; v2 retracts v1's framing |
| [substack-gut-feeling-draft.md](substack-gut-feeling-draft.md) | Gut feelings as index-free memory |
