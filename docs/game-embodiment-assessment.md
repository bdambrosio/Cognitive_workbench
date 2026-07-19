# Game Embodiment — Factorio as an agent testbed

**Status:** Assessment + v1 sketch (updated 2026-07-12; originally an
end-of-session musing 2026-07-13). No code yet. Updated after a second,
independent analysis (claude.ai session with GitHub access) reached
substantially the same conclusions — convergent points below are treated
as settled. Prompted by: "how close are we to an agent that could join a
multiplayer non-time-critical game like Satisfactory 1.2?"

**Update 2026-07-19: v1 complete.** Both success criteria below passed —
criterion 1 (45-min cooperative session) 2026-07-15, criterion 2
(grounded CLI conversation) 2026-07-17. Implementation:
`docs/factorio-bridge-architecture.md` (design) and `factorio/README.md`
(as built). Results narrative: `docs/cohabitation-writeup.md`.

## Verdict

The cognitive layer is largely ready; the game interface is essentially
the entire cost. For Satisfactory specifically the interface cost is
months (Unreal/SML modding in C++ — no agent-usable API exists; the
dedicated-server HTTPS API is admin-only). **Factorio is the same genre
with the opposite interface story**: RCON + rich Lua mod API, an existing
research harness (Factorio Learning Environment), multiplayer, biters
optional (keeps it non-time-critical). Recommended path: prototype on
Factorio; Satisfactory later only if the itch persists.

## Settled by convergence (two independent analyses agree)

- **Symbolic interface, not screen control.** Mod-API observations and
  actions; never pixels/mouse. Pixel grounding, camera state, GUI modals,
  and gesture latency are all avoidable problems.
- **Coarse transactional macros are the real work.** "Construct this
  validated belt path from A to B" — one tool call that returns exact
  placements, failures, obstructions. LLM-per-primitive won't survive
  (FLE's published results confirm this empirically).
- **Bridge is a separate process** (peripheral pattern, Body/ChatterBot
  lesson): rate asymmetry between cognition and embodiment → thin
  Factorio Lua mod exposing semantic observations + bounded actions,
  external bridge translating to CW tools/sensors.
- **The research question is multiplayer sociality, not throughput.**
  Division of labor, resource-conflict avoidance, persistent social
  commitments ("I'm handling copper; you improve power"), explaining
  before acting, resuming after a human altered the factory, staying
  present without spamming chat.

## What maps directly (no new invention)

- **Concerns = factory assignments.** "Keep power positive", "Jill
  manages iron smelting" are standing evaluative pressure with
  rhythm-based revisit — concern-shaped exactly. WIP + the NEXT: line
  (b3dde3b0) is the right planning model: nothing time-critical, always
  "most valuable next improvement."
- **Infospace = factory model.** A factory is a dependency graph;
  notes/collections hold that natively. world_map fuser is the precedent
  for a separate process maintaining world state the agent reads at its
  own rate.
- **Multiplayer chat** = the chat loop over a different transport (RCON
  carries chat both ways).
- **Minecraft layer is a working precedent, not just specs.** Every
  `src/world-tools/minecraft/mc-*` skill has an implemented `tool.py`
  hitting an HTTP bot server (`MINECRAFT_URL`, default `localhost:3003`),
  with shared `nav_core`/`map_core`. The Node/Mineflayer server lives
  outside this repo (same split as Body). The tool-vocabulary shape
  (Skill.md + tool.py + uniform returns + world_url) carries over
  directly to `factorio-*` tools.
- **Measurement gift: environment-grounded valence.** Power uptime and
  throughput are objective. Fire-outcome valence stops depending on a
  judge or on reading user reactions — the strongest available M-series
  testbed. Design consequence: the bridge exports production/power
  telemetry from day one, even before any bench exists.

## Roadmap convergence (why this isn't a detour)

- The **macro layer** is the Multi-Loop Execution Gap the WIP reviewer
  escalated 2026-07-12, and METHOD_TOOLS.md (since deleted as
  superseded; in git history) made necessary. The game is
  its forcing function.
- **Event→concern ingress** ("power dipped" spiking the power concern)
  is the same machinery as the penciled M2 candidate.
- Net: weeks spent on roadmap items with a game as the test rig, not
  weeks spent on a game.

## v1 design decisions

**Role: junior factory engineer, not autonomous general player.**
Specialist co-player with an assigned bounded work zone and rememberable
responsibilities. Capabilities: walk/path to named areas; inspect
machines, belts, inventories, recipes, power and production; fetch and
deliver materials; place small approved designs; connect belts and
inserters; clear simple bottlenecks; report what she sees; discuss plans
in multiplayer chat.

**Control stack: two layers, not three.** The proposed reflex layer
(collision recovery, train avoidance, pause-on-player-entry) is imported
from Body, where physics doesn't wait. Factorio with biters off and no
trains has nothing fast; an atomic macro executed by the mod IS the
reflex layer. So: (1) transactional macro executor in the mod —
validates prerequisites, executes atomically or aborts with a structured
deviation report; (2) Jill deliberative — picks objectives, selects and
parameterizes macros, judges results, decides when to ask vs. act.

**Controls: trust + audit, not a permission matrix.** Exactly two
enforced-below-the-LLM mechanisms: a hard "Jill, stop" chat command, and
a visible activity log (attribution: when she makes a bad decision, know
which belief/plan/action produced it). Norms like "don't deconstruct
what you didn't place", "don't spend modules" live in her instructions,
promoted to enforcement only on evidence of violation.

**Interface strategy: crib FLE's plumbing, discard its agent loop.**
FLE = [JackHopkins/factorio-learning-environment](https://github.com/JackHopkins/factorio-learning-environment)
([paper](https://arxiv.org/pdf/2503.09617)). Its RCON plumbing, action/
observation vocabulary, and server tooling are the part that took time —
fork or crib that. Its Python-REPL code-synthesis agent loop and
throughput scoring are the part to discard. Caveats: FLE pins Factorio
1.1.x (Space Age 2.0 reworked parts of the mod API — pin versions
deliberately); RCON runs server-side with full trust (client-faithful
constraints are optional, imposed by the mod if wanted).

**Server setup for the first experiment:** enemies disabled entirely
(not merely peaceful), small map, normal tech progression, no trains
initially, no autonomous deconstruction, Jill assigned a bounded work
zone, one modest objective (e.g. maintain iron-plate production).

## Success criteria (v1)

1. **Cooperative session.** For a 45-minute multiplayer session, Jill
   accepts assignments, performs several material-handling and
   construction tasks, reports genuine blockers, does not damage human
   work, and maintains a coherent account of the shared plan.
2. **CLI grounding.** Through the normal CW chat CLI (not in-game chat),
   Jill can talk intelligently about game status, her current activity,
   and recent events — "power margin is 12% and falling, I'm adding a
   boiler row", "storage backed up while you were away, here's what I
   did." This exercises the full sensor → infospace → chat loop and
   proves the factory model is genuinely in her situation, not just in
   the bridge.

Not a success criterion: launching a rocket, factory throughput, or any
single-agent benchmark score.

## Real gaps, ranked

1. **Game bridge** — mod + RCON bridge process; weeks, reduced by
   cribbing FLE's interface layer.
2. **Macro layer** — small library of parameterized transactional macros
   (build-smelter-column, run-belt-A-to-B, extend-power); the
   load-bearing work item, shared with the roadmap.
3. **Event→concern ingress** — sensor events spiking concerns;
   convergent with M2 candidate, not additional.
4. **Spatial navigation** — mostly dissolves: mod API exposes pathing;
   the agent plans over the factory graph, not 3D space.

## Estimate

Factorio v1 (joins server as a visible character, holds standing
factory concerns, executes macro-level improvements on rhythm,
coordinates in chat, answerable about the game via CLI): **weeks** of
bridge + macro work; cognition needs almost nothing new. Satisfactory
1.2: add **months** of modding first.

## Next step (resume here)

Write the concrete architecture note: map the existing Minecraft tool
plumbing (Skill.md + tool.py + HTTP bot server + world_url convention)
onto a minimal Factorio mod/bridge API — observation set, initial macro
set (checked against FLE's action vocabulary for overlap), sensor/event
list, and the fork-FLE vs. fresh-thin-mod decision.

## Discipline note

Separate subproject (Body-scope rule,
[[project-body-subproject-scope]]) — keep it out of M1/M2 measurement
until the bench story for it exists. The environment-grounded-valence
property means it may eventually *become* a bench substrate, but that is
a later decision, not a v1 goal.
