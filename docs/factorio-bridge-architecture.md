# Factorio bridge — v1 architecture

**Status:** Architecture note (2026-07-13). Fulfils the "next step" of
docs/game-embodiment-assessment.md: maps the existing tool/sensor
plumbing onto a concrete Factorio interface. Based on source-level
review of FLE at HEAD (f748ec4, 2026-06-11) and a full map of the
Minecraft world-tools layer. No code yet.

## Corrections to the assessment (facts that changed)

1. **FLE is on Factorio 2.0.73, not 1.1.x.** v0.4.0 (2026) was a full
   1.1.110 → 2.0.73 migration; v0.4.3 is current, actively maintained
   (882 commits, last 2026-06-11), MIT licensed. Pin `2.0.73` to match.
2. **There is no FLE mod to crib.** FLE injects all its Lua over RCON
   at connect time (`/silent-command`, functions registered into
   `storage.actions.*`, checksummed to skip re-injection).
   ~~"Runtime-injected Lua, no mod packaging"~~ — **overturned by
   build-order step 2** (see "Step-2 reversal" below): injection is
   fine for FLE's disposable single-tenant benchmark servers but
   incompatible with a persistent, human-joinable world. The
   assessment's original "thin Factorio Lua mod" was right after all;
   we crib FLE's per-tool `server.lua` bodies into it.
3. **The macro layer is substantially pre-built.** FLE's
   `connect_entities(*waypoints, connection_type)` is exactly the
   "construct this validated belt path from A to B" transactional macro
   the assessment called the load-bearing work item: resolves
   connection points per type (belt/pipe/power/wall), pathfinds around
   obstacles, places+rotates every segment, uses undergrounds when
   allowed, supports `dry_run`, returns structured entity groups.
   `place_entity`, `insert_item`, `craft_item` (recursive), `move_to`
   (real pathfinding), `harvest_resource` etc. cover the rest of the
   junior-engineer verb set. Gap #2 shrinks from "build a macro
   library" to "wrap and extend one".
4. **One seam correction on our side:** the mc-* world-tools were
   consumed by the OODA-era `InfospaceExecutor` (deleted 2026-05-02).
   The live tool path is the ReAct chat loop's discovery of
   `src/tools/*/` (`Skill.md` + `react_invoke`). Factorio tools follow
   the *chat-tool* pattern (like check-x-feed), not the legacy
   world-tools executor pattern. The mc-* layer still contributes its
   HTTP contract shape and its pain-point ledger (below).

## Step-2 reversal: the Lua must be a mod (empirical, 2026-07-13)

Found live during build-order step 2, with a human client connected:

- **Lua functions in `storage` make the world unsaveable.** Factorio
  serializes `storage` on save and cannot serialize functions; the
  failure is non-recoverable in headless mode, so **the server process
  exits**. Docker's `restart: unless-stopped` then reloads the last
  good save — a silent crash-restart-regress loop (observed
  RestartCount=4; every "lost entities" mystery during steps 1–2 was
  this, not scenario scripting). Autosaves hit the same wall, so an
  injected world also cannot autosave.
- **Late join needs serialization too.** Serving the map to a joining
  player is a save; a human cannot even join a server while FLE's
  functions sit in `storage`. And even with functions moved to plain
  Lua globals, `/sc` commands execute deterministically on *every*
  peer — a late joiner who never executed the injection would desync
  on the first action call. Runtime injection and persistent
  multiplayer are structurally incompatible, not just inconvenient.

**Consequence — the `fle-bridge` mod.** A thin mod, mounted into the
container (`data/mods/`), whose `control.lua` registers FLE's cribbed
per-tool action bodies behind `remote.add_interface("fle_bridge", ...)`.
The bridge process calls them via RCON `/sc remote.call(...)`. This is
the standard Factorio-correct shape: functions live in every peer's
mod Lua state (recreated on every load — nothing to re-inject),
`storage` holds only data, saves/autosaves/late-joins all work, and
tick handlers (walking queues) re-register in `on_load` like any mod.
The mod is also the natural home for the policy layer (safe character
spawn, "Jill, stop") and the `on_console_chat` capture. Porting cost:
mechanical — wrap each cribbed `server.lua` body as a remote-interface
function instead of a `storage.actions` assignment; FLE's Python-side
entity models and `connect_entities` resolver logic still carry over
in the bridge process.

## Fork-FLE vs fresh decision

**Depend on `fle` as a pinned library (v0.4.3); do not fork; write our
own thin bridge process around it.** Rationale: MIT, active, 2.0-native,
and the parts we want (LuaScriptManager, the 28 agent tools'
`server.lua`+`client.py`, `serialize.lua`, connection resolvers) are
cleanly separable from the parts we discard (the Python-REPL agent
namespace, gym wrapper, throughput scoring, A2A protocol). If the
library's API churns, we're pinned; if we ever need surgery, MIT lets
us vendor the ~dozen files we actually touch.

**What we must NOT use from FLE (human-coexistence hazards, verified in
source):**
- `create_agent_characters` — destroys ALL `character` entities on the
  surface first (would kill Bruce's player). Bridge spawns Jill's
  character with its own non-destructive Lua.
- `reset`, `clear_entities`, `regenerate_resources` — assume FLE owns
  the world.
- `game.speed` / `storage.fast` teleport-movement — assume the agent
  owns the clock. Multiplayer with a human runs at 1× and Jill's
  character genuinely walks (`move_to`'s real walking mode, updated per
  5 ticks). Visible presence is a v1 requirement, not a nicety.

Everything else in FLE's per-tool Lua enforces character reach
(`validate_distance` → "Move closer"), which is exactly the embodiment
realism we want — keep it.

## Topology

```
Factorio headless server (Docker, factoriotools/factorio:2.0.73,
  biters disabled, whitelist, RCON)          [game clock: 60 tps]
        ▲ RCON (/silent-command + storage.actions.*)
        │
factorio_bridge  — separate process, peripheral pattern   [~seconds]
  · imports fle (pinned): LuaScriptManager, tool Lua, connect resolvers
  · policy layer: human-safe subset, Jill's character, activity log
  · telemetry cache: production/power/alerts polled ~10 s
  · chat relay: on_console_chat capture + /say
  · HTTP server (localhost:3004), ok/error envelope
        ▲ HTTP (FACTORIO_URL, default http://localhost:3004)
        │
CW side                                     [Jill's rhythm: minutes]
  · src/tools/fac-*/            chat tools (react_invoke)
  · src/utils/factorio_link.py  shared HTTP client (one canonical impl)
  · src/sensors/factorio-telemetry/  sensor → sense_data → concerns
```

Rate asymmetry is honored at two boundaries: the bridge absorbs the
60 tps game into a ~10 s telemetry cache; Jill reads that cache at her
own concern rhythm. Same shape as world_map (Body) — the precedent the
assessment cites.

## Bridge HTTP contract (mirrors the Minecraft bot-server shape)

JSON everywhere; `{ok: true, ...}` / `{ok: false, error, deviation?}`.

Read (GET):
- `/status` — Jill's character position, held items, current activity,
  server tick, connected players.
- `/observe?x&y&radius&types` — entities in range via FLE
  `get_entities` (grouped belts/pipes/poles), resource patches.
- `/inventory` — Jill's inventory.
- `/entity?x&y&prototype` — one entity, full status + warnings.
- `/telemetry` — cached: power produced/consumed/satisfaction,
  production flows (last 1m/10m), active alerts. Exported from day one
  (environment-grounded valence, per the assessment).
- `/chat?since_seq` — game chat messages (captured server-side).
- `/log?since_seq` — Jill's own action log (the audit control).

Act (POST):
- `/act/walk` `{x, y}` — real walking via move_to; returns arrival or
  deviation (unreachable, timeout).
- `/act/place` `{prototype, x, y, direction}` — place_entity.
- `/act/place_next_to` `{prototype, ref, direction, spacing}`
- `/act/connect` `{waypoints, kind, allow_underground, dry_run}` —
  connect_entities. THE macro. `dry_run:true` is Jill's "check my plan"
  affordance.
- `/act/insert` / `/act/extract` `{item, target, quantity}`
- `/act/craft` `{item, quantity}` ; `/act/recipe` `{entity, recipe}`
- `/act/harvest` `{x, y, quantity}` ; `/act/pickup` `{entity|position}`
- `/act/rotate` `{entity, direction}`
- `/say` `{message}` — multiplayer chat.
- `/act/stop` — abort current walk/queue. Backs the hard "Jill, stop"
  chat command, which the bridge also enforces itself on seeing the
  phrase in chat (control below the LLM, per the locked decision).

Norm enforcement stays out of the bridge in v1 (norms live in Jill's
instructions; promoted to bridge-level checks only on evidence of
violation) — with one exception: `/act/*` refuses to deconstruct or
insert into entities inside another player's force-flagged area if we
later define zones. v1 ships without it.

## Deviation reports (the coordination signal)

Every `/act/*` failure carries a structured `deviation` distinguishing:
- `world_changed` — prerequisite held at last observation but an entity
  appeared/vanished since (someone acted). Feeds event→concern ingress:
  this is a *coordination event*, often worth a chat line.
- `stale_model` — Jill's belief was wrong at observation time (never
  observed, or cache expired). Triggers re-observation, not chat.
- `infeasible` — valid observation, impossible request (no path, out of
  reach, insufficient items). Plan-level failure.

The engine gives us this cheaply: FLE tools already error with typed
messages; the bridge classifies by re-checking the failed prerequisite
against its observation timestamp. This implements the "engine
rejection as high-bandwidth desync signal" point from the Jill
conversation, and is the piece FLE does not have.

## What the Minecraft ledger buys us (checked against its pain points)

The mc-* layer's five documented pain classes, and their Factorio fate:
1. *Async accepted-but-ineffective actions* (mc-place →
   mc-place-until-supported compensator) — absent: RCON Lua executes
   synchronously inside a tick; place either returns the Entity or
   errors.
2. *Pose drift / grid alignment* (ensure_grid_aligned, re-snap after
   every move) — absent: entities occupy exact tiles; the character's
   position only matters for reach, which the Lua validates itself.
3. *Hand-rolled retries inside primitives* (nav-climb recovery digs) —
   not needed; the one retry FLE keeps (RCON `[processing]` busy loop)
   lives in transport, not semantics.
4. *Per-op timeout tuning* (5s/10s/30s/60s ladder) — collapses to one
   RCON round-trip timeout plus a walk timeout.
5. *Observation staleness compensators* (spatial_map expiry,
   absolute-not-relative storage) — the lesson carries over as design,
   not code: the bridge cache stores absolute positions + timestamps,
   and deviation classification uses them.

## CW-side tools (v1 set)

`src/tools/fac-*/`, standard chat-tool anatomy (Skill.md frontmatter +
`react_invoke` returning `{status, text}`), all HTTP via
`src/utils/factorio_link.py` (env `FACTORIO_URL`, default
localhost:3004). Tool → endpoint mapping is 1:1 except `fac-observe`
(status+observe+telemetry composition for a one-call situational read):

- `fac-status`, `fac-observe`, `fac-inventory` — epistemic
- `fac-walk`, `fac-place`, `fac-connect`, `fac-insert`, `fac-extract`,
  `fac-craft`, `fac-harvest`, `fac-rotate` — action
- `fac-say` — social; `fac-stop` — safety
- Deferred: blueprint/small-design placement (needs the approved-design
  library), `fac-map-render` (FLE's admin `render` returns a base64 PNG
  — a natural fit for Jill's canvas `display`, but a nicety).

Returns embed the deviation report verbatim in `text` so the ReAct
loop reasons over it directly.

## Sensors and event→concern ingress

- `src/sensors/factorio-telemetry/` (standard sensor_loader anatomy:
  SKILL.md `type: code, schedule: "2m"` + sensor.py `run(context)`).
  Polls `/telemetry`; `gate` passes only on threshold events: power
  satisfaction < 95%, a watched production flow hitting zero, new
  alert kinds, chat messages addressed to Jill. Publishes the standard
  sense_data envelope; concern evidence-bumps do the rest (same
  machinery as the M2 event→concern candidate — this is the convergent
  roadmap item, now concrete).
- Game chat *to* Jill arrives through the same sensor (bridge `/chat`),
  so "talk to her in-game" and "talk to her via CLI" use one ingress
  pattern. CLI grounding (success criterion 2) needs nothing extra:
  the telemetry digests land in her context via fires; her WIP holds
  the factory narrative.

Concern seeds (jill-factorio.yaml, a scenario like jill-embodied was):
one durable assignment concern ("keep iron-plate production healthy in
my zone", rhythm 1–2 h, instruction = observe → judge → at most one
improvement macro → report), plus the standing norms in the character
instructions. No new concern machinery.

## Server setup (first experiment, per locked decisions)

Docker `factoriotools/factorio:2.0.73`; `open_world`-style normal map;
map-gen with enemies **disabled entirely**; whitelist Bruce + bridge;
RCON on localhost only, non-default password; no trains; Jill assigned
a bounded work zone by instruction. FLE's compose generator is the
crib for the container config; we write our own static compose file
(no need for their multi-instance cluster).

## Known risks / open questions

1. ~~Injected Lua persistence across save/load~~ — superseded by the
   step-2 reversal: the `fle-bridge` mod loads on every save/load like
   any mod; nothing to re-inject. Replaced by: **mod-porting effort** —
   each cribbed `server.lua` needs mechanical rewrapping plus a pass
   for hidden `storage`-function assumptions (helpers in `utils.lua`
   land in `storage.utils` today and must become mod-local functions).
2. **`/silent-command` in live multiplayer** — VERIFIED step 2:
   reads and entity-creating writes with a human client connected, no
   desync. (Remote-interface calls via `/sc` are the same input-stream
   mechanism, so this carries over to the mod design.)
3. **fle API churn** — pinned at 0.4.3; the import surface we use is
   small (entity models, connect resolvers, rcon utils). Listed so an
   upgrade is a decision, not an accident.
4. **Walking realism vs responsiveness** — real walking at 1× makes
   long hauls slow (minutes across a map). Acceptable for v1 (nothing
   time-critical); if it grates, the compromise is bounded-teleport
   inside her work zone only, never near players. Decide on evidence.
5. **Chat capture** — `on_console_chat` handler must be registered from
   injected code; confirm it survives in multiplayer and captures both
   player chat and Jill's own `/say` (for the log). Fallback: poll
   console log via RCON.

## Build order

1. **Server up + fle smoke** — DONE 2026-07-13 (factorio/smoke_fle.py):
   RCON connect, Lua injection, place/pickup/connect_entities/
   get_entities all verified against the live 2.0.73 server; human
   client joined and watched.
2. **Persistence + coexistence checks** — DONE 2026-07-13
   (factorio/coexist_fle.py): human-safe character spawn verified
   (HumanSafeInstance; stock FLE destroys the human's character);
   /sc with human connected verified no-desync; save/reload check
   FAILED and produced the step-2 reversal above.
2b. **fle-bridge mod skeleton** (new, before the bridge process):
   info.json + control.lua with `remote.add_interface`; port 2–3
   actions first (place_entity, get_entities, move_to). Verify:
   save/autosave succeed with the mod loaded and actions used; a
   client can join AFTER actions ran; walking tick-handler survives
   save/reload. Then port the rest of the action set incl.
   connect_entities, plus the safe character spawn and chat capture.
3. **Bridge process** (the main chunk): policy-wrapped tool surface,
   HTTP endpoints, telemetry cache, chat relay, activity log,
   deviation classifier. Verify: curl-level script builds a small
   smelting column end-to-end, including one forced deviation of each
   class.
4. **CW tools + factorio_link** (a day or two): fac-* set, discovery
   scan check, mocked-bridge smoke tests (check-x-feed pattern).
5. **Sensor + scenario** (a day): factorio-telemetry, jill-factorio
   scenario with the one assignment concern; wire "Jill, stop".
6. **The 45-minute session** (success criterion 1), then CLI-grounding
   probes (criterion 2).

Estimate unchanged from the assessment — weeks — but with the macro
layer largely inherited, the critical path is now the bridge's policy
layer and deviation classifier, not construction geometry.
