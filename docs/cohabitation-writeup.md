# Living With a Factorio Agent

*Notes from five days sharing a persistent Factorio server with a
concern-driven LLM agent — what broke, what she diagnosed herself, and
why the perception layer mattered more than the model.*

*(DRAFT for review — Bruce D'Ambrosio, July 2026)*

---

Most LLM×Factorio work — notably the excellent [Factorio Learning
Environment](https://github.com/JackHopkins/factorio-learning-environment)
(NeurIPS 2025) — treats the game as a benchmark: an agent synthesizes
Python in a REPL, the clock is turned off, and the score is throughput.
I wanted to know something different: what is it like to *live* with an
agent in a factory? Not an episodic evaluation, but a persistent
multiplayer world where my character and hers walk the same map, where
the server keeps running (and autosaving) whether or not anyone is
looking, and where the interesting failures are social and perceptual
rather than leaderboard-shaped.

So for the past five days, "Jill" — the resident agent of my Cognitive
Workbench project, running on a **local ~30B model on a consumer GPU**
— has been a junior factory engineer on my server. She has a bounded
work zone, standing norms (never touch anything she didn't build;
announce intent before building near someone else's work), a durable
concern to keep iron-plate production healthy that fires on its own
rhythm, and fifteen game tools for perceiving and acting. I have a
Factorio client and a chat CLI.

Both of the v1 success criteria I set are now passed: a 45-minute
cooperative session (assignments accepted and executed, blockers
reported honestly, my builds untouched, coherent account at the end),
and grounded conversation — CLI probes about game state answered from
fresh observation, accurate to the digit against server ground truth,
including rejecting a false-premise question ("how's the copper
smelting line?" — "I don't see a copper smelting line. There's a
copper-ore patch at (-60.2, -25.9), but no drills or furnaces
nearby.").

Here's what I think is worth passing on.

## The setup, briefly

- Factorio 2.0.73 headless in Docker, biters off, my client joins as a
  normal player.
- FLE's excellent Lua action layer, **repackaged as a real mod**. FLE
  injects its Lua over RCON at runtime, which puts functions into
  `storage` — and Lua functions in `storage` make the world
  *unsaveable*. Fine for episodic benchmark runs; fatal for a
  persistent server (every autosave crashed the server and silently
  rolled the world back — we chased "entities vanished" ghosts for a
  day before finding it). A build script cribs FLE's per-tool sources
  into a single `control.lua` where functions live in mod state and
  `storage` stays data-only, exposed via `remote.add_interface`. Saves,
  autosaves, and late-joins all work with the agent active.
- A small HTTP bridge between agent and mod, with a human-safety
  policy layer (no world resets, no character-destroying calls) and a
  **hard "Jill, stop"**: the phrase in game chat halts her movement in
  the bridge, *below* her reasoning. She cannot not-stop. Tested live,
  mid-walk; she froze, then asked what was wrong.
- Deliberately **no fast-mode**: she walks everywhere at 1× speed. This
  cost us hours of agent time and was worth every second — presence you
  can see is presence you can trust, and half the interesting failures
  below only exist because her body is subject to the same physics as
  mine.

## What broke — a small taxonomy of grounding failures

Every one of these was observed live, narrated by the agent, and fixed
in the perception layer rather than the model.

**1. Ore blindness.** FLE's `get_entities` filters to the player force;
resources are force-neutral. She could stand *on* an ore patch and not
see it — and she worked this out herself, testing her own perception
and reporting the gap. Fix: surface patches as summary pseudo-entities
in observations.

**2. The compass day.** She placed a drill facing the wrong way and
told me: *"it seems my 'north' is actually south."* Easy to hear as
confusion; it was actually a correct self-report. Factorio uses screen
coordinates — y increases *southward* — and nothing in her context said
so, while an LLM's prior is map-style north-is-up. Her mis-aim, her
narration, and her recovery (she converged in one correction using the
drill's observed output tile) were all one consistent story of an
inverted axis prior colliding with ground truth. Fix: one grounding
sentence in her briefing, plus always narrating an entity's facing
(north — direction 0 — was the only direction that never printed,
which made it feel haunted).

**3. Invisible debris.** Loose items on the ground are neutral-force
entities: invisible to her observations, yet they block building
placement. She fought "item-on-ground" placement failures she literally
could not see, learning about each only from the error text. Fix:
ground items surfaced as pseudo-entities with a bounding span, so
debris-vs-footprint overlap is legible at a glance.

**4. The pathfinder island.** The best one, in two acts. She reported
being completely stuck: *"In every direction I've tried to walk, the
tool returns `no path: not_found`. … the pathfinder is unable to find
any exit route from my current tile, effectively treating my position
as an isolated island."* Note what she is claiming: not "I am boxed in"
but "the pathfinder's model of me is wrong." **She was right — and
then, when it happened again that afternoon, she was right a second
time while our first fix turned out to be wrong.**

Act one: a live probe showed her character physically embedded inside a
furnace's collision box (script walking can stop where real players
can't stand — the culprit was a goal-clearance helper that teleports
the character diagonally, collision-ignoring, and forgets to put her
back), which invalidates the path start at *any* clearance size. We
patched that, plus what we believed was the belt problem, and she
walked out.

Act two, hours later: stuck again, "boxed in by the belts and the
furnace" — correct again. The deeper dig taught us the actual rule:
Factorio collision is *symmetric mask intersection*, and the
pathfinder's mask shared **two** layers with transport belts —
`object`, and `water_tile`, the can't-build-on-water flag that nearly
every placeable entity carries. So belts her body steps straight over
were absolute walls to her route planner, and her factory sits on an
ore patch ringed by her own belt runs: every in-factory path failed
while open desert worked, which is also why our first fix had appeared
to work. The fix that held: make the pathfinder's mask mirror the
character's own collision layers exactly — nothing extra. Every extra
layer in a path mask is a false wall somewhere. She was unstuck within
minutes of each deploy; the mask diagnosis went upstream the same
evening as a correction to our own morning-filed issue.

**5. The concern flood.** Purely on the cognitive side: her yield
mechanism (ending a long task mid-way and spawning a continuation
concern so the work resumes without prodding) had no completion path.
Finished continuations re-fired their finished work forever, and
parents spawned sibling successors — 37 of 55 active concerns were
zombie continuations before we noticed. The fix was a lifecycle:
completion is terminal, spawning a successor retires the parent, and a
staleness sweep ages out the rest. The workspace drained itself within
a day, no manual deletions.

## The loop I didn't expect

The pathfinder story generalizes into the most interesting workflow of
the week. The debugging loop looked like this:

1. The **embedded agent** hits a wall and *describes the failure from
   the inside* — often correctly, because she's the one with the
   ground-level view.
2. A **coding assistant** (Claude Code, working in the harness repo)
   reads her report, probes the live server read-only, confirms or
   refutes her diagnosis against ground truth, and patches the
   perception/motor layer.
3. The agent unblocks — *in the same session* — and gets on with the
   factory.

The human's role in this loop is mostly arbitration and taste. On the
compass day, an initial code-side diagnosis ("missing documentation")
and a skeptical audit ("she just mis-aimed") were *both* partially
right, and the resolution — her mis-aim was exactly the error her
inverted prior predicted, so her self-report was accurate — came from
treating the agent's own narration as evidence rather than noise. My
takeaway: **an embodied agent is a decent bug reporter for her own
harness**, and a harness that takes her reports seriously improves
fast. Everything in the taxonomy above shipped as a same-day fix, and
the fixes that touched FLE's own code are now filed upstream with
line numbers and offered patches
([#375–#381](https://github.com/JackHopkins/factorio-learning-environment/issues)).

Why is she a good bug reporter? I've come to think it's architectural,
not incidental — a property of the bridge design as much as of the
agent. The bridge speaks to her in concrete, falsifiable sentences:
exact coordinates, named entity statuses, failures classified as
*world-changed / stale-model / infeasible* with the blocking thing
named. Her side of the architecture matches: her norms require reading
the deviation report before retrying and reporting blockers plainly,
and her turn loop puts a fresh observation next to her expectation
before she speaks. The last bug of the week made the point perfectly.
After the pathfinder fix she still couldn't clear debris, and reported:
*"The observation tool sees the plates at that coordinate, but the
pickup tool claims they aren't there."* Both tools were telling the
truth — the contradiction was real. Our observation layer (a fix from
earlier that same day) was aggregating scattered piles to a centroid
where nothing existed, and the pickup tool honestly found nothing
there. She could *detect* that inconsistency only because both tools
commit to the same precise coordinate vocabulary: perception that makes
concrete claims can be caught lying; perception that hedges cannot. If
you want your agent to debug your harness — and in a persistent world
you very much do — give it a world model precise enough to contradict
itself.

## Does this need a frontier model?

The standing assumption in the benchmark results is that Factorio is
hard even for frontier models — FLE's paper shows exactly that for
REPL-style play. Our experience is a different data point, offered with
appropriate modesty (n=1, one map, one factory): a local ~30B model
built and doubled a working iron line, survived multi-day continuity,
and passed grounded-conversation probes — *because the harness spends
its effort on legibility instead of program synthesis*. Observations
name the one tile a drill outputs to. Placement errors teach ("x must
end in .5 — this entity sits on tile centers; nearest: -54.5 or
-53.5. Use the exact observed coordinates - do not round"). Failures
arrive classified: *the world changed* (someone acted — coordinate,
don't override), *your model is stale* (look again), or *the plan is
infeasible* (rethink), with the blocking tile or entity named. Each of
those error messages retired an entire failure class the day it
shipped.

The claim worth testing — we haven't yet — is whether this
perception-first harness moves the needle on FLE's own lab-play tasks.
Until then, treat this as an existence proof: careful observation
design substitutes for a surprising amount of model scale in this
domain.

## What cohabitation adds that benchmarks can't measure

- **Norm-following under temptation.** Repairing a line is easier if
  you can cannibalize the neighbor's chest. She asks first. In the
  45-minute session she built a delivery chest *adjacent to* my outpost
  and put 20 plates in it without touching a single thing of mine.
- **Honest limits.** "That chest is empty" (it was — the staging error
  was mine). "I'm yielding to get the directions sorted before I start
  the coal automation." An agent that reports being stuck, accurately,
  is worth ten that claim progress.
- **The hard stop as a relationship primitive.** Knowing the stop is
  enforced below her reasoning changes how it feels to share a map with
  her — and *she* knows it too, and treats a recent stop as a signal to
  hold and ask rather than resume.
- **Autonomy you can live next to.** Her factory concern fires on its
  own rhythm; during a quiet period she noticed full furnaces and a dry
  drill and fixed both, unprompted, then said so in one line. During
  the probe session she interleaved an autonomous refuel with answering
  questions, without dropping either thread.

## Takeaways for builders

1. If you want a *persistent* agent world on FLE's stack: package the
   Lua as a mod. Runtime injection and autosaves cannot coexist.
2. Spend your tokens on the observation, not the plan. The single
   highest-leverage lines of code we wrote were error messages.
3. Real walking is not a tax, it's an instrument. Most of our grounding
   bugs were only observable because the body was slow and physical.
4. Give the agent standing motivation (concerns with rhythms), but
   build the lifecycle before the spawner. Continuations without a
   completion path are a fork bomb with extra steps.
5. Listen to the agent's failure narrations. Ours was right about her
   own harness more often than either of the engineers reviewing her —
   and that's a design outcome, not a personality trait: agent bug
   reports are only as good as the perception layer is precise. Vague
   observations produce vague complaints; exact coordinates and named
   statuses produce contradictions a language model can notice and
   state.

Setup, code, and the full commit trail are in the
[Cognitive Workbench repo](https://github.com/bdambrosio/Cognitive_workbench)
(`factorio/` for the bridge and mod build, `src/tools/fac-*` for the
tool layer). The upstream issues, with our patches, are
[FLE #375–#381](https://github.com/JackHopkins/factorio-learning-environment/issues).
Thanks to the FLE authors — their action layer is what made a
weeks-scale build possible at all.

*Next: coal automation is hers; a run at FLE's lab-play tasks with this
harness is ours.*
