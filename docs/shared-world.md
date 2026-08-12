# The shared world

A walkable 3D space Bruce and the characters occupy together, behind
`--world`. Not a game and not a simulation of anything: it exists so
co-presence is available as a channel alongside text. Jill can notice
Bruce arrive rather than being told, point at a place instead of reciting
a coordinate, and be somewhere while a conversation happens elsewhere.

```
python3 launcher.py jill-chat.yaml --cli --autonomy --world
```

Everyone launched gets a body unless `--world-characters` narrows it. You
are `Bruce` unless `--world-player` says otherwise. `--world` and
`--factorio` are mutually exclusive, and each drops the other's tools from
every catalog — with fifteen `fac-*` tools against three `world-*` ones,
"come to me at -18, 25" matched `fac-status`.

## Pieces

| Piece | What it is |
|---|---|
| `src/world/server.py` | Authoritative state. WS `8790` (state out at 20 Hz, browser input in), HTTP `8791` (agent tools + static files). Every mutation goes through one lock. |
| `src/world/terrain.py` | Deterministic heightmap from a seed: 384 m square, ~1 m grid, 22 m of relief, water below `y=1.15`, ~21k trees. Sight lines and travel cost live here too. |
| `src/world/state.py` | `Occupant` (name, `kind`, position, heading, gait, colour) and `Marker`. |
| `src/world/display/static/world.js` | Browser renderer. Samples the same bilinear heightmap formula as `terrain.height_at()`, so what Bruce walks on and what Jill reasons about are one surface. |
| `src/world/display/static/avatars.js` | What an occupant looks like — see below. |
| `src/tools/world-look` | An occupant's perception: ground underfoot, and every other occupant with distance, bearing, whether they are moving, whether they are facing you. |
| `src/tools/world-move` | Post a goal; the server walks you there at 1.7 m/s. |
| `src/tools/world-mark` | Leave a labelled marker in the world (max 40). |
| `src/sensors/world-presence` | Edge-triggered arrival/departure, with hysteresis so someone loitering at the boundary does not flap. |

Humans move themselves and the browser reports the pose; agents post a
goal and get walked by the tick loop, which is what keeps a multi-second
agent turn from having to block on a multi-second walk.

Visibility is decided server-side and filtered out of the state stream, so
an occupant you cannot see is not in your data. The display's fog is haze
over that, not the mechanism — 70 m sight in the open, 22 m among trees.

## Travel cost

Nothing is impassable. Ground is only ever slower, over a band of exactly
2:1 — open and level at full pace, wading at half, forest and slope in
between. `walkable()` survives as a *placement* predicate for spawns and
props; it is no longer consulted by movement, so a goal is never refused
and a walker is never stranded partway.

The band exists to break the identity between a sector's area and the time
to search it. Once area stops predicting time, an even split is the wrong
split, one searcher finishes early, and the only way the other learns that
is to walk it and say so.

Two things make it perceptible rather than merely true. `world-look`
reports the going where you stand ("heavy going (61% of walking pace)"),
so an occupant reads its own pace instead of inferring it from elapsed
time; and `world-move` returns `eta_s_best_case`, deliberately naive,
because integrating the real cost would hand out for free the answer that
is supposed to be worth walking for. Measured on the default seed: a
single 134 m leg ranged 0.99–1.55 m/s, while whole 96 m sectors differ by
only about 1.2x — the variation is along a route, not between regions.

Forest weighs more than slope because this generator makes it so: slope
sits between 0.05 and 0.25 over most of the map and never once exceeds the
old `MAX_WALK_SLOPE`, while the forest field is uniform by construction
and covers half the world. It also puts the cost where sight is already
cut to 22 m, so the slow ground is the blind ground.

The browser computes the same factor from the same constants, shipped in
the terrain payload alongside a uint8 forest field. Both sides were
measured to agree within 0.0012 — the uint8 quantum. Getting this wrong
does not look like a bug; it looks like Bruce and Jill crossing identical
ground at different speeds and neither of them noticing.

## Avatars

Abstract low-poly creatures, a dozen primitives each, in `avatars.js`:

| Occupant | Form | Species cue |
|---|---|---|
| Bruce | crow | pale beak, fan tail, slate-blue wings |
| Jill | kitten | ears, muzzle and whiskers, curled tail that sways when walking |
| Sentinel | owl | large facial discs with amber eyes, ear tufts, squat body |
| Jack | hare | ears three times a cat's, lateral eyes, low haunches, pale scut |
| anyone else | the original capsule | a dark brow |

Three constraints shape them, and each is load-bearing:

- **All stand roughly 1.75 m and face `+z`.** The server gives every
  occupant one eye height, and heading `0` is `+z`, so the beak, the
  muzzle and the eye discs are facing indicators as much as species
  markers — at 26 m the three silhouettes are still distinguishable.
- **Species carries identity; colour cannot.** `color` comes from a
  join-order palette server-side, so it is not a stable per-character hue.
  Each rig wears it as a collar, which keeps the body agreeing with the
  nameplate.
- **An unlisted name renders as the old capsule.** The species table is a
  presentation lookup on a stable identity — the same kind of table as the
  colour palette — so a rename costs a shape, never a crash.

Gait drives a small walk bob. For agents it comes from the server; for the
human it comes from local input, because the server's `gait` is unusable
there: `set_human_pose` marks `walk` on every input packet while `step()`
forces `idle` for anyone without a goal, and a human never has one.

**Changing a rig:** `avatar_preview.html` renders every species side by
side on flat ground, served by the world server itself so it runs under
the same origin and CSP as the real page. With any `--world` session up:

```
http://127.0.0.1:8791/avatar_preview.html?view=front|side|back|far
```

Use it. Three of these shapes were wrong in ways only a render showed —
the crow was a featureless black blob, the kitten's eyes and muzzle were
sunk inside its head sphere, and every accent collar was buried in a body.
A rig checked only by reading the code has not been checked.
