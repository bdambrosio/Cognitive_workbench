---
name: world-presence
description: Notices when someone comes near you or moves away in the shared world. Fires on the transition only, and carries the current situation with it.
type: code
schedule: "30s"
disposition: inform
parameters:
  enter_m: 12.0
  exit_m: 20.0
---

# world-presence

The push half of world embodiment. `world-look` answers "who is here?"
when the agent thinks to ask; this fires when the answer *changes*, so
the agent can notice someone arriving without being prompted.

## What it reports

**Transitions, not state.** A level-reporting sensor on a 30-second
schedule would emit "Bruce is still standing there" a hundred times an
hour, and every emission costs a full agent turn. So it stays quiet
while nothing changes, and speaks when someone crosses the proximity
boundary in either direction.

**But it carries state in the payload.** A bare "Bruce arrived" would
force a `world-look` before the agent could say anything useful — a
wasted iteration on every wake. The event ships the current positions,
distances and ground with it, so the first iteration can already act.

Current presence at any other moment is not this sensor's job: the
system prompt's Embodiment block carries who is in the world, and
`world-look` answers on demand.

## Hysteresis

`enter_m` (12 m) is closer than `exit_m` (20 m) on purpose. With one
threshold, someone standing near the boundary flaps arrive/leave on
every tick and buries the agent in turns. Crossing in costs 12 m;
crossing back out costs 20 m.

## Cold start

The first run after launch records who is nearby and emits nothing.
Someone already standing next to you when the process starts is not
news — that is the Embodiment block's job — and firing on it would make
every restart look like an arrival.

## Quiet when the world is down

If the world server is unreachable the sensor reports nothing rather
than erroring each cycle. A world that is not running is the normal
case for a chat-only session.
