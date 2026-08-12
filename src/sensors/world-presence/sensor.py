"""world-presence — fires when someone comes near, or moves away.

Edge-triggered, level-carrying: it stays silent while the world is
quiescent, and when it does fire it ships the whole current situation so
the woken turn does not have to spend an iteration on world-look.

Per-character baselines live in module state. That is deliberately not
persisted: a fresh process means a fresh baseline, seeded silently on the
first run, which is exactly the behaviour that keeps a restart from
re-reporting everyone standing nearby as a new arrival (the failure mode
rss-watcher's in-memory dedup has for feed titles).
"""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.world_link import BridgeError, compass, world_get  # noqa: E402

logger = logging.getLogger(__name__)

_DEFAULT_ENTER_M = 12.0
_DEFAULT_EXIT_M = 20.0
_LOOK_RADIUS_M = 40.0

# character -> set of names currently counted as near. Absent means the
# baseline has not been seeded yet for this process.
_near: dict = {}

# character -> whether it had a walking goal at the last poll. The other
# edge this sensor watches: goal set, then goal gone, means arrived.
#
# Without it an agent that posts a goal and ends its turn is asleep until
# somebody walks past it. Nothing else wakes it: proximity is about other
# occupants, and a searcher walking away from its partner meets nobody.
# Observed 2026-08-12 three times in one run — an agent reached its
# sector, had the ground in front of it, and said nothing until it was
# prodded. Since travel is never blocked (terrain only slows), a goal that
# clears means the walk finished; it has no other way to end.
#
# A walk that starts and finishes between two polls raises no event. That
# is correct: it was shorter than the poll interval, so nobody waited.
_walking: dict = {}

_NOTHING = {'status': 'nothing', 'content': '', 'metadata': {}}

# character -> reason this sensor is currently blind, or absent while the
# world answers. One log line per outage edge: enough to tell a working
# sensor with nothing to report from one that cannot see at all.
_blind: dict = {}


def _note_outage(me: str, reason: str) -> None:
    if _blind.get(me) is None:
        logger.warning(f"world-presence[{me}]: {reason} — no presence events "
                       f"will be emitted until the world answers; "
                       f"suppressing repeats")
        _blind[me] = reason
    else:
        logger.debug(f"world-presence[{me}]: still blind: {reason}")


def _clear_outage(me: str) -> None:
    if _blind.get(me) is not None:
        logger.warning(f"world-presence[{me}]: world answering again "
                       f"(was: {_blind[me]})")
        _blind[me] = None


def _describe(look: dict, arrived: list, departed: list,
              reached: bool = False) -> str:
    """A self-contained situation report.

    Non-tick sensors arrive as user-like turns, so this has to read as an
    event the agent is noticing rather than as something anyone said.
    """
    me = look['me']
    lines = []
    if reached:
        lines.append(f"You have arrived at ({me['pos'][0]}, {me['pos'][1]}) "
                     f"— the walk you posted is finished.")
    for name in arrived:
        lines.append(f"{name} has come near you in the world.")
    for name in departed:
        lines.append(f"{name} has moved away from you in the world.")

    lines.append("")
    lines.append(f"You are at ({me['pos'][0]}, {me['pos'][1]}) facing "
                 f"{compass(me['heading_deg'])} — {look['ground']}.")
    if me.get('goal'):
        lines.append(f"You are walking toward "
                     f"({me['goal'][0]:.1f}, {me['goal'][1]:.1f}).")

    others = look.get('occupants') or []
    if others:
        for o in others:
            gait = 'walking' if o.get('gait') == 'walk' else 'standing still'
            note = ', looking at you' if o.get('looking_at_me') else ''
            lines.append(f"  {o['name']} — {o['distance_m']} m to the "
                         f"{compass(o['bearing_deg'])}, at "
                         f"({o['pos'][0]}, {o['pos'][1]}), {gait}{note}")

    lines.append("")
    lines.append("Nobody has said anything — you noticed this yourself. "
                 "Reply only if it is worth remarking on or acting on; "
                 "otherwise stay silent.")
    return "\n".join(lines)


def run(context):
    me = context.get('character_name') or ''
    if not me:
        return _NOTHING
    params = context.get('parameters') or {}
    enter_m = float(params.get('enter_m', _DEFAULT_ENTER_M))
    exit_m = float(params.get('exit_m', _DEFAULT_EXIT_M))

    try:
        look = world_get('/look', {'name': me, 'radius': _LOOK_RADIUS_M})
    except BridgeError as e:
        # A world that is not running is the normal case for a chat-only
        # session, so this is not an error every cycle — but it must be
        # visible ONCE. At DEBUG it wasn't: a sensor silently returning
        # NOTHING because the world is unreachable looked exactly like one
        # returning NOTHING because nobody moved, and the logs could not
        # tell "working, quiet" from "not working at all".
        _note_outage(me, f"world unreachable: {e}")
        return _NOTHING
    if look.get('error'):
        _note_outage(me, str(look['error']))
        return _NOTHING
    _clear_outage(me)

    was_near = _near.get(me)
    prior = was_near or set()

    # Hysteresis: crossing in costs enter_m, crossing back out costs exit_m,
    # so someone loitering at the boundary does not flap every tick.
    now_near = set()
    for o in look.get('occupants') or []:
        threshold = exit_m if o['name'] in prior else enter_m
        if float(o['distance_m']) <= threshold:
            now_near.add(o['name'])

    was_walking = _walking.get(me)
    now_walking = bool((look.get('me') or {}).get('goal'))

    if was_near is None:
        # Cold start: seed both baselines, say nothing. Who is already here
        # is the Embodiment block's job, not an arrival — and a walk already
        # in progress at seeding time will still report when it ends.
        _near[me] = now_near
        _walking[me] = now_walking
        logger.info(f"world-presence[{me}]: baseline seeded "
                    f"({len(now_near)} nearby, walking={now_walking}), "
                    f"no event emitted")
        return _NOTHING

    arrived = sorted(now_near - prior)
    departed = sorted(prior - now_near)
    reached = bool(was_walking) and not now_walking
    _near[me] = now_near
    _walking[me] = now_walking
    if not arrived and not departed and not reached:
        return _NOTHING

    logger.info(f"world-presence[{me}]: arrived={arrived} "
                f"departed={departed} reached_goal={reached}")
    return {
        'status': 'ok',
        'content': _describe(look, arrived, departed, reached),
        'metadata': {'arrived': arrived, 'departed': departed,
                     'reached_goal': reached, 'near': sorted(now_near)},
    }
