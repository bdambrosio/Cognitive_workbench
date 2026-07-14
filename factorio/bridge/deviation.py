"""Deviation classification: world_changed | stale_model | infeasible.

On an action failure the bridge re-observes the target area and compares
it with its last observation of the same area (architecture note,
"Deviation reports"):

- fresh prior observation that contradicts the world now -> world_changed
  (someone acted; a coordination event, often worth a chat line)
- no prior observation, or one older than OBS_TTL_S -> stale_model
  (re-observe, don't chat)
- fresh prior observation that matches -> infeasible
  (the request itself is impossible)
"""
import time
from collections import deque

OBS_TTL_S = 45.0
_NEIGHBORHOOD = 2.5  # tiles around the target compared for change


class ObservationCache:
    """Recent observations: absolute positions + timestamps (mc-* ledger lesson #5)."""

    def __init__(self):
        self._obs = deque(maxlen=256)

    def record(self, x, y, radius, entity_keys):
        entry = {
            "t": time.time(),
            "x": x,
            "y": y,
            "radius": radius,
            "ents": frozenset(entity_keys),
        }
        self._obs.append(entry)
        return entry

    def latest_covering(self, x, y, exclude=None):
        best = None
        for obs in self._obs:
            if obs is exclude:
                continue
            if ((x - obs["x"]) ** 2 + (y - obs["y"]) ** 2) ** 0.5 <= obs["radius"]:
                if best is None or obs["t"] > best["t"]:
                    best = obs
        return best


def entity_keys(entities):
    """(name, x, y) identity keys, positions snapped to half-tiles."""
    keys = set()
    for e in entities or []:
        if not isinstance(e, dict):
            continue
        pos = e.get("position") or {}
        try:
            keys.add((
                e.get("name"),
                round(float(pos.get("x")) * 2) / 2,
                round(float(pos.get("y")) * 2) / 2,
            ))
        except (TypeError, ValueError):
            continue
    return keys


def _near(keys, x, y, r=_NEIGHBORHOOD):
    return {k for k in keys if ((k[1] - x) ** 2 + (k[2] - y) ** 2) ** 0.5 <= r}


def _fmt(keys):
    return sorted("%s@(%s,%s)" % k for k in keys)


def classify(cache: ObservationCache, x, y, error: str, observe_now) -> dict:
    """Classify a failed action at (x, y). observe_now(x, y, radius) must
    re-observe via the mod, record into the cache, and return
    (entities, cache_entry)."""
    entities, entry = observe_now(x, y, 4)
    prior = cache.latest_covering(x, y, exclude=entry)
    now_keys = _near(entity_keys(entities), x, y)
    deviation = {
        "error": error,
        "target": {"x": x, "y": y},
        "observed_now": _fmt(now_keys),
    }
    if prior is None or entry["t"] - prior["t"] > OBS_TTL_S:
        deviation["kind"] = "stale_model"
        deviation["detail"] = "no recent observation of the target area; re-observe before retrying"
    elif _near(prior["ents"], x, y) != now_keys:
        deviation["kind"] = "world_changed"
        deviation["detail"] = (
            "target area changed since it was observed %.0fs ago; someone acted here"
            % (entry["t"] - prior["t"])
        )
        deviation["last_observed"] = _fmt(_near(prior["ents"], x, y))
    else:
        deviation["kind"] = "infeasible"
        deviation["detail"] = "observation is current and unchanged; the request itself is impossible"
    return deviation
