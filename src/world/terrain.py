"""terrain — the seeded heightmap both the renderer and the agents read.

Generated once, in Python, and shipped to the browser. The renderer never
generates anything itself: a second generator in JS would drift from this
one, and then what Jill reasons about would stop matching what Bruce sees.

World coordinates are metres, x/z horizontal (y is up, matching three.js),
spanning [-EXTENT_M/2, +EXTENT_M/2] on both axes.
"""
from __future__ import annotations

import base64
import hashlib
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

EXTENT_M = 384.0          # world is EXTENT_M square
GRID_N = 385              # heightmap samples per axis (~1 m resolution)
# Peak-to-trough relief. Scale this with EXTENT_M: the same relief spread
# over a wider world is a flatter world, and terrain stops occluding
# anything. At 384 m with the old 14 m, only 1% of in-range pairs were
# blocked by a rise and the fog degenerated into a plain radius.
HEIGHT_SCALE_M = 22.0
WATER_Y = 1.15            # anything below this is pond
MAX_WALK_SLOPE = 0.62     # rise/run above which ground is unfit to stand on

# Travel cost. Nothing is impassable — ground is only ever slower — and the
# whole band is 2:1, because the point is to break the identity between a
# sector's area and the time to search it, not to model terrain. Once area
# stops predicting time, an even split is the wrong split, one searcher
# finishes early, and the only way either learns that is to walk it and say
# so. That is the conversation the world exists to provoke.
#
# The weights are graded against what this generator actually produces:
# slope sits between 0.05 and 0.25 over most of the map and only 9% of it
# is steeper than 0.25, while the forest field is uniform by construction
# and covers half the world. So forest carries most of the cost — and it
# carries it where sight_range() has already cut vision to 22 m, which is
# the useful part: the slow ground is the blind ground, so a partner's
# report is worth most exactly where you can least check it.
SLOPE_EASY = 0.05         # at or below this, slope costs nothing
SLOPE_HARD = 0.30         # at or above this, slope costs all it can
SLOPE_COST = 0.20         # speed lost to the steepest ground
FOREST_COST = 0.30        # speed lost to the densest forest
MIN_SPEED_FACTOR = 0.50   # wading, and the floor for everything else

# Sight. Without a limit, world-look hands every agent every position and
# there is never a reason to ask anyone anything — communication becomes
# decorative. A radius plus terrain occlusion is what gives "what can you
# see from there?" an answer worth having.
EYE_H_M = 1.6
OPEN_SIGHT_M = 70.0       # how far you see standing in the open
FOREST_SIGHT_M = 22.0     # standing among trees

# Candidates before biome rejection; roughly a third survive, all of them
# in the forest half. InstancedMesh draws the survivors in one call.
# Scaled with area so density stays constant when EXTENT_M changes.
TREE_TARGET = 21000
ROCK_TARGET = 1400


def _upsample(lattice: np.ndarray, n: int) -> np.ndarray:
    """Smoothstep-interpolate a small random lattice up to n x n."""
    cells = lattice.shape[0] - 1
    xs = np.linspace(0.0, cells, n)
    i0 = np.clip(np.floor(xs).astype(int), 0, cells - 1)
    i1 = i0 + 1
    t = xs - i0
    t = t * t * (3.0 - 2.0 * t)                     # smoothstep
    rows = lattice[i0, :] * (1 - t)[:, None] + lattice[i1, :] * t[:, None]
    return rows[:, i0] * (1 - t)[None, :] + rows[:, i1] * t[None, :]


def _fbm(rng: np.random.Generator, n: int, octaves: Tuple[int, ...]) -> np.ndarray:
    """Fractal value noise in [0, 1] — coarse octaves dominate."""
    total = np.zeros((n, n), dtype=np.float64)
    amplitude, norm = 1.0, 0.0
    for cells in octaves:
        total += amplitude * _upsample(rng.random((cells + 1, cells + 1)), n)
        norm += amplitude
        amplitude *= 0.5
    return total / norm


def _rank_uniform(field: np.ndarray) -> np.ndarray:
    """Map a field onto uniform [0, 1] by rank, preserving its shape."""
    flat = field.ravel()
    order = flat.argsort()
    ranks = np.empty(flat.size, dtype=np.float64)
    ranks[order] = np.arange(flat.size)
    return (ranks / max(flat.size - 1, 1)).reshape(field.shape)


@dataclass
class Terrain:
    seed: int
    heights: np.ndarray       # (GRID_N, GRID_N) metres, indexed [iz, ix]
    forest: np.ndarray        # (GRID_N, GRID_N) in [0,1]; high = forest
    trees: List[dict]
    rocks: List[dict]

    # -- lookups shared by the server, the agent tools and the tests ----

    def _grid_coords(self, x: float, z: float) -> Tuple[float, float]:
        half = EXTENT_M / 2.0
        gx = (x + half) / EXTENT_M * (GRID_N - 1)
        gz = (z + half) / EXTENT_M * (GRID_N - 1)
        return (float(np.clip(gx, 0, GRID_N - 1)),
                float(np.clip(gz, 0, GRID_N - 1)))

    def height_at(self, x: float, z: float) -> float:
        """Bilinear height, matching what the renderer's mesh interpolates."""
        gx, gz = self._grid_coords(x, z)
        x0, z0 = int(gx), int(gz)
        x1, z1 = min(x0 + 1, GRID_N - 1), min(z0 + 1, GRID_N - 1)
        tx, tz = gx - x0, gz - z0
        h0 = self.heights[z0, x0] * (1 - tx) + self.heights[z0, x1] * tx
        h1 = self.heights[z1, x0] * (1 - tx) + self.heights[z1, x1] * tx
        return float(h0 * (1 - tz) + h1 * tz)

    def slope_at(self, x: float, z: float) -> float:
        """Local gradient magnitude (rise over run), sampled a metre apart."""
        d = 1.0
        dx = (self.height_at(x + d, z) - self.height_at(x - d, z)) / (2 * d)
        dz = (self.height_at(x, z + d) - self.height_at(x, z - d)) / (2 * d)
        return float(np.hypot(dx, dz))

    def in_bounds(self, x: float, z: float) -> bool:
        half = EXTENT_M / 2.0 - 1.0
        return -half <= x <= half and -half <= z <= half

    def walkable(self, x: float, z: float) -> bool:
        """Ground fit to be *placed* on — spawns and props. Not a movement
        test: travel is never blocked, only slowed (see speed_factor)."""
        if not self.in_bounds(x, z):
            return False
        if self.height_at(x, z) < WATER_Y:
            return False                       # pond
        return self.slope_at(x, z) <= MAX_WALK_SLOPE

    def forest_at(self, x: float, z: float) -> float:
        """Raw forest density in [0, 1]. biome_at is this thresholded; the
        continuous value is what travel cost reads, so the treeline slows
        you gradually instead of at a step."""
        gx, gz = self._grid_coords(x, z)
        return float(self.forest[int(gz), int(gx)])

    def speed_factor(self, x: float, z: float) -> float:
        """Fraction of walking pace this ground allows, in
        [MIN_SPEED_FACTOR, 1.0]. Water is the slowest thing there is, and
        it is still passable — you wade."""
        if self.height_at(x, z) < WATER_Y:
            return MIN_SPEED_FACTOR
        slope_t = (self.slope_at(x, z) - SLOPE_EASY) / (SLOPE_HARD - SLOPE_EASY)
        cost = (SLOPE_COST * min(max(slope_t, 0.0), 1.0)
                + FOREST_COST * min(max(self.forest_at(x, z), 0.0), 1.0))
        return max(1.0 - cost, MIN_SPEED_FACTOR)

    def biome_at(self, x: float, z: float) -> str:
        gx, gz = self._grid_coords(x, z)
        return 'forest' if self.forest[int(gz), int(gx)] > 0.5 else 'plains'

    def sight_range(self, x: float, z: float) -> float:
        """How far the observer can see from here. Standing among trees
        cuts it hard — this is what makes a scouting report worth
        something."""
        return (FOREST_SIGHT_M if self.biome_at(x, z) == 'forest'
                else OPEN_SIGHT_M)

    def line_of_sight(self, x1: float, z1: float,
                      x2: float, z2: float) -> bool:
        """Is the straight line between two standing figures unblocked?

        Marches the segment at roughly metre steps and fails as soon as
        the ground rises above eye-to-eye. Terrain only — individual trees
        are not occluders, which is why sight_range does the work of
        modelling foliage instead.
        """
        dist = float(np.hypot(x2 - x1, z2 - z1))
        if dist < 1.0:
            return True
        y1 = self.height_at(x1, z1) + EYE_H_M
        y2 = self.height_at(x2, z2) + EYE_H_M
        steps = int(dist)
        for i in range(1, steps):
            t = i / steps
            x = x1 + (x2 - x1) * t
            z = z1 + (z2 - z1) * t
            if self.height_at(x, z) > y1 + (y2 - y1) * t:
                return False
        return True

    def can_see(self, x1: float, z1: float, x2: float, z2: float) -> bool:
        """Range and terrain together, from the first point's vantage."""
        if float(np.hypot(x2 - x1, z2 - z1)) > self.sight_range(x1, z1):
            return False
        return self.line_of_sight(x1, z1, x2, z2)

    def describe(self, x: float, z: float) -> str:
        """One line of terrain context, for agent perception.

        Carries the going as well as the look of the place. Without it an
        occupant has no way to notice its sector is slow except by tracking
        elapsed time, and then there is nothing to report to a partner but
        lateness."""
        h = self.height_at(x, z)
        slope = self.slope_at(x, z)
        grade = ('flat' if slope < 0.08 else
                 'gently sloping' if slope < 0.25 else
                 'steep')
        factor = self.speed_factor(x, z)
        going = ('wading' if h < WATER_Y else
                 'heavy going' if factor < 0.7 else
                 'slow going' if factor < 0.88 else
                 'easy going')
        return (f"{self.biome_at(x, z)}, {grade} ground, "
                f"elevation {h:.1f} m, {going} "
                f"({factor * 100:.0f}% of walking pace)")

    # -- wire format ----------------------------------------------------

    def payload(self) -> Dict:
        """Compact one-shot payload for the renderer. Heights go as base64
        float32 rather than a JSON array — 66k numbers is ~600 KB of text
        and ~264 KB of bytes."""
        heights32 = self.heights.astype(np.float32).tobytes()
        # Forest goes too, because the browser has to compute the same
        # speed_factor the server does — a second formula over different
        # data is the drift this module exists to prevent. uint8 is ample
        # for a speed multiplier and half the bytes of float32; it leaves
        # the two sides disagreeing by at most FOREST_COST/255 ≈ 0.0012,
        # measured, against a band of 0.5. Heights stay float32 because
        # the mesh is built from them and the eye is less forgiving.
        forest8 = (np.clip(self.forest, 0.0, 1.0) * 255.0).astype(np.uint8)
        return {
            'seed': self.seed,
            'extent_m': EXTENT_M,
            'grid_n': GRID_N,
            'water_y': WATER_Y,
            'heights_b64': base64.b64encode(heights32).decode('ascii'),
            'forest_b64': base64.b64encode(forest8.tobytes()).decode('ascii'),
            'speed': {'slope_easy': SLOPE_EASY, 'slope_hard': SLOPE_HARD,
                      'slope_cost': SLOPE_COST, 'forest_cost': FOREST_COST,
                      'min_factor': MIN_SPEED_FACTOR},
            'trees': self.trees,
            'rocks': self.rocks,
        }

    def fingerprint(self) -> str:
        """Stable hash of the generated terrain — the determinism guard."""
        h = hashlib.sha256()
        h.update(self.heights.astype(np.float32).tobytes())
        h.update(str(len(self.trees)).encode())
        h.update(str(len(self.rocks)).encode())
        return h.hexdigest()[:16]


def _scatter(rng: np.random.Generator, terrain_heights: np.ndarray,
             forest: np.ndarray, count: int, min_forest: float,
             kinds: int) -> List[dict]:
    """Rejection-sample props onto walkable, in-biome ground."""
    half = EXTENT_M / 2.0
    xs = rng.uniform(-half + 2, half - 2, count)
    zs = rng.uniform(-half + 2, half - 2, count)
    scales = rng.uniform(0.75, 1.45, count)
    rots = rng.uniform(0, 2 * np.pi, count)
    kind_ids = rng.integers(0, kinds, count)

    gi = ((xs + half) / EXTENT_M * (GRID_N - 1)).astype(int)
    gj = ((zs + half) / EXTENT_M * (GRID_N - 1)).astype(int)
    gi = np.clip(gi, 0, GRID_N - 1)
    gj = np.clip(gj, 0, GRID_N - 1)
    ys = terrain_heights[gj, gi]
    dens = forest[gj, gi]

    # Denser where the biome field is stronger; nothing in the water.
    keep = (ys > WATER_Y + 0.3) & (dens >= min_forest)
    keep &= rng.random(count) < np.clip((dens - min_forest) * 2.2 + 0.15, 0, 1)

    return [
        {'x': round(float(xs[i]), 2), 'y': round(float(ys[i]), 2),
         'z': round(float(zs[i]), 2), 's': round(float(scales[i]), 2),
         'r': round(float(rots[i]), 3), 'k': int(kind_ids[i])}
        for i in np.nonzero(keep)[0]
    ]


def generate(seed: int = 20260808) -> Terrain:
    """Build the world. Same seed always yields the same terrain."""
    rng = np.random.default_rng(seed)

    relief = _fbm(rng, GRID_N, (2, 4, 8, 16, 32))
    heights = (relief - relief.min()) / max(float(np.ptp(relief)), 1e-9)
    heights = heights * HEIGHT_SCALE_M

    # Independent low-frequency field decides forest vs plains, so the
    # treeline doesn't simply track elevation. Percentile-normalised
    # rather than min-max: raw fBm clusters around its mean, which turned
    # a 0.55 cut into an 85% forest world. Uniform ranks make the
    # threshold mean what it says — 0.5 is half the map.
    forest = _rank_uniform(_fbm(rng, GRID_N, (2, 4, 8)))

    trees = _scatter(rng, heights, forest, TREE_TARGET, 0.5, kinds=2)
    # Rocks favour the open half — 1 - forest is uniform there too.
    rocks = _scatter(rng, heights, 1.0 - forest, ROCK_TARGET, 0.5, kinds=1)

    return Terrain(seed=seed, heights=heights, forest=forest,
                   trees=trees, rocks=rocks)
