"""
MiniVillage: a minimal single-agent testbed with simple needs, actions, and
instrumentation for reflective learning research.

No reflection/critic yet—this file focuses on:
  - Deterministic, seedable environment (5x5 grid)
  - Simple needs: hunger and warmth [0..100]
  - Day/Night cycle (configurable)
  - Small action set (<= 10) with success/failure signals
  - Baseline naive policy for demonstration
  - Rich, structured step traces + episode summaries

Standard library only.
"""
from __future__ import annotations

import dataclasses
import enum
import json
import math
import os
import random
from collections import deque
from dataclasses import dataclass, field
import sys
from typing import Deque, Dict, Iterable, List, Optional, Tuple
import logging

import zenoh
#from sentence_transformers import SentenceTransformer
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Messages import SystemMessage, UserMessage
from utils import hash_utils

# Type checking imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/mini_village.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('mini_village')

# Add dedicated handler for llm_api logger


# Add dedicated handler for llm_client logger
llm_client_logger = logging.getLogger('llm_client')
llm_client_file_handler = logging.FileHandler('logs/llm_client.log', mode='a')
llm_client_file_handler.setLevel(logging.INFO)
llm_client_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
llm_client_logger.addHandler(llm_client_file_handler)
llm_client_logger.setLevel(logging.INFO)

from utils.llm_api import LLM

# ──────────────────────────────────────────────────────────────────────────────
# Core types
# ──────────────────────────────────────────────────────────────────────────────


class Tile(enum.Enum):
    EMPTY = 0
    BERRIES = 1
    WOOD = 2
    CAVE = 3


class Action(enum.Enum):
    MOVE_N = "move_n"
    MOVE_S = "move_s"
    MOVE_E = "move_e"
    MOVE_W = "move_w"
    LOOK = "look"
    GATHER_WOOD = "gather_wood"
    EAT_BERRIES = "eat_berries"
    BUILD_FIRE = "build_fire"
    REST = "rest"


@dataclass(frozen=True)
class Pos:
    x: int
    y: int

    def neighbors(self, w: int, h: int) -> List["Pos"]:
        cand = [
            Pos(self.x, self.y - 1),
            Pos(self.x, self.y + 1),
            Pos(self.x + 1, self.y),
            Pos(self.x - 1, self.y),
        ]
        return [p for p in cand if 0 <= p.x < w and 0 <= p.y < h]

    def manhattan(self, other: "Pos") -> int:
        return abs(self.x - other.x) + abs(self.y - other.y)


@dataclass
class AgentState:
    pos: Pos
    hunger: int = 100  # 0..100
    warmth: int = 100  # 0..100
    inventory: Dict[str, int] = field(default_factory=lambda: {"wood": 0})


@dataclass
class StepRecord:
    step: int
    seed: int
    time_of_day: str  # "day" | "night"
    pos: Tuple[int, int]
    hunger: int
    warmth: int
    inventory: Dict[str, int]
    action: str
    valid: bool
    outcome: str
    reward: int
    events: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return dataclasses.asdict(self)


@dataclass
class EpisodeSummary:
    seed: int
    success: bool
    steps: int
    reason: str
    total_reward: int
    berries_eaten: int
    wood_gathered: int
    fires_built: int
    invalid_actions: int


# ──────────────────────────────────────────────────────────────────────────────
# Environment
# ──────────────────────────────────────────────────────────────────────────────


class MiniVillage:
    def __init__(
        self,
        width: int = 5,
        height: int = 5,
        day_length: int = 8,
        night_length: int = 8,
        max_steps: int = 40,
        berries_count: int = 4,
        wood_count: int = 4,
        seed: int = 0,
        fog_of_war: bool = False,
    ) -> None:
        self.w = width
        self.h = height
        self.day_len = day_length
        self.night_len = night_length
        self.cycle_len = self.day_len + self.night_len
        self.max_steps = max_steps
        self.seed = seed
        self.rng = random.Random(seed)
        self.fog_of_war = fog_of_war

        # Grid & placements
        self.grid: List[List[Tile]] = [[Tile.EMPTY for _ in range(self.w)] for _ in range(self.h)]
        self.cave_pos: Pos = self._place_unique(Tile.CAVE)
        self._scatter(Tile.BERRIES, berries_count)
        self._scatter(Tile.WOOD, wood_count)

        # Dynamic state
        start_pos = self._random_empty_pos()
        self.agent = AgentState(pos=start_pos)
        self.step_count = 0
        self.fire_timer = 0  # steps remaining of active fire at cave

        # Stats for summary
        self._berries_eaten = 0
        self._wood_gathered = 0
        self._fires_built = 0
        self._invalid_actions = 0

    # ── map generation helpers ─────────────────────────────────────────────
    def _random_empty_pos(self) -> Pos:
        while True:
            p = Pos(self.rng.randrange(self.w), self.rng.randrange(self.h))
            if self.grid[p.y][p.x] == Tile.EMPTY and p != self.cave_pos:
                return p

    def _place_unique(self, tile: Tile) -> Pos:
        assert tile == Tile.CAVE
        while True:
            p = Pos(self.rng.randrange(self.w), self.rng.randrange(self.h))
            if self.grid[p.y][p.x] == Tile.EMPTY:
                self.grid[p.y][p.x] = tile
                return p

    def _scatter(self, tile: Tile, n: int) -> None:
        count = 0
        while count < n:
            p = Pos(self.rng.randrange(self.w), self.rng.randrange(self.h))
            if self.grid[p.y][p.x] == Tile.EMPTY:
                self.grid[p.y][p.x] = tile
                count += 1

    # ── environment queries ────────────────────────────────────────────────
    def time_of_day(self) -> str:
        phase = self.step_count % self.cycle_len
        return "day" if phase < self.day_len else "night"

    def in_bounds(self, p: Pos) -> bool:
        return 0 <= p.x < self.w and 0 <= p.y < self.h

    def tile_at(self, p: Pos) -> Tile:
        return self.grid[p.y][p.x]

    def is_terminal(self) -> bool:
        a = self.agent
        return a.hunger <= 0 or a.warmth <= 0 or self.step_count >= self.max_steps

    # ── action execution ──────────────────────────────────────────────────
    def step(self, action: Action) -> StepRecord:
        events: List[str] = []
        valid = True
        reward = 0
        outcome = ""
        a = self.agent

        # Execute action semantics
        if action in (Action.MOVE_N, Action.MOVE_S, Action.MOVE_E, Action.MOVE_W):
            outcome = self._do_move(action, events)
        elif action == Action.LOOK:
            outcome = self._do_look()
        elif action == Action.GATHER_WOOD:
            outcome, valid = self._do_gather(events)
        elif action == Action.EAT_BERRIES:
            outcome, valid = self._do_eat(events)
        elif action == Action.BUILD_FIRE:
            outcome, valid = self._do_build_fire(events)
        elif action == Action.REST:
            outcome, valid = self._do_rest(events)
        else:
            valid = False
            outcome = "unknown action"

        if not valid:
            self._invalid_actions += 1

        # Needs dynamics (end-of-step update)
        self._update_needs(events)

        # Reward shaping (simple): alive per step + bonus at day boundary
        if a.hunger > 0 and a.warmth > 0:
            reward += 1
        if self.step_count % self.cycle_len == self.day_len and self.time_of_day() == "night":
            reward += 10  # completed daytime

        # Advance time
        self.step_count += 1

        rec = StepRecord(
            step=self.step_count,
            seed=self.seed,
            time_of_day=self.time_of_day(),
            pos=(a.pos.x, a.pos.y),
            hunger=a.hunger,
            warmth=a.warmth,
            inventory=dict(a.inventory),
            action=action.value,
            valid=valid,
            outcome=outcome,
            reward=reward,
            events=events,
        )
        return rec

    # ── individual actions ────────────────────────────────────────────────
    def _do_move(self, action: Action, events: List[str]) -> str:
        dx, dy = 0, 0
        if action == Action.MOVE_N:
            dy = -1
        elif action == Action.MOVE_S:
            dy = 1
        elif action == Action.MOVE_E:
            dx = 1
        elif action == Action.MOVE_W:
            dx = -1
        newp = Pos(self.agent.pos.x + dx, self.agent.pos.y + dy)
        if not self.in_bounds(newp):
            events.append("invalid_move_oob")
            return "bump into boundary"
        self.agent.pos = newp
        return "moved"

    def _do_look(self) -> str:
        # For now: reveal adjacent tiles (no fog persistence yet)
        adj = {"N": None, "S": None, "E": None, "W": None}
        p = self.agent.pos
        if p.y - 1 >= 0:
            adj["N"] = self.grid[p.y - 1][p.x].name
        if p.y + 1 < self.h:
            adj["S"] = self.grid[p.y + 1][p.x].name
        if p.x + 1 < self.w:
            adj["E"] = self.grid[p.y][p.x + 1].name
        if p.x - 1 >= 0:
            adj["W"] = self.grid[p.y][p.x - 1].name
        return f"adjacent={adj}"

    def _do_gather(self, events: List[str]) -> Tuple[str, bool]:
        tile = self.tile_at(self.agent.pos)
        if tile == Tile.WOOD:
            self.grid[self.agent.pos.y][self.agent.pos.x] = Tile.EMPTY
            self.agent.inventory["wood"] = self.agent.inventory.get("wood", 0) + 1
            self._wood_gathered += 1
            return "gathered wood", True
        events.append("gather_fail_not_on_wood")
        return "no wood here", False

    def _do_eat(self, events: List[str]) -> Tuple[str, bool]:
        tile = self.tile_at(self.agent.pos)
        if tile == Tile.BERRIES:
            self.grid[self.agent.pos.y][self.agent.pos.x] = Tile.EMPTY
            self.agent.hunger = min(100, self.agent.hunger + 30)
            self._berries_eaten += 1
            return "ate berries", True
        events.append("eat_fail_not_on_berries")
        return "no berries here", False

    def _do_build_fire(self, events: List[str]) -> Tuple[str, bool]:
        if self.agent.pos != self.cave_pos:
            events.append("fire_fail_not_in_cave")
            return "not in cave", False
        if self.agent.inventory.get("wood", 0) <= 0:
            events.append("fire_fail_no_wood")
            return "no wood to build fire", False
        self.agent.inventory["wood"] -= 1
        self.fire_timer = 6  # fire lasts for 6 steps
        self._fires_built += 1
        return "built fire", True

    def _do_rest(self, events: List[str]) -> Tuple[str, bool]:
        if self.agent.pos != self.cave_pos:
            events.append("rest_fail_not_in_cave")
            return "not in cave", False
        # resting provides a small warmth boost; bigger if fire is active
        boost = 8 if self.fire_timer > 0 else 3
        self.agent.warmth = min(100, self.agent.warmth + boost)
        return "rested", True

    def _update_needs(self, events: List[str]) -> None:
        a = self.agent
        # Hunger always decays modestly
        a.hunger = max(0, a.hunger - 5)
        tod = self.time_of_day()
        if tod == "night":
            # Warmth decays faster at night; a fire in cave helps only when *in* cave
            if a.pos == self.cave_pos and self.fire_timer > 0:
                a.warmth = min(100, a.warmth + 2)  # recover slowly while by fire
            else:
                a.warmth = max(0, a.warmth - 10)
        else:
            # Daytime recovery is mild
            a.warmth = min(100, a.warmth + 1)

        if self.fire_timer > 0:
            self.fire_timer -= 1

    # ── utility for planners/policies ──────────────────────────────────────
    def find_nearest(self, start: Pos, target: Tile) -> Optional[List[Pos]]:
        """Breadth-first search path from start to the nearest cell with target tile.
        Returns the path (including start and goal) or None if unreachable.
        """
        if self.tile_at(start) == target:
            return [start]
        q: Deque[Pos] = deque([start])
        prev: Dict[Pos, Optional[Pos]] = {start: None}
        while q:
            p = q.popleft()
            for nb in p.neighbors(self.w, self.h):
                if nb in prev:
                    continue
                prev[nb] = p
                if self.tile_at(nb) == target:
                    # reconstruct
                    path = [nb]
                    cur = p
                    while cur is not None:
                        path.append(cur)
                        cur = prev[cur]
                    path.reverse()
                    return path
                q.append(nb)
        return None


# ──────────────────────────────────────────────────────────────────────────────
# LLM-driven policy via prompt templating (object-level action selection)
# ──────────────────────────────────────────────────────────────────────────────

class LLMClient:
    """Minimal interface the host can implement. Replace `ask` with your
    actual model call (OpenAI, vLLM, etc.). Expected to return a raw string.
    """
    llm = LLM(server_name="vllm", model_name="llama3.1-8b-instruct")

    @staticmethod
    def ask(system: str, user: str, *, max_tokens: int = 200, stops: Optional[List[str]] = None, temp: float = 0.0) -> str:
        return LLMClient.llm.ask({}, [SystemMessage(content=system), UserMessage(content=user)], max_tokens=max_tokens, stops=stops, temp=temp)

def state_snapshot(env: MiniVillage) -> Dict[str, object]:
    a = env.agent
    return {
        "seed": env.seed,
        "step": env.step_count,
        "time_of_day": env.time_of_day(),
        "pos": {"x": a.pos.x, "y": a.pos.y},
        "hunger": a.hunger,
        "warmth": a.warmth,
        "inventory": dict(a.inventory),
        "in_cave": (a.pos == env.cave_pos),
        "on_tile": env.tile_at(a.pos).name,
        "distances": {
            "to_cave": a.pos.manhattan(env.cave_pos),
            "to_nearest_berries": _nearest_dist(env, Tile.BERRIES),
            "to_nearest_wood": _nearest_dist(env, Tile.WOOD),
        },
        "fire_active": env.fire_timer > 0,
        "map_size": {"w": env.w, "h": env.h},
        "max_steps": env.max_steps,
    }


def _nearest_dist(env: MiniVillage, t: Tile) -> Optional[int]:
    path = env.find_nearest(env.agent.pos, t)
    if path is None:
        return None
    return len(path) - 1


_ACTION_DOCS: Dict[Action, str] = {
    Action.MOVE_N: "Move north by one cell (must remain in bounds).",
    Action.MOVE_S: "Move south by one cell (must remain in bounds).",
    Action.MOVE_E: "Move east by one cell (must remain in bounds).",
    Action.MOVE_W: "Move west by one cell (must remain in bounds).",
    Action.LOOK: "Look around to reveal adjacent tiles.",
    Action.GATHER_WOOD: "Gather wood if standing on a WOOD tile.",
    Action.EAT_BERRIES: "Eat berries if standing on a BERRIES tile (restores hunger).",
    Action.BUILD_FIRE: "Build a fire if in the CAVE and you have >=1 wood (warms you at night).",
    Action.REST: "Rest if in the CAVE (minor warmth recovery; more if fire is active).",
}


def build_llm_prompts(env: MiniVillage) -> Tuple[str, str]:
    """Return (system_prompt, user_prompt) for object-level action selection.

    Contract: the model must emit one single-line JSON object with keys:
      {"action": <one of Action values>, "reason": <short string>}
    where <Action values> are the enum values (e.g., "eat_berries", "move_n").
    """
    state = state_snapshot(env)

    system = (
        "You are a careful action selector for a tiny survival world. "
        "Choose exactly one next action that keeps the agent alive and progresses goals. "
        "You must obey preconditions and stay within bounds. "
        "Respond ONLY as compact JSON: {\"action\": ..., \"reason\": ...}."
    )

    # Enumerate allowed actions with concise docs; include soft priorities for clarity
    actions_sec = []
    for a in Action:
        actions_sec.append({"name": a.value, "doc": _ACTION_DOCS[a]})

    constraints = [
        "If on BERRIES, prefer EAT_BERRIES when hunger < 70.",
        "At night, being in the CAVE is safer; resting or building a fire are valid only in the cave.",
        "BUILD_FIRE requires at least one wood in inventory.",
        "GATHER_WOOD only works on a WOOD tile.",
        "All MOVE_* must remain within the map bounds.",
        "LOOK is safe if no obvious productive action applies.",
    ]

    user = (
        "# STATE\n" + json.dumps(state, separators=(",", ":")) \
        + "\n# ACTIONS\n" + json.dumps(actions_sec, separators=(",", ":")) \
        + "\n# CONSTRAINTS\n" + json.dumps(constraints, separators=(",", ":")) \
        + "\nEmit ONE SINGLE LINE of JSON with the keys 'action' and 'reason'; no markdown, no extra text. end your response with </end>"
    )
    return system, user

def parse_llm_action(raw: str) -> Tuple[Optional[Action], str]:
    """Parse the model's raw output into (Action|None, reason).
    Returns (None, reason) if parsing fails or action is unknown.
    """
    raw = raw.strip()
    # Best effort: find first JSON object
    start = raw.find("{")
    end = raw.rfind("}")
    if start == -1 or end == -1 or end <= start:
        return None, f"invalid_json: {raw[:80]}"
    try:
        obj = json.loads(raw[start:end + 1])
        act = str(obj.get("action", "")).lower().strip()
        reason = str(obj.get("reason", ""))
        for a in Action:
            if act == a.value:
                return a, reason
        return None, f"unknown_action:{act}"
    except Exception as e:
        return None, f"json_error:{type(e).__name__}"


class LLMPolicy:
    def __init__(self, client: LLMClient, *, temperature: float = 0.0, max_tokens: int = 120):
        self.client = client
        self.temperature = temperature
        self.max_tokens = max_tokens

    def decide(self, env: MiniVillage) -> Action:
        sys_p, usr_p = build_llm_prompts(env)
        raw = self.client.ask(sys_p, usr_p, max_tokens=self.max_tokens, stops=["</end>"], temp=self.temperature)
        action, reason = parse_llm_action(raw)

        # Validate preconditions; if invalid, fall back to a safe default (LOOK or bounded move).
        if action is None or not self._precond_ok(env, action):
            # Simple conservative fallback
            if env.tile_at(env.agent.pos) == Tile.BERRIES:
                return Action.EAT_BERRIES
            return Action.LOOK
        return action

    def _precond_ok(self, env: MiniVillage, action: Action) -> bool:
        a = env.agent
        if action in (Action.MOVE_N, Action.MOVE_S, Action.MOVE_E, Action.MOVE_W):
            dx, dy = 0, 0
            if action == Action.MOVE_N:
                dy = -1
            elif action == Action.MOVE_S:
                dy = 1
            elif action == Action.MOVE_E:
                dx = 1
            elif action == Action.MOVE_W:
                dx = -1
            np = Pos(a.pos.x + dx, a.pos.y + dy)
            return env.in_bounds(np)
        if action == Action.GATHER_WOOD:
            return env.tile_at(a.pos) == Tile.WOOD
        if action == Action.EAT_BERRIES:
            return env.tile_at(a.pos) == Tile.BERRIES
        if action == Action.BUILD_FIRE:
            return (a.pos == env.cave_pos) and (a.inventory.get("wood", 0) > 0)
        if action == Action.REST:
            return a.pos == env.cave_pos
        if action == Action.LOOK:
            return True
        return False


# ──────────────────────────────────────────────────────────────────────────────
# Instrumented runner
# ──────────────────────────────────────────────────────────────────────────────


def run_episode(seed: int, max_steps: int = 40, policy_factory=None, client: Optional[LLMClient] = None) -> Tuple[List[StepRecord], EpisodeSummary, MiniVillage]:
    env = MiniVillage(seed=seed, max_steps=max_steps)
    if policy_factory is None:
        # Default to a conservative LLM policy if a client is provided; else fallback to a trivial random walk using LOOK safety.
        if client is not None:
            policy = LLMPolicy(client)
        else:
            policy = None
    else:
        policy = policy_factory(env)

    trace: List[StepRecord] = []
    total_reward = 0
    reason = ""
    while not env.is_terminal():
        if policy is None:
            # Minimal safe heuristic if no policy provided
            action = Action.LOOK if env.rng.random() < 0.3 else random.choice([Action.MOVE_N, Action.MOVE_S, Action.MOVE_E, Action.MOVE_W])
            # Keep moves in-bounds
            if action != Action.LOOK and not LLMPolicy._precond_ok(LLMPolicy.__new__(LLMPolicy), env, action):
                action = Action.LOOK
        else:
            action = policy.decide(env)
        rec = env.step(action)
        total_reward += rec.reward
        trace.append(rec)

    a = env.agent
    if a.hunger <= 0:
        reason = "starved"
    elif a.warmth <= 0:
        reason = "froze"
    else:
        reason = "max_steps"

    summ = EpisodeSummary(
        seed=seed,
        success=(reason == "max_steps"),
        steps=len(trace),
        reason=reason,
        total_reward=total_reward,
        berries_eaten=env._berries_eaten,
        wood_gathered=env._wood_gathered,
        fires_built=env._fires_built,
        invalid_actions=env._invalid_actions,
    )
    return trace, summ, env


def run_many(seeds: Iterable[int]) -> Dict[str, object]:
    all_summaries: List[EpisodeSummary] = []
    for s in seeds:
        _, summ, _ = run_episode(seed=s, client=LLMClient)
        all_summaries.append(summ)

    # Aggregate metrics
    success_rate = sum(1 for s in all_summaries if s.success) / len(all_summaries)
    median_steps = sorted(s.steps for s in all_summaries)[len(all_summaries) // 2]
    invalid_rate = sum(s.invalid_actions for s in all_summaries) / max(1, sum(s.steps for s in all_summaries))

    return {
        "n_episodes": len(all_summaries),
        "success_rate": success_rate,
        "median_steps": median_steps,
        "avg_invalid_actions_per_step": invalid_rate,
        "summaries": [dataclasses.asdict(s) for s in all_summaries],
    }


# ──────────────────────────────────────────────────────────────────────────────
# Simple demo when executed as a script (optional)
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    seeds = list(range(10))
    agg = run_many(seeds)
    print(json.dumps(agg, indent=2))
