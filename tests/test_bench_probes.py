"""Tests for the benchmarks-v2 scorers.

A scorer that is quietly wrong corrupts every row it ever produces, and unlike
a broken feature nothing downstream complains. These pin the arithmetic; the
runners (which need a live backend) are not covered here.

The tic-tac-toe cases replay the real 2026-08-18 Jill-vs-Jack game, so the
rules engine is checked against a game that actually happened rather than one
invented to match the implementation.
"""

import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from bench.convergence.score import _cells_match, _truth_cell
from bench.yield_probe.score import score_yield
from bench.tictactoe import game
from bench.tictactoe.score import score as ttt_score

GROUND_TRUTH = json.loads(
    (REPO / "bench" / "convergence" / "ground_truth.json").read_text())


def _path(key):
    return next(p for p in GROUND_TRUTH["paths"] if p["key"] == key)


# --------------------------------------------------------------------------
# The live game, as a fixture for the rules engine
# --------------------------------------------------------------------------

LIVE_GAME = [(5, "X"), (1, "O"), (9, "X"), (3, "O"), (2, "X"),
             (8, "O"), (7, "X"), (4, "O"), (6, "X")]


def _played(moves):
    b = game.new_board()
    for sq, mark in moves:
        b[sq] = mark
    return b


def test_live_game_was_a_genuine_draw():
    assert game.result(_played(LIVE_GAME)) == "draw"


def test_live_game_move_three_was_a_forced_block():
    """O at 1 and 3 threatens the top row; 2 is the only block."""
    assert game.forced_blocks(_played(LIVE_GAME[:4]), "X") == [2]


def test_opponent_is_deterministic():
    b = game.new_board()
    b[5] = "X"
    assert game.opponent_move(b) == game.opponent_move(dict(b))


def test_opponent_takes_a_win_over_a_block():
    b = game.new_board()
    b[1] = b[2] = "O"        # O can win at 3
    b[4] = b[5] = "X"        # X threatens 6
    assert game.opponent_move(b, "O") == 3


# --------------------------------------------------------------------------
# Convergence cell matching
# --------------------------------------------------------------------------

@pytest.mark.parametrize("cell,claimed,key,expect", [
    ("activation", 0.0, "seed", True),
    ("activation", 0.7, "seed", False),
    ("activation", 0.70, "successor", True),
    ("activation", 0.0, "successor", False),
    ("rhythm_hours", "from_yaml", "seed", True),
    ("rhythm_hours", 1, "seed", False),
    ("rhythm_hours", 1, "successor", True),
    ("rhythm_source", "External", "seed", True),
    ("rhythm_source", "urgency", "seed", False),
    ("fires_without_autonomy", False, "successor", True),
    ("fires_without_autonomy", True, "successor", False),
])
def test_cell_matching(cell, claimed, key, expect):
    p = _path(key)
    assert _cells_match(cell, claimed, _truth_cell(p, cell)) is expect


def test_unstated_cell_is_never_correct():
    """None means the report did not say. It must not score as agreement."""
    for p in GROUND_TRUTH["paths"]:
        for cell in ("activation", "rhythm_hours", "rhythm_source",
                     "fires_without_autonomy"):
            assert _cells_match(cell, None, _truth_cell(p, cell)) is False


def test_every_path_needs_autonomy_to_fire():
    """The gate is unconditional (chat_loop.py:2211). The user-yield path is
    CREATED outside the tick, which is the trap — creation is not firing."""
    assert all(p["fires_without_autonomy"] is False
               for p in GROUND_TRUTH["paths"])


# --------------------------------------------------------------------------
# Yield scoring
# --------------------------------------------------------------------------

GOOD_CONTINUATION = {
    "note_id": "Note_4", "kind": "agent_concern", "seed": False,
    "activation": 0.7, "rhythm_hours": 1, "rhythm_source": "urgency",
    "category": "one_shot", "system_spawned": True,
}


def test_yield_full_marks():
    s = score_yield([{"exit_reason": "yield", "iters": 10}], [GOOD_CONTINUATION])
    assert s["score"] == 1.0
    assert s["yielded"] and s["continuation_primed_at_threshold"]


def test_yield_catches_the_note_19_shape():
    """Spawned but left at activation 0.0 against a 1h rhythm — created, and
    then never able to fire. The failure the probe exists for."""
    unprimed = dict(GOOD_CONTINUATION, activation=0.0)
    s = score_yield([{"exit_reason": "max_iters", "iters": 12}], [unprimed])
    assert s["continuation_spawned"] is True
    assert s["continuation_primed_at_threshold"] is False
    assert s["hit_max_iters"] is True
    assert s["score"] < 0.5


def test_yield_with_no_continuation_scores_zero():
    assert score_yield([{"exit_reason": "respond", "iters": 3}], [])["score"] == 0.0


# --------------------------------------------------------------------------
# Tic-tac-toe scoring
# --------------------------------------------------------------------------

def _ply(played, wins=(), blocks=(), fmt=False, illegal=False):
    return {"mover": "X", "played": played, "wins_available": list(wins),
            "blocks_required": list(blocks), "format_violation": fmt,
            "illegal_move": illegal}


PERFECT = [_ply(5), {"mover": "O", "played": 1}, _ply(9),
           {"mover": "O", "played": 3}, _ply(2, blocks=[2]),
           {"mover": "O", "played": 8}, _ply(7),
           {"mover": "O", "played": 4}, _ply(6)]


def test_perfect_replay_scores_one():
    s = ttt_score({"plies": PERFECT, "true_result": "draw",
                   "claimed_result": "draw"})
    assert s["score"] == 1.0
    assert s["forced_blocks_made"] == s["forced_blocks_faced"] == 1


def test_missed_block_and_bad_terminal_claim_are_caught():
    bad = [dict(p) for p in PERFECT]
    bad[4]["played"] = 6                     # ignored the forced block
    bad[6]["format_violation"] = True
    s = ttt_score({"plies": bad, "true_result": "draw",
                   "claimed_result": "X wins"})
    assert s["forced_blocks_made"] == 0
    assert s["terminal_agreement"] is False
    assert s["legal_move_rate"] < 1.0
    assert s["score"] < 1.0


def test_a_win_outranks_a_block():
    """Taking an available win must not be penalised as a missed block."""
    s = ttt_score({"plies": [_ply(7, wins=[7], blocks=[2])],
                   "true_result": "X wins", "claimed_result": "X wins"})
    assert s["wins_taken"] == 1
    assert s["forced_blocks_faced"] == 0


def test_absent_situation_is_omitted_not_zeroed():
    """A game with no forced block did not fail to block."""
    s = ttt_score({"plies": [_ply(5)], "true_result": "draw",
                   "claimed_result": "draw"})
    assert s["forced_block_rate"] is None
    assert s["score"] == 1.0


# --------------------------------------------------------------------------
# Yield N/A — the distinction added 2026-08-18
# --------------------------------------------------------------------------

def test_no_reply_is_not_a_completed_answer():
    """An empty reply cannot be 'nothing left to hand off'. It also must not
    reach the extractor — that would spend a live request to learn nothing,
    and would make this suite require a backend."""
    s = score_yield([{"exit_reason": "respond", "iters": 3}], [], "")
    assert s["score"] == 0.0
    assert s["probe_applies"] is True
    assert s["answer_completeness"]["complete"] is False
    assert s["answer_completeness"]["extraction_ok"] is None


def test_yielding_run_never_calls_the_extractor():
    """A run that yielded is scored on the handoff alone; completeness is
    irrelevant and must not cost a request."""
    s = score_yield([{"exit_reason": "yield", "iters": 10}], [GOOD_CONTINUATION],
                    "some long reply that would otherwise be extracted")
    assert s["answer_completeness"] is None
    assert s["score"] == 1.0
