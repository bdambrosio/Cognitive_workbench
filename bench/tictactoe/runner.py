#!/usr/bin/env python3
"""Probe 2 — multi-turn state, with no board in the prompt.

    cd src
    python ../bench/tictactoe/runner.py --backend gemma

The agent is X and moves first against a deterministic scripted opponent. The
point is NOT that it plays well. The point is that the board exists only in
the conversation: after the opening instruction, each turn carries one digit
and nothing else, so tracking the position across nine turn boundaries is the
thing being measured. This is the failure family that has been costing time —
intentions and state not surviving the crossing.

Records every position and every reply; scoring is a separate pass so it can
be re-run without spending the game again.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO))

from bench.common import (build_loop_config, load_arm,  # noqa: E402
                          read_reasoning_trace, turn_costs, verify_served_model)
from bench.tictactoe import game  # noqa: E402

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("bench.tictactoe.runner")

SOURCE = "User"
SCENARIO = REPO / "scenarios" / "yield_test.yaml"

OPENING = """Let's play tic-tac-toe. You are X and move first; I am O.

Squares are numbered 1-9, left to right, top to bottom: 1 is top-left, 3 is
top-right, 9 is bottom-right.

I will send only my move as a single digit each turn. Keep track of the board
yourself — I will not repeat it back to you.

Reply with your move and nothing else, on its own final line, in exactly this
form:
MOVE: <digit>

Make your opening move now."""

RESULT_Q = """The game is over. On its own final line, state the result in
exactly this form, choosing one:
RESULT: X wins
RESULT: O wins
RESULT: draw"""

_MOVE_RE = re.compile(r"MOVE:\s*([1-9])", re.IGNORECASE)
_RESULT_RE = re.compile(r"RESULT:\s*(X wins|O wins|draw)", re.IGNORECASE)


def _latest_reply(loop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


def _parse_move(reply: str) -> Optional[int]:
    """Read a committed answer in a specified format — the same convention
    bench/hle/runner.py uses with `ANSWER:`. A reply that does not honour the
    contract is a scored failure, not a parser bug, so there is no fuzzy
    fallback here on purpose."""
    m = _MOVE_RE.search(reply or "")
    return int(m.group(1)) if m else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", required=True)
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    arm = load_arm(args.backend)
    served = verify_served_model(arm)
    logger.info("arm=%s served=%s", arm["name"], served.get("served"))

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    world = f"bench-ttt-{ts}-{args.backend}"
    out = args.output_dir or (HERE / "results" / f"{ts}_{args.backend}")
    out.mkdir(parents=True, exist_ok=True)

    from chat.chat_loop import ChatLoop  # noqa: E402

    char_name, char_cfg = build_loop_config(SCENARIO, arm, world, autonomy=False)
    loop = ChatLoop(character_name=char_name, character_config=char_cfg)

    board = game.new_board()
    plies: List[Dict[str, Any]] = []
    t0 = time.time()
    error = None
    prompt = OPENING

    try:
        while not game.over(board):
            # Snapshot what the agent faces BEFORE it moves — the scorer needs
            # the position, not just the move, to know what was forced.
            pre = dict(board)
            wins_avail = game.winning_moves(pre, "X")
            blocks_req = game.forced_blocks(pre, "X")

            loop._process_user_turn(source=SOURCE, text=prompt, close=False)
            reply = _latest_reply(loop, SOURCE)
            move = _parse_move(reply)

            legal_now = game.legal(board)
            illegal = move is not None and move not in legal_now
            substituted = None
            if move is None or illegal:
                # Keep the game going so one bad ply does not zero the probe.
                # Recorded as a failure either way.
                substituted = min(legal_now)
                logger.warning("ply %d: %s -> substituting %d", len(plies) + 1,
                               "unparseable" if move is None else f"illegal {move}",
                               substituted)
            played = substituted if substituted is not None else move
            board[played] = "X"

            ply = {
                "ply": len(plies) + 1,
                "mover": "X",
                "pre_board": pre,
                "reply": reply,
                "parsed_move": move,
                "format_violation": move is None,
                "illegal_move": bool(illegal),
                "substituted": substituted,
                "played": played,
                "wins_available": wins_avail,
                "blocks_required": blocks_req,
            }
            plies.append(ply)

            if game.over(board):
                break
            omove = game.opponent_move(board, "O")
            board[omove] = "O"
            plies.append({"ply": len(plies) + 1, "mover": "O", "played": omove})
            prompt = str(omove)

        # Terminal agreement — does it know how the game it just played ended?
        loop._process_user_turn(source=SOURCE, text=RESULT_Q, close=False)
        result_reply = _latest_reply(loop, SOURCE)
        rm = _RESULT_RE.search(result_reply or "")
        claimed_result = rm.group(1).lower().replace(" wins", " wins") if rm else None
    except Exception as e:
        error = f"{type(e).__name__}: {e}"
        logger.exception("game failed: %s", e)
        result_reply, claimed_result = "", None
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning("executor shutdown failed: %s", e)
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning("final persist failed: %s", e)

    wall = round(time.time() - t0, 1)
    trace = read_reasoning_trace(world, char_name)

    (out / "raw.json").write_text(json.dumps({
        "plies": plies,
        "final_board": board,
        "true_result": game.result(board),
        "result_reply": result_reply,
        "claimed_result": claimed_result,
        "error": error,
        "wall_clock_s": wall,
        "trace": trace,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    (out / "run_meta.json").write_text(json.dumps({
        "probe": "tictactoe (probe 2)",
        "backend_arm": arm["name"],
        "backend_label": arm.get("label"),
        "served_model_check": served,
        "llm_config": arm.get("llm_config"),
        "world_name": world,
        "character": char_name,
        "wall_clock_s": wall,
        "costs": turn_costs(trace),
        "captured_at": ts,
        "error": error,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\nwrote {out}")
    print(game.render(board))
    print(f"true={game.result(board)} claimed={claimed_result} wall={wall}s")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
