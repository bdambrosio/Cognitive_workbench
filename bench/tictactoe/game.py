#!/usr/bin/env python3
"""Board rules and the scripted opponent. Pure functions, no I/O, no model.

Squares are 1-9, 1=top-left, 3=top-right, 9=bottom-right — the numbering the
2026-08-18 live game used, kept so that game remains a readable reference
instance.

Everything the probe scores is computable from here, which is why probe 2
needs no judge: whether a block was forced, whether a win was on offer, and
what the terminal result is are all facts about the position.
"""

from __future__ import annotations

from typing import Dict, List, Optional

LINES = [(1, 2, 3), (4, 5, 6), (7, 8, 9),      # rows
         (1, 4, 7), (2, 5, 8), (3, 6, 9),      # cols
         (1, 5, 9), (3, 5, 7)]                 # diagonals

# Deterministic opponent preference once nothing is forced: centre, then
# corners, then edges. Centre-first makes threats appear early, so the agent
# meets a forced block within the first few moves rather than never.
PREFERENCE = (5, 1, 3, 7, 9, 2, 4, 6, 8)

EMPTY = " "


def new_board() -> Dict[int, str]:
    return {i: EMPTY for i in range(1, 10)}


def legal(board: Dict[int, str]) -> List[int]:
    return [i for i in range(1, 10) if board[i] == EMPTY]


def winner(board: Dict[int, str]) -> Optional[str]:
    for a, b, c in LINES:
        if board[a] != EMPTY and board[a] == board[b] == board[c]:
            return board[a]
    return None


def full(board: Dict[int, str]) -> bool:
    return not legal(board)


def over(board: Dict[int, str]) -> bool:
    return winner(board) is not None or full(board)


def result(board: Dict[int, str]) -> str:
    w = winner(board)
    return f"{w} wins" if w else "draw"


def winning_moves(board: Dict[int, str], mark: str) -> List[int]:
    """Squares that complete a line for `mark` right now."""
    out = []
    for sq in legal(board):
        board[sq] = mark
        if winner(board) == mark:
            out.append(sq)
        board[sq] = EMPTY
    return out


def forced_blocks(board: Dict[int, str], mark: str) -> List[int]:
    """Squares `mark` must take to deny the opponent an immediate win."""
    other = "O" if mark == "X" else "X"
    return winning_moves(board, other)


def opponent_move(board: Dict[int, str], mark: str = "O") -> int:
    """Win if it can, block if it must, else first by preference.

    Deterministic on purpose: the arms must face identical positions, or the
    probe is measuring which arm drew the easier game.
    """
    wins = winning_moves(board, mark)
    if wins:
        return min(wins)
    blocks = forced_blocks(board, mark)
    if blocks:
        return min(blocks)
    for sq in PREFERENCE:
        if board[sq] == EMPTY:
            return sq
    raise RuntimeError("opponent asked to move on a full board")


def render(board: Dict[int, str]) -> str:
    r = [f" {board[i]} " for i in range(1, 10)]
    return (f"{r[0]}|{r[1]}|{r[2]}\n---+---+---\n"
            f"{r[3]}|{r[4]}|{r[5]}\n---+---+---\n"
            f"{r[6]}|{r[7]}|{r[8]}")
