# tictactoe — probe 2: multi-turn state

The agent is X and moves first against a deterministic scripted opponent.

**The point is not that it plays well.** The point is that the board exists
only in the conversation. After the opening instruction each turn carries one
digit and nothing else, so holding the position across nine turn boundaries is
the thing being measured — the failure family that has been costing time, where
intentions and state do not survive the crossing.

## Run

```bash
cd src
python ../bench/tictactoe/runner.py --backend gemma     # or qwen / luna
cd ..
python bench/tictactoe/score.py --run-dir bench/tictactoe/results/<ts>_<arm>
```

## Metrics — all mechanical, no judge, no network

| metric | what it catches |
|---|---|
| `legal_move_rate` | format violations + moves onto occupied squares |
| `forced_block_rate` | blocks made / blocks the position forced |
| `win_take_rate` | immediate wins taken / offered |
| `terminal_agreement` | does it know how the game it just played ended |

Every one is a fact about a recorded position, so `score.py` is deterministic
and re-runnable — re-scoring an old run after changing a metric costs nothing.

Two scoring choices worth knowing:

- **A win outranks a block.** On a ply where an immediate win was available,
  a missed block is not counted; penalising the winning move would be wrong.
- **A component whose situation never arose is omitted, not zeroed.** A game
  with no forced block did not fail to block.

An unparseable or illegal move is recorded as a failure and then *substituted*
with the lowest legal square so the game continues. One bad ply degrades the
score; it does not zero the probe.

## Why the format contract is strict

The runner parses `MOVE: <digit>` exactly, with no fuzzy fallback. This is the
same committed-answer convention `bench/hle/runner.py` uses with `ANSWER:`.
Following an output contract across nine turns is itself harness behaviour, so
a violation is a scored failure rather than a parser bug to work around.

## v1 limitation

Board-render accuracy is **not** scored. The design doc lists it, but the agent
only renders a board if it chooses to (the 2026-08-18 live game used the
`display` tool on every move; nothing requires it). Scoring an optional
behaviour would penalise an arm for a choice the prompt never asked for.
Whether a render happened is visible in the trace if you want it.

## Reference instance

The rules engine is validated against the real 2026-08-18 Jill-vs-Jack game:
moves X→5, O→1, X→9, O→3, X→2, O→8, X→7, O→4, X→6, final board
`O X O / O X X / X O X`, a genuine draw with a forced block at move 3. Gemma
played it correctly with no reasoning and no board in the prompt, which is what
suggested the probe.
