"""Post-turn work shares one process-wide pool.

Each character used to get its own single-worker executor, so Jill could be
six deep while Jack sat idle and neither could borrow the other's slot.
Measured 2026-08-15: a 6-minute post-turn tail settling three visible turns.

Characters are threads in one process (launcher.py:988), so a shared pool
is a better allocation of the same capacity — width 2 keeps the concurrent
request ceiling where it was (2 agents x 2 threads == the server's
--max-num-seqs 4) while letting one agent use both slots.

The subtle part is shutdown. Callers use `shutdown(wait=True)` to mean "my
post-turn work has finished" — a per-question-isolated runner relies on it —
and on a shared pool that must not mean "stop the pool", which would strand
every other character.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import chat.chat_loop as cl  # noqa: E402


def test_pool_width_matches_the_server_ceiling():
    """2 agents x 2 threads is what --max-num-seqs 4 accommodates; a wider
    pool would queue at the server instead of here."""
    assert cl._POST_TURN_POOL_WIDTH == 2


def test_one_loops_backlog_uses_both_slots():
    """The point of sharing: a single agent with a queue is no longer
    limited to one worker."""
    q = cl._PostTurnQueue('Solo')
    for _ in range(4):
        q.submit(time.sleep, 0.2)
    t0 = time.time()
    q.shutdown(wait=True)
    elapsed = time.time() - t0
    # Serial would be ~0.8s; width 2 should land near 0.4s.
    assert elapsed < 0.7, f'ran serially: {elapsed:.2f}s'


def test_shutdown_drains_only_the_caller_and_leaves_the_pool_running():
    """One character finishing must not strand the others."""
    jill, jack = cl._PostTurnQueue('Jill'), cl._PostTurnQueue('Jack')
    done = []
    jill.submit(done.append, 'jill')
    jill.shutdown(wait=True)
    assert 'jill' in done
    assert not cl._shared_post_turn_pool()._shutdown, 'pool was shut down'
    # Jack can still work afterwards.
    jack.submit(done.append, 'jack')
    jack.shutdown(wait=True)
    assert 'jack' in done


def test_shutdown_waits_for_completion():
    """The bench runner's isolation contract: after shutdown(wait=True) the
    submitted work has actually finished, not merely been queued."""
    q = cl._PostTurnQueue('Waiter')
    done = []
    q.submit(lambda: (time.sleep(0.15), done.append('x')))
    q.shutdown(wait=True)
    assert done == ['x']


def test_a_failing_task_does_not_break_the_drain():
    """A raised exception in one post-turn pass must not prevent the rest
    from being awaited, and must not propagate into teardown."""
    q = cl._PostTurnQueue('Faulty')
    done = []

    def boom():
        raise RuntimeError('reflection exploded')

    q.submit(boom)
    q.submit(done.append, 'after')
    q.shutdown(wait=True)     # must not raise
    assert 'after' in done


def test_completed_futures_are_pruned():
    """A long session must not accumulate futures for every turn it ran."""
    q = cl._PostTurnQueue('LongRun')
    for _ in range(5):
        q.submit(lambda: None)
    q.shutdown(wait=True)
    for _ in range(3):
        q.submit(lambda: None)
    assert len(q._futures) <= 3


def test_reflection_passes_send_no_reasoning_effort():
    """The two reflection call sites hardcoded reasoning_effort='low',
    intending LESS analysis. On a backend whose chat template has no
    reasoning_effort variable (Gemma-4), vLLM falls back to
    enable_thinking=(effort != 'none') and switches thinking ON from a
    default of off — 12,049 reasoning chars to emit 'NONE END_TRIAGE'.
    Reflection reasoning belongs behind the launcher flag, not a literal.
    """
    src = Path(cl.__file__).read_text()
    assert "reasoning_effort='low'" not in src, (
        'a hardcoded reasoning_effort literal is back in chat_loop.py')
