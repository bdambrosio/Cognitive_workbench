"""Output bounds on the recall subagent's grep primitive.

From a live diagnosis (2026-08-15): asked what was said earlier in a
conversation, Jill answered from her self-model that the record was gone.
It was not — a whole-directory grep found the phrase in 1.8s. But that
grep returned 100,294 characters, because `read` had been capped since it
was written and `grep` never was, and one line of chat_trace.txt is an
entire ReAct emission. 100KB of mostly-archive noise does not fit a
subagent working on a 4096-token budget.

Two bounds, tested here: per-hit and total truncation, and a size skip so
a blind whole-directory sweep does not stream a 339MB append-only archive
before reaching the 2MB file that holds the answer. The skip is reported
rather than silent, and an explicit `file=` still reaches anything —
otherwise a miss would read as "not in memory" when it was "not searched".
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import chat.subagents.recall as rec  # noqa: E402


def _mem(tmp_path, files):
    d = tmp_path / 'memory'
    d.mkdir(parents=True, exist_ok=True)
    for name, body in files.items():
        (d / name).write_text(body)
    return d


def test_ordinary_grep_is_unchanged(tmp_path):
    d = _mem(tmp_path, {'notes.txt': 'alpha\nbeta needle gamma\ndelta\n'})
    out = rec._tool_grep(d, 'needle', None)
    assert 'notes.txt:2' in out
    assert 'beta needle gamma' in out
    assert 'not searched' not in out


def test_no_matches_still_reads_as_no_matches(tmp_path):
    d = _mem(tmp_path, {'notes.txt': 'nothing here\n'})
    assert rec._tool_grep(d, 'needle', None).startswith('(no matches)')


def test_single_oversized_hit_is_truncated(tmp_path):
    d = _mem(tmp_path, {'big.txt': 'needle ' + ('x' * 5000) + '\n'})
    out = rec._tool_grep(d, 'needle', None)
    assert 'hit truncated at' in out
    assert len(out) < rec._MAX_GREP_HIT_CHARS + 200


def test_total_output_is_capped(tmp_path):
    """Many medium hits must not add up past the total ceiling — the
    per-hit cap alone would still allow 50 x 600 chars."""
    body = ''.join(f'needle {"y" * 400}\n' for _ in range(60))
    d = _mem(tmp_path, {'many.txt': body})
    out = rec._tool_grep(d, 'needle', None)
    assert len(out) < rec._MAX_GREP_OUT_CHARS + 500, len(out)
    assert 'output capped' in out


def test_large_files_are_skipped_and_reported(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, '_GREP_MAX_FILE_BYTES', 100)
    d = _mem(tmp_path, {
        'small.txt': 'needle in the small file\n',
        'archive.txt': 'needle ' + ('z' * 500) + '\n',
    })
    out = rec._tool_grep(d, 'needle', None)
    assert 'small.txt' in out
    assert 'not searched' in out
    assert 'archive.txt' in out
    # The skipped file's CONTENT must not appear.
    assert 'zzz' not in out


def test_skip_is_reported_even_when_nothing_matched(tmp_path, monkeypatch):
    """The failure this guards against: concluding a memory does not exist
    when the file holding it was never opened."""
    monkeypatch.setattr(rec, '_GREP_MAX_FILE_BYTES', 100)
    d = _mem(tmp_path, {'archive.txt': 'needle ' + ('z' * 500) + '\n'})
    out = rec._tool_grep(d, 'needle', None)
    assert out.startswith('(no matches)')
    assert 'not searched' in out and 'archive.txt' in out


def test_explicit_file_argument_bypasses_the_size_skip(tmp_path, monkeypatch):
    """Naming a file reaches it at any size — the cap bounds blind
    sweeps, not what is reachable."""
    monkeypatch.setattr(rec, '_GREP_MAX_FILE_BYTES', 100)
    d = _mem(tmp_path, {'archive.txt': 'needle in the archive\n'})
    out = rec._tool_grep(d, 'needle', 'archive.txt')
    assert 'archive.txt:1' in out
    assert 'needle in the archive' in out
    assert 'not searched' not in out


def test_hit_cap_still_applies(tmp_path):
    d = _mem(tmp_path, {'many.txt': 'needle\n' * 200})
    out = rec._tool_grep(d, 'needle', None)
    assert 'capped at' in out
