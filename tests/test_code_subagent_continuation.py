"""At the step cap the code subagent gives a last word instead of machine
salvage, offers one continuation, and resumes from its whole prior log;
`cite` carries several spans in one call, in the order given."""
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.subagents import code_subagent as cs                   # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("git") is None, reason="needs git")


class _Backend:
    """Emits `read` until told to respond; records every prompt it saw."""
    def __init__(self, respond_at=None, respond_text="what I found"):
        self.calls = 0
        self.respond_at = respond_at
        self.respond_text = respond_text
        self.prompts = []

    def chat(self, messages, **kw):
        self.calls += 1
        self.prompts.append(messages)
        user = messages[1]['content']
        if 'you are out of steps' in user or (self.respond_at and self.calls >= self.respond_at):
            return json.dumps({"thought": "done", "tool": "respond", "text": self.respond_text})
        return json.dumps({"thought": "look", "tool": "read", "file": "a.txt"})


def _repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / "a.txt").write_text("".join(f"line {i}\n" for i in range(1, 30)))
    (tmp_path / "b.txt").write_text("".join(f"bee {i}\n" for i in range(1, 10)))
    return tmp_path


def _sa(repo, tmp_path, backend):
    traces = tmp_path.parent / f"{tmp_path.name}_world" / "inspect_traces"
    return cs.CodeSubagent(repo, backend, traces, mode='external')


def test_multi_span_cite_keeps_the_order_given_and_caps_the_count(tmp_path):
    sa = _sa(_repo(tmp_path), tmp_path, _Backend())
    out = sa._tool_cite({'spans': [
        {'file': 'b.txt', 'start_line': 2, 'end_line': 3},
        {'file': 'a.txt', 'start_line': 5, 'end_line': 6},
        {'file': 'b.txt', 'start_line': 1, 'end_line': 1},
    ]})
    assert out.startswith("OK: cited 3 of 3 span(s)")
    assert [(f, s, e) for f, s, e, _ in sa._cited] == [('b.txt', 2, 3), ('a.txt', 5, 6), ('b.txt', 1, 1)]
    suffix = sa.answer_suffix()
    assert suffix.index('b.txt:2-3') < suffix.index('a.txt:5-6') < suffix.index('b.txt:1-1')
    single = sa._tool_cite({'file': 'a.txt', 'start_line': 1, 'end_line': 2})
    assert single.startswith("OK: cited a.txt:1-2")
    too_many = sa._tool_cite({'spans': [{'file': 'a.txt', 'start_line': 1, 'end_line': 1}] * 11})
    assert too_many.startswith("ERROR: cite carries at most 10")


def test_cap_gives_a_last_word_and_offers_one_continuation(tmp_path):
    repo = _repo(tmp_path)
    be = _Backend()
    sa = _sa(repo, tmp_path, be)
    sa.max_iters = 3
    out = sa.run("what is in a.txt")
    assert be.calls == 4                                   # 3 steps + the last word
    assert "you are out of steps" in be.prompts[-1][1]['content']
    assert "hit the step cap; reporting what was found" in out and "what I found" in out
    m = re.search(r'`continue: "([0-9a-f]{32})"`', out)
    assert m, out
    cid = m.group(1)
    state = json.loads((sa.state_dir / f"{cid}.json").read_text())
    assert state['query'] == "what is in a.txt"
    assert any(ln.startswith("LAST WORD:") for ln in state['log_lines'])
    assert sum(1 for ln in state['log_lines'] if ln.startswith("ACTION ")) == 3
    # The harness names its own lines by index; the file keeps them as written.
    marks = state['harness']
    assert state['log_lines'][marks['note']].startswith("NOTE: you are out of steps")
    assert state['log_lines'][marks['last_word']] == "LAST WORD: what I found"

    # Resume: whole prior log in the prompt, preamble on the system prompt,
    # a fresh round of steps, no second offer.
    be2 = _Backend(respond_at=2, respond_text="finished")
    sa2 = _sa(repo, tmp_path, be2)
    sa2.max_iters = 3
    out2 = cs._run_or_resume(sa2, "ignored", cid)
    assert out2.strip().startswith("finished")
    sysp = be2.prompts[0][0]['content']
    assert sysp.startswith("This is a continuation")
    resumed = be2.prompts[0][1]['content']
    assert "ACTION 1:" in resumed
    # The stale note is gone from the view; the last word stays, relabelled.
    assert "you are out of steps" not in resumed
    assert "LAST WORD:" not in resumed
    assert "INTERIM REPORT at the step cap" in resumed and "what I found" in resumed
    assert "continue:" not in out2
    assert not (sa.state_dir / f"{cid}.json").exists()
    again = cs._run_or_resume(_sa(repo, tmp_path, _Backend()), "x", cid)
    assert again.startswith("ERROR: no continuation")


def test_last_word_that_is_not_a_respond_falls_back_to_salvage(tmp_path):
    class _Never(_Backend):
        def chat(self, messages, **kw):
            self.calls += 1
            return json.dumps({"thought": "more", "tool": "read", "file": "a.txt"})
    sa = _sa(_repo(tmp_path), tmp_path, _Never())
    sa.max_iters = 2
    out = sa.run("q")
    assert "hit max iterations without responding" in out
    assert "PARTIAL" in out
