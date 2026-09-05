"""The subagent's `cite` primitive carries file lines into its answer
verbatim, and the trace it leaves reads back through the claims-audit
runner's own functions: claim tags, files read, and a trimmed form that
keeps the cited span.

    python3 tests/v2_test_cite.py
"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.subagents.code_subagent import CodeSubagent               # noqa: E402
from workflowsv2.claims_audit.runner import (                        # noqa: E402
    evidence_traces, files_read, trace_claims, trim_trace)

SRC = "fn a() {}\n// Generate a random link\nfn gen_link(len: usize) {}\n" \
      "let hits = hits + 1;\n" + "".join(f"// pad {i}\n" for i in range(40))


class FakeBackend:
    """Emits a fixed sequence of actions; what a model would say."""
    last_finish_reason = "stop"

    def __init__(self, actions):
        self._actions = [json.dumps(a) for a in actions]

    def chat(self, messages, max_tokens=None, temperature=None,
             reasoning_effort=None, response_schema=None):
        return self._actions.pop(0)


def _repo(tmp: Path) -> Path:
    root = tmp / "target"
    (root / "backend").mkdir(parents=True)
    (root / "backend" / "utils.rs").write_text(SRC, encoding="utf-8")
    return root


def test_cite_carries_verbatim_lines_and_traces():
    with tempfile.TemporaryDirectory() as d:
        root = _repo(Path(d)); traces = Path(d) / "inspect_traces"
        backend = FakeBackend([
            {"thought": "cite the generator", "tool": "cite",
             "file": "backend/utils.rs", "start_line": 2, "end_line": 3},
            {"thought": "done", "tool": "respond",
             "text": "The generator is at backend/utils.rs:2-3; it takes a length."},
        ])
        sub = CodeSubagent(root, backend, traces, mode="external")
        answer = sub.run("[claims 22, 9] where is the random link generated?")
        assert "CITED (verbatim" in answer, answer
        assert "backend/utils.rs:2-3\n2|// Generate a random link\n3|fn gen_link" in answer
        assert "4|let hits" not in answer
        t = evidence_traces(traces)
        assert len(t) == 1 and t[0].name.startswith("inspect_external_")
        text = t[0].read_text()
        assert trace_claims(text) == [9, 22]
        assert "backend/utils.rs" in files_read(traces, root)
        assert "OK: cited backend/utils.rs:2-3 (2 line(s))" in text
        trimmed = trim_trace(text)
        assert "2|// Generate a random link" in trimmed and "CITED" in trimmed


def test_cite_refusals_leave_nothing_behind():
    with tempfile.TemporaryDirectory() as d:
        root = _repo(Path(d)); traces = Path(d) / "inspect_traces"
        backend = FakeBackend([
            {"thought": "too much", "tool": "cite", "file": "backend/utils.rs",
             "start_line": 1, "end_line": 900},
            {"thought": "missing", "tool": "cite", "file": "backend/nope.rs",
             "start_line": 1, "end_line": 2},
            {"thought": "done", "tool": "respond", "text": "nothing cited"},
        ])
        answer = CodeSubagent(root, backend, traces, mode="external").run("q")
        assert "CITED" not in answer
        text = evidence_traces(traces)[0].read_text()
        assert "ERROR: cite span of 900 lines exceeds the cap" in text
        assert "ERROR: read invalid or out-of-scope file" in text


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print("ok", name)
