"""`extract_external` returns the file's lines unchanged and leaves a trace
the claims-audit runner reads exactly as it reads an `inspect_external`
trace: the claims it was filed under, the file it opened, the lines a search
matched, and a trimmed form that keeps the span.

    python3 tests/v2_test_extract_tool.py
"""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.subagents.extract import extract_external               # noqa: E402
from workflowsv2.claims_audit.runner import (                     # noqa: E402
    evidence_traces, files_matched, files_read, trace_claims, trim_trace)

SRC = "fn a() {}\n// Generate a random link\nfn gen_link(len: usize) {}\n" \
      "let hits = hits + 1;\n" + "".join(f"// pad {i}\n" for i in range(40))


def _repo(tmp: Path) -> Path:
    root = tmp / "target"
    (root / "backend").mkdir(parents=True)
    (root / "backend" / "utils.rs").write_text(SRC, encoding="utf-8")
    return root


def test_read_span_is_verbatim_and_traced():
    with tempfile.TemporaryDirectory() as d:
        root = _repo(Path(d)); traces = Path(d) / "inspect_traces"
        out = extract_external(root, traces, file="backend/utils.rs",
                               lines=[2, 3], claims=[22, "9"])
        assert out.startswith("OK: backend/utils.rs:2-3\n"), out
        assert "2|// Generate a random link" in out and "3|fn gen_link" in out
        assert "1|fn a()" not in out and "4|let hits" not in out
        t = evidence_traces(traces)
        assert len(t) == 1 and t[0].name.startswith("extract_external_")
        text = t[0].read_text()
        assert trace_claims(text) == [9, 22]
        assert files_read(traces, root) == {"backend/utils.rs": files_read(traces, root)["backend/utils.rs"]}
        # trim keeps the cited span, and the numbered lines survive it
        trimmed = trim_trace(text)
        assert "2|// Generate a random link" in trimmed and "FINAL ANSWER" in trimmed


def test_pattern_is_traced_as_a_match():
    with tempfile.TemporaryDirectory() as d:
        root = _repo(Path(d)); traces = Path(d) / "inspect_traces"
        out = extract_external(root, traces, pattern="hits \\+ 1", claims=[27])
        assert "backend/utils.rs:4:" in out, out
        assert files_matched(traces, root) == ["backend/utils.rs"]
        assert trace_claims(evidence_traces(traces)[0].read_text()) == [27]


def test_refusals():
    with tempfile.TemporaryDirectory() as d:
        root = _repo(Path(d)); traces = Path(d) / "inspect_traces"
        assert extract_external(root, traces, file="backend/utils.rs").startswith("ERROR")
        assert extract_external(root, traces, file="backend/utils.rs", lines=[1, 500]).startswith("ERROR")
        assert extract_external(root, traces).startswith("ERROR")
        assert not traces.exists() or not list(traces.glob("*.txt"))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print("ok", name)
