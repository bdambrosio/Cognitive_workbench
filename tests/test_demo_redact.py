"""Redaction: a list of literal identifiers, replaced wherever they appear."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from demo.redact import Redactor, banner, LINK_WITHHELD          # noqa: E402


def test_identifiers_urls_and_roots(tmp_path):
    r = Redactor("the target", ["chhoto-url", "chhoto", "SinTan1729"], roots=[tmp_path])
    s = f"See https://github.com/SinTan1729/chhoto-url/blob/main/x.rs and {tmp_path}/a/b.rs; Chhoto is by sintan1729."
    out = r.text(s)
    assert "chhoto" not in out.lower() and "SinTan" not in out and "github.com" not in out
    assert LINK_WITHHELD in out and "a/b.rs" in out and str(tmp_path) not in out
    assert r.leaks(out) == [] and r.leaks(s)
    d = r.deep({"a": ["chhoto-url", {"b": "ok"}], "c": 3})
    assert d == {"a": ["the target", {"b": "ok"}], "c": 3}
    assert "identity withheld" in banner("the target", 43, 43, 1, "a local model")
