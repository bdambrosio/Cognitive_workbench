"""A quote that copied the runner's line-number display (`123|text`) resolves
once the prefixes are stripped, and the finding is rewritten to source text.
Anything else that starts with digits and a pipe is left alone."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.claims_audit import schemas                    # noqa: E402

SRC = "line one\nApache License\n  Version 2.0\n12|a legit pipe line\nlast\n"
FROZEN = [{"id": 1, "quote": "x", "lines": [1, 1], "statement": "s", "about": "target"}]


def _run(tmp_path, quote, lines):
    (tmp_path / "LICENSE").write_text(SRC)
    (tmp_path / "README.md").write_text("x\n")
    obj = {"claim_source": "README.md", "findings": [
        {"claim_id": 1, "adjudication": {"verdict": "real"},
         "evidence": [{"form": "citation", "document": "LICENSE", "lines": lines,
                       "quote": quote, "shows": "it"}]}]}
    res = schemas.check_output(obj, tmp_path, "README.md", FROZEN, read=None)
    return res, obj["findings"][0]["evidence"][0]["quote"]


def test_strip_line_prefixes_recognises_only_the_display():
    assert schemas.strip_line_prefixes("2|Apache License\n3|  Version 2.0", 2) == "Apache License\n  Version 2.0"
    assert schemas.strip_line_prefixes("2|Apache License\n4|  Version 2.0", 2) is None   # not consecutive
    assert schemas.strip_line_prefixes("3|Apache License", 2) is None                    # wrong start
    assert schemas.strip_line_prefixes("Apache License\n3|x", 2) is None                 # a line without one


def test_copied_display_resolves_and_is_rewritten_to_source_text(tmp_path):
    res, quote = _run(tmp_path, "2|Apache License\n3|  Version 2.0", [2, 3])
    assert not [p for p in res["problems"] if "quote" in p]
    assert res["figures"]["prefixed_quotes"] == ["finding 1 evidence 1"]
    assert quote == "Apache License\n  Version 2.0"


def test_a_real_quote_that_starts_with_digits_and_a_pipe_is_untouched(tmp_path):
    res, quote = _run(tmp_path, "12|a legit pipe line", [4, 4])
    assert not [p for p in res["problems"] if "quote" in p] and res["figures"]["prefixed_quotes"] == []
    assert quote == "12|a legit pipe line"


def test_a_prefixed_quote_that_still_does_not_resolve_is_still_a_problem(tmp_path):
    res, quote = _run(tmp_path, "2|Apache Licence\n3|  Version 3.0", [2, 3])
    assert any("quote is not at LICENSE:2-3" in p for p in res["problems"])
    assert quote == "2|Apache Licence\n3|  Version 3.0" and res["figures"]["prefixed_quotes"] == []
