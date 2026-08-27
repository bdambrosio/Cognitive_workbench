"""The delivery-block vocabulary (workflows/blocks.py, METHOD §16).

Every case here is one the old turn-based scheme got wrong, or one the marker
regexes have to survive because a model has already produced it.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from workflows import blocks                                   # noqa: E402

FULL = """=== CLAIM SURFACE ===
28 claims
=== END CLAIM SURFACE ===
=== REPORT ===
body here
=== END REPORT ===
=== LIMITATIONS ===
three lines
=== END LIMITATIONS ===
=== GAP MAP ===
gm body
=== END GAP MAP ==="""


def test_all_four_blocks_are_found_in_one_emission():
    # A leg carrying several blocks is not a special case — it was the
    # "salvage path" before, and salvage is where defects hide.
    assert blocks.present(FULL) == {n: True for n in blocks.BLOCKS}
    assert blocks.missing(blocks.present(FULL)) == []


def test_content_excludes_the_markers_and_span_keeps_them():
    assert blocks.content(FULL, "REPORT") == "body here"
    assert blocks.span(FULL, "REPORT") == (
        "=== REPORT ===\nbody here\n=== END REPORT ===")


def test_an_opener_never_matches_inside_its_own_closer():
    # `=== END REPORT ===` puts END where REPORT would have to be. If this
    # broke, every closer would read as a second delivery.
    assert not blocks.opened("=== END REPORT ===", "REPORT")
    assert blocks.closed("=== END REPORT ===", "REPORT")


def test_a_truncated_block_keeps_its_body():
    # Truncation loses the closer. Losing the body as well would turn one
    # defect into two, and the body is what a reader needs to see the defect.
    u = "=== REPORT ===\nbody\n=== GAP MAP ===\ngm"
    assert blocks.content(u, "REPORT") == "body"
    assert blocks.content(u, "GAP MAP") == "gm"
    assert not blocks.closed(u, "REPORT")


def test_a_line_wrapped_marker_still_matches():
    # 2026-08-25, on a real target: a complete limitations statement under
    # `===\nLIMITATIONS ===`. Exact matching called the requirement absent.
    assert blocks.opened("===\nLIMITATIONS ===", "LIMITATIONS")


def test_missing_reports_in_emission_order():
    partial = "=== REPORT ===\nx\n=== END REPORT ==="
    assert blocks.missing(blocks.present(partial)) == [
        "CLAIM SURFACE", "LIMITATIONS", "GAP MAP"]


def test_an_absent_block_is_None_not_empty():
    # None and "" are different answers: one says the block never arrived, the
    # other says it arrived empty. Collapsing them is how "no report" and "an
    # empty report" became the same verdict.
    assert blocks.content("nothing here", "REPORT") is None
    assert blocks.span("nothing here", "REPORT") is None


def test_the_rejection_states_what_was_observed():
    # It may say a marker is absent. It may NOT say a report was not written —
    # the runner cannot check that, and a model that wrote a good report and
    # forgot the marker would be told something untrue.
    msg = blocks.rejection("GAP MAP")
    assert "=== GAP MAP ===" in msg
    assert "METHOD §16" in msg
    assert "no report received" not in msg.lower()
