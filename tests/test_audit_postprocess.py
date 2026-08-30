"""The delivery stage reorganises. It must never lose or alter content.

That is the whole risk of markdown surgery: a section moved to an appendix and
quietly truncated looks like a shorter report, not like a bug. These tests hold
the invariant rather than the layout, so the assembly can be reshaped freely.
"""
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from workflows.audit_postprocess.deliver import (      # noqa: E402
    assemble, is_inventory, sections, _FINDING, _INVENTORY)

REPORT = """=== REPORT ===
**Conclusion: Material.** Three claims are contradicted.

## Findings — material

**Finding 1: Cross-session memory — [delta]**
Claim (README.md): memory across sessions.
Evidence: chat_agent.py:898 — single session_id.
Gap: per-session only.

**Finding 2: Auto translation — [delta]**
Claim (README.md): auto translation.
Evidence: workflow.py:41 — column only.
Gap: no implementation.

## Findings — supported

3. Business hours — [real]: business_hours.py:51-83.
5. Ask Anything — [real]: widget.py:169.
6. RBAC — [real]: role.py:22-26.
9. Channels — [real]: channels/base.py:70.

## Coverage statement

Coverage: 6 of 7 identified claims resolved.

## Questions the client should ask the seller before closing

1. Is the CLI planned or deprecated?
=== END REPORT ===

=== LIMITATIONS ===
1. Materials examined: the repository as bound to this session.
=== END LIMITATIONS ===
"""


def _run(tmp_path):
    (tmp_path / "report.md").write_text(REPORT)
    (tmp_path / "gap_map.md").write_text("=== GAP MAP ===\nx\n=== END GAP MAP ===\n")
    return tmp_path


def test_every_finding_survives_assembly(tmp_path):
    out = assemble(_run(tmp_path))
    assert sorted(_FINDING.findall(out)) == sorted(_FINDING.findall(REPORT))


def test_every_inventory_entry_survives_assembly(tmp_path):
    out = assemble(_run(tmp_path))
    assert len(_INVENTORY.findall(out)) == len(_INVENTORY.findall(REPORT)) == 4


def test_the_parts_a_client_reads_all_survive(tmp_path):
    out = assemble(_run(tmp_path))
    for probe in ("Coverage: 6 of 7",
                  "Questions the client should ask",
                  "Materials examined: the repository as bound to this session",
                  "Is the CLI planned or deprecated?",
                  "Evidence: chat_agent.py:898 — single session_id."):
        assert probe in out, probe


def test_the_conclusion_is_stated_once_and_keeps_its_verdict(tmp_path):
    """The label moves into the heading rather than being printed twice. The
    VERDICT must survive that move, and the sentence after it must be intact."""
    out = assemble(_run(tmp_path))
    assert "## Conclusion — Material" in out
    assert out.count("Material") >= 1
    assert "Three claims are contradicted." in out
    assert "**Conclusion: Material.**" not in out      # not said twice


def test_a_conclusion_that_does_not_match_the_pattern_is_left_alone(tmp_path):
    """Unrecognised shapes keep their text verbatim under a plain heading."""
    (tmp_path / "report.md").write_text(
        REPORT.replace("**Conclusion: Material.** Three claims are contradicted.",
                       "The audit concludes that three claims are contradicted."))
    (tmp_path / "gap_map.md").write_text("x")
    out = assemble(tmp_path)
    assert "## Conclusion\n" in out
    assert "The audit concludes that three claims are contradicted." in out


def test_delivery_markers_never_reach_the_client(tmp_path):
    """`=== LIMITATIONS ===` is protocol between the agent and the runner. It
    means something to blocks.py and nothing to a buyer. The limitations TEXT
    must survive; the marker must not."""
    out = assemble(_run(tmp_path))
    assert not re.search(r"(?m)^\s*===", out)
    assert "## Limitations" in out
    assert "Materials examined: the repository as bound to this session" in out


def test_limitations_travel_with_the_report_not_after_the_appendices(tmp_path):
    """ISAE 3000 / AT-C 205, and the reason blocks.py assembles report.md from
    REPORT + LIMITATIONS: a reader stops at the appendices."""
    out = assemble(_run(tmp_path))
    assert out.index("## Limitations") < out.index("## Appendix A")


def test_the_document_identifies_itself(tmp_path):
    out = assemble(_run(tmp_path))
    assert out.startswith("# Technical claims audit")
    assert out.index("## Conclusion") < out.index("## Findings")


def test_the_inventory_moves_below_the_narrative(tmp_path):
    """The point of the exercise: 4 of 6 entries out of what is read first."""
    out = assemble(_run(tmp_path))
    assert out.index("## Appendix") > out.index("Questions the client should ask")
    assert out.index("Business hours") > out.index("## Appendix")
    assert out.index("Finding 1") < out.index("## Appendix")


def test_a_moved_heading_nests_under_the_appendix(tmp_path):
    out = assemble(_run(tmp_path))
    assert "### Findings — supported" in out
    assert "\n## Findings — supported" not in out


def test_inventory_is_recognised_by_shape_not_by_heading_text():
    """Sections are classified by what they contain, per CLAUDE.md's rule
    against keyword matching. A findings section full of `Finding N` headers is
    never an inventory, whatever its heading says."""
    body = REPORT
    by_heading = dict(sections(body))
    narrative = next(v for k, v in by_heading.items() if "material" in k)
    listed = next(v for k, v in by_heading.items() if "supported" in k)
    assert is_inventory(listed)
    assert not is_inventory(narrative)


def test_a_report_with_no_inventory_is_returned_intact(tmp_path):
    """No appendix is invented when there is nothing to move."""
    trimmed = re.sub(r"## Findings — supported.*?(?=## Coverage)", "", REPORT,
                     flags=re.S)
    (tmp_path / "report.md").write_text(trimmed)
    out = assemble(tmp_path)
    assert "## Appendix" not in out
    assert sorted(_FINDING.findall(out)) == sorted(_FINDING.findall(trimmed))
