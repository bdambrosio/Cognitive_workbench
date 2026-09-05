"""Enumeration by section: the split, the assembled surface with
restatements folded in, and the surface check over every location."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.claims_audit import schemas as sch             # noqa: E402

DOC = """# Title
intro line
## Features
- fast
```bash
# not a heading
echo hi
```
## Pricing
| plan | price |
|---|---|
| free | 0 |
## FAQ
Is it fast? Yes, fast.
"""


def test_split_sections_cuts_at_headings_outside_fences_and_merges_small():
    secs = sch.split_sections(DOC, minimum=1, maximum=10_000)
    lines = DOC.splitlines()
    assert [lines[a - 1] for a, _ in secs] == ["# Title", "## Features", "## Pricing", "## FAQ"]
    assert secs[-1][1] == len(lines)
    # small sections join forward; the tail joins back
    secs = sch.split_sections(DOC, minimum=60, maximum=10_000)
    assert len(secs) < 4 and secs[0][0] == 1 and secs[-1][1] == len(lines)
    # an oversized section is cut at a blank line
    big = "# H\n" + ("x" * 50 + "\n\n") * 10
    secs = sch.split_sections(big, minimum=1, maximum=200)
    assert len(secs) > 1 and all(a <= b for a, b in secs)
    assert secs[0][0] == 1 and secs[-1][1] == len(big.splitlines())
    assert sch.split_sections("") == []
    assert sch.split_sections("no heading\nat all") == [(1, 2)]


def test_assemble_surface_assigns_ids_and_folds_restatements():
    parts = [
        {"claims": [{"quote": "fast", "lines": [4, 4], "statement": "s", "about": "document"}]},
        {"claims": [{"quote": "free | 0", "lines": [12, 12], "statement": "s", "about": "seller"},
                    {"quote": "Yes, fast.", "lines": [14, 14], "statement": "s",
                     "about": "target", "restates": 1},
                    {"quote": "dangling", "lines": [14, 14], "statement": "s",
                     "about": "target", "restates": 9}]}]
    out = sch.assemble_surface("d.md", parts)
    ids = [c["id"] for c in out["claims"]]
    assert ids == [1, 2, 3]
    assert out["claims"][0]["locations"] == [{"quote": "Yes, fast.", "lines": [14, 14]}]
    assert out["claims"][2]["restates"] == 9      # kept, reported by the check
    assert sch.locations(out["claims"][0]) == [
        {"quote": "fast", "lines": [4, 4]}, {"quote": "Yes, fast.", "lines": [14, 14]}]
    nc = sch.assemble_surface("d.md", [{"claims": [], "not_completed": "unreadable"}])
    assert nc["not_completed"] == "unreadable" and nc["claims"] == []


def test_check_surface_covers_locations_about_and_dangling_restates(tmp_path):
    (tmp_path / "d.md").write_text(DOC)
    parts = [
        {"claims": [{"quote": "fast", "lines": [4, 4], "statement": "s", "about": "target"}]},
        {"claims": [{"quote": "Yes, fast.", "lines": [14, 14], "statement": "s",
                     "about": "target", "restates": 1},
                    {"quote": "free | 0", "lines": [12, 12], "statement": "s", "about": "nobody"},
                    {"quote": "elsewhere", "lines": [2, 2], "statement": "s",
                     "about": "target", "restates": 7}]}]
    res = sch.check_surface(sch.assemble_surface("d.md", parts), tmp_path, "d.md")
    text = "\n".join(res["problems"])
    assert "claim 2: `about` 'nobody'" in text
    assert "claim 3: restates claim 7, which is not assigned" in text
    assert "claim 3: quote is not at lines 2-2" in text
    assert "location" not in text            # the folded location resolves
    assert res["figures"]["restatements"] == 1
    assert res["figures"]["about"] == {"target": 2, "nobody": 1}
    bad = [{"claims": [{"quote": "fast", "lines": [4, 4], "statement": "s", "about": "target"}]},
           {"claims": [{"quote": "not here", "lines": [14, 14], "statement": "s",
                        "about": "target", "restates": 1}]}]
    res = sch.check_surface(sch.assemble_surface("d.md", bad), tmp_path, "d.md")
    assert any(p.startswith("claim 1 location 2: quote is not at lines 14-14") for p in res["problems"])


def test_assemble_surface_folds_duplicate_quotes_within_a_section():
    parts = [{"claims": [
        {"quote": "multi-model AI (OpenAI, Claude)", "lines": [9, 9], "statement": "OpenAI", "about": "target"},
        {"quote": "multi-model AI (OpenAI, Claude)", "lines": [9, 9], "statement": "Claude", "about": "target"},
        {"quote": "multi-model  AI (OpenAI, Claude)", "lines": [9, 9], "statement": "Claude", "about": "target"},
        {"quote": "self-host it", "lines": [10, 10], "statement": "s", "about": "target"}]}]
    out = sch.assemble_surface("d.md", parts)
    assert [c["id"] for c in out["claims"]] == [1, 2]
    first = out["claims"][0]
    assert len(first["locations"]) == 2
    assert first["statement"] == "OpenAI" and first["statements"] == ["Claude"]
    # a duplicate in a LATER section is not folded by this rule: that is what
    # `restates` is for, and the surface check reports the repeat
    later = parts + [{"claims": [
        {"quote": "self-host it", "lines": [40, 40], "statement": "s", "about": "target"}]}]
    out = sch.assemble_surface("d.md", later)
    assert [c["id"] for c in out["claims"]] == [1, 2, 3]
