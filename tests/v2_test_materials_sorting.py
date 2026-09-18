"""Materials sorting: the mechanical listings, the rule that turns kinds into
the two lists, confirmation, and the block on enumeration. No model is
called; `decide_kind` is replaced."""
import sys
import zipfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2 import engagement_state as st                   # noqa: E402
from workflowsv2.materials_sorting import runner as sorting      # noqa: E402

PAGE = ("<html><head><title>Fast shortener</title>"
        "<meta name='description' content='No tracking.'></head><body>"
        "<header>Beta: free</header><main><h1>Short links</h1>"
        "<p>" + "It shortens links quickly and privately. " * 5 + "</p></main>"
        "<footer>(c) seller</footer></body></html>")
KINDS = {"README.md": "description", "docs/GUIDE.md": "description", "LICENSE": "instrument",
         "site/index.html": "description", "plugin/readme.txt": "description"}


@pytest.fixture
def eng(tmp_path, monkeypatch):
    monkeypatch.setattr(st, "ENGAGEMENTS", tmp_path)
    d = st.new_engagement(tmp_path / "e")
    t = d / "target"
    (t / "docs").mkdir(parents=True)
    (t / "site").mkdir()
    (t / "src").mkdir()
    (t / "README.md").write_text("# Tool\n\nSee the [guide](docs/GUIDE.md) and https://docs.example.com/x.\n"
                                 + "It does a thing. " * 10, encoding="utf-8")
    (t / "docs/GUIDE.md").write_text("Install it like this. " * 10, encoding="utf-8")
    (t / "LICENSE").write_text("Permission is hereby granted. " * 10, encoding="utf-8")
    (t / "site/index.html").write_text(PAGE, encoding="utf-8")
    (t / "src/main.py").write_text("print('hello')\n", encoding="utf-8")
    with zipfile.ZipFile(t / "plugin.zip", "w") as z:
        z.writestr("plugin/readme.txt", "The plugin adds a chat widget to any page. " * 5)
    monkeypatch.setattr(sorting, "backend_from_model", lambda _: type("B", (), {"resolved_model": lambda self: "fake"})())
    monkeypatch.setattr(sorting, "hosting_metadata", lambda _: None)
    monkeypatch.setattr(sorting, "decide_kind", lambda backend, c, linked: {
        "kind": KINDS[c["path"]], "parts": [], "reason": "test", "parse": "parsed", "parse_error": None})
    return d


def test_listing_reads_prose_archives_and_links(eng):
    cands, passed = sorting.list_candidates(eng / "target")
    assert {(c["path"], c["archive"]) for c in cands} == {
        ("README.md", None), ("docs/GUIDE.md", None), ("LICENSE", None),
        ("site/index.html", None), ("plugin/readme.txt", "plugin.zip")}
    assert passed == {".py": 1}
    linked, hosts = sorting.find_links(eng / "target", cands)
    assert linked == {"docs/GUIDE.md": ["README.md"]}
    assert hosts["docs.example.com"]["in"] == ["README.md"]


def test_every_description_is_a_source_and_an_exclude(eng):
    st.update_engagement(eng, claim_sources=["README.md"])
    sel = sorting.sort_materials(eng, Path("unused.yaml"))
    assert sel["proposal"]["claim_sources"] == [
        "README.md", "docs/GUIDE.md", "claim_sources/plugin_zip_plugin_readme_txt.md",
        "claim_sources/site_index_html.md"]
    assert sel["proposal"]["evidence_excludes"] == [
        "README.md", "docs/GUIDE.md", "site/index.html", "claim_sources/"]
    assert st.stage_value(eng, "sorting") == "proposed"
    assert st.claim_sources(eng) == ["README.md"]          # nothing applied before confirmation
    record = (eng / "sorting" / "SELECTION.md").read_text(encoding="utf-8")
    assert "Beta: free" in record and "docs.example.com" in record


def test_confirm_applies_extracts_and_records_changes(eng):
    sorting.sort_materials(eng, Path("unused.yaml"))
    prop = sorting.load(eng)["proposal"]
    sources = [s for s in prop["claim_sources"] if "plugin" not in s]
    sel = sorting.confirm(eng, "p@example.com", sources, prop["evidence_excludes"])
    assert st.claim_sources(eng) == sources and st.stage_value(eng, "sorting") == "confirmed"
    text = (eng / "target/claim_sources/site_index_html.md").read_text(encoding="utf-8")
    assert text.startswith("Page title: Fast shortener\n\nPage description: No tracking.")
    assert "Beta: free" not in text
    (check,) = sel["confirmed"]["extracted"]
    assert check["words"] == check["original_words"]
    assert sel["confirmed"]["changes"] == ["claim source removed: claim_sources/plugin_zip_plugin_readme_txt.md"]


def test_enumeration_waits_for_confirmation(eng):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from client_ui import jobs
    st.update_engagement(eng, claim_sources=["README.md"])
    assert jobs._precheck(eng, "enumerate", "m", "ts") == "the materials sorting is not confirmed"
    st.set_stage(eng, "sorting", "confirmed", "p@example.com")
    assert jobs._precheck(eng, "enumerate", "m", "ts") is None
