"""doc-read — reading a PDF that is already on the machine.

Built after a 403 sent the user to download a paper by hand and there was
no way to read it; the agent reviewed a different paper found by title
similarity instead. The properties worth holding: it only reads where
documents live, it presents the same way fetch-text does, and it reports
the document's own title so a substitution is visible.
"""
import importlib.util
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))


def _tool():
    spec = importlib.util.spec_from_file_location(
        "doc_read", SRC / "tools" / "doc-read" / "tool.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


mod = _tool()

_CHUNKS = [
    ("Abstract", "we survey self-evolution."),
    ("Introduction", "first half."),
    ("Introduction", "second half."),          # GROBID splits long sections
    ("The L0-L4 Taxonomy", "levels zero through four."),
]


@pytest.fixture
def paper(tmp_path, monkeypatch):
    """A readable PDF inside an allowed root, with GROBID stubbed."""
    monkeypatch.setattr(mod, "_ROOTS", [tmp_path])
    pdf = tmp_path / "paper.pdf"
    pdf.write_bytes(b"%PDF-1.4\n")             # never parsed; grobid stubbed
    monkeypatch.setattr(
        "utils.grobid.parse_pdf_grobid",
        lambda **kw: {"title": "Diving into Reliable Self-Evolving Agents",
                      "abstract": "A survey.", "chunks": _CHUNKS})
    return pdf


def test_index_names_the_documents_own_title(paper):
    """The whole failure this tool was built for was reviewing a paper
    identified by title similarity. The index has to state what the file
    actually is, so a substitution is visible rather than assumed away."""
    r = mod.react_invoke({"path": "paper.pdf"})
    assert r["status"] == "ok"
    assert "Diving into Reliable Self-Evolving Agents" in r["text"]
    assert "## Abstract" in r["text"]
    # Index, not body: section names and sizes, no section text.
    assert "The L0-L4 Taxonomy" in r["text"]
    assert "levels zero through four" not in r["text"]


def test_split_sections_are_rejoined_in_document_order(paper):
    r = mod.react_invoke({"path": "paper.pdf", "section": "Introduction"})
    assert r["text"] == "## Introduction\n\nfirst half.\n\nsecond half."


def test_section_resolves_by_unambiguous_prefix(paper):
    r = mod.react_invoke({"path": "paper.pdf", "section": "the l0"})
    assert r["status"] == "ok" and "levels zero through four" in r["text"]


def test_unknown_section_lists_what_is_available(paper):
    r = mod.react_invoke({"path": "paper.pdf", "section": "Methodology"})
    assert r["status"] == "error"
    assert "Abstract" in r["text"] and "Introduction" in r["text"]


def test_reads_relative_to_an_allowed_root(paper):
    assert mod.react_invoke({"path": "paper.pdf"})["status"] == "ok"
    assert mod.react_invoke({"path": str(paper)})["status"] == "ok"


def test_path_outside_the_roots_is_refused(paper):
    r = mod.react_invoke({"path": "/etc/passwd"})
    assert r["status"] == "error"
    assert "outside the folders I can read" in r["text"]


def test_symlink_out_of_the_root_is_refused(tmp_path, monkeypatch):
    """Resolution happens before the containment check: a link planted
    inside a readable root must not become a way out of it — the agent
    reading this can be steered by an RSS feed."""
    root = tmp_path / "root"
    root.mkdir()
    outside = tmp_path / "secret.pdf"
    outside.write_bytes(b"%PDF-1.4\n")
    (root / "innocent.pdf").symlink_to(outside)
    monkeypatch.setattr(mod, "_ROOTS", [root])
    r = mod.react_invoke({"path": "innocent.pdf"})
    assert r["status"] == "error", "symlink escaped the geofence"


def test_missing_file_and_non_pdf_say_which(paper, tmp_path):
    assert "no such file" in mod.react_invoke({"path": "absent.pdf"})["text"]
    (tmp_path / "notes.txt").write_text("hello")
    r = mod.react_invoke({"path": "notes.txt"})
    assert r["status"] == "error" and "PDF" in r["text"]


def test_missing_path_argument(paper):
    assert mod.react_invoke({})["status"] == "error"


def test_falls_back_to_flat_text_without_grobid(tmp_path, monkeypatch):
    """GROBID supplies structure, not the text. When it is down the tool
    still reads the document — worse to navigate, but readable."""
    monkeypatch.setattr(mod, "_ROOTS", [tmp_path])
    pdf = tmp_path / "flat.pdf"
    pdf.write_bytes(b"%PDF-1.4\n")
    monkeypatch.setattr("utils.grobid.parse_pdf_grobid",
                        lambda **kw: (_ for _ in ()).throw(OSError("down")))
    monkeypatch.setattr("utils.doc_extract.extract_to_markdown",
                        lambda p, **kw: "page one text")
    r = mod.react_invoke({"path": "flat.pdf"})
    assert r["status"] == "ok"
    assert "page one text" in r["text"]
    assert "no section structure" in r["text"], \
        "the agent must know it is reading flat text, not a section"
