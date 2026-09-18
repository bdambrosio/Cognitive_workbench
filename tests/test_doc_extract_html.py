"""html_to_markdown: every visible word of <main> is kept, once, in blocks."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from utils.doc_extract import extract_to_markdown, html_to_markdown  # noqa: E402

PAGE = """<html><body><nav><a href="/">Home</a></nav><main>
<h1>Title <span>here</span></h1>
<p>One <b>bold</b> claim.<br>Second line.</p>
<div class="step"><div>01</div><div><strong>Loose text.</strong><p>Inside.</p></div></div>
<div><span>Verdict</span><span>Rating</span></div>
<ul><li><a href="#">First</a></li><li>Second</li></ul>
<table><tr><th>A</th><th>B</th></tr><tr><td>x | y</td><td>z</td></tr></table>
<script>var hidden = 1;</script><form><label>Name</label></form>
</main><footer>foot</footer></body></html>"""


def test_blocks_and_markers():
    assert html_to_markdown(PAGE).split("\n\n") == [
        "# Title here",
        "One bold claim. Second line.",
        "01",
        "Loose text.",
        "Inside.",
        "Verdict Rating",
        "- First",
        "- Second",
        "| A | B |\n| --- | --- |\n| x \\| y | z |",
    ]


def test_dispatch_on_suffix(tmp_path):
    f = tmp_path / "page.html"
    f.write_text(PAGE, encoding="utf-8")
    assert extract_to_markdown(f).startswith("# Title here")
