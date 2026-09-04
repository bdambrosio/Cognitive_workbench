"""report.md → report.html: a self-contained page with a print stylesheet.

    python3 -m workflowsv2.audit_report.printable <report.md> [<report.html>]

Markdown stays the source of record — it is what the writer emits into and
what continuation reads. This is the rendering for a reader and a printer:
page breaks at the boundaries a reader treats as separate documents, each
finding kept on one page where it fits, table headers that repeat across
pages, and the materials' date and commit on every page. One file, no
external assets, so it opens anywhere and prints from any browser.
"""
from __future__ import annotations

import html as html_std
import re
import sys
from pathlib import Path

from markdown_it import MarkdownIt

#: Sections that start a new page in print.
PAGE_BREAK_BEFORE = ("Executive summary", "What the review showed",
                     "Appendix — every claim and its verdict")

CSS = """
:root { --ink: #1a1a1a; --muted: #555; --rule: #cfcfcf; --box: #f6f6f6; }
html { font-family: Georgia, "Times New Roman", serif; color: var(--ink); }
body { max-width: 52em; margin: 2em auto; padding: 0 1.5em; line-height: 1.45; font-size: 11.5pt; }
h1 { font-size: 20pt; margin: 0 0 .3em; }
h2 { font-size: 15pt; margin: 1.6em 0 .5em; border-bottom: 1px solid var(--rule); padding-bottom: .2em; }
h3 { font-size: 11.5pt; margin: 0 0 .4em; }
p { margin: .5em 0; }
blockquote { margin: .4em 0 .6em; padding: .3em .8em; border-left: 3px solid var(--rule); color: var(--muted); font-style: italic; }
code { font-family: "DejaVu Sans Mono", Menlo, monospace; font-size: 9.5pt; background: var(--box); padding: 0 .2em; }
table { border-collapse: collapse; width: 100%; font-size: 9.5pt; margin: .6em 0; }
th, td { border: 1px solid var(--rule); padding: .25em .45em; vertical-align: top; text-align: left; }
thead { display: table-header-group; }
tr { break-inside: avoid; }
.finding { border: 1px solid var(--rule); border-radius: 3px; padding: .7em .9em .3em; margin: .9em 0; break-inside: avoid; }
.finding h3 { margin-top: 0; }
.finding ul { margin: .3em 0 .4em 1.2em; padding: 0; }
.front { color: var(--muted); font-size: 10pt; }
.page-break { break-before: page; }
@page { size: A4; margin: 20mm 18mm 22mm; }
@media print {
  body { max-width: none; margin: 0; padding: 0; font-size: 10.5pt; }
  a { color: inherit; text-decoration: none; }
}
"""


def to_body(md_text: str) -> str:
    """The report's markdown as the HTML body a page carries: page breaks
    before the sections a reader treats as separate documents, each finding
    boxed, the materials line marked as front matter. The browser's document
    pane uses this; `to_html` wraps it in a page."""
    md = MarkdownIt("commonmark").enable("table")
    body = md.render(md_text)
    # Page breaks before the sections a reader treats as separate documents.
    for name in PAGE_BREAK_BEFORE:
        body = re.sub(r"<h2>(" + re.escape(html_std.escape(name, quote=False)) + r")</h2>",
                      r'<h2 class="page-break">\1</h2>', body, count=1)
    # Each finding — an h3 and everything to the next heading — in a box.
    parts = re.split(r"(?=<h[23]>)", body)
    out = []
    for part in parts:
        if part.startswith("<h3>"):
            out.append('<section class="finding">' + part + "</section>")
        else:
            out.append(part)
    body = "".join(out)
    # The materials line under the title is front matter.
    body = re.sub(r"(</h1>\s*)<p>(Materials as of[^<]*)</p>",
                  r'\1<p class="front">\2</p>', body, count=1)
    return body


def to_html(md_text: str, title: str = "Claims review") -> str:
    body = to_body(md_text)
    # A running page header needs @page margin boxes, which Chrome's print
    # engine does not place; a fixed element lands on the page body instead.
    # Left to a PDF engine that supports them (WeasyPrint) if wanted.
    return (f"<!doctype html><html><head><meta charset=\"utf-8\">"
            f"<title>{html_std.escape(title)}</title><style>{CSS}</style></head>"
            f"<body>{body}</body></html>")


def render_file(md_path: Path, html_path: Path | None = None) -> Path:
    md_path = Path(md_path)
    text = md_path.read_text(encoding="utf-8")
    m = re.search(r"(?m)^# (.+)$", text)
    title = m.group(1) if m else "Claims review"
    html_path = Path(html_path) if html_path else md_path.with_suffix(".html")
    html_path.write_text(to_html(text, title), encoding="utf-8")
    return html_path


def to_pdf(html_path: Path, pdf_path: Path | None = None) -> Path | None:
    """Print the HTML to PDF with a headless Chrome, when one is installed;
    None when none is. Chrome honours the print stylesheet's page breaks and
    repeating table headers; it does not place page-margin boxes, so there
    is no running header — a PDF engine such as WeasyPrint would add one."""
    import shutil
    import subprocess
    chrome = next((c for c in ("google-chrome", "chromium", "chromium-browser")
                   if shutil.which(c)), None)
    if not chrome:
        return None
    pdf_path = Path(pdf_path) if pdf_path else Path(html_path).with_suffix(".pdf")
    try:
        subprocess.run([chrome, "--headless=new", "--no-sandbox", "--disable-gpu",
                        "--no-pdf-header-footer",
                        f"--print-to-pdf={pdf_path}",
                        f"file://{Path(html_path).resolve()}"],
                       check=True, capture_output=True, timeout=300)
    except (subprocess.SubprocessError, OSError):
        return None
    return pdf_path if pdf_path.is_file() else None


if __name__ == "__main__":
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    print(render_file(src, dst))
