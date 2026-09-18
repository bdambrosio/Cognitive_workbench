"""Document → markdown extraction.

Narrow helper for pulling readable text (and clean tables) out of binary
documents that the existing read/grep flow can't handle — PDFs read as
mojibake under utf-8, and ripgrep skips them entirely.

Uses `pymupdf` (fitz), already a project dependency (fetch-text relies on
it for PDF extraction) — one PDF library, not two. On real financial
filings pymupdf's `find_tables()` also preserves word spacing in line-item
labels where pdfplumber jammed them together (e.g. "Total liabilities" vs
"Totalliabilities"), which matters for grep/readability — validated on a
Berkshire 10-K balance sheet, 2026-06-19.

Scope: PDF and HTML. `extract_to_markdown` dispatches on file extension and
is the seam a future document-inspection subagent would reuse to
materialize a text view of a mixed-document directory (see
docs/financial-analysis-tools-plan.md, Decision 2/3).
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import List, Optional, Union

logger = logging.getLogger(__name__)

# Cell text can run long (paragraph footnotes inside a table cell); cap so a
# single cell can't blow up a row.
_MAX_CELL_CHARS = 300


def _table_to_markdown(table: List[List[Optional[str]]]) -> str:
    """Render an extracted table (list of rows of cells) as a GitHub
    markdown table. None cells → empty; newlines flattened to spaces;
    ragged rows padded to the widest row. Columns that are empty in every
    row are dropped — financial-statement extraction emits many such
    alignment-spacer columns (the `$` sign and number columns sit apart).
    Returns '' for an empty table."""
    rows: List[List[str]] = []
    for row in table:
        if not row:
            continue
        cells = []
        for c in row:
            text = "" if c is None else str(c).replace("\n", " ").strip()
            if len(text) > _MAX_CELL_CHARS:
                text = text[:_MAX_CELL_CHARS] + "…"
            # Escape pipes so they don't break the column structure.
            cells.append(text.replace("|", "\\|"))
        rows.append(cells)
    if not rows:
        return ""
    width = max(len(r) for r in rows)
    rows = [r + [""] * (width - len(r)) for r in rows]
    keep = [c for c in range(width) if any(r[c] for r in rows)]
    if not keep:
        return ""
    rows = [[r[c] for c in keep] for r in rows]
    header = rows[0]
    lines = [
        "| " + " | ".join(header) + " |",
        "| " + " | ".join(["---"] * len(keep)) + " |",
    ]
    for r in rows[1:]:
        lines.append("| " + " | ".join(r) + " |")
    return "\n".join(lines)


def pdf_to_markdown(source: Union[str, Path, bytes, bytearray],
                    *, max_pages: Optional[int] = None) -> str:
    """Extract a PDF to markdown: per page, the narrative text followed by
    any tables rendered as markdown.

    Args:
        source: a filesystem path (str/Path) or raw PDF bytes (e.g. a
            web-fetched filing).
        max_pages: if set, only the first N pages are extracted.

    Returns the markdown string. Raises on unreadable/corrupt PDFs — the
    caller decides how to surface that (no silent empty-string fallback).
    """
    import pymupdf

    if isinstance(source, (bytes, bytearray)):
        doc = pymupdf.open(stream=bytes(source), filetype="pdf")
    else:
        doc = pymupdf.open(str(source))

    parts: List[str] = []
    try:
        for i, page in enumerate(doc, start=1):
            if max_pages is not None and i > max_pages:
                break
            parts.append(f"## Page {i}")
            text = (page.get_text() or "").strip()
            if text:
                parts.append(text)
            for table in page.find_tables().tables:
                md = _table_to_markdown(table.extract())
                if md:
                    parts.append(md)
    finally:
        doc.close()
    return "\n\n".join(parts).strip()


# Elements that start a new block of output. Everything else is inline and
# continues the block it sits in.
_HTML_BLOCKS = {"h1", "h2", "h3", "h4", "h5", "h6", "p", "li", "ul", "ol",
                "div", "section", "article", "aside", "blockquote", "dl",
                "dt", "dd", "figure", "figcaption", "details", "summary",
                "pre", "hr", "address"}
# Left out: not the page's own text, or not text a reader sees.
_HTML_DROPPED = {"script", "style", "nav", "header", "footer", "video",
                 "audio", "form", "noscript", "template"}


def html_to_markdown(source: Union[str, Path]) -> str:
    """Extract the readable text of an HTML page as markdown: the page's
    `<main>` element (the `<body>` if there is none), headings as `#` lines,
    list items as `- ` lines, every other block as one line, tables as
    markdown tables. Navigation, header, footer, forms, scripts and media
    elements are left out. `source` is a path, or the HTML itself as a
    string."""
    from bs4 import BeautifulSoup
    from bs4.element import Comment, NavigableString

    html = source.read_text(encoding="utf-8", errors="replace") \
        if isinstance(source, Path) else str(source)
    soup = BeautifulSoup(html, "html.parser")
    root = soup.find("main") or soup.body or soup

    parts: List[str] = []
    run: List[str] = []       # inline text of the block being read
    prefix = [""]             # markdown marker for the next block emitted

    def flush() -> None:
        text = " ".join("".join(run).split())
        run.clear()
        if text:
            parts.append(prefix[0] + text)
            prefix[0] = ""

    def walk(node) -> None:
        for child in node.children:
            if isinstance(child, Comment):
                continue
            if isinstance(child, NavigableString):
                run.append(str(child))
            elif child.name in _HTML_DROPPED:
                continue
            elif child.name == "table":
                flush()
                md = _table_to_markdown(
                    [[" ".join(c.get_text(" ").split())
                      for c in tr.find_all(["th", "td"])]
                     for tr in child.find_all("tr")])
                if md:
                    parts.append(md)
            elif child.name in _HTML_BLOCKS:
                flush()
                if child.name[0] == "h" and child.name[1:].isdigit():
                    prefix[0] = "#" * int(child.name[1:]) + " "
                elif child.name == "li":
                    prefix[0] = "- "
                elif child.name == "blockquote":
                    prefix[0] = "> "
                walk(child)
                flush()
                prefix[0] = ""
            else:
                # Two inline elements with nothing between them are set apart
                # by styling, not by a space in the text: two chips in a row.
                if child.name == "br" or getattr(child.previous_sibling,
                                                 "name", None):
                    run.append(" ")
                walk(child)

    walk(root)
    flush()
    return "\n\n".join(parts).strip()


def extract_to_markdown(path: Union[str, Path],
                        *, max_pages: Optional[int] = None) -> str:
    """Dispatch on file extension and extract to markdown. Handles PDF and
    HTML; other types raise ValueError so the gap is explicit rather than
    silently returning nothing."""
    p = Path(path)
    suffix = p.suffix.lower()
    if suffix == ".pdf":
        return pdf_to_markdown(p, max_pages=max_pages)
    if suffix in (".html", ".htm"):
        return html_to_markdown(p)
    raise ValueError(
        f"extract_to_markdown: unsupported file type {suffix!r} "
        f"(supports .pdf, .html)")
