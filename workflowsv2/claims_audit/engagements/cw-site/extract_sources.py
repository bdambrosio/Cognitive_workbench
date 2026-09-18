#!/usr/bin/env python3
"""Write this engagement's claim-source files from the site pages in its
target, as confirmed in sorting/SELECTION.md. Run from the
repository root: zenoh_venv/bin/python3 <this file>."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "src"))
from bs4 import BeautifulSoup                              # noqa: E402
from utils.doc_extract import extract_to_markdown          # noqa: E402

TARGET = Path(__file__).resolve().parent / "target"
OUT = TARGET / "claim_sources" / "site"
PAGES = ["how-it-works", "index", "introduction", "pricing", "demo", "about",
         "privacy", "contact", "404"]
BANNER = ("Banner shown in the header of every page: "
          "Beta: free claims reviews, limited")

for name in PAGES:
    page = TARGET / "site" / f"{name}.html"
    soup = BeautifulSoup(page.read_text(encoding="utf-8"), "html.parser")
    head = [f"Page title: {soup.title.get_text(strip=True)}"]
    meta = soup.find("meta", attrs={"name": "description"})
    if meta:
        head.append(f"Page description: {meta['content']}")
    if name == "index":
        head.append(BANNER)
    body = extract_to_markdown(page)
    if name == "introduction":
        body, sep, transcript = body.partition("## Transcript\n")
        assert sep, "introduction.html no longer has a Transcript heading"
        (OUT / "introduction-transcript.md").write_text(
            "# Transcript of the introduction video\n\n" + transcript.lstrip() + "\n",
            encoding="utf-8")
    (OUT / f"{name}.md").write_text(
        "\n\n".join(head) + "\n\n" + body.rstrip() + "\n", encoding="utf-8")
    print(f"{name}: {len(body.split())} words")
