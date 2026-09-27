#!/usr/bin/env python3
"""Write the site's FAQ page, site/faq.html, from the approved answers.

    python3 workflowsv2/outreach/faq_page.py

Run it after ANSWERS.md changes, read the page, then deploy the site as usual.
The page shows each entry's question and answer. It leaves out the file's
opening paragraphs, which are instructions to the practice, and each entry's
Source line, which is the practice's record of where the facts come from.
"""
from __future__ import annotations

import html
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ANSWERS = HERE / "method" / "ANSWERS.md"
OUT = HERE.parents[1] / "site" / "faq.html"

#: A bare tuuyi.com address in an answer, made a link on the page.
LINK = re.compile(r"\b((?:demo\.)?tuuyi\.com(?:/[a-z-]+)?)\b")

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Questions — Tuuyi</title>
<meta name="description" content="Answers to questions people ask about Tuuyi's claims review of software.">
<link rel="canonical" href="https://tuuyi.com/faq">
<link rel="icon" href="/favicon.svg" type="image/svg+xml">
<link rel="icon" href="/favicon.ico" sizes="32x32">
<meta property="og:type" content="website">
<meta property="og:site_name" content="Tuuyi">
<meta property="og:title" content="Questions — Tuuyi">
<meta property="og:description" content="Answers to questions people ask about Tuuyi's claims review of software.">
<meta property="og:url" content="https://tuuyi.com/faq">
<meta property="og:image" content="https://tuuyi.com/og.png">
<meta property="og:image:width" content="1200">
<meta property="og:image:height" content="630">
<meta name="twitter:card" content="summary_large_image">
<link rel="stylesheet" href="/site.css?v=20260925">
<style>
  .faq h2 {{ font-size: 21px; margin: 40px 0 8px; }}
  .faq p {{ max-width: 68ch; }}
</style>
</head>
<body>
<div class="wrap">
<header class="top">
  <a class="brand" href="/">Tuuyi</a>
  <nav><a href="/">Home</a><a href="/how-it-works">How it works</a><a href="/pricing">Pricing</a><a href="/demo">Demo</a><a href="/faq" aria-current="page">FAQ</a><a href="/sellers">For sellers</a><a href="/about">About</a><a class="pill" href="/pricing">Beta: free claims reviews, limited</a></nav>
</header>
<main class="faq">

<h1>Questions about Tuuyi.</h1>
<p class="lede">Answers to questions people ask about the claims review. Something not here? <a href="/contact">Write to us</a>.</p>
{entries}
</main>
<footer>
  <div>© 2026 Tuuyi · <a href="/contact">Contact</a></div>
  <div><a href="/privacy">Privacy</a><a href="/demo">Demo</a><a href="/about">About</a></div>
</footer>
</div>
</body>
</html>
"""


def entries(text: str) -> list:
    """(question, [paragraph, ...]) for each entry, without its Source line."""
    out = []
    for block in re.split(r"^## ", text, flags=re.M)[1:]:
        question, _, body = block.partition("\n")
        paras = [" ".join(p.split()) for p in re.split(r"\n\s*\n", body) if p.strip()]
        out.append((question.strip(), [p for p in paras if not p.startswith("Source:")]))
    return out


def paragraph(p: str) -> str:
    return "<p>" + LINK.sub(lambda m: f'<a href="https://{m.group(1)}">{m.group(1)}</a>', html.escape(p, quote=False)) + "</p>"


def main() -> int:
    found = entries(ANSWERS.read_text(encoding="utf-8"))
    body = "\n".join(f"\n<h2>{html.escape(q, quote=False)}</h2>\n" + "\n".join(paragraph(p) for p in ps)
                     for q, ps in found)
    OUT.write_text(PAGE.format(entries=body), encoding="utf-8")
    print(f"{OUT}: {len(found)} questions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
