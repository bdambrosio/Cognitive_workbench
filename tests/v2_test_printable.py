"""Seller text quoted into the report is text in the page, never markup."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2.audit_report import printable, render                   # noqa: E402


def test_a_tag_in_quoted_text_renders_as_text_not_markup():
    md = "# Claims review — t\n\n## What the review showed\n\n### t, claim 1\n\n" \
         "> \"" + render._md_safe('return <script> window.x = 1; </script> and <b>bold</b>') + "\"\n\n" \
         "### t, claim 2\n\nafter\n"
    body = printable.to_body(md)
    assert "<script" not in body and "<b>" not in body
    assert "&lt;script&gt;" in body and "claim 2" in body and "after" in body


def test_raw_html_in_markdown_is_off_even_without_md_safe():
    body = printable.to_body("para\n\n<script>alert(1)</script>\n\nafter\n")
    assert "<script>" not in body and "&lt;script&gt;" in body and "after" in body


def test_md_safe_escapes_the_markup_starters():
    assert render._md_safe("a | b ``` <c>") == "a \\| b \\`\\`\\` \\<c>"
