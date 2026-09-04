"""Every link in site/ resolves to a page or a placeholder, and every page
carries the placeholders the practice fills in."""
import re
from pathlib import Path

SITE = Path(__file__).resolve().parents[1] / "site"


def test_every_href_resolves_and_placeholders_present():
    pages = sorted(SITE.glob("*.html"))
    assert len(pages) == 6
    names = {p.name for p in pages}
    for p in pages:
        html = p.read_text()
        for href in re.findall(r'href="([^"]+)"', html):
            if href.startswith(("mailto:", "http", "{{")):
                continue
            assert href.split("#")[0] in names or href in ("site.css",), f"{p.name}: {href}"
        assert "{{NAME}}" in html and "{{EMAIL}}" in html
        assert "<title>" in html and 'name="description"' in html
