"""Every internal link on the site leads somewhere: a visitor never meets a
broken link. Links are resolved as nginx serves the site (clean URLs since
4b6965e8: `/` is index.html, `/demo` is demo.html; a file by its path)."""
import re
from pathlib import Path
from typing import Optional

SITE = Path(__file__).resolve().parents[1] / "site"


def _target(href: str) -> Optional[Path]:
    """The file an internal href serves, or None for an external one."""
    if href.startswith(("mailto:", "tel:", "http:", "https:", "//", "#")):
        return None
    path = href.split("#")[0].split("?")[0]
    if path in ("", "/"):
        return SITE / "index.html"
    rel = path.lstrip("/")
    page = SITE / f"{rel}.html"
    return page if page.is_file() else SITE / rel


def test_every_internal_link_resolves_and_every_page_has_a_title():
    pages = sorted(SITE.glob("*.html"))
    assert pages
    broken = []
    for p in pages:
        html = p.read_text(encoding="utf-8")
        assert "<title>" in html, f"{p.name} has no title"
        for href in re.findall(r'(?:href|src)="([^"]+)"', html):
            target = _target(href)
            if target is not None and not target.is_file():
                broken.append(f"{p.name}: {href}")
    assert not broken, "broken links:\n" + "\n".join(broken)
