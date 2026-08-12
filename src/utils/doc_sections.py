"""Section addressing shared by the document readers.

`fetch-text` (remote) and `doc-read` (local) both hand an agent a section
index and then a named section from it. The resolution rule has to be
identical in both or the same request behaves differently depending on
where the document happens to live, which is the sort of difference
nobody debugs — they just conclude the tool is flaky.
"""
from typing import List, Optional


def resolve_section(requested: str, names: List[str]) -> Optional[str]:
    """Map a requested section name onto one from the index: exact, then
    case-insensitive, then an unambiguous case-insensitive prefix. Returns
    None when nothing matches or a prefix is ambiguous — this resolves an
    identifier the caller read off our own index, so it stays literal
    rather than guessing at intent."""
    req = (requested or '').strip()
    if not req:
        return None
    if req in names:
        return req
    low = req.lower()
    ci = [n for n in names if n.lower() == low]
    if len(ci) == 1:
        return ci[0]
    pref = [n for n in names if n.lower().startswith(low)]
    return pref[0] if len(pref) == 1 else None
