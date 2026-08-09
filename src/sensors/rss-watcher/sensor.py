"""RSS feed watcher sensor.

Returns up to MAX_PER_CYCLE new titles. Writes them to a persistent Note
named '_rss_pending_titles' so the triggered goal can read them via load().
If more new articles exist than the cap, the overflow count is noted in
the first line of the Note content.

Per-character seen-sets live in module state and are deliberately not
persisted: a fresh process seeds its baseline silently on the first poll.
Without that seeding every restart re-reported each feed's whole current
front page as new articles — the re-flood that made this sensor unusable
for chat characters.

Emits a prose situation report, not structured content: non-tick sensors
reach the chat loop through the `text` field of the sense_data envelope,
and a {'summary': ..., 'data': ...} payload has no `text` for it to read.
"""
import logging
import feedparser

logger = logging.getLogger(__name__)

# character -> set of entry ids already seen. Absent means the baseline
# has not been seeded yet for this process.
_seen: dict = {}

_NOTHING = {'status': 'nothing', 'content': '', 'metadata': {}}

MAX_PER_CYCLE = 10


def _describe(items: list, overflow: int) -> str:
    """A self-contained report of what showed up in the feeds.

    Non-tick sensors arrive as user-like turns, so this has to read as an
    event the agent is noticing rather than as something anyone said.
    """
    n = len(items)
    lines = [f"{n} new article{'s' if n != 1 else ''} appeared in the "
             f"RSS feeds you watch."]
    if overflow > 0:
        lines.append(f"({overflow} more are not shown.)")
    lines.append("")

    for item in items:
        lines.append(f"  \"{item['title']}\"")
        if item['url']:
            lines.append(f"    {item['url']}")

    lines.append("")
    lines.append("Nobody has said anything — you noticed this yourself. "
                 "A feed producing articles is not news in itself. Reply "
                 "only if one of these genuinely bears on what you and the "
                 "user are working on; otherwise stay silent.")
    return "\n".join(lines)


def run(context):
    me = context.get('character_name') or ''
    if not me:
        return _NOTHING

    feeds = context['parameters'].get('feeds', [])
    resource_manager = context.get('resource_manager')

    prior = _seen.get(me)
    seeding = prior is None
    seen = set() if seeding else prior
    new_items = []

    for feed_url in feeds:
        try:
            feed = feedparser.parse(feed_url)
        except Exception as e:
            logger.warning(f"rss-watcher: cannot parse {feed_url}: {e}")
            continue
        for entry in feed.entries:
            eid = entry.get('id', entry.get('link', ''))
            if eid in seen:
                continue
            seen.add(eid)
            title = entry.get('title', '')
            if not title:
                continue
            if seeding:
                # Record it as known, but do not report it: what a feed
                # already had when this process started is not an event.
                continue
            new_items.append({
                'title': title,
                'url': entry.get('link', ''),
                'feed': feed_url,
            })

    _seen[me] = seen

    if seeding:
        logger.info(f"rss-watcher[{me}]: baseline seeded ({len(seen)} "
                    f"entries across {len(feeds)} feed(s)), no event emitted")
        return _NOTHING

    if not new_items:
        return _NOTHING

    # Sort by feed order (newest first in most RSS feeds), cap at MAX_PER_CYCLE
    total_new = len(new_items)
    capped = new_items[:MAX_PER_CYCLE]
    overflow = total_new - len(capped)

    # Build Note content: one title per line (with source URL when the
    # feed provides one — dropping it here would lose the only pointer
    # back to the article), plus overflow header if needed
    lines = []
    if overflow > 0:
        lines.append(f"[{overflow} additional new titles not shown]")
    for item in capped:
        if item['url']:
            lines.append(f"{item['title']} — {item['url']}")
        else:
            lines.append(item['title'])
    note_content = "\n".join(lines)

    # Write to persistent Note '_rss_pending_titles' so the goal can read it
    if resource_manager:
        try:
            # Check if Note already exists
            existing_id = resource_manager.named_notes.get('_rss_pending_titles')
            if existing_id:
                resource_manager.update_note_content(existing_id, note_content)
            else:
                success, note_id, error, _ = resource_manager.create_note(
                    character_name=me,
                    content=note_content,
                    format_type='text',
                    source_skill='rss-watcher',
                    source_value='RSS pending titles',
                    note_name='_rss_pending_titles',
                    extra_props={'persistent': True},
                )
                if not success:
                    logger.warning(f"rss-watcher: failed to create _rss_pending_titles Note: {error}")
        except Exception as e:
            logger.warning(f"rss-watcher: failed to write _rss_pending_titles: {e}")

    n = len(capped)
    logger.info(f"rss-watcher[{me}]: {n} new item(s), overflow={overflow}")
    return {
        'status': 'ok',
        'content': _describe(capped, overflow),
        'metadata': {'item_count': n, 'overflow': overflow,
                     'total_new': total_new},
    }
