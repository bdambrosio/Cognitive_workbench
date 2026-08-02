"""RSS feed watcher sensor.

Returns up to MAX_PER_CYCLE new titles. Writes them to a persistent Note
named '_rss_pending_titles' so the triggered goal can read them via load().
If more new articles exist than the cap, the overflow count is noted in
the first line of the Note content.
"""
import logging
import feedparser

logger = logging.getLogger(__name__)

_seen_ids = set()
MAX_PER_CYCLE = 10


def run(context):
    feeds = context['parameters'].get('feeds', [])
    resource_manager = context.get('resource_manager')
    new_items = []

    for feed_url in feeds:
        try:
            feed = feedparser.parse(feed_url)
        except Exception:
            continue
        for entry in feed.entries:
            eid = entry.get('id', entry.get('link', ''))
            if eid in _seen_ids:
                continue
            _seen_ids.add(eid)
            title = entry.get('title', '')
            if not title:
                continue
            new_items.append({
                'title': title,
                'url': entry.get('link', ''),
                'feed': feed_url,
            })

    if not new_items:
        return {'status': 'nothing', 'content': '', 'metadata': {}}

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
                    character_name=context.get('character_name', 'Jill'),
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
    summary = f"{n} new article{'s' if n != 1 else ''} from RSS feeds"
    if overflow > 0:
        summary += f" ({overflow} more not shown)"

    return {
        'status': 'ok',
        'content': {
            'summary': summary,
            'data': capped,
        },
        'metadata': {'item_count': n, 'overflow': overflow, 'total_new': total_new},
    }
