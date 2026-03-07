"""RSS feed watcher sensor."""
import feedparser

_seen_ids = set()


def run(context):
    feeds = context['parameters'].get('feeds', [])
    keywords = context['parameters'].get('keywords', [])
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
            if keywords and not any(kw.lower() in title.lower() for kw in keywords):
                continue
            new_items.append(f"- {title}: {entry.get('link', '')}")

    if not new_items:
        return {'status': 'nothing', 'content': '', 'metadata': {}}

    return {
        'status': 'ok',
        'content': f"RSS updates ({len(new_items)} new):\n" + "\n".join(new_items),
        'metadata': {'item_count': len(new_items)},
    }
