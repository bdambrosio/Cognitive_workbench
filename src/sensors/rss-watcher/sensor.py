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
            new_items.append({
                'title': title,
                'url': entry.get('link', ''),
                'feed': feed_url,
            })

    if not new_items:
        return {'status': 'nothing', 'content': '', 'metadata': {}}

    n = len(new_items)
    return {
        'status': 'ok',
        'content': {
            'summary': f"{n} new article{'s' if n != 1 else ''} from RSS feeds",
            'data': new_items,
        },
        'metadata': {'item_count': n},
    }
