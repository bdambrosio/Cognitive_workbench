---
name: rss-watcher
description: Monitors RSS feeds for new items matching configured keywords
type: code
schedule: "15m"
parameters:
  feeds: []
  keywords: []
---

Polls configured RSS feeds and reports new items that match any of the configured keywords.
Items are deduplicated across runs using entry IDs.
